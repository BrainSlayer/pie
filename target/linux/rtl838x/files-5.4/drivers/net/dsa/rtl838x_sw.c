// SPDX-License-Identifier: GPL-2.0-only

#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phylink.h>
#include <linux/phy_fixed.h>
#include <net/dsa.h>
#include <net/switchdev.h>

#include <asm/mach-rtl838x/mach-rtl838x.h>
#include "rtl838x.h"

#define RTL8380_VERSION_A 'A'
#define RTL8390_VERSION_A 'A'
#define RTL9310_VERSION_A 'A'
#define RTL8380_VERSION_B 'B'

extern struct rtl838x_soc_info soc_info;

extern void rtl9300_dump(void);

DEFINE_MUTEX(smi_lock);

extern void rtl8380_sds_rst(int mac);

struct table_reg {
	u16 addr;
	u16 data;
	u8  max_data;
	u8 c_bit;
	u8 t_bit;
	u8 rmode;
	u8 tbl;
	struct mutex lock;
};

#define TBL_DESC(_addr, _data, _max_data, _c_bit, _t_bit, _rmode) \
		{  .addr = _addr, .data = _data, .max_data = _max_data, .c_bit = _c_bit, \
		    .t_bit = _t_bit, .rmode = _rmode \
		}

static struct table_reg rtl838x_tbl_regs[] = {
	TBL_DESC(0x6900, 0x6908, 3, 15, 13, 1),		// RTL8380_TBL_L2
	TBL_DESC(0x6914, 0x6918, 18, 14, 12, 1),	// RTL8380_TBL_0
	TBL_DESC(0xA4C8, 0xA4CC, 6, 14, 12, 1),		// RTL8380_TBL_1

	TBL_DESC(0x1180, 0x1184, 3, 16, 14, 0),		// RTL8390_TBL_L2
	TBL_DESC(0x1190, 0x1194, 17, 15, 12, 0),	// RTL8390_TBL_0
	TBL_DESC(0x6B80, 0x6B84, 4, 14, 12, 0),		// RTL8390_TBL_1
	TBL_DESC(0x611C, 0x6120, 9, 8, 6, 0),		// RTL8390_TBL_2

	TBL_DESC(0xB320, 0xB334, 3, 18, 16, 0),		// RTL9300_TBL_L2
	TBL_DESC(0xB340, 0xB344, 19, 16, 12, 0),	// RTL9300_TBL_0
	TBL_DESC(0xB3A0, 0xB3A4, 20, 16, 13, 0),	// RTL9300_TBL_1
	TBL_DESC(0xCE04, 0xCE08, 6, 14, 12, 0),		// RTL9300_TBL_2
	TBL_DESC(0xD600, 0xD604, 30, 7, 6, 0),		// RTL9300_TBL_HSB
	TBL_DESC(0x7880, 0x7884, 22, 9, 8, 0),		// RTL9300_TBL_HSA

	TBL_DESC(0x8500, 0x8508, 8, 19, 15, 0),		// RTL9310_TBL_0
	TBL_DESC(0x40C0, 0x40C4, 22, 16, 14, 0),	// RTL9310_TBL_1
	TBL_DESC(0x8528, 0x852C, 6, 18, 14, 0),		// RTL9310_TBL_2
	TBL_DESC(0x0200, 0x0204, 9, 15, 12, 0),		// RTL9310_TBL_3
	TBL_DESC(0x20dc, 0x20e0, 29, 7, 6, 0),		// RTL9310_TBL_4
	TBL_DESC(0x7e1c, 0x7e20, 53, 8, 6, 0),		// RTL9310_TBL_5
};

typedef enum {
	RTL8380_TBL_L2 = 0,
	RTL8380_TBL_0,
	RTL8380_TBL_1,
	RTL8390_TBL_L2,
	RTL8390_TBL_0,
	RTL8390_TBL_1,
	RTL8390_TBL_2,
	RTL9300_TBL_L2,
	RTL9300_TBL_0,
	RTL9300_TBL_1,
	RTL9300_TBL_2,
	RTL9300_TBL_HSB,
	RTL9300_TBL_HSA,
	RTL9310_TBL_0,
	RTL9310_TBL_1,
	RTL9310_TBL_2,
	RTL9310_TBL_3,
	RTL9310_TBL_4,
	RTL9310_TBL_5,
	RTL_TBL_END
} rtl838x_tbl_reg_t;


void rtl_table_init(void)
{
	int i;

	for (i = 0; i < RTL_TBL_END; i++)
		mutex_init(&rtl838x_tbl_regs[i].lock);
}

/*
 * Request access to table t in table access register r
 * Returns a handle to a lock for that table
 */
struct table_reg *rtl_table_get(rtl838x_tbl_reg_t r, int t)
{
	if (r >= RTL_TBL_END)
		return NULL;

	if (t >= BIT(rtl838x_tbl_regs[r].c_bit-rtl838x_tbl_regs[r].t_bit))
		return NULL;

	mutex_lock(&rtl838x_tbl_regs[r].lock);
	rtl838x_tbl_regs[r].tbl = t;

	return &rtl838x_tbl_regs[r];
}

/*
 * Release a table r, unlock the corresponding lock
 */
void rtl_table_release(struct table_reg *r)
{
	if (!r)
		return;

//	pr_info("Unlocking %08x\n", (u32)r);
	mutex_unlock(&r->lock);
//	pr_info("Unlock done\n");
}

/*
 * Reads table index idx into the data registers of the table
 */
void rtl_table_read(struct table_reg *r, int idx)
{
	u32 cmd = r->rmode ? BIT(r->c_bit) : 0;

	cmd |= BIT(r->c_bit + 1) | (r->tbl << r->t_bit) | (idx & (BIT(r->t_bit) - 1));
	sw_w32(cmd, r->addr);
	pr_debug("Writing %08x to %x for read\n", cmd, r->addr);
	do { } while (sw_r32(r->addr) & BIT(r->c_bit + 1));
}

/*
 * Writes the content of the table data registers into the table at index idx
 */
void rtl_table_write(struct table_reg *r, int idx)
{
	u32 cmd = r->rmode ? 0 : BIT(r->c_bit);

	cmd |= BIT(r->c_bit + 1) | (r->tbl << r->t_bit) | (idx & (BIT(r->t_bit) - 1));
	pr_debug("Writing %08x to %x for write, value %08x\n",
		cmd, r->addr, sw_r32(0xb344));
	sw_w32(cmd, r->addr);
	do { } while (sw_r32(r->addr) & BIT(r->c_bit + 1));
}

/*
 * Returns the address of the ith data register of table register r
 * the address is relative to the beginning of the Switch-IO block at 0xbb000000
 */
inline u16 rtl_table_data(struct table_reg *r, int i)
{
	if (i >= r->max_data)
		i = r->max_data - 1;
	return r->data + i * 4;
}

inline u32 rtl_table_data_r(struct table_reg *r, int i)
{
	return sw_r32(rtl_table_data(r, i));
}

inline void rtl_table_data_w(struct table_reg *r, u32 v, int i)
{
	sw_w32(v, rtl_table_data(r, i));
}

#define MIB_DESC(_size, _offset, _name) {.size = _size, .offset = _offset, .name = _name}
struct rtl838x_mib_desc {
	unsigned int size;
	unsigned int offset;
	const char *name;
};

inline void rtl838x_mask_port_reg(u64 clear, u64 set, int reg)
{
	sw_w32_mask((u32)clear, (u32)set, reg);
}

inline void rtl838x_set_port_reg(u64 set, int reg)
{
	sw_w32((u32)set, reg);
}

inline u64 rtl838x_get_port_reg(int reg)
{
	return ((u64) sw_r32(reg));
}

inline void rtl839x_mask_port_reg_be(u64 clear, u64 set, int reg)
{
	sw_w32_mask((u32)(clear >> 32), (u32)(set >> 32), reg);
	sw_w32_mask((u32)(clear & 0xffffffff), (u32)(set & 0xffffffff), reg + 4);
}

inline void rtl839x_mask_port_reg_le(u64 clear, u64 set, int reg)
{
	sw_w32_mask((u32)clear, (u32)set, reg);
	sw_w32_mask((u32)(clear >> 32), (u32)(set >> 32), reg + 4);
}

inline void rtl839x_set_port_reg_be(u64 set, int reg)
{
	sw_w32(set >> 32, reg);
	sw_w32(set & 0xffffffff, reg + 4);
}

inline void rtl839x_set_port_reg_le(u64 set, int reg)
{
	sw_w32(set, reg);
	sw_w32(set >> 32, reg + 4);
}

inline u64 rtl839x_get_port_reg_be(int reg)
{
	u64 v = sw_r32(reg);

	v <<= 32;
	v |= sw_r32(reg + 4);
	return v;
}

inline u64 rtl839x_get_port_reg_le(int reg)
{
	u64 v = sw_r32(reg + 4);

	v <<= 32;
	v |= sw_r32(reg);
	return v;
}

inline int rtl838x_port_iso_ctrl(int p)
{
	return RTL838X_PORT_ISO_CTRL(p);
}

inline int rtl839x_port_iso_ctrl(int p)
{
	return RTL839X_PORT_ISO_CTRL(p);
}

inline void rtl838x_exec_tbl0_cmd(u32 cmd)
{
	sw_w32(cmd, RTL838X_TBL_ACCESS_CTRL_0);
	do { } while (sw_r32(RTL838X_TBL_ACCESS_CTRL_0) & (1 << 15));
}

inline void rtl839x_exec_tbl0_cmd(u32 cmd)
{
	sw_w32(cmd, RTL839X_TBL_ACCESS_CTRL_0);
	do { } while (sw_r32(RTL839X_TBL_ACCESS_CTRL_0) & (1 << 16));
}

inline void rtl930x_exec_tbl0_cmd(u32 cmd)
{
	sw_w32(cmd, RTL930X_TBL_ACCESS_CTRL_0);
	do { } while (sw_r32(RTL930X_TBL_ACCESS_CTRL_0) & (1 << 17));
}

inline void rtl931x_exec_tbl0_cmd(u32 cmd)
{
	sw_w32(cmd, RTL931X_TBL_ACCESS_CTRL_0);
	do { } while (sw_r32(RTL931X_TBL_ACCESS_CTRL_0) & (1 << 20));
}

inline void rtl838x_exec_tbl1_cmd(u32 cmd)
{
	sw_w32(cmd, RTL838X_TBL_ACCESS_CTRL_1);
	do { } while (sw_r32(RTL838X_TBL_ACCESS_CTRL_1) & (1 << 15));
}

inline void rtl839x_exec_tbl1_cmd(u32 cmd)
{
	sw_w32(cmd, RTL839X_TBL_ACCESS_CTRL_1);
	do { } while (sw_r32(RTL839X_TBL_ACCESS_CTRL_1) & (1 << 15));
}

inline void rtl930x_exec_tbl1_cmd(u32 cmd)
{
	sw_w32(cmd, RTL930X_TBL_ACCESS_CTRL_1);
	do { } while (sw_r32(RTL930X_TBL_ACCESS_CTRL_1) & (1 << 17));
}

inline void rtl931x_exec_tbl1_cmd(u32 cmd)
{
	sw_w32(cmd, RTL931X_TBL_ACCESS_CTRL_1);
	do { } while (sw_r32(RTL931X_TBL_ACCESS_CTRL_1) & (1 << 17));
}

inline void rtl839x_exec_tbl2_cmd(u32 cmd)
{
	sw_w32(cmd, RTL839X_TBL_ACCESS_CTRL_2);
	do { } while (sw_r32(RTL839X_TBL_ACCESS_CTRL_2) & (1 << 9));
}

inline void rtl930x_exec_tbl2_cmd(u32 cmd)
{
	sw_w32(cmd, RTL930X_TBL_ACCESS_CTRL_2);
	do { } while (sw_r32(RTL930X_TBL_ACCESS_CTRL_2) & (1 << 15));
}

inline void rtl931x_exec_tbl2_cmd(u32 cmd)
{
	sw_w32(cmd, RTL931X_TBL_ACCESS_CTRL_2);
	do { } while (sw_r32(RTL931X_TBL_ACCESS_CTRL_2) & (1 << 19));
}

inline int rtl838x_tbl_access_data_0(int i)
{
	return RTL838X_TBL_ACCESS_DATA_0(i);
}

inline int rtl839x_tbl_access_data_0(int i)
{
	return RTL839X_TBL_ACCESS_DATA_0(i);
}

inline int rtl930x_tbl_access_data_0(int i)
{
	return RTL930X_TBL_ACCESS_DATA_0(i);
}

inline int rtl931x_tbl_access_data_0(int i)
{
	return RTL931X_TBL_ACCESS_DATA_0(i);
}

inline static int rtl838x_trk_mbr_ctr(int group)
{
	return RTL838X_TRK_MBR_CTR + (group << 2);
}

inline static int rtl839x_trk_mbr_ctr(int group)
{
	return RTL839X_TRK_MBR_CTR + (group << 3);
}

static void rtl839x_vlan_tables_read(u32 vlan, struct rtl838x_vlan_info *info)
{
	u32 cmd;
	u64 v;
	u32 u, w;

	cmd = 1 << 16 /* Execute cmd */
		| 0 << 15 /* Read */
		| 0 << 12 /* Table type 0b000 */
		| (vlan & 0xfff);
	rtl839x_exec_tbl0_cmd(cmd);

	v = sw_r32(RTL839X_TBL_ACCESS_DATA_0(0));
	v <<= 32;
	u = sw_r32(RTL839X_TBL_ACCESS_DATA_0(1));
	v |= u;
	info->tagged_ports = v >> 11;

	w = sw_r32(RTL839X_TBL_ACCESS_DATA_0(2));

	info->profile_id = w >> 30 | ((u & 1) << 2);
	info->hash_mc_fid = !!(u & 2);
	info->hash_uc_fid = !!(u & 4);
	info->fid = (u >> 3) & 0xff;

	cmd = 1 << 15 /* Execute cmd */
		| 0 << 14 /* Read */
		| 0 << 12 /* Table type 0b00 */
		| (vlan & 0xfff);
	rtl839x_exec_tbl1_cmd(cmd);
	v = sw_r32(RTL839X_TBL_ACCESS_DATA_1(0));
	v <<= 32;
	v |= sw_r32(RTL839X_TBL_ACCESS_DATA_1(1));
	info->untagged_ports = v >> 11;
}

static void rtl838x_vlan_tables_read(u32 vlan, struct rtl838x_vlan_info *info)
{
	u32 cmd, v;

	cmd = 1 << 15 /* Execute cmd */
		| 1 << 14 /* Read */
		| 0 << 12 /* Table type 0b00 */
		| (vlan & 0xfff);
	rtl838x_exec_tbl0_cmd(cmd);
	info->tagged_ports = sw_r32(RTL838X_TBL_ACCESS_DATA_0(0));
	v = sw_r32(RTL838X_TBL_ACCESS_DATA_0(1));
	info->profile_id = v & 0x7;
	info->hash_mc_fid = !!(v & 0x8);
	info->hash_uc_fid = !!(v & 0x10);
	info->fid = (v >> 5) & 0x3f;

	cmd = 1 << 15 /* Execute cmd */
		| 1 << 14 /* Read */
		| 0 << 12 /* Table type 0b00 */
		| (vlan & 0xfff);
	rtl838x_exec_tbl1_cmd(cmd);
	info->untagged_ports = sw_r32(RTL838X_TBL_ACCESS_DATA_1(0));
}

static void rtl930x_vlan_tables_read(u32 vlan, struct rtl838x_vlan_info *info)
{
	u32 v, w;
	// Read VLAN table (0) via register 0
	struct table_reg *r = rtl_table_get(RTL9300_TBL_0, 1);

	rtl_table_read(r, vlan);
	v = sw_r32(rtl_table_data(r, 0));
	w = sw_r32(rtl_table_data(r, 1));
	pr_debug("VLAN_READ %d: %08x %08x\n", vlan, v, w);
	rtl_table_release(r);

	info->tagged_ports = v >> 3;
	info->profile_id = (w >> 24) & 7;
	info->hash_mc_fid = !!(w & BIT(27));
	info->hash_uc_fid = !!(w & BIT(28));
	info->fid = ((v & 0x7) << 3) | ((w >> 29) & 0x7);

	// Read UNTAG table via table register 2
	r = rtl_table_get(RTL9300_TBL_2, 0);
	rtl_table_read(r, vlan);
	v = sw_r32(rtl_table_data(r, 0));
	rtl_table_release(r);

	info->untagged_ports = v >> 3;
}


static void rtl839x_vlan_set_tagged(u32 vlan, struct rtl838x_vlan_info *info)
{
	u32 cmd = 1 << 16 /* Execute cmd */
		| 1 << 15 /* Write */
		| 0 << 12 /* Table type 0b00 */
		| (vlan & 0xfff);
	u32 w;
	u64 v = info->tagged_ports << 11;

	v |= info->profile_id >> 2;
	v |= info->hash_mc_fid ? 2 : 0;
	v |= info->hash_uc_fid ? 4 : 0;
	v |= ((u32)info->fid) << 3;
	rtl839x_set_port_reg_be(v, RTL839X_TBL_ACCESS_DATA_0(0));

	w = info->profile_id;
	sw_w32(w << 30, RTL839X_TBL_ACCESS_DATA_0(2));
	rtl839x_exec_tbl0_cmd(cmd);
}

static void rtl930x_vlan_set_tagged(u32 vlan, struct rtl838x_vlan_info *info)
{
	u32 v, w;
	// Access VLAN table (1) via register 0
	struct table_reg *r = rtl_table_get(RTL9300_TBL_0, 1);

	v = info->tagged_ports << 3;
	v |= ((u32)info->fid) >> 3;

	w = ((u32)info->fid) << 29;
	w |= info->hash_mc_fid ? BIT(27) : 0;
	w |= info->hash_uc_fid ? BIT(28) : 0;
	w |= info->profile_id << 24;

	sw_w32(v, rtl_table_data(r, 0));
	sw_w32(w, rtl_table_data(r, 1));

	rtl_table_write(r, vlan);
	rtl_table_release(r);
}

void rtl838x_vlan_profile_dump(int index)
{
	u32 profile;

	if (index < 0 || index > 7)
		return;

	profile = sw_r32(RTL838X_VLAN_PROFILE(index));

	pr_info("VLAN %d: L2 learning: %d, L2 Unknown MultiCast Field %x, \
		IPv4 Unknown MultiCast Field %x, IPv6 Unknown MultiCast Field: %x",
		index, profile & 1, (profile >> 1) & 0x1ff, (profile >> 10) & 0x1ff,
		(profile >> 19) & 0x1ff);
}

void rtl839x_vlan_profile_dump(int index)
{
	u32 profile, profile1;

	if (index < 0 || index > 7)
		return;

	profile1 = sw_r32(RTL839X_VLAN_PROFILE(index) + 4);
	profile = sw_r32(RTL839X_VLAN_PROFILE(index));

	pr_info("VLAN %d: L2 learning: %d, L2 Unknown MultiCast Field %x, \
		IPv4 Unknown MultiCast Field %x, IPv6 Unknown MultiCast Field: %x",
		index, profile & 1, (profile >> 1) & 0xfff, (profile >> 13) & 0xfff,
		(profile1) & 0xfff);
}

void rtl930x_vlan_profile_dump(int index)
{
	u32 profile[5];

	if (index < 0 || index > 7)
		return;

	profile[0] = sw_r32(RTL930X_VLAN_PROFILE_SET(index));
	profile[1] = sw_r32(RTL930X_VLAN_PROFILE_SET(index) + 4);
	profile[2] = sw_r32(RTL930X_VLAN_PROFILE_SET(index) + 8) & 0x1FFFFFFF;
	profile[3] = sw_r32(RTL930X_VLAN_PROFILE_SET(index) + 12) & 0x1FFFFFFF;
	profile[4] = sw_r32(RTL930X_VLAN_PROFILE_SET(index) + 16) & 0x1FFFFFFF;

	pr_info("VLAN %d: L2 learning: %d, L2 Unknown MultiCast Field %x, \
		IPv4 Unknown MultiCast Field %x, IPv6 Unknown MultiCast Field: %x",
		index, profile[0] & (3 << 21), profile[2], profile[3], profile[4]);
}

void rtl931x_vlan_profile_dump(int index)
{
	u64 profile[4];

	if (index < 0 || index > 15)
		return;

	profile[0] = sw_r32(RTL931X_VLAN_PROFILE_SET(index));
	profile[1] = (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 4) & 0x1FFFFFFFULL) << 32
		| (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 8) & 0xFFFFFFFF);
	profile[2] = (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 16) & 0xFFFFFFFFULL) << 32 
		| (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 12) & 0x1FFFFFFULL);
	profile[3] = (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 20) & 0x1FFFFFFFULL) << 32
		| (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 24) & 0xFFFFFFFF);

	pr_info("VLAN %d: L2 learning: %d, L2 Unknown MultiCast Field %llx, \
		IPv4 Unknown MultiCast Field %llx, IPv6 Unknown MultiCast Field: %llx",
		index, (u32) (profile[0] & (3 << 14)), profile[1], profile[2], profile[3]);
}

static void rtl838x_vlan_set_tagged(u32 vlan, struct rtl838x_vlan_info *info)
{
	u32 cmd = 1 << 15 /* Execute cmd */
		| 0 << 14 /* Write */
		| 0 << 12 /* Table type 0b00 */
		| (vlan & 0xfff);
	u32 v;

	sw_w32(info->tagged_ports, RTL838X_TBL_ACCESS_DATA_0(0));

	v = info->profile_id;
	v |= info->hash_mc_fid ? 0x8 : 0;
	v |= info->hash_uc_fid ? 0x10 : 0;
	v |= ((u32)info->fid) << 5;

	sw_w32(v, RTL838X_TBL_ACCESS_DATA_0(1));
	rtl838x_exec_tbl0_cmd(cmd);
}

static void rtl838x_vlan_set_untagged(u32 vlan, u64 portmask)
{
	u32 cmd = 1 << 15 /* Execute cmd */
		| 0 << 14 /* Write */
		| 0 << 12 /* Table type 0b00 */
		| (vlan & 0xfff);
	sw_w32(portmask & 0x1fffffff, RTL838X_TBL_ACCESS_DATA_1(0));
	rtl838x_exec_tbl1_cmd(cmd);
}

static void rtl839x_vlan_set_untagged(u32 vlan, u64 portmask)
{
	u32 cmd = 1 << 16 /* Execute cmd */
		| 1 << 15 /* Write */
		| 0 << 12 /* Table type 0b00 */
		| (vlan & 0xfff);
	rtl839x_set_port_reg_be(portmask << 11, RTL839X_TBL_ACCESS_DATA_1(0));
	rtl839x_exec_tbl1_cmd(cmd);
}

static void rtl930x_vlan_set_untagged(u32 vlan, u64 portmask)
{
	struct table_reg *r = rtl_table_get(RTL9300_TBL_2, 0);

	sw_w32(portmask << 3, rtl_table_data(r, 0));
	rtl_table_write(r, vlan);
	rtl_table_release(r);
}

static void rtl838x_stp_get(struct rtl838x_switch_priv *priv, u16 msti, u32 port_state[])
{
	int i;
	u32 cmd = 1 << 15 /* Execute cmd */
		| 1 << 14 /* Read */
		| 2 << 12 /* Table type 0b10 */
		| (msti & 0xfff);
	priv->r->exec_tbl0_cmd(cmd);

	for (i = 0; i < 2; i++)
		port_state[i] = sw_r32(priv->r->tbl_access_data_0(i));
}

static void rtl839x_stp_get(struct rtl838x_switch_priv *priv, u16 msti, u32 port_state[])
{
	int i;
	u32 cmd = 1 << 16 /* Execute cmd */
		| 0 << 15 /* Read */
		| 5 << 12 /* Table type 0b101 */
		| (msti & 0xfff);
	priv->r->exec_tbl0_cmd(cmd);

	for (i = 0; i < 4; i++)
		port_state[i] = sw_r32(priv->r->tbl_access_data_0(i));
}

static void rtl930x_stp_get(struct rtl838x_switch_priv *priv, u16 msti, u32 port_state[])
{
	int i;
	u32 cmd = 1 << 17 /* Execute cmd */
		| 0 << 16 /* Read */
		| 4 << 12 /* Table type 0b10 */
		| (msti & 0xfff);
	priv->r->exec_tbl0_cmd(cmd);

	for (i = 0; i < 2; i++)
		port_state[i] = sw_r32(RTL930X_TBL_ACCESS_DATA_0(i));
	pr_debug("MSTI: %d STATE: %08x, %08x\n", msti, port_state[0], port_state[1]);
}

static void rtl931x_stp_get(struct rtl838x_switch_priv *priv, u16 msti, u32 port_state[])
{
	int i;
	u32 cmd = 1 << 20 /* Execute cmd */
		| 0 << 19 /* Read */
		| 2 << 15 /* Table type 0b10 */
		| (msti & 0x3fff);
	priv->r->exec_tbl0_cmd(cmd);

	for (i = 0; i < 4; i++)
		port_state[i] = sw_r32(priv->r->tbl_access_data_0(i));
}

static void rtl838x_stp_set(struct rtl838x_switch_priv *priv, u16 msti, u32 port_state[])
{
	int i;
	u32 cmd = 1 << 15 /* Execute cmd */
		| 0 << 14 /* Write */
		| 2 << 12 /* Table type 0b10 */
		| (msti & 0xfff);

	for (i = 0; i < 2; i++)
		sw_w32(port_state[i], priv->r->tbl_access_data_0(i));
	priv->r->exec_tbl0_cmd(cmd);
}

static void rtl839x_stp_set(struct rtl838x_switch_priv *priv, u16 msti, u32 port_state[])
{
	int i;
	u32 cmd = 1 << 16 /* Execute cmd */
		| 1 << 15 /* Write */
		| 5 << 12 /* Table type 0b101 */
		| (msti & 0xfff);
	for (i = 0; i < 4; i++)
		sw_w32(port_state[i], priv->r->tbl_access_data_0(i));
	priv->r->exec_tbl0_cmd(cmd);
}

static void rtl930x_stp_set(struct rtl838x_switch_priv *priv, u16 msti, u32 port_state[])
{
	int i;
	u32 cmd = 1 << 17 /* Execute cmd */
		| 1 << 16 /* Write */
		| 4 << 12 /* Table type 4 */
		| (msti & 0xfff);

	for (i = 0; i < 2; i++)
		sw_w32(port_state[i], RTL930X_TBL_ACCESS_DATA_0(i));
	priv->r->exec_tbl0_cmd(cmd);
}

static void rtl931x_stp_set(struct rtl838x_switch_priv *priv, u16 msti, u32 port_state[])
{
	int i;
	u32 cmd = 1 << 20 /* Execute cmd */
		| 1 << 19 /* Write */
		| 5 << 15 /* Table type 0b101 */
		| (msti & 0x3fff);
	for (i = 0; i < 4; i++)
		sw_w32(port_state[i], priv->r->tbl_access_data_0(i));
	priv->r->exec_tbl0_cmd(cmd);
}

static inline int rtl838x_mac_force_mode_ctrl(int p)
{
	return RTL838X_MAC_FORCE_MODE_CTRL + (p << 2);
}

static inline int rtl839x_mac_force_mode_ctrl(int p)
{
	return RTL839X_MAC_FORCE_MODE_CTRL + (p << 2);
}

static inline int rtl930x_mac_force_mode_ctrl(int p)
{
	return RTL930X_MAC_FORCE_MODE_CTRL + (p << 2);
}

static inline int rtl838x_mac_port_ctrl(int p)
{
	return RTL838X_MAC_PORT_CTRL(p);
}

static inline int rtl839x_mac_port_ctrl(int p)
{
	return RTL839X_MAC_PORT_CTRL(p);
}

static inline int rtl930x_mac_port_ctrl(int p)
{
	return RTL930X_MAC_L2_PORT_CTRL(p);
}

static inline int rtl838x_l2_port_new_salrn(int p)
{
	return RTL838X_L2_PORT_NEW_SALRN(p);
}

static inline int rtl839x_l2_port_new_salrn(int p)
{
	return RTL839X_L2_PORT_NEW_SALRN(p);
}

static inline int rtl838x_l2_port_new_sa_fwd(int p)
{
	return RTL838X_L2_PORT_NEW_SA_FWD(p);
}

static inline int rtl839x_l2_port_new_sa_fwd(int p)
{
	return RTL839X_L2_PORT_NEW_SA_FWD(p);
}

static inline int rtl838x_mac_link_spd_sts(int p)
{
	return RTL838X_MAC_LINK_SPD_STS(p);
}

static inline int rtl839x_mac_link_spd_sts(int p)
{
	return RTL839X_MAC_LINK_SPD_STS(p);
}

static inline int rtl930x_mac_link_spd_sts(int p)
{
	return RTL930X_MAC_LINK_SPD_STS(p);
}

static inline int rtl838x_mir_ctrl(int group)
{
	return RTL838X_MIR_CTRL(group);
}

static inline int rtl839x_mir_ctrl(int group)
{
	return RTL839X_MIR_CTRL(group);
}

static inline int rtl838x_mir_dpm(int group)
{
	return RTL838X_MIR_DPM_CTRL(group);
}

static inline int rtl839x_mir_dpm(int group)
{
	return RTL839X_MIR_DPM_CTRL(group);
}

static inline int rtl838x_mir_spm(int group)
{
	return RTL838X_MIR_SPM_CTRL(group);
}

static inline int rtl839x_mir_spm(int group)
{
	return RTL839X_MIR_SPM_CTRL(group);
}

static u64 rtl838x_read_l2_entry_using_hash(u32 hash, u32 position, struct rtl838x_l2_entry *e)
{
	u64 entry;
	u32 r[3];

	/* Search in SRAM, with hash and at position in hash bucket (0-3) */
	u32 idx = (0 << 14) | (hash << 2) | position;

	u32 cmd = 1 << 16 /* Execute cmd */
		| 1 << 15 /* Read */
		| 0 << 13 /* Table type 0b00 */
		| (idx & 0x1fff);

	sw_w32(cmd, RTL838X_TBL_ACCESS_L2_CTRL);
	do { }  while (sw_r32(RTL838X_TBL_ACCESS_L2_CTRL) & (1 << 16));
	r[0] = sw_r32(RTL838X_TBL_ACCESS_L2_DATA(0));
	r[1] = sw_r32(RTL838X_TBL_ACCESS_L2_DATA(1));
	r[2] = sw_r32(RTL838X_TBL_ACCESS_L2_DATA(2));

	e->mac[0] = (r[1] >> 20);
	e->mac[1] = (r[1] >> 12);
	e->mac[2] = (r[1] >> 4);
	e->mac[3] = (r[1] & 0xf) << 4 | (r[2] >> 28);
	e->mac[4] = (r[2] >> 20);
	e->mac[5] = (r[2] >> 12);
	e->is_static = !!((r[0] >> 19) & 1);
	e->vid = r[0] & 0xfff;
	e->rvid = r[2] & 0xfff;
	e->port = (r[0] >> 12) & 0x1f;

	e->valid = true;
	if (!(r[0] >> 17)) /* Check for invalid entry */
		e->valid = false;

	if (e->valid)
		pr_info("Found in Hash: R1 %x R2 %x R3 %x\n", r[0], r[1], r[2]);

	entry = (((u64) r[1]) << 32) | (r[2] & 0xfffff000) | (r[0] & 0xfff);
	return entry;
}

static void rtl839x_fill_l2_entry(u32 r[], struct rtl838x_l2_entry *e)
{
	/* Table contains different entry types, we need to identify the right one:
	 * Check for MC entries, first
	 */
	e->is_ip_mc = !!(r[2] & BIT(31));
	e->is_ipv6_mc = !!(r[2] & BIT(30));
	e->type = L2_INVALID;
	if (!e->is_ip_mc) {
		e->mac[0] = (r[0] >> 12);
		e->mac[1] = (r[0] >> 4);
		e->mac[2] = ((r[1] >> 28) | (r[0] << 4));
		e->mac[3] = (r[1] >> 20);
		e->mac[4] = (r[1] >> 12);
		e->mac[5] = (r[1] >> 4);

		/* Is it a unicast entry? check multicast bit */
		if (!(e->mac[0] & 1)) {
			e->is_static = !!((r[2] >> 18) & 1);
			e->vid = (r[2] >> 4) & 0xfff;
			e->rvid = (r[0] >> 20) & 0xfff;
			e->port = (r[2] >> 24) & 0x3f;
			e->block_da = !!(r[2] & (1 << 19));
			e->block_sa = !!(r[2] & (1 << 20));
			e->suspended = !!(r[2] & (1 << 17));
			e->next_hop = !!(r[2] & (1 << 16));
			if (e->next_hop)
				pr_info("Found next hop entry, need to read data\n");
			e->age = (r[2] >> 21) & 3;
			e->valid = true;
			if (!(r[2] & 0xc0fd0000)) /* Check for valid entry */
				e->valid = false;
			else
				e->type = L2_UNICAST;
		} else {
			e->valid = true;
			e->type = L2_MULTICAST;
			e->mc_portmask_index = (r[2]>>6) & 0xfff;
		}
	}
	if (e->is_ip_mc) {
		e->valid = true;
		e->type = IP4_MULTICAST;
	}
	if (e->is_ipv6_mc) {
		e->valid = true;
		e->type = IP6_MULTICAST;
	}
}

static u64 rtl839x_read_l2_entry_using_hash(u32 hash, u32 position, struct rtl838x_l2_entry *e)
{
	u64 entry;
	u32 r[3];

	/* Search in SRAM, with hash and at position in hash bucket (0-3) */
	u32 idx = (0 << 14) | (hash << 2) | position;

	u32 cmd = 1 << 17 /* Execute cmd */
		| 0 << 16 /* Read */
		| 0 << 14 /* Table type 0b00 */
		| (idx & 0x3fff);

	sw_w32(cmd, RTL839X_TBL_ACCESS_L2_CTRL);
	do { }  while (sw_r32(RTL839X_TBL_ACCESS_L2_CTRL) & (1 << 17));
	r[0] = sw_r32(RTL839X_TBL_ACCESS_L2_DATA(0));
	r[1] = sw_r32(RTL839X_TBL_ACCESS_L2_DATA(1));
	r[2] = sw_r32(RTL839X_TBL_ACCESS_L2_DATA(2));

	rtl839x_fill_l2_entry(r, e);

	entry = (((u64) r[0]) << 12) | ((r[1] & 0xfffffff0) << 12) | ((r[2] >> 4) & 0xfff);
	return entry;
}

static void rtl930x_fill_l2_entry(u32 r[], struct rtl838x_l2_entry *e)
{
	e->valid = !!(r[2] & BIT(31));
	if (!e->valid)
		return;

	// TODO: Is there not a function to copy directly MAC memory?
	e->mac[0] = (r[0] >> 24);
	e->mac[1] = (r[0] >> 16);
	e->mac[2] = (r[0] >> 8);
	e->mac[3] = r[0];
	e->mac[4] = (r[1] >> 24);
	e->mac[5] = (r[1] >> 16);

	/* Is it a unicast entry? check multicast bit */
	if (!(e->mac[0] & 1)) {
		e->type = L2_UNICAST;
		e->is_static = !!(r[2] & BIT(14));
		e->vid = r[2] & 0xfff;
		e->rvid = r[1] & 0xfff;
		e->port = (r[2] >> 20) & 0x3ff;
		// Check for trunk port
		if (r[2] & BIT(30)) {
			e->stackDev = (e->port >> 9) & 1;
			e->trunk = e->port & 0x3f;
		} else {
			e->stackDev = (e->port >> 6) & 0xf;
			e->port = e->port & 0x3f;
		}

		e->block_da = !!(r[2] & BIT(15));
		e->block_sa = !!(r[2] & BIT(16));
		e->suspended = !!(r[2] & BIT(13));
		e->next_hop = !!(r[2] & BIT(12));
		e->age = (r[2] >> 17) & 3;
		e->valid = true;

	} else {
		e->valid = true;
		e->type = L2_MULTICAST;
		e->mc_portmask_index = (r[2]>>6) & 0xfff;
	}
}

static u64 rtl930x_read_l2_entry_using_hash(u32 hash, u32 position, struct rtl838x_l2_entry *e)
{
	u64 entry;
	u32 r[3];
	struct table_reg *q = rtl_table_get(RTL9300_TBL_L2, 0);
	u32 idx = (0 << 14) | (hash << 2) | position;
	int i;

	rtl_table_read(q, idx);
	for (i= 0; i < 3; i++)
		r[i] = sw_r32(rtl_table_data(q, i));

	rtl_table_release(q);

	rtl930x_fill_l2_entry(r, e);
	if (!e->valid)
		return 0;

	entry = ((u64)r[0] << 32) | (r[1] & 0xffff0000) | e->vid;
	return entry;
}

static u64 rtl838x_read_cam(int idx, struct rtl838x_l2_entry *e)
{
	u64 entry;
	u32 r[3];

	u32 cmd = 1 << 16 /* Execute cmd */
		| 1 << 15 /* Read */
		| 1 << 13 /* Table type 0b01 */
		| (idx & 0x3f);
	sw_w32(cmd, RTL838X_TBL_ACCESS_L2_CTRL);
	do { }  while (sw_r32(RTL838X_TBL_ACCESS_L2_CTRL) & (1 << 16));
	r[0] = sw_r32(RTL838X_TBL_ACCESS_L2_DATA(0));
	r[1] = sw_r32(RTL838X_TBL_ACCESS_L2_DATA(1));
	r[2] = sw_r32(RTL838X_TBL_ACCESS_L2_DATA(2));

	e->mac[0] = (r[1] >> 20);
	e->mac[1] = (r[1] >> 12);
	e->mac[2] = (r[1] >> 4);
	e->mac[3] = (r[1] & 0xf) << 4 | (r[2] >> 28);
	e->mac[4] = (r[2] >> 20);
	e->mac[5] = (r[2] >> 12);
	e->is_static = !!((r[0] >> 19) & 1);
	e->vid = r[0] & 0xfff;
	e->rvid = r[2] & 0xfff;
	e->port = (r[0] >> 12) & 0x1f;

	e->valid = true;
	if (!(r[0] >> 17)) /* Check for invalid entry */
		e->valid = false;

	if (e->valid)
		pr_info("Found in CAM: R1 %x R2 %x R3 %x\n", r[0], r[1], r[2]);

	entry = (((u64) r[1]) << 32) | (r[2] & 0xfffff000) | (r[0] & 0xfff);
	return entry;
}

static u64 rtl839x_read_cam(int idx, struct rtl838x_l2_entry *e)
{
	u64 entry;
	u32 r[3];

	u32 cmd = 1 << 17 /* Execute cmd */
		| 0 << 16 /* Read */
		| 1 << 14 /* Table type 0b01 */
		| (idx & 0x3f);
	sw_w32(cmd, RTL839X_TBL_ACCESS_L2_CTRL);
	do { }  while (sw_r32(RTL839X_TBL_ACCESS_L2_CTRL) & (1 << 17));
	r[0] = sw_r32(RTL839X_TBL_ACCESS_L2_DATA(0));
	r[1] = sw_r32(RTL839X_TBL_ACCESS_L2_DATA(1));
	r[2] = sw_r32(RTL839X_TBL_ACCESS_L2_DATA(2));


	rtl839x_fill_l2_entry(r, e);
	if (e->valid)
		pr_info("Found in CAM: R1 %x R2 %x R3 %x\n", r[0], r[1], r[2]);
	else
		return 0;

	entry = (((u64) r[0]) << 12) | ((r[1] & 0xfffffff0) << 12) | ((r[2] >> 4) & 0xfff);
	return entry;
}

static u64 rtl930x_read_cam(int idx, struct rtl838x_l2_entry *e)
{
	u64 entry;
	u32 r[3];
	struct table_reg *q = rtl_table_get(RTL9300_TBL_L2, 1);
	int i;

	rtl_table_read(q, idx);
	for (i= 0; i < 3; i++)
		r[i] = sw_r32(rtl_table_data(q, i));

	rtl_table_release(q);

	rtl930x_fill_l2_entry(r, e);
	if (!e->valid)
		return 0;

	entry = ((u64)r[0] << 32) | (r[1] & 0xffff0000) | e->vid;

	return entry;
}

static void rtl839x_read_scheduling_table(int port)
{
	u32 cmd = 1 << 9 /* Execute cmd */
		| 0 << 8 /* Read */
		| 0 << 6 /* Table type 0b00 */
		| (port & 0x3f);
	rtl839x_exec_tbl2_cmd(cmd);
}

u64 rtl930x_traffic_get(int source)
{
	u32 v;
	struct table_reg *r = rtl_table_get(RTL9300_TBL_0, 6);

	rtl_table_read(r, source);
	v = sw_r32(rtl_table_data(r, 0));
	rtl_table_release(r);
	return v >> 3;
}

/*
 * Enable traffic between a source port and a destination port matrix
 */
void rtl930x_traffic_set(int source, u64 dest_matrix)
{
	struct table_reg *r = rtl_table_get(RTL9300_TBL_0, 6);

	sw_w32((dest_matrix << 3), rtl_table_data(r, 0));
	rtl_table_write(r, source);
	rtl_table_release(r);
}

void rtl930x_traffic_enable(int source, int dest)
{
	struct table_reg *r = rtl_table_get(RTL9300_TBL_0, 6);
	rtl_table_read(r, source);
	sw_w32_mask(0, BIT(dest + 3), rtl_table_data(r, 0));
	rtl_table_write(r, source);
	rtl_table_release(r);
}

void rtl930x_traffic_disable(int source, int dest)
{
	struct table_reg *r = rtl_table_get(RTL9300_TBL_0, 6);
	rtl_table_read(r, source);
	sw_w32_mask(BIT(dest + 3), 0, rtl_table_data(r, 0));
	rtl_table_write(r, source);
	rtl_table_release(r);
}

u64 rtl838x_traffic_get(int source)
{
	return rtl838x_get_port_reg(rtl838x_port_iso_ctrl(source));
}

void rtl838x_traffic_set(int source, u64 dest_matrix)
{
	rtl838x_set_port_reg(dest_matrix, rtl838x_port_iso_ctrl(source));
}

void rtl838x_traffic_enable(int source, int dest)
{
	rtl838x_mask_port_reg(BIT(dest), 0, rtl838x_port_iso_ctrl(source));
}

void rtl838x_traffic_disable(int source, int dest)
{
	rtl838x_mask_port_reg(BIT(dest), 0, rtl838x_port_iso_ctrl(source));
}

u64 rtl839x_traffic_get(int source)
{
	return rtl839x_get_port_reg_be(rtl839x_port_iso_ctrl(source));
}

void rtl839x_traffic_set(int source, u64 dest_matrix)
{
	rtl839x_set_port_reg_be(dest_matrix, rtl839x_port_iso_ctrl(source));
}

void rtl839x_traffic_enable(int source, int dest)
{
	rtl839x_mask_port_reg_be(0, BIT_ULL(dest), rtl839x_port_iso_ctrl(source));
}

void rtl839x_traffic_disable(int source, int dest)
{
	rtl839x_mask_port_reg_be(0, BIT(dest), rtl839x_port_iso_ctrl(source));
}

static const struct rtl838x_reg rtl838x_reg = {
	.mask_port_reg_be = rtl838x_mask_port_reg,
	.set_port_reg_be = rtl838x_set_port_reg,
	.get_port_reg_be = rtl838x_get_port_reg,
	.mask_port_reg_le = rtl838x_mask_port_reg,
	.set_port_reg_le = rtl838x_set_port_reg,
	.get_port_reg_le = rtl838x_get_port_reg,
	.stat_port_rst = RTL838X_STAT_PORT_RST,
	.stat_rst = RTL838X_STAT_RST,
	.stat_port_std_mib = RTL838X_STAT_PORT_STD_MIB,
	.port_iso_ctrl = rtl838x_port_iso_ctrl,
	.traffic_enable = rtl838x_traffic_enable,
	.traffic_disable = rtl838x_traffic_disable,
	.traffic_get = rtl838x_traffic_get,
	.traffic_set = rtl838x_traffic_set,
	.l2_ctrl_0 = RTL838X_L2_CTRL_0,
	.l2_ctrl_1 = RTL838X_L2_CTRL_1,
	.l2_port_aging_out = RTL838X_L2_PORT_AGING_OUT,
	.smi_poll_ctrl = RTL838X_SMI_POLL_CTRL,
	.l2_tbl_flush_ctrl = RTL838X_L2_TBL_FLUSH_CTRL,
	.exec_tbl0_cmd = rtl838x_exec_tbl0_cmd,
	.exec_tbl1_cmd = rtl838x_exec_tbl1_cmd,
	.tbl_access_data_0 = rtl838x_tbl_access_data_0,
	.isr_glb_src = RTL838X_ISR_GLB_SRC,
	.isr_port_link_sts_chg = RTL838X_ISR_PORT_LINK_STS_CHG,
	.imr_port_link_sts_chg = RTL838X_IMR_PORT_LINK_STS_CHG,
	.imr_glb = RTL838X_IMR_GLB,
	.vlan_tables_read = rtl838x_vlan_tables_read,
	.vlan_set_tagged = rtl838x_vlan_set_tagged,
	.vlan_set_untagged = rtl838x_vlan_set_untagged,
	.vlan_profile_dump = rtl838x_vlan_profile_dump,
	.stp_get = rtl838x_stp_get,
	.stp_set = rtl838x_stp_set,
	.mac_force_mode_ctrl = rtl838x_mac_force_mode_ctrl,
	.mac_port_ctrl = rtl838x_mac_port_ctrl,
	.l2_port_new_salrn = rtl838x_l2_port_new_salrn,
	.l2_port_new_sa_fwd = rtl838x_l2_port_new_sa_fwd,
	.mir_ctrl = rtl838x_mir_ctrl,
	.mir_dpm = rtl838x_mir_dpm,
	.mir_spm = rtl838x_mir_spm,
	.mac_link_sts = RTL838X_MAC_LINK_STS,
	.mac_link_dup_sts = RTL838X_MAC_LINK_DUP_STS,
	.mac_link_spd_sts = rtl838x_mac_link_spd_sts,
	.mac_rx_pause_sts = RTL838X_MAC_RX_PAUSE_STS,
	.mac_tx_pause_sts = RTL838X_MAC_TX_PAUSE_STS,
	.read_l2_entry_using_hash = rtl838x_read_l2_entry_using_hash,
	.read_cam = rtl838x_read_cam,
	.vlan_port_egr_filter = RTL838X_VLAN_PORT_EGR_FLTR,
	.vlan_port_igr_filter = RTL838X_VLAN_PORT_IGR_FLTR(0),
	.vlan_port_pb = RTL838X_VLAN_PORT_PB_VLAN,
	.vlan_port_tag_sts_ctrl = RTL838X_VLAN_PORT_TAG_STS_CTRL,
	.trk_mbr_ctr = rtl838x_trk_mbr_ctr,
};

static const struct rtl838x_reg rtl839x_reg = {
	.mask_port_reg_be = rtl839x_mask_port_reg_be,
	.set_port_reg_be = rtl839x_set_port_reg_be,
	.get_port_reg_be = rtl839x_get_port_reg_be,
	.mask_port_reg_le = rtl839x_mask_port_reg_le,
	.set_port_reg_le = rtl839x_set_port_reg_le,
	.get_port_reg_le = rtl839x_get_port_reg_le,
	.stat_port_rst = RTL839X_STAT_PORT_RST,
	.stat_rst = RTL839X_STAT_RST,
	.stat_port_std_mib = RTL839X_STAT_PORT_STD_MIB,
	.port_iso_ctrl = rtl839x_port_iso_ctrl,
	.traffic_enable = rtl839x_traffic_enable,
	.traffic_disable = rtl839x_traffic_disable,
	.traffic_get = rtl839x_traffic_get,
	.traffic_set = rtl839x_traffic_set,
	.l2_ctrl_0 = RTL839X_L2_CTRL_0,
	.l2_ctrl_1 = RTL839X_L2_CTRL_1,
	.l2_port_aging_out = RTL839X_L2_PORT_AGING_OUT,
	.smi_poll_ctrl = RTL839X_SMI_PORT_POLLING_CTRL,
	.l2_tbl_flush_ctrl = RTL839X_L2_TBL_FLUSH_CTRL,
	.exec_tbl0_cmd = rtl839x_exec_tbl0_cmd,
	.exec_tbl1_cmd = rtl839x_exec_tbl1_cmd,
	.tbl_access_data_0 = rtl839x_tbl_access_data_0,
	.isr_glb_src = RTL839X_ISR_GLB_SRC,
	.isr_port_link_sts_chg = RTL839X_ISR_PORT_LINK_STS_CHG,
	.imr_port_link_sts_chg = RTL839X_IMR_PORT_LINK_STS_CHG,
	.imr_glb = RTL839X_IMR_GLB,
	.vlan_tables_read = rtl839x_vlan_tables_read,
	.vlan_set_tagged = rtl839x_vlan_set_tagged,
	.vlan_set_untagged = rtl839x_vlan_set_untagged,
	.vlan_profile_dump = rtl839x_vlan_profile_dump,
	.stp_get = rtl839x_stp_get,
	.stp_set = rtl839x_stp_set,
	.mac_force_mode_ctrl = rtl839x_mac_force_mode_ctrl,
	.mac_port_ctrl = rtl839x_mac_port_ctrl,
	.l2_port_new_salrn = rtl839x_l2_port_new_salrn,
	.l2_port_new_sa_fwd = rtl839x_l2_port_new_sa_fwd,
	.mir_ctrl = rtl839x_mir_ctrl,
	.mir_dpm = rtl839x_mir_dpm,
	.mir_spm = rtl839x_mir_spm,
	.mac_link_sts = RTL839X_MAC_LINK_STS,
	.mac_link_dup_sts = RTL839X_MAC_LINK_DUP_STS,
	.mac_link_spd_sts = rtl839x_mac_link_spd_sts,
	.mac_rx_pause_sts = RTL839X_MAC_RX_PAUSE_STS,
	.mac_tx_pause_sts = RTL839X_MAC_TX_PAUSE_STS,
	.read_l2_entry_using_hash = rtl839x_read_l2_entry_using_hash,
	.read_cam = rtl839x_read_cam,
	.vlan_port_egr_filter = RTL839X_VLAN_PORT_EGR_FLTR(0),
	.vlan_port_igr_filter = RTL839X_VLAN_PORT_IGR_FLTR(0),
	.vlan_port_pb = RTL839X_VLAN_PORT_PB_VLAN,
	.vlan_port_tag_sts_ctrl = RTL839X_VLAN_PORT_TAG_STS_CTRL,
	.trk_mbr_ctr = rtl839x_trk_mbr_ctr,
};

static const struct rtl838x_reg rtl930x_reg = {
	.mask_port_reg_be = rtl838x_mask_port_reg,
	.set_port_reg_be = rtl838x_set_port_reg,
	.get_port_reg_be = rtl838x_get_port_reg,
	.mask_port_reg_le = rtl838x_mask_port_reg,
	.set_port_reg_le = rtl838x_set_port_reg,
	.get_port_reg_le = rtl838x_get_port_reg,
	.stat_port_rst = RTL930X_STAT_PORT_RST,
	.stat_rst = RTL930X_STAT_RST,
	.stat_port_std_mib = RTL930X_STAT_PORT_MIB_CNTR,
	.traffic_enable = rtl930x_traffic_enable,
	.traffic_disable = rtl930x_traffic_disable,
	.traffic_get = rtl930x_traffic_get,
	.traffic_set = rtl930x_traffic_set,
	.l2_ctrl_0 = RTL930X_L2_CTRL,
	.l2_ctrl_1 = RTL930X_L2_AGE_CTRL,
	.l2_port_aging_out = RTL930X_L2_PORT_AGE_CTRL,
	.smi_poll_ctrl = RTL930X_SMI_POLL_CTRL, // TODO: Difference to RTL9300_SMI_PRVTE_POLLING_CTRL
	.l2_tbl_flush_ctrl = RTL930X_L2_TBL_FLUSH_CTRL,
	.exec_tbl0_cmd = rtl930x_exec_tbl0_cmd,
	.exec_tbl1_cmd = rtl930x_exec_tbl1_cmd,
	.tbl_access_data_0 = rtl930x_tbl_access_data_0,
	.isr_glb_src = RTL930X_ISR_GLB,
	.isr_port_link_sts_chg = RTL930X_ISR_PORT_LINK_STS_CHG,
	.imr_port_link_sts_chg = RTL930X_IMR_PORT_LINK_STS_CHG,
	.imr_glb = RTL930X_IMR_GLB,
	.vlan_tables_read = rtl930x_vlan_tables_read,
	.vlan_set_tagged = rtl930x_vlan_set_tagged,
	.vlan_set_untagged = rtl930x_vlan_set_untagged,
	.vlan_profile_dump = rtl930x_vlan_profile_dump,
	.stp_get = rtl930x_stp_get,
	.stp_set = rtl930x_stp_set,
	.mac_force_mode_ctrl = rtl930x_mac_force_mode_ctrl,
	.mac_port_ctrl = rtl930x_mac_port_ctrl,
	.l2_port_new_salrn = rtl839x_l2_port_new_salrn,
	.l2_port_new_sa_fwd = rtl839x_l2_port_new_sa_fwd,
	.mir_ctrl = rtl839x_mir_ctrl,
	.mir_dpm = rtl839x_mir_dpm,
	.mir_spm = rtl839x_mir_spm,
	.mac_link_sts = RTL930X_MAC_LINK_STS,
	.mac_link_dup_sts = RTL930X_MAC_LINK_DUP_STS,
	.mac_link_spd_sts = rtl930x_mac_link_spd_sts,
	.mac_rx_pause_sts = RTL930X_MAC_RX_PAUSE_STS,
	.mac_tx_pause_sts = RTL930X_MAC_TX_PAUSE_STS,
	.read_l2_entry_using_hash = rtl930x_read_l2_entry_using_hash,
	.read_cam = rtl930x_read_cam,
	.vlan_port_egr_filter = RTL930X_VLAN_PORT_EGR_FLTR,
	.vlan_port_igr_filter = RTL930X_VLAN_PORT_IGR_FLTR(0),
	.vlan_port_pb = RTL930X_VLAN_PORT_PB_VLAN,
	.vlan_port_tag_sts_ctrl = RTL930X_VLAN_PORT_TAG_STS_CTRL,
	.trk_mbr_ctr = rtl839x_trk_mbr_ctr,
};

static const struct rtl838x_reg rtl931x_reg = {
	.mask_port_reg_be = rtl839x_mask_port_reg_be,
	.set_port_reg_be = rtl839x_set_port_reg_be,
	.get_port_reg_be = rtl839x_get_port_reg_be,
	.mask_port_reg_le = rtl839x_mask_port_reg_le,
	.set_port_reg_le = rtl839x_set_port_reg_le,
	.get_port_reg_le = rtl839x_get_port_reg_le,
	.stat_port_rst = RTL931X_STAT_PORT_RST,
	.stat_rst = RTL931X_STAT_RST,
	.stat_port_std_mib = 0,  // Not defined
	.l2_ctrl_0 = RTL931X_L2_CTRL,
	.l2_ctrl_1 = RTL931X_L2_AGE_CTRL,
	.l2_port_aging_out = RTL931X_L2_PORT_AGE_CTRL,
	// .smi_poll_ctrl does not exist
	.l2_tbl_flush_ctrl = RTL931X_L2_TBL_FLUSH_CTRL,
	.exec_tbl0_cmd = rtl931x_exec_tbl0_cmd,
	.exec_tbl1_cmd = rtl931x_exec_tbl1_cmd,
	.tbl_access_data_0 = rtl931x_tbl_access_data_0,
	.isr_glb_src = RTL931X_ISR_GLB_SRC,
	.isr_port_link_sts_chg = RTL931X_ISR_PORT_LINK_STS_CHG,
	.imr_port_link_sts_chg = RTL931X_IMR_PORT_LINK_STS_CHG,
	// imr_glb does not exist on RTL931X
	.vlan_tables_read = rtl839x_vlan_tables_read,
	.vlan_set_tagged = rtl839x_vlan_set_tagged,
	.vlan_set_untagged = rtl839x_vlan_set_untagged,
	.vlan_profile_dump = rtl931x_vlan_profile_dump,
	.stp_get = rtl931x_stp_get,
	.stp_set = rtl931x_stp_set,
	.mac_force_mode_ctrl = rtl839x_mac_force_mode_ctrl,
	.mac_port_ctrl = rtl839x_mac_port_ctrl,
	.l2_port_new_salrn = rtl839x_l2_port_new_salrn,
	.l2_port_new_sa_fwd = rtl839x_l2_port_new_sa_fwd,
	.mir_ctrl = rtl839x_mir_ctrl,
	.mir_dpm = rtl839x_mir_dpm,
	.mir_spm = rtl839x_mir_spm,
	.mac_link_sts = RTL839X_MAC_LINK_STS,
	.mac_link_dup_sts = RTL839X_MAC_LINK_DUP_STS,
	.mac_link_spd_sts = rtl839x_mac_link_spd_sts,
	.mac_rx_pause_sts = RTL839X_MAC_RX_PAUSE_STS,
	.mac_tx_pause_sts = RTL839X_MAC_TX_PAUSE_STS,
	.read_l2_entry_using_hash = rtl839x_read_l2_entry_using_hash,
	.read_cam = rtl839x_read_cam,
	.vlan_port_egr_filter = RTL931X_VLAN_PORT_EGR_FLTR(0),
	.vlan_port_igr_filter = RTL931X_VLAN_PORT_IGR_FLTR(0),
//	.vlan_port_pb = does not exist
	.vlan_port_tag_sts_ctrl = RTL931X_VLAN_PORT_TAG_CTRL,
	.trk_mbr_ctr = rtl839x_trk_mbr_ctr,
};

static const struct rtl838x_mib_desc rtl838x_mib[] = {
	MIB_DESC(2, 0xf8, "ifInOctets"),
	MIB_DESC(2, 0xf0, "ifOutOctets"),
	MIB_DESC(1, 0xec, "dot1dTpPortInDiscards"),
	MIB_DESC(1, 0xe8, "ifInUcastPkts"),
	MIB_DESC(1, 0xe4, "ifInMulticastPkts"),
	MIB_DESC(1, 0xe0, "ifInBroadcastPkts"),
	MIB_DESC(1, 0xdc, "ifOutUcastPkts"),
	MIB_DESC(1, 0xd8, "ifOutMulticastPkts"),
	MIB_DESC(1, 0xd4, "ifOutBroadcastPkts"),
	MIB_DESC(1, 0xd0, "ifOutDiscards"),
	MIB_DESC(1, 0xcc, ".3SingleCollisionFrames"),
	MIB_DESC(1, 0xc8, ".3MultipleCollisionFrames"),
	MIB_DESC(1, 0xc4, ".3DeferredTransmissions"),
	MIB_DESC(1, 0xc0, ".3LateCollisions"),
	MIB_DESC(1, 0xbc, ".3ExcessiveCollisions"),
	MIB_DESC(1, 0xb8, ".3SymbolErrors"),
	MIB_DESC(1, 0xb4, ".3ControlInUnknownOpcodes"),
	MIB_DESC(1, 0xb0, ".3InPauseFrames"),
	MIB_DESC(1, 0xac, ".3OutPauseFrames"),
	MIB_DESC(1, 0xa8, "DropEvents"),
	MIB_DESC(1, 0xa4, "tx_BroadcastPkts"),
	MIB_DESC(1, 0xa0, "tx_MulticastPkts"),
	MIB_DESC(1, 0x9c, "CRCAlignErrors"),
	MIB_DESC(1, 0x98, "tx_UndersizePkts"),
	MIB_DESC(1, 0x94, "rx_UndersizePkts"),
	MIB_DESC(1, 0x90, "rx_UndersizedropPkts"),
	MIB_DESC(1, 0x8c, "tx_OversizePkts"),
	MIB_DESC(1, 0x88, "rx_OversizePkts"),
	MIB_DESC(1, 0x84, "Fragments"),
	MIB_DESC(1, 0x80, "Jabbers"),
	MIB_DESC(1, 0x7c, "Collisions"),
	MIB_DESC(1, 0x78, "tx_Pkts64Octets"),
	MIB_DESC(1, 0x74, "rx_Pkts64Octets"),
	MIB_DESC(1, 0x70, "tx_Pkts65to127Octets"),
	MIB_DESC(1, 0x6c, "rx_Pkts65to127Octets"),
	MIB_DESC(1, 0x68, "tx_Pkts128to255Octets"),
	MIB_DESC(1, 0x64, "rx_Pkts128to255Octets"),
	MIB_DESC(1, 0x60, "tx_Pkts256to511Octets"),
	MIB_DESC(1, 0x5c, "rx_Pkts256to511Octets"),
	MIB_DESC(1, 0x58, "tx_Pkts512to1023Octets"),
	MIB_DESC(1, 0x54, "rx_Pkts512to1023Octets"),
	MIB_DESC(1, 0x50, "tx_Pkts1024to1518Octets"),
	MIB_DESC(1, 0x4c, "rx_StatsPkts1024to1518Octets"),
	MIB_DESC(1, 0x48, "tx_Pkts1519toMaxOctets"),
	MIB_DESC(1, 0x44, "rx_Pkts1519toMaxOctets"),
	MIB_DESC(1, 0x40, "rxMacDiscards")
};

static irqreturn_t rtl838x_switch_irq(int irq, void *dev_id)
{
	struct dsa_switch *ds = dev_id;
	u32 status = sw_r32(RTL838X_ISR_GLB_SRC);
	u32 ports = sw_r32(RTL838X_ISR_PORT_LINK_STS_CHG);
	u32 link;
	int i;

	/* Clear status */
	sw_w32(ports, RTL838X_ISR_PORT_LINK_STS_CHG);
	pr_info("RTL8380 Link change: status: %x, ports %x\n", status, ports);

	for (i = 0; i < 28; i++) {
		if (ports & (1 << i)) {
			link = sw_r32(RTL838X_MAC_LINK_STS);
			if (link & BIT(i))
				dsa_port_phylink_mac_change(ds, i, true);
			else
				dsa_port_phylink_mac_change(ds, i, false);
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t rtl839x_switch_irq(int irq, void *dev_id)
{
	struct dsa_switch *ds = dev_id;
	u32 status = sw_r32(RTL839X_ISR_GLB_SRC);
	u64 ports = rtl839x_get_port_reg_le(RTL839X_ISR_PORT_LINK_STS_CHG);
	u64 link;
	int i;

	/* Clear status */
	rtl839x_set_port_reg_le(ports, RTL839X_ISR_PORT_LINK_STS_CHG);
	pr_info("RTL8390 Link change: status: %x, ports %llx\n", status, ports);

	for (i = 0; i < 52; i++) {
		if (ports & BIT_ULL(i)) {
			link = rtl839x_get_port_reg_le(RTL839X_MAC_LINK_STS);
			if (link & BIT_ULL(i))
				dsa_port_phylink_mac_change(ds, i, true);
			else
				dsa_port_phylink_mac_change(ds, i, false);
		}
	}
	return IRQ_HANDLED;
}

void rtl930x_print_matrix(void);

void rtl9300_dump_debug(void)
{
	int i;
	u16 r = RTL930X_STAT_PRVTE_DROP_COUNTER0;

	for (i = 0; i < 10; i ++) {
		pr_info("# %d %08x %08x %08x %08x %08x %08x %08x %08x\n", i * 8,
			sw_r32(r), sw_r32(r + 4), sw_r32(r + 8), sw_r32(r + 12),
			sw_r32(r + 16), sw_r32(r + 20), sw_r32(r + 24), sw_r32(r + 28));
		r += 32;
	}
	pr_info("# %08x %08x %08x %08x %08x\n",
		sw_r32(r), sw_r32(r + 4), sw_r32(r + 8), sw_r32(r + 12), sw_r32(r + 16));
	rtl930x_print_matrix();
	pr_info("RTL930X_L2_PORT_SABLK_CTRL: %08x, RTL930X_L2_PORT_DABLK_CTRL %08x\n",
		sw_r32(RTL930X_L2_PORT_SABLK_CTRL), sw_r32(RTL930X_L2_PORT_DABLK_CTRL)

	);
}

static irqreturn_t rtl930x_switch_irq(int irq, void *dev_id)
{
	struct dsa_switch *ds = dev_id;
	u32 status = sw_r32(RTL930X_ISR_GLB);
	u32 ports = sw_r32(RTL930X_ISR_PORT_LINK_STS_CHG);
	u32 link;
	int i;

	/* Clear status */
	sw_w32(ports, RTL930X_ISR_PORT_LINK_STS_CHG);
	pr_info("RTL9300 Link change: status: %x, ports %x\n", status, ports);

	rtl9300_dump();

	for (i = 0; i < 28; i++) {
		if (ports & BIT(i)) {
			/* Read the register twice because of issues with latency at least
			 * with the external RTL8226 PHY on the XGS1210 */
			link = sw_r32(RTL930X_MAC_LINK_STS);
			link = sw_r32(RTL930X_MAC_LINK_STS);
			if (link & BIT(i))
				dsa_port_phylink_mac_change(ds, i, true);
			else
				dsa_port_phylink_mac_change(ds, i, false);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t rtl931x_switch_irq(int irq, void *dev_id)
{
	struct dsa_switch *ds = dev_id;
	u32 status = sw_r32(RTL931X_ISR_GLB_SRC);
	u64 ports = rtl839x_get_port_reg_le(RTL931X_ISR_PORT_LINK_STS_CHG);
	u64 link;
	int i;

	/* Clear status */
	rtl839x_set_port_reg_le(ports, RTL931X_ISR_PORT_LINK_STS_CHG);
	pr_info("RTL9310 Link change: status: %x, ports %llx\n", status, ports);

	for (i = 0; i < 56; i++) {
		if (ports & BIT_ULL(i)) {
			link = rtl839x_get_port_reg_le(RTL931X_MAC_LINK_STS);
			if (link & BIT_ULL(i))
				dsa_port_phylink_mac_change(ds, i, true);
			else
				dsa_port_phylink_mac_change(ds, i, false);
		}
	}
	return IRQ_HANDLED;
}

struct fdb_update_work {
	struct work_struct work;
	struct net_device *ndev;
	u64 macs[];
};

void rtl838x_fdb_sync(struct work_struct *work)
{
	const struct fdb_update_work *uw =
		container_of(work, struct fdb_update_work, work);
	struct switchdev_notifier_fdb_info info;
	u8 addr[ETH_ALEN];
	int i = 0;
	int action;

	while (uw->macs[i]) {
		action = (uw->macs[i] & BIT_ULL(63)) ? SWITCHDEV_FDB_ADD_TO_BRIDGE
				: SWITCHDEV_FDB_DEL_TO_BRIDGE;
		u64_to_ether_addr(uw->macs[i] & 0xffffffffffffULL, addr);
		info.addr = &addr[0];
		info.vid = 0;
		info.offloaded = 1;
		pr_debug("FDB entry %d: %llx, action %d\n", i, uw->macs[0], action);
		call_switchdev_notifiers(action, uw->ndev, &info.info, NULL);
		i++;
	}
	kfree(work);
}

int rtl8380_sds_power(int mac, int val)
{
	u32 mode = (val == 1) ? 0x4 : 0x9;
	u32 offset = (mac == 24) ? 5 : 0;

	if ((mac != 24) && (mac != 26)) {
		pr_err("%s: not a fibre port: %d\n", __func__, mac);
		return -1;
	}

	sw_w32_mask(0x1f << offset, mode << offset, RTL838X_SDS_MODE_SEL);

	rtl8380_sds_rst(mac);

	return 0;
}

int rtl8390_sds_power(int mac, int val)
{
	u32 offset = (mac == 48) ? 0x0 : 0x100;
	u32 mode = val ? 0 : 1;

	pr_info("In %s: mac %d, set %d\n", __func__, mac, val);

	if ((mac != 48) && (mac != 49)) {
		pr_err("%s: not an SFP port: %d\n", __func__, mac);
		return -1;
	}

	// Set bit 1003. 1000 starts at 7c
	sw_w32_mask(1 << 11, mode << 11, RTL839X_SDS12_13_PWR0 + offset);

	return 0;
}

int rtl9300_sds_power(int mac, int val)
{
	int sds_num;
	u32 mode;

	// TODO: these numbers are hard-coded for the Zyxel XGS1210 12 Switch
	pr_info("SerDes: %s %d\n", __func__, mac);
	switch (mac) {
	case 24:
		sds_num = 6;
		mode = 0x12; // HISGMII
		break;
	case 25:
		sds_num = 7;
		mode = 0x12; // HISGMII
		break;
	case 26:
		sds_num = 8;
		mode = 0x1b; // 10GR/1000BX auto
		break;
	case 27:
		sds_num = 9;
		mode = 0x1b; // 10GR/1000BX auto
		break;
	default:
		return -1;
	}
	if (!val)
		mode = 0x1f; // OFF

	rtl9300_sds_rst(sds_num, mode);

	return 0;
}

static int rtl838x_smi_wait_op(int timeout)
{
	do {
		timeout--;
		udelay(10);
	} while ((sw_r32(RTL838X_SMI_ACCESS_PHY_CTRL_1) & 0x1) && (timeout >= 0));
	if (timeout <= 0)
		return -1;
	return 0;
}

/*
 * Write to a register in a page of the PHY
 */
int rtl838x_write_phy(u32 port, u32 page, u32 reg, u32 val)
{
	u32 v;
	u32 park_page;

	val &= 0xffff;
	if (port > 31 || page > 4095 || reg > 31)
		return -ENOTSUPP;

	mutex_lock(&smi_lock);
	if (rtl838x_smi_wait_op(10000))
		goto timeout;

	sw_w32(1 << port, RTL838X_SMI_ACCESS_PHY_CTRL_0);
	mdelay(10);

	sw_w32_mask(0xffff0000, val << 16, RTL838X_SMI_ACCESS_PHY_CTRL_2);

	park_page = sw_r32(RTL838X_SMI_ACCESS_PHY_CTRL_1) & ((0x1f << 15) | 0x2);
	v = reg << 20 | page << 3 | 0x4;
	sw_w32(v | park_page, RTL838X_SMI_ACCESS_PHY_CTRL_1);
	sw_w32_mask(0, 1, RTL838X_SMI_ACCESS_PHY_CTRL_1);

	if (rtl838x_smi_wait_op(10000))
		goto timeout;

	mutex_unlock(&smi_lock);
	return 0;

timeout:
	mutex_unlock(&smi_lock);
	return -ETIMEDOUT;
}

int rtl839x_write_phy(u32 port, u32 page, u32 reg, u32 val)
{
	u32 v;
	int err = 0;

	val &= 0xffff;
	if (port > 63 || page > 4095 || reg > 31)
		return -ENOTSUPP;

	mutex_lock(&smi_lock);
	/* Clear both port registers */
	sw_w32(0, RTL839X_PHYREG_PORT_CTRL);
	sw_w32(0, RTL839X_PHYREG_PORT_CTRL + 4);
	sw_w32_mask(0, 1 << port, RTL839X_PHYREG_PORT_CTRL + (port % 32) * 4);

	sw_w32_mask(0xffff0000, val << 16, RTL839X_PHYREG_DATA_CTRL);

	v = reg << 5 | page << 10 | ((page == 0x1fff) ? 0x1f : 0) << 23;
	sw_w32(v, RTL839X_PHYREG_ACCESS_CTRL);

	sw_w32(0x1ff, RTL839X_PHYREG_CTRL);

	v |= 1 << 3 | 1; /* Write operation and execute */
	sw_w32(v, RTL839X_PHYREG_ACCESS_CTRL);

	do {
	} while (sw_r32(RTL839X_PHYREG_ACCESS_CTRL) & 0x1);

	if (sw_r32(RTL839X_PHYREG_ACCESS_CTRL) & 0x2)
		err = -EIO;

	mutex_unlock(&smi_lock);
	return err;
}

int rtl930x_write_phy(u32 port, u32 page, u32 reg, u32 val)
{
	u32 v;
	int err = 0;

	pr_info("%s: port %d, page: %d, reg: %x, val: %x\n", __func__, port, page, reg, val);

	if (port > 63 || page > 4095 || reg > 31)
		return -ENOTSUPP;

	val &= 0xffff;
	mutex_lock(&smi_lock);

	sw_w32(BIT(port), RTL930X_SMI_ACCESS_PHY_CTRL_0);
	sw_w32_mask(0xffff << 16, val << 16, RTL930X_SMI_ACCESS_PHY_CTRL_2);
	v = reg << 20 | page << 3 | 0x1f << 15 | BIT(2) | BIT(0);
	sw_w32(v, RTL930X_SMI_ACCESS_PHY_CTRL_1);

	do {
		v = sw_r32(RTL930X_SMI_ACCESS_PHY_CTRL_1);
	} while (v & 0x1);

	if (v & 0x2)
		err = -EIO;

	mutex_unlock(&smi_lock);

	return err;
}

int rtl931x_write_phy(u32 port, u32 page, u32 reg, u32 val)
{
	u32 v;
	int err = 0;

	val &= 0xffff;
	if (port > 63 || page > 4095 || reg > 31)
		return -ENOTSUPP;

	mutex_lock(&smi_lock);
	/* Clear both port registers */
	sw_w32(0, RTL931X_SMI_INDRT_ACCESS_CTRL_2);
	sw_w32(0, RTL931X_SMI_INDRT_ACCESS_CTRL_2 + 4);
	sw_w32_mask(0, BIT(port), RTL931X_SMI_INDRT_ACCESS_CTRL_2+ (port % 32) * 4);

	sw_w32_mask(0xffff0000, val << 16, RTL931X_SMI_INDRT_ACCESS_CTRL_3);

	v = reg << 6 | page << 11 ;
	sw_w32(v, RTL931X_SMI_INDRT_ACCESS_CTRL_0);

	sw_w32(0x1ff, RTL931X_SMI_INDRT_ACCESS_CTRL_1);

	v |= 1 << 3 | 1; /* Write operation and execute */
	sw_w32(v, RTL931X_SMI_INDRT_ACCESS_CTRL_0);

	do {
	} while (sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_0) & 0x1);

	if (sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_0) & 0x2)
		err = -EIO;

	mutex_unlock(&smi_lock);
	return err;
}

/*
 * Reads a register in a page from the PHY
 */
int rtl838x_read_phy(u32 port, u32 page, u32 reg, u32 *val)
{
	u32 v;
	u32 park_page;

	if (port > 31) {
		*val = 0xffff;
		return 0;
	}

	if (page > 4095 || reg > 31)
		return -ENOTSUPP;

	mutex_lock(&smi_lock);

	if (rtl838x_smi_wait_op(10000))
		goto timeout;

	sw_w32_mask(0xffff0000, port << 16, RTL838X_SMI_ACCESS_PHY_CTRL_2);

	park_page = sw_r32(RTL838X_SMI_ACCESS_PHY_CTRL_1) & ((0x1f << 15) | 0x2);
	v = reg << 20 | page << 3;
	sw_w32(v | park_page, RTL838X_SMI_ACCESS_PHY_CTRL_1);
	sw_w32_mask(0, 1, RTL838X_SMI_ACCESS_PHY_CTRL_1);

	if (rtl838x_smi_wait_op(10000))
		goto timeout;

	*val = sw_r32(RTL838X_SMI_ACCESS_PHY_CTRL_2) & 0xffff;

	mutex_unlock(&smi_lock);
	return 0;

timeout:
	mutex_unlock(&smi_lock);
	return -ETIMEDOUT;
}

int rtl839x_read_phy(u32 port, u32 page, u32 reg, u32 *val)
{
	u32 v;
	int err = 0;

	if (port > 63 || page > 4095 || reg > 31)
		return -ENOTSUPP;

	mutex_lock(&smi_lock);

	sw_w32_mask(0xffff0000, port << 16, RTL839X_PHYREG_DATA_CTRL);
	v = reg << 5 | page << 10 | ((page == 0x1fff) ? 0x1f : 0) << 23;
	sw_w32(v, RTL839X_PHYREG_ACCESS_CTRL);

	sw_w32(0x1ff, RTL839X_PHYREG_CTRL);

	v |= 1;
	sw_w32(v, RTL839X_PHYREG_ACCESS_CTRL);

	do {
	} while (sw_r32(RTL839X_PHYREG_ACCESS_CTRL) & 0x1);
	if (sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_0) & 0x2) {
		pr_err("%s: port %d, page: %d, reg: %x FAILED\n", __func__, port, page, reg);
		err = -1;
	}
	*val = sw_r32(RTL839X_PHYREG_DATA_CTRL) & 0xffff;
	
	mutex_unlock(&smi_lock);
	return 0;
}

int rtl930x_read_phy(u32 port, u32 page, u32 reg, u32 *val)
{
	u32 v;
	int err = 0;

//	pr_info("In %s\n", __func__);
	if (port > 63 || page > 4095 || reg > 31)
		return -ENOTSUPP;

	mutex_lock(&smi_lock);

	sw_w32_mask(0xffff << 16, port << 16, RTL930X_SMI_ACCESS_PHY_CTRL_2);
	v = reg << 20 | page << 3 | 0x1f << 15 | 1;
	sw_w32(v, RTL930X_SMI_ACCESS_PHY_CTRL_1);

	do {
		v = sw_r32(RTL930X_SMI_ACCESS_PHY_CTRL_1);
	} while ( v & 0x1);

	if (v & BIT(25)) {
		pr_debug("Error reading phy %d, register %d\n", port, reg);
		err = -EIO;
	}
	*val = (sw_r32(RTL930X_SMI_ACCESS_PHY_CTRL_2) & 0xffff);

	pr_debug("%s: port %d, page: %d, reg: %x, val: %x\n", __func__, port, page, reg, *val);

	mutex_unlock(&smi_lock);

	return err;
}

int rtl931x_read_phy(u32 port, u32 page, u32 reg, u32 *val)
{
	u32 v;

	if (port > 63 || page > 4095 || reg > 31)
		return -ENOTSUPP;

	mutex_lock(&smi_lock);

	sw_w32_mask(0xffff, port, RTL931X_SMI_INDRT_ACCESS_CTRL_3);
	v = reg << 6 | page << 11; // TODO: ACCESS Offset? Park page
	sw_w32(v, RTL931X_SMI_INDRT_ACCESS_CTRL_0);

	sw_w32(0x1ff, RTL931X_SMI_INDRT_ACCESS_CTRL_1);

	v |= 1;
	sw_w32(v, RTL931X_SMI_INDRT_ACCESS_CTRL_0);

	do {
	} while (sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_0) & 0x1);

	*val = (sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_3) & 0xffff0000) >> 16;

	pr_info("%s: port %d, page: %d, reg: %x, val: %x\n", __func__, port, page, reg, *val);

	mutex_unlock(&smi_lock);
	return 0;
}

static int read_phy(u32 port, u32 page, u32 reg, u32 *val)
{
	switch (soc_info.family) {
	case RTL8380_FAMILY_ID:
		return rtl838x_read_phy(port, page, reg, val);
	case RTL8390_FAMILY_ID:
		return rtl839x_read_phy(port, page, reg, val);
	case RTL9300_FAMILY_ID:
		return rtl930x_read_phy(port, page, reg, val);
	case RTL9310_FAMILY_ID:
		return rtl931x_read_phy(port, page, reg, val);
	}
	return -1;
}

static int write_phy(u32 port, u32 page, u32 reg, u32 val)
{
	switch (soc_info.family) {
	case RTL8380_FAMILY_ID:
		return rtl838x_write_phy(port, page, reg, val);
	case RTL8390_FAMILY_ID:
		return rtl839x_write_phy(port, page, reg, val);
	case RTL9300_FAMILY_ID:
		return rtl930x_write_phy(port, page, reg, val);
	case RTL9310_FAMILY_ID:
		return rtl931x_write_phy(port, page, reg, val);
	}
	return -1;
}

/*
 * Write to an mmd register of the PHY
 */
int rtl838x_write_mmd_phy(u32 port, u32 addr, u32 reg, u32 val)
{
	u32 v;

	pr_debug("MMD write: port %d, dev %d, reg %d, val %x\n", port, addr, reg, val);
	val &= 0xffff;
	mutex_lock(&smi_lock);

	if (rtl838x_smi_wait_op(10000))
		goto timeout;

	sw_w32(1 << port, RTL838X_SMI_ACCESS_PHY_CTRL_0);
	mdelay(10);

	sw_w32_mask(0xffff0000, val << 16, RTL838X_SMI_ACCESS_PHY_CTRL_2);

	sw_w32_mask(0x1f << 16, addr << 16, RTL838X_SMI_ACCESS_PHY_CTRL_3);
	sw_w32_mask(0xffff, reg, RTL838X_SMI_ACCESS_PHY_CTRL_3);
	/* mmd-access | write | cmd-start */
	v = 1 << 1 | 1 << 2 | 1;
	sw_w32(v, RTL838X_SMI_ACCESS_PHY_CTRL_1);

	if (rtl838x_smi_wait_op(10000))
		goto timeout;

	mutex_unlock(&smi_lock);
	return 0;

timeout:
	mutex_unlock(&smi_lock);
	return -ETIMEDOUT;
}

/*
 * Read an mmd register of the PHY
 */
int rtl838x_read_mmd_phy(u32 port, u32 addr, u32 reg, u32 *val)
{
	u32 v;

	mutex_lock(&smi_lock);

	if (rtl838x_smi_wait_op(10000))
		goto timeout;

	sw_w32(1 << port, RTL838X_SMI_ACCESS_PHY_CTRL_0);
	mdelay(10);

	sw_w32_mask(0xffff0000, port << 16, RTL838X_SMI_ACCESS_PHY_CTRL_2);

	v = addr << 16 | reg;
	sw_w32(v, RTL838X_SMI_ACCESS_PHY_CTRL_3);

	/* mmd-access | read | cmd-start */
	v = 1 << 1 | 0 << 2 | 1;
	sw_w32(v, RTL838X_SMI_ACCESS_PHY_CTRL_1);

	if (rtl838x_smi_wait_op(10000))
		goto timeout;

	*val = sw_r32(RTL838X_SMI_ACCESS_PHY_CTRL_2) & 0xffff;

	mutex_unlock(&smi_lock);
	return 0;

timeout:
	mutex_unlock(&smi_lock);
	return -ETIMEDOUT;
}

int rtl930x_read_mmd_phy(u32 port, u32 devnum, u32 regnum, u32 *val)
{
	int err = 0;
	u32 v;

	mutex_lock(&smi_lock);

	// Set PHY to access
	sw_w32_mask(0xffff << 16, port << 16, RTL930X_SMI_ACCESS_PHY_CTRL_2);

	// Set MMD device number and register to write to
	sw_w32(devnum << 16 | (regnum & 0xffff), RTL930X_SMI_ACCESS_PHY_CTRL_3);

	v = BIT(1)| BIT(0); // MMD-access | EXEC
	sw_w32(v, RTL930X_SMI_ACCESS_PHY_CTRL_1);

	do {
		v = sw_r32(RTL930X_SMI_ACCESS_PHY_CTRL_1);
	} while ( v & 0x1);
	// There is no error-checking via BIT 25 of v, as it does not seem to be set correctly
	*val = (sw_r32(RTL930X_SMI_ACCESS_PHY_CTRL_2) & 0xffff);
	pr_debug("%s: port %d, regnum: %x, val: %x (err %d)\n", __func__, port, regnum, *val, err);

	mutex_unlock(&smi_lock);

	return err;
}

int rtl930x_write_mmd_phy(u32 port, u32 devnum, u32 regnum, u32 val)
{
	int err = 0;
	u32 v;

	mutex_lock(&smi_lock);

	// Set PHY to access
	sw_w32(BIT(port), RTL930X_SMI_ACCESS_PHY_CTRL_0);

	// Set data to write
	sw_w32_mask(0xffff << 16, val << 16, RTL930X_SMI_ACCESS_PHY_CTRL_2);

	// Set MMD device number and register to write to
	sw_w32(devnum << 16 | (regnum & 0xffff), RTL930X_SMI_ACCESS_PHY_CTRL_3);

	v = BIT(2)| BIT(1)| BIT(0); // WRITE | MMD-access | EXEC
	sw_w32(v, RTL930X_SMI_ACCESS_PHY_CTRL_1);

	do {
		v = sw_r32(RTL930X_SMI_ACCESS_PHY_CTRL_1);
	} while ( v & BIT(0));

// 	if (v & BIT(25)) {  // TODO: Verify whether this gets correctly set
// 		pr_err("Error reading phy %d, register %d\n", port, regnum);
// 		err = -EIO;
// 	}
	pr_info("%s: port %d, regnum: %x, val: %x (err %d)\n", __func__, port, regnum, val, err);
	mutex_unlock(&smi_lock);
	return err;
}

static void rtl8380_get_version(struct rtl838x_switch_priv *priv)
{
	u32 rw_save, info_save;
	u32 info;

	if (priv->id)
		pr_debug("SoC ID: %4x: %s\n", priv->id, soc_info.name);
	else
		pr_err("Unknown chip id (%04x)\n", priv->id);

	rw_save = sw_r32(RTL838X_INT_RW_CTRL);
	sw_w32(rw_save | 0x3, RTL838X_INT_RW_CTRL);

	info_save = sw_r32(RTL838X_CHIP_INFO);
	sw_w32(info_save | 0xA0000000, RTL838X_CHIP_INFO);

	info = sw_r32(RTL838X_CHIP_INFO);
	sw_w32(info_save, RTL838X_CHIP_INFO);
	sw_w32(rw_save, RTL838X_INT_RW_CTRL);

	if ((info & 0xFFFF) == 0x6275) {
		if (((info >> 16) & 0x1F) == 0x1)
			priv->version = RTL8380_VERSION_A;
		else if (((info >> 16) & 0x1F) == 0x2)
			priv->version = RTL8380_VERSION_B;
		else
			priv->version = RTL8380_VERSION_B;
	} else {
		priv->version = '-';
	}
}

static void rtl8390_get_version(struct rtl838x_switch_priv *priv)
{
	u32 info;

	sw_w32_mask(0xf << 28, 0xa << 28, RTL839X_CHIP_INFO);
	info = sw_r32(RTL839X_CHIP_INFO);
	pr_info("Chip-Info: %x\n", info);
	priv->version = RTL8390_VERSION_A;
}

int dsa_phy_read(struct dsa_switch *ds, int phy_addr, int phy_reg)
{
	u32 val;
	u32 offset = 0;
	struct rtl838x_switch_priv *priv = ds->priv;

	/* The inbuilt Serdes need special treatment. TODO: 930x */
	if (phy_addr >= 24 && phy_addr <= 27
		&& priv->ports[24].phy == PHY_RTL838X_SDS) {
		if (phy_addr == 26)
			offset = 0x100;
		val = sw_r32(MAPLE_SDS4_FIB_REG0r + offset + (phy_reg << 2)) & 0xffff;
		return val;
	}

	read_phy(phy_addr, 0, phy_reg, &val);
	return val;
}

int dsa_phy_write(struct dsa_switch *ds, int phy_addr, int phy_reg, u16 val)
{
	u32 offset = 0;
	struct rtl838x_switch_priv *priv = ds->priv;

	if (phy_addr >= 24 && phy_addr <= 27
	     && priv->ports[24].phy == PHY_RTL838X_SDS) {
		if (phy_addr == 26)
			offset = 0x100;
		sw_w32(val, MAPLE_SDS4_FIB_REG0r + offset + (phy_reg << 2));
		return 0;
	}
	return write_phy(phy_addr, 0, phy_reg, val);
}

static void rtl838x_enable_phy_polling(struct rtl838x_switch_priv *priv)
{
	int i;
	u64 v = 0;

	msleep(1000);
	/* Enable all ports with a PHY, including the SFP-ports */
	for (i = 0; i < priv->cpu_port; i++) {
		if (priv->ports[i].phy)
			v |= 1 << i;
	}

	pr_info("%s: %16llx\n", __func__, v);
	priv->r->set_port_reg_le(v, priv->r->smi_poll_ctrl);

	/* PHY update complete, there is no global PHY polling enable bit on the 9300 */
	if (priv->family_id == RTL8390_FAMILY_ID)
		sw_w32_mask(0, 1 << 7, RTL839X_SMI_GLB_CTRL);
	else if (priv->family_id == RTL8380_FAMILY_ID)
		sw_w32_mask(0, 0x8000, RTL838X_SMI_GLB_CTRL);
}

void rtl839x_print_matrix(void)
{
	volatile u64 *ptr = RTL838X_SW_BASE + RTL839X_PORT_ISO_CTRL(0);
	int i;

	for (i = 0; i < 52; i += 4)
		pr_info("> %16llx %16llx %16llx %16llx\n",
			ptr[i + 0], ptr[i + 1], ptr[i + 2], ptr[i + 3]);
	pr_info("CPU_PORT> %16llx\n", ptr[52]);
}

void rtl838x_print_matrix(void)
{
	unsigned volatile int *ptr = RTL838X_SW_BASE + RTL838X_PORT_ISO_CTRL(0);
	int i;

	if (soc_info.family == RTL8390_FAMILY_ID)
		return rtl839x_print_matrix();

	if (soc_info.family == RTL9310_FAMILY_ID)
		return;

	for (i = 0; i < 28; i += 8)
		pr_info("> %8x %8x %8x %8x %8x %8x %8x %8x\n",
			ptr[i + 0], ptr[i + 1], ptr[i + 2], ptr[i + 3], ptr[i + 4], ptr[i + 5],
			ptr[i + 6], ptr[i + 7]);
	pr_info("CPU_PORT> %8x\n", ptr[28]);
}

void rtl930x_print_matrix(void)
{
	int i;
	struct table_reg *r = rtl_table_get(RTL9300_TBL_0, 6);

	for (i = 0; i < 29; i++) {
		rtl_table_read(r, i);
		pr_info("> %08x\n", sw_r32(rtl_table_data(r, 0)));
	}
	rtl_table_release(r);
}

void rtl931x_print_matrix(void)
{
	volatile u64 *ptr = RTL838X_SW_BASE + RTL839X_PORT_ISO_CTRL(0);
	int i;

	for (i = 0; i < 52; i += 4)
		pr_info("> %16llx %16llx %16llx %16llx\n",
			ptr[i + 0], ptr[i + 1], ptr[i + 2], ptr[i + 3]);
	pr_info("CPU_PORT> %16llx\n", ptr[52]);
}

static void rtl838x_init_stats(struct rtl838x_switch_priv *priv)
{
	mutex_lock(&priv->reg_mutex);

	/* Enable statistics module: all counters plus debug.
	 * On RTL839x all counters are enabled by default
	 */
	if (priv->family_id == RTL8380_FAMILY_ID)
		sw_w32_mask(0, 3, RTL838X_STAT_CTRL);

	/* Reset statistics counters */
	sw_w32_mask(0, 1, priv->r->stat_rst);

	mutex_unlock(&priv->reg_mutex);
}



static int rtl930x_setup(struct dsa_switch *ds)
{
	int i;
	struct rtl838x_switch_priv *priv = ds->priv;
	u32 port_bitmap = BIT(priv->cpu_port);

	pr_info("%s called\n", __func__);

	// Enable CSTI STP mode
//	sw_w32(1, RTL930X_ST_CTRL);

	/* Disable MAC polling the PHY so that we can start configuration */
	sw_w32(0, RTL930X_SMI_POLL_CTRL);

	// Disable all ports except CPU port
	for (i = 0; i < ds->num_ports; i++)
		priv->ports[i].enable = false;
	priv->ports[priv->cpu_port].enable = true;

	for (i = 0; i < priv->cpu_port; i++) {
		if (priv->ports[i].phy) {
			rtl930x_traffic_set(i, BIT(priv->cpu_port) | BIT(i));
			port_bitmap |= 1ULL << i;
		}
	}
	rtl930x_traffic_set(priv->cpu_port, port_bitmap);

	rtl930x_print_matrix();
	ds->configure_vlan_while_not_filtering = true;

	rtl838x_enable_phy_polling(priv);

	/* This is for test purposes only */
	

	return 0;
}

static int rtl838x_setup(struct dsa_switch *ds)
{
	int i;
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 port_bitmap = 1ULL << priv->cpu_port;

	pr_info("%s called\n", __func__);

	/* Disable MAC polling the PHY so that we can start configuration */
	priv->r->set_port_reg_le(0ULL, priv->r->smi_poll_ctrl);

	for (i = 0; i < ds->num_ports; i++)
		priv->ports[i].enable = false;
	priv->ports[priv->cpu_port].enable = true;

	if (soc_info.family != RTL9300_FAMILY_ID) {
		/* Isolate ports from each other: traffic only CPU <-> port */
		/* Setting bit j in register RTL838X_PORT_ISO_CTRL(i) allows
		* traffic from source port i to destination port j
		*/
		for (i = 0; i < priv->cpu_port; i++) {
			if (priv->ports[i].phy) {
				priv->r->set_port_reg_be(BIT_ULL(priv->cpu_port) | BIT_ULL(i),
						priv->r->port_iso_ctrl(i));
				port_bitmap |= 1ULL << i;
			}
		}
		priv->r->set_port_reg_be(port_bitmap, priv->r->port_iso_ctrl(priv->cpu_port));
	}

	rtl838x_print_matrix();

	rtl838x_init_stats(priv);
	ds->configure_vlan_while_not_filtering = true;

	/* Enable MAC Polling PHY again */
	rtl838x_enable_phy_polling(priv);
	pr_info("Please wait until PHY is settled\n");
	msleep(1000);
	return 0;
}

static void rtl838x_get_strings(struct dsa_switch *ds,
				int port, u32 stringset, u8 *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < ARRAY_SIZE(rtl838x_mib); i++)
		strncpy(data + i * ETH_GSTRING_LEN, rtl838x_mib[i].name,
			ETH_GSTRING_LEN);
}

static void rtl838x_get_ethtool_stats(struct dsa_switch *ds, int port,
				      uint64_t *data)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	const struct rtl838x_mib_desc *mib;
	int i;
	u64 high;

	for (i = 0; i < ARRAY_SIZE(rtl838x_mib); i++) {
		mib = &rtl838x_mib[i];

		data[i] = sw_r32(priv->r->stat_port_std_mib + (port << 8) + 252 - mib->offset);
		if (mib->size == 2) {
			high = sw_r32(priv->r->stat_port_std_mib + (port << 8 ) + 248 - mib->offset);
			data[i] |= high << 32;
		}
	}
}

static int rtl838x_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return 0;

	return ARRAY_SIZE(rtl838x_mib);
}

static enum dsa_tag_protocol
rtl838x_get_tag_protocol(struct dsa_switch *ds, int port)
{
	/* The switch does not tag the frames, instead internally the header
	 * structure for each packet is tagged accordingly.
	 */
	return DSA_TAG_PROTO_TRAILER;
}

static int rtl838x_get_l2aging(struct rtl838x_switch_priv *priv)
{
	int t = sw_r32(priv->r->l2_ctrl_1);

	t &= priv->family_id == RTL8380_FAMILY_ID ? 0x7fffff : 0x1FFFFF;

	if (priv->family_id == RTL8380_FAMILY_ID)
		t = t * 128 / 625; /* Aging time in seconds. 0: L2 aging disabled */
	else
		t = (t * 3) / 5;

	pr_info("L2 AGING time: %d sec\n", t);
	pr_info("Dynamic aging for ports: %x\n", sw_r32(priv->r->l2_port_aging_out));
	return t;
}

/*
 * Set Switch L2 Aging time, t is time in milliseconds
 * t = 0: aging is disabled
 */
static int rtl838x_set_l2aging(struct dsa_switch *ds, u32 t)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	int t_max = priv->family_id == RTL8380_FAMILY_ID ? 0x7fffff : 0x1FFFFF;

	/* Convert time in mseconds to internal value */
	if (t > 0x10000000) { /* Set to maximum */
		t = t_max;
	} else {
		if (priv->family_id == RTL8380_FAMILY_ID)
			t = ((t * 625) / 1000 + 127) / 128;
		else
			t = (t * 5 + 2) / 3;
	}

// BUG:	sw_w32(t, priv->r->l2_ctrl_1);
	return 0;
}



void rtl838x_fast_age(struct dsa_switch *ds, int port)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	int s = priv->family_id == RTL8390_FAMILY_ID ? 2 : 0;

	pr_info("FAST AGE port %d\n", port);
	mutex_lock(&priv->reg_mutex);
	/* RTL838X_L2_TBL_FLUSH_CTRL register bits, 839x has 1 bit larger
	 * port fields:
	 * 0-4: Replacing port
	 * 5-9: Flushed/replaced port
	 * 10-21: FVID
	 * 22: Entry types: 1: dynamic, 0: also static
	 * 23: Match flush port
	 * 24: Match FVID
	 * 25: Flush (0) or replace (1) L2 entries
	 * 26: Status of action (1: Start, 0: Done)
	 */
	sw_w32(1 << (26 + s) | 1 << (23 + s) | port << (5 + (s / 2)), priv->r->l2_tbl_flush_ctrl);

	do { } while (sw_r32(priv->r->l2_tbl_flush_ctrl) & (1 << (26 + s)));

	mutex_unlock(&priv->reg_mutex);
}

void rtl930x_fast_age(struct dsa_switch *ds, int port)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_info("FAST AGE port %d\n", port);
	mutex_lock(&priv->reg_mutex);
	sw_w32(port << 11, RTL930X_L2_TBL_FLUSH_CTRL + 4);

	sw_w32(BIT(26) | BIT(30), RTL930X_L2_TBL_FLUSH_CTRL);

	do { } while (sw_r32(priv->r->l2_tbl_flush_ctrl) & BIT(30));

	mutex_unlock(&priv->reg_mutex);
}

/*
 * Applies the same hash algorithm as the one used currently by the ASIC
 */
static u32 rtl838x_hash(struct rtl838x_switch_priv *priv, u64 seed)
{
	u32 h1, h2, h3, h;

	if (sw_r32(priv->r->l2_ctrl_0) & 1) {
		h1 = (seed >> 11) & 0x7ff;
		h1 = ((h1 & 0x1f) << 6) | ((h1 >> 5) & 0x3f);

		h2 = (seed >> 33) & 0x7ff;
		h2 = ((h2 & 0x3f) << 5) | ((h2 >> 6) & 0x1f);

		h3 = (seed >> 44) & 0x7ff;
		h3 = ((h3 & 0x7f) << 4) | ((h3 >> 7) & 0xf);

		h = h1 ^ h2 ^ h3 ^ ((seed >> 55) & 0x1ff);
		h ^= ((seed >> 22) & 0x7ff) ^ (seed & 0x7ff);
	} else {
		h = ((seed >> 55) & 0x1ff) ^ ((seed >> 44) & 0x7ff)
			^ ((seed >> 33) & 0x7ff) ^ ((seed >> 22) & 0x7ff)
			^ ((seed >> 11) & 0x7ff) ^ (seed & 0x7ff);
	}

	return h;
}

static u32 rtl839x_hash(struct rtl838x_switch_priv *priv, u64 seed)
{
	u32 h1, h2, h;

	if (sw_r32(priv->r->l2_ctrl_0) & 1) {
		h1 = (u32) (((seed >> 60) & 0x3f) ^ ((seed >> 54) & 0x3f)
				^ ((seed >> 36) & 0x3f) ^ ((seed >> 30) & 0x3f)
				^ ((seed >> 12) & 0x3f) ^ ((seed >> 6) & 0x3f));
		h2 = (u32) (((seed >> 48) & 0x3f) ^ ((seed >> 42) & 0x3f)
				^ ((seed >> 24) & 0x3f) ^ ((seed >> 18) & 0x3f)
				^ (seed & 0x3f));
		h = (h1 << 6) | h2;
	} else {
		h = (seed >> 60)
			^ ((((seed >> 48) & 0x3f) << 6) | ((seed >> 54) & 0x3f))
			^ ((seed >> 36) & 0xfff) ^ ((seed >> 24) & 0xfff)
			^ ((seed >> 12) & 0xfff) ^ (seed & 0xfff);
	}

	return h;
}

/*
 * Calculate both the block 0 and the block 1 hash, and return in
 * lower and higher word of the return value since only 12 bit of
 * the hash are significant
 */
static u32 rtl930x_hash(struct rtl838x_switch_priv *priv, u64 seed)
{
	u32 k0, k1, h1, h2, h;

	k0 = (u32) (((seed >> 55) & 0x1f) ^ ((seed >> 44) & 0x7ff)
		^ ((seed >> 33) & 0x7ff) ^ ((seed >> 22) & 0x7ff)
		^ ((seed >> 11) & 0x7ff) ^ (seed & 0x7ff));

	h1 = (seed >> 11) & 0x7ff;
	h1 = ((h1 & 0x1f) << 6) | ((h1 >> 5) & 0x3f);

	h2 = (seed >> 33) & 0x7ff;
	h2 = ((h2 & 0x3f) << 5)| ((h2 >> 6) & 0x3f);

	k1 = (u32) (((seed << 55) & 0x1f) ^ ((seed >> 44) & 0x7ff) ^ h2
		    ^ ((seed >> 22) & 0x7ff) ^ h1
		    ^ (seed & 0x7ff));

	// Algorithm choice for block 0
	if (sw_r32(RTL930X_L2_CTRL) & BIT(0))
		h = k1;
	else
		h = k0;

	/* Algorithm choice for block 1
	 * Since k0 and k1 are < 2048, adding 2048 will offset the hash into the second
	 * half of hash-space
	 * 2048 is in fact the hash-table size 16384 divided by 4 hashes per bucket
	 * divided by 2 to divide the hash space in 2
	 */
	if (sw_r32(RTL930X_L2_CTRL) & BIT(1))
		h |= (k1 + 2048) << 16;
	else
		h |= (k0 + 2048) << 16;

	return h;
}

static u64 rtl838x_hash_key(struct rtl838x_switch_priv *priv, u64 mac, u32 vid)
{
	switch (priv->family_id) {
	case RTL8380_FAMILY_ID:
		return rtl838x_hash(priv, mac << 12 | vid);
	case RTL8390_FAMILY_ID:
		return rtl839x_hash(priv, mac << 12 | vid);
	case RTL9300_FAMILY_ID:
		return rtl930x_hash(priv, ((u64)vid) << 48 | mac);
	default:
		pr_err("Hash not implemented\n");
	}
	return 0;
}

static void rtl838x_write_cam(int idx, u32 *r)
{
	u32 cmd = 1 << 16 /* Execute cmd */
		| 1 << 15 /* Read */
		| 1 << 13 /* Table type 0b01 */
		| (idx & 0x3f);

	sw_w32(r[0], RTL838X_TBL_ACCESS_L2_DATA(0));
	sw_w32(r[1], RTL838X_TBL_ACCESS_L2_DATA(1));
	sw_w32(r[2], RTL838X_TBL_ACCESS_L2_DATA(2));

	sw_w32(cmd, RTL838X_TBL_ACCESS_L2_CTRL);
	do { }  while (sw_r32(RTL838X_TBL_ACCESS_L2_CTRL) & (1 << 16));
}

static void rtl838x_write_hash(int idx, u32 *r)
{
	u32 cmd = 1 << 16 /* Execute cmd */
		| 0 << 15 /* Write */
		| 0 << 13 /* Table type 0b00 */
		| (idx & 0x1fff);

	sw_w32(0, RTL838X_TBL_ACCESS_L2_DATA(0));
	sw_w32(0, RTL838X_TBL_ACCESS_L2_DATA(1));
	sw_w32(0, RTL838X_TBL_ACCESS_L2_DATA(2));
	sw_w32(cmd, RTL838X_TBL_ACCESS_L2_CTRL);
	do { }  while (sw_r32(RTL838X_TBL_ACCESS_L2_CTRL) & (1 << 16));
}

static void dump_fdb(struct rtl838x_switch_priv *priv)
{
	struct rtl838x_l2_entry e;
	int i;

	mutex_lock(&priv->reg_mutex);

	for (i = 0; i < priv->fib_entries; i++) {
		priv->r->read_l2_entry_using_hash(i >> 2, i & 0x3, &e);

		if (!e.valid) /* Check for invalid entry */
			continue;

		pr_info("-> port %02d: mac %pM, vid: %d, rvid: %d, MC: %d, %d\n",
			e.port, &e.mac[0], e.vid, e.rvid, e.is_ip_mc, e.is_ipv6_mc);
	}

	mutex_unlock(&priv->reg_mutex);
}

int rtl838x_port_get_stp_state(struct rtl838x_switch_priv *priv, int port)
{
	u32 msti = 0;
	u32 port_state[4];
	int index, bit;
	int pos = port;
	int n = priv->port_width << 1;

	/* Ports above CPU port can never be configured */
	if (port > priv->cpu_port)
		return -1;

	/* Only devices where the cpu-port is 28 can be configured for that port */
	if (port == priv->cpu_port && port != 28)
		return -1;

	mutex_lock(&priv->reg_mutex);

	/* For the RTL839x and following, the bits are left-aligned in the 64/128 bit field */
	if (priv->family_id == RTL8390_FAMILY_ID)
		pos += 12;
	if (priv->family_id == RTL9300_FAMILY_ID)
		pos += 3;
	if (priv->family_id == RTL9310_FAMILY_ID)
		pos += 8;

	index = n - (pos >> 4) - 1;
	bit = (pos << 1) % 32;

	priv->r->stp_get(priv, msti, port_state);

	mutex_unlock(&priv->reg_mutex);

	return (port_state[index] >> bit) & 3;
}

static int rtl838x_port_fdb_dump(struct dsa_switch *ds, int port,
				 dsa_fdb_dump_cb_t *cb, void *data)
{
	struct rtl838x_l2_entry e;
	struct rtl838x_switch_priv *priv = ds->priv;
	int i;
	u32 fid;
	u32 pkey;
	u64 mac;

	mutex_lock(&priv->reg_mutex);

	for (i = 0; i < priv->fib_entries; i++) {
		priv->r->read_l2_entry_using_hash(i >> 2, i & 0x3, &e);

		if (!e.valid)
			continue;

		if (e.port == port) {
			fid = (i & 0x3ff) | (e.rvid & ~0x3ff);
			mac = ether_addr_to_u64(&e.mac[0]);
			pkey = rtl838x_hash(priv, mac << 12 | fid);
			fid = (pkey & 0x3ff) | (fid & ~0x3ff);
			pr_info("-> index: %x, port %d mac: %016llx, vlan: %d fid: %d\n",
				i, port, mac, e.vid, fid);
			cb(e.mac, e.vid, e.is_static, data);
		}
	}

	for (i = 0; i < 64; i++) {
		priv->r->read_cam(i, &e);

		if (!e.valid)
			continue;

		if (e.port == port)
			cb(e.mac, e.vid, e.is_static, data);
	}

	mutex_unlock(&priv->reg_mutex);
	return 0;
}

static int rtl838x_port_fdb_del(struct dsa_switch *ds, int port,
			   const unsigned char *addr, u16 vid)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 mac = ether_addr_to_u64(addr);
	u32 key = rtl838x_hash_key(priv, mac, vid);
	struct rtl838x_l2_entry e;
	u32 r[3];
	u64 entry;
	int idx = -1, err = 0, i;

	pr_info("In %s, mac %llx, vid: %d, key: %x08x\n", __func__, mac, vid, key);
	mutex_lock(&priv->reg_mutex);
	for (i = 0; i < 4; i++) {
		entry = priv->r->read_l2_entry_using_hash(key, i, &e);
		if (!e.valid)
			continue;
		if ((entry & 0x0fffffffffffffffULL) == ((mac << 12) | vid)) {
			idx = (key << 2) | i;
			break;
		}
	}

	if (idx >= 0) {
		r[0] = r[1] = r[2] = 0;
		rtl838x_write_hash(idx, r);
		goto out;
	}

	/* Check CAM for spillover from hash buckets */
	for (i = 0; i < 64; i++) {
		entry = priv->r->read_cam(i, &e);
		if ((entry & 0x0fffffffffffffffULL) == ((mac << 12) | vid)) {
			idx = i;
			break;
		}
	}
	if (idx >= 0) {
		r[0] = r[1] = r[2] = 0;
		rtl838x_write_cam(idx, r);
		goto out;
	}
	err = -ENOENT;
out:
	mutex_unlock(&priv->reg_mutex);
	return err;
}

static int rtl838x_port_fdb_add(struct dsa_switch *ds, int port,
				const unsigned char *addr, u16 vid)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 mac = ether_addr_to_u64(addr);
	u32 key = rtl838x_hash_key(priv, mac, vid);
	struct rtl838x_l2_entry e;
	u32 r[3];
	u64 entry;
	int idx = -1, err = 0, i;

	mutex_lock(&priv->reg_mutex);
	for (i = 0; i < 4; i++) {
		entry = priv->r->read_l2_entry_using_hash(key, i, &e);
		if (!e.valid) {
			idx = (key << 2) | i;
			break;
		}
		if ((entry & 0x0fffffffffffffffULL) == ((mac << 12) | vid)) {
			idx = (key << 2) | i;
			break;
		}
	}
	if (idx >= 0) {
		r[0] = 3 << 17 | port << 12; // Aging and  port
		r[0] |= vid;
		r[1] = mac >> 16;
		r[2] = (mac & 0xffff) << 12; /* rvid = 0 */
		rtl838x_write_hash(idx, r);
		goto out;
	}

	/* Hash buckets full, try CAM */
	for (i = 0; i < 64; i++) {
		entry = rtl838x_read_cam(i, &e);
		if (!e.valid) {
			if (idx < 0) /* First empty entry? */
				idx = i;
			break;
		} else if ((entry & 0x0fffffffffffffffULL) == ((mac << 12) | vid)) {
			pr_debug("Found entry in CAM\n");
			idx = i;
			break;
		}
	}
	if (idx >= 0) {
		r[0] = 3 << 17 | port << 12; // Aging
		r[0] |= vid;
		r[1] = mac >> 16;
		r[2] = (mac & 0xffff) << 12; /* rvid = 0 */
		rtl838x_write_cam(idx, r);
		goto out;
	}
	err = -ENOTSUPP;
out:
	mutex_unlock(&priv->reg_mutex);
	return err;
}

void rtl838x_port_stp_state_set(struct dsa_switch *ds, int port, u8 state)
{
	u32 msti = 0;
	u32 port_state[4];
	int index, bit;
	int pos = port;
	struct rtl838x_switch_priv *priv = ds->priv;
	int n = priv->port_width << 1;

	/* Ports above CPU port can never be configured */
	if (port > priv->cpu_port)
		return;

	/* Only devices where the cpu-port is 28 can be configured for that port */
	if (port == priv->cpu_port && port != 28)
		return;

	mutex_lock(&priv->reg_mutex);

	/* For the RTL839x and following, the bits are left-aligned, 838x and 930x
	 * have 64 bit fields, 839x and 931x have 128 bit fields
	 */
	if (priv->family_id == RTL8390_FAMILY_ID)
		pos += 12;
	if (priv->family_id == RTL9300_FAMILY_ID)
		pos += 3;
	if (priv->family_id == RTL9310_FAMILY_ID)
		pos += 8;

	index = n - (pos >> 4) - 1;
	bit = (pos << 1) % 32;

	priv->r->stp_get(priv, msti, port_state);

	pr_info("Current state, port %d: %d\n", port, (port_state[index] >> bit) & 3);
	port_state[index] &= ~(3 << bit);

	switch (state) {
	case BR_STATE_DISABLED: /* 0 */
		port_state[index] |= (0 << bit);
		break;
	case BR_STATE_BLOCKING:  /* 4 */
	case BR_STATE_LISTENING: /* 1 */
		port_state[index] |= (1 << bit);
		break;
	case BR_STATE_LEARNING: /* 2 */
		port_state[index] |= (2 << bit);
		break;
	case BR_STATE_FORWARDING: /* 3*/
		port_state[index] |= (3 << bit);
	default:
		break;
	}

	priv->r->stp_set(priv, msti, port_state);

	mutex_unlock(&priv->reg_mutex);
}

static int rtl838x_port_mirror_add(struct dsa_switch *ds, int port,
				   struct dsa_mall_mirror_tc_entry *mirror,
				   bool ingress)
{
	/* We support 4 mirror groups, one destination port per group */
	int group;
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_info("In %s\n", __func__);

	for (group = 0; group < 4; group++) {
		if (priv->mirror_group_ports[group] == mirror->to_local_port)
			break;
	}
	if (group >= 4) {
		for (group = 0; group < 4; group++) {
			if (priv->mirror_group_ports[group] < 0)
				break;
		}
	}

	if (group >= 4)
		return -ENOSPC;

	pr_debug("Using group %d\n", group);
	mutex_lock(&priv->reg_mutex);

	if (priv->family_id == RTL8380_FAMILY_ID) {
		/* Enable mirroring to port across VLANs (bit 11) */
		sw_w32(1 << 11 | (mirror->to_local_port << 4) | 1, RTL838X_MIR_CTRL(group));
	} else {
		/* Enable mirroring to destination port */
		sw_w32((mirror->to_local_port << 4) | 1, RTL839X_MIR_CTRL(group));
	}

	if (ingress && (priv->r->get_port_reg_be(priv->r->mir_spm(group)) & BIT_ULL(port))) {
		mutex_unlock(&priv->reg_mutex);
		return -EEXIST;
	}
	if ((!ingress) && (priv->r->get_port_reg_be(priv->r->mir_dpm(group)) & BIT_ULL(port))) {
		mutex_unlock(&priv->reg_mutex);
		return -EEXIST;
	}

	if (ingress)
		priv->r->mask_port_reg_be(0, 1ULL << port, priv->r->mir_spm(group));
	else
		priv->r->mask_port_reg_be(0, 1ULL << port, priv->r->mir_dpm(group));

	priv->mirror_group_ports[group] = mirror->to_local_port;
	mutex_unlock(&priv->reg_mutex);
	return 0;
}

static void rtl838x_port_mirror_del(struct dsa_switch *ds, int port,
				    struct dsa_mall_mirror_tc_entry *mirror)
{
	int group = 0;
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_info("In %s\n", __func__);
	for (group = 0; group < 4; group++) {
		if (priv->mirror_group_ports[group] == mirror->to_local_port)
			break;
	}
	if (group >= 4)
		return;

	mutex_lock(&priv->reg_mutex);
	if (mirror->ingress) {
		/* Ingress, clear source port matrix */
		priv->r->mask_port_reg_be(BIT_ULL(port), 0, priv->r->mir_spm(group));
	} else {
		/* Egress, clear destination port matrix */
		priv->r->mask_port_reg_be(BIT_ULL(port), 0, priv->r->mir_dpm(group));
	}

	if (!(sw_r32(priv->r->mir_spm(group)) || sw_r32(priv->r->mir_dpm(group)))) {
		priv->mirror_group_ports[group] = -1;
		sw_w32(0, priv->r->mir_ctrl(group));
	}

	mutex_unlock(&priv->reg_mutex);
}

/* Caller must hold priv->reg_mutex */
int rtl838x_lag_add(struct dsa_switch *ds, int group, int port)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	int i;

	pr_info("%s: Adding port %d to LA-group %d\n", __func__, port, group);
	if (group >= priv->n_lags) {
		pr_err("Link Agrregation group too large.\n");
		return -EINVAL;
	}

	if (port >= priv->cpu_port) {
		pr_err("Invalid port number.\n");
		return -EINVAL;
	}

	for (i = 0; i < priv->n_lags; i++) {
		if (priv->lags_port_members[i] & (1ULL < i))
			break;
	}
	if (i != priv->n_lags) {
		pr_err("%s: Port already member of LAG: %d\n", __func__, i);
		return -ENOSPC;
	}

	priv->r->mask_port_reg_be(0, 1ULL << port, priv->r->trk_mbr_ctr(group));
	priv->lags_port_members[group] |= 1ULL << port;

	pr_info("lags_port_members %d now %016llx\n", group, priv->lags_port_members[group]);
	return 0;
}

/* Caller must hold priv->reg_mutex */
int rtl838x_lag_del(struct dsa_switch *ds, int group, int port)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_info("%s: Removing port %d from LA-group %d\n", __func__, port, group);

	if (group >= priv->n_lags) {
		pr_err("Link Agrregation group too large.\n");
		return -EINVAL;
	}

	if (port >= priv->cpu_port) {
		pr_err("Invalid port number.\n");
		return -EINVAL;
	}


	if (!(priv->lags_port_members[group] & BIT_ULL(port))) {
		pr_err("%s: Port not member of LAG: %d\n", __func__, group
		);
		return -ENOSPC;
	}

	priv->r->mask_port_reg_be(BIT_ULL(port), 0, priv->r->trk_mbr_ctr(group));
	priv->lags_port_members[group] &= ~BIT_ULL(port);

	pr_info("lags_port_members %d now %016llx\n", group, priv->lags_port_members[group]);
	return 0;
}

static int rtl838x_vlan_filtering(struct dsa_switch *ds, int port,
				  bool vlan_filtering)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_info("%s: port %d\n", __func__, port);
	mutex_lock(&priv->reg_mutex);

	if (vlan_filtering) {
		/* Enable ingress and egress filtering
		 * The VLAN_PORT_IGR_FILTER register uses 2 bits for each port to define
		 * the filter action:
		 * 0: Always Forward
		 * 1: Drop packet
		 * 2: Trap packet to CPU port
		 * The Egress filter used 1 bit per state (0: DISABLED, 1: ENABLED)
		 */
		if (port != priv->cpu_port)
			sw_w32_mask(0b10 << ((port % 16) << 1), 0b01 << ((port % 16) << 1),
				    priv->r->vlan_port_igr_filter + ((port >> 5) << 2));
		sw_w32_mask(0, BIT(port % 32), priv->r->vlan_port_egr_filter + ((port >> 4) << 2));
	} else {
		/* Disable ingress and egress filtering */
		if (port != priv->cpu_port)
			sw_w32_mask(0b11 << ((port % 16) << 1), 0,
				    priv->r->vlan_port_igr_filter + ((port >> 5) << 2));
		sw_w32_mask(BIT(port % 32), 0, priv->r->vlan_port_egr_filter + ((port >> 4) << 2));
	}

	/* Do we need to do something to the CPU-Port, too? */
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static int rtl838x_vlan_prepare(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan)
{
	struct rtl838x_vlan_info info;
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_info("%s: port %d\n", __func__, port);

	mutex_lock(&priv->reg_mutex);

	priv->r->vlan_profile_dump(1);
	pr_info("%s: C1\n", __func__);
	priv->r->vlan_tables_read(1, &info);

	pr_info("Tagged ports %llx, untag %llx, prof %x, MC# %d, UC# %d, FID %x\n",
		info.tagged_ports, info.untagged_ports, info.profile_id,
		info.hash_mc_fid, info.hash_uc_fid, info.fid);

	// BUG: Test purposes only
	info.fid = 0;
	info.hash_mc_fid = true;
	info.hash_uc_fid = true;
	info.profile_id = 0;
	info.untagged_ports = 0x1fffffff;
	info.tagged_ports = 0x1fffffff;

	priv->r->vlan_set_untagged(1, info.untagged_ports);
	pr_info("SET: Untagged ports, VLAN %d: %llx\n", 1, info.untagged_ports);

	priv->r->vlan_set_tagged(1, &info);
	pr_info("SET: Tagged ports, VLAN %d: %llx\n", 1, info.tagged_ports);

	mutex_unlock(&priv->reg_mutex);
	return 0;
}

static void rtl838x_vlan_add(struct dsa_switch *ds, int port,
			    const struct switchdev_obj_port_vlan *vlan)
{
	struct rtl838x_vlan_info info;
	struct rtl838x_switch_priv *priv = ds->priv;
	int v;

	pr_info("%s port %d, vid_end %d, vid_end %d, flags %x\n", __func__,
		port, vlan->vid_begin, vlan->vid_end, vlan->flags);

	if (vlan->vid_begin > 4095 || vlan->vid_end > 4095) {
		dev_err(priv->dev, "VLAN out of range: %d - %d",
			vlan->vid_begin, vlan->vid_end);
		return;
	}

	mutex_lock(&priv->reg_mutex);

	if (vlan->flags & BRIDGE_VLAN_INFO_PVID) {
		for (v = vlan->vid_begin; v <= vlan->vid_end; v++) {
			if (!v)
				continue;
			/* Set both inner and outer PVID of the port */
			sw_w32((v << 16) | v << 2, priv->r->vlan_port_pb + (port << 2));
			priv->ports[port].pvid = vlan->vid_end;
		}
	}

	for (v = vlan->vid_begin; v <= vlan->vid_end; v++) {
		if (!v)
			continue;
		
		/* Get port memberships of this vlan */
		priv->r->vlan_tables_read(v, &info);

		/* new VLAN? */
		if (!info.tagged_ports) {
			info.fid = 0;
			info.hash_mc_fid = false;
			info.hash_uc_fid = false;
			info.profile_id = 0;
		}

		/* sanitize untagged_ports - must be a subset */
		if (info.untagged_ports & ~info.tagged_ports)
			info.untagged_ports = 0;

		info.tagged_ports |= BIT_ULL(port);
		if (vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED)
			info.untagged_ports |= BIT_ULL(port);

		priv->r->vlan_set_untagged(v, info.untagged_ports);
		pr_info("Untagged ports, VLAN %d: %llx\n", v, info.untagged_ports);

		priv->r->vlan_set_tagged(v, &info);
		pr_info("Tagged ports, VLAN %d: %llx\n", v, info.tagged_ports);
	}

	mutex_unlock(&priv->reg_mutex);
}

static int rtl838x_vlan_del(struct dsa_switch *ds, int port,
			    const struct switchdev_obj_port_vlan *vlan)
{
	struct rtl838x_vlan_info info;
	struct rtl838x_switch_priv *priv = ds->priv;
	int v;
	u16 pvid;

	pr_info("%s: port %d, vid_end %d, vid_end %d, flags %x\n", __func__,
		port, vlan->vid_begin, vlan->vid_end, vlan->flags);

	if (vlan->vid_begin > 4095 || vlan->vid_end > 4095) {
		dev_err(priv->dev, "VLAN out of range: %d - %d",
			vlan->vid_begin, vlan->vid_end);
		return -ENOTSUPP;
	}

	mutex_lock(&priv->reg_mutex);
	pvid = priv->ports[port].pvid;

	for (v = vlan->vid_begin; v <= vlan->vid_end; v++) {
		if (!v)
			continue;
		/* Reset to default if removing the current PVID */
		if (v == pvid)
			sw_w32(0, priv->r->vlan_port_pb + (port << 2));

		/* Get port memberships of this vlan */
		priv->r->vlan_tables_read(v, &info);

		/* remove port from both tables */
		info.untagged_ports &= (~BIT_ULL(port));

		/* always leave vid 1 */
		if (v != 1)
			info.tagged_ports &= (~BIT_ULL(port));

		priv->r->vlan_set_untagged(v, info.untagged_ports);
		pr_info("Untagged ports, VLAN %d: %llx\n", v, info.untagged_ports);

		priv->r->vlan_set_tagged(v, &info);
		pr_info("Tagged ports, VLAN %d: %llx\n", v, info.tagged_ports);
	}

	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static void rtl838x_port_bridge_leave(struct dsa_switch *ds, int port,
					struct net_device *bridge)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 port_bitmap = 1ULL << priv->cpu_port, v;
	int i;

	pr_info("%s %x: %d", __func__, (u32)priv, port);
	mutex_lock(&priv->reg_mutex);
	for (i = 0; i < ds->num_ports; i++) {
		/* Remove this port from the port matrix of the other ports
		 * in the same bridge. If the port is disabled, port matrix
		 * is kept and not being setup until the port becomes enabled.
		 * And the other port's port matrix cannot be broken when the
		 * other port is still a VLAN-aware port.
		 */
		if (dsa_is_user_port(ds, i) && i != port) {
			if (dsa_to_port(ds, i)->bridge_dev != bridge)
				continue;
			if (priv->ports[i].enable)
				priv->r->traffic_disable(i, port);

			priv->ports[i].pm |= 1ULL << port;
			port_bitmap &= ~BIT_ULL(i);
		}
	}

	/* Add all other ports to this port matrix. */
	if (priv->ports[port].enable) {
		v = priv->r->traffic_get(port);
		v |= port_bitmap;
		priv->r->traffic_set(port, v);
	}
	priv->ports[port].pm &= ~port_bitmap;

	mutex_unlock(&priv->reg_mutex);
}

static int rtl838x_port_bridge_join(struct dsa_switch *ds, int port,
					struct net_device *bridge)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 port_bitmap = 1ULL << priv->cpu_port, v;
	int i;

	pr_info("%s %x: %d %llx", __func__, (u32)priv, port, port_bitmap);
	mutex_lock(&priv->reg_mutex);
	for (i = 0; i < ds->num_ports; i++) {
		/* Add this port to the port matrix of the other ports in the
		 * same bridge. If the port is disabled, port matrix is kept
		 * and not being setup until the port becomes enabled.
		 */
		if (dsa_is_user_port(ds, i) && i != port) {
			if (dsa_to_port(ds, i)->bridge_dev != bridge)
				continue;
			if (priv->ports[i].enable)
				priv->r->traffic_enable(i, port);

			priv->ports[i].pm |= 1ULL << port;
			port_bitmap |= 1ULL << i;
		}
	}

	/* Add all other ports to this port matrix. */
	if (priv->ports[port].enable) {
		priv->r->traffic_enable(priv->cpu_port, port);
		v = priv->r->traffic_get(port);
		v |= port_bitmap;
		priv->r->traffic_set(port, v);
	}
	priv->ports[port].pm |= port_bitmap;
	mutex_unlock(&priv->reg_mutex);

	rtl930x_print_matrix();
	return 0;
}

static int rtl838x_port_enable(struct dsa_switch *ds, int port,
				struct phy_device *phydev)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 v;

	pr_info("%s: %x %d", __func__, (u32) priv, port);
	priv->ports[port].enable = true;

	/* enable inner tagging on egress, do not keep any tags */
	if (priv->family_id == RTL9310_FAMILY_ID)
		sw_w32(BIT(4), priv->r->vlan_port_tag_sts_ctrl + (port << 2));
	else
		sw_w32(1, priv->r->vlan_port_tag_sts_ctrl + (port << 2));

	if (dsa_is_cpu_port(ds, port))
		return 0;

	/* add port to switch mask of CPU_PORT */
	priv->r->traffic_enable(priv->cpu_port, port);

	/* add all other ports in the same bridge to switch mask of port */
	v = priv->r->traffic_get(port);
	v |= priv->ports[port].pm;
	priv->r->traffic_set(port, v);

	sw_w32_mask(0, BIT(port), RTL930X_L2_PORT_SABLK_CTRL);
	sw_w32_mask(0, BIT(port), RTL930X_L2_PORT_DABLK_CTRL);

	rtl930x_print_matrix();
	
	return 0;
}

static void rtl838x_port_disable(struct dsa_switch *ds, int port)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 v;

	pr_info("%s %x: %d", __func__, (u32)priv, port);
	/* you can only disable user ports */
	if (!dsa_is_user_port(ds, port))
		return;

	// BUG:
	/* remove port from switch mask of CPU_PORT */
	priv->r->traffic_disable(priv->cpu_port, port);

	/* remove all other ports in the same bridge from switch mask of port */
	v = priv->r->traffic_get(port);
	v &= ~priv->ports[port].pm;
	priv->r->traffic_set(port, v);

	priv->ports[port].enable = false;
}

static int rtl838x_get_mac_eee(struct dsa_switch *ds, int port,
			       struct ethtool_eee *e)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_info("%s: port %d", __func__, port);
	e->supported = SUPPORTED_100baseT_Full | SUPPORTED_1000baseT_Full;
	if (sw_r32(priv->r->mac_force_mode_ctrl(port)) & (1 << 9))
		e->advertised |= ADVERTISED_100baseT_Full;

	if (sw_r32(priv->r->mac_force_mode_ctrl(port)) & (1 << 10))
		e->advertised |= ADVERTISED_1000baseT_Full;

	e->eee_enabled = priv->ports[port].eee_enabled;
	pr_info("enabled: %d, active %x\n", e->eee_enabled, e->advertised);

	if (sw_r32(RTL838X_MAC_EEE_ABLTY) & (1 << port)) {
		e->lp_advertised = ADVERTISED_100baseT_Full;
		e->lp_advertised |= ADVERTISED_1000baseT_Full;
	}

	e->eee_active = !!(e->advertised & e->lp_advertised);
	pr_info("active: %d, lp %x\n", e->eee_active, e->lp_advertised);

	return 0;
}

static int rtl838x_set_mac_eee(struct dsa_switch *ds, int port,
			       struct ethtool_eee *e)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_info("%s: port %d", __func__, port);
	if (e->eee_enabled) {
		pr_info("Globally enabling EEE\n");
		sw_w32_mask(0x4, 0, RTL838X_SMI_GLB_CTRL);
	}
	if (e->eee_enabled) {
		pr_info("Enabling EEE for MAC %d\n", port);
		sw_w32_mask(0, 3 << 9, priv->r->mac_force_mode_ctrl(port));
		sw_w32_mask(0, 1 << port, RTL838X_EEE_PORT_TX_EN);
		sw_w32_mask(0, 1 << port, RTL838X_EEE_PORT_RX_EN);
		priv->ports[port].eee_enabled = true;
		e->eee_enabled = true;
	} else {
		pr_info("Disabling EEE for MAC %d\n", port);
		sw_w32_mask(3 << 9, 0, priv->r->mac_force_mode_ctrl(port));
		sw_w32_mask(1 << port, 0, RTL838X_EEE_PORT_TX_EN);
		sw_w32_mask(1 << port, 0, RTL838X_EEE_PORT_RX_EN);
		priv->ports[port].eee_enabled = false;
		e->eee_enabled = false;
	}
	return 0;
}

static void rtl838x_phylink_mac_config(struct dsa_switch *ds, int port,
					unsigned int mode,
					const struct phylink_link_state *state)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u32 reg;
	int speed_bit = priv->family_id == RTL8380_FAMILY_ID ? 4 : 3;

	pr_info("%s port %d, mode %x\n", __func__, port, mode);

	// BUG:
	if (priv->family_id >= RTL9300_FAMILY_ID)
		return;

	if (port == priv->cpu_port) {
		/* Set Speed, duplex, flow control
		 * FORCE_EN | LINK_EN | NWAY_EN | DUP_SEL
		 * | SPD_SEL = 0b10 | FORCE_FC_EN | PHY_MASTER_SLV_MANUAL_EN
		 * | MEDIA_SEL
		 */
		if (priv->family_id == RTL8380_FAMILY_ID) {
			sw_w32(0x6192F, priv->r->mac_force_mode_ctrl(priv->cpu_port));
			/* allow CRC errors on CPU-port */
			sw_w32_mask(0, 0x8, RTL838X_MAC_PORT_CTRL(priv->cpu_port));
		} else {
			sw_w32_mask(0, 3, priv->r->mac_force_mode_ctrl(priv->cpu_port));
		}
		return;
	}

	reg = sw_r32(priv->r->mac_force_mode_ctrl(port));
	/* Auto-Negotiation does not work for MAC in RTL8390 */
	if (priv->family_id == RTL8380_FAMILY_ID) {
		if (mode == MLO_AN_PHY) {
			pr_info("PHY autonegotiates\n");
			reg |= 1 << 2;
			sw_w32(reg, priv->r->mac_force_mode_ctrl(port));
			return;
		}
	}

	if (mode != MLO_AN_FIXED)
		pr_info("Fixed state.\n");

	if (priv->family_id == RTL8380_FAMILY_ID) {
		/* Clear id_mode_dis bit, and the existing port mode, let
		 * RGMII_MODE_EN bet set by mac_link_{up,down}
		 */
		reg &= ~(RX_PAUSE_EN | TX_PAUSE_EN);

		if (state->pause & MLO_PAUSE_TXRX_MASK) {
			if (state->pause & MLO_PAUSE_TX)
				reg |= TX_PAUSE_EN;
			reg |= RX_PAUSE_EN;
		}
	}

	reg &= ~(3 << speed_bit);
	switch (state->speed) {
	case SPEED_1000:
		reg |= 2 << speed_bit;
		break;
	case SPEED_100:
		reg |= 1 << speed_bit;
		break;
	}

	if (priv->family_id == RTL8380_FAMILY_ID) {
		reg &= ~(DUPLEX_FULL | FORCE_LINK_EN);
		if (state->link)
			reg |= FORCE_LINK_EN;
		if (state->duplex == DUPLEX_FULL)
			reg |= DUPLX_MODE;
	}

	// Disable AN
	if (priv->family_id == RTL8380_FAMILY_ID)
		reg &= ~(1 << 2);
	sw_w32(reg, priv->r->mac_force_mode_ctrl(port));
}

static void rtl838x_phylink_mac_link_down(struct dsa_switch *ds, int port,
				     unsigned int mode,
				     phy_interface_t interface)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	/* Stop TX/RX to port */
	sw_w32_mask(0x3, 0, priv->r->mac_port_ctrl(port));
}

static void rtl838x_phylink_mac_link_up(struct dsa_switch *ds, int port,
				   unsigned int mode,
				   phy_interface_t interface,
				   struct phy_device *phydev)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	/* Restart TX/RX to port */
	sw_w32_mask(0, 0x3, priv->r->mac_port_ctrl(port));
}

static void rtl838x_phylink_validate(struct dsa_switch *ds, int port,
				     unsigned long *supported,
				     struct phylink_link_state *state)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	pr_info("In %s port %d", __func__, port);

	if (!phy_interface_mode_is_rgmii(state->interface) &&
	    state->interface != PHY_INTERFACE_MODE_1000BASEX &&
	    state->interface != PHY_INTERFACE_MODE_MII &&
	    state->interface != PHY_INTERFACE_MODE_REVMII &&
	    state->interface != PHY_INTERFACE_MODE_GMII &&
	    state->interface != PHY_INTERFACE_MODE_QSGMII &&
	    state->interface != PHY_INTERFACE_MODE_INTERNAL &&
	    state->interface != PHY_INTERFACE_MODE_SGMII) {
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		dev_err(ds->dev,
			"Unsupported interface: %d for port %d\n",
			state->interface, port);
		return;
	}

	/* Allow all the expected bits */
	phylink_set(mask, Autoneg);
	phylink_set_port_modes(mask);
	phylink_set(mask, Pause);
	phylink_set(mask, Asym_Pause);

	/* With the exclusion of MII and Reverse MII, we support Gigabit,
	 * including Half duplex
	 */
	if (state->interface != PHY_INTERFACE_MODE_MII &&
	    state->interface != PHY_INTERFACE_MODE_REVMII) {
		phylink_set(mask, 1000baseT_Full);
		phylink_set(mask, 1000baseT_Half);
	}

	/* On both the 8380 and 8382, ports 24-27 are SFP ports */
	if (port >= 24 && port <= 27 && priv->family_id == RTL8380_FAMILY_ID)
		phylink_set(mask, 1000baseX_Full);

	phylink_set(mask, 10baseT_Half);
	phylink_set(mask, 10baseT_Full);
	phylink_set(mask, 100baseT_Half);
	phylink_set(mask, 100baseT_Full);

	bitmap_and(supported, supported, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
}

static int rtl838x_phylink_mac_link_state(struct dsa_switch *ds, int port,
					  struct phylink_link_state *state)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 speed;
	u64 link;

	if (port < 0 || port > priv->cpu_port)
		return -EINVAL;

	state->link = 0;
	link = priv->r->get_port_reg_le(priv->r->mac_link_sts);
	link = priv->r->get_port_reg_le(priv->r->mac_link_sts);
	if (link & BIT_ULL(port))
		state->link = 1;
	pr_info("%s: link state: %llx\n", __func__, link & BIT_ULL(port));
	state->duplex = 0;
	if (priv->r->get_port_reg_le(priv->r->mac_link_dup_sts) & BIT_ULL(port))
		state->duplex = 1;

	speed = priv->r->get_port_reg_le(priv->r->mac_link_spd_sts(port));
	speed >>= (port % 16) << 1;
	switch (speed & 0x3) {
	case 0:
		state->speed = SPEED_10;
		break;
	case 1:
		state->speed = SPEED_100;
		break;
	case 2:
		state->speed = SPEED_1000;
		break;
	case 3:
		if (port == 24 || port == 26) /* Internal serdes */
			state->speed = SPEED_2500;
		else
			state->speed = SPEED_100; /* Is in fact 500Mbit */
	}

	state->pause &= (MLO_PAUSE_RX | MLO_PAUSE_TX);
	if (priv->r->get_port_reg_le(priv->r->mac_rx_pause_sts) & BIT_ULL(port))
		state->pause |= MLO_PAUSE_RX;
	if (priv->r->get_port_reg_le(priv->r->mac_tx_pause_sts) & BIT_ULL(port))
		state->pause |= MLO_PAUSE_TX;
	return 1;
}

static void rtl838x_storm_enable(struct rtl838x_switch_priv *priv, int port, bool enable)
{
	// Enable Storm control for that port for UC, MC, and BC
	if (enable)
		sw_w32(0x7, RTL838X_STORM_CTRL_LB_CTRL(port));
	else
		sw_w32(0x0, RTL838X_STORM_CTRL_LB_CTRL(port));
}

u32 rtl838x_get_egress_rate(struct rtl838x_switch_priv *priv, int port)
{
	u32 rate;

	if (port > priv->cpu_port)
		return 0;
	rate = sw_r32(RTL838X_SCHED_P_EGR_RATE_CTRL(port)) & 0x3fff;
	return rate;
}

/* Sets the rate limit, 10MBit/s is equal to a rate value of 625 */
void rtl838x_set_egress_rate(struct rtl838x_switch_priv *priv, int port, u32 rate)
{
	if (port > priv->cpu_port)
		return;
	sw_w32(rate, RTL838X_SCHED_P_EGR_RATE_CTRL(port));
}

/* Set the rate limit for a particular queue in Bits/s
 * units of the rate is 16Kbps
 */
void rtl838x_egress_rate_queue_limit(struct rtl838x_switch_priv *priv, int port,
					    int queue, u32 rate)
{
	if (port > priv->cpu_port)
		return;
	if (queue > 7)
		return;
	sw_w32(rate, RTL838X_SCHED_Q_EGR_RATE_CTRL(port, queue));
}

static void rtl838x_rate_control_init(struct rtl838x_switch_priv *priv)
{
	int i;

	pr_info("Enabling Storm control\n");
	// TICK_PERIOD_PPS
	if (priv->id == 0x8380)
		sw_w32_mask(0x3ff << 20, 434 << 20, RTL838X_SCHED_LB_TICK_TKN_CTRL_0);

	// Set burst rate
	sw_w32(0x00008000, RTL838X_STORM_CTRL_BURST_0); // UC
	sw_w32(0x80008000, RTL838X_STORM_CTRL_BURST_1); // MC and BC

	// Set burst Packets per Second to 32
	sw_w32(0x00000020, RTL838X_STORM_CTRL_BURST_PPS_0); // UC
	sw_w32(0x00200020, RTL838X_STORM_CTRL_BURST_PPS_1); // MC and BC

	// Include IFG in storm control, rate based on bytes/s (0 = packets)
	sw_w32_mask(0, 1 << 6 | 1 << 5, RTL838X_STORM_CTRL);
	// Bandwidth control includes preamble and IFG (10 Bytes)
	sw_w32_mask(0, 1, RTL838X_SCHED_CTRL);

	// On SoCs except RTL8382M, set burst size of port egress
	if (priv->id != 0x8382)
		sw_w32_mask(0xffff, 0x800, RTL838X_SCHED_LB_THR);

	/* Enable storm control on all ports with a PHY and limit rates,
	 * for UC and MC for both known and unknown addresses */
	for (i = 0; i < priv->cpu_port; i++) {
		if (priv->ports[i].phy) {
			sw_w32((1 << 18) | 0x8000, RTL838X_STORM_CTRL_PORT_UC(i));
			sw_w32((1 << 18) | 0x8000, RTL838X_STORM_CTRL_PORT_MC(i));
			sw_w32(0x8000, RTL838X_STORM_CTRL_PORT_BC(i));
			rtl838x_storm_enable(priv, i, true);
		}
	}

	// Attack prevention, enable all attack prevention measures
	//sw_w32(0x1ffff, RTL838X_ATK_PRVNT_CTRL);
	/* Attack prevention, drop (bit = 0) problematic packets on all ports.
	 * Setting bit = 1 means: trap to CPU
	 */
	//sw_w32(0, RTL838X_ATK_PRVNT_ACT);
	// Enable attack prevention on all ports
	//sw_w32(0x0fffffff, RTL838X_ATK_PRVNT_PORT_EN);
}

/* Sets the rate limit, 10MBit/s is equal to a rate value of 625 */
u32 rtl839x_get_egress_rate(struct rtl838x_switch_priv *priv, int port)
{
	u32 rate;

	pr_debug("%s: Getting egress rate on port %d to %d\n", __func__, port, rate);
	if (port >= priv->cpu_port)
		return 0;

	mutex_lock(&priv->reg_mutex);

	rtl839x_read_scheduling_table(port);

	rate = sw_r32(RTL839X_TBL_ACCESS_DATA_2(7));
	rate <<= 12;
	rate |= sw_r32(RTL839X_TBL_ACCESS_DATA_2(8)) >> 20;

	mutex_unlock(&priv->reg_mutex);

	return rate;
}

/* Sets the rate limit, 10MBit/s is equal to a rate value of 625 */
void rtl839x_set_egress_rate(struct rtl838x_switch_priv *priv, int port, u32 rate)
{
	u32 cmd;

	pr_debug("%s: Setting egress rate on port %d to %d\n", __func__, port, rate);
	if (port >= priv->cpu_port)
		return;

	mutex_lock(&priv->reg_mutex);

	rtl839x_read_scheduling_table(port);

	sw_w32_mask(0xff, (rate >> 12) & 0xff, RTL839X_TBL_ACCESS_DATA_2(7));
	sw_w32_mask(0xfff << 20, rate << 20, RTL839X_TBL_ACCESS_DATA_2(8));

	cmd = 1 << 9 /* Execute cmd */
		| 1 << 8 /* Read */
		| 0 << 6 /* Table type 0b00 */
		| (port & 0x3f);
	rtl839x_exec_tbl2_cmd(cmd);

	mutex_unlock(&priv->reg_mutex);
}

/* Set the rate limit for a particular queue in Bits/s
 * units of the rate is 16Kbps
 */
void rtl839x_egress_rate_queue_limit(struct rtl838x_switch_priv *priv, int port,
					int queue, u32 rate)
{
	u32 cmd;
	int lsb = 128 + queue * 20;
	int low_byte = 8 - (lsb >> 5);
	int start_bit = lsb - (low_byte << 5);
	u32 high_mask = 0xfffff	>> (32 - start_bit);

	pr_debug("%s: Setting egress rate on port %d, queue %d to %d\n",
		__func__, port, queue, rate);
	if (port >= priv->cpu_port)
		return;
	if (queue > 7)
		return;

	mutex_lock(&priv->reg_mutex);

	rtl839x_read_scheduling_table(port);

	sw_w32_mask(0xfffff << start_bit, (rate & 0xfffff) << start_bit,
		    RTL839X_TBL_ACCESS_DATA_2(low_byte));
	if (high_mask)
		sw_w32_mask(high_mask, (rate & 0xfffff) >> (32- start_bit),
			    RTL839X_TBL_ACCESS_DATA_2(low_byte - 1));

	cmd = 1 << 9 /* Execute cmd */
		| 1 << 8 /* Read */
		| 0 << 6 /* Table type 0b00 */
		| (port & 0x3f);
	rtl839x_exec_tbl2_cmd(cmd);

	mutex_unlock(&priv->reg_mutex);
}

static void rtl839x_rate_control_init(struct rtl838x_switch_priv *priv)
{
	int p, q;

	pr_info("%s: enabling rate control\n", __func__);
	/* Tick length and token size settings for SoC with 250MHz,
	 * RTL8350 family would use 50MHz
	 */
	// Set the special tick period
	sw_w32(976563, RTL839X_STORM_CTRL_SPCL_LB_TICK_TKN_CTRL);
	// Ingress tick period and token length 10G
	sw_w32(18 << 11 | 151, RTL839X_IGR_BWCTRL_LB_TICK_TKN_CTRL_0);
	// Ingress tick period and token length 1G
	sw_w32(245 << 11 | 129, RTL839X_IGR_BWCTRL_LB_TICK_TKN_CTRL_1);
	// Egress tick period 10G, bytes/token 10G and tick period 1G, bytes/token 1G
	sw_w32(18 << 24 | 151 << 16 | 185 << 8 | 97, RTL839X_SCHED_LB_TICK_TKN_CTRL);
	// Set the tick period of the CPU and the Token Len
	sw_w32(3815 << 8 | 1, RTL839X_SCHED_LB_TICK_TKN_PPS_CTRL);

	// Set the Weighted Fair Queueing burst size
	sw_w32_mask(0xffff, 4500, RTL839X_SCHED_LB_THR);

	// Storm-rate calculation is based on bytes/sec (bit 5), include IFG (bit 6)
	sw_w32_mask(0, 1 << 5 | 1 << 6, RTL839X_STORM_CTRL);

	/* Based on the rate control mode being bytes/s
	 * set tick period and token length for 10G
	 */
	sw_w32(18 << 10 | 151, RTL839X_STORM_CTRL_LB_TICK_TKN_CTRL_0);
	/* and for 1G ports */
	sw_w32(246 << 10 | 129, RTL839X_STORM_CTRL_LB_TICK_TKN_CTRL_1);

	/* Set default burst rates on all ports (the same for 1G / 10G) with a PHY
	 * for UC, MC and BC
	 * For 1G port, the minimum burst rate is 1700, maximum 65535,
	 * For 10G ports it is 2650 and 1048575 respectively */
	for (p = 0; p < priv->cpu_port; p++) {
		if (priv->ports[p].phy && !priv->ports[p].is10G) {
			sw_w32_mask(0xffff, 0x8000, RTL839X_STORM_CTRL_PORT_UC_1(p));
			sw_w32_mask(0xffff, 0x8000, RTL839X_STORM_CTRL_PORT_MC_1(p));
			sw_w32_mask(0xffff, 0x8000, RTL839X_STORM_CTRL_PORT_BC_1(p));
		}
	}

	/* Setup ingress/egress per-port rate control */
	for (p = 0; p < priv->cpu_port; p++) {
		if (!priv->ports[p].phy)
			continue;

		if (priv->ports[p].is10G)
			rtl839x_set_egress_rate(priv, p, 625000); // 10GB/s
		else
			rtl839x_set_egress_rate(priv, p, 62500);  // 1GB/s

		// Setup queues: all RTL83XX SoCs have 8 queues, maximum rate
		for (q = 0; q < 8; q++)
			rtl839x_egress_rate_queue_limit(priv, p, q, 0xfffff);

		if (priv->ports[p].is10G) {
			// Set high threshold to maximum
			sw_w32_mask(0xffff, 0xffff, RTL839X_IGR_BWCTRL_PORT_CTRL_10G_0(p));
		} else {
			// Set high threshold to maximum
			sw_w32_mask(0xffff, 0xffff, RTL839X_IGR_BWCTRL_PORT_CTRL_1(p));
		}
	}

	// Set global ingress low watermark rate
	sw_w32(65532, RTL839X_IGR_BWCTRL_CTRL_LB_THR);
}

static int rtl838x_mdio_probe(struct rtl838x_switch_priv *priv)
{
	struct device *dev = priv->dev;
	struct device_node *dn, *mii_np = dev->of_node;
	struct mii_bus *bus;
	int ret;
	u32 pn;

	pr_info("In %s\n", __func__);
	mii_np = of_find_compatible_node(NULL, NULL, "realtek,rtl838x-mdio");
	if (mii_np) {
		pr_info("Found compatible MDIO node!\n");
	} else {
		dev_err(priv->dev, "no %s child node found", "mdio-bus");
		return -ENODEV;
	}

	priv->mii_bus = of_mdio_find_bus(mii_np);
	if (!priv->mii_bus) {
		pr_info("Deferring probe of mdio bus\n");
		return -EPROBE_DEFER;
	}
	if (!of_device_is_available(mii_np))
		ret = -ENODEV;

	bus = devm_mdiobus_alloc(priv->ds->dev);
	if (!bus)
		return -ENOMEM;

	bus->name = "rtl838x slave mii";
	bus->read = priv->mii_bus->read;
	bus->write = priv->mii_bus->write;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-%d", bus->name, dev->id);
	bus->parent = dev;
	priv->ds->slave_mii_bus = bus;
	priv->ds->slave_mii_bus->priv = priv;

	ret = mdiobus_register(priv->ds->slave_mii_bus);
	if (ret && mii_np) {
		of_node_put(dn);
		return ret;
	}

	dn = mii_np;
	for_each_node_by_name(dn, "ethernet-phy") {
		if (of_property_read_u32(dn, "reg", &pn))
			continue;

		priv->ports[pn].dp = dsa_to_port(priv->ds, pn);

		// Check for the integrated SerDes of the RTL8380M first
		if (of_property_read_bool(dn, "phy-is-integrated")
			&& priv->id == 0x8380 && pn >= 24) {
			pr_info("----> FÓUND A SERDES\n");
			priv->ports[pn].phy = PHY_RTL838X_SDS;
			continue;
		}

		if (of_property_read_bool(dn, "phy-is-integrated")
			&& !of_property_read_bool(dn, "sfp")) {
			priv->ports[pn].phy = PHY_RTL8218B_INT;
			continue;
		}

		if (!of_property_read_bool(dn, "phy-is-integrated")
			&& of_property_read_bool(dn, "sfp")) {
			priv->ports[pn].phy = PHY_RTL8214FC;
			continue;
		}

		if (!of_property_read_bool(dn, "phy-is-integrated")
			&& !of_property_read_bool(dn, "sfp")) {
			priv->ports[pn].phy = PHY_RTL8218B_EXT;
			continue;
		}
	}

	dump_fdb(priv);  // TODO: Remove, this just shows the state after u-boot finished
	// TODO: Do this needs to come from the .dts, at least the SerDes number
	if (priv->family_id == RTL9300_FAMILY_ID) {
		priv->ports[24].is2G5 = true;
		priv->ports[25].is2G5 = true;
		priv->ports[24].sds_num = 1;
		priv->ports[24].sds_num = 2;
	}

	/* Disable MAC polling the PHY so that we can start configuration */
	priv->r->set_port_reg_le(0ULL, priv->r->smi_poll_ctrl);

	/* Enable PHY control via SoC */
	if (priv->family_id == RTL8380_FAMILY_ID) {
		/* Enable PHY control via SoC */
		sw_w32_mask(0, 1 << 15, RTL838X_SMI_GLB_CTRL);
	} else {
		/* Disable PHY polling via SoC */
		sw_w32_mask(1 << 7, 0, RTL839X_SMI_GLB_CTRL);
	}

	/* Power on fibre ports and reset them if necessary */
	if (priv->ports[24].phy == PHY_RTL838X_SDS) {
		pr_info("Powering on fibre ports & reset\n");
		rtl8380_sds_power(24, 1);
		rtl8380_sds_power(26, 1);
	}

	if (priv->family_id == RTL9300_FAMILY_ID) {
		pr_info("RTL9300 Powering on SerDes ports\n");
		rtl9300_sds_power(24, 1);
		rtl9300_sds_power(25, 1);
		rtl9300_sds_power(26, 1);
		rtl9300_sds_power(27, 1);
	}

	pr_info("%s done\n", __func__);
	return 0;
}


static int rtl838x_handle_changeupper(struct rtl838x_switch_priv *priv,
				      struct net_device *ndev,
				      struct netdev_notifier_changeupper_info *info)
{
	struct net_device *upper = info->upper_dev;
	int i, j, err;

	if (!netif_is_lag_master(upper))
		return 0;

	mutex_lock(&priv->reg_mutex);

	for (i = 0; i < priv->n_lags; i++) {
		if ((!priv->lag_devs[i]) || (priv->lag_devs[i] == upper))
			break;
	}
	for (j = 0; j < priv->cpu_port; j++) {
		if (priv->ports[j].dp->slave == ndev)
			break;
	}
	if (j >= priv->cpu_port) {
		err = -EINVAL;
		goto out;
	}

	if (info->linking) {
		if (!priv->lag_devs[i])
			priv->lag_devs[i] = upper;
		err = rtl838x_lag_add(priv->ds, i, priv->ports[j].dp->index);
		if (err) {
			err = -EINVAL;
			goto out;
		}
	} else {
		if (!priv->lag_devs[i])
			err = -EINVAL;
		err = rtl838x_lag_del(priv->ds, i, priv->ports[j].dp->index);
		if (err) {
			err = -EINVAL;
			goto out;
		}
		if (!priv->lags_port_members[i])
			priv->lag_devs[i] = NULL;
	}

out:
	mutex_unlock(&priv->reg_mutex);
	return 0;
}

static int rtl838x_netdevice_event(struct notifier_block *this,
				   unsigned long event, void *ptr)
{
	struct net_device *ndev = netdev_notifier_info_to_dev(ptr);
	struct rtl838x_switch_priv *priv;
	int err;

	pr_debug("In: %s, event: %lu\n", __func__, event);

	if ((event != NETDEV_CHANGEUPPER) && (event != NETDEV_CHANGELOWERSTATE))
		return NOTIFY_DONE;

	priv = container_of(this, struct rtl838x_switch_priv, nb);
	switch (event) {
	case NETDEV_CHANGEUPPER:
		err = rtl838x_handle_changeupper(priv, ndev, ptr);
		break;
	}

	if (err)
		return err;

	return NOTIFY_DONE;
}

static const struct dsa_switch_ops rtl930x_switch_ops = {
	.get_tag_protocol	= rtl838x_get_tag_protocol,
	.setup			= rtl930x_setup,
	.port_vlan_filtering	= rtl838x_vlan_filtering,
	.port_vlan_prepare	= rtl838x_vlan_prepare,
	.port_vlan_add		= rtl838x_vlan_add,
	.port_vlan_del		= rtl838x_vlan_del,
	.port_bridge_join	= rtl838x_port_bridge_join,
	.port_bridge_leave	= rtl838x_port_bridge_leave,
	.port_stp_state_set	= rtl838x_port_stp_state_set,
	.set_ageing_time	= rtl838x_set_l2aging,
	.port_fast_age		= rtl930x_fast_age,
	.port_fdb_add		= rtl838x_port_fdb_add,
	.port_fdb_del		= rtl838x_port_fdb_del,
	.port_fdb_dump		= rtl838x_port_fdb_dump,
	.port_enable		= rtl838x_port_enable,
	.port_disable		= rtl838x_port_disable,
	.port_mirror_add	= rtl838x_port_mirror_add,
	.port_mirror_del	= rtl838x_port_mirror_del,
	.phy_read		= dsa_phy_read,
	.phy_write		= dsa_phy_write,
	.get_strings		= rtl838x_get_strings,
	.get_ethtool_stats	= rtl838x_get_ethtool_stats,
	.get_sset_count		= rtl838x_get_sset_count,
	.phylink_validate	= rtl838x_phylink_validate,
	.phylink_mac_link_state	= rtl838x_phylink_mac_link_state,
	.phylink_mac_config	= rtl838x_phylink_mac_config,
	.phylink_mac_link_down	= rtl838x_phylink_mac_link_down,
	.phylink_mac_link_up	= rtl838x_phylink_mac_link_up,
	.set_mac_eee		= rtl838x_set_mac_eee,
	.get_mac_eee		= rtl838x_get_mac_eee,
};

static const struct dsa_switch_ops rtl838x_switch_ops = {
	.get_tag_protocol	= rtl838x_get_tag_protocol,
	.setup			= rtl838x_setup,
	.port_vlan_filtering	= rtl838x_vlan_filtering,
	.port_vlan_prepare	= rtl838x_vlan_prepare,
	.port_vlan_add		= rtl838x_vlan_add,
	.port_vlan_del		= rtl838x_vlan_del,
	.port_bridge_join	= rtl838x_port_bridge_join,
	.port_bridge_leave	= rtl838x_port_bridge_leave,
	.port_stp_state_set	= rtl838x_port_stp_state_set,
	.set_ageing_time	= rtl838x_set_l2aging,
	.port_fast_age		= rtl838x_fast_age,
	.port_fdb_add		= rtl838x_port_fdb_add,
	.port_fdb_del		= rtl838x_port_fdb_del,
	.port_fdb_dump		= rtl838x_port_fdb_dump,
	.port_enable		= rtl838x_port_enable,
	.port_disable		= rtl838x_port_disable,
	.port_mirror_add	= rtl838x_port_mirror_add,
	.port_mirror_del	= rtl838x_port_mirror_del,
	.phy_read		= dsa_phy_read,
	.phy_write		= dsa_phy_write,
	.get_strings		= rtl838x_get_strings,
	.get_ethtool_stats	= rtl838x_get_ethtool_stats,
	.get_sset_count		= rtl838x_get_sset_count,
	.phylink_validate	= rtl838x_phylink_validate,
	.phylink_mac_link_state	= rtl838x_phylink_mac_link_state,
	.phylink_mac_config	= rtl838x_phylink_mac_config,
	.phylink_mac_link_down	= rtl838x_phylink_mac_link_down,
	.phylink_mac_link_up	= rtl838x_phylink_mac_link_up,
	.set_mac_eee		= rtl838x_set_mac_eee,
	.get_mac_eee		= rtl838x_get_mac_eee,
};

static int __init rtl838x_sw_probe(struct platform_device *pdev)
{
	int err = 0, i;
	struct rtl838x_switch_priv *priv;
	struct device *dev = &pdev->dev;

	pr_info("Probing RTL838X switch device\n");

	rtl_table_init();

	if (!pdev->dev.of_node) {
		dev_err(dev, "No DT found\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ds = dsa_switch_alloc(dev, DSA_MAX_PORTS);

	if (!priv->ds)
		return -ENOMEM;
	priv->ds->dev = dev;
	priv->ds->priv = priv;
	priv->dev = dev;

	priv->family_id = soc_info.family;
	priv->id = soc_info.id;
	switch(soc_info.family) {
	case RTL8380_FAMILY_ID:
		priv->ds->ops = &rtl838x_switch_ops;
		priv->cpu_port = RTL838X_CPU_PORT;
		priv->port_mask = 0x1f;
		priv->port_width = 1;
		priv->irq_mask = 0x0FFFFFFF;
		priv->r = &rtl838x_reg;
		priv->ds->num_ports = 29;
		priv->fib_entries = 8192;
		rtl8380_get_version(priv);
		priv->n_lags = 8;
		break;
	case RTL8390_FAMILY_ID:
		priv->ds->ops = &rtl838x_switch_ops;
		priv->cpu_port = RTL839X_CPU_PORT;
		priv->port_mask = 0x3f;
		priv->port_width = 2;
		priv->irq_mask = 0xFFFFFFFFFFFFFULL;
		priv->r = &rtl839x_reg;
		priv->ds->num_ports = 53;
		priv->fib_entries = 16384;
		rtl8390_get_version(priv);
		priv->n_lags = 16;
		break;
	case RTL9300_FAMILY_ID:
		priv->ds->ops = &rtl930x_switch_ops;
		priv->cpu_port = RTL930X_CPU_PORT;
		priv->port_mask = 0x1f;
		priv->port_width = 1;
		priv->irq_mask = 0x0FFFFFFF;
		priv->r = &rtl930x_reg;
		priv->ds->num_ports = 29;
		priv->fib_entries = 16384;
		priv->version = RTL8390_VERSION_A;
		priv->n_lags = 16;
		sw_w32(1, RTL930X_ST_CTRL);
		break;
	case RTL9310_FAMILY_ID:
		priv->ds->ops = &rtl930x_switch_ops;
		priv->cpu_port = RTL931X_CPU_PORT;
		priv->port_mask = 0x3f;
		priv->port_width = 2;
		priv->irq_mask = 0xFFFFFFFFFFFFFULL;
		priv->r = &rtl931x_reg;
		priv->ds->num_ports = 57;
		priv->fib_entries = 16384;
		priv->version = RTL8390_VERSION_A;
		priv->n_lags = 16;
		break;
	}
	pr_info("Chip version %c\n", priv->version);

	err = rtl838x_mdio_probe(priv);
	if (err) {
		/* Probing fails the 1st time because of missing ethernet driver
		 * initialization. Use this to disable traffic in case the bootloader left if on
		 */
		return err;
	}
	err = dsa_register_switch(priv->ds);
	if (err) {
		dev_err(dev, "Error registering switch: %d\n", err);
		return err;
	}

	/* TODO: Enable link and media change interrupts. Are the SERDES masks needed? */
	sw_w32_mask(0, 3, priv->r->isr_glb_src);
	/* ... for all ports */

	priv->r->set_port_reg_le(priv->irq_mask, priv->r->isr_port_link_sts_chg);
	priv->r->set_port_reg_le(priv->irq_mask, priv->r->imr_port_link_sts_chg);

	switch (priv->family_id) {
	case RTL8380_FAMILY_ID:
		priv->link_state_irq = 20;
		err = request_irq(priv->link_state_irq, rtl838x_switch_irq,
				IRQF_SHARED, "rtl838x-link-state", priv->ds);
		break;
	case RTL8390_FAMILY_ID:
		priv->link_state_irq = 20;
		err = request_irq(priv->link_state_irq, rtl839x_switch_irq,
				IRQF_SHARED, "rtl839x-link-state", priv->ds);
		break;
	case RTL9300_FAMILY_ID:
		priv->link_state_irq = 23;
		err = request_irq(priv->link_state_irq, rtl930x_switch_irq,
				IRQF_SHARED, "rtl930x-link-state", priv->ds);
		break;
	case RTL9310_FAMILY_ID:
		priv->link_state_irq = 20;
		err = request_irq(priv->link_state_irq, rtl931x_switch_irq,
				IRQF_SHARED, "rtl931x-link-state", priv->ds);
		break;
	}
	if (err) {
		dev_err(dev, "Error setting up switch interrupt.\n");
		/* Need to free allocated switch here */
	}

	/* Enable interrupts for switch, on RTL931x, the IRQ is always on */
	if (soc_info.family != RTL9310_FAMILY_ID)
		sw_w32(0x1, priv->r->imr_glb);

	rtl838x_get_l2aging(priv);

	pr_info("Enabling rate control\n");
	if (priv->family_id == RTL8380_FAMILY_ID)
		rtl838x_rate_control_init(priv);
	else if (priv->family_id == RTL8390_FAMILY_ID)
		rtl839x_rate_control_init(priv);

	/* Clear all destination ports for mirror groups */
	for (i = 0; i < 4; i++)
		priv->mirror_group_ports[i] = -1;

	priv->nb.notifier_call = rtl838x_netdevice_event;
		if (register_netdevice_notifier(&priv->nb)) {
			priv->nb.notifier_call = NULL;
			dev_err(dev, "Failed to register LAG netdev notifier\n");
	}

	rtl838x_dbgfs_init(priv);

	return err;
}

static int rtl838x_sw_remove(struct platform_device *pdev)
{
	pr_info("Removing platform driver for rtl838x-sw\n");
	return 0;
}

static const struct of_device_id rtl838x_switch_of_ids[] = {
	{ .compatible = "realtek,rtl838x-switch"},
	{ /* sentinel */ }
};


MODULE_DEVICE_TABLE(of, rtl838x_switch_of_ids);

static struct platform_driver rtl838x_switch_driver = {
	.probe = rtl838x_sw_probe,
	.remove = rtl838x_sw_remove,
	.driver = {
		.name = "rtl838x-switch",
		.pm = NULL,
		.of_match_table = rtl838x_switch_of_ids,
	},
};

module_platform_driver(rtl838x_switch_driver);

MODULE_AUTHOR("B. Koblitz");
MODULE_DESCRIPTION("RTL838X SoC Switch Driver");
MODULE_LICENSE("GPL");
