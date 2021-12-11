// SPDX-License-Identifier: GPL-2.0-only

#include <asm/mach-rtl838x/mach-rtl83xx.h>
#include "rtl83xx.h"

extern struct mutex smi_lock;
extern struct rtl83xx_soc_info soc_info;

inline void rtl931x_exec_tbl0_cmd(u32 cmd)
{
	sw_w32(cmd, RTL931X_TBL_ACCESS_CTRL_0);
	do { } while (sw_r32(RTL931X_TBL_ACCESS_CTRL_0) & (1 << 20));
}

inline void rtl931x_exec_tbl1_cmd(u32 cmd)
{
	sw_w32(cmd, RTL931X_TBL_ACCESS_CTRL_1);
	do { } while (sw_r32(RTL931X_TBL_ACCESS_CTRL_1) & (1 << 17));
}

inline int rtl931x_tbl_access_data_0(int i)
{
	return RTL931X_TBL_ACCESS_DATA_0(i);
}

void rtl931x_vlan_profile_dump(int index)
{
	u64 profile[4];

	if (index < 0 || index > 15)
		return;

	profile[0] = sw_r32(RTL931X_VLAN_PROFILE_SET(index));
	profile[1] = (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 4) & 0x1FFFFFFFULL) << 32
		| (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 8) & 0xFFFFFFFF);
	profile[2] = (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 16) & 0x1FFFFFFFULL) << 32
		| (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 12) & 0xFFFFFFFF);
	profile[3] = (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 20) & 0x1FFFFFFFULL) << 32
		| (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 24) & 0xFFFFFFFF);

	pr_info("VLAN %d: L2 learning: %d, L2 Unknown MultiCast Field %llx, \
		IPv4 Unknown MultiCast Field %llx, IPv6 Unknown MultiCast Field: %llx",
		index, (u32) (profile[0] & (3 << 14)), profile[1], profile[2], profile[3]);
}

static void rtl931x_stp_get(struct rtl838x_switch_priv *priv, u16 msti, u32 port_state[])
{
	int i;
	u32 cmd = 1 << 20 /* Execute cmd */
		| 0 << 19 /* Read */
		| 5 << 15 /* Table type 0b101 */
		| (msti & 0x3fff);
	priv->r->exec_tbl0_cmd(cmd);

	for (i = 0; i < 4; i++)
		port_state[i] = sw_r32(priv->r->tbl_access_data_0(i));
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

inline static int rtl931x_trk_mbr_ctr(int group)
{
	return RTL931X_TRK_MBR_CTRL + (group << 2);
}

static void rtl931x_vlan_tables_read(u32 vlan, struct rtl838x_vlan_info *info)
{
	u32 v, w, x, y;
	// Read VLAN table (3) via register 0
	struct table_reg *r = rtl_table_get(RTL931X_TBL_0, 3);

	rtl_table_read(r, vlan);
	v = sw_r32(rtl_table_data(r, 0));
	w = sw_r32(rtl_table_data(r, 1));
	x = sw_r32(rtl_table_data(r, 2));
	y = sw_r32(rtl_table_data(r, 3));
	rtl_table_release(r);

	pr_debug("VLAN_READ %d: %08x %08x %08x %08x\n", vlan, v, w, x, y);
	info->tagged_ports = ((u64) v) << 25 | (w >> 7);
	info->profile_id = (x >> 16) & 0xf;
	info->fid = w & 0x7f;				// AKA MSTI depending on context
	info->hash_uc_fid = !!(x & BIT(31));
	info->hash_mc_fid = !!(x & BIT(30));
	info->if_id = (x >> 20) & 0x3ff;
	info->profile_id = (x >> 16) & 0xf;
	info->multicast_grp_mask = x & 0xffff;
	if (x & BIT(31))
		info->l2_tunnel_list_id = y >> 18;
	else
		info->l2_tunnel_list_id = -1;
	pr_debug("%s read tagged %016llx, profile-id %d, uc %d, mc %d, intf-id %d\n", __func__,
		info->tagged_ports, info->profile_id, info->hash_uc_fid, info->hash_mc_fid,
		info->if_id);

	// Read UNTAG table via table register 3
	r = rtl_table_get(RTL931X_TBL_3, 0);
	rtl_table_read(r, vlan);
	v = ((u64)sw_r32(rtl_table_data(r, 0))) << 25;
	v |= sw_r32(rtl_table_data(r, 1)) >> 7;
	rtl_table_release(r);

	info->untagged_ports = v;
}

static void rtl931x_vlan_set_tagged(u32 vlan, struct rtl838x_vlan_info *info)
{
	u32 v, w, x, y;
	// Access VLAN table (1) via register 0
	struct table_reg *r = rtl_table_get(RTL931X_TBL_0, 3);

	v = info->tagged_ports >> 25;
	w = (info->tagged_ports & 0x1fffff) << 7;
	w |= info->fid & 0x7f;
	x = info->hash_uc_fid ? BIT(31) : 0;
	x |= info->hash_mc_fid ? BIT(30) : 0;
	x |= info->if_id & 0x3ff << 20;
	x |= (info->profile_id & 0xf) << 16;
	x |= info->multicast_grp_mask & 0xffff;
	if (info->l2_tunnel_list_id >= 0) {
		y = info->l2_tunnel_list_id << 18;
		y |= BIT(31);
	} else {
		y = 0;
	}

	sw_w32(v, rtl_table_data(r, 0));
	sw_w32(w, rtl_table_data(r, 1));
	sw_w32(x, rtl_table_data(r, 2));
	sw_w32(y, rtl_table_data(r, 3));

	rtl_table_write(r, vlan);
	rtl_table_release(r);
}

static void rtl931x_vlan_set_untagged(u32 vlan, u64 portmask)
{
	struct table_reg *r = rtl_table_get(RTL931X_TBL_3, 0);

	rtl839x_set_port_reg_be(portmask << 7, rtl_table_data(r, 0));
	rtl_table_write(r, vlan);
	rtl_table_release(r);
}

static inline int rtl931x_mac_force_mode_ctrl(int p)
{
	return RTL931X_MAC_FORCE_MODE_CTRL + (p << 2);
}

static inline int rtl931x_mac_link_spd_sts(int p)
{
	return RTL931X_MAC_LINK_SPD_STS + (((p >> 3) << 2));
}

static inline int rtl931x_mac_port_ctrl(int p)
{
	return RTL930X_MAC_L2_PORT_CTRL + (p << 7);
}

static inline int rtl931x_l2_port_new_salrn(int p)
{
	return RTL931X_L2_PORT_NEW_SALRN(p);
}

static inline int rtl931x_l2_port_new_sa_fwd(int p)
{
	return RTL931X_L2_PORT_NEW_SA_FWD(p);
}

irqreturn_t rtl931x_switch_irq(int irq, void *dev_id)
{
	struct dsa_switch *ds = dev_id;
	u32 status = sw_r32(RTL931X_ISR_GLB_SRC);
	u64 ports = rtl839x_get_port_reg_le(RTL931X_ISR_PORT_LINK_STS_CHG);
	u64 link;
	int i;

	/* Clear status */
	rtl839x_set_port_reg_le(ports, RTL931X_ISR_PORT_LINK_STS_CHG);
	pr_debug("RTL931X Link change: status: %x, ports %016llx\n", status, ports);

	link = rtl839x_get_port_reg_le(RTL931X_MAC_LINK_STS);
	// Must re-read this to get correct status
	link = rtl839x_get_port_reg_le(RTL931X_MAC_LINK_STS);
	pr_debug("RTL931X Link change: status: %x, link status %016llx\n", status, link);

	for (i = 0; i < 56; i++) {
		if (ports & BIT_ULL(i)) {
			if (link & BIT_ULL(i)) {
				pr_info("%s port %d up\n", __func__, i);
				dsa_port_phylink_mac_change(ds, i, true);
			} else {
				pr_info("%s port %d down\n", __func__, i);
				dsa_port_phylink_mac_change(ds, i, false);
			}
		}
	}
	return IRQ_HANDLED;
}
#define RTL931X_SERDES_INDRT_ACCESS_CTRL	(0x5638)
#define RTL931X_SERDES_INDRT_DATA_CTRL		(0x563C)
#define RTL931X_SERDES_MODE_CTRL		(0x13cc)
#define RTL931X_PS_SERDES_OFF_MODE_CTRL		(0x13f4)

#define RTL931X_SMI_GLB_CTRL1		(0x0CBC)
#define RTL931X_SMI_GLB_CTRL0		(0x0CC0)
#define RTL931X_SMI_PORT_POLLING_CTRL	(0x0CCC)
#define RTL931X_SMI_PORT_ADDR		(0x0C74)
#define RTL931X_SMI_PORT_POLLING_SEL	(0x0C9C)
#define RTL931X_SMI_PORT_POLLING_CTRL	(0x0CCC)
#define RTL931X_SMI_INDRT_ACCESS_CTRL_0	(0x0C00)
#define RTL931X_SMI_INDRT_ACCESS_CTRL_1	(0x0C04)
#define RTL931X_SMI_INDRT_ACCESS_CTRL_2	(0x0C08)
#define RTL931X_SMI_INDRT_ACCESS_CTRL_3	(0x0C10)
#define RTL931X_SMI_INDRT_ACCESS_BC_PHYID_CTRL (0x0C14)
#define RTL931X_SMI_INDRT_ACCESS_MMD_CTRL (0xC18)
#define RTL931X_MAC_L2_GLOBAL_CTRL2	(0x1358)
#define RTL931X_MAC_L2_GLOBAL_CTRL1	(0x5548)

int rtl931x_write_phy(u32 port, u32 page, u32 reg, u32 val)
{
	u32 v;
	int err = 0;

	val &= 0xffff;
	if (port > 63 || page > 4095 || reg > 31)
		return -ENOTSUPP;

	mutex_lock(&smi_lock);
	pr_debug("%s: writing to phy %d %d %d %d\n", __func__, port, page, reg, val);
	/* Clear both port registers */
	sw_w32(0, RTL931X_SMI_INDRT_ACCESS_CTRL_2);
	sw_w32(0, RTL931X_SMI_INDRT_ACCESS_CTRL_2 + 4);
	sw_w32_mask(0, BIT(port % 32), RTL931X_SMI_INDRT_ACCESS_CTRL_2 + (port / 32) * 4);

	sw_w32_mask(0xffff, val, RTL931X_SMI_INDRT_ACCESS_CTRL_3);

	v = reg << 6 | page << 11 ;
	sw_w32(v, RTL931X_SMI_INDRT_ACCESS_CTRL_0);

	sw_w32(0x1ff, RTL931X_SMI_INDRT_ACCESS_CTRL_1);

	v |= BIT(4) | 1; /* Write operation and execute */
	sw_w32(v, RTL931X_SMI_INDRT_ACCESS_CTRL_0);

	do {
	} while (sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_0) & 0x1);

	if (sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_0) & 0x2)
		err = -EIO;

	mutex_unlock(&smi_lock);
	return err;
}

int rtl931x_read_phy(u32 port, u32 page, u32 reg, u32 *val)
{
	u32 v;

	if (port > 63 || page > 4095 || reg > 31)
		return -ENOTSUPP;

	mutex_lock(&smi_lock);

	sw_w32(port << 5, RTL931X_SMI_INDRT_ACCESS_BC_PHYID_CTRL);

	v = reg << 6 | page << 11 | 1;
	sw_w32(v, RTL931X_SMI_INDRT_ACCESS_CTRL_0);

	do {
	} while (sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_0) & 0x1);

	v = sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_0);
	*val = sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_3);
	*val = (*val & 0xffff0000) >> 16;

	pr_debug("%s: port %d, page: %d, reg: %x, val: %x, v: %08x\n",
		__func__, port, page, reg, *val, v);

	mutex_unlock(&smi_lock);
	return 0;
}

/*
 * Read an mmd register of the PHY
 */
int rtl931x_read_mmd_phy(u32 port, u32 devnum, u32 regnum, u32 *val)
{
	int err = 0;
	u32 v;
	int type = 2; // TODO:2, for C45 PHYs need to set to 1 sometimes

	mutex_lock(&smi_lock);

	// Set PHY to access via port-number
	sw_w32(port << 5, RTL931X_SMI_INDRT_ACCESS_BC_PHYID_CTRL);

	// Set MMD device number and register to write to
	sw_w32(devnum << 16 | (regnum & 0xffff), RTL931X_SMI_INDRT_ACCESS_MMD_CTRL);

	v = type << 2 | BIT(0); // MMD-access-type | EXEC
	sw_w32(v, RTL931X_SMI_INDRT_ACCESS_CTRL_0);

	do {
		v = sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_0);
	} while (v & BIT(0));

	// Check for error condition
	if (v & BIT(1))
		err = -EIO;

	*val = sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_3) >> 16;

	pr_debug("%s: port %d, regnum: %x, val: %x (err %d)\n", __func__, port, regnum, *val, err);

	mutex_unlock(&smi_lock);

	return err;
}

/*
 * Write to an mmd register of the PHY
 */
int rtl931x_write_mmd_phy(u32 port, u32 devnum, u32 regnum, u32 val)
{
	int err = 0;
	u32 v;
	int type = 1; // TODO: For C45 PHYs need to set to 2

	mutex_lock(&smi_lock);

	// Set PHY to access via port-number
	sw_w32(port << 5, RTL931X_SMI_INDRT_ACCESS_BC_PHYID_CTRL);

	// Set data to write
	sw_w32_mask(0xffff, val, RTL931X_SMI_INDRT_ACCESS_CTRL_3);

	// Set MMD device number and register to write to
	sw_w32(devnum << 16 | (regnum & 0xffff), RTL931X_SMI_INDRT_ACCESS_MMD_CTRL);

	v = BIT(4) | type << 2 | BIT(0); // WRITE | MMD-access-type | EXEC
	sw_w32(v, RTL931X_SMI_INDRT_ACCESS_CTRL_0);

	do {
		v = sw_r32(RTL931X_SMI_INDRT_ACCESS_CTRL_0);
	} while (v & BIT(0));

	pr_debug("%s: port %d, regnum: %x, val: %x (err %d)\n", __func__, port, regnum, val, err);
	mutex_unlock(&smi_lock);
	return err;
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

void rtl931x_set_distribution_algorithm(int group, int algoidx, u32 algomsk)
{
	u32 l3shift = 0;
	u32 newmask = 0;
	/* unless we clarified how the algo index is configured, we set it to 0 */
	algoidx=0;
	if (algomsk & TRUNK_DISTRIBUTION_ALGO_SIP_BIT) {
		l3shift = 4;
		newmask |= TRUNK_DISTRIBUTION_ALGO_L3_SIP_BIT;
	}
	if (algomsk & TRUNK_DISTRIBUTION_ALGO_DIP_BIT) {
		l3shift = 4;
		newmask |= TRUNK_DISTRIBUTION_ALGO_L3_DIP_BIT;
	}
	if (algomsk & TRUNK_DISTRIBUTION_ALGO_SRC_L4PORT_BIT) {
		l3shift = 4;
		newmask |= TRUNK_DISTRIBUTION_ALGO_L3_SRC_L4PORT_BIT;
	}
	if (algomsk & TRUNK_DISTRIBUTION_ALGO_SRC_L4PORT_BIT) {
		l3shift = 4;
		newmask |= TRUNK_DISTRIBUTION_ALGO_L3_SRC_L4PORT_BIT;
	}
	if (l3shift == 4)
	{
		if (algomsk & TRUNK_DISTRIBUTION_ALGO_SMAC_BIT) {
			newmask |= TRUNK_DISTRIBUTION_ALGO_L3_SMAC_BIT;
		}
		if (algomsk & TRUNK_DISTRIBUTION_ALGO_DMAC_BIT) {
			newmask |= TRUNK_DISTRIBUTION_ALGO_L3_DMAC_BIT;
		}
	} else  {
		if (algomsk & TRUNK_DISTRIBUTION_ALGO_SMAC_BIT) {
			newmask |= TRUNK_DISTRIBUTION_ALGO_L2_SMAC_BIT;
		}
		if (algomsk & TRUNK_DISTRIBUTION_ALGO_DMAC_BIT) {
			newmask |= TRUNK_DISTRIBUTION_ALGO_L2_DMAC_BIT;
		}
	}
	sw_w32(newmask << l3shift, RTL931X_TRK_HASH_CTRL + (algoidx << 2));
}

void rtl931x_set_receive_management_action(int port, rma_ctrl_t type, action_type_t action)
{
	u32 value = 0;
	
	/* hack for value mapping */
	if (type == GRATARP && action == COPY2CPU)
		action = TRAP2MASTERCPU;

	switch(action) {
	case FORWARD:
	    value = 0;
	break;
	case DROP:
	    value = 1;
	break;
	case TRAP2CPU:
	    value = 2;
	break;
	case TRAP2MASTERCPU:
	    value = 3;
	break;
	case FLOODALL:
	    value = 4;
	break;
	default:
	break;
	}
	
	switch(type) {
	case BPDU:
		sw_w32_mask(7 << ((port % 10) * 3), value << ((port % 10) * 3), RTL931X_RMA_BPDU_CTRL + ((port / 10) << 2));
	break;
	case PTP:
		//udp
		sw_w32_mask(3 << 2, value << 2, RTL931X_RMA_PTP_CTRL + (port << 2));
		//eth2
		sw_w32_mask(3, value, RTL931X_RMA_PTP_CTRL + (port << 2));
	break;
	case PTP_UDP:
		sw_w32_mask(3 << 2, value << 2, RTL931X_RMA_PTP_CTRL + (port << 2));
	break;
	case PTP_ETH2:
		sw_w32_mask(3, value, RTL931X_RMA_PTP_CTRL + (port << 2));
	break;
	case LLTP:
		sw_w32_mask(7 << ((port % 10) * 3), value << ((port % 10) * 3), RTL931X_RMA_LLTP_CTRL + ((port / 10) << 2));
	break;
	case EAPOL:
		sw_w32_mask(7 << ((port % 10) * 3), value << ((port % 10) * 3), RTL931X_RMA_EAPOL_CTRL + ((port / 10) << 2));
	break;
	case GRATARP:
		sw_w32_mask(3 << ((port & 0xf) << 1), value << ((port & 0xf) << 1), RTL931X_TRAP_ARP_GRAT_PORT_ACT + ((port >> 4) << 2));
	break;
	}
}

u64 rtl931x_traffic_get(int source)
{
	u32 v;
	struct table_reg *r = rtl_table_get(RTL931X_TBL_0, 6);

	rtl_table_read(r, source);
	v = sw_r32(rtl_table_data(r, 0));
	rtl_table_release(r);
	return v >> 3;
}

/*
 * Enable traffic between a source port and a destination port matrix
 */
void rtl931x_traffic_set(int source, u64 dest_matrix)
{
	struct table_reg *r = rtl_table_get(RTL931X_TBL_0, 6);

	sw_w32((dest_matrix << 3), rtl_table_data(r, 0));
	rtl_table_write(r, source);
	rtl_table_release(r);
}

void rtl931x_traffic_enable(int source, int dest)
{
	struct table_reg *r = rtl_table_get(RTL931X_TBL_0, 6);
	rtl_table_read(r, source);
	sw_w32_mask(0, BIT(dest + 3), rtl_table_data(r, 0));
	rtl_table_write(r, source);
	rtl_table_release(r);
}

void rtl931x_traffic_disable(int source, int dest)
{
	struct table_reg *r = rtl_table_get(RTL931X_TBL_0, 6);
	rtl_table_read(r, source);
	sw_w32_mask(BIT(dest + 3), 0, rtl_table_data(r, 0));
	rtl_table_write(r, source);
	rtl_table_release(r);
}

static u64 rtl931x_l2_hash_seed(u64 mac, u32 vid)
{
	u64 v = vid;

	v <<= 48;
	v |= mac;

	return v;
}

/*
 * Calculate both the block 0 and the block 1 hash by applyingthe same hash
 * algorithm as the one used currently by the ASIC to the seed, and return
 * both hashes in the lower and higher word of the return value since only 12 bit of
 * the hash are significant
 */
static u32 rtl931x_l2_hash_key(struct rtl838x_switch_priv *priv, u64 seed)
{
	u32 h, h0, h1, h2, h3, h4, k0, k1;

	h0 = seed & 0xfff;
	h1 = (seed >> 12) & 0xfff;
	h2 = (seed >> 24) & 0xfff;
	h3 = (seed >> 36) & 0xfff;
	h4 = (seed >> 48) & 0xfff;
	h4 = ((h4 & 0x7) << 9) | ((h4 >> 3) & 0x1ff);
	k0 = h0 ^ h1 ^ h2 ^ h3 ^ h4;

	h0 = seed & 0xfff;
	h0 = ((h0 & 0x1ff) << 3) | ((h0 >> 9) & 0x7);
	h1 = (seed >> 12) & 0xfff;
	h1 = ((h1 & 0x3f) << 6) | ((h1 >> 6) & 0x3f);
	h2 = (seed >> 24) & 0xfff;
	h3 = (seed >> 36) & 0xfff;
	h3 = ((h3 & 0x3f) << 6) | ((h3 >> 6) & 0x3f);
	h4 = (seed >> 48) & 0xfff;
	k1 = h0 ^ h1 ^ h2 ^ h3 ^ h4;

	// Algorithm choice for block 0
	if (sw_r32(RTL931X_L2_CTRL) & BIT(0))
		h = k1;
	else
		h = k0;

	/* Algorithm choice for block 1
	 * Since k0 and k1 are < 4096, adding 4096 will offset the hash into the second
	 * half of hash-space
	 * 4096 is in fact the hash-table size 32768 divided by 4 hashes per bucket
	 * divided by 2 to divide the hash space in 2
	 */
	if (sw_r32(RTL931X_L2_CTRL) & BIT(1))
		h |= (k1 + 4096) << 16;
	else
		h |= (k0 + 4096) << 16;

	return h;
}

/*
 * Fills an L2 entry structure from the SoC registers
 */
static void rtl931x_fill_l2_entry(u32 r[], struct rtl838x_l2_entry *e)
{
	pr_debug("In %s valid?\n", __func__);
	e->valid = !!(r[0] & BIT(31));
	if (!e->valid)
		return;

	pr_debug("%s: entry valid, raw: %08x %08x %08x %08x\n", __func__, r[0], r[1], r[2], r[3]);
	e->is_ip_mc = false;
	e->is_ipv6_mc = false;

	e->mac[0] = r[0] >> 8;
	e->mac[1] = r[0];
	e->mac[2] = r[1] >> 24;
	e->mac[3] = r[1] >> 16;
	e->mac[4] = r[1] >> 8;
	e->mac[5] = r[1];

	e->is_open_flow = !!(r[0] & BIT(30));
	e->is_pe_forward = !!(r[0] & BIT(29));
	e->next_hop = !!(r[2] & BIT(30));
	e->rvid = (r[0] >> 16) & 0xfff;

	/* Is it a unicast entry? check multicast bit */
	if (!(e->mac[0] & 1)) {
		e->type = L2_UNICAST;
		e->is_l2_tunnel = !!(r[2] & BIT(31));
		e->is_static = !!(r[2] & BIT(13));
		e->port = (r[2] >> 19) & 0x3ff;
		// Check for trunk port
		if (r[2] & BIT(29)) {
			e->is_trunk = true;
			e->stack_dev = (e->port >> 9) & 1;
			e->trunk = e->port & 0x3f;
		} else {
			e->is_trunk = false;
			e->stack_dev = (e->port >> 6) & 0xf;
			e->port = e->port & 0x3f;
		}

		e->block_da = !!(r[2] & BIT(14));
		e->block_sa = !!(r[2] & BIT(15));
		e->suspended = !!(r[2] & BIT(12));
		e->age = (r[2] >> 16) & 3;
		
		// the UC_VID field in hardware is used for the VID or for the route id
		if (e->next_hop) {
			e->nh_route_id = r[2] & 0x7ff;
			e->vid = 0;
		} else {
			e->vid = r[2] & 0xfff;
			e->nh_route_id = 0;
		}
		if (e->is_l2_tunnel)
			e->l2_tunnel_id = ((r[2] & 0xff) << 4) | (r[3] >> 28);
		// TODO: Implement VLAN conversion
	} else {
		e->type = L2_MULTICAST;
		e->is_local_forward = !!(r[2] & BIT(31));
		e->is_remote_forward = !!(r[2] & BIT(17));
		e->mc_portmask_index = (r[2] >> 18) & 0xfff;
		e->l2_tunnel_list_id = (r[2] >> 4) & 0x1fff;
	}
}

/*
 * Fills the 3 SoC table registers r[] with the information of in the rtl838x_l2_entry
 */
static void rtl931x_fill_l2_row(u32 r[], struct rtl838x_l2_entry *e)
{
	u32 port;

	if (!e->valid) {
		r[0] = r[1] = r[2] = 0;
		return;
	}

	r[2] = BIT(31);	// Set valid bit

	r[0] = ((u32)e->mac[0]) << 24 | ((u32)e->mac[1]) << 16 
		| ((u32)e->mac[2]) << 8 | ((u32)e->mac[3]);
	r[1] = ((u32)e->mac[4]) << 24 | ((u32)e->mac[5]) << 16;

	r[2] |= e->next_hop ? BIT(12) : 0;

	if (e->type == L2_UNICAST) {
		r[2] |= e->is_static ? BIT(14) : 0;
		r[1] |= e->rvid & 0xfff;
		r[2] |= (e->port & 0x3ff) << 20;
		if (e->is_trunk) {
			r[2] |= BIT(30);
			port = e->stack_dev << 9 | (e->port & 0x3f);
		} else {
			port = (e->stack_dev & 0xf) << 6;
			port |= e->port & 0x3f;
		}
		r[2] |= port << 20;
		r[2] |= e->block_da ? BIT(15) : 0;
		r[2] |= e->block_sa ? BIT(17) : 0;
		r[2] |= e->suspended ? BIT(13) : 0;
		r[2] |= (e->age & 0x3) << 17;
		// the UC_VID field in hardware is used for the VID or for the route id
		if (e->next_hop)
			r[2] |= e->nh_route_id & 0x7ff;
		else
			r[2] |= e->vid & 0xfff;
	} else { // L2_MULTICAST
		r[2] |= (e->mc_portmask_index & 0x3ff) << 16;
		r[2] |= e->mc_mac_index & 0x7ff;
	}
}

/*
 * Read an L2 UC or MC entry out of a hash bucket of the L2 forwarding table
 * hash is the id of the bucket and pos is the position of the entry in that bucket
 * The data read from the SoC is filled into rtl838x_l2_entry
 */
static u64 rtl931x_read_l2_entry_using_hash(u32 hash, u32 pos, struct rtl838x_l2_entry *e)
{
	u32 r[4];
	struct table_reg *q = rtl_table_get(RTL931X_TBL_0, 0);
	u32 idx;
	int i;
	u64 mac;
	u64 seed;

	pr_debug("%s: hash %08x, pos: %d\n", __func__, hash, pos);

	/* On the RTL93xx, 2 different hash algorithms are used making it a total of
	 * 8 buckets that need to be searched, 4 for each hash-half
	 * Use second hash space when bucket is between 4 and 8 */
	if (pos >= 4) {
		pos -= 4;
		hash >>= 16;
	} else {
		hash &= 0xffff;
	}

	idx = (0 << 14) | (hash << 2) | pos; // Search SRAM, with hash and at pos in bucket
	pr_debug("%s: NOW hash %08x, pos: %d\n", __func__, hash, pos);

	rtl_table_read(q, idx);
	for (i = 0; i < 4; i++)
		r[i] = sw_r32(rtl_table_data(q, i));

	rtl_table_release(q);

	rtl931x_fill_l2_entry(r, e);

	pr_debug("%s: valid: %d, nh: %d\n", __func__, e->valid, e->next_hop);
	if (!e->valid)
		return 0;

	mac = ((u64)e->mac[0]) << 40 | ((u64)e->mac[1]) << 32 | ((u64)e->mac[2]) << 24
		| ((u64)e->mac[3]) << 16 | ((u64)e->mac[4]) << 8 | ((u64)e->mac[5]);

	seed = rtl931x_l2_hash_seed(mac, e->rvid);
	pr_debug("%s: mac %016llx, seed %016llx\n", __func__, mac, seed);
	// return vid with concatenated mac as unique id
	return seed;
}

static u64 rtl931x_read_cam(int idx, struct rtl838x_l2_entry *e)
{
		return 0;
}

static void rtl931x_write_cam(int idx, struct rtl838x_l2_entry *e)
{
}

static void rtl931x_write_l2_entry_using_hash(u32 hash, u32 pos, struct rtl838x_l2_entry *e)
{
}
static void rtl931x_vlan_fwd_on_inner(int port, bool is_set)
{
}

static void rtl931x_vlan_profile_setup(int profile)
{
	u32 p[7];
	int i;

	pr_info("In %s\n", __func__);

	if (profile > 15)
		return;

	p[0] = sw_r32(RTL931X_VLAN_PROFILE_SET(profile));

	// Enable routing of Ipv4/6 Unicast and IPv4/6 Multicast traffic
	//p[0] |= BIT(17) | BIT(16) | BIT(13) | BIT(12);
	p[0] |= 0x3 << 11; // COPY2CPU

	p[1] = 0x1FFFFFF; // L2 unknwon MC flooding portmask all ports, including the CPU-port
	p[2] = 0xFFFFFFFF;
	p[3] = 0x1FFFFFF; // IPv4 unknwon MC flooding portmask
	p[4] = 0xFFFFFFFF;
	p[5] = 0x1FFFFFF; // IPv6 unknwon MC flooding portmask
	p[6] = 0xFFFFFFFF;

	for (i = 0; i < 7; i++)
		sw_w32(p[i], RTL931X_VLAN_PROFILE_SET(profile) + i * 4);
	pr_info("Leaving %s\n", __func__);
}


static u64 rtl931x_read_mcast_pmask(int idx)
{
	u64 portmask;
	// Read MC_PMSK (2) via register RTL931X_TBL_0
	struct table_reg *q = rtl_table_get(RTL931X_TBL_0, 2);

	rtl_table_read(q, idx);
	portmask = sw_r32(rtl_table_data(q, 0));
	portmask <<= 32;
	portmask |= sw_r32(rtl_table_data(q, 1));
	portmask >>= 7;
	rtl_table_release(q);

	pr_debug("%s: Index idx %d has portmask %016llx\n", __func__, idx, portmask);
	return portmask;
}

static void rtl931x_write_mcast_pmask(int idx, u64 portmask)
{
	u64 pm = portmask;

	// Access MC_PMSK (2) via register RTL931X_TBL_0
	struct table_reg *q = rtl_table_get(RTL931X_TBL_0, 2);

	pr_info("%s: Index idx %d has portmask %016llx\n", __func__, idx, pm);
	pm <<= 7;
	sw_w32((u32)(pm >> 32), rtl_table_data(q, 0));
	sw_w32((u32)pm, rtl_table_data(q, 1));
	rtl_table_write(q, idx);
	rtl_table_release(q);
}

static void rtl931x_pie_init(struct rtl838x_switch_priv *priv)
{
}

typedef struct {
	u8 page;
	u8 reg;
	u16 data;
} sds_config;

static sds_config dal_mango_construct_ana_common[] = {
	{ 0x21, 0x00, 0x1800 }, { 0x21, 0x01, 0x0060 }, { 0x21, 0x02, 0x3000 }, { 0x21, 0x03, 0xFFFF },
	{ 0x21, 0x04, 0x0603 }, { 0x21, 0x05, 0x1104 }, { 0x21, 0x06, 0x4444 }, { 0x21, 0x07, 0x7044 },
	{ 0x21, 0x08, 0xF104 }, { 0x21, 0x09, 0xF104 }, { 0x21, 0x0A, 0xF104 }, { 0x21, 0x0B, 0x0003 },
	{ 0x21, 0x0C, 0x007F }, { 0x21, 0x0D, 0x3FE4 }, { 0x21, 0x0E, 0x31F9 }, { 0x21, 0x0F, 0x0618 },
	{ 0x21, 0x10, 0x1FF8 }, { 0x21, 0x11, 0x7C9F }, { 0x21, 0x12, 0x7C9F }, { 0x21, 0x13, 0x13FF },
	{ 0x21, 0x14, 0x001F }, { 0x21, 0x15, 0x01F0 }, { 0x21, 0x16, 0x1067 }, { 0x21, 0x17, 0x8AF1 },
	{ 0x21, 0x18, 0x210A }, { 0x21, 0x19, 0xF0F0 }
};

static sds_config dal_mango_construct_ana_10p3125g[] = {
	{ 0x2E, 0x00, 0x0107 }, { 0x2E, 0x01, 0x0200 }, { 0x2E, 0x02, 0x6A24 }, { 0x2E, 0x03, 0xD10D },
	{ 0x2E, 0x04, 0xD550 }, { 0x2E, 0x05, 0xA95E }, { 0x2E, 0x06, 0xE31D }, { 0x2E, 0x07, 0x000E },
	{ 0x2E, 0x08, 0x0294 }, { 0x2E, 0x09, 0x0CE4 }, { 0x2E, 0x0a, 0x7FC8 }, { 0x2E, 0x0b, 0xE0E7 },
	{ 0x2E, 0x0c, 0x0200 }, { 0x2E, 0x0d, 0xDF80 }, { 0x2E, 0x0e, 0x0000 }, { 0x2E, 0x0f, 0x1FC4 },
	{ 0x2E, 0x10, 0x0C3F }, { 0x2E, 0x11, 0x0000 }, { 0x2E, 0x12, 0x27C0 }, { 0x2E, 0x13, 0x7F1C },
	{ 0x2E, 0x14, 0x1300 }, { 0x2E, 0x15, 0x003F }, { 0x2E, 0x16, 0xBE7F }, { 0x2E, 0x17, 0x0090 },
	{ 0x2E, 0x18, 0x0000 }, { 0x2E, 0x19, 0x4000 }, { 0x2E, 0x1a, 0x0000 }, { 0x2E, 0x1b, 0x8000 },
	{ 0x2E, 0x1c, 0x011E }, { 0x2E, 0x1d, 0x0000 }, { 0x2E, 0x1e, 0xC8FF }, { 0x2E, 0x1f, 0x0000 },
	{ 0x2F, 0x00, 0xC000 }, { 0x2F, 0x01, 0xF000 }, { 0x2F, 0x02, 0x6010 },
	{ 0x2F, 0x12, 0x0EEE }, { 0x2F, 0x13, 0x0000 }, { 0x6, 0x0, 0x0000 }
};

static sds_config dal_mango_construct_ana_10p3125g_cmu[] = {
	{ 0x2F, 0x03, 0x4210 }, { 0x2F, 0x04, 0x0000 }, { 0x2F, 0x05, 0x3FD9 }, { 0x2F, 0x06, 0x58A6 },
	{ 0x2F, 0x07, 0x2990 }, { 0x2F, 0x08, 0xFFF4 }, { 0x2F, 0x09, 0x1F08 }, { 0x2F, 0x0A, 0x0000 },
	{ 0x2F, 0x0B, 0x8000 }, { 0x2F, 0x0C, 0x4224 }, { 0x2F, 0x0D, 0x0000 }, { 0x2F, 0x0E, 0x0400 },
	{ 0x2F, 0x0F, 0xA464 }, { 0x2F, 0x10, 0x8000 }, { 0x2F, 0x11, 0x0165 }, { 0x20, 0x11, 0x000D },
	{ 0x20, 0x12, 0x510F }, { 0x20, 0x00, 0x0030 }
};

static sds_config dal_mango_construct_ana_5g[] = {
	{ 0x2A, 0x00, 0x0104 }, { 0x2A, 0x01, 0x0200 }, { 0x2A, 0x02, 0x2A24 }, { 0x2A, 0x03, 0xD10D },
	{ 0x2A, 0x04, 0xD550 }, { 0x2A, 0x05, 0xA95E }, { 0x2A, 0x06, 0xE31D }, { 0x2A, 0x07, 0x800E },
	{ 0x2A, 0x08, 0x0294 }, { 0x2A, 0x09, 0x28E4 }, { 0x2A, 0x0A, 0x7FC8 }, { 0x2A, 0x0B, 0xE0E7 },
	{ 0x2A, 0x0C, 0x0200 }, { 0x2A, 0x0D, 0x9F80 }, { 0x2A, 0x0E, 0x0800 }, { 0x2A, 0x0F, 0x1FC8 },
	{ 0x2A, 0x10, 0x0C3F }, { 0x2A, 0x11, 0x0000 }, { 0x2A, 0x12, 0x27C0 }, { 0x2A, 0x13, 0x7F1C },
	{ 0x2A, 0x14, 0x1300 }, { 0x2A, 0x15, 0x003F }, { 0x2A, 0x16, 0xBE7F }, { 0x2A, 0x17, 0x0090 },
	{ 0x2A, 0x18, 0x0000 }, { 0x2A, 0x19, 0x407F }, { 0x2A, 0x1A, 0x0000 }, { 0x2A, 0x1B, 0x8000 },
	{ 0x2A, 0x1C, 0x011E }, { 0x2A, 0x1D, 0x0000 }, { 0x2A, 0x1E, 0xC8FF }, { 0x2A, 0x1F, 0x0000 },
	{ 0x2B, 0x00, 0xC000 }, { 0x2B, 0x01, 0xF000 }, { 0x2B, 0x02, 0x6010 },
	{ 0x2B, 0x12, 0x0EEE }, { 0x2B, 0x13, 0x0000 }
};

static sds_config dal_mango_construct_ana_5g_cmu[] = {
	{ 0x2B, 0x03, 0x5010 }, { 0x2B, 0x04, 0x0000 }, { 0x2B, 0x05, 0x27D9 }, { 0x2B, 0x06, 0x58A6 },
	{ 0x2B, 0x07, 0x2990 }, { 0x2B, 0x08, 0xFFF4 }, { 0x2B, 0x09, 0x2682 }, { 0x2B, 0x0A, 0x0000 },
	{ 0x2B, 0x0B, 0x8000 }, { 0x2B, 0x0C, 0x5024 }, { 0x2B, 0x0D, 0x0000 }, { 0x2B, 0x0E, 0x0000 },
	{ 0x2B, 0x0F, 0xA470 }, { 0x2B, 0x10, 0x8000 }, { 0x2B, 0x11, 0x0362 }
};

static sds_config dal_mango_construct_ana_3p125g[] = {
	{ 0x28, 0x00, 0x0104 }, { 0x28, 0x01, 0x0200 }, { 0x28, 0x02, 0x2A24 }, { 0x28, 0x03, 0xD10D },
	{ 0x28, 0x04, 0xD550 }, { 0x28, 0x05, 0xA95E }, { 0x28, 0x06, 0xE31D }, { 0x28, 0x07, 0x800E },
	{ 0x28, 0x08, 0x0294 }, { 0x28, 0x09, 0x04E4 }, { 0x28, 0x0A, 0x7FC8 }, { 0x28, 0x0B, 0xE0E7 },
	{ 0x28, 0x0C, 0x0200 }, { 0x28, 0x0D, 0xDF80 }, { 0x28, 0x0E, 0x0800 }, { 0x28, 0x0F, 0x1FD8 },
	{ 0x28, 0x10, 0x0C3F }, { 0x28, 0x11, 0x0000 }, { 0x28, 0x12, 0x27C0 }, { 0x28, 0x13, 0x7F1C },
	{ 0x28, 0x14, 0x1300 }, { 0x28, 0x15, 0x003F }, { 0x28, 0x16, 0xBE7F }, { 0x28, 0x17, 0x0090 },
	{ 0x28, 0x18, 0x0000 }, { 0x28, 0x19, 0x407F }, { 0x28, 0x1A, 0x0000 }, { 0x28, 0x1B, 0x8000 },
	{ 0x28, 0x1C, 0x011E }, { 0x28, 0x1D, 0x0000 }, { 0x28, 0x1E, 0xC8FF }, { 0x28, 0x1F, 0x0000 },
	{ 0x29, 0x00, 0xC000 }, { 0x29, 0x01, 0xF000 }, { 0x29, 0x02, 0x6010 },
	{ 0x29, 0x12, 0x0EEE }, { 0x29, 0x13, 0x0000 }
};

static sds_config dal_mango_construct_ana_3p125g_cmu[] = {
	{ 0x2D, 0x03, 0x6410 }, { 0x2D, 0x04, 0x0000 }, { 0x2D, 0x05, 0x27D9 }, { 0x2D, 0x06, 0x58A6 },
	{ 0x2D, 0x07, 0x2990 }, { 0x2D, 0x08, 0xFFF4 }, { 0x2D, 0x09, 0x3082 }, { 0x2D, 0x0A, 0x0000 },
	{ 0x2D, 0x0B, 0x8000 }, { 0x2D, 0x0C, 0x6424 }, { 0x2D, 0x0D, 0x0000 }, { 0x2D, 0x0E, 0x0000 },
	{ 0x2D, 0x0F, 0xA470 }, { 0x2D, 0x10, 0x8000 }, { 0x2D, 0x11, 0x037B }
};

static sds_config dal_mango_construct_ana_2p5g[] = {
	{ 0x26, 0x00, 0x0104 }, { 0x26, 0x01, 0x0200 }, { 0x26, 0x02, 0x2A24 }, { 0x26, 0x03, 0xD10D },
	{ 0x26, 0x04, 0xD550 }, { 0x26, 0x05, 0xA95E }, { 0x26, 0x06, 0xE31D }, { 0x26, 0x07, 0x800E },
	{ 0x26, 0x08, 0x0294 }, { 0x26, 0x09, 0x80E4 }, { 0x26, 0x0A, 0x7FC8 }, { 0x26, 0x0B, 0xE0E7 },
	{ 0x26, 0x0C, 0x0200 }, { 0x26, 0x0D, 0xDF80 }, { 0x26, 0x0E, 0x0000 }, { 0x26, 0x0F, 0x1FE0 },
	{ 0x26, 0x10, 0x0C3F }, { 0x26, 0x11, 0x0000 }, { 0x26, 0x12, 0x27C0 }, { 0x26, 0x13, 0x7F1C },
	{ 0x26, 0x14, 0x1300 }, { 0x26, 0x15, 0x003F }, { 0x26, 0x16, 0xBE7F }, { 0x26, 0x17, 0x0090 },
	{ 0x26, 0x18, 0x0000 }, { 0x26, 0x19, 0x407F }, { 0x26, 0x1A, 0x0000 }, { 0x26, 0x1B, 0x8000 },
	{ 0x26, 0x1C, 0x011E }, { 0x26, 0x1D, 0x0000 }, { 0x26, 0x1E, 0xC8FF }, { 0x26, 0x1F, 0x0000 },
	{ 0x27, 0x00, 0xC000 }, { 0x27, 0x01, 0xF000 }, { 0x27, 0x02, 0x6010 },
	{ 0x27, 0x12, 0x0EEE }, { 0x27, 0x13, 0x0000 }
};

static sds_config dal_mango_construct_ana_1p25g[] = {
	{ 0x24, 0x00, 0x0104 }, { 0x24, 0x01, 0x0200 }, { 0x24, 0x02, 0x2A24 }, { 0x24, 0x03, 0xD10D },
	{ 0x24, 0x04, 0xD550 }, { 0x24, 0x05, 0xA95E }, { 0x24, 0x06, 0xE31D }, { 0x24, 0x07, 0x800E },
	{ 0x24, 0x08, 0x0294 }, { 0x24, 0x09, 0x04E4 }, { 0x24, 0x0A, 0x7FC8 }, { 0x24, 0x0B, 0xE0E7 },
	{ 0x24, 0x0C, 0x0200 }, { 0x24, 0x0D, 0x9F80 }, { 0x24, 0x0E, 0x0000 }, { 0x24, 0x0F, 0x1FF0 },
	{ 0x24, 0x10, 0x0C3F }, { 0x24, 0x11, 0x0000 }, { 0x24, 0x12, 0x27C0 }, { 0x24, 0x13, 0x7F1C },
	{ 0x24, 0x14, 0x1300 }, { 0x24, 0x15, 0x003F }, { 0x24, 0x16, 0xBE7F }, { 0x24, 0x17, 0x0090 },
	{ 0x24, 0x18, 0x0000 }, { 0x24, 0x19, 0x407F }, { 0x24, 0x1A, 0x0000 }, { 0x24, 0x1B, 0x8000 },
	{ 0x24, 0x1C, 0x011E }, { 0x24, 0x1D, 0x0000 }, { 0x24, 0x1E, 0xC8FF }, { 0x24, 0x1F, 0x0000 },
	{ 0x25, 0x00, 0xC000 }, { 0x25, 0x01, 0xF000 }, { 0x25, 0x02, 0x6010 },
	{ 0x25, 0x12, 0x0EEE }, { 0x25, 0x13, 0x0000 }
};

static sds_config dal_mango_construct_ana2[] = {
	{ 0x20, 0x12, 0x150F },
	{ 0x2E, 0x7, 0x800E },
	{ 0x2A, 0x7, 0x800E },
	{ 0x24, 0x7, 0x000E },
	{ 0x26, 0x7, 0x000E },
	{ 0x27, 0x7, 0x000E },

	{ 0x2F, 0x12, 0x0AAA },

	{ 0x2A, 0x12, 0x2740 },
	{ 0x2B, 0x0, 0x0 },
	{ 0x2B, 0x2, 0x2010 },
	{ 0x20, 0x0, 0x0C30 },

	{ 0x2F, 0x3, 0x84A0 },
	{ 0x2F, 0xC, 0x84A4 },

	{ 0x24, 0xD, 0xDF80 },
	{ 0x2A, 0xD, 0xDF80 },

	{ 0x2F, 0x5, 0x2FD9 },
	{ 0x2F, 0x5, 0x3FD9 },
	{ 0x21, 0x16, 0x1065 },
	{ 0x21, 0x16, 0x1067 },

	{ 0x21, 0x19, 0xF0A5 },
};

static sds_config dal_mango_construct_ana_common_type1[] = {
	{ 0x21, 0x00, 0x1800 }, { 0x21, 0x01, 0x0060 }, { 0x21, 0x02, 0x3000 }, { 0x21, 0x03, 0xFFFF },
	{ 0x21, 0x04, 0x0603 }, { 0x21, 0x05, 0x1104 }, { 0x21, 0x06, 0x4444 }, { 0x21, 0x07, 0x7044 },
	{ 0x21, 0x08, 0xF104 }, { 0x21, 0x09, 0xF104 }, { 0x21, 0x0A, 0xF104 }, { 0x21, 0x0B, 0x0003 },
	{ 0x21, 0x0C, 0x007F }, { 0x21, 0x0D, 0x3FE4 }, { 0x21, 0x0E, 0x31F9 }, { 0x21, 0x0F, 0x0618 },
	{ 0x21, 0x10, 0x1FF8 }, { 0x21, 0x11, 0x7C9F }, { 0x21, 0x12, 0x7C9F }, { 0x21, 0x13, 0x13FF },
	{ 0x21, 0x14, 0x001F }, { 0x21, 0x15, 0x01F0 }, { 0x21, 0x16, 0x1064 }, { 0x21, 0x17, 0x8AF1 },
	{ 0x21, 0x18, 0x210A }, { 0x21, 0x19, 0xF0F1 }
};

static sds_config dal_mango_construct_ana_10p3125g_type1[] = {
	{ 0x2E, 0x00, 0x0107 }, { 0x2E, 0x01, 0x01A3 }, { 0x2E, 0x02, 0x6A24 }, { 0x2E, 0x03, 0xD10D },
	{ 0x2E, 0x04, 0x8000 }, { 0x2E, 0x05, 0xA17E }, { 0x2E, 0x06, 0xE31D }, { 0x2E, 0x07, 0x800E },
	{ 0x2E, 0x08, 0x0294 }, { 0x2E, 0x09, 0x0CE4 }, { 0x2E, 0x0A, 0x7FC8 }, { 0x2E, 0x0B, 0xE0E7 },
	{ 0x2E, 0x0C, 0x0200 }, { 0x2E, 0x0D, 0xDF80 }, { 0x2E, 0x0E, 0x0000 }, { 0x2E, 0x0F, 0x1FC2 },
	{ 0x2E, 0x10, 0x0C3F }, { 0x2E, 0x11, 0x0000 }, { 0x2E, 0x12, 0x27C0 }, { 0x2E, 0x13, 0x7E1D },
	{ 0x2E, 0x14, 0x1300 }, { 0x2E, 0x15, 0x003F }, { 0x2E, 0x16, 0xBE7F }, { 0x2E, 0x17, 0x0090 },
	{ 0x2E, 0x18, 0x0000 }, { 0x2E, 0x19, 0x4000 }, { 0x2E, 0x1A, 0x0000 }, { 0x2E, 0x1B, 0x8000 },
	{ 0x2E, 0x1C, 0x011F }, { 0x2E, 0x1D, 0x0000 }, { 0x2E, 0x1E, 0xC8FF }, { 0x2E, 0x1F, 0x0000 },
	{ 0x2F, 0x00, 0xC000 }, { 0x2F, 0x01, 0xF000 }, { 0x2F, 0x02, 0x6010 },
	{ 0x2F, 0x12, 0x0EE7 }, { 0x2F, 0x13, 0x0000 }
};

static sds_config dal_mango_construct_ana_10p3125g_cmu_type1[] = {
	{ 0x2F, 0x03, 0x4210 }, { 0x2F, 0x04, 0x0000 }, { 0x2F, 0x05, 0x0019 }, { 0x2F, 0x06, 0x18A6 },
	{ 0x2F, 0x07, 0x2990 }, { 0x2F, 0x08, 0xFFF4 }, { 0x2F, 0x09, 0x1F08 }, { 0x2F, 0x0A, 0x0000 },
	{ 0x2F, 0x0B, 0x8000 }, { 0x2F, 0x0C, 0x4224 }, { 0x2F, 0x0D, 0x0000 }, { 0x2F, 0x0E, 0x0000 },
	{ 0x2F, 0x0F, 0xA470 }, { 0x2F, 0x10, 0x8000 }, { 0x2F, 0x11, 0x037B }
};

static sds_config dal_mango_construct_ana_5g_type1[] = {
	{ 0x2A, 0x00, 0xF904 }, { 0x2A, 0x01, 0x0200 }, { 0x2A, 0x02, 0x2A20 }, { 0x2A, 0x03, 0xD10D },
	{ 0x2A, 0x04, 0x8000 }, { 0x2A, 0x05, 0xA17E }, { 0x2A, 0x06, 0xE115 }, { 0x2A, 0x07, 0x000E },
	{ 0x2A, 0x08, 0x0294 }, { 0x2A, 0x09, 0x28E4 }, { 0x2A, 0x0A, 0x7FC8 }, { 0x2A, 0x0B, 0xE0E7 },
	{ 0x2A, 0x0C, 0x0200 }, { 0x2A, 0x0D, 0xDF80 }, { 0x2A, 0x0E, 0x0000 }, { 0x2A, 0x0F, 0x1FC8 },
	{ 0x2A, 0x10, 0x0C3F }, { 0x2A, 0x11, 0x0000 }, { 0x2A, 0x12, 0x27C0 }, { 0x2A, 0x13, 0x7E1D },
	{ 0x2A, 0x14, 0x1300 }, { 0x2A, 0x15, 0x003F }, { 0x2A, 0x16, 0xBE7F }, { 0x2A, 0x17, 0x0090 },
	{ 0x2A, 0x18, 0x0000 }, { 0x2A, 0x19, 0x407F }, { 0x2A, 0x1A, 0x0000 }, { 0x2A, 0x1B, 0x8000 },
	{ 0x2A, 0x1C, 0x011E }, { 0x2A, 0x1D, 0x0000 }, { 0x2A, 0x1E, 0xC8FF }, { 0x2A, 0x1F, 0x0000 },
	{ 0x2B, 0x00, 0xC000 }, { 0x2B, 0x01, 0xF000 }, { 0x2B, 0x02, 0x6010 },
	{ 0x2B, 0x12, 0x0EE7 }, { 0x2B, 0x13, 0x0000 }
};

static sds_config dal_mango_construct_ana_5g_cmu_type1[] = {
	{ 0x2B, 0x03, 0x5010 }, { 0x2B, 0x04, 0x0000 }, { 0x2B, 0x05, 0x0019 }, { 0x2B, 0x06, 0x18A6 },
	{ 0x2B, 0x07, 0x2990 }, { 0x2B, 0x08, 0xFF84 }, { 0x2B, 0x09, 0x2682 }, { 0x2B, 0x0A, 0x0000 },
	{ 0x2B, 0x0B, 0x8000 }, { 0x2B, 0x0C, 0x5024 }, { 0x2B, 0x0D, 0x0000 }, { 0x2B, 0x0E, 0x0000 },
	{ 0x2B, 0x0F, 0xA470 }, { 0x2B, 0x10, 0x8000 }, { 0x2B, 0x11, 0x0362 }
};

static sds_config dal_mango_construct_ana_3p125g_type1[] = {
	{ 0x28, 0x00, 0xF904 }, { 0x28, 0x01, 0x0200 }, { 0x28, 0x02, 0x2A20 }, { 0x28, 0x03, 0xD10D },
	{ 0x28, 0x04, 0x8000 }, { 0x28, 0x05, 0xA17E }, { 0x28, 0x06, 0xE115 }, { 0x28, 0x07, 0x000E },
	{ 0x28, 0x08, 0x0294 }, { 0x28, 0x09, 0x04E4 }, { 0x28, 0x0A, 0x7FC8 }, { 0x28, 0x0B, 0xE0E7 },
	{ 0x28, 0x0C, 0x0200 }, { 0x28, 0x0D, 0xDF80 }, { 0x28, 0x0E, 0x0000 }, { 0x28, 0x0F, 0x1FD8 },
	{ 0x28, 0x10, 0x0C3F }, { 0x28, 0x11, 0x0000 }, { 0x28, 0x12, 0x27C0 }, { 0x28, 0x13, 0x7E1D },
	{ 0x28, 0x14, 0x1300 }, { 0x28, 0x15, 0x003F }, { 0x28, 0x16, 0xBE7F }, { 0x28, 0x17, 0x0090 },
	{ 0x28, 0x18, 0x0000 }, { 0x28, 0x19, 0x407F }, { 0x28, 0x1A, 0x0000 }, { 0x28, 0x1B, 0x8000 },
	{ 0x28, 0x1C, 0x011E }, { 0x28, 0x1D, 0x0000 }, { 0x28, 0x1E, 0xC8FF }, { 0x28, 0x1F, 0x0000 },
	{ 0x29, 0x00, 0xC000 }, { 0x29, 0x01, 0xF000 }, { 0x29, 0x02, 0x6010 },
	{ 0x29, 0x12, 0x0EE7 }, { 0x29, 0x13, 0x0000 }
};

static sds_config dal_mango_construct_ana_3p125g_cmu_type1[] = {
	{ 0x2D, 0x03, 0x6410 }, { 0x2D, 0x04, 0x0000 }, { 0x2D, 0x05, 0x0019 }, { 0x2D, 0x06, 0x18A6 },
	{ 0x2D, 0x07, 0x2990 }, { 0x2D, 0x08, 0xFF84 }, { 0x2D, 0x09, 0x3082 }, { 0x2D, 0x0A, 0x0000 },
	{ 0x2D, 0x0B, 0x8000 }, { 0x2D, 0x0C, 0x6424 }, { 0x2D, 0x0D, 0x0000 }, { 0x2D, 0x0E, 0x0000 },
	{ 0x2D, 0x0F, 0xA470 }, { 0x2D, 0x10, 0x8000 }, { 0x2D, 0x11, 0x037B }
};

static sds_config dal_mango_construct_ana_2p5g_type1[] = {
	{ 0x26, 0x00, 0xF904 }, { 0x26, 0x01, 0x0200 }, { 0x26, 0x02, 0x2A20 }, { 0x26, 0x03, 0xD10D },
	{ 0x26, 0x04, 0x8000 }, { 0x26, 0x05, 0xA17E }, { 0x26, 0x06, 0xE115 }, { 0x26, 0x07, 0x000E },
	{ 0x26, 0x08, 0x0294 }, { 0x26, 0x09, 0x04E4 }, { 0x26, 0x0A, 0x7FC8 }, { 0x26, 0x0B, 0xE0E7 },
	{ 0x26, 0x0C, 0x0200 }, { 0x26, 0x0D, 0xDF80 }, { 0x26, 0x0E, 0x0000 }, { 0x26, 0x0F, 0x1FE0 },
	{ 0x26, 0x10, 0x0C3F }, { 0x26, 0x11, 0x0000 }, { 0x26, 0x12, 0x27C0 }, { 0x26, 0x13, 0x7E1D },
	{ 0x26, 0x14, 0x1300 }, { 0x26, 0x15, 0x003F }, { 0x26, 0x16, 0xBE7F }, { 0x26, 0x17, 0x0090 },
	{ 0x26, 0x18, 0x0000 }, { 0x26, 0x19, 0x407F }, { 0x26, 0x1A, 0x0000 }, { 0x26, 0x1B, 0x8000 },
	{ 0x26, 0x1C, 0x011E }, { 0x26, 0x1D, 0x0000 }, { 0x26, 0x1E, 0xC8FF }, { 0x26, 0x1F, 0x0000 },
	{ 0x27, 0x00, 0xC000 }, { 0x27, 0x01, 0xF000 }, { 0x27, 0x02, 0x6010 },
	{ 0x27, 0x12, 0x0EE7 }, { 0x27, 0x13, 0x0000 }
};

static sds_config dal_mango_construct_ana_1p25g_type1[] = {
	{ 0x24, 0x00, 0xF904 }, { 0x24, 0x01, 0x0200 }, { 0x24, 0x02, 0x2A20 }, { 0x24, 0x03, 0xD10D },
	{ 0x24, 0x04, 0x8000 }, { 0x24, 0x05, 0xA17E }, { 0x24, 0x06, 0xE115 }, { 0x24, 0x07, 0x000E },
	{ 0x24, 0x08, 0x0294 }, { 0x24, 0x09, 0x84E4 }, { 0x24, 0x0A, 0x7FC8 }, { 0x24, 0x0B, 0xE0E7 },
	{ 0x24, 0x0C, 0x0200 }, { 0x24, 0x0D, 0xDF80 }, { 0x24, 0x0E, 0x0000 }, { 0x24, 0x0F, 0x1FF0 },
	{ 0x24, 0x10, 0x0C3F }, { 0x24, 0x11, 0x0000 }, { 0x24, 0x12, 0x27C0 }, { 0x24, 0x13, 0x7E1D },
	{ 0x24, 0x14, 0x1300 }, { 0x24, 0x15, 0x003F }, { 0x24, 0x16, 0xBE7F }, { 0x24, 0x17, 0x0090 },
	{ 0x24, 0x18, 0x0000 }, { 0x24, 0x19, 0x407F }, { 0x24, 0x1A, 0x0000 }, { 0x24, 0x1B, 0x8000 },
	{ 0x24, 0x1C, 0x011E }, { 0x24, 0x1D, 0x0000 }, { 0x24, 0x1E, 0xC8FF }, { 0x24, 0x1F, 0x0000 },
	{ 0x25, 0x00, 0xC000 }, { 0x25, 0x01, 0xF000 }, { 0x25, 0x02, 0x6010 },
	{ 0x25, 0x12, 0x0EE7 }, { 0x25, 0x13, 0x0000 }
};


static u16 rtl931x_read_sds_phy(int phy_addr, int page, int phy_reg)
{
	int i;
	u32 cmd = phy_addr << 2 | page << 7 | phy_reg << 13 | 1;

	pr_debug("%s: phy_addr(SDS-ID) %d, page: 0x%08X phy_reg: 0x%08X\n", __func__, phy_addr, page, phy_reg);
	sw_w32(cmd, RTL931X_SERDES_INDRT_ACCESS_CTRL);

	for (i = 0; i < 100; i++) {
		if (!(sw_r32(RTL931X_SERDES_INDRT_ACCESS_CTRL) & 0x1))
			break;
		mdelay(1);
	}

	if (i >= 100)
		return -EIO;

	pr_debug("%s: returning %04x\n", __func__, sw_r32(RTL931X_SERDES_INDRT_DATA_CTRL));
	return sw_r32(RTL931X_SERDES_INDRT_DATA_CTRL) & 0xffff;
}


static int rtl931x_write_sds_phy(int phy_addr, int page, int phy_reg, u16 v)
{
	int i;
	u32 cmd;
	cmd = phy_addr << 2 | page << 7 | phy_reg << 13;
	sw_w32(cmd, RTL931X_SERDES_INDRT_ACCESS_CTRL);

	sw_w32(v, RTL931X_SERDES_INDRT_DATA_CTRL);
		
	cmd =  sw_r32(RTL931X_SERDES_INDRT_ACCESS_CTRL) | 0x3;
	sw_w32(cmd, RTL931X_SERDES_INDRT_ACCESS_CTRL);

	for (i = 0; i < 100; i++) {
		if (!(sw_r32(RTL931X_SERDES_INDRT_ACCESS_CTRL) & 0x1))
			break;
		mdelay(1);
	}

	if (i >= 100)
		return -EIO;

	return 0;
}

static void sds_field_read(u32 sds, u32 page, u32 reg, u32 endBit, u32 startBit, u32 *data)
{
	u32 configVal, len, mask;
	int ret;

	if (endBit < startBit)
		return;

	configVal = rtl931x_read_sds_phy(sds, page, reg);

	len = endBit - startBit + 1;

	if (32 == len)
		*data = configVal;
	else {
		mask = (1 << len) - 1;
		*data = (configVal >> startBit) & mask;
	}

	return;
}				/* end of hal_sds_field_read */

static void sds_field_write(u32 sds, u32 page, u32 reg, u32 endBit, u32 startBit, u32 data)
{
	u32 configVal, len, mask;
	int ret;

	len = endBit - startBit + 1;

	if (endBit < startBit)
		return;

	if (32 == len)
		configVal = data;
	else {
		mask = (1 << len) - 1;
		configVal = rtl931x_read_sds_phy(sds, page, reg);

		configVal &= ~(mask << startBit);
		configVal |= ((data & mask) << startBit);
	}
	rtl931x_write_sds_phy(sds, page, reg, configVal);

	return;
}				/* end of hal_sds_field_write */

#define SDS_FIELD_R(_s, _p, _r, _end, _start, _d) \
     sds_field_read(_s, _p, _r, _end, _start, _d)
#define SDS_FIELD_W(_s, _p, _r, _end, _start, _d) \
     sds_field_write(_s, _p, _r, _end,  _start, _d)

static void rtl931x_sds_rst(u32 sds)
{
	u32 asds;
	int ret;
	u32 sdsMap[] = { 0, 1, 2, 3, 6, 7, 10, 11, 14, 15, 18, 19, 22, 23 };
	asds = sdsMap[sds];
	if (sds < 2)
		return;

	rtl931x_write_sds_phy(asds, 0x2e, 0x12, 0x2740);
	rtl931x_write_sds_phy(asds, 0x2f, 0x0, 0x0);
	rtl931x_write_sds_phy(asds, 0x2f, 0x2, 0x2010);
	rtl931x_write_sds_phy(asds, 0x20, 0x0, 0xc10);

	rtl931x_write_sds_phy(asds, 0x2e, 0x12, 0x27c0);
	rtl931x_write_sds_phy(asds, 0x2f, 0x0, 0xc000);
	rtl931x_write_sds_phy(asds, 0x2f, 0x2, 0x6010);
	rtl931x_write_sds_phy(asds, 0x20, 0x0, 0xc30);

	mdelay(50);
}

static void rtl931x_sds_mii_mode_set(u32 sds, serdes_mode_t mode)
{
	u32 val;
	int ret;

	switch (mode) {
	case MII_DISABLE:
		/* serdes off */
		val = 0x1f;
		break;
	case MII_QSGMII:
		/* serdes mode QSGMII */
		val = 0x6;
		break;
	case MII_XSGMII:
		/* serdes mode XSGMII */
		val = 0x10;
		break;
		//case MII_USXGMII:
	case MII_USXGMII_10GSXGMII:
	case MII_USXGMII_10GDXGMII:
	case MII_USXGMII_10GQXGMII:
	case MII_USXGMII_5GSXGMII:
	case MII_USXGMII_5GDXGMII:
	case MII_USXGMII_2_5GSXGMII:
		val = 0xD;
		break;
	case MII_HISGMII:
		val = 0x12;
		break;
	case MII_XSMII:
		val = 0x9;
		break;
	case MII_SGMII:
		val = 0x2;
		break;
	default:
		return;
	}

	val |= (1 << 7);	// mac1g mode
	
	pr_debug("%s: RTL931X_MAC_SERDES_MODE_CTRL_ADDR(%d) 0x%08X\n", __func__, sds, sw_r32(RTL931X_MAC_SERDES_MODE_CTRL_ADDR(sds)));
	sw_w32(val, RTL931X_MAC_SERDES_MODE_CTRL_ADDR(sds));

	return;
}

#if 0
int rtl931x_10gr_symErr_get(u32 sds, rtk_sds_symErr_t * info)
{
	u32 aSds, dSds, val, val2;
	u32 evenSds, evenASds;
	int ret;
	rtk_port_t port;
	rtk_port_10gMedia_t media;
	u32 sdsMap[] = { 0, 1, 2, 3, 6, 7, 10, 11, 14, 15, 18, 19, 22, 23 };
	asds = sdsMap[sds];

	port = HWP_SDS_ID2MACID(unit, sds);

	rtl931x_10gMedia_get(unit, port, &media);

	osal_memset(info, 0, sizeof(rtk_sds_symErr_t));

	switch (media) {
	case PORT_10GMEDIA_DAC_50CM ... PORT_10GMEDIA_DAC_END:
	case PORT_10GMEDIA_FIBER_10G:
		drv_rtl9310_sds2AnaSds_get(unit, sds, &aSds);
		SDS_FIELD_R(unit, aSds, 0x5, 1, 7, 0, &val);
		info->ch[0] = val;
		info->blk_err = val;

		SDS_FIELD_R(unit, aSds, 0x5, 1, 15, 14, &val);
		info->latch_blk_lock = (val >> 1) & 0x1;
		info->latch_hiber = (val & 0x1);

		evenSds = sds - (sds % 2);
		drv_rtl9310_sds2AnaSds_get(unit, evenSds, &evenASds);
		hal_serdes_reg_set(unit, evenASds, 0x1f, 0x02, 0x35);
		SDS_FIELD_R(unit, evenASds, 0x1f, 0x14, 3, 2, &val);
		info->ber = ((val >> (sds % 2)) & 0x1);

		break;
	case PORT_10GMEDIA_FIBER_1G:
		drv_rtl9310_sds2XsgmSds_get(unit, sds, &dSds);
		SDS_FIELD_W(unit, dSds, 0x1, 24, 2, 0, 0x0);

		SDS_FIELD_R(unit, dSds, 0x1, 3, 15, 8, &val);
		SDS_FIELD_R(unit, dSds, 0x1, 2, 15, 0, &val2);
		val = (val << 16) | val2;
		info->ch[0] = val;

		SDS_FIELD_W(unit, dSds, 0x1, 3, 15, 8, 0x0);
		SDS_FIELD_W(unit, dSds, 0x1, 2, 15, 0, 0x0);
		break;
	default:
		return ret;
	}

	return ret;
}				/* end of _phy_rtl9310_10gr_symErr_get */
#endif
static void rtl931x_symerr_clear(u32 sds, serdes_mode_t mode)
{
	u32 i;
	u32 xsg_sdsid_0, xsg_sdsid_1;
	int ret;

	/* function body */
	switch (mode) {
	case MII_DISABLE:
		break;
	case MII_XSGMII:
		if (sds < 2)
			xsg_sdsid_0 = sds;
		else
			xsg_sdsid_0 = (sds - 1) * 2;
		xsg_sdsid_1 = xsg_sdsid_0 + 1;

		for (i = 0; i < 4; ++i) {
			SDS_FIELD_W(xsg_sdsid_0, 0x1, 24, 2, 0, i);
			SDS_FIELD_W(xsg_sdsid_0, 0x1, 3, 15, 8, 0x0);
			SDS_FIELD_W(xsg_sdsid_0, 0x1, 2, 15, 0, 0x0);
		}

		for (i = 0; i < 4; ++i) {
			SDS_FIELD_W(xsg_sdsid_1, 0x1, 24, 2, 0, i);
			SDS_FIELD_W(xsg_sdsid_1, 0x1, 3, 15, 8, 0x0);
			SDS_FIELD_W(xsg_sdsid_1, 0x1, 2, 15, 0, 0x0);
		}

		SDS_FIELD_W(xsg_sdsid_0, 0x1, 0, 15, 0, 0x0);
		SDS_FIELD_W(xsg_sdsid_0, 0x1, 1, 15, 8, 0x0);
		SDS_FIELD_W(xsg_sdsid_1, 0x1, 0, 15, 0, 0x0);
		SDS_FIELD_W(xsg_sdsid_1, 0x1, 1, 15, 8, 0x0);
		break;
	default:
		//        _phy_rtl9310_10gr_symErr_get(sds, &info);
		break;
	}

	return;
}

static void rtl931x_sds_fiber_mode_set(u32 sds, serdes_mode_t mode)
{
	u32 val, asds;
	int ret;
	u32 sdsMap[] = { 0, 1, 2, 3, 6, 7, 10, 11, 14, 15, 18, 19, 22, 23 };
	asds = sdsMap[sds];

	/* clear symbol error count before change mode */
	rtl931x_symerr_clear(sds, mode);

	val = 0x9F;
	sw_w32(val, RTL931X_MAC_SERDES_MODE_CTRL_ADDR(sds));

	switch (mode) {
	case MII_SGMII:
		val = 0x5;
		break;
	case MII_1000BX_FIBER:
		/* serdes mode FIBER1G */
		val = 0x9;
		break;
	case MII_10GR:
		/* serdes mode 10G Base-R */
		val = 0x35;
		break;
	case MII_10GR1000BX_AUTO:
		val = 0x39;
		break;
	case MII_DISABLE:
		/* serdes off */
		val = 0x3F;
		break;
	case MII_USXGMII_10GSXGMII:
	case MII_USXGMII_10GDXGMII:
	case MII_USXGMII_10GQXGMII:
	case MII_USXGMII_5GSXGMII:
	case MII_USXGMII_5GDXGMII:
	case MII_USXGMII_2_5GSXGMII:
		val = 0x1B;
		break;
	default:
		return;
	}

	SDS_FIELD_W(asds, 0x1F, 0x9, 11, 6, val);

	return;
}

static void cmuType_set(u32 aSds, serdes_mode_t mode, int chiptype)
{
	int cmuType = 0;
	u32 cmuPage = 0;
	u32 frc_cmu_spd;
	u32 evenSds;
	u32 lane, frc_lc_mode_bitnum, frc_lc_mode_val_bitnum;
	int ret;

	switch (mode) {
	case MII_DISABLE:
	case MII_10GR:
	case MII_XSGMII:
		//case MII_USXGMII:
	case MII_USXGMII_10GSXGMII:
	case MII_USXGMII_10GDXGMII:
	case MII_USXGMII_10GQXGMII:
	case MII_USXGMII_5GSXGMII:
	case MII_USXGMII_5GDXGMII:
	case MII_USXGMII_2_5GSXGMII:
		return;
	case MII_10GR1000BX_AUTO:
		if (chiptype) {
			SDS_FIELD_W(aSds, 0x24, 0xd, 14, 14, 0);
		}
		return;
	case MII_QSGMII:
		cmuType = 1;
		cmuPage = 0x2a;
		frc_cmu_spd = 0;
		break;
	case MII_HISGMII:
		cmuType = 1;
		cmuPage = 0x28;
		frc_cmu_spd = 1;
		break;
	case MII_XSMII:
		cmuType = 1;
		cmuPage = 0x26;
		frc_cmu_spd = 0;
		break;
	case MII_1000BX_FIBER:
		cmuType = 1;
		cmuPage = 0x24;
		frc_cmu_spd = 0;
		break;
	case MII_100BX_FIBER:
		cmuType = 1;
		cmuPage = 0x24;
		frc_cmu_spd = 0;
		break;
	case MII_1000BX100BX_AUTO:
		cmuType = 1;
		cmuPage = 0x24;
		frc_cmu_spd = 0;
		break;
	case MII_SGMII:
		cmuType = 1;
		cmuPage = 0x24;
		frc_cmu_spd = 0;
		break;
	case MII_2500Base_X:
		cmuType = 1;
		cmuPage = 0x28;
		frc_cmu_spd = 1;
		break;
	default:
		pr_info("SerDes %d mode is invalid\n", aSds);
		return;
	}

	lane = aSds % 2;

	if (0 == lane) {
		frc_lc_mode_bitnum = 4;
		frc_lc_mode_val_bitnum = 5;
	} else {
		frc_lc_mode_bitnum = 6;
		frc_lc_mode_val_bitnum = 7;
	}

	evenSds = aSds - lane;
	if (cmuType == 1) {
		SDS_FIELD_W(aSds, cmuPage, 0x7, 15, 15, 0);
		if (chiptype) {
			SDS_FIELD_W(aSds, cmuPage, 0xd, 14, 14, 0);
		}

		SDS_FIELD_W(evenSds, 0x20, 0x12, 3, 2, 0x3);
		SDS_FIELD_W(evenSds, 0x20, 0x12, frc_lc_mode_bitnum, frc_lc_mode_bitnum, 1);
		SDS_FIELD_W(evenSds, 0x20, 0x12, frc_lc_mode_val_bitnum, frc_lc_mode_val_bitnum, 0);
		SDS_FIELD_W(evenSds, 0x20, 0x12, 12, 12, 1);
		SDS_FIELD_W(evenSds, 0x20, 0x12, 15, 13, frc_cmu_spd);
	} else if (cmuType == 2)	// not used
	{
		SDS_FIELD_W(aSds, cmuPage, 0x7, 15, 15, 1);
		if (chiptype) {
			SDS_FIELD_W(aSds, cmuPage, 0xd, 14, 14, 1);
		}

		SDS_FIELD_W(evenSds, 0x20, 0x12, 1, 0, 0x3);
		SDS_FIELD_W(evenSds, 0x20, 0x12, frc_lc_mode_bitnum, frc_lc_mode_bitnum, 1);
		SDS_FIELD_W(evenSds, 0x20, 0x12, frc_lc_mode_val_bitnum, frc_lc_mode_val_bitnum, 1);
		SDS_FIELD_W(evenSds, 0x20, 0x12, 8, 8, 1);
		SDS_FIELD_W(evenSds, 0x20, 0x12, 11, 9, frc_cmu_spd);
	}

	return;
}

void rtl931x_sds_init(u32 sds, enum serdes_mode mode)
{

	u32 board_sds_tx_type1[] = { 0x1C3, 0x1C3, 0x1C3, 0x1A3, 0x1A3,
		0x1A3, 0x143, 0x143, 0x143, 0x143, 0x163, 0x163
	};

	u32 board_sds_tx[] = { 0x1A00, 0x1A00, 0x200, 0x200, 0x200,
		0x200, 0x1A3, 0x1A3, 0x1A3, 0x1A3, 0x1E3, 0x1E3
	};

	u32 board_sds_tx2[] = { 0xDC0, 0x1C0, 0x200, 0x180, 0x160,
		0x123, 0x123, 0x163, 0x1A3, 0x1A0, 0x1C3, 0x9C3
	};

	u32 sdsMap[] = { 0, 1, 2, 3, 6, 7, 10, 11, 14, 15, 18, 19, 22, 23 };
	u32 aSds, dSds, val, ori, model_info;
	aSds = sdsMap[sds];
	int ret;
	int chiptype = 0;
	pr_debug("%s: set sds %d to mode %d\n", __func__, sds, mode);
	pr_debug("%s: fibermode %08X", __func__, rtl931x_read_sds_phy(aSds, 0x1f, 0x9));
	pr_debug("%s: serdes_mode_ctrl %08X", __func__, RTL931X_MAC_SERDES_MODE_CTRL_ADDR(2));
	if (14 <= sds)
		return;

	model_info = sw_r32(RTL931X_MODEL_NAME_INFO_ADDR);
	if ((model_info >> 4) & 0x1) {
		pr_info("detected chiptype 1\n");
		chiptype = 1;
	} else {
		pr_info("detected chiptype 0\n");
	}

	if (sds < 2)
		dSds = sds;
	else
		dSds = (sds - 1) * 2;

	pr_debug("%s: RTL931X_PS_SERDES_OFF_MODE_CTRL_ADDR 0x%08X\n", __func__, sw_r32(RTL931X_PS_SERDES_OFF_MODE_CTRL_ADDR));
	ori = sw_r32(RTL931X_PS_SERDES_OFF_MODE_CTRL_ADDR);
	val = ori | (1 << sds);
	sw_w32(val, RTL931X_PS_SERDES_OFF_MODE_CTRL_ADDR);

	switch (mode) {
	case MII_DISABLE:
		break;
	case MII_XSGMII:

		if (chiptype) {
			u32 xsg_sdsid_1;
			xsg_sdsid_1 = dSds + 1;
			//fifo inv clk
			SDS_FIELD_W(dSds, 0x1, 0x1, 7, 4, 0xf);
			SDS_FIELD_W(dSds, 0x1, 0x1, 3, 0, 0xf);

			SDS_FIELD_W(xsg_sdsid_1, 0x1, 0x1, 7, 4, 0xf);
			SDS_FIELD_W(xsg_sdsid_1, 0x1, 0x1, 3, 0, 0xf);

		}

		SDS_FIELD_W(dSds, 0x0, 0xE, 12, 12, 1);
		SDS_FIELD_W(dSds + 1, 0x0, 0xE, 12, 12, 1);
		break;
	case MII_USXGMII_10GSXGMII:
	case MII_USXGMII_10GDXGMII:
	case MII_USXGMII_10GQXGMII:
		u32 i, evenSds;
		int ret;
		u32 op_code = 0x6003;

		if (chiptype) {
			SDS_FIELD_W(aSds, 0x6, 0x2, 12, 12, 1);

			for (i = 0; i < sizeof(dal_mango_construct_ana_10p3125g_type1) / sizeof(sds_config); ++i) {
				rtl931x_write_sds_phy(aSds, dal_mango_construct_ana_10p3125g_type1[i].page - 0x4, dal_mango_construct_ana_10p3125g_type1[i].reg, dal_mango_construct_ana_10p3125g_type1[i].data);
			}

			evenSds = aSds - (aSds % 2);

			for (i = 0; i < sizeof(dal_mango_construct_ana_10p3125g_cmu_type1) / sizeof(sds_config); ++i) {
				rtl931x_write_sds_phy(evenSds,
						      dal_mango_construct_ana_10p3125g_cmu_type1[i].page - 0x4, dal_mango_construct_ana_10p3125g_cmu_type1[i].reg, dal_mango_construct_ana_10p3125g_cmu_type1[i].data);
			}

			SDS_FIELD_W(aSds, 0x6, 0x2, 12, 12, 0);
		} else {

			SDS_FIELD_W(aSds, 0x2e, 0xd, 6, 0, 0x0);
			SDS_FIELD_W(aSds, 0x2e, 0xd, 7, 7, 0x1);

			SDS_FIELD_W(aSds, 0x2e, 0x1c, 5, 0, 0x1E);
			SDS_FIELD_W(aSds, 0x2e, 0x1d, 11, 0, 0x00);
			SDS_FIELD_W(aSds, 0x2e, 0x1f, 11, 0, 0x00);
			SDS_FIELD_W(aSds, 0x2f, 0x0, 11, 0, 0x00);
			SDS_FIELD_W(aSds, 0x2f, 0x1, 11, 0, 0x00);

			SDS_FIELD_W(aSds, 0x2e, 0xf, 12, 6, 0x7F);
			rtl931x_write_sds_phy(aSds, 0x2f, 0x12, 0xaaa);

			rtl931x_sds_rst(sds);

			rtl931x_write_sds_phy(aSds, 0x7, 0x10, op_code);
			rtl931x_write_sds_phy(aSds, 0x6, 0x1d, 0x0480);
			rtl931x_write_sds_phy(aSds, 0x6, 0xe, 0x0400);
		}
		break;
	case MII_10GR:
	case MII_10GR1000BX_AUTO:
		//configure 10GR fiber mode=1
		SDS_FIELD_W(aSds, 0x1f, 0xb, 1, 1, 1);

		//init fiber_1g
		SDS_FIELD_W(dSds, 0x3, 0x13, 15, 14, 0);

		SDS_FIELD_W(dSds, 0x2, 0x0, 12, 12, 1);
		SDS_FIELD_W(dSds, 0x2, 0x0, 6, 6, 1);
		SDS_FIELD_W(dSds, 0x2, 0x0, 13, 13, 0);

		//init auto
		SDS_FIELD_W(aSds, 0x1f, 13, 15, 0, 0x109e);
		SDS_FIELD_W(aSds, 0x1f, 0x6, 14, 10, 0x8);
		SDS_FIELD_W(aSds, 0x1f, 0x7, 10, 4, 0x7f);
		break;
	case MII_QSGMII:
		// unsupported
		break;
	case MII_HISGMII:
		SDS_FIELD_W(dSds, 0x1, 0x14, 8, 8, 1);
		break;
	case MII_XSMII:
		// unsupported
		break;
	case MII_1000BX_FIBER:
		SDS_FIELD_W(dSds, 0x3, 0x13, 15, 14, 0);

		SDS_FIELD_W(dSds, 0x2, 0x0, 12, 12, 1);
		SDS_FIELD_W(dSds, 0x2, 0x0, 6, 6, 1);
		SDS_FIELD_W(dSds, 0x2, 0x0, 13, 13, 0);
		break;
	case MII_100BX_FIBER:
		SDS_FIELD_W(dSds, 0x3, 0x13, 15, 14, 0);

		SDS_FIELD_W(dSds, 0x2, 0x0, 12, 12, 0);
		SDS_FIELD_W(dSds, 0x2, 0x0, 6, 6, 0);
		SDS_FIELD_W(dSds, 0x2, 0x0, 13, 13, 1);
		break;
	case MII_1000BX100BX_AUTO:
		//same as 1000BX_FIBER
		SDS_FIELD_W(dSds, 0x3, 0x13, 15, 14, 0);

		SDS_FIELD_W(dSds, 0x2, 0x0, 12, 12, 1);
		SDS_FIELD_W(dSds, 0x2, 0x0, 6, 6, 1);
		SDS_FIELD_W(dSds, 0x2, 0x0, 13, 13, 0);
		break;
	case MII_SGMII:
		SDS_FIELD_W(sds, 0x24, 0x9, 15, 15, 0);
		break;
	case MII_2500Base_X:
		SDS_FIELD_W(dSds, 0x1, 0x14, 8, 8, 1);
		break;
	default:
		pr_info("SerDes %d mode is invalid\n", sds);
		return;
	}

	cmuType_set(aSds, mode, chiptype);

	if (sds >= 2 && sds <= 13) {
		if (chiptype)
			rtl931x_write_sds_phy(aSds, 0x2E, 0x1, board_sds_tx_type1[sds - 2]);
		else {
			val = 0xa0000;
			sw_w32(val, RTL931X_CHIP_INFO_ADDR);
			val = sw_r32(RTL931X_CHIP_INFO_ADDR);
			if (1 == (val >> 28))	// consider 9311 etc. RTL9313_CHIP_ID == HWP_CHIP_ID(unit))
			{
				rtl931x_write_sds_phy(aSds, 0x2E, 0x1, board_sds_tx2[sds - 2]);
			} else {
				rtl931x_write_sds_phy(aSds, 0x2E, 0x1, board_sds_tx[sds - 2]);
			}
			val = 0;
			sw_w32(val, RTL931X_CHIP_INFO_ADDR);
		}
	}

	val = ori & ~(1 << sds);
	sw_w32(val, RTL931X_PS_SERDES_OFF_MODE_CTRL_ADDR);
	pr_debug("%s: RTL931X_PS_SERDES_OFF_MODE_CTRL_ADDR 0x%08X\n", __func__, sw_r32(RTL931X_PS_SERDES_OFF_MODE_CTRL_ADDR));

	switch (mode) {
	case MII_XSGMII:
	case MII_QSGMII:
	case MII_HISGMII:
	case MII_XSMII:
	case MII_SGMII:
	case MII_USXGMII_10GSXGMII:
	case MII_USXGMII_10GDXGMII:
	case MII_USXGMII_10GQXGMII:
		if (mode == MII_XSGMII)
			rtl931x_sds_mii_mode_set(sds, mode);
		else
			rtl931x_sds_fiber_mode_set(sds, mode);

		break;
	case MII_100BX_FIBER:
		SDS_FIELD_W(dSds, 0x2, 0x0, 12, 12, 0);
		SDS_FIELD_W(dSds, 0x2, 0x0, 6, 6, 0);
		SDS_FIELD_W(dSds, 0x2, 0x0, 13, 13, 1);
		break;
	default:
		break;
	}

	return;
}				/* end of _dal_mango_construct_sdsMode_set */

void rtl931x_sw_init(struct rtl838x_switch_priv *priv)
{
//	rtl931x_sds_init(priv);
}

int rtl931x_l3_setup(struct rtl838x_switch_priv *priv)
{
	return 0;
}

void rtl931x_vlan_port_pvidmode_set(int port, enum pbvlan_type type, enum pbvlan_mode mode) {
	if (type == PBVLAN_TYPE_INNER)
		sw_w32_mask(0x3 << 12, mode << 12, RTL931X_VLAN_PORT_IGR_CTRL + (port << 2));
	else
		sw_w32_mask(0x3 << 26, mode << 26, RTL931X_VLAN_PORT_IGR_CTRL + (port << 2));

}

void rtl931x_vlan_port_pvid_set(int port, enum pbvlan_type type, int pvid) {
	if (type == PBVLAN_TYPE_INNER)
		sw_w32_mask(0xfff, pvid, RTL931X_VLAN_PORT_IGR_CTRL + (port << 2));
	else
		sw_w32_mask(0xfff << 14, pvid << 14, RTL931X_VLAN_PORT_IGR_CTRL + (port << 2));

}

static int rtl931x_set_ageing_time(unsigned long msec)
{
	int t = sw_r32(RTL931X_L2_AGE_CTRL);

	t &= 0x1FFFFF;
	t = (t * 8) / 10;
	pr_debug("L2 AGING time: %d sec\n", t);

	t = (msec / 100 + 7) / 8;
	t = t > 0x1FFFFF ? 0x1FFFFF : t;
	sw_w32_mask(0x1FFFFF, t, RTL931X_L2_AGE_CTRL);
	pr_debug("Dynamic aging for ports: %x\n", sw_r32(RTL931X_L2_PORT_AGE_CTRL));
	return 0;
}

static void rtl931x_set_igr_filter(int port, enum igr_filter state)
{
	sw_w32_mask(0x3 << ((port & 0xf)<<1), state << ((port & 0xf)<<1), RTL931X_VLAN_PORT_IGR_FLTR + (((port >> 4) << 2)));
}

static void rtl931x_set_egr_filter(int port,  enum egr_filter state)
{
	sw_w32_mask(0x1 << (port % 0x20), state << (port % 0x20), RTL931X_VLAN_PORT_EGR_FLTR + (((port >> 5) << 2)));
}


const struct rtl838x_reg rtl931x_reg = {
	.mask_port_reg_be = rtl839x_mask_port_reg_be,
	.set_port_reg_be = rtl839x_set_port_reg_be,
	.get_port_reg_be = rtl839x_get_port_reg_be,
	.mask_port_reg_le = rtl839x_mask_port_reg_le,
	.set_port_reg_le = rtl839x_set_port_reg_le,
	.get_port_reg_le = rtl839x_get_port_reg_le,
	.stat_port_rst = RTL931X_STAT_PORT_RST,
	.stat_rst = RTL931X_STAT_RST,
	.stat_port_std_mib = 0,  // Not defined
	.traffic_enable = rtl931x_traffic_enable,
	.traffic_disable = rtl931x_traffic_disable,
	.traffic_get = rtl931x_traffic_get,
	.traffic_set = rtl931x_traffic_set,
	.l2_ctrl_0 = RTL931X_L2_CTRL,
	.set_ageing_time = rtl931x_set_ageing_time,
	.smi_poll_ctrl = RTL931X_SMI_PORT_POLLING_CTRL,
	.l2_tbl_flush_ctrl = RTL931X_L2_TBL_FLUSH_CTRL,
	.exec_tbl0_cmd = rtl931x_exec_tbl0_cmd,
	.exec_tbl1_cmd = rtl931x_exec_tbl1_cmd,
	.tbl_access_data_0 = rtl931x_tbl_access_data_0,
	.isr_glb_src = RTL931X_ISR_GLB_SRC,
	.isr_port_link_sts_chg = RTL931X_ISR_PORT_LINK_STS_CHG,
	.imr_port_link_sts_chg = RTL931X_IMR_PORT_LINK_STS_CHG,
	// imr_glb does not exist on RTL931X
	.vlan_tables_read = rtl931x_vlan_tables_read,
	.vlan_set_tagged = rtl931x_vlan_set_tagged,
	.vlan_set_untagged = rtl931x_vlan_set_untagged,
	.vlan_profile_dump = rtl931x_vlan_profile_dump,
	.vlan_profile_setup = rtl931x_vlan_profile_setup,
	.vlan_fwd_on_inner = rtl931x_vlan_fwd_on_inner,
	.l3_setup = rtl931x_l3_setup,

	.stp_get = rtl931x_stp_get,
	.stp_set = rtl931x_stp_set,
	.mac_force_mode_ctrl = rtl931x_mac_force_mode_ctrl,
	.mac_port_ctrl = rtl931x_mac_port_ctrl,
	.l2_port_new_salrn = rtl931x_l2_port_new_salrn,
	.l2_port_new_sa_fwd = rtl931x_l2_port_new_sa_fwd,
	.mir_ctrl = RTL931X_MIR_CTRL,
	.mir_dpm = RTL931X_MIR_DPM_CTRL,
	.mir_spm = RTL931X_MIR_SPM_CTRL,
	.mac_link_sts = RTL931X_MAC_LINK_STS,
	.mac_link_dup_sts = RTL931X_MAC_LINK_DUP_STS,
	.mac_link_spd_sts = rtl931x_mac_link_spd_sts,
	.mac_rx_pause_sts = RTL931X_MAC_RX_PAUSE_STS,
	.mac_tx_pause_sts = RTL931X_MAC_TX_PAUSE_STS,
	.read_l2_entry_using_hash = rtl931x_read_l2_entry_using_hash,
	.read_cam = rtl931x_read_cam,
	.write_l2_entry_using_hash = rtl931x_write_l2_entry_using_hash,
	.write_cam = rtl931x_write_cam,
	.read_mcast_pmask = rtl931x_read_mcast_pmask,
	.write_mcast_pmask = rtl931x_write_mcast_pmask,
	.l2_hash_seed = rtl931x_l2_hash_seed,
	.l2_hash_key = rtl931x_l2_hash_key,
	.vlan_port_pvidmode_set = rtl931x_vlan_port_pvidmode_set,
	.vlan_port_pvid_set = rtl931x_vlan_port_pvid_set,
	.vlan_port_tag_sts_ctrl = RTL931X_VLAN_PORT_TAG_CTRL,
	.trk_mbr_ctr = rtl931x_trk_mbr_ctr,
	.rma_bpdu_ctrl = RTL931X_RMA_BPDU_CTRL,
	.rma_ptp_ctrl = RTL931X_RMA_PTP_CTRL,
	.rma_lltp_ctrl = RTL931X_RMA_LLTP_CTRL,
	.rma_eapol_ctrl = RTL931X_RMA_EAPOL_CTRL,
	.rma_bpdu_ctrl_div = 10,
	.rma_ptp_ctrl_div = 1,
	.rma_lltp_ctrl_div = 10,
	.rma_eapol_ctrl_div = 10,
	.storm_ctrl_port_uc = RTL931X_STORM_CTRL_PORT_UC_0(0),
	.storm_ctrl_port_bc = RTL931X_STORM_CTRL_PORT_BC_0(0),
	.storm_ctrl_port_mc = RTL931X_STORM_CTRL_PORT_MC_0(0),
	.storm_ctrl_port_uc_shift = 3,
	.storm_ctrl_port_bc_shift = 3,
	.storm_ctrl_port_mc_shift = 3,
	.vlan_ctrl = RTL931X_VLAN_CTRL,
	.sflow_ctrl = RTL931X_SFLOW_CTRL,
	.sflow_port_rate_ctrl = RTL931X_SFLOW_PORT_RATE_CTRL,
	.trk_hash_ctrl = RTL931X_TRK_HASH_CTRL,
//	.trk_hash_idx_ctrl = RTL931X_TRK_HASH_IDX_CTRL,
	.set_distribution_algorithm = rtl931x_set_distribution_algorithm,
	.set_receive_management_action = rtl931x_set_receive_management_action,
	.pie_init = rtl931x_pie_init,
	.set_vlan_igr_filter = rtl931x_set_igr_filter,
	.set_vlan_egr_filter = rtl931x_set_egr_filter,
	.sw_init = rtl931x_sw_init,
};

