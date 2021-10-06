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
	profile[2] = (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 16) & 0xFFFFFFFFULL) << 32
		| (sw_r32(RTL931X_VLAN_PROFILE_SET(index) + 12) & 0x1FFFFFFULL);
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
		| 2 << 15 /* Table type 0b10 */
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
	struct table_reg *r = rtl_table_get(RTL9310_TBL_0, 3);

	rtl_table_read(r, vlan);
	v = sw_r32(rtl_table_data(r, 0));
	w = sw_r32(rtl_table_data(r, 1));
	x = sw_r32(rtl_table_data(r, 2));
	y = sw_r32(rtl_table_data(r, 3));
	pr_debug("VLAN_READ %d: %08x %08x\n", vlan, v, w);
	rtl_table_release(r);

	info->tagged_ports = ((u64) v) << 25 | (w >> 7);
	info->profile_id = (x >> 16) & 0xf;
	info->hash_mc_fid = !!(x & BIT(30));
	info->hash_uc_fid = !!(x & BIT(31));
	info->fid = w & 0x7f;
	// TODO: use also info in 4th register

	// Read UNTAG table via table register 3
	r = rtl_table_get(RTL9310_TBL_3, 0);
	rtl_table_read(r, vlan);
	v = ((u64)sw_r32(rtl_table_data(r, 0))) << 25;
	v |= sw_r32(rtl_table_data(r, 1)) >> 7;
	rtl_table_release(r);

	info->untagged_ports = v;
}

static void rtl931x_vlan_set_tagged(u32 vlan, struct rtl838x_vlan_info *info)
{
	u32 v, w, x;
	// Access VLAN table (1) via register 0
	struct table_reg *r = rtl_table_get(RTL9310_TBL_0, 3);

	v = info->tagged_ports << 7;
	w = (info->tagged_ports & 0x7f000000) << 25;
	w |= (u32)info->fid;
	x = info->profile_id << 16;
	w |= info->hash_mc_fid ? BIT(30) : 0;
	w |= info->hash_uc_fid ? BIT(31) : 0;
	// TODO: use also info in 4th register

	sw_w32(v, rtl_table_data(r, 0));
	sw_w32(w, rtl_table_data(r, 1));
	sw_w32(x, rtl_table_data(r, 2));

	rtl_table_write(r, vlan);
	rtl_table_release(r);
}

static void rtl931x_vlan_set_untagged(u32 vlan, u64 portmask)
{
	struct table_reg *r = rtl_table_get(RTL9310_TBL_3, 0);

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
	return RTL931X_MAC_LINK_SPD_STS(p);
}

static inline int rtl931x_mac_port_ctrl(int p)
{
	return RTL930X_MAC_L2_PORT_CTRL(p);
}

static inline int rtl931x_l2_port_new_salrn(int p)
{
	return RTL931X_L2_PORT_NEW_SALRN(p);
}

static inline int rtl931x_l2_port_new_sa_fwd(int p)
{
	return RTL931X_L2_PORT_NEW_SA_FWD(p);
}

#define RTL931X_DMA_IF_CTRL                     (0x0928)
#define RTL931X_DMA_IF_INTR_RX_RUNOUT_STS       (0x091c)
#define RTL931X_DMA_IF_INTR_RX_DONE_STS         (0x0920)
#define RTL931X_DMA_IF_INTR_TX_DONE_STS         (0x0924)
#define RTL931X_DMA_IF_INTR_RX_RUNOUT_MSK       (0x0910)
#define RTL931X_DMA_IF_INTR_RX_DONE_MSK         (0x0914)
#define RTL931X_DMA_IF_INTR_TX_DONE_MSK         (0x0918)
#define RTL931X_L2_NTFY_IF_INTR_MSK             (0x09E4)
#define RTL931X_L2_NTFY_IF_INTR_STS             (0x09E8)

irqreturn_t rtl931x_switch_irq(int irq, void *dev_id)
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
			// Must re-read this to get correct status
			link = rtl839x_get_port_reg_le(RTL931X_MAC_LINK_STS);
			if (link & BIT_ULL(i))
				dsa_port_phylink_mac_change(ds, i, true);
			else
				dsa_port_phylink_mac_change(ds, i, false);
		}
	}
	return IRQ_HANDLED;
}

int rtl931x_write_phy(u32 port, u32 page, u32 reg, u32 val)
{
	u32 v;
	int err = 0;

	val &= 0xffff;
	if (port > 63 || page > 4095 || reg > 31)
		return -ENOTSUPP;

	mutex_lock(&smi_lock);
	pr_info("%s: writing to phy %d %d %d %d\n", __func__, port, page, reg, val);
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
	struct table_reg *r = rtl_table_get(RTL9310_TBL_0, 6);

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
	struct table_reg *r = rtl_table_get(RTL9310_TBL_0, 6);

	sw_w32((dest_matrix << 3), rtl_table_data(r, 0));
	rtl_table_write(r, source);
	rtl_table_release(r);
}

void rtl931x_traffic_enable(int source, int dest)
{
	struct table_reg *r = rtl_table_get(RTL9310_TBL_0, 6);
	rtl_table_read(r, source);
	sw_w32_mask(0, BIT(dest + 3), rtl_table_data(r, 0));
	rtl_table_write(r, source);
	rtl_table_release(r);
}

void rtl931x_traffic_disable(int source, int dest)
{
	struct table_reg *r = rtl_table_get(RTL9310_TBL_0, 6);
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
	if (sw_r32(RTL931X_L2_CTRL) & BIT(0))
		h = k1;
	else
		h = k0;

	/* Algorithm choice for block 1
	 * Since k0 and k1 are < 2048, adding 2048 will offset the hash into the second
	 * half of hash-space
	 * 2048 is in fact the hash-table size 16384 divided by 4 hashes per bucket
	 * divided by 2 to divide the hash space in 2
	 */
	if (sw_r32(RTL931X_L2_CTRL) & BIT(1))
		h |= (k1 + 2048) << 16;
	else
		h |= (k0 + 2048) << 16;

	return h;
}

/*
 * Fills an L2 entry structure from the SoC registers
 */
static void rtl931x_fill_l2_entry(u32 r[], struct rtl838x_l2_entry *e)
{
	pr_debug("In %s valid?\n", __func__);
	e->valid = !!(r[2] & BIT(31));
	if (!e->valid)
		return;

	pr_debug("In %s is valid\n", __func__);
	e->is_ip_mc = false;
	e->is_ipv6_mc = false;

	// TODO: Is there not a function to copy directly MAC memory?
	e->mac[0] = (r[0] >> 24);
	e->mac[1] = (r[0] >> 16);
	e->mac[2] = (r[0] >> 8);
	e->mac[3] = r[0];
	e->mac[4] = (r[1] >> 24);
	e->mac[5] = (r[1] >> 16);

	e->next_hop = !!(r[2] & BIT(12));
	e->rvid = r[1] & 0xfff;

	/* Is it a unicast entry? check multicast bit */
	if (!(e->mac[0] & 1)) {
		e->type = L2_UNICAST;
		e->is_static = !!(r[2] & BIT(14));
		e->port = (r[2] >> 20) & 0x3ff;
		// Check for trunk port
		if (r[2] & BIT(30)) {
			e->is_trunk = true;
			e->stack_dev = (e->port >> 9) & 1;
			e->trunk = e->port & 0x3f;
		} else {
			e->is_trunk = false;
			e->stack_dev = (e->port >> 6) & 0xf;
			e->port = e->port & 0x3f;
		}

		e->block_da = !!(r[2] & BIT(15));
		e->block_sa = !!(r[2] & BIT(16));
		e->suspended = !!(r[2] & BIT(13));
		e->age = (r[2] >> 17) & 3;
		e->valid = true;
		// the UC_VID field in hardware is used for the VID or for the route id
		if (e->next_hop) {
			e->nh_route_id = r[2] & 0x7ff;
			e->vid = 0;
		} else {
			e->vid = r[2] & 0xfff;
			e->nh_route_id = 0;
		}
	} else {
		e->valid = true;
		e->type = L2_MULTICAST;
		e->mc_portmask_index = (r[2] >> 16) & 0x3ff;
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
	return 0;
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
	pr_info("Leaving %s\n", __func__);
}

static u64 rtl931x_read_mcast_pmask(int idx)
{
	return 0;
}

static void rtl931x_write_mcast_pmask(int idx, u64 portmask)
{
}
static void rtl931x_pie_init(struct rtl838x_switch_priv *priv)
{
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

static void rtl931x_set_igr_filter(int port, int state)
{
	sw_w32_mask(0x3 << ((port & 0xf)<<1), state << ((port & 0xf)<<1), RTL931X_VLAN_PORT_IGR_FLTR + (((port >> 4) << 2)));
}

static void rtl931x_set_egr_filter(int port, int state)
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
};

