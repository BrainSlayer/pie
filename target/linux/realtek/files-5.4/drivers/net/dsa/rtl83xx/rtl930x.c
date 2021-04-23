// SPDX-License-Identifier: GPL-2.0-only

#include <asm/mach-rtl838x/mach-rtl83xx.h>
#include <linux/bitops.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <net/nexthop.h>
#include <net/neighbour.h>
#include <net/arp.h>

#include "rtl83xx.h"

extern struct mutex smi_lock;
extern struct rtl83xx_soc_info soc_info;

void rtl930x_print_matrix(void)
{//
	int i;
	struct table_reg *r = rtl_table_get(RTL9300_TBL_0, 6);

	for (i = 0; i < 29; i++) {
		rtl_table_read(r, i);
		pr_info("> %08x\n", sw_r32(rtl_table_data(r, 0)));
	}
	rtl_table_release(r);
}

inline void rtl930x_exec_tbl0_cmd(u32 cmd)
{
	sw_w32(cmd, RTL930X_TBL_ACCESS_CTRL_0);
	do { } while (sw_r32(RTL930X_TBL_ACCESS_CTRL_0) & (1 << 17));
}

inline void rtl930x_exec_tbl1_cmd(u32 cmd)
{
	sw_w32(cmd, RTL930X_TBL_ACCESS_CTRL_1);
	do { } while (sw_r32(RTL930X_TBL_ACCESS_CTRL_1) & (1 << 17));
}

inline int rtl930x_tbl_access_data_0(int i)
{
	return RTL930X_TBL_ACCESS_DATA_0(i);
}

static inline int rtl930x_l2_port_new_salrn(int p)
{
	return RTL930X_L2_PORT_SALRN(p);
}

static inline int rtl930x_l2_port_new_sa_fwd(int p)
{
	// TODO: The definition of the fields changed, because of the master-cpu in a stack
	return RTL930X_L2_PORT_NEW_SA_FWD(p);
}

inline static int rtl930x_trk_mbr_ctr(int group)
{
	return RTL930X_TRK_MBR_CTRL + (group << 2);
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

	pr_debug("VLAN %d: L2 learning: %d, L2 Unknown MultiCast Field %x, \
		IPv4 Unknown MultiCast Field %x, IPv6 Unknown MultiCast Field: %x",
		index, profile[0] & (3 << 21), profile[2], profile[3], profile[4]);
}

static void rtl930x_vlan_set_untagged(u32 vlan, u64 portmask)
{
	struct table_reg *r = rtl_table_get(RTL9300_TBL_2, 0);

	sw_w32(portmask << 3, rtl_table_data(r, 0));
	rtl_table_write(r, vlan);
	rtl_table_release(r);
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

static inline int rtl930x_mac_force_mode_ctrl(int p)
{
	return RTL930X_MAC_FORCE_MODE_CTRL + (p << 2);
}

static inline int rtl930x_mac_port_ctrl(int p)
{
	return RTL930X_MAC_L2_PORT_CTRL(p);
}

static inline int rtl930x_mac_link_spd_sts(int p)
{
	return RTL930X_MAC_LINK_SPD_STS(p);
}
static u64 rtl930x_l2_hash_seed(u64 mac, u32 vid)
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
static u32 rtl930x_l2_hash_key(struct rtl838x_switch_priv *priv, u64 seed)
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

/*
 * Fills an L2 entry structure from the SoC registers
 */
static void rtl930x_fill_l2_entry(u32 r[], struct rtl838x_l2_entry *e)
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
			e->nh_route_id = r[2] & 0xfff;
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
static void rtl930x_fill_l2_row(u32 r[], struct rtl838x_l2_entry *e)
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
			r[2] |= e->nh_route_id & 0xfff;
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
static u64 rtl930x_read_l2_entry_using_hash(u32 hash, u32 pos, struct rtl838x_l2_entry *e)
{
	u32 r[3];
	struct table_reg *q = rtl_table_get(RTL9300_TBL_L2, 0);
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
	for (i = 0; i < 3; i++)
		r[i] = sw_r32(rtl_table_data(q, i));

	rtl_table_release(q);

	rtl930x_fill_l2_entry(r, e);

	pr_debug("%s: valid: %d, nh: %d\n", __func__, e->valid, e->next_hop);
	if (!e->valid)
		return 0;

	mac = ((u64)e->mac[0]) << 40 | ((u64)e->mac[1]) << 32 | ((u64)e->mac[2]) << 24
		| ((u64)e->mac[3]) << 16 | ((u64)e->mac[4]) << 8 | ((u64)e->mac[5]);

	seed = rtl930x_l2_hash_seed(mac, e->rvid);
	pr_debug("%s: mac %016llx, seed %016llx\n", __func__, mac, seed);
	// return vid with concatenated mac as unique id
	return seed;
}

static void rtl930x_write_l2_entry_using_hash(u32 hash, u32 pos, struct rtl838x_l2_entry *e)
{
	u32 r[3];
	struct table_reg *q = rtl_table_get(RTL9300_TBL_L2, 0);
	u32 idx = (0 << 14) | (hash << 2) | pos; // Access SRAM, with hash and at pos in bucket
	int i;

	pr_info("%s: hash %d, pos %d\n", __func__, hash, pos);
	pr_info("%s: index %d -> mac %02x:%02x:%02x:%02x:%02x:%02x\n", __func__, idx,
		e->mac[0], e->mac[1], e->mac[2], e->mac[3],e->mac[4],e->mac[5]);

	rtl930x_fill_l2_row(r, e);

	for (i= 0; i < 3; i++)
		sw_w32(r[i], rtl_table_data(q, i));

	rtl_table_write(q, idx);
	rtl_table_release(q);
}

static u64 rtl930x_read_cam(int idx, struct rtl838x_l2_entry *e)
{
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

	// return mac with concatenated vid as unique id
	return ((u64)r[0] << 28) | ((r[1] & 0xffff0000) >> 4) | e->vid;
}

static void rtl930x_write_cam(int idx, struct rtl838x_l2_entry *e)
{
	u32 r[3];
	struct table_reg *q = rtl_table_get(RTL9300_TBL_L2, 1); // Access L2 Table 1
	int i;

	rtl930x_fill_l2_row(r, e);

	for (i= 0; i < 3; i++)
		sw_w32(r[i], rtl_table_data(q, i));

	rtl_table_write(q, idx);
	rtl_table_release(q);
}

static void dump_l2_entry(struct rtl838x_l2_entry *e)
{
	pr_info("MAC: %02x:%02x:%02x:%02x:%02x:%02x vid: %d, rvid: %d, port: %d, valid: %d\n",
		e->mac[0], e->mac[1], e->mac[2], e->mac[3], e->mac[4], e->mac[5],
		e->vid, e->rvid, e->port, e->valid);
	pr_info("Type: %d, is_static: %d, is_ip_mc: %d, is_ipv6_mc: %d, block_da: %d\n",
		e->type, e->is_static, e->is_ip_mc, e->is_ipv6_mc, e->block_da);
	pr_info("  block_sa: %d, suspended: %d, next_hop: %d, age: %d, is_trunk: %d, trunk: %d\n",
		e->block_sa, e->suspended, e->next_hop, e->age, e->is_trunk, e->trunk);
	if (e->is_ip_mc || e->is_ipv6_mc)
		pr_info("  mc_portmask_index: %d, mc_gip: %d, mc_sip: %d\n",
			e->mc_portmask_index, e->mc_gip, e->mc_sip);
	pr_info("  stac_dev: %d, nh_route_id: %d, port: %d, dev_id\n",
		e->stack_dev, e->nh_route_id, e->port);
}

/*
 * Add an L2 nexthop entry for the L3 routing system in the SoC
 * Use VID and MAC in rtl838x_l2_entry to identify either a free slot in the L2 hash table
 * or mark an existing entry as a nexthop by setting it's nexthop bit
 * Called from the L3 layer
 * The index in the L2 hash table is filled into nh->l2_id;
 */
static int rtl930x_l2_nexthop_add(struct rtl838x_switch_priv *priv, struct rtl838x_nexthop *nh)
{
	struct rtl838x_l2_entry e;
	u64 seed = rtl930x_l2_hash_seed(nh->mac, nh->vid);
	u32 key = rtl930x_l2_hash_key(priv, seed);
	int i, idx = -1;
	u64 entry;

	pr_info("%s searching for %08llx vid %d with key %d, seed: %016llx\n",
		__func__, nh->mac, nh->vid, key, seed);
	
	e.type = L2_UNICAST;
	e.rvid = nh->fid; // Verify its the forwarding ID!!! l2_entry.un.unicast.fid
	u64_to_ether_addr(nh->mac, &e.mac[0]);
	e.port = RTL930X_PORT_IGNORE;

	// Loop over all entries in the hash-bucket and over the second block on 93xx SoCs
	for (i = 0; i < priv->l2_bucket_size; i++) {
		entry = rtl930x_read_l2_entry_using_hash(key, i, &e);
		pr_info("%s i: %d, entry %016llx, seed %016llx\n", __func__, i, entry, seed);
		if (e.valid && e.next_hop)
			continue;
		if (!e.valid || ((entry & 0x0fffffffffffffffULL) == seed)) {
			idx = i > 3 ? ((key >> 14) & 0xffff) | i >> 1
					: ((key << 2) | i) & 0xffff;
			break;
		}
	}

	pr_info("%s: found idx %d and i %d\n", __func__, idx, i);

	if (idx < 0) {
		pr_err("%s: No more L2 forwarding entries available\n", __func__);
		return -1;
	}

	// Found an existing or empty entry, make it a nexthop entry
	pr_info("%s BEFORE -> key %d, pos: %d, index: %d\n", __func__, key, i, idx);
	dump_l2_entry(&e);
	nh->l2_id = idx;

	// Found an existing (e->valid is true) or empty entry, make it a nexthop entry
	if (e.valid) {
		nh->port = e.port;
		nh->fid = e.rvid;
		nh->vid = e.vid;
		nh->dev_id = e.stack_dev;
	} else {
		e.valid = true;
		e.is_static = false;
		e.vid = nh->vid;
		e.rvid = nh->fid;
		e.port = RTL930X_PORT_IGNORE;
		u64_to_ether_addr(nh->mac, &e.mac[0]);
	}
	e.next_hop = true;
	// For nexthop entries, the vid field in the table is used to denote the dest mac_id
	e.nh_route_id = nh->mac_id;
	pr_info("%s AFTER\n", __func__);
	dump_l2_entry(&e);

	rtl930x_write_l2_entry_using_hash(idx >> 2, idx & 0x3, &e);

	// _dal_longan_l2_nexthop_add
	return 0;
}

static u64 rtl930x_read_mcast_pmask(int idx)
{
	u32 portmask;
	// Read MC_PMSK (2) via register RTL8380_TBL_L2
	struct table_reg *q = rtl_table_get(RTL8380_TBL_L2, 2);

	rtl_table_read(q, idx);
	portmask = sw_r32(rtl_table_data(q, 0));
	portmask >>= 3;
	rtl_table_release(q);

	pr_info("%s: Index idx %d has portmask %08x\n", __func__, idx, portmask);
	return portmask;
}

static void rtl930x_write_mcast_pmask(int idx, u64 portmask)
{
	u32 pm = portmask;

	// Access MC_PMSK (2) via register RTL8380_TBL_L2
	struct table_reg *q = rtl_table_get(RTL8380_TBL_L2, 2);

	pr_info("%s: Index idx %d has portmask %08x\n", __func__, idx, pm);
	pm <<= 3;
	sw_w32(pm, rtl_table_data(q, 0));
	rtl_table_write(q, idx);
	rtl_table_release(q);
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

/*
 * Enable traffic between a source port and a destination port
 */
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

irqreturn_t rtl930x_switch_irq(int irq, void *dev_id)
{
	struct dsa_switch *ds = dev_id;
	u32 status = sw_r32(RTL930X_ISR_GLB);
	u32 ports = sw_r32(RTL930X_ISR_PORT_LINK_STS_CHG);
	u32 link;
	int i;

	/* Clear status */
	sw_w32(ports, RTL930X_ISR_PORT_LINK_STS_CHG);
	pr_info("RTL9300 Link change: status: %x, ports %x\n", status, ports);

	rtl9300_dump_debug();

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


int rtl930x_write_phy(u32 port, u32 page, u32 reg, u32 val)
{
	u32 v;
	int err = 0;

	pr_debug("%s: port %d, page: %d, reg: %x, val: %x\n", __func__, port, page, reg, val);

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
	} while (v & BIT(0));

	if (v & 0x2)
		err = -EIO;

	mutex_unlock(&smi_lock);

	return err;
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
	} while (v & BIT(0));

	if (v & BIT(25)) {
		pr_debug("Error reading phy %d, register %d\n", port, reg);
		err = -EIO;
	}
	*val = (sw_r32(RTL930X_SMI_ACCESS_PHY_CTRL_2) & 0xffff);

	pr_debug("%s: port %d, page: %d, reg: %x, val: %x\n", __func__, port, page, reg, *val);

	mutex_unlock(&smi_lock);

	return err;
}

/*
 * Write to an mmd register of the PHY
 */
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

	v = BIT(2) | BIT(1) | BIT(0); // WRITE | MMD-access | EXEC
	sw_w32(v, RTL930X_SMI_ACCESS_PHY_CTRL_1);

	do {
		v = sw_r32(RTL930X_SMI_ACCESS_PHY_CTRL_1);
	} while (v & BIT(0));

	pr_debug("%s: port %d, regnum: %x, val: %x (err %d)\n", __func__, port, regnum, val, err);
	mutex_unlock(&smi_lock);
	return err;
}

/*
 * Read an mmd register of the PHY
 */
int rtl930x_read_mmd_phy(u32 port, u32 devnum, u32 regnum, u32 *val)
{
	int err = 0;
	u32 v;

	mutex_lock(&smi_lock);

	// Set PHY to access
	sw_w32_mask(0xffff << 16, port << 16, RTL930X_SMI_ACCESS_PHY_CTRL_2);

	// Set MMD device number and register to write to
	sw_w32(devnum << 16 | (regnum & 0xffff), RTL930X_SMI_ACCESS_PHY_CTRL_3);

	v = BIT(1) | BIT(0); // MMD-access | EXEC
	sw_w32(v, RTL930X_SMI_ACCESS_PHY_CTRL_1);

	do {
		v = sw_r32(RTL930X_SMI_ACCESS_PHY_CTRL_1);
	} while (v & BIT(0));
	// There is no error-checking via BIT 25 of v, as it does not seem to be set correctly
	*val = (sw_r32(RTL930X_SMI_ACCESS_PHY_CTRL_2) & 0xffff);
	pr_debug("%s: port %d, regnum: %x, val: %x (err %d)\n", __func__, port, regnum, *val, err);

	mutex_unlock(&smi_lock);

	return err;
}

/*
 * Enables or disables the EEE/EEEP capability of a port
 */
void rtl930x_port_eee_set(struct rtl838x_switch_priv *priv, int port, bool enable)
{
	u32 v;

	// This works only for Ethernet ports, and on the RTL930X, ports from 26 are SFP
	if (port >= 26)
		return;

	pr_debug("In %s: setting port %d to %d\n", __func__, port, enable);
	v = enable ? 0x3f : 0x0;

	// Set EEE/EEEP state for 100, 500, 1000MBit and 2.5, 5 and 10GBit
	sw_w32_mask(0, v << 10, rtl930x_mac_force_mode_ctrl(port));

	// Set TX/RX EEE state
	v = enable ? 0x3 : 0x0;
	sw_w32(v, RTL930X_EEE_CTRL(port));

	priv->ports[port].eee_enabled = enable;
}

/*
 * Get EEE own capabilities and negotiation result
 */
int rtl930x_eee_port_ability(struct rtl838x_switch_priv *priv, struct ethtool_eee *e, int port)
{
	u32 link, a;

	if (port >= 26)
		return -ENOTSUPP;

	pr_info("In %s, port %d\n", __func__, port);
	link = sw_r32(RTL930X_MAC_LINK_STS);
	link = sw_r32(RTL930X_MAC_LINK_STS);
	if (!(link & BIT(port)))
		return 0;

	pr_info("Setting advertised\n");
	if (sw_r32(rtl930x_mac_force_mode_ctrl(port)) & BIT(10))
		e->advertised |= ADVERTISED_100baseT_Full;

	if (sw_r32(rtl930x_mac_force_mode_ctrl(port)) & BIT(12))
		e->advertised |= ADVERTISED_1000baseT_Full;

	if (priv->ports[port].is2G5 && sw_r32(rtl930x_mac_force_mode_ctrl(port)) & BIT(13)) {
		pr_info("ADVERTISING 2.5G EEE\n");
		e->advertised |= ADVERTISED_2500baseX_Full;
	}

	if (priv->ports[port].is10G && sw_r32(rtl930x_mac_force_mode_ctrl(port)) & BIT(15))
		e->advertised |= ADVERTISED_10000baseT_Full;

	a = sw_r32(RTL930X_MAC_EEE_ABLTY);
	a = sw_r32(RTL930X_MAC_EEE_ABLTY);
	pr_info("Link partner: %08x\n", a);
	if (a & BIT(port)) {
		e->lp_advertised = ADVERTISED_100baseT_Full;
		e->lp_advertised |= ADVERTISED_1000baseT_Full;
		if (priv->ports[port].is2G5)
			e->lp_advertised |= ADVERTISED_2500baseX_Full;
		if (priv->ports[port].is10G)
			e->lp_advertised |= ADVERTISED_10000baseT_Full;
	}

	// Read 2x to clear latched state
	a = sw_r32(RTL930X_EEEP_PORT_CTRL(port));
	a = sw_r32(RTL930X_EEEP_PORT_CTRL(port));
	pr_info("%s RTL930X_EEEP_PORT_CTRL: %08x\n", __func__, a);

	return 0;
}

static void rtl930x_init_eee(struct rtl838x_switch_priv *priv, bool enable)
{
	int i;

	pr_info("Setting up EEE, state: %d\n", enable);

	// Setup EEE on all ports
	for (i = 0; i < priv->cpu_port; i++) {
		if (priv->ports[i].phy)
			rtl930x_port_eee_set(priv, i, enable);
	}

	priv->eee_enabled = enable;
}

/*
 * Read a prefix route entry from the table
 * We currently only support IPv4 and IPv6 unicast route
 */
static void rtl930x_prefix_route_read(u32 idx, struct rtl838x_route_info *info)
{
	u32 v, ip4_m;
	bool host_route, default_route;
	struct in6_addr ip6_m;

	// Read L3_PREFIX_ROUTE_IPUC table (2) via register RTL9300_TBL_1
	struct table_reg *r = rtl_table_get(RTL9300_TBL_1, 2);

	rtl_table_read(r, idx);
	// The table has a size of 11 registers
	info->valid = !!(sw_r32(rtl_table_data(r, 0)) & BIT(31));
	if (!info->valid)
		goto out;

	info->type = (sw_r32(rtl_table_data(r, 0)) >> 29) & 0x3;

	v = sw_r32(rtl_table_data(r, 10));
	host_route = !!(v & BIT(21));
	default_route = !!(v & BIT(20));
	info->prefix_len = -1;
	pr_info("%s: host route %d, default_route %d\n", __func__, host_route, default_route);

	switch (info->type) {
	case 0: // IPv4 Unicast route
		info->ip4_r = sw_r32(rtl_table_data(r, 4));
		ip4_m = sw_r32(rtl_table_data(r, 9));
		pr_info("%s: Read ip4 mask: %08x\n", __func__, ip4_m);
		info->prefix_len = host_route ? 32 : -1;
		info->prefix_len = (info->prefix_len < 0 && default_route) ? 0 : -1;
		if (info->prefix_len < 0)
			info->prefix_len = inet_mask_len(ip4_m);
		break;
	case 2: // IPv6 Unicast route
		ipv6_addr_set(&info->ip6_r,
			      sw_r32(rtl_table_data(r, 1)), sw_r32(rtl_table_data(r, 2)),
			      sw_r32(rtl_table_data(r, 3)), sw_r32(rtl_table_data(r, 4)));
		ipv6_addr_set(&ip6_m,
			      sw_r32(rtl_table_data(r, 6)), sw_r32(rtl_table_data(r, 7)),
			      sw_r32(rtl_table_data(r, 8)), sw_r32(rtl_table_data(r, 9)));
		info->prefix_len = host_route ? 128 : 0;
		info->prefix_len = (info->prefix_len < 0 && default_route) ? 0 : -1;
		if (info->prefix_len < 0)
			info->prefix_len = find_last_bit((unsigned long int *)&ip6_m.s6_addr32,
							 128);
		break;
	case 1: // IPv4 Multicast route
	case 3: // IPv6 Multicast route
		pr_warn("%s: route type not supported\n", __func__);
		goto out;
	}

	info->hit = !!(v & BIT(22));
	info->action = (v >> 18) & 3;
	info->next_hop = (v >> 7) & 0x7ff;
	info->ttl_dec = !!(v & BIT(6));
	info->ttl_check = !!(v & BIT(5));
	info->dst_null = !!(v & BIT(4));
	info->qos_as = !!(v & BIT(3));
	info->qos_prio =  v & 0x7;
	pr_info("%s: index %d is valid: %d\n", __func__, idx, info->valid);
	pr_info("%s: next_hop: %d, hit: %d, action :%d, ttl_dec %d, ttl_check %d, dst_null %d\n",
		__func__, info->next_hop, info->hit, info->action, info->ttl_dec, info->ttl_check,
		info->dst_null);
	pr_info("%s: GW: %pI4, prefix_len: %d\n", __func__, &info->ip4_r, info->prefix_len);
out:
	rtl_table_release(r);
}

static void rtl930x_net6_mask(int prefix_len, struct in6_addr *ip6_m)
{
	int o, b;
	// Define network mask
	o = prefix_len >> 3;
	b = prefix_len & 0x7;
	memset(ip6_m->s6_addr, 0xff, o);
	ip6_m->s6_addr[o] |= b ? 0xff00 >> b : 0x00;
}

/*
 * Write a prefix route into the routing table CAM at position idx
 * Currently only IPv4 and IPv6 unicast routes are supported
 */
static void rtl930x_prefix_route_write(u32 idx, struct rtl838x_route_info *info)
{
	u32 v, ip4_m;
	struct in6_addr ip6_m;
	// Access L3_PREFIX_ROUTE_IPUC table (2) via register RTL9300_TBL_1
	// The table has a size of 11 registers (20 for MC)
	struct table_reg *r = rtl_table_get(RTL9300_TBL_1, 2);

	pr_info("%s: index %d is valid: %d\n", __func__, idx, info->valid);
	pr_info("%s: next_hop: %d, hit: %d, action :%d, ttl_dec %d, ttl_check %d, dst_null %d\n",
		__func__, info->next_hop, info->hit, info->action, info->ttl_dec, info->ttl_check,
		info->dst_null);
	pr_info("%s: GW: %pI4, prefix_len: %d\n", __func__, &info->ip4_r, info->prefix_len);

	v = info->valid ? BIT(31) : 0;
	v |= (info->type & 0x3) << 29;
	sw_w32(v, rtl_table_data(r, 0));

	v = info->hit ? BIT(22) : 0;
	v |= (info->action & 0x3) << 18;
	v |= (info->next_hop & 0x7ff) << 7;
	v |= info->ttl_dec ? BIT(6) : 0;
	v |= info->ttl_check ? BIT(5) : 0;
	v |= info->dst_null ? BIT(6) : 0;
	v |= info->qos_as ? BIT(6) : 0;
	v |= info->qos_prio & 0x7;
	v |= info->prefix_len == 0 ? BIT(20) : 0; // set default route bit

	// 320 0, 288 1, 256 2, 224 3, 192 4
	// set bit mask for entry type always to 0x3
	sw_w32(0x3 << 29, rtl_table_data(r, 5));

	switch (info->type) {
	case 0: // IPv4 Unicast route
		sw_w32(info->ip4_r, rtl_table_data(r, 4));

		v |= info->prefix_len == 32 ? BIT(21) : 0; // set host-route bit
		ip4_m = inet_make_mask(info->prefix_len);
		sw_w32(ip4_m, rtl_table_data(r, 9));
		break;
	case 2: // IPv6 Unicast route
		sw_w32(info->ip6_r.s6_addr32[0], rtl_table_data(r, 1));
		sw_w32(info->ip6_r.s6_addr32[1], rtl_table_data(r, 2));
		sw_w32(info->ip6_r.s6_addr32[2], rtl_table_data(r, 3));
		sw_w32(info->ip6_r.s6_addr32[3], rtl_table_data(r, 4));

		v |= info->prefix_len == 128 ? BIT(21) : 0; // set host-route bit

		rtl930x_net6_mask(info->prefix_len, &ip6_m);

		sw_w32(ip6_m.s6_addr32[0], rtl_table_data(r, 6));
		sw_w32(ip6_m.s6_addr32[1], rtl_table_data(r, 7));
		sw_w32(ip6_m.s6_addr32[2], rtl_table_data(r, 8));
		sw_w32(ip6_m.s6_addr32[3], rtl_table_data(r, 9));
		break;
	case 1: // IPv4 Multicast route
	case 3: // IPv6 Multicast route
		pr_warn("%s: route type not supported\n", __func__);
		rtl_table_release(r);
		return;
	}
	sw_w32(v, rtl_table_data(r, 10));

	rtl_table_write(r, idx);
	rtl_table_release(r);
}

/*
 * Read a host route entry from the table using its index
 * We currently only support IPv4 and IPv6 unicast route
 */
static void rtl930x_host_route_read(u32 idx, struct rtl838x_route_info *info)
{
	u32 v;
	// Read L3_HOST_ROUTE_IPUC table (1) via register RTL9300_TBL_1
	struct table_reg *r = rtl_table_get(RTL9300_TBL_1, 1);

	pr_info("In %s\n", __func__);
	rtl_table_read(r, idx);
	// The table has a size of 5 (for UC, 11 for MC) registers
	v = sw_r32(rtl_table_data(r, 0));
	info->valid = !!(v & BIT(31));
	if (!info->valid)
		goto out;
	info->type = (v >> 29) & 0x3;
	switch (info->type) {
	case 0: // IPv4 Unicast route
		info->ip4_r = sw_r32(rtl_table_data(r, 4));
		break;
	case 2: // IPv6 Unicast route
		ipv6_addr_set(&info->ip6_r,
			      sw_r32(rtl_table_data(r, 3)), sw_r32(rtl_table_data(r, 2)),
			      sw_r32(rtl_table_data(r, 1)), sw_r32(rtl_table_data(r, 0)));
		break;
	case 1: // IPv4 Multicast route
	case 3: // IPv6 Multicast route
		pr_warn("%s: route type not supported\n", __func__);
		goto out;
	}

	info->hit = !!(v & BIT(20));
	info->dst_null = !!(v & BIT(19));
	info->action = (v >> 17) & 3;
	info->next_hop = (v >> 6) & 0x7ff;
	info->ttl_dec = !!(v & BIT(5));
	info->ttl_check = !!(v & BIT(4));
	info->qos_as = !!(v & BIT(3));
	info->qos_prio =  v & 0x7;
	pr_info("%s: index %d is valid: %d\n", __func__, idx, info->valid);
	pr_info("%s: next_hop: %d, hit: %d, action :%d, ttl_dec %d, ttl_check %d, dst_null %d\n",
		__func__, info->next_hop, info->hit, info->action, info->ttl_dec, info->ttl_check,
		info->dst_null);
	pr_info("%s: GW: %pI4, prefix_len: %d\n", __func__, &info->ip4_r, info->prefix_len);

out:
	rtl_table_release(r);
}

/*
 * Write a host route entry from the table using its index
 * We currently only support IPv4 and IPv6 unicast route
 */
static void rtl930x_host_route_write(u32 idx, struct rtl838x_route_info *info)
{
	u32 v;
	// Access L3_HOST_ROUTE_IPUC table (1) via register RTL9300_TBL_1
	struct table_reg *r = rtl_table_get(RTL9300_TBL_1, 1);
	// The table has a size of 5 (for UC, 11 for MC) registers

	pr_info("%s: index %d is valid: %d\n", __func__, idx, info->valid);
	pr_info("%s: next_hop: %d, hit: %d, action :%d, ttl_dec %d, ttl_check %d, dst_null %d\n",
		__func__, info->next_hop, info->hit, info->action, info->ttl_dec, info->ttl_check,
		info->dst_null);
	pr_info("%s: GW: %pI4, prefix_len: %d\n", __func__, &info->ip4_r, info->prefix_len);

	v = BIT(31); // Entry is valid
	v |= (info->type & 0x3) << 29;
	v |= info->hit ? BIT(20) : 0;
	v |= info->dst_null ? BIT(19) : 0;
	v |= (info->action & 0x3) << 17;
	v |= (info->next_hop & 0x7ff) << 6;
	v |= info->ttl_dec ? BIT(5) : 0;
	v |= info->ttl_check ? BIT(4) : 0;
	v |= info->qos_as ? BIT(3) : 0;
	v |= info->qos_prio & 0x7;

	sw_w32(v, rtl_table_data(r, 0));
	switch (info->type) {
	case 0: // IPv4 Unicast route
		sw_w32(info->ip4_r, rtl_table_data(r, 4));
		break;
	case 2: // IPv6 Unicast route
		sw_w32(info->ip6_r.s6_addr32[0], rtl_table_data(r, 1));
		sw_w32(info->ip6_r.s6_addr32[1], rtl_table_data(r, 2));
		sw_w32(info->ip6_r.s6_addr32[2], rtl_table_data(r, 3));
		sw_w32(info->ip6_r.s6_addr32[3], rtl_table_data(r, 4));
		break;
	case 1: // IPv4 Multicast route
	case 3: // IPv6 Multicast route
		pr_warn("%s: route type not supported\n", __func__);
		goto out;
	}

	rtl_table_write(r, idx);

out:
	rtl_table_release(r);

}

/*
 * Look up the index of a prefix route in the routing table CAM for unicast IPv4/6 routes
 * using hardware offload.
 */
static int rtl930x_route_lookup_hw(struct rtl838x_route_info *info)
{
	u32 ip4_m, v;
	struct in6_addr ip6_m;
	int i;

	if (info->type == 1 || info->type == 3) // Hardware only supports UC routes
		return -1;

	sw_w32_mask(0x3 << 19, info->type, RTL930X_L3_HW_LU_KEY_CTRL);
	if (info->type) { // IPv6
		rtl930x_net6_mask(info->prefix_len, &ip6_m);
		for (i = 0; i < 4; i++)
			sw_w32(info->ip6_r.s6_addr32[0] & ip6_m.s6_addr32[0],
			       RTL930X_L3_HW_LU_KEY_IP_CTRL + (i << 2));
	} else { // IPv4
		ip4_m = inet_make_mask(info->prefix_len);
		sw_w32(0, RTL930X_L3_HW_LU_KEY_IP_CTRL);
		sw_w32(0, RTL930X_L3_HW_LU_KEY_IP_CTRL + 4);
		sw_w32(0, RTL930X_L3_HW_LU_KEY_IP_CTRL + 8);
		v = info->ip4_r & ip4_m;
		pr_info("%s: searching for %pI4\n", __func__, &v);
		sw_w32(v, RTL930X_L3_HW_LU_KEY_IP_CTRL + 12);
	}

	// Execute CAM lookup in SoC
	sw_w32(BIT(15), RTL930X_L3_HW_LU_CTRL);

	// Wait until execute bit clears and result is ready
	do {
		v = sw_r32(RTL930X_L3_HW_LU_CTRL);
	} while (v & BIT(15));

	pr_info("%s: found: %d, index: %d\n", __func__, !!(v & BIT(14)), v & 0x1ff);

	// Test if search successful (BIT 14 set)
	if (v & BIT(14))
		return v & 0x1ff;

	return -1;
}

// TODO: Hold a lock
int rtl930x_prefix_route_alloc(struct rtl838x_switch_priv *priv, bool is_ip6)
{
	unsigned long int *use_bm = is_ip6 ? &priv->prefix_ip6route_use_bm[0]
					: &priv->prefix_ip4route_use_bm[0];
	int i = find_first_zero_bit(use_bm, MAX_ROUTE_CAM_ENTRIES / 2);
	struct rtl838x_route *r;

	pr_info("%s: found index %d\n", __func__, i);
	if (i >= MAX_ROUTE_CAM_ENTRIES / 2)
		return -1;
	pr_info("before %08lx\n", priv->prefix_ip4route_use_bm[0]);
	set_bit(i, use_bm);
	pr_info("after: %08lx\n", priv->prefix_ip4route_use_bm[0]);

	// We saparate the IPv4 and IPv6 routes into different halves of the CAM memory
	if (is_ip6)
		i += MAX_ROUTE_CAM_ENTRIES / 2;

	// Add a route entry into the list of routes
	r = kzalloc(sizeof *r, GFP_KERNEL);
	if (!r)
		return -ENOMEM;
	r->rt.id = i;
	list_add(&r->list, &priv->routes.list);

	pr_info("%s Returning index %d\n", __func__, i);
	return i;
}

// TODO: HOLD A LOCK
int rtl930x_prefix_route_free(struct rtl838x_switch_priv *priv, int index)
{
	unsigned long int *use_bm;

	use_bm = index >= MAX_ROUTE_CAM_ENTRIES / 2 ? &priv->prefix_ip6route_use_bm[0]
					: &priv->prefix_ip4route_use_bm[0];
	index %= MAX_ROUTE_CAM_ENTRIES / 2;

	if (!test_bit(index, use_bm))
		return -1;

	clear_bit(index, use_bm);

	return 0;
}

/*
 * Returns the route list entry with matchin hw id
 */
struct rtl838x_route *rtl930x_prefix_route_find_id(struct rtl838x_switch_priv *priv, int id)
{
	struct list_head *i;
	struct rtl838x_route *r;

	list_for_each(i, &priv->routes.list) {
		r = list_entry(i, struct rtl838x_route, list);
		if (r->rt.id == id)
			return r;
	}

	return NULL;
}

/*
 * Returns the route list entry with matchin given gateway ip
 */
struct rtl838x_route *rtl930x_prefix_route_find_gw(struct rtl838x_switch_priv *priv, __be32 ip)
{
	struct list_head *i;
	struct rtl838x_route *r;

	list_for_each(i, &priv->routes.list) {
		r = list_entry(i, struct rtl838x_route, list);
		pr_debug("Looking at %pI4 for %pI4\n", &r->nh.ip, &ip);
		if (r->nh.ip == ip)
			return r;
	}

	return NULL;
}

/*
 * Returns the route list entry with matchin given network ip
 */
struct rtl838x_route *rtl930x_prefix_route_find_ip(struct rtl838x_switch_priv *priv, __be32 ip)
{
	struct list_head *i;
	struct rtl838x_route *r;

	list_for_each(i, &priv->routes.list) {
		r = list_entry(i, struct rtl838x_route, list);
		if (r->rt.ip4_r == ip)
			return r;
	}

	return NULL;
}

/*
 * Get the Destination-MAC of an L3 egress interface from the SoC's L3_EGR_INTF_MAC table
 */
static u64 rtl930x_get_l3_egress_mac(u32 idx)
{
	u64 mac;
	// Read L3_EGR_INTF_MAC table (2) via register RTL9300_TBL_2
	struct table_reg *r = rtl_table_get(RTL9300_TBL_2, 2);

	rtl_table_read(r, idx);
	// The table has a size of 2 registers
	mac = sw_r32(rtl_table_data(r, 0));
	mac <<= 32;
	mac |= sw_r32(rtl_table_data(r, 1));
	rtl_table_release(r);

	return mac;
}

/*
 * Set the Destination-MAC of an L3 egress interface in the SoC's L3_EGR_INTF_MAC table
 */
static void rtl930x_set_l3_egress_mac(u32 idx, u64 mac)
{
	// Access L3_EGR_INTF_MAC table (2) via register RTL9300_TBL_2
	struct table_reg *r = rtl_table_get(RTL9300_TBL_2, 2);

	pr_info("%s: Writing to L3_EGR_INTF_MAC table, index %d, dmac %016llx\n",
		__func__, idx, mac);
	// The table has a size of 2 registers
	sw_w32(mac >> 32, rtl_table_data(r, 0));
	sw_w32(mac, rtl_table_data(r, 1));

	rtl_table_write(r, idx);
	rtl_table_release(r);
}

/*
 * Get the destination MAC and L3 egress interface ID of a nexthop entry from
 * the SoC's L3_NEXTHOP table
 */
static void rtl930x_get_l3_nexthop(int idx, u16 *dmac_id, u16 *interface)
{
	u32 v;
	// Read L3_NEXTHOP table (3) via register RTL9300_TBL_1
	struct table_reg *r = rtl_table_get(RTL9300_TBL_1, 3);

	rtl_table_read(r, idx);
	// The table has a size of 1 register
	v = sw_r32(rtl_table_data(r, 0));
	rtl_table_release(r);

	*dmac_id = (v >> 7) & 0x7fff;
	*interface = v & 0x7f;
}

/*
 * Set the destination MAC and L3 egress interface ID for a nexthop entry in the SoC's
 * L3_NEXTHOP table. The nexthop entry is identified by idx.
 * dmac_id is the reference to the L2 entry in the L2 forwarding table, special values are
 * 0x7ffe: TRAP2CPU
 * 0x7ffd: TRAP2MASTERCPU
 * 0x7fff: DMAC_ID_DROP
 */
static void rtl930x_set_l3_nexthop(int idx, u16 dmac_id, u16 interface)
{
	// Access L3_NEXTHOP table (3) via register RTL9300_TBL_1
	struct table_reg *r = rtl_table_get(RTL9300_TBL_1, 3);

	pr_info("%s: Writing to L3_NEXTHOP table, index %d, dmac_id %d, interface %d\n",
		__func__, idx, dmac_id, interface);
	sw_w32(((dmac_id & 0x7fff) << 7) | (interface & 0x7f), rtl_table_data(r, 0));

	rtl_table_write(r, idx);
	rtl_table_release(r);
}

/*
 * Reads a source MAC entry for L3 routing out of the hardware table
 * idx is the index into the L3_ROUTER_MAC table
 */
static void rtl930x_get_l3_router_mac(u32 idx, struct rtl838x_rt_mac *m)
{
	u32 v, w;
	// Read L3_ROUTER_MAC table (0) via register RTL9300_TBL_1
	struct table_reg *r = rtl_table_get(RTL9300_TBL_1, 0);

	rtl_table_read(r, idx);
	// The table has a size of 7 registers, 64 entries
	v = sw_r32(rtl_table_data(r, 0));
	w = sw_r32(rtl_table_data(r, 3));
	m->valid = !!(v & BIT(20));
	if (!m->valid)
		return;

	m->p_type = !!(v & BIT(19));
	m->p_id = (v >> 13) & 0x3f;  // trunk id of port
	m->vid = v & 0xfff;
	m->vid_mask = w & 0xfff;
	m->action = sw_r32(rtl_table_data(r, 6)) & 0x7;
	m->mac_mask = ((((u64)sw_r32(rtl_table_data(r, 5))) << 32) & 0xffffffffffffULL)
			| (sw_r32(rtl_table_data(r, 4)));
	m->mac = ((((u64)sw_r32(rtl_table_data(r, 1))) << 32) & 0xffffffffffffULL) 
			| (sw_r32(rtl_table_data(r, 2)));
	// Bits L3_INTF and BMSK_L3_INTF are 0

	rtl_table_release(r);
}

/*
 * Writes a source MAC entry for L3 routing into the hardware table
 * idx is the index into the L3_ROUTER_MAC table
 */
static void rtl930x_set_l3_router_mac(u32 idx, struct rtl838x_rt_mac *m)
{
	u32 v, w;
	// Read L3_ROUTER_MAC table (0) via register RTL9300_TBL_1
	struct table_reg *r = rtl_table_get(RTL9300_TBL_1, 0);

	// The table has a size of 7 registers, 64 entries
	v = BIT(20); // mac entry valid, port type is 0: individual
	v |= (m->p_id & 0x3f) << 13;
	v |= (m->vid & 0xfff); // Set the interface_id to the vlan id
	w = m->vid_mask;
	sw_w32(v, rtl_table_data(r, 0));
	sw_w32(w, rtl_table_data(r, 3));
	sw_w32((u32)(m->mac), rtl_table_data(r, 2));
	// Bit L3_INTF (bit 12 in register 1) needs to be 0
	sw_w32(m->mac >> 32, rtl_table_data(r, 1));
	// Bit BMSK_L3_INTF (bit 12 in register 5) needs to be 0
	sw_w32((u32)(m->mac_mask >> 32), rtl_table_data(r, 4));
	sw_w32((u32)m->mac_mask, rtl_table_data(r, 5));
	sw_w32(m->action & 0x7, rtl_table_data(r, 6));

	rtl_table_write(r, idx);
	rtl_table_release(r);
}

static void rtl930x_setup_port_macs(struct rtl838x_switch_priv *priv)
{
	int i;
	struct net_device *dev;
	struct rtl838x_rt_mac m;

	for (i = 0; i < priv->cpu_port; i++) {
		if (!priv->ports[i].dp)
			continue;
		pr_info("%s: got port %08x\n", __func__, (u32)priv->ports[i].dp);
		dev = priv->ports[i].dp->slave;
		m.valid = true;
		m.mac = ether_addr_to_u64(dev->dev_addr) + i + 1; // BUG: VRRP for testing
		m.p_type = 0; // An individual port, not a trunk port
		m.p_id = i;

		// TODO: For now we do not care about the VID
		m.vid = 0;
		m.vid_mask = 0x3ff;

		m.mac_mask = 0xffffffffffffULL;
		m.action = 0; // Forward routed package
		rtl930x_set_l3_router_mac(i, &m);
	}
}

static void rtl930x_get_l3_egress_intf(int idx, struct rtl838x_l3_intf *intf)
{
	u32 u, v;
	// Read L3_EGR_INTF table (4) via register RTL9300_TBL_1
	struct table_reg *r = rtl_table_get(RTL9300_TBL_1, 4);

	rtl_table_read(r, idx);

	// The table has a size of 2 registers
	u = sw_r32(rtl_table_data(r, 0));
	v = sw_r32(rtl_table_data(r, 1));
	rtl_table_release(r);
	
	intf->vid = (u >> 9) & 0xfff;
	intf->smac_idx = (u >> 3) & 0x3f;
	intf->ip4_mtu_id = u & 0x3;
	intf->ip6_mtu_id = (v >> 28) & 0x3;
	intf->ttl_scope = (v >> 20) & 0xff;
	intf->hl_scope = (v >> 12) & 0xff;
	intf->ip4_icmp_redirect = (v >> 9) & 0x3;
	intf->ip6_icmp_redirect = (v >> 6) & 0x3;
	intf->ip4_pbr_icmp_redirect = (v >> 3) & 0x3;
	intf->ip6_pbr_icmp_redirect = v & 0x3;
}

/*
 * Sets up an egress interface for L3 actions
 * Actions for ip4/6_icmp_redirect, ip4/6_pbr_icmp_redirect are:
 * 0: FORWARD, 1: DROP, 2: TRAP2CPU, 3: COPY2CPU, 4: TRAP2MASTERCPU 5: COPY2MASTERCPU
 * 6: HARDDROP
 * idx is the index in the HW interface table: idx < 0x80
 */
static void rtl930x_set_l3_egress_intf(int idx, struct rtl838x_l3_intf *intf)
{
	u32 u, v;
	// Read L3_EGR_INTF table (4) via register RTL9300_TBL_1
	struct table_reg *r = rtl_table_get(RTL9300_TBL_1, 4);

	// The table has 2 registers
	u = (intf->vid & 0xfff) << 9;
	u |= (intf->smac_idx & 0x3f) << 3;
	u |= (intf->ip4_mtu_id & 0x7);

	v = (intf->ip6_mtu_id & 0x7) << 28;
	v |= (intf->ttl_scope & 0xff) << 20;
	v |= (intf->hl_scope & 0xff) << 12;
	v |= (intf->ip4_icmp_redirect & 0x7) << 9;
	v |= (intf->ip6_icmp_redirect & 0x7)<< 6;
	v |= (intf->ip4_pbr_icmp_redirect & 0x7) << 3;
	v |= (intf->ip6_pbr_icmp_redirect & 0x7);

	sw_w32(u, rtl_table_data(r, 0));
	sw_w32(v, rtl_table_data(r, 1));
	rtl_table_write(r, idx & 0x7f);
	rtl_table_release(r);
}

static int rtl930x_l3_mtu_add(struct rtl838x_switch_priv *priv, int mtu)
{
	int i, free_mtu;
	int mtu_id;

	// Try to find an existing mtu-value or a free slot
	free_mtu = MAX_INTF_MTUS;
	for (i = 0; i < MAX_INTF_MTUS && priv->intf_mtus[i] != mtu; i++) {
		if ((!priv->intf_mtu_count[i]) && (free_mtu == MAX_INTF_MTUS))
			free_mtu = i;
	}
	i = (i < MAX_INTF_MTUS) ? i : free_mtu;
	if (i < MAX_INTF_MTUS) {
		mtu_id = i;
	} else {
		pr_err("%s: No free MTU slot available!\n", __func__);
		return -EINVAL;
	}

	priv->intf_mtus[i] = mtu;
	pr_info("Writing MTU %d to slot %d\n", priv->intf_mtus[i], i);
	// Set MTU-value of the slot TODO: distinguish between IPv4/IPv6 routes / slots
	sw_w32_mask(0xffff << ((i % 2) * 16), priv->intf_mtus[i] << ((i % 2) * 16),
		    RTL930X_L3_IP_MTU_CTRL(i));
	sw_w32_mask(0xffff << ((i % 2) * 16), priv->intf_mtus[i] << ((i % 2) * 16),
		    RTL930X_L3_IP6_MTU_CTRL(i));

	priv->intf_mtu_count[i]++;

	return mtu_id;
}

static int rtl930x_l3_mtu_del(struct rtl838x_switch_priv *priv, int mtu)
{
	int i;

	for (i = 0; i < MAX_INTF_MTUS; i++) {
		if (mtu == priv->intf_mtus[i])
			break;
	}
	if (i >= MAX_INTF_MTUS || !priv->intf_mtu_count[i]) {
		pr_err("%s: No MTU slot found for MTU: %d\n", __func__, mtu);
		return -EINVAL;
	}

	priv->intf_mtu_count[i]--;
}

// dal_longan_l3_intf_create, TODO: dal_longan_l3_intf_set
/*
 * Creates an interface for a route by setting up the HW tables in the SoC
 */
static int rtl930x_l3_intf_add(struct rtl838x_switch_priv *priv, struct rtl838x_l3_intf *intf)
{
	int i, intf_id, mtu_id;
	// number of MTU-values < 16384

	// Use the same IPv6 mtu as the ip4 mtu for this route if unset
	intf->ip6_mtu = intf->ip6_mtu ? intf->ip6_mtu : intf->ip4_mtu;

	mtu_id = rtl930x_l3_mtu_add(priv, intf->ip4_mtu);
	pr_info("%s: added mtu %d with mtu-id %d\n", __func__, intf->ip4_mtu, mtu_id);
	if (mtu_id < 0)
		return -ENOSPC;
	intf->ip4_mtu_id = mtu_id;
	intf->ip6_mtu_id = mtu_id;

	for (i = 0; i < MAX_INTERFACES; i++) {
		if (!priv->interfaces[i])
			break;
	}
	if (i >= MAX_INTERFACES) {
		pr_err("%s: cannot find free interface entry\n", __func__);
		return -EINVAL;
	}
	intf_id = i;
	priv->interfaces[i] = kzalloc(sizeof(struct rtl838x_l3_intf), GFP_KERNEL);
	if (!priv->interfaces[i]) {
		pr_err("%s: no memory to allocate new interface\n", __func__);
		return -ENOMEM;
	}
}

static void rtl930x_l3_dump(void)
{
	pr_info("L3 UC Routing enabled: %d\n", sw_r32(RTL930X_L3_IPUC_ROUTE_CTRL) & 0x1);
	pr_info("RTL930X_L3_IPUC_ROUTE_CTRL: %08x\n", sw_r32(RTL930X_L3_IPUC_ROUTE_CTRL));
}

#define DMAC_ID_DROP 0x7FFF
#define DMAC_ID_TRAP2CPU 0x7FFE
#define DMAC_ID_TRAP2MASTER 0x7FFD

static int rtl930x_l3_nexthop_update(struct rtl838x_switch_priv *priv,  __be32 ip_addr,
				     u64 mac, u16 vlan)
{
	struct rtl838x_route *r;
	int index;
	bool needs_del = false;
	bool needs_add = false;

	pr_info("%s: Setting up forwarding rule for ip %pI4\n", __func__, &ip_addr);
	// TODO: Lock while updating internal data structures and route configuration on SoC

	r = rtl930x_prefix_route_find_gw(priv, ip_addr);
	if (!r) {
		pr_info("Could not find route with GW-IP: %pI4\n", &ip_addr);
		return -1;
	}

	rtl930x_prefix_route_read(r->rt.id, &r->rt);
	pr_info("Found route valid: %d\n", r->rt.valid);
	pr_info("Route to %pI4, len %d\n", &r->rt.ip4_r, r->rt.prefix_len);

	needs_add = r->nh.mac == mac ? false : true;
	if (needs_add && r->nh.mac)
		needs_del = true;

	r->rt.valid = true;
	r->rt.next_hop = r->rt.id + 1;
	r->nh.mac = mac;
	r->nh.vid = vlan;
	r->nh.if_id = 1; // Default interface
	// Use 1-to-1 mapping between route-id, nexthop ID and mac index for now. TODO
	r->nh.id = r->rt.id + 1;
	r->nh.mac_id = r->rt.id + 1;

	// r->nh.mac set to hash index, L3 mac index from mac_idx
	if (needs_add)
		rtl930x_l2_nexthop_add(priv, &r->nh);

	//RT_ERR_HDL(_dal_longan_l3_nhDmac_set(unit, dstIdx, nh_mac_addr), errL3NhSet, ret);
	rtl930x_set_l3_egress_mac(r->nh.id, mac);

	// Use dmac_id as taken from L2 Forwarding DB (l2_id)
	pr_info("%s: dmac_id %d, interface %d\n", __func__, r->nh.l2_id, r->nh.if_id);
	rtl930x_set_l3_nexthop(r->nh.id, r->nh.l2_id, r->nh.if_id);

	if (r->rt.prefix_len < 32)
		rtl930x_prefix_route_write(r->rt.id, &r->rt);
	else
		rtl930x_host_route_write(r->rt.id, &r->rt);

	pr_info("%s: calling rtl930x_route_lookup_hw\n", __func__);
	index = rtl930x_route_lookup_hw(&r->rt);
	pr_info("Found route with index: %d\n", index);

	return 0;
}

// dmac_idx l3_egr_intf_idx RTK_L3_FLAG_WITH_NH_DMAC
static int rtl930x_l3_nexthop_add(struct rtl838x_switch_priv *priv, u64 mac, u16 vlan)
{
	struct rtl838x_nexthop nh;

	nh.mac = mac;
	nh.vid = vlan;

	rtl930x_l2_nexthop_add(priv, &nh);

	return 0;
}

static int rtl930x_port_ipv4_resolve(struct rtl838x_switch_priv *priv,
				     struct net_device *dev, __be32 ip_addr)
{
	struct neighbour *n = neigh_lookup(&arp_tbl, &ip_addr, dev);
	int err = 0;
	u64 mac;

	pr_info("%s: neighbour %08x\n", __func__, (u32)n);
	if (!n) {
		n = neigh_create(&arp_tbl, &ip_addr, dev);
		if (IS_ERR(n))
			return PTR_ERR(n);
	}
	pr_info("%s: neighbour now %08x\n", __func__, (u32)n);
	if (n)
		pr_info("%s: NUD-state %d\n", __func__, n->nud_state);
	/* If the neigh is already resolved, then go ahead and
	 * install the entry, otherwise start the ARP process to
	 * resolve the neigh.
	 */
	if (n->nud_state & NUD_VALID) {
		pr_info("%s: already valid\n", __func__);
		mac = ether_addr_to_u64(n->ha);
		pr_info("%s: resolved mac: %016llx\n", __func__, mac);
		rtl930x_l3_nexthop_update(priv, ip_addr, mac, 0);
	} else {
		pr_info("%s: need to wait\n", __func__);
		neigh_event_send(n, NULL);
	}

	neigh_release(n);
	return err;
}

/*
 * Is the lower network device a DSA slave network device of our RTL930X-switch?
 * Unfortunately we cannot just follow dev->dsa_prt as this is only set for the
 * DSA master device.
 */
static int rtl930x_port_is_under(const struct net_device * dev, struct rtl838x_switch_priv *priv)
{
	int i;

	pr_debug("%s\n", __func__);
	for (i = 0; i < RTL83XX_MAX_PORTS; i++) {
		if (!priv->ports[i].dp)
			continue;
		pr_debug("dp-port: %08x, dev: %08x\n", (u32)(priv->ports[i].dp->slave), (u32)dev);
		if (priv->ports[i].dp->slave == dev)
			return i;
	}
	return -1;
}

struct rtl930x_walk_data {
	struct rtl838x_switch_priv *priv;
	int port;
};

static int rtl930x_port_lower_walk(struct net_device *lower, void *_data)
{
	struct rtl930x_walk_data *data = _data;
	struct rtl838x_switch_priv *priv = data->priv;
	int ret = 0;
	int index;

	index = rtl930x_port_is_under(lower, priv);
	data->port = index;
	if (index >= 0) {
		pr_debug("Found DSA-port, index %d\n", index);
		ret = 1;
	}

	return ret;
}

int rtl930x_port_dev_lower_find(struct net_device *dev, struct rtl838x_switch_priv *priv)
{
	struct rtl930x_walk_data data;

	data.priv = priv;
	data.port = 0;

	netdev_walk_all_lower_dev(dev, rtl930x_port_lower_walk, &data);

	return data.port;
}

static int rtl930x_fib4_del(struct rtl838x_switch_priv *priv,
			    struct fib_entry_notifier_info *info)
{
	struct net_device *dev = fib_info_nh(info->fi, 0)->fib_nh_dev;
	struct dsa_port *dp = dev->dsa_ptr;
	u64 mac;

	pr_info("In %s, ip %pI4, len %d\n", __func__, &info->dst, info->dst_len);
	if (dp) {
		mac = ether_addr_to_u64(dev->dev_addr);
		pr_info("DSA-port: %08x, mac: %016llx\n", (u32)dp, mac);
	}
	if (dev) {
		mac = ether_addr_to_u64(dev->dev_addr);
		pr_info("DEV mac: %016llx\n", mac);
	}
	// nh->fib_nh_flags &= ~RTNH_F_OFFLOAD;
	return 0;
}

int rtl930x_fib4_add(struct rtl838x_switch_priv *priv,
		     struct fib_entry_notifier_info *info)
{
	struct fib_nh *nh = fib_info_nh(info->fi, 0);
	struct net_device *dev = fib_info_nh(info->fi, 0)->fib_nh_dev;
	struct dsa_port *dp = dev->dsa_ptr;
	u64 mac;
	int port;
	struct rtl838x_route *r;
//	struct rtl838x_route_info rt;
	int route_id;

	pr_info("In %s, ip %pI4, len %d\n", __func__, &info->dst, info->dst_len);
	if (!info->dst) {
		pr_info("Not offloading default route for now\n");
		return 0;
	}

	if (dp) {
		mac = ether_addr_to_u64(dev->dev_addr);
		pr_info("DSA-port: %08x, mac: %016llx\n", (u32)dp, mac);
	}
	if (dev) {
		mac = ether_addr_to_u64(dev->dev_addr);
		pr_info("DEV mac: %016llx\n", mac);
	}
	pr_info("GW: %pI4\n", &nh->fib_nh_gw4);

	pr_info("interface name: %s\n", dev->name);
	port = rtl930x_port_dev_lower_find(dev, priv);
	if (port < 0)
		return -1;

	// For now we only work with routes that have a gateway
	if (!nh->fib_nh_gw4)
		return 0;

	// Allocate route entry
	route_id = rtl930x_prefix_route_alloc(priv, false);
	if (route_id < 0) {
		pr_err("%s: No more free route entries\n", __func__);
		return -1;
	}

	// Configure route entry in route list
	r = rtl930x_prefix_route_find_id(priv, route_id);
	r->rt.type = 0; // IPv4 Unicast route
	r->rt.ip4_r = info->dst;
	r->rt.prefix_len = info->dst_len;
//	if (info->dst_len == 32)
//		r->rt.ip4_r += 0x100;
	r->nh.ip = nh->fib_nh_gw4;

	// We need to resolve the mac address of the GW
	rtl930x_port_ipv4_resolve(priv, dev, nh->fib_nh_gw4);

/*
	// test with ip route add 192.168.3.0/24 via 192.168.2.150
	// ip route add 192.168.4.1 via 192.168.2.150
	rt.type = 0; // IPv4 Unicast route
	rt.ip4_r = 0xc0a80500; // 192.168.5.0
	rt.prefix_len = 24;
	index = rtl930x_route_lookup_hw(&rt);
	pr_info("Found 192.168.5.0 route with index: %d\n", index);
*/

	nh->fib_nh_flags |= RTNH_F_OFFLOAD;

	return 0;
}

int rtl930x_l3_setup(struct rtl838x_switch_priv *priv)
{
	int i;
	struct rtl838x_l3_intf intf;

	// Setup MTU with id 0 for default interface
	for (i = 0; i < MAX_INTF_MTUS; i++)
		priv->intf_mtu_count[i] = priv->intf_mtus[i] = 0;
	priv->intf_mtu_count[0] = 0; // Needs to stay forever
	priv->intf_mtus[0] = DEFAULT_MTU;
	sw_w32_mask(0xffff, DEFAULT_MTU, RTL930X_L3_IP_MTU_CTRL(0));
	sw_w32_mask(0xffff, DEFAULT_MTU, RTL930X_L3_IP6_MTU_CTRL(0));
	priv->intf_mtus[1] = DEFAULT_MTU;
	sw_w32_mask(0xffff0000, DEFAULT_MTU << 16, RTL930X_L3_IP_MTU_CTRL(1));
	sw_w32_mask(0xffff0000, DEFAULT_MTU << 16, RTL930X_L3_IP6_MTU_CTRL(1));

	// Set up source MACs for the ports
	rtl930x_setup_port_macs(priv);

	// Configure the default hash algorithm
	sw_w32_mask(BIT(2), 0, RTL930X_L3_HOST_TBL_CTRL);  // Algorithm selection 0 = 0
	sw_w32_mask(0, BIT(3), RTL930X_L3_HOST_TBL_CTRL);  // Algorithm selection 1 = 1

	// Set up default egress interface 1
	intf.vid = 1;  // Or 0 ????? XXXXXXXXXXXXX
	intf.smac_idx = 1;
	intf.ip4_mtu_id = 1;
	intf.ip6_mtu_id = 1;
	intf.ttl_scope = 1; // TTL
	intf.hl_scope = 1;  // Hop Limit
	intf.ip4_icmp_redirect = intf.ip6_icmp_redirect = 0;
	intf.ip4_pbr_icmp_redirect = intf.ip6_pbr_icmp_redirect = 0;
	rtl930x_set_l3_egress_intf(1, &intf);

	// look at : dal_longan_l3_nullIntf_nhEntry_init

	// Enable IPv4 UC routing
	pr_info("%s: RTL930X_L3_IPUC_ROUTE_CTRL was: %08x\n", 
		__func__, sw_r32(RTL930X_L3_IPUC_ROUTE_CTRL));
	sw_w32_mask(0, 1, RTL930X_L3_IPUC_ROUTE_CTRL);
	// Enable IPv6 UC routing
	sw_w32_mask(0, 1, RTL930X_L3_IP6UC_ROUTE_CTRL);
	// Disable IPv4 MC routing
	sw_w32_mask(1, 0, RTL930X_L3_IPMC_ROUTE_CTRL);
	// Disable IPv6 MC routing
	sw_w32_mask(1, 0, RTL930X_L3_IP6MC_ROUTE_CTRL);

	return 0;
}

const struct rtl838x_reg rtl930x_reg = {
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
	.l2_port_new_salrn = rtl930x_l2_port_new_salrn,
	.l2_port_new_sa_fwd = rtl930x_l2_port_new_sa_fwd,
	.mir_ctrl = RTL930X_MIR_CTRL,
	.mir_dpm = RTL930X_MIR_DPM_CTRL,
	.mir_spm = RTL930X_MIR_SPM_CTRL,
	.mac_link_sts = RTL930X_MAC_LINK_STS,
	.mac_link_dup_sts = RTL930X_MAC_LINK_DUP_STS,
	.mac_link_spd_sts = rtl930x_mac_link_spd_sts,
	.mac_rx_pause_sts = RTL930X_MAC_RX_PAUSE_STS,
	.mac_tx_pause_sts = RTL930X_MAC_TX_PAUSE_STS,
	.read_l2_entry_using_hash = rtl930x_read_l2_entry_using_hash,
	.write_l2_entry_using_hash = rtl930x_write_l2_entry_using_hash,
	.read_cam = rtl930x_read_cam,
	.write_cam = rtl930x_write_cam,
	.vlan_port_egr_filter = RTL930X_VLAN_PORT_EGR_FLTR,
	.vlan_port_igr_filter = RTL930X_VLAN_PORT_IGR_FLTR(0),
	.vlan_port_pb = RTL930X_VLAN_PORT_PB_VLAN,
	.vlan_port_tag_sts_ctrl = RTL930X_VLAN_PORT_TAG_STS_CTRL,
	.trk_mbr_ctr = rtl930x_trk_mbr_ctr,
	.rma_bpdu_fld_pmask = RTL930X_RMA_BPDU_FLD_PMSK,
	.init_eee = rtl930x_init_eee,
	.port_eee_set = rtl930x_port_eee_set,
	.eee_port_ability = rtl930x_eee_port_ability,
	.fib4_del = rtl930x_fib4_del,
	.fib4_add = rtl930x_fib4_add,
	.l2_hash_seed = rtl930x_l2_hash_seed, 
	.l2_hash_key = rtl930x_l2_hash_key,
	.port_dev_lower_find = rtl930x_port_dev_lower_find,
	.l3_nexthop_update = rtl930x_l3_nexthop_update,
	.read_mcast_pmask = rtl930x_read_mcast_pmask,
	.write_mcast_pmask = rtl930x_write_mcast_pmask,
};
