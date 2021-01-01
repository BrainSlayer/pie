// SPDX-License-Identifier: GPL-2.0-only
/*
 * linux/drivers/net/ethernet/rtl838x_eth.c
 * Copyright (C) 2020 B. Koblitz
 */

#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/module.h>
#include <linux/phylink.h>
#include <net/dsa.h>
#include <net/switchdev.h>
#include <asm/cacheflush.h>

#include <asm/mach-rtl838x/mach-rtl838x.h>
#include "rtl838x_eth.h"

extern struct rtl838x_soc_info soc_info;

/*
 * Maximum number of RX rings is 8, assigned by switch based on
 * packet/port priortity (not implemented)
 * Maximum number of TX rings is 2 (only ring 0 used)
 * RX ringlength needs to be at least 200, otherwise CPU and Switch
 * may gridlock.
 */
#define RXRINGS		32
#define RXRINGLEN	32
#define TXRINGS		2
// Should be minimum 160:
#define TXRINGLEN	32
#define NOTIFY_EVENTS	10
#define NOTIFY_BLOCKS	10
#define TX_EN		0x8
#define RX_EN		0x4
#define TX_EN_93XX	0x20
#define RX_EN_93XX	0x10
#define TX_DO		0x2
#define WRAP		0x2

#define RING_BUFFER	1600

#define RTL838X_STORM_CTRL_PORT_BC_EXCEED	(0x470C)
#define RTL839X_STORM_CTRL_PORT_BC_EXCEED	(0x180C)
#define RTL838X_STORM_CTRL_PORT_MC_EXCEED	(0x4710)
#define RTL839X_STORM_CTRL_PORT_MC_EXCEED	(0x1814)
#define RTL838X_STORM_CTRL_PORT_UC_EXCEED	(0x4714)
#define RTL839X_STORM_CTRL_PORT_UC_EXCEED	(0x181C)
#define RTL838X_ATK_PRVNT_STS			(0x5B1C)

struct p_hdr {
	uint8_t		*buf;
	uint16_t	reserved;
	uint16_t	size;		/* buffer size */
	uint16_t	offset;
	uint16_t	len;		/* pkt len */
	// TODO: Incorporate previous reserved word into cpu_tag below for 83xx SoCs
	uint16_t	cpu_tag[10];	// RTL83xx only use 10 bytes
	
} __packed __aligned(1);

struct n_event {
	uint32_t	type:2;
	uint32_t	fidVid:12;
	uint64_t	mac:48;
	uint32_t	slp:6;
	uint32_t	valid:1;
	uint32_t	reserved:27;
} __packed __aligned(1);

struct ring_b {
	uint32_t	rx_r[RXRINGS][RXRINGLEN];
	uint32_t	tx_r[TXRINGS][TXRINGLEN];
	struct	p_hdr	rx_header[RXRINGS][RXRINGLEN];
	struct	p_hdr	tx_header[TXRINGS][TXRINGLEN];
	uint32_t	c_rx[RXRINGS];
	uint32_t	c_tx[TXRINGS];
	uint8_t		rx_space[RXRINGS*RXRINGLEN*RING_BUFFER];
	uint8_t		tx_space[TXRINGLEN*RING_BUFFER];
};

struct notify_block {
	struct n_event	events[NOTIFY_EVENTS];
};

struct notify_b {
	struct notify_block	blocks[NOTIFY_BLOCKS];
	u32			reserved1[8];
	u32			ring[NOTIFY_BLOCKS];
	u32			reserved2[8];
};

inline void rtl838x_create_tx_header(struct p_hdr *h, int dest_port)
{
	if (dest_port > 0) {
		h->cpu_tag[0] = 0;
		h->cpu_tag[1] = 0x0400;
		h->cpu_tag[2] = 0x0200;
		h->cpu_tag[3] = 0x0000;
		h->cpu_tag[4] = (1 << dest_port) >> 16;
		h->cpu_tag[5] = (1 << dest_port) & 0xffff;
	}
}

inline void rtl839x_create_tx_header(struct p_hdr *h, int dest_port)
{
	if (dest_port > 0) {
		h->cpu_tag[0] = 0;
		h->cpu_tag[1] = 0x0100;
		h->cpu_tag[2] = ((1 << (dest_port - 32)) >> 16) | (1 << 21);
		h->cpu_tag[3] = (1 << (dest_port - 32)) & 0xffff;
		h->cpu_tag[4] = (1 << dest_port) >> 16;
		h->cpu_tag[5] = (1 << dest_port) & 0xffff;
	}
}

inline void rtl93xx_create_tx_header(struct p_hdr *h, int dest_port)
{
	h->cpu_tag[0] = 0x8000;
	h->cpu_tag[1] = 0;
	h->cpu_tag[2] = 0;
	h->cpu_tag[3] = 0;
	h->cpu_tag[4] = 0;
	h->cpu_tag[5] = 0;
	h->cpu_tag[6] = 0;
	h->cpu_tag[7] = 0xffff;
}

extern void rtl838x_fdb_sync(struct work_struct *work);

struct rtl838x_eth_priv {
	struct net_device *netdev;
	struct platform_device *pdev;
	void		*membase;
	spinlock_t	lock;
	struct mii_bus	*mii_bus;
	struct napi_struct napi;
	struct phylink *phylink;
	struct phylink_config phylink_config;
	u16 id;
	u16 family_id;
	const struct rtl838x_reg *r;
	u8 cpu_port;
	u8 port_mask;
	u8 tag_offset;
	u32 lastEvent;
};



extern int rtl838x_phy_init(struct rtl838x_eth_priv *priv);
extern int rtl838x_read_sds_phy(int phy_addr, int phy_reg);
extern int rtl839x_read_sds_phy(int phy_addr, int phy_reg);
extern int rtl839x_write_sds_phy(int phy_addr, int phy_reg, u16 v);

void rtl838x_update_cntr(int r, int released)
{
	// This feature is not available on RTL838x SoCs
}

void rtl839x_update_cntr(int r, int released)
{
	int pos;
	u32 reg = RTL839X_DMA_IF_RX_RING_CNTR + ((r >> 3) << 2);

	// TODO: Test
	pos = (r % 7) << 2;
	sw_w32_mask(0xf << pos, released << pos, reg);
}

void rtl930x_update_cntr(int r, int released)
{
	int pos = (r % 3) * 10;
	u32 reg = RTL930X_DMA_IF_RX_RING_CNTR + ((r / 3) << 2);
	u32 v = sw_r32(reg);

	v = (v >> pos) & 0x3ff;
	pr_debug("RX: Work done %d, old value: %d, pos %d, reg %04x\n", released, v, pos, reg);
	sw_w32_mask(0x3ff << pos, released << pos, reg);
	sw_w32(v, reg);
}

void rtl931x_update_cntr(int r, int released)
{
	int pos = (r % 3) * 10;
	u32 reg = RTL931X_DMA_IF_RX_RING_CNTR + ((r / 3) << 2);

	sw_w32_mask(0x3ff << pos, released << pos, reg);
}


/*
 * Discard the RX ring-buffers, called as part of the net-ISR
 * when the buffer runs over
 * Caller needs to hold priv->lock
 */
static void rtl838x_rb_cleanup(struct rtl838x_eth_priv *priv, int status)
{
	int r;
	u32	*last;
	struct p_hdr *h;
	struct ring_b *ring = priv->membase;

	for (r = 0; r < RXRINGS; r++) {
		pr_debug("In %s working on r: %d\n", __func__, r);
		last = (u32 *)KSEG1ADDR(sw_r32(priv->r->dma_if_rx_cur(r)));
		do {
			if ((ring->rx_r[r][ring->c_rx[r]] & 0x1))
				break;
			pr_debug("Got something: %d\n", ring->c_rx[r]);
			h = &ring->rx_header[r][ring->c_rx[r]];
			memset(h, 0, sizeof(struct p_hdr));
			h->buf = (u8 *)KSEG1ADDR(ring->rx_space
					+ r * RXRINGLEN * RING_BUFFER
					+ ring->c_rx[r] * RING_BUFFER);
			h->size = RING_BUFFER;
			/* make sure the header is visible to the ASIC */
			mb();

			ring->rx_r[r][ring->c_rx[r]] = KSEG1ADDR(h) | 0x1
				| (ring->c_rx[r] == (RXRINGLEN - 1) ? WRAP : 0x1);
			ring->c_rx[r] = (ring->c_rx[r] + 1) % RXRINGLEN;
		} while (&ring->rx_r[r][ring->c_rx[r]] != last);
	}
}

void rtl9300_dump(void)
{
	u32 status_rx_r = sw_r32(RTL930X_DMA_IF_INTR_RX_RUNOUT_STS);
	u32 status_rx = sw_r32(RTL930X_DMA_IF_INTR_RX_DONE_STS);
	u32 status_tx = sw_r32(RTL930X_DMA_IF_INTR_TX_DONE_STS);
	u32 mask_rx_r = sw_r32(RTL930X_DMA_IF_INTR_RX_RUNOUT_MSK);
	u32 mask_rx = sw_r32(RTL930X_DMA_IF_INTR_RX_DONE_MSK);
	u32 mask_tx = sw_r32(RTL930X_DMA_IF_INTR_TX_DONE_MSK);

	pr_info("In %s, status_tx: %08x, status_rx: %08x, status_rx_r: %08x\n",
		__func__, status_tx, status_rx, status_rx_r);
	pr_info("In %s, msk_tx: %08x, msk_rx: %08x, msk_rx_r: %08x\n",
		__func__, mask_tx, mask_rx, mask_rx_r);
	pr_info("DMA_IF_CTRL: %08x, MAC_PORT_CTRL(CPU): %08x\n",
		sw_r32(RTL930X_DMA_IF_CTRL), sw_r32(RTL930X_MAC_FORCE_MODE_CTRL + 28 * 4));
	pr_info("L2_E 0: %08x L2_E 1: %08x L2_E 2: %08x L2_E 3: %08x L2_E C:%08x\n",
		sw_r32(RTL930X_MAC_L2_PORT_CTRL + 0), sw_r32(RTL930X_MAC_L2_PORT_CTRL + 64),
		sw_r32(RTL930X_MAC_L2_PORT_CTRL + 128), sw_r32(RTL930X_MAC_L2_PORT_CTRL + 192),
		sw_r32(RTL930X_MAC_L2_PORT_CTRL + 28 * 64));
	pr_info("Current: RX-ptr: %08x\n", sw_r32(0xDF80));
}

struct fdb_update_work {
	struct work_struct work;
	struct net_device *ndev;
	u64 macs[NOTIFY_EVENTS + 1];
};

static void rtl839x_l2_notification_handler(struct rtl838x_eth_priv *priv)
{
	struct notify_b *nb = priv->membase + sizeof(struct ring_b);
	u32 e = priv->lastEvent;
	struct n_event *event;
	int i;
	u64 mac;
	struct fdb_update_work *w;

	while (!(nb->ring[e] & 1)) {
		w = kzalloc(sizeof(*w), GFP_ATOMIC);
		if (!w) {
			pr_err("Out of memory: %s", __func__);
			return;
		}
		INIT_WORK(&w->work, rtl838x_fdb_sync);

		for (i = 0; i < NOTIFY_EVENTS; i++) {
			event = &nb->blocks[e].events[i];
			if (!event->valid)
				continue;
			mac = event->mac;
			if (event->type)
				mac |= 1ULL << 63;
			w->ndev = priv->netdev;
			w->macs[i] = mac;
		}

		/* Hand the ring entry back to the switch */
		nb->ring[e] = nb->ring[e] | 1;
		e = (e + 1) % NOTIFY_BLOCKS;

		w->macs[i] = 0ULL;
		schedule_work(&w->work);
	}
	priv->lastEvent = e;
}

static bool rtl838x_rate_warn(void)
{
	bool triggered = false;
	u32 atk = sw_r32(RTL838X_ATK_PRVNT_STS);
	u32 storm_uc = sw_r32(RTL838X_STORM_CTRL_PORT_UC_EXCEED);
	u32 storm_mc = sw_r32(RTL838X_STORM_CTRL_PORT_MC_EXCEED);
	u32 storm_bc = sw_r32(RTL838X_STORM_CTRL_PORT_BC_EXCEED);

	if (storm_uc || storm_mc || storm_bc) {

		pr_warn("Storm control UC: %08x, MC: %08x, BC: %08x\n",
			storm_uc, storm_mc, storm_bc);

		sw_w32(storm_uc, RTL838X_STORM_CTRL_PORT_UC_EXCEED);
		sw_w32(storm_mc, RTL838X_STORM_CTRL_PORT_MC_EXCEED);
		sw_w32(storm_bc, RTL838X_STORM_CTRL_PORT_BC_EXCEED);

		triggered = true;
	}

	if (atk) {
		pr_debug("Attack prevention triggered: %08x\n", atk);
		sw_w32(atk, RTL838X_ATK_PRVNT_STS);
	}

	return triggered;
}

static bool rtl839x_rate_warn(void)
{
	bool triggered = false;
	u64 storm_uc = rtl839x_get_port_reg_be(RTL839X_STORM_CTRL_PORT_UC_EXCEED);
	u64 storm_mc = rtl839x_get_port_reg_be(RTL839X_STORM_CTRL_PORT_MC_EXCEED);
	u64 storm_bc = rtl839x_get_port_reg_be(RTL839X_STORM_CTRL_PORT_BC_EXCEED);

	if (storm_uc || storm_mc || storm_bc) {

		pr_warn("Storm control UC: %016llx, MC: %016llx, BC: %016llx\n",
			storm_uc, storm_mc, storm_bc);

		rtl839x_set_port_reg_be(storm_uc, RTL839X_STORM_CTRL_PORT_UC_EXCEED);
		rtl839x_set_port_reg_be(storm_mc, RTL839X_STORM_CTRL_PORT_MC_EXCEED);
		rtl839x_set_port_reg_be(storm_bc, RTL839X_STORM_CTRL_PORT_BC_EXCEED);

		triggered = true;
	}

	return triggered;
}

static irqreturn_t rtl838x_net_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	u32 status = sw_r32(priv->r->dma_if_intr_sts);
	bool triggered = false;

	spin_lock(&priv->lock);

	if (priv->family_id == RTL8390_FAMILY_ID)
		triggered = rtl839x_rate_warn();
	else
		triggered = rtl838x_rate_warn();

	/*  Ignore TX interrupt */
	if ((status & 0xf0000)) {
		/* Clear ISR */
		sw_w32(0x000f0000, priv->r->dma_if_intr_sts);
	}

	/* RX interrupt */
	if (status & 0x0ff00) {
		/* Disable RX interrupt */
		if (triggered)
			pr_info("RX\n");
		sw_w32_mask(0xff00, 0, priv->r->dma_if_intr_msk);
		sw_w32(0x0000ff00, priv->r->dma_if_intr_sts);
		napi_schedule(&priv->napi);
	}

	/* RX buffer overrun */
	if (status & 0x000ff) {
		pr_info("RX buffer overrun: status %x, mask: %x\n",
			 status, sw_r32(priv->r->dma_if_intr_msk));
		sw_w32(status, priv->r->dma_if_intr_sts);
		rtl838x_rb_cleanup(priv, status & 0xff);
	}

	if (priv->family_id == RTL8390_FAMILY_ID && status & 0x00100000) {
		sw_w32(0x00100000, priv->r->dma_if_intr_sts);
		rtl839x_l2_notification_handler(priv);
	}

	if (priv->family_id == RTL8390_FAMILY_ID && status & 0x00200000) {
		sw_w32(0x00200000, priv->r->dma_if_intr_sts);
		rtl839x_l2_notification_handler(priv);
	}

	if (priv->family_id == RTL8390_FAMILY_ID && status & 0x00400000) {
		sw_w32(0x00400000, priv->r->dma_if_intr_sts);
		rtl839x_l2_notification_handler(priv);
	}

	spin_unlock(&priv->lock);
	return IRQ_HANDLED;
}

static irqreturn_t rtl93xx_net_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	u32 status_rx_r = sw_r32(priv->r->dma_if_intr_rx_runout_sts);
	u32 status_rx = sw_r32(priv->r->dma_if_intr_rx_done_sts);
	u32 status_tx = sw_r32(priv->r->dma_if_intr_tx_done_sts);
	u32 mask;

	pr_debug("In %s, status_tx: %08x, status_rx: %08x, status_rx_r: %08x\n",
		__func__, status_tx, status_rx, status_rx_r);
	spin_lock(&priv->lock);

	/*  Ignore TX interrupt */
	if (status_tx) {
		/* Clear ISR */
		pr_debug("TX done\n");
		sw_w32(status_tx, priv->r->dma_if_intr_tx_done_sts);
	}

	/* RX interrupt */
	if (status_rx) {
		/* Disable RX interrupt */
		mask = sw_r32(priv->r->dma_if_intr_rx_done_msk);
		sw_w32(0, priv->r->dma_if_intr_rx_done_msk);
		sw_w32(status_rx, priv->r->dma_if_intr_rx_done_sts);
		if (mask) {
			pr_debug("NAPI scheduled\n");
			napi_schedule(&priv->napi);
		}
	}

	/* RX buffer overrun */
	if (status_rx_r) {
		pr_debug("RX buffer overrun: status %x, mask: %x\n",
			 status_rx_r, sw_r32(priv->r->dma_if_intr_rx_runout_msk));
		sw_w32(status_rx_r, priv->r->dma_if_intr_rx_runout_sts);
		rtl838x_rb_cleanup(priv, status_rx_r);
	}

	spin_unlock(&priv->lock);
	return IRQ_HANDLED;
}

static const struct rtl838x_reg rtl838x_reg = {
	.net_irq = rtl838x_net_irq,
	.mac_port_ctrl = rtl838x_mac_port_ctrl,
	.dma_if_intr_sts = RTL838X_DMA_IF_INTR_STS,
	.dma_if_intr_msk = RTL838X_DMA_IF_INTR_MSK,
	.dma_if_ctrl = RTL838X_DMA_IF_CTRL,
	.mac_force_mode_ctrl = rtl838x_mac_force_mode_ctrl,
	.dma_rx_base = rtl838x_dma_rx_base,
	.dma_tx_base = rtl838x_dma_tx_base,
	.dma_if_rx_ring_size = rtl838x_dma_if_rx_ring_size,
	.dma_if_rx_ring_cntr = rtl838x_dma_if_rx_ring_cntr,
	.dma_if_rx_cur = rtl838x_dma_if_rx_cur,
	.rst_glb_ctrl = RTL838X_RST_GLB_CTRL_0,
	.get_mac_link_sts = rtl838x_get_mac_link_sts,
	.get_mac_link_dup_sts = rtl838x_get_mac_link_dup_sts,
	.get_mac_link_spd_sts = rtl838x_get_mac_link_spd_sts,
	.get_mac_rx_pause_sts = rtl838x_get_mac_rx_pause_sts,
	.get_mac_tx_pause_sts = rtl838x_get_mac_tx_pause_sts,
	.mac = RTL838X_MAC,
	.l2_tbl_flush_ctrl = RTL838X_L2_TBL_FLUSH_CTRL,
	.update_cntr = rtl838x_update_cntr,
};

static const struct rtl838x_reg rtl839x_reg = {
	.net_irq = rtl838x_net_irq,
	.mac_port_ctrl = rtl839x_mac_port_ctrl,
	.dma_if_intr_sts = RTL839X_DMA_IF_INTR_STS,
	.dma_if_intr_msk = RTL839X_DMA_IF_INTR_MSK,
	.dma_if_ctrl = RTL839X_DMA_IF_CTRL,
	.mac_force_mode_ctrl = rtl839x_mac_force_mode_ctrl,
	.dma_rx_base = rtl839x_dma_rx_base,
	.dma_tx_base = rtl839x_dma_tx_base,
	.dma_if_rx_ring_size = rtl839x_dma_if_rx_ring_size,
	.dma_if_rx_ring_cntr = rtl839x_dma_if_rx_ring_cntr,
	.dma_if_rx_cur = rtl839x_dma_if_rx_cur,
	.rst_glb_ctrl = RTL839X_RST_GLB_CTRL,
	.get_mac_link_sts = rtl839x_get_mac_link_sts,
	.get_mac_link_dup_sts = rtl839x_get_mac_link_dup_sts,
	.get_mac_link_spd_sts = rtl839x_get_mac_link_spd_sts,
	.get_mac_rx_pause_sts = rtl839x_get_mac_rx_pause_sts,
	.get_mac_tx_pause_sts = rtl839x_get_mac_tx_pause_sts,
	.mac = RTL839X_MAC,
	.l2_tbl_flush_ctrl = RTL839X_L2_TBL_FLUSH_CTRL,
	.update_cntr = rtl839x_update_cntr,
};

static const struct rtl838x_reg rtl930x_reg = {
	.net_irq = rtl93xx_net_irq,
	.mac_port_ctrl = rtl930x_mac_port_ctrl,
	.dma_if_intr_rx_runout_sts = RTL930X_DMA_IF_INTR_RX_RUNOUT_STS,
	.dma_if_intr_rx_done_sts = RTL930X_DMA_IF_INTR_RX_DONE_STS,
	.dma_if_intr_tx_done_sts = RTL930X_DMA_IF_INTR_TX_DONE_STS,
	.dma_if_intr_rx_runout_msk = RTL930X_DMA_IF_INTR_RX_RUNOUT_MSK,
	.dma_if_intr_rx_done_msk = RTL930X_DMA_IF_INTR_RX_DONE_MSK,
	.dma_if_intr_tx_done_msk = RTL930X_DMA_IF_INTR_TX_DONE_MSK,
	.l2_ntfy_if_intr_sts = RTL930X_L2_NTFY_IF_INTR_STS,
	.l2_ntfy_if_intr_msk = RTL930X_L2_NTFY_IF_INTR_MSK,
	.dma_if_ctrl = RTL930X_DMA_IF_CTRL,
	.mac_force_mode_ctrl = rtl930x_mac_force_mode_ctrl,
	.dma_rx_base = rtl930x_dma_rx_base,
	.dma_tx_base = rtl930x_dma_tx_base,
	.dma_if_rx_ring_size = rtl930x_dma_if_rx_ring_size,
	.dma_if_rx_ring_cntr = rtl930x_dma_if_rx_ring_cntr,
	.dma_if_rx_cur = rtl930x_dma_if_rx_cur,
	.rst_glb_ctrl = RTL930X_RST_GLB_CTRL_0,
	.get_mac_link_sts = rtl930x_get_mac_link_sts,
	.get_mac_link_dup_sts = rtl930x_get_mac_link_dup_sts,
	.get_mac_link_spd_sts = rtl930x_get_mac_link_spd_sts,
	.get_mac_rx_pause_sts = rtl930x_get_mac_rx_pause_sts,
	.get_mac_tx_pause_sts = rtl930x_get_mac_tx_pause_sts,
	.mac = RTL930X_MAC_L2_ADDR_CTRL,
	.l2_tbl_flush_ctrl = RTL930X_L2_TBL_FLUSH_CTRL,
	.update_cntr = rtl930x_update_cntr,
};

static const struct rtl838x_reg rtl931x_reg = {
	.net_irq = rtl93xx_net_irq,
	.mac_port_ctrl = rtl931x_mac_port_ctrl,
	.dma_if_intr_rx_runout_sts = RTL931X_DMA_IF_INTR_RX_RUNOUT_STS,
	.dma_if_intr_rx_done_sts = RTL931X_DMA_IF_INTR_RX_DONE_STS,
	.dma_if_intr_tx_done_sts = RTL931X_DMA_IF_INTR_TX_DONE_STS,
	.dma_if_intr_rx_runout_msk = RTL931X_DMA_IF_INTR_RX_RUNOUT_MSK,
	.dma_if_intr_rx_done_msk = RTL931X_DMA_IF_INTR_RX_DONE_MSK,
	.dma_if_intr_tx_done_msk = RTL931X_DMA_IF_INTR_TX_DONE_MSK,
	.l2_ntfy_if_intr_sts = RTL931X_L2_NTFY_IF_INTR_STS,
	.l2_ntfy_if_intr_msk = RTL931X_L2_NTFY_IF_INTR_MSK,
	.dma_if_ctrl = RTL931X_DMA_IF_CTRL,
	.mac_force_mode_ctrl = rtl931x_mac_force_mode_ctrl,
	.dma_rx_base = rtl931x_dma_rx_base,
	.dma_tx_base = rtl931x_dma_tx_base,
	.dma_if_rx_ring_size = rtl931x_dma_if_rx_ring_size,
	.dma_if_rx_ring_cntr = rtl931x_dma_if_rx_ring_cntr,
	.dma_if_rx_cur = rtl931x_dma_if_rx_cur,
	.rst_glb_ctrl = RTL931X_RST_GLB_CTRL,
	.get_mac_link_sts = rtl931x_get_mac_link_sts,
	.get_mac_link_dup_sts = rtl931x_get_mac_link_dup_sts,
	.get_mac_link_spd_sts = rtl931x_get_mac_link_spd_sts,
	.get_mac_rx_pause_sts = rtl931x_get_mac_rx_pause_sts,
	.get_mac_tx_pause_sts = rtl931x_get_mac_tx_pause_sts,
	.mac = RTL931X_MAC_L2_ADDR_CTRL,
	.l2_tbl_flush_ctrl = RTL931X_L2_TBL_FLUSH_CTRL,
	.update_cntr = rtl931x_update_cntr,
};

static void rtl838x_hw_reset(struct rtl838x_eth_priv *priv)
{
	u32 int_saved, nbuf, v;
	int i, pos;
	
	pr_info("RESETTING %x, CPU_PORT %d\n", priv->family_id, priv->cpu_port);
	sw_w32_mask(0x3, 0, priv->r->mac_port_ctrl(priv->cpu_port));
	mdelay(100);

	/* Disable and clear interrupts */
	if (priv->family_id == RTL9300_FAMILY_ID || priv->family_id == RTL9310_FAMILY_ID) {
		sw_w32(0x00000000, priv->r->dma_if_intr_rx_runout_msk);
		sw_w32(0xffffffff, priv->r->dma_if_intr_rx_runout_sts);
		sw_w32(0x00000000, priv->r->dma_if_intr_rx_done_msk);
		sw_w32(0xffffffff, priv->r->dma_if_intr_rx_done_sts);
		sw_w32(0x00000000, priv->r->dma_if_intr_tx_done_msk);
		sw_w32(0x0000000f, priv->r->dma_if_intr_tx_done_sts);
	} else {
		sw_w32(0x00000000, priv->r->dma_if_intr_msk);
		sw_w32(0xffffffff, priv->r->dma_if_intr_sts);
	}

	if (priv->family_id == RTL8390_FAMILY_ID) {
		/* Preserve L2 notification and NBUF settings */
		int_saved = sw_r32(priv->r->dma_if_intr_msk);
		nbuf = sw_r32(RTL839X_DMA_IF_NBUF_BASE_DESC_ADDR_CTRL);

		/* Disable link change interrupt on RTL839x */
		sw_w32(0, RTL839X_IMR_PORT_LINK_STS_CHG);
		sw_w32(0, RTL839X_IMR_PORT_LINK_STS_CHG + 4);

		sw_w32(0x00000000, priv->r->dma_if_intr_msk);
		sw_w32(0xffffffff, priv->r->dma_if_intr_sts);
	}

	/* Reset NIC  */
	if (priv->family_id == RTL9300_FAMILY_ID || priv->family_id == RTL9310_FAMILY_ID)
		sw_w32(0x4, priv->r->rst_glb_ctrl);
	else
		sw_w32(0x8, priv->r->rst_glb_ctrl);

	do { /* Wait for reset of NIC and Queues done */
		udelay(20);
	} while (sw_r32(priv->r->rst_glb_ctrl) & 0xc);
	mdelay(100);

	/* Disable Head of Line */
	if (priv->family_id == RTL8390_FAMILY_ID)
		sw_w32(0xffffffff, RTL839X_DMA_IF_RX_RING_CNTR);
	if (priv->family_id == RTL9300_FAMILY_ID) {
		for (i = 0; i < RXRINGS; i++) {
			pos = (i % 3) * 10;
			sw_w32_mask(0x3ff << pos, 0, priv->r->dma_if_rx_ring_size(i));
			sw_w32_mask(0x3ff << pos, RXRINGLEN, priv->r->dma_if_rx_ring_cntr(i));
		}
	}

	/* Re-enable link change interrupt */
	if (priv->family_id == RTL8390_FAMILY_ID) {
		sw_w32(0xffffffff, RTL839X_ISR_PORT_LINK_STS_CHG);
		sw_w32(0xffffffff, RTL839X_ISR_PORT_LINK_STS_CHG + 4);
		sw_w32(0xffffffff, RTL839X_IMR_PORT_LINK_STS_CHG);
		sw_w32(0xffffffff, RTL839X_IMR_PORT_LINK_STS_CHG + 4);

		/* Restore notification settings: on RTL838x these bits are null */
		sw_w32_mask(7 << 20, int_saved & (7 << 20), priv->r->dma_if_intr_msk);
		sw_w32(nbuf, RTL839X_DMA_IF_NBUF_BASE_DESC_ADDR_CTRL);
	}
}

static void rtl838x_hw_ring_setup(struct rtl838x_eth_priv *priv)
{
	int i;
	struct ring_b *ring = priv->membase;

	for (i = 0; i < RXRINGS; i++)
		sw_w32(KSEG1ADDR(&ring->rx_r[i]), priv->r->dma_rx_base(i));

	for (i = 0; i < TXRINGS; i++)
		sw_w32(KSEG1ADDR(&ring->tx_r[i]), priv->r->dma_tx_base(i));
}

static void rtl838x_hw_en_rxtx(struct rtl838x_eth_priv *priv)
{
	/* Disable Head of Line features for all RX rings */
	sw_w32(0xffffffff, priv->r->dma_if_rx_ring_size(0));

	/* Truncate RX buffer to 0x640 (1600) bytes, pad TX */
	sw_w32(0x06400020, priv->r->dma_if_ctrl);

	/* Enable RX done, RX overflow and TX done interrupts */
	sw_w32(0xfffff, priv->r->dma_if_intr_msk);

	/* Enable DMA, engine expects empty FCS field */
	sw_w32_mask(0, RX_EN | TX_EN, priv->r->dma_if_ctrl);

	/* Restart TX/RX to CPU port */
	sw_w32_mask(0x0, 0x3, priv->r->mac_port_ctrl(priv->cpu_port));
	/* Set Speed, duplex, flow control
	 * FORCE_EN | LINK_EN | NWAY_EN | DUP_SEL
	 * | SPD_SEL = 0b10 | FORCE_FC_EN | PHY_MASTER_SLV_MANUAL_EN
	 * | MEDIA_SEL
	 */
	sw_w32(0x6192F, priv->r->mac_force_mode_ctrl(priv->cpu_port));
	/* allow CRC errors on CPU-port */
	sw_w32_mask(0, 0x8, priv->r->mac_port_ctrl(priv->cpu_port));
}

static void rtl839x_hw_en_rxtx(struct rtl838x_eth_priv *priv)
{
	/* Setup CPU-Port: RX Buffer */
	pr_info("DMA_IF_CTRL before: %08x", sw_r32(priv->r->dma_if_ctrl));
	sw_w32(0x0000c808, priv->r->dma_if_ctrl);

	/* Enable Notify, RX done, RX overflow and TX done interrupts */
	sw_w32(0x007fffff, priv->r->dma_if_intr_msk); // Notify IRQ!

	/* Enable DMA */
	sw_w32_mask(0, RX_EN | TX_EN, priv->r->dma_if_ctrl);

	/* Restart TX/RX to CPU port */
	sw_w32_mask(0x0, 0x3, priv->r->mac_port_ctrl(priv->cpu_port));

	/* CPU port joins Lookup Miss Flooding Portmask */
	// TODO: The code below should also work for the RTL838x
	sw_w32(0x28000, RTL839X_TBL_ACCESS_L2_CTRL);
	sw_w32_mask(0, 0x80000000, RTL839X_TBL_ACCESS_L2_DATA(0));
	sw_w32(0x38000, RTL839X_TBL_ACCESS_L2_CTRL);

	/* Force CPU port link up */
	sw_w32_mask(0, 3, priv->r->mac_force_mode_ctrl(priv->cpu_port));

}

static void rtl93xx_hw_en_rxtx(struct rtl838x_eth_priv *priv)
{
	int i, pos;
	u32 v;

	pr_info("DMA_IF_CTRL before: %08x", sw_r32(priv->r->dma_if_ctrl));
	/* Setup CPU-Port: RX Buffer truncated at 1600 Bytes */
	sw_w32(0x06400040, priv->r->dma_if_ctrl);

	for (i = 0; i < RXRINGS; i++) {
		pos = (i % 3) * 10;
		sw_w32_mask(0x3ff << pos, RXRINGLEN << pos, priv->r->dma_if_rx_ring_size(i));

		// Some SoCs have issues with missing underflow protection
		v = (sw_r32(priv->r->dma_if_rx_ring_cntr(i)) >> pos) & 0x3ff;
		sw_w32_mask(0x3ff << pos, v, priv->r->dma_if_rx_ring_cntr(i));
	}

	/* Enable Notify, RX done, RX overflow and TX done interrupts */
	sw_w32(0xffffffff, priv->r->dma_if_intr_rx_runout_msk);
	sw_w32(0xffffffff, priv->r->dma_if_intr_rx_done_msk);
	sw_w32(0x0000000f, priv->r->dma_if_intr_tx_done_msk);

	/* Enable DMA */
	sw_w32_mask(0, RX_EN_93XX | TX_EN_93XX, priv->r->dma_if_ctrl);
	pr_info("ENABLING TRAFFIC: %08x\n", priv->r->dma_if_ctrl);

	/* Restart TX/RX to CPU port */
	sw_w32_mask(0x0, 0x3, priv->r->mac_port_ctrl(priv->cpu_port));

	sw_w32_mask(0, BIT(priv->cpu_port), RTL930X_L2_UNKN_UC_FLD_PMSK);
	pr_info("MAC_FORCE_MODE WAS: %08x\n",
		sw_r32(priv->r->mac_force_mode_ctrl(priv->cpu_port)));
	sw_w32(0x217, priv->r->mac_force_mode_ctrl(priv->cpu_port));

}

static void rtl838x_setup_ring_buffer(struct ring_b *ring)
{
	int i, j;

	struct p_hdr *h;

	for (i = 0; i < RXRINGS; i++) {
		for (j = 0; j < RXRINGLEN; j++) {
			h = &ring->rx_header[i][j];
			memset(h, 0, sizeof(struct p_hdr));
			h->buf = (u8 *)KSEG1ADDR(ring->rx_space
					+ i * RXRINGLEN * RING_BUFFER
					+ j * RING_BUFFER);
			h->size = RING_BUFFER;
			/* All rings owned by switch, last one wraps */
			ring->rx_r[i][j] = KSEG1ADDR(h) | 1 | (j == (RXRINGLEN - 1) ? WRAP : 0);
		}
		ring->c_rx[i] = 0;
	}

	for (i = 0; i < TXRINGS; i++) {
		for (j = 0; j < TXRINGLEN; j++) {
			h = &ring->tx_header[i][j];
			memset(h, 0, sizeof(struct p_hdr));
			h->buf = (u8 *)KSEG1ADDR(ring->tx_space 
					+ i * TXRINGLEN * RING_BUFFER
					+ j * RING_BUFFER);
			h->size = RING_BUFFER;
			ring->tx_r[i][j] = KSEG1ADDR(&ring->tx_header[i][j]);
		}
		/* Last header is wrapping around */
		ring->tx_r[i][j-1] |= WRAP;
		ring->c_tx[i] = 0;
	}
}

static void rtl839x_setup_notify_ring_buffer(struct rtl838x_eth_priv *priv)
{
	int i;
	struct notify_b *b = priv->membase + sizeof(struct ring_b);

	for (i = 0; i < NOTIFY_BLOCKS; i++)
		b->ring[i] = KSEG1ADDR(&b->blocks[i]) | 1 | (i == (NOTIFY_BLOCKS - 1) ? WRAP : 0);

	sw_w32((u32) b->ring, RTL839X_DMA_IF_NBUF_BASE_DESC_ADDR_CTRL);
	sw_w32_mask(0x3ff << 2, 100 << 2, RTL839X_L2_NOTIFICATION_CTRL);

	/* Setup notification events */
	sw_w32_mask(0, 1 << 14, RTL839X_L2_CTRL_0); // RTL8390_L2_CTRL_0_FLUSH_NOTIFY_EN
	sw_w32_mask(0, 1 << 12, RTL839X_L2_NOTIFICATION_CTRL); // SUSPEND_NOTIFICATION_EN

	/* Enable Notification */
	sw_w32_mask(0, 1 << 0, RTL839X_L2_NOTIFICATION_CTRL);
	priv->lastEvent = 0;
}

static int rtl838x_eth_open(struct net_device *ndev)
{
	unsigned long flags;
	struct rtl838x_eth_priv *priv = netdev_priv(ndev);
	struct ring_b *ring = priv->membase;
	int err;

	pr_info("%s called: RX rings %d, TX rings %d\n", __func__, RXRINGS, TXRINGS);

	spin_lock_irqsave(&priv->lock, flags);
	rtl838x_hw_reset(priv);
	rtl838x_setup_ring_buffer(ring);
	if (priv->family_id == RTL8390_FAMILY_ID) {
		rtl839x_setup_notify_ring_buffer(priv);
		/* Make sure the ring structure is visible to the ASIC */
		mb();
		flush_cache_all();
	}

	rtl838x_hw_ring_setup(priv);
	err = request_irq(ndev->irq, priv->r->net_irq, IRQF_SHARED, ndev->name, ndev);
	if (err) {
		netdev_err(ndev, "%s: could not acquire interrupt: %d\n",
			   __func__, err);
		return err;
	}
	phylink_start(priv->phylink);

	napi_enable(&priv->napi);
	netif_start_queue(ndev);

	switch (priv->family_id) {
	case RTL8380_FAMILY_ID:
		rtl838x_hw_en_rxtx(priv);
		/* Trap IGMP traffic to CPU-Port */
		sw_w32(0x3, RTL838X_SPCL_TRAP_IGMP_CTRL);
		/* Flush learned FDB entries on link down of a port */
		sw_w32_mask(0, BIT(7), RTL838X_L2_CTRL_0);
		break;
	case RTL8390_FAMILY_ID:
		rtl839x_hw_en_rxtx(priv);
		sw_w32(0x3, RTL839X_SPCL_TRAP_IGMP_CTRL);
		/* Flush learned FDB entries on link down of a port */
		sw_w32_mask(0, BIT(7), RTL839X_L2_CTRL_0);
		break;
	case RTL9300_FAMILY_ID: /* fallthrough */
		rtl93xx_hw_en_rxtx(priv);
		/* Flush learned FDB entries on link down of a port */
		sw_w32_mask(0, BIT(7), RTL930X_L2_CTRL);
		sw_w32_mask(BIT(28), 0, RTL930X_L2_PORT_SABLK_CTRL);
		sw_w32_mask(BIT(28), 0, RTL930X_L2_PORT_DABLK_CTRL);
		break;

	case RTL9310_FAMILY_ID:
		rtl93xx_hw_en_rxtx(priv);
		
//		TODO: Add trapping of IGMP frames to CPU-port
		break;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void rtl838x_hw_stop(struct rtl838x_eth_priv *priv)
{
	u32 force_mac = priv->family_id == RTL8380_FAMILY_ID ? 0x6192D : 0x75;
	u32 clear_irq = priv->family_id == RTL8380_FAMILY_ID ? 0x000fffff : 0x007fffff;
	int i;

	// Disable RX/TX from/to CPU-port
	sw_w32_mask(0x3, 0, priv->r->mac_port_ctrl(priv->cpu_port));

	pr_info("X2\n");
	/* Disable traffic */
	if (priv->family_id == RTL9300_FAMILY_ID || priv->family_id == RTL9310_FAMILY_ID)
		sw_w32_mask(RX_EN_93XX | TX_EN_93XX, 0, priv->r->dma_if_ctrl);
	else
		sw_w32_mask(RX_EN | TX_EN, 0, priv->r->dma_if_ctrl);
	mdelay(200); // Test, whether this is needed

	/* Block all ports */
	if (priv->family_id == RTL8380_FAMILY_ID) {
		sw_w32(0x03000000, RTL838X_TBL_ACCESS_DATA_0(0));
		sw_w32(0x00000000, RTL838X_TBL_ACCESS_DATA_0(1));
		sw_w32(1 << 15 | 2 << 12, RTL838X_TBL_ACCESS_CTRL_0);
	}

	/* Flush L2 address cache */
	if (priv->family_id == RTL8380_FAMILY_ID) {
		for (i = 0; i <= priv->cpu_port; i++) {
			sw_w32(1 << 26 | 1 << 23 | i << 5, priv->r->l2_tbl_flush_ctrl);
			do { } while (sw_r32(priv->r->l2_tbl_flush_ctrl) & (1 << 26));
		}
	} else if (priv->family_id == RTL8390_FAMILY_ID) {
		for (i = 0; i <= priv->cpu_port; i++) {
			sw_w32(1 << 28 | 1 << 25 | i << 5, priv->r->l2_tbl_flush_ctrl);
			do { } while (sw_r32(priv->r->l2_tbl_flush_ctrl) & (1 << 28));
		}
	}
	// TODO: L2 flush register is 64 bit on RTL931X and 930X

	pr_info("X1\n");
	/* CPU-Port: Link down */
	if (priv->family_id == RTL8380_FAMILY_ID || priv->family_id == RTL8390_FAMILY_ID)
		sw_w32(force_mac, priv->r->mac_force_mode_ctrl(priv->cpu_port));
	else
		sw_w32_mask(0x3, 0, priv->r->mac_force_mode_ctrl(priv->cpu_port));
	mdelay(1000); // BUG: Must be 100

	pr_info("X3\n");
	/* Disable all TX/RX interrupts */
	if (priv->family_id == RTL9300_FAMILY_ID || priv->family_id == RTL9310_FAMILY_ID) {
		sw_w32(0x00000000, priv->r->dma_if_intr_rx_runout_msk);
		sw_w32(0xffffffff, priv->r->dma_if_intr_rx_runout_sts);
		sw_w32(0x00000000, priv->r->dma_if_intr_rx_done_msk);
		sw_w32(0xffffffff, priv->r->dma_if_intr_rx_done_sts);
		sw_w32(0x00000000, priv->r->dma_if_intr_tx_done_msk);
		sw_w32(0x0000000f, priv->r->dma_if_intr_tx_done_sts);
	} else {
		sw_w32(0x00000000, priv->r->dma_if_intr_msk);
		sw_w32(clear_irq, priv->r->dma_if_intr_sts);
	}

	pr_info("X4\n");
	/* Disable TX/RX DMA */
	sw_w32(0x00000000, priv->r->dma_if_ctrl);
	mdelay(200);
}

static int rtl838x_eth_stop(struct net_device *ndev)
{
	unsigned long flags;
	struct rtl838x_eth_priv *priv = netdev_priv(ndev);

	pr_info("in %s\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);
	phylink_stop(priv->phylink);
	pr_info("A\n");
	rtl838x_hw_stop(priv);
	pr_info("B\n");
	free_irq(ndev->irq, ndev);
	pr_info("C\n");
	napi_disable(&priv->napi);
	pr_info("D\n");
	netif_stop_queue(ndev);
	pr_info("E\n");
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void rtl839x_eth_set_multicast_list(struct net_device *ndev)
{
	if (!(ndev->flags & (IFF_PROMISC | IFF_ALLMULTI))) {
		sw_w32(0x0, RTL839X_RMA_CTRL_0);
		sw_w32(0x0, RTL839X_RMA_CTRL_1);
		sw_w32(0x0, RTL839X_RMA_CTRL_2);
		sw_w32(0x0, RTL839X_RMA_CTRL_3);
	}
	if (ndev->flags & IFF_ALLMULTI) {
		sw_w32(0x7fffffff, RTL839X_RMA_CTRL_0);
		sw_w32(0x7fffffff, RTL839X_RMA_CTRL_1);
		sw_w32(0x7fffffff, RTL839X_RMA_CTRL_2);
	}
	if (ndev->flags & IFF_PROMISC) {
		sw_w32(0x7fffffff, RTL839X_RMA_CTRL_0);
		sw_w32(0x7fffffff, RTL839X_RMA_CTRL_1);
		sw_w32(0x7fffffff, RTL839X_RMA_CTRL_2);
		sw_w32(0x3ff, RTL839X_RMA_CTRL_3);
	}
}

static void rtl838x_eth_set_multicast_list(struct net_device *ndev)
{
	struct rtl838x_eth_priv *priv = netdev_priv(ndev);

	if (priv->family_id == RTL8390_FAMILY_ID)
		return rtl839x_eth_set_multicast_list(ndev);

	if (priv->family_id == RTL9300_FAMILY_ID || priv->family_id == RTL9310_FAMILY_ID)
		return;
	
	if (!(ndev->flags & (IFF_PROMISC | IFF_ALLMULTI))) {
		sw_w32(0x0, RTL838X_RMA_CTRL_0);
		sw_w32(0x0, RTL838X_RMA_CTRL_1);
	}
	if (ndev->flags & IFF_ALLMULTI)
		sw_w32(0x1fffff, RTL838X_RMA_CTRL_0);
	if (ndev->flags & IFF_PROMISC) {
		sw_w32(0x1fffff, RTL838X_RMA_CTRL_0);
		sw_w32(0x7fff, RTL838X_RMA_CTRL_1);
	}
}

static void rtl838x_eth_tx_timeout(struct net_device *ndev)
{
	unsigned long flags;
	struct rtl838x_eth_priv *priv = netdev_priv(ndev);

	pr_info("in %s\n", __func__);
	spin_lock_irqsave(&priv->lock, flags);
	rtl838x_hw_stop(priv);
	rtl838x_hw_ring_setup(priv);
	rtl838x_hw_en_rxtx(priv);
	netif_trans_update(ndev);
	netif_start_queue(ndev);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static int rtl838x_eth_tx(struct sk_buff *skb, struct net_device *dev)
{
	int len, i;
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	struct ring_b *ring = priv->membase;
	uint32_t val;
	int ret;
	unsigned long flags;
	struct p_hdr *h;
	int dest_port = -1;

	spin_lock_irqsave(&priv->lock, flags);
	len = skb->len;

	/* Check for DSA tagging at the end of the buffer */
	if (netdev_uses_dsa(dev) && skb->data[len-4] == 0x80 && skb->data[len-3] > 0
			&& skb->data[len-3] < 28 &&  skb->data[len-2] == 0x10
			&&  skb->data[len-1] == 0x00) {
		/* Reuse tag space for CRC */
		dest_port = skb->data[len-3];
		len -= 4;
	}
	if (len < ETH_ZLEN)
		len = ETH_ZLEN;

	/* ASIC expects that packet includes CRC, so we extend by 4 bytes */
	len += 4;

	if (skb_padto(skb, len)) {
		ret = NETDEV_TX_OK;
		goto txdone;
	}

	/* We can send this packet if CPU owns the descriptor */
	if (!(ring->tx_r[0][ring->c_tx[0]] & 0x1)) {
		/* Set descriptor for tx */
		h = &ring->tx_header[0][ring->c_tx[0]];
		h->size = len;
		h->len = len;

		/* Create cpu_tag */
		if (priv->family_id == RTL8380_FAMILY_ID)
			rtl838x_create_tx_header(h, dest_port);
		else if (priv->family_id == RTL8390_FAMILY_ID)
			rtl839x_create_tx_header(h, dest_port);
		else if (priv->family_id == RTL9310_FAMILY_ID || priv->family_id == RTL9310_FAMILY_ID)
			rtl93xx_create_tx_header(h, dest_port);

		/* Copy packet data to tx buffer */
		memcpy((void *)KSEG1ADDR(h->buf), skb->data, len);
		/* Make sure packet data is visible to ASIC */
		mb(); /* wmb() probably works, too */

		/* Hand over to switch */
		ring->tx_r[0][ring->c_tx[0]] |= 0x1;

		/* BUG: before tx fetch, need to make sure right data is accessed
		 * This might not be necessary on newer RTL839x, though.
		 */
		if (priv->family_id != RTL9310_FAMILY_ID && priv->family_id != RTL9300_FAMILY_ID) {
			for (i = 0; i < 10; i++) {
				val = sw_r32(priv->r->dma_if_ctrl);
				if ((val & 0xc) == 0xc)
					break;
			}
		}

		/* Tell switch to send data */
		if (priv->family_id == RTL9310_FAMILY_ID || priv->family_id == RTL9300_FAMILY_ID) {
			// Ring ID == 0: Low priority, Ring ID = 1: High prio queue
			sw_w32_mask(0, BIT(2), priv->r->dma_if_ctrl);
			// TODO: For High priority: sw_w32_mask(0, BIT(3), priv->r->dma_if_ctrl);
		} else {
			sw_w32_mask(0, TX_DO, priv->r->dma_if_ctrl);
		}
		dev->stats.tx_packets++;
		dev->stats.tx_bytes += len;
		dev_kfree_skb(skb);
		ring->c_tx[0] = (ring->c_tx[0] + 1) % TXRINGLEN;
		ret = NETDEV_TX_OK;
	} else {
		dev_warn(&priv->pdev->dev, "Data is owned by switch\n");
		ret = NETDEV_TX_BUSY;
	}
txdone:
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}

static int rtl838x_hw_receive(struct net_device *dev, int r, int budget)
{
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	struct ring_b *ring = priv->membase;
	struct sk_buff *skb;
	unsigned long flags;
	int i, len, work_done = 0;
	u8 *data, *skb_data;
	unsigned int val;
	u32	*last;
	struct p_hdr *h;
	bool dsa = netdev_uses_dsa(dev);
	u16 h1, reason;

	spin_lock_irqsave(&priv->lock, flags);
	last = (u32 *)KSEG1ADDR(sw_r32(priv->r->dma_if_rx_cur(r)));

	if (r < 2)
		pr_debug("->>>>>>>>>>> RX %d: last %08x, cur %08x. CNTR: %08x SIZE: %08x\n",
			r, (u32) last, (u32)(&ring->rx_r[r][ring->c_rx[r]]),
			sw_r32(RTL930X_DMA_IF_RX_RING_CNTR), sw_r32(RTL930X_DMA_IF_RX_RING_SIZE)
		);
	do {

		if ((ring->rx_r[r][ring->c_rx[r]] & 0x1) ) {
			if (&ring->rx_r[r][ring->c_rx[r]] != last) {
			netdev_warn(dev, "WARNING Ring contention: ring %x, last %x, current %x," \
				"cPTR %x, ISR %x\n", r, (uint32_t)last,
				    (u32) &ring->rx_r[r][ring->c_rx[r]],
				    ring->rx_r[r][ring->c_rx[r]],
				sw_r32(priv->r->dma_if_intr_sts));
			}
			break;
		}

		h = &ring->rx_header[r][ring->c_rx[r]];
		data = (u8 *)KSEG1ADDR(h->buf);
		len = h->len;
		if (h->offset)
			pr_info("RX offset: %04x\n", h->offset);
		if (!len)
			break;
		work_done++;

		len -= 4; /* strip the CRC */
		/* Add 4 bytes for cpu_tag */
		if (dsa)
			len += 4;

		skb = alloc_skb(len + 4, GFP_KERNEL);
		skb_reserve(skb, NET_IP_ALIGN);

		if (likely(skb)) {
			/* BUG: Prevent bug on RTL838x SoCs*/
			if (priv->family_id == RTL8380_FAMILY_ID) {
				sw_w32(0xffffffff, priv->r->dma_if_rx_ring_size(0));
				for (i = 0; i < RXRINGS; i++) {
					/* Update each ring cnt */
					val = sw_r32(priv->r->dma_if_rx_ring_cntr(i));
					sw_w32(val, priv->r->dma_if_rx_ring_cntr(i));
				}
			}

			skb_data = skb_put(skb, len);
			/* Make sure data is visible */
			mb();
			memcpy(skb->data, data, len);
			/* Overwrite CRC with cpu_tag */
			if (dsa) {
				skb->data[len-4] = 0x80;
				h1 = h->cpu_tag[priv->tag_offset];
				if (priv->tag_offset)  // Works also on RTL9310
					skb->data[len-3] = h1 & priv->port_mask;
				else
					skb->data[len-3] = (h1 >> 8) & priv->port_mask;
				skb->data[len-2] = 0x10;
				skb->data[len-1] = 0x00;
				reason = h->cpu_tag[7] & 0x3f;
				pr_debug("Reason %d\n", reason);
			}

			skb->protocol = eth_type_trans(skb, dev);
			dev->stats.rx_packets++;
			dev->stats.rx_bytes += len;

			netif_receive_skb(skb);
		} else {
			if (net_ratelimit())
				dev_warn(&dev->dev, "low on memory - packet dropped\n");
			dev->stats.rx_dropped++;
		}

		/* Reset header structure */
		memset(h, 0, sizeof(struct p_hdr));
		h->buf = data;
		h->size = RING_BUFFER;

		ring->rx_r[r][ring->c_rx[r]]
			= KSEG1ADDR(h) | 0x1 | (ring->c_rx[r] == (RXRINGLEN-1) ? WRAP : 0x1);
		ring->c_rx[r] = (ring->c_rx[r] + 1) % RXRINGLEN;
	} while (&ring->rx_r[r][ring->c_rx[r]] != last && work_done < budget);

	// Update counters
	priv->r->update_cntr(r, 0);

	spin_unlock_irqrestore(&priv->lock, flags);
	return work_done;
}

static int rtl838x_poll_rx(struct napi_struct *napi, int budget)
{
	struct rtl838x_eth_priv *priv = container_of(napi, struct rtl838x_eth_priv, napi);
	int work_done = 0, r = 0;

	pr_debug("NAPI: %s\n", __func__);
	while (work_done < budget && r < RXRINGS) {
		work_done += rtl838x_hw_receive(priv->netdev, r, budget - work_done);
		r++;
	}

	if (work_done < budget) {
		napi_complete_done(napi, work_done);
		/* Enable RX interrupt */
		if (priv->family_id == RTL9300_FAMILY_ID || priv->family_id == RTL9310_FAMILY_ID)
			sw_w32(0xffffffff, priv->r->dma_if_intr_rx_done_msk);
		else
			sw_w32_mask(0, 0xfffff, priv->r->dma_if_intr_msk);
	}
	return work_done;
}


static void rtl838x_validate(struct phylink_config *config,
			 unsigned long *supported,
			 struct phylink_link_state *state)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	pr_info("In %s\n", __func__);

	if (!phy_interface_mode_is_rgmii(state->interface) &&
	    state->interface != PHY_INTERFACE_MODE_1000BASEX &&
	    state->interface != PHY_INTERFACE_MODE_MII &&
	    state->interface != PHY_INTERFACE_MODE_REVMII &&
	    state->interface != PHY_INTERFACE_MODE_GMII &&
	    state->interface != PHY_INTERFACE_MODE_QSGMII &&
	    state->interface != PHY_INTERFACE_MODE_INTERNAL &&
	    state->interface != PHY_INTERFACE_MODE_SGMII) {
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		pr_err("Unsupported interface: %d\n", state->interface);
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

	phylink_set(mask, 10baseT_Half);
	phylink_set(mask, 10baseT_Full);
	phylink_set(mask, 100baseT_Half);
	phylink_set(mask, 100baseT_Full);

	bitmap_and(supported, supported, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
}


static void rtl838x_mac_config(struct phylink_config *config,
			       unsigned int mode,
			       const struct phylink_link_state *state)
{
	/* This is only being called for the master device,
	 * i.e. the CPU-Port. We don't need to do anything.
	 */

	pr_info("In %s, mode %x\n", __func__, mode);
}

static void rtl838x_mac_an_restart(struct phylink_config *config)
{
	struct net_device *dev = container_of(config->dev, struct net_device, dev);
	struct rtl838x_eth_priv *priv = netdev_priv(dev);

	/* This works only on RTL838x chips */
	if (priv->family_id != RTL8380_FAMILY_ID)
		return;

	pr_info("In %s\n", __func__);
	/* Restart by disabling and re-enabling link */
	sw_w32(0x6192D, priv->r->mac_force_mode_ctrl(priv->cpu_port));
	mdelay(20);
	sw_w32(0x6192F, priv->r->mac_force_mode_ctrl(priv->cpu_port));
}

static int rtl838x_mac_pcs_get_state(struct phylink_config *config,
				  struct phylink_link_state *state)
{
	u32 speed;
	struct net_device *dev = container_of(config->dev, struct net_device, dev);
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	int port = priv->cpu_port;

	pr_info("In %s\n", __func__);

	state->link = priv->r->get_mac_link_sts(port) ? 1 : 0;
	state->duplex = priv->r->get_mac_link_dup_sts(port) ? 1 : 0;

	speed = priv->r->get_mac_link_spd_sts(port);
	switch (speed) {
	case 0:
		state->speed = SPEED_10;
		break;
	case 1:
		state->speed = SPEED_100;
		break;
		state->speed = SPEED_1000;
		break;
	default:
		state->speed = SPEED_UNKNOWN;
		break;
	}

	state->pause &= (MLO_PAUSE_RX | MLO_PAUSE_TX);
	if (priv->r->get_mac_rx_pause_sts(port))
		state->pause |= MLO_PAUSE_RX;
	if (priv->r->get_mac_tx_pause_sts(port))
		state->pause |= MLO_PAUSE_TX;

	return 1;
}

static void rtl838x_mac_link_down(struct phylink_config *config,
				  unsigned int mode,
				  phy_interface_t interface)
{
	struct net_device *dev = container_of(config->dev, struct net_device, dev);
	struct rtl838x_eth_priv *priv = netdev_priv(dev);

	pr_info("In %s\n", __func__);
	/* Stop TX/RX to port */
	sw_w32_mask(0x03, 0, priv->r->mac_port_ctrl(priv->cpu_port));
}

static void rtl838x_mac_link_up(struct phylink_config *config, unsigned int mode,
			    phy_interface_t interface,
			    struct phy_device *phy)
{
	struct net_device *dev = container_of(config->dev, struct net_device, dev);
	struct rtl838x_eth_priv *priv = netdev_priv(dev);

	pr_info("In %s\n", __func__);
	/* Restart TX/RX to port */
	sw_w32_mask(0, 0x03, priv->r->mac_port_ctrl(priv->cpu_port));
}

static void rtl838x_set_mac_hw(struct net_device *dev, u8 *mac)
{
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	unsigned long flags;

	pr_info("In %s\n", __func__);
	spin_lock_irqsave(&priv->lock, flags);

	sw_w32((mac[0] << 8) | mac[1], priv->r->mac);
	sw_w32((mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5], priv->r->mac + 4);

	if (priv->family_id == RTL8380_FAMILY_ID) {
		/* 2 more registers, ALE/MAC block */
		sw_w32((mac[0] << 8) | mac[1], RTL838X_MAC_ALE);
		sw_w32((mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5],
		       (RTL838X_MAC_ALE + 4));

		sw_w32((mac[0] << 8) | mac[1], RTL838X_MAC2);
		sw_w32((mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5],
		       RTL838X_MAC2 + 4);
	}
	spin_unlock_irqrestore(&priv->lock, flags);
}

static int rtl838x_set_mac_address(struct net_device *dev, void *p)
{
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	const struct sockaddr *addr = p;
	u8 *mac = (u8 *) (addr->sa_data);

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, addr->sa_data, ETH_ALEN);
	rtl838x_set_mac_hw(dev, mac);

	pr_info("Using MAC %08x%08x\n", sw_r32(priv->r->mac), sw_r32(priv->r->mac + 4));
	return 0;
}

static int rtl8390_init_mac(struct rtl838x_eth_priv *priv)
{
	// We will need to set-up EEE and the egress-rate limitation
	return 0;
}

static int rtl9300_init_mac(struct rtl838x_eth_priv *priv)
{
	// We will need to set-up EEE and the egress-rate limitation
	return 0;
}

static int rtl8380_init_mac(struct rtl838x_eth_priv *priv)
{
	int i;

	pr_info("%s\n", __func__);
	/* fix timer for EEE */
	sw_w32(0x5001411, RTL838X_EEE_TX_TIMER_GIGA_CTRL);
	sw_w32(0x5001417, RTL838X_EEE_TX_TIMER_GELITE_CTRL);

	/* Init VLAN */
	if (priv->id == 0x8382) {
		for (i = 0; i <= 28; i++)
			sw_w32(0, 0xd57c + i * 0x80);
	}
	if (priv->id == 0x8380) {
		for (i = 8; i <= 28; i++)
			sw_w32(0, 0xd57c + i * 0x80);
	}
	return 0;
}

static int rtl838x_get_link_ksettings(struct net_device *ndev,
				      struct ethtool_link_ksettings *cmd)
{
	struct rtl838x_eth_priv *priv = netdev_priv(ndev);

	pr_info("%s called\n", __func__);
	return phylink_ethtool_ksettings_get(priv->phylink, cmd);
}

static int rtl838x_set_link_ksettings(struct net_device *ndev,
				      const struct ethtool_link_ksettings *cmd)
{
	struct rtl838x_eth_priv *priv = netdev_priv(ndev);

	pr_info("%s called\n", __func__);
	return phylink_ethtool_ksettings_set(priv->phylink, cmd);
}

static int rtl838x_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	u32 val;
	int err;
	struct rtl838x_eth_priv *priv = bus->priv;

	if (mii_id >= 24 && mii_id <= 27 && priv->id == 0x8380)
		return rtl838x_read_sds_phy(mii_id, regnum);
	err = rtl838x_read_phy(mii_id, 0, regnum, &val);
	if (err)
		return err;
	return val;
}

static int rtl839x_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	u32 val;
	int err;
	struct rtl838x_eth_priv *priv = bus->priv;

	if (mii_id >= 48 && mii_id <= 49 && priv->id == 0x8393)
		return rtl839x_read_sds_phy(mii_id, regnum);

	err = rtl839x_read_phy(mii_id, 0, regnum, &val);
	if (err)
		return err;
	return val;
}

static int rtl930x_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	u32 val;
	int err;
//	struct rtl838x_eth_priv *priv = bus->priv;

//	if (mii_id >= 48 && mii_id <= 49 && priv->id == 0x8393)
//		return rtl839x_read_sds_phy(mii_id, regnum);

	err = rtl930x_read_phy(mii_id, 0, regnum, &val);
	if (err)
		return err;
	return val;
}
static int rtl931x_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	u32 val;
	int err;
//	struct rtl838x_eth_priv *priv = bus->priv;

//	if (mii_id >= 48 && mii_id <= 49 && priv->id == 0x8393)
//		return rtl839x_read_sds_phy(mii_id, regnum);

	err = rtl931x_read_phy(mii_id, 0, regnum, &val);
	if (err)
		return err;
	return val;
}

static int rtl838x_mdio_write(struct mii_bus *bus, int mii_id,
			      int regnum, u16 value)
{
	u32 offset = 0;
	struct rtl838x_eth_priv *priv = bus->priv;

	if (mii_id >= 24 && mii_id <= 27 && priv->id == 0x8380) {
		if (mii_id == 26)
			offset = 0x100;
		sw_w32(value, MAPLE_SDS4_FIB_REG0r + offset + (regnum << 2));
		return 0;
	}
	return rtl838x_write_phy(mii_id, 0, regnum, value);
}

static int rtl839x_mdio_write(struct mii_bus *bus, int mii_id,
			      int regnum, u16 value)
{
	struct rtl838x_eth_priv *priv = bus->priv;

	if (mii_id >= 48 && mii_id <= 49 && priv->id == 0x8393)
		return rtl839x_write_sds_phy(mii_id, regnum, value);

	return rtl839x_write_phy(mii_id, 0, regnum, value);
}

static int rtl930x_mdio_write(struct mii_bus *bus, int mii_id,
			      int regnum, u16 value)
{
//	struct rtl838x_eth_priv *priv = bus->priv;

//	if (mii_id >= 48 && mii_id <= 49 && priv->id == 0x8393)
//		return rtl839x_write_sds_phy(mii_id, regnum, value);

	return rtl930x_write_phy(mii_id, 0, regnum, value);
}

static int rtl931x_mdio_write(struct mii_bus *bus, int mii_id,
			      int regnum, u16 value)
{
//	struct rtl838x_eth_priv *priv = bus->priv;

//	if (mii_id >= 48 && mii_id <= 49 && priv->id == 0x8393)
//		return rtl839x_write_sds_phy(mii_id, regnum, value);

	return rtl931x_write_phy(mii_id, 0, regnum, value);
}

static int rtl838x_mdio_reset(struct mii_bus *bus)
{
	pr_info("%s called\n", __func__);
	/* Disable MAC polling the PHY so that we can start configuration */
	sw_w32(0x00000000, RTL838X_SMI_POLL_CTRL);

	/* Enable PHY control via SoC */
	sw_w32_mask(0, 1 << 15, RTL838X_SMI_GLB_CTRL);

	// Probably should reset all PHYs here...
	return 0;
}

static int rtl839x_mdio_reset(struct mii_bus *bus)
{
	return 0;

	pr_info("%s called\n", __func__);
	/* BUG: The following does not work, but should! */
	/* Disable MAC polling the PHY so that we can start configuration */
	sw_w32(0x00000000, RTL839X_SMI_PORT_POLLING_CTRL);
	sw_w32(0x00000000, RTL839X_SMI_PORT_POLLING_CTRL + 4);
	/* Disable PHY polling via SoC */
	sw_w32_mask(1 << 7, 0, RTL839X_SMI_GLB_CTRL);

	// Probably should reset all PHYs here...
	return 0;
}

static int rtl931x_mdio_reset(struct mii_bus *bus)
{
	sw_w32(0x00000000, RTL931X_SMI_PORT_POLLING_CTRL);
	sw_w32(0x00000000, RTL931X_SMI_PORT_POLLING_CTRL + 4);

	pr_info("%s called\n", __func__);

	return 0;
}

static int rtl930x_mdio_reset(struct mii_bus *bus)
{
	int i;
	int pos;

	pr_info("RTL930X_SMI_PORT0_15_POLLING_SEL %08x 16-27: %08x\n",
		sw_r32(RTL930X_SMI_PORT0_15_POLLING_SEL),
		sw_r32(RTL930X_SMI_PORT16_27_POLLING_SEL));
	// RTL930X_SMI_PORT0_15_POLLING_SEL 55550000 16-27: 00f9aaaa
	// i.e SMI=0 for all ports
	for (i = 0; i < 5; i++)
		pr_info("port phy: %08x\n", sw_r32(RTL930X_SMI_PORT0_5_ADDR + i *4));

	// 1-to-1 mapping of port to phy-address
	for (i = 0; i < 24; i++) {
		pos = (i % 6) * 5;
		sw_w32_mask(0x1f << pos, i << pos, RTL930X_SMI_PORT0_5_ADDR + (i / 6) * 4);
	}
	// ports 24 and 25 have PHY addresses 8 and 9, ports 26/27 PHY 26/27
	sw_w32(8 | 9 << 5 | 26 << 10 | 27 << 15, RTL930X_SMI_PORT0_5_ADDR + 4 * 4);

	return 0;
}

static int rtl838x_mdio_init(struct rtl838x_eth_priv *priv)
{
	struct device_node *mii_np;
	int ret;

	pr_info("%s called\n", __func__);
	mii_np = of_get_child_by_name(priv->pdev->dev.of_node, "mdio-bus");

	if (!mii_np) {
		dev_err(&priv->pdev->dev, "no %s child node found", "mdio-bus");
		return -ENODEV;
	}

	if (!of_device_is_available(mii_np)) {
		ret = -ENODEV;
		goto err_put_node;
	}

	priv->mii_bus = devm_mdiobus_alloc(&priv->pdev->dev);
	if (!priv->mii_bus) {
		ret = -ENOMEM;
		goto err_put_node;
	}

	switch(priv->family_id) {
	case RTL8380_FAMILY_ID:
		priv->mii_bus->name = "rtl838x-eth-mdio";
		priv->mii_bus->read = rtl838x_mdio_read;
		priv->mii_bus->write = rtl838x_mdio_write;
		priv->mii_bus->reset = rtl838x_mdio_reset;
		break;
	case RTL8390_FAMILY_ID:
		priv->mii_bus->name = "rtl839x-eth-mdio";
		priv->mii_bus->read = rtl839x_mdio_read;
		priv->mii_bus->write = rtl839x_mdio_write;
		priv->mii_bus->reset = rtl839x_mdio_reset;
		break;
	case RTL9300_FAMILY_ID:
		priv->mii_bus->name = "rtl930x-eth-mdio";
		priv->mii_bus->read = rtl930x_mdio_read;
		priv->mii_bus->write = rtl930x_mdio_write;
		priv->mii_bus->reset = rtl930x_mdio_reset;
		break;
	case RTL9310_FAMILY_ID:
		priv->mii_bus->name = "rtl931x-eth-mdio";
		priv->mii_bus->read = rtl931x_mdio_read;
		priv->mii_bus->write = rtl931x_mdio_write;
		priv->mii_bus->reset = rtl931x_mdio_reset;
		break;
	
	}
	priv->mii_bus->priv = priv;
	priv->mii_bus->parent = &priv->pdev->dev;

	snprintf(priv->mii_bus->id, MII_BUS_ID_SIZE, "%pOFn", mii_np);
	ret = of_mdiobus_register(priv->mii_bus, mii_np);

err_put_node:
	of_node_put(mii_np);
	return ret;
}

static int rtl838x_mdio_remove(struct rtl838x_eth_priv *priv)
{
	pr_info("%s called\n", __func__);
	if (!priv->mii_bus)
		return 0;

	mdiobus_unregister(priv->mii_bus);
	mdiobus_free(priv->mii_bus);

	return 0;
}

static const struct net_device_ops rtl838x_eth_netdev_ops = {
	.ndo_open = rtl838x_eth_open,
	.ndo_stop = rtl838x_eth_stop,
	.ndo_start_xmit = rtl838x_eth_tx,
	.ndo_set_mac_address = rtl838x_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_rx_mode = rtl838x_eth_set_multicast_list,
	.ndo_tx_timeout = rtl838x_eth_tx_timeout,
};

static const struct phylink_mac_ops rtl838x_phylink_ops = {
	.validate = rtl838x_validate,
	.mac_link_state = rtl838x_mac_pcs_get_state,
	.mac_an_restart = rtl838x_mac_an_restart,
	.mac_config = rtl838x_mac_config,
	.mac_link_down = rtl838x_mac_link_down,
	.mac_link_up = rtl838x_mac_link_up,
};

static const struct ethtool_ops rtl838x_ethtool_ops = {
	.get_link_ksettings     = rtl838x_get_link_ksettings,
	.set_link_ksettings     = rtl838x_set_link_ksettings,
};

static int __init rtl838x_eth_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct device_node *dn = pdev->dev.of_node;
	struct rtl838x_eth_priv *priv;
	struct resource *res, *mem;
	const void *mac;
	phy_interface_t phy_mode;
	struct phylink *phylink;
	int err = 0;

	pr_info("Probing RTL838X eth device pdev: %x, dev: %x\n",
		(u32)pdev, (u32)(&(pdev->dev)));

	if (!dn) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}

	dev = alloc_etherdev(sizeof(struct rtl838x_eth_priv));
	if (!dev) {
		err = -ENOMEM;
		goto err_free;
	}
	SET_NETDEV_DEV(dev, &pdev->dev);
	priv = netdev_priv(dev);

	/* obtain buffer memory space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		mem = devm_request_mem_region(&pdev->dev, res->start,
			resource_size(res), res->name);
		if (!mem) {
			dev_err(&pdev->dev, "cannot request memory space\n");
			err = -ENXIO;
			goto err_free;
		}

		dev->mem_start = mem->start;
		dev->mem_end   = mem->end;
	} else {
		dev_err(&pdev->dev, "cannot request IO resource\n");
		err = -ENXIO;
		goto err_free;
	}

	/* Allocate buffer memory */
	priv->membase = dmam_alloc_coherent(&pdev->dev,
				sizeof(struct ring_b) + sizeof(struct notify_b),
				(void *)&dev->mem_start, GFP_KERNEL);
	if (!priv->membase) {
		dev_err(&pdev->dev, "cannot allocate DMA buffer\n");
		err = -ENOMEM;
		goto err_free;
	}

	spin_lock_init(&priv->lock);

	/* obtain device IRQ number */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot obtain IRQ, using default 24\n");
		dev->irq = 24;
	} else {
		dev->irq = res->start;
	}
	dev->ethtool_ops = &rtl838x_ethtool_ops;

	priv->id = soc_info.id;
	priv->family_id = soc_info.family;
	if (priv->id) {
		pr_info("Found SoC ID: %4x: %s, family %x\n",
			priv->id, soc_info.name, priv->family_id);
	} else {
		pr_err("Unknown chip id (%04x)\n", priv->id);
		return -ENODEV;
	}

	switch (priv->family_id) {
	case RTL8380_FAMILY_ID:
		priv->cpu_port = RTL838X_CPU_PORT;
		priv->r = &rtl838x_reg;
		priv->port_mask = 0x1f;
		priv->tag_offset = 1;
		rtl8380_init_mac(priv);
		break;
	case RTL8390_FAMILY_ID:
		priv->cpu_port = RTL839X_CPU_PORT;
		priv->r = &rtl839x_reg;
		priv->port_mask = 0x3f;
		priv->tag_offset = 1;
		rtl8390_init_mac(priv);
		break;
	case RTL9300_FAMILY_ID:
		priv->cpu_port = RTL930X_CPU_PORT;
		priv->r = &rtl930x_reg;
		priv->port_mask = 0x1f;
		priv->tag_offset = 0;
		rtl9300_init_mac(priv);
		break;
	case RTL9310_FAMILY_ID:
		priv->cpu_port = RTL931X_CPU_PORT;
		priv->r = &rtl931x_reg;
		priv->port_mask = 0x3f;
		priv->tag_offset = 0;
		rtl9300_init_mac(priv);
		break;
	}

	rtl8380_init_mac(priv);

	/* try to get mac address in the following order:
	 * 1) from device tree data
	 * 2) from internal registers set by bootloader
	 */
	mac = of_get_mac_address(pdev->dev.of_node);
	if (!IS_ERR(mac)) {
		memcpy(dev->dev_addr, mac, ETH_ALEN);
		rtl838x_set_mac_hw(dev, (u8 *)mac);
	} else {
		pr_info("Setting DEVICE MAC\n");
		sw_w32(0x0000bccf, RTL930X_MAC_L2_ADDR_CTRL);
		sw_w32(0x4fbe0b91, RTL930X_MAC_L2_ADDR_CTRL + 4);
		dev->dev_addr[0] = (sw_r32(priv->r->mac) >> 8) & 0xff;
		dev->dev_addr[1] = sw_r32(priv->r->mac) & 0xff;
		dev->dev_addr[2] = (sw_r32(priv->r->mac + 4) >> 24) & 0xff;
		dev->dev_addr[3] = (sw_r32(priv->r->mac + 4) >> 16) & 0xff;
		dev->dev_addr[4] = (sw_r32(priv->r->mac + 4) >> 8) & 0xff;
		dev->dev_addr[5] = sw_r32(priv->r->mac + 4) & 0xff;
	}
	/* if the address is invalid, use a random value */
	if (!is_valid_ether_addr(dev->dev_addr)) {
		struct sockaddr sa = { AF_UNSPEC };

		netdev_warn(dev, "Invalid MAC address, using random\n");
		eth_hw_addr_random(dev);
		memcpy(sa.sa_data, dev->dev_addr, ETH_ALEN);
		if (rtl838x_set_mac_address(dev, &sa))
			netdev_warn(dev, "Failed to set MAC address.\n");
	}
	pr_info("Using MAC %08x%08x\n", sw_r32(priv->r->mac),
					sw_r32(priv->r->mac + 4));
	strcpy(dev->name, "eth%d");
	dev->netdev_ops = &rtl838x_eth_netdev_ops;
	priv->pdev = pdev;
	priv->netdev = dev;

	err = rtl838x_mdio_init(priv);
	if (err)
		goto err_free;

	err = register_netdev(dev);
	if (err)
		goto err_free;

	netif_napi_add(dev, &priv->napi, rtl838x_poll_rx, 64);
	platform_set_drvdata(pdev, dev);

	phy_mode = of_get_phy_mode(dn);
	if (phy_mode < 0) {
		dev_err(&pdev->dev, "incorrect phy-mode\n");
		err = -EINVAL;
		goto err_free;
	}
	priv->phylink_config.dev = &dev->dev;
	priv->phylink_config.type = PHYLINK_NETDEV;

	phylink = phylink_create(&priv->phylink_config, pdev->dev.fwnode,
				 phy_mode, &rtl838x_phylink_ops);
	if (IS_ERR(phylink)) {
		err = PTR_ERR(phylink);
		goto err_free;
	}
	priv->phylink = phylink;

	return 0;

err_free:
	pr_err("Error setting up netdev, freeing it again.\n");
	free_netdev(dev);
	return err;
}

static int rtl838x_eth_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct rtl838x_eth_priv *priv = netdev_priv(dev);

	if (dev) {
		pr_info("Removing platform driver for rtl838x-eth\n");
		rtl838x_mdio_remove(priv);
		rtl838x_hw_stop(priv);
		netif_stop_queue(dev);
		netif_napi_del(&priv->napi);
		unregister_netdev(dev);
		free_netdev(dev);
	}
	return 0;
}

static const struct of_device_id rtl838x_eth_of_ids[] = {
	{ .compatible = "realtek,rtl838x-eth"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtl838x_eth_of_ids);

static struct platform_driver rtl838x_eth_driver = {
	.probe = rtl838x_eth_probe,
	.remove = rtl838x_eth_remove,
	.driver = {
		.name = "rtl838x-eth",
		.pm = NULL,
		.of_match_table = rtl838x_eth_of_ids,
	},
};

module_platform_driver(rtl838x_eth_driver);

MODULE_AUTHOR("B. Koblitz");
MODULE_DESCRIPTION("RTL838X SoC Ethernet Driver");
MODULE_LICENSE("GPL");
