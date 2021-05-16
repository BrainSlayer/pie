// SPDX-License-Identifier: GPL-2.0-only

#include <net/dsa.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <net/flow_offload.h>

#include <asm/mach-rtl838x/mach-rtl83xx.h>
#include "rtl83xx.h"
#include "rtl838x.h"

static int rtl83xx_configure_flower(struct rtl838x_switch_priv *priv,
				    struct flow_cls_offload * cls_flower, u32 flags)
{
	pr_info("In %s\n", __func__);
	return 0;
}

static int rtl83xx_delete_flower(struct rtl838x_switch_priv *priv,
				 struct flow_cls_offload * cls_flower, u32 flags)
{
	pr_info("In %s\n", __func__);
	return 0;
}

static int rtl83xx_stats_flower(struct rtl838x_switch_priv *priv,
				struct flow_cls_offload * cls_flower, u32 flags)
{
	pr_info("In %s\n", __func__);
	return 0;
}

static int rtl83xx_setup_tc_cls_flower(struct rtl838x_switch_priv *priv,
				       struct flow_cls_offload *cls_flower,
					u32 flags)
{
	pr_info("%s: %d\n", __func__, cls_flower->command);
	switch (cls_flower->command) {
	case FLOW_CLS_REPLACE:
		return rtl83xx_configure_flower(priv, cls_flower, flags);
	case FLOW_CLS_DESTROY:
		return rtl83xx_delete_flower(priv, cls_flower, flags);
	case FLOW_CLS_STATS:
		return rtl83xx_stats_flower(priv, cls_flower, flags);
	default:
		return -EOPNOTSUPP;
	}
}


static int rtl83xx_setup_tc_block_cb(enum tc_setup_type type, void *type_data,
				     void *cb_priv)
{
	u32 flags = 0;
	struct rtl838x_switch_priv *priv = cb_priv;

	pr_info("%s: %d\n", __func__, type);
	switch (type) {
	case TC_SETUP_CLSFLOWER:
		pr_info("%s: TC_SETUP_CLSFLOWER\n", __func__);
		return rtl83xx_setup_tc_cls_flower(priv, type_data, flags);
	default:
		return -EOPNOTSUPP;
	}
}

static LIST_HEAD(rtl83xx_block_cb_list);

int rtl83xx_setup_tc(struct net_device *dev, enum tc_setup_type type, void *type_data)
{
	struct rtl838x_switch_priv *priv;
	struct flow_block_offload *f = type_data;

	pr_info("%s: %d\n", __func__, type);

	if(!netdev_uses_dsa(dev)) {
		pr_info("%s: no DSA\n", __func__);
		return 0;
	}
	priv = dev->dsa_ptr->ds->priv;

	switch (type) {
	case TC_SETUP_BLOCK:
		pr_info("%s: setting up CB\n", __func__);
		f->unlocked_driver_cb = true;
		return flow_block_cb_setup_simple(type_data,
						  &rtl83xx_block_cb_list,
						  rtl83xx_setup_tc_block_cb,
						  priv, priv, true);
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}
