// SPDX-License-Identifier: GPL-2.0-only

#include <linux/debugfs.h>
#include <linux/kernel.h>

#include <asm/mach-rtl838x/mach-rtl838x.h>
#include "rtl838x.h"

#define RTL838X_DRIVER_NAME "rtl838x"

static ssize_t rtl838x_common_read(char __user *buffer, size_t count,
					loff_t *ppos, unsigned int value)
{
	char *buf;
	ssize_t len;

	if (*ppos != 0)
		return 0;

	buf = kasprintf(GFP_KERNEL, "0x%08x\n", value);
	if (!buf)
		return -ENOMEM;

	if (count < strlen(buf)) {
		kfree(buf);
		return -ENOSPC;
	}

	len = simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));
	kfree(buf);

	return len;
}

static ssize_t rtl838x_common_write(const char __user *buffer, size_t count,
				 loff_t *ppos, unsigned int *value)
{
	char b[32];
	ssize_t len;
	int ret;

	if (*ppos != 0)
		return -EINVAL;

	if (count >= sizeof(b))
		return -ENOSPC;

	len = simple_write_to_buffer(b, sizeof(b) - 1, ppos,
				     buffer, count);
	if (len < 0)
		return len;

	b[len] = '\0';
	ret = kstrtouint(b, 16, value);
	if (ret)
		return -EIO;

	return len;
}

static ssize_t stp_state_read(struct file *filp, char __user *buffer, size_t count,
			     loff_t *ppos)
{
	struct rtl838x_port *p = filp->private_data;
	struct dsa_switch *ds = p->dp->ds;
	int value = rtl838x_port_get_stp_state(ds->priv, p->dp->index);

	if (value < 0)
		return -EINVAL;

	return rtl838x_common_read(buffer, count, ppos, (u32)value);
}

static ssize_t stp_state_write(struct file *filp, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct rtl838x_port *p = filp->private_data;
	u32 value;
	size_t res = rtl838x_common_write(buffer, count, ppos, &value);
	if (res < 0)
		return res;

	rtl838x_port_stp_state_set(p->dp->ds, p->dp->index, (u8)value);

	return res;
}

static const struct file_operations stp_state_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = stp_state_read,
	.write = stp_state_write,
};

static ssize_t age_out_read(struct file *filp, char __user *buffer, size_t count,
			     loff_t *ppos)
{
	struct rtl838x_port *p = filp->private_data;
	struct dsa_switch *ds = p->dp->ds;
	struct rtl838x_switch_priv *priv = ds->priv;
	int value = sw_r32(priv->r->l2_port_aging_out);

	if (value < 0)
		return -EINVAL;

	return rtl838x_common_read(buffer, count, ppos, (u32)value);
}

static ssize_t age_out_write(struct file *filp, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct rtl838x_port *p = filp->private_data;
	u32 value;
	size_t res = rtl838x_common_write(buffer, count, ppos, &value);
	if (res < 0)
		return res;

	rtl838x_fast_age(p->dp->ds, p->dp->index);

	return res;
}

static const struct file_operations age_out_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = age_out_read,
	.write = age_out_write,
};

static ssize_t port_egress_rate_read(struct file *filp, char __user *buffer, size_t count,
				loff_t *ppos)
{
	struct rtl838x_port *p = filp->private_data;
	struct dsa_switch *ds = p->dp->ds;
	struct rtl838x_switch_priv *priv = ds->priv;
	int value;
	if (priv->family_id == RTL8380_FAMILY_ID)
		value = rtl838x_get_egress_rate(priv, p->dp->index);
	else
		value = rtl839x_get_egress_rate(priv, p->dp->index);

	if (value < 0)
		return -EINVAL;

	return rtl838x_common_read(buffer, count, ppos, (u32)value);
}

static ssize_t port_egress_rate_write(struct file *filp, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct rtl838x_port *p = filp->private_data;
	struct dsa_switch *ds = p->dp->ds;
	struct rtl838x_switch_priv *priv = ds->priv;
	u32 value;
	size_t res = rtl838x_common_write(buffer, count, ppos, &value);
	if (res < 0)
		return res;

	if (priv->family_id == RTL8380_FAMILY_ID)
		rtl838x_set_egress_rate(priv, p->dp->index, value);
	else
		rtl839x_set_egress_rate(priv, p->dp->index, value);

	return res;
}

static const struct file_operations port_egress_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = port_egress_rate_read,
	.write = port_egress_rate_write,
};


static const struct debugfs_reg32 port_ctrl_regs[] = {
	{ .name = "port_isolation", .offset = RTL838X_PORT_ISO_CTRL(0), },
	{ .name = "mac_force_mode", .offset = RTL838X_MAC_FORCE_MODE_CTRL, },
};

void rtl838x_dbgfs_cleanup(struct rtl838x_switch_priv *priv)
{
	debugfs_remove_recursive(priv->dbgfs_dir);

//	kfree(priv->dbgfs_entries);
}

static int rtl838x_dbgfs_port_init(struct dentry *parent, struct rtl838x_switch_priv *priv,
				   int port)
{
	struct dentry *port_dir;
	struct debugfs_regset32 *port_ctrl_regset;

	port_dir = debugfs_create_dir(priv->ports[port].dp->name, parent);

	if (priv->family_id == RTL8380_FAMILY_ID){
		debugfs_create_x32("storm_rate_uc", 0644, port_dir,
				(u32 *)(RTL838X_SW_BASE + RTL838X_STORM_CTRL_PORT_UC(port)));

		debugfs_create_x32("storm_rate_mc", 0644, port_dir,
				(u32 *)(RTL838X_SW_BASE + RTL838X_STORM_CTRL_PORT_MC(port)));

		debugfs_create_x32("storm_rate_bc", 0644, port_dir,
				(u32 *)(RTL838X_SW_BASE + RTL838X_STORM_CTRL_PORT_BC(port)));
	} else {
		debugfs_create_x32("storm_rate_uc", 0644, port_dir,
				(u32 *)(RTL838X_SW_BASE + RTL839X_STORM_CTRL_PORT_UC_0(port)));

		debugfs_create_x32("storm_rate_mc", 0644, port_dir,
				(u32 *)(RTL838X_SW_BASE + RTL839X_STORM_CTRL_PORT_MC_0(port)));

		debugfs_create_x32("storm_rate_bc", 0644, port_dir,
				(u32 *)(RTL838X_SW_BASE + RTL839X_STORM_CTRL_PORT_BC_0(port)));
	}

	debugfs_create_u32("id", 0444, port_dir, &priv->ports[port].dp->index);

	port_ctrl_regset = devm_kzalloc(priv->dev, sizeof(*port_ctrl_regset), GFP_KERNEL);
	if (!port_ctrl_regset)
		return -ENOMEM;

	port_ctrl_regset->regs = port_ctrl_regs;
	port_ctrl_regset->nregs = ARRAY_SIZE(port_ctrl_regs);
	port_ctrl_regset->base = RTL838X_SW_BASE + (port << 2);
	debugfs_create_regset32("port_ctrl", 0400, port_dir, port_ctrl_regset);

	debugfs_create_file("stp_state", 0600, port_dir, &priv->ports[port], &stp_state_fops);
	debugfs_create_file("age_out", 0600, port_dir, &priv->ports[port], &age_out_fops);
	debugfs_create_file("port_egress_rate", 0600, port_dir, &priv->ports[port],
			    &port_egress_fops);
	return 0;
}

void rtl838x_dbgfs_init(struct rtl838x_switch_priv *priv)
{
	struct dentry *rtl838x_dir;
	struct dentry *port_dir;
	struct debugfs_regset32 *port_ctrl_regset;
	int ret, i;

	pr_info("%s called\n", __func__);
	rtl838x_dir = debugfs_lookup(RTL838X_DRIVER_NAME, NULL);
	if (!rtl838x_dir)
		rtl838x_dir = debugfs_create_dir(RTL838X_DRIVER_NAME, NULL);

	priv->dbgfs_dir = rtl838x_dir;

	debugfs_create_u32("soc", 0444, rtl838x_dir,
			   (u32 *)(RTL838X_SW_BASE + RTL838X_MODEL_NAME_INFO));

	/* Create one directory per port */
	for (i = 0; i < priv->cpu_port; i++) {
		if (priv->ports[i].phy) {
			ret = rtl838x_dbgfs_port_init(rtl838x_dir, priv, i);
			if (ret)
				goto err;
		}
	}

	/* Create directory for CPU-port */
	port_dir = debugfs_create_dir("cpu_port", rtl838x_dir);	port_ctrl_regset = devm_kzalloc(priv->dev, sizeof(*port_ctrl_regset), GFP_KERNEL);
	if (!port_ctrl_regset) {
		ret = -ENOMEM;
		goto err;
	}

	port_ctrl_regset->regs = port_ctrl_regs;
	port_ctrl_regset->nregs = ARRAY_SIZE(port_ctrl_regs);
	port_ctrl_regset->base = RTL838X_SW_BASE + (priv->cpu_port << 2);
	debugfs_create_regset32("port_ctrl", 0400, port_dir, port_ctrl_regset);
	debugfs_create_u8("id", 0444, port_dir, &priv->cpu_port);

	return;
err:
	rtl838x_dbgfs_cleanup(priv);
}
