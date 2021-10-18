// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Birger Koblitz <mail@birger-koblitz.de>
 * Copyright (C) 2020 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2020 John Crispin <john@phrozen.org>
 */

#include <linux/of_irq.h>
#include <linux/irqchip.h>
#include <linux/spinlock.h>
#include <linux/of_address.h>
#include <linux/irqchip/chained_irq.h>

/* Global Interrupt Mask Register */
#define RTL_ICTL_GIMR		0x00
/* Global Interrupt Status Register */
#define RTL_ICTL_GISR		0x04
/* Interrupt Routing Registers */
#define RTL_ICTL_IRR0		0x08
#define RTL_ICTL_IRR1		0x0c
#define RTL_ICTL_IRR2		0x10
#define RTL_ICTL_IRR3		0x14

#define REG(x)		(cd->base + x)

struct realtek_intc_chip_data {
	raw_spinlock_t	lock;
	void __iomem	*base;
};

static void realtek_ictl_unmask_irq(struct irq_data *i)
{
	struct realtek_intc_chip_data *cd = i->domain->host_data;
	unsigned long flags;
	u32 value;

	pr_debug("%s: cd %08x base %08x irq %d\n", __func__, (u32)cd, (u32)cd->base, i->hwirq);
	raw_spin_lock_irqsave(&cd->lock, flags);

	value = readl(REG(RTL_ICTL_GIMR));
	value |= BIT(i->hwirq);
	writel(value, REG(RTL_ICTL_GIMR));

	raw_spin_unlock_irqrestore(&cd->lock, flags);
}

static void realtek_ictl_mask_irq(struct irq_data *i)
{
	struct realtek_intc_chip_data *cd = i->domain->host_data;
	unsigned long flags;
	u32 value;

	raw_spin_lock_irqsave(&cd->lock, flags);

	value = readl(REG(RTL_ICTL_GIMR));
	value &= ~BIT(i->hwirq);
	writel(value, REG(RTL_ICTL_GIMR));

	raw_spin_unlock_irqrestore(&cd->lock, flags);
}

static struct irq_chip realtek_ictl_irq = {
	.name = "realtek-rtl-intc",
	.irq_mask = realtek_ictl_mask_irq,
	.irq_unmask = realtek_ictl_unmask_irq,
};

static int intc_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(hw, &realtek_ictl_irq, handle_level_irq);

	return 0;
}

static const struct irq_domain_ops irq_domain_ops = {
	.map = intc_map,
	.xlate = irq_domain_xlate_onecell,
};

static void realtek_irq_dispatch(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_domain *domain = irq_desc_get_handler_data(desc);
	struct realtek_intc_chip_data *cd = domain->host_data;
	unsigned int pending;

	chained_irq_enter(chip, desc);
	pending = readl(REG(RTL_ICTL_GIMR)) & readl(REG(RTL_ICTL_GISR));
	pr_debug("%s: with base %08x pending %08x\n", __func__, (u32)cd->base, pending);
	if (unlikely(!pending)) {
		spurious_interrupt();
		goto out;
	}
	generic_handle_irq(irq_find_mapping(domain, __ffs(pending)));

out:
	chained_irq_exit(chip, desc);
}

/*
 * SoC interrupts are cascaded to MIPS CPU interrupts according to the
 * interrupt-map in the device tree. Each SoC interrupt gets 4 bits for
 * the CPU interrupt in an Interrupt Routing Register. Max 32 SoC interrupts
 * thus go into 4 IRRs.
 */
static int __init map_interrupts(struct device_node *node, struct irq_domain *domain)
{
	struct realtek_intc_chip_data *cd = domain->host_data;
	struct device_node *cpu_ictl;
	const __be32 *imap;
	u32 imaplen, soc_int, cpu_int, tmp, regs[4];
	int ret, i, irr_regs[] = {
		RTL_ICTL_IRR3,
		RTL_ICTL_IRR2,
		RTL_ICTL_IRR1,
		RTL_ICTL_IRR0,
	};
	u8 mips_irqs_set;

	pr_info("%s, chip data %08x\n", __func__, (u32)cd);
	ret = of_property_read_u32(node, "#address-cells", &tmp);
	if (ret || tmp)
		return -EINVAL;

	imap = of_get_property(node, "interrupt-map", &imaplen);
	if (!imap || imaplen % 3)
		return -EINVAL;

	mips_irqs_set = 0;
	memset(regs, 0, sizeof(regs));
	for (i = 0; i < imaplen; i += 3 * sizeof(u32)) {
		soc_int = be32_to_cpup(imap);
		if (soc_int > 31)
			return -EINVAL;

		cpu_ictl = of_find_node_by_phandle(be32_to_cpup(imap + 1));
		if (!cpu_ictl)
			return -EINVAL;
		ret = of_property_read_u32(cpu_ictl, "#interrupt-cells", &tmp);
		if (ret || tmp != 1)
			return -EINVAL;
		of_node_put(cpu_ictl);

		cpu_int = be32_to_cpup(imap + 2);
		if (cpu_int > 7)
			return -EINVAL;

		if (!(mips_irqs_set & BIT(cpu_int))) {
			irq_set_chained_handler_and_data(cpu_int, realtek_irq_dispatch,
							 domain);
			mips_irqs_set |= BIT(cpu_int);
		}

		regs[(soc_int * 4) / 32] |= cpu_int << (soc_int * 4) % 32;
		imap += 3;
	}

	for (i = 0; i < 4; i++)
		writel(regs[i], REG(irr_regs[i]));

	return 0;
}

static int __init realtek_rtl_of_init(struct device_node *node, struct device_node *parent)
{
	struct irq_domain *domain;
	struct realtek_intc_chip_data *cd;
	int ret;

	cd = kzalloc(sizeof(*cd), GFP_KERNEL);
	if (!cd)
		return -ENOMEM;

	cd->base = of_iomap(node, 0);
	if (!cd->base)
		return -ENXIO;

	raw_spin_lock_init(&cd->lock);

	/* Disable all cascaded interrupts */
	writel(0, REG(RTL_ICTL_GIMR));
	pr_info("%s: new handler with base %08x\n", __func__, (u32)cd->base);

	domain = irq_domain_add_simple(node, 32, 0,
				       &irq_domain_ops, cd);

	pr_info("Domain data %08x\n", (u32)domain->host_data);
	ret = map_interrupts(node, domain);
	if (ret) {
		pr_err("invalid interrupt map\n");
		return ret;
	}

	return 0;
}

IRQCHIP_DECLARE(realtek_rtl_intc, "realtek,rtl-intc", realtek_rtl_of_init);
