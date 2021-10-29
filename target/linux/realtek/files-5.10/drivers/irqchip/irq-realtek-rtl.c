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

#define N_CPU_IRQ	2

#define REG(x)		(base[cpu] + x)

void __iomem	*base[2];
char cpu_map[32];

static DEFINE_RAW_SPINLOCK(realtek_ictl_lock);

static void realtek_ictl_unmask_irq(struct irq_data *i)
{
	unsigned long flags;
	u32 value;
	int cpu;

/*	if (i->hwirq != 10)
		pr_debug("%s: cd %08x base %08x irq %lu\n",
			__func__, (u32)cd, (u32)cd->base[cpu], i->hwirq); */
	raw_spin_lock_irqsave(&realtek_ictl_lock, flags);

	for (cpu = 0; cpu < NR_CPUS; cpu++) {
		if (cpu_map[i->hwirq] >= 0 && cpu != cpu_map[i->hwirq])
			continue;
		value = readl(REG(RTL_ICTL_GIMR));
		value |= BIT(i->hwirq);
		writel(value, REG(RTL_ICTL_GIMR));
	}

	raw_spin_unlock_irqrestore(&realtek_ictl_lock, flags);
}

static void realtek_ictl_mask_irq(struct irq_data *i)
{
	unsigned long flags;
	u32 value;
	int cpu;

	raw_spin_lock_irqsave(&realtek_ictl_lock, flags);
	for (cpu = 0; cpu < NR_CPUS; cpu++) {
		value = readl(REG(RTL_ICTL_GIMR));
		value &= ~BIT(i->hwirq);
		writel(value, REG(RTL_ICTL_GIMR));
	}
	raw_spin_unlock_irqrestore(&realtek_ictl_lock, flags);
}

#ifdef CONFIG_SMP
static int realtek_ictl_set_affinity(struct irq_data *d, const struct cpumask *mask, bool force)
{
	unsigned long flags;
	int cpu, new_cpu = cpumask_first(mask);
	u32 v, w, cpu_int, irq = d->hwirq, j;
	int irr_regs[] = {
		RTL_ICTL_IRR3,
		RTL_ICTL_IRR2,
		RTL_ICTL_IRR1,
		RTL_ICTL_IRR0,
	};
	bool irq_on;

	pr_info("%s for hw-irq %d\n", __func__, irq);
	raw_spin_lock_irqsave(&realtek_ictl_lock, flags);

	// Save the current IRR register
	cpu = cpu_map[irq];
	if (cpu < 0)
		cpu = (new_cpu + 1) % 2;
	v = readl(REG(irr_regs[irq >> 3]));
	cpu_int = v & (0xf << ((irq * 4) % 32));
	w = v & ~(0xf << ((irq * 4) % 32));

	// Clear the current routing in IRR
	writel(w, REG(irr_regs[irq >> 3]));

	// Mask the Interrupt if it was on
	w = readl(REG(RTL_ICTL_GIMR));
	irq_on = w & BIT(irq);
	if (irq_on) {
		w &= ~BIT(irq);
		writel(w, REG(RTL_ICTL_GIMR));
	}

	cpu = new_cpu;
	if (cpu >= NR_CPUS)
		return -EINVAL;

	cpu_map[irq] = new_cpu;
	// Set target IRR
	v = readl(REG(irr_regs[irq >> 3]));
	w = v & ~(0xf << ((irq * 4) % 32));
	w |= cpu_int;
	writel(w, REG(irr_regs[irq >> 3]));

	// Unmask the Interrupt on the new GIMR
	if (irq_on) {
		w = readl(REG(RTL_ICTL_GIMR));
		w |= BIT(irq);
		writel(w, REG(RTL_ICTL_GIMR));
	}

	irq_data_update_effective_affinity(d, cpumask_of(cpu));

	raw_spin_unlock_irqrestore(&realtek_ictl_lock, flags);

	for (j = 0; j < 2; j++) {
		pr_info("%d %08x %08x %08x %08x %08x %08x\n", j,
			readl(0xb8003000 + j * 24), readl(0xb8003004 + j * 24),
			readl(0xb8003008 + j * 24), readl(0xb800300c + j * 24),
			readl(0xb8003010 + j * 24), readl(0xb8003014 + j * 24));
	}

	return IRQ_SET_MASK_OK_DONE;
}
#endif

static struct irq_chip realtek_ictl_irq = {
	.name = "realtek-rtl-intc",
	.irq_mask = realtek_ictl_mask_irq,
	.irq_unmask = realtek_ictl_unmask_irq,
#ifdef CONFIG_SMP
	.irq_set_affinity = realtek_ictl_set_affinity,
#endif
};

static int intc_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	pr_info("%s mapping hw-irq %lu linux irq %lu\n", __func__, hw, irq);
	irq_set_chip_and_handler(irq, &realtek_ictl_irq, handle_level_irq);

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
	unsigned int pending[N_CPU_IRQ];
	unsigned int all_pending = 0;
	int cpu;

	chained_irq_enter(chip, desc);
	for (cpu = 0; cpu < N_CPU_IRQ; cpu++) {
		pending[cpu] = readl(REG(RTL_ICTL_GIMR)) & readl(REG(RTL_ICTL_GISR));
		all_pending |= pending[cpu];
	}
//	if (pending[1])
//		pr_info("%s: with base %08x pending0 %08x pending1 %08x all pending %08x cpu%d\n",
//			__func__, (u32)cd->base[0], pending[0], pending[1], all_pending, smp_processor_id());
	if (unlikely(!all_pending)) {
		spurious_interrupt();
		goto out;
	}

	for (cpu = 0; cpu < N_CPU_IRQ; cpu++)
		generic_handle_irq(irq_find_mapping(domain, __ffs(pending[cpu])));

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
	int cpu;

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

		regs[((soc_int * 4) / 32)] |= cpu_int << (soc_int * 4) % 32;
		imap += 3;
	}

	for (cpu = 0; cpu < NR_CPUS; cpu++) {
		for (i = 0; i < 4; i++) {
			pr_info("%s: %d writing %08x to %08x\n",
				__func__, i, regs[i], (u32)REG(irr_regs[i % 4]));
			writel(regs[i], REG(irr_regs[i]));
		}
	}
	for (i = 0; i < 2; i++)
		pr_info("%d %08x %08x %08x %08x %08x %08x\n", i,
			readl(0xb8003000 + i * 24), readl(0xb8003004 + i * 24),
			readl(0xb8003008 + i * 24), readl(0xb800300c + i * 24),
			readl(0xb8003010 + i * 24), readl(0xb8003014 + i * 24));

	return 0;
}

static int __init realtek_rtl_of_init(struct device_node *node, struct device_node *parent)
{
	struct irq_domain *domain;
	struct realtek_intc_chip_data *cd;
	int ret;
	int cpu, i;

	base[0] = of_iomap(node, 0);
	if (!base[0])
		return -ENXIO;

	base[1] = base[0] + 0x18;

	for (i = 0; i < 32; i++)
		cpu_map[i] = -1;
	
	/* Disable all cascaded interrupts */
	for (cpu = 0; cpu < N_CPU_IRQ; cpu++)
		writel(0, REG(RTL_ICTL_GIMR));

	pr_info("%s: new handler with base 0 %08x and base 1 %08x\n",
		__func__, (u32)(base[0]), (u32)(base[1]));

	domain = irq_domain_add_simple(node, 32, 0, &irq_domain_ops, cd);

	pr_info("Domain data %08x\n", (u32)domain->host_data);
	ret = map_interrupts(node, domain);
	if (ret) {
		pr_err("invalid interrupt map\n");
		return ret;
	}

	return 0;
}

IRQCHIP_DECLARE(realtek_rtl_intc, "realtek,rtl-intc", realtek_rtl_of_init);
