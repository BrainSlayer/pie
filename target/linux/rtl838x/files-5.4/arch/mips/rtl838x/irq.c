// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL838X architecture specific IRQ handling
 *
 * Copyright  (C) 2020 B. Koblitz
 * based on the original BSP
 * Copyright (C) 2006-2012 Tony Wu (tonywu@realtek.com)
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>

#include <asm/setup.h>
#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <mach-rtl838x.h>

extern struct rtl838x_soc_info soc_info;

#define icu_r32(reg)			rtl838x_r32(soc_info.icu_base + reg)
#define icu_w32(val, reg)		rtl838x_w32(val, soc_info.icu_base + reg)
#define icu_w32_mask(clear, set, reg)	rtl838x_w32_mask(clear, set, soc_info.icu_base + reg)

static DEFINE_RAW_SPINLOCK(irq_lock);

extern irqreturn_t c0_compare_interrupt(int irq, void *dev_id);


static void rtl838x_ictl_enable_irq(struct irq_data *i)
{
	unsigned long flags;

	pr_info("Enabling IRQ %d\n", i->irq);
	raw_spin_lock_irqsave(&irq_lock, flags);
	icu_w32_mask(0, 1 << i->irq, GIMR);
	raw_spin_unlock_irqrestore(&irq_lock, flags);
}

static void rtl838x_ictl_disable_irq(struct irq_data *i)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&irq_lock, flags);
	icu_w32_mask(1 << i->irq, 0, GIMR);
	raw_spin_unlock_irqrestore(&irq_lock, flags);
}

static void rtl838x_ictl_eoi_irq(struct irq_data *i)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&irq_lock, flags);
	icu_w32_mask(0, 1 << i->irq, GIMR);
	raw_spin_unlock_irqrestore(&irq_lock, flags);
}

static struct irq_chip rtl838x_ictl_irq = {
	.name = "RTL83xx",
	.irq_enable = rtl838x_ictl_enable_irq,
	.irq_disable = rtl838x_ictl_disable_irq,
	.irq_ack = rtl838x_ictl_disable_irq,
	.irq_mask = rtl838x_ictl_disable_irq,
	.irq_unmask = rtl838x_ictl_enable_irq,
	.irq_eoi = rtl838x_ictl_eoi_irq,
};

/*
 *  RTL8390/80/28 Interrupt Scheme
 *
 *  Source       IRQ      CPU INT
 *  --------   -------    -------
 *  UART0          31        IP3
 *  UART1          30        IP2
 *  TIMER0         29        IP6
 *  TIMER1         28        IP2
 *  OCPTO          27        IP2
 *  HLXTO          26        IP2
 *  SLXTO          25        IP2
 *  NIC            24        IP5
 *  GPIO_ABCD      23        IP5
 *  SWCORE         20        IP4
 *  WDT_IP1        19        IP5
 *  WDT_IP2        18        IP5
 *  USB_H2         17        IP2
 * 
 * RTL9300 Interrupt Scheme
 *  UART0          31        IP3
 *  UART1          30        IP2
 *  RTL9300_TC0_IRQ 7        IP6
 */

static void rtl83xx_irqdispatch_2(void)
{			
		pr_info("IRQ 2\n");
		do_IRQ(2);
}

static void rtl83xx_irqdispatch_3(void)
{			
		pr_info("IRQ 3\n");
		do_IRQ(3);
}

static void rtl83xx_irqdispatch_4(void)
{			
		pr_info("IRQ 4\n");
		do_IRQ(4);
}
static void rtl83xx_irqdispatch_5(void)
{			
		pr_info("IRQ 5\n");
		do_IRQ(5);
}

static void rtl83xx_irqdispatch_6(void)
{
		do_IRQ(RTL9300_TC0_IRQ);
}

static void rtl83xx_irqdispatch_7(void)
{			
		pr_info("IRQ 7\n");
		do_IRQ(7);
}

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending, ext_int;

	pending =  read_c0_cause();

	pr_info("In %s\n", __func__);
	if (pending & CAUSEF_IP7) {
		spurious_interrupt();
		//c0_compare_interrupt(7, NULL);
	} else if (pending & CAUSEF_IP6) {
		do_IRQ(TC0_IRQ);
	} else if (pending & CAUSEF_IP5) {
		ext_int = icu_r32(GIMR) & icu_r32(GISR);
		if (ext_int & NIC_IP)
			do_IRQ(NIC_IRQ);
		else if (ext_int & GPIO_ABCD_IP)
			do_IRQ(GPIO_ABCD_IRQ);
		else if ((ext_int & GPIO_EFGH_IP) && (soc_info.family == RTL8328_FAMILY_ID))
			do_IRQ(GPIO_EFGH_IRQ);
		else
			spurious_interrupt();
	} else if (pending & CAUSEF_IP4) {
		do_IRQ(SWCORE_IRQ);
	} else if (pending & CAUSEF_IP3) {
		do_IRQ(UART0_IRQ);
	} else if (pending & CAUSEF_IP2) {
		ext_int = icu_r32(GIMR) & icu_r32(GISR);
		if (ext_int & TC1_IP)
			do_IRQ(TC1_IRQ);
		else if (ext_int & UART1_IP)
			do_IRQ(UART1_IRQ);
		else
			spurious_interrupt();
	} else {
		spurious_interrupt();
	}
}

static int intc_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(hw, &rtl838x_ictl_irq, handle_level_irq);

	return 0;
}

static const struct irq_domain_ops irq_domain_ops = {
	.xlate = irq_domain_xlate_onecell,
	.map = intc_map,
};

int __init icu_of_init(struct device_node *node, struct device_node *parent)
{
	int i;
	struct irq_domain *domain;
	struct resource res;

	pr_info("Found Interrupt controller: %s (%s)\n", node->name, node->full_name);
	if (of_address_to_resource(node, 0, &res))
		panic("Failed to get icu memory range");

	if (!request_mem_region(res.start, resource_size(&res), res.name))
		pr_err("Failed to request icu memory\n");

	soc_info.icu_base = ioremap(res.start, resource_size(&res));
	pr_info("ICU Memory: %08x\n", (u32)soc_info.icu_base);
	pr_info("cpu_has_vint %d\n", cpu_has_vint);
	pr_info("cpu_has_veic %d\n", cpu_has_veic);
	
	mips_cpu_irq_init();

	domain = irq_domain_add_simple(node, 32, 0, &irq_domain_ops, NULL);

	/* Setup all external HW irqs */
	for (i = 8; i < RTL838X_IRQ_ICTL_NUM; i++) {
		irq_domain_associate(domain, i, i);
		irq_set_chip_and_handler(RTL838X_IRQ_ICTL_BASE + i,
					 &rtl838x_ictl_irq, handle_level_irq);
	}

	if (!cpu_has_vint) {
		if (request_irq(RTL838X_ICTL1_IRQ, no_action, IRQF_NO_THREAD,
				"IRQ cascade 1", NULL)) {
			pr_err("request_irq() cascade 1 for irq %d failed\n", RTL838X_ICTL1_IRQ);
		}
		if (request_irq(RTL838X_ICTL2_IRQ, no_action, IRQF_NO_THREAD,
				"IRQ cascade 2", NULL)) {
			pr_err("request_irq() cascade 2 for irq %d failed\n", RTL838X_ICTL2_IRQ);
		}
		if (request_irq(RTL838X_ICTL3_IRQ, no_action, IRQF_NO_THREAD,
				"IRQ cascade 3", NULL)) {
			pr_err("request_irq() cascade 3 for irq %d failed\n", RTL838X_ICTL3_IRQ);
		}
		if (request_irq(RTL838X_ICTL4_IRQ, no_action, IRQF_NO_THREAD,
				"IRQ cascade 4", NULL)) {
			pr_err("request_irq() cascade 4 for irq %d failed\n", RTL838X_ICTL4_IRQ);
		}
		if (request_irq(RTL838X_ICTL5_IRQ, no_action, IRQF_NO_THREAD,
				"IRQ cascade 5", NULL)) {
			pr_err("request_irq() cascade 5 for irq %d failed\n", RTL838X_ICTL5_IRQ);
		}
	} else {
		pr_info("Setting up Vectored Interrupt\n");
		set_vi_handler(2, rtl83xx_irqdispatch_2);
		set_vi_handler(3, rtl83xx_irqdispatch_3);
		set_vi_handler(4, rtl83xx_irqdispatch_4);
		set_vi_handler(5, rtl83xx_irqdispatch_5);
		set_vi_handler(6, rtl83xx_irqdispatch_6);
		set_vi_handler(7, rtl83xx_irqdispatch_7);
	}

	change_c0_status(ST0_IM, STATUSF_IP0 | STATUSF_IP1| STATUSF_IP2 | STATUSF_IP3
				| STATUSF_IP4 | STATUSF_IP5 | STATUSF_IP6 | STATUSF_IP7);

	// Set up interrupt routing scheme
	if (soc_info.family == RTL9300_FAMILY_ID) {
		pr_info("Setting up RTL9300 IRQ\n");
		icu_w32(IRR0_SETTING_RTL9300, IRR0);
		icu_w32(IRR1_SETTING_RTL9300, IRR1);
		icu_w32(IRR2_SETTING_RTL9300, IRR2);
		icu_w32(IRR3_SETTING_RTL9300, IRR3);
		icu_w32(UART0_IE | BIT(RTL9300_TC0_IRQ), GIMR);
	} else {
		icu_w32(IRR0_SETTING, IRR0);
		if (soc_info.family == RTL8380_FAMILY_ID)
			icu_w32(IRR1_SETTING_RTL838X, IRR1);
		else
			icu_w32(IRR1_SETTING_RTL839X, IRR1);
		icu_w32(IRR2_SETTING, IRR2);
		icu_w32(IRR3_SETTING, IRR3);
		/* Enable timer0 and uart0 interrupts */
		icu_w32(TC0_IE | UART0_IE, GIMR);
	}

	return 0;
}

void __init arch_init_irq(void)
{
	/* do board-specific irq initialization */
	irqchip_init();
}

IRQCHIP_DECLARE(mips_cpu_intc, "rtl838x,icu", icu_of_init);
