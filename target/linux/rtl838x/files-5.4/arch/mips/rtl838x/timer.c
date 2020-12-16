// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clockchips.h>
#include <linux/init.h>
#include <asm/time.h>
#include <linux/interrupt.h>

#include <mach-rtl838x.h>

#define timer_r32(reg)			rtl838x_r32(soc_info.timer_base + reg)
#define timer_w32(val, reg)		rtl838x_w32(val, soc_info.timer_base + reg)
#define timer_w32_mask(clear, set, reg)	rtl838x_w32_mask(clear, set, soc_info.timer_base + reg)

extern struct rtl838x_soc_info soc_info;

static int rtl93xx_watchdog_default_func(void)
{
	return 0;
}

int rtl93xx_timer_state(void)
{
	return 0;
}

int rtl93xx_timer_set_base_clock(unsigned int hz)
{
	return 0;
}

static int rtl93xx_timer_set_next_event(unsigned long delta, struct clock_event_device *evt)
{
	return -EINVAL;
}

static void rtl93xx_timer_event_handler(struct clock_event_device *dev)
{
	
}

DEFINE_PER_CPU(struct clock_event_device, rtl93xx_clockevent_device);

void inline rtl93xx_timer_ack(void)
{
	unsigned int offset = smp_processor_id() * 0x10;

	timer_w32_mask(0, BIT(16), RTL93XX_TC0INT + offset);

	rtl93xx_watchdog_default_func();
}

static irqreturn_t rtl93xx_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *cd;

//	pr_info("In %s\n", __func__);
	cd = &per_cpu(rtl93xx_clockevent_device, smp_processor_id());

	/* Ack the RTC interrupt. */
	rtl93xx_timer_ack();

	cd->event_handler(cd);
	return IRQ_HANDLED;
}

static struct irqaction rtl93xx_irqaction = {
	.handler	= rtl93xx_timer_interrupt,
	.flags		= IRQF_TIMER | IRQF_PERCPU,
	.name		= "RTL93XX_TIMER",
};

int __init rtl93xx_clockevent_init(void)
{
	u32 cpu = smp_processor_id();
	u32 irq = RTL9300_TC0_IRQ + cpu;
	struct clock_event_device * cd = &per_cpu(rtl93xx_clockevent_device, cpu);

	cd->name		= "RTL93XX_TIMER";
	cd->features		= CLOCK_EVT_FEAT_PERIODIC;

	cd->event_handler	= rtl93xx_timer_event_handler;
	cd->set_next_event	= rtl93xx_timer_set_next_event;
	cd->rating = 100;
	cd->irq = irq;

	clockevent_set_clock(cd, 32768);
	cd->max_delta_ns = clockevent_delta2ns(0x7fffffff, cd);
	cd->min_delta_ns = clockevent_delta2ns(0x300, cd);
	cd->cpumask = cpumask_of(cpu);

	clockevents_register_device(cd);

	setup_irq(irq, &rtl93xx_irqaction);
	irq_set_handler(irq, handle_percpu_irq);

	return 0;
}

void __init rtl9300_timer_init(void)
{
	int i, offset;
	u32 v;

	pr_info("In %s\n", __func__);
	v = ( SYSTEM_FREQ_RTL9300 / ((int) DIVISOR_RTL9300 * HZ) );

	for (i = 0; i < NR_CPUS && i < RTL93XX_TC_MAX; i++) {
		pr_info("Setting up timer #%d\n", i);
		offset = i << 4;
		if (timer_r32(RTL93XX_TC0INT + offset) & BIT(16))
			timer_w32_mask(0, BIT(16), RTL93XX_TC0INT + offset);
	
		/* disable timer before setting CDBR */
		timer_w32(0, RTL93XX_TC0CTL + offset);
		timer_w32(v, RTL93XX_TC0DATA + offset);
	}
	
	/* Enable timer for all CPUs at once. */
	v = BIT(28) | BIT(24) | DIVISOR_RTL9300 ;
	for (i = 0; i < NR_CPUS && i < RTL93XX_TC_MAX; i++){
		offset = i << 4;
		timer_w32(v, RTL93XX_TC0CTL + offset);
	}

	rtl93xx_clockevent_init();
	// We only enable Timer 0, other timers are enabled by their CPUs
	timer_w32(BIT(20), RTL93XX_TC0INT);
}
