// SPDX-License-Identifier: GPL-2.0-only
/*
 * prom.c
 * Early intialization code for the Realtek RTL838X SoC
 *
 * based on the original BSP by
 * Copyright (C) 2006-2012 Tony Wu (tonywu@realtek.com)
 * Copyright (C) 2020 B. Koblitz
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/of_fdt.h>
#include <linux/libfdt.h>
#include <asm/bootinfo.h>
#include <asm/addrspace.h>
#include <asm/page.h>
#include <asm/cpu.h>

#include <mach-rtl838x.h>

extern char arcs_cmdline[];
const void *fdt;
extern const char __appended_dtb;
//extern int __init rtl838x_serial_init(void);

struct rtl838x_soc_info soc_info;

static void freq_detect(void)
{
	unsigned char pdiv = 1 << ((rtl838x_r32(MEMPLL95_64) >> 14) & 0x3);
	u32 vco_freq;
	u32 cmu_sel_prediv = sw_r32(RTL930X_PLL_SW_CTRL0) & 0x3;
	u32 cmu_del_div4 = (sw_r32(RTL930X_PLL_SW_CTRL0) >> 2) & 0x1;
	u32 cmu_ncode_in = (sw_r32(RTL930X_PLL_SW_CTRL0) >> 4) & 0xff;
	u32 cmu_divn2_cpu = (sw_r32(RTL930X_PLL_CPU_MISC_CTRL) >> 1) & 0x7;
	u32 cmu_divn3_cpu = (sw_r32(RTL930X_PLL_CPU_CTRL0) >> 25) & 0x3;
	u32 lx_div;

	soc_info.mem_freq = ((((rtl838x_r32(MEMPLL127_96) >> 24) & 0x0FF) + 2) * 25) >> 1;
	soc_info.mem_freq /= pdiv;

//	soc_info.lx_freq = 1000 / ((rtl838x_r32(LX_CLK_PLL) & 0xf) + 2);
	pr_info("LX_CLK_PLL: %08x\n", rtl838x_r32(LX_CLK_PLL));

	pr_info("cmu_del_div4 : %d, cmu_ncode_in: %d, cmu_sel_prediv: %d\n",
		cmu_del_div4, cmu_ncode_in, cmu_sel_prediv
	);
	vco_freq = 25 * ((cmu_del_div4 ? 4 : 1) * (2 * (cmu_ncode_in + 1)))
			/ (1 << cmu_sel_prediv);

	lx_div = (vco_freq / (2 * 175)) - 2;
	if (lx_div > ((vco_freq / (2 * 133)) - 2))
		lx_div = (vco_freq / (2 * 175)) - 2;
	soc_info.lx_freq =  vco_freq / (2 * (lx_div + 2));

	soc_info.cpu_freq = vco_freq / ((cmu_divn2_cpu + 2) * (cmu_divn3_cpu + 1));

	soc_info.spi_freq = (((rtl838x_r32(SFCR_ADDR) >> 29) & 0xf) + 1) * 2;
	soc_info.spi_freq = 1000 / (((rtl838x_r32(LX_CLK_PLL) >> 8) & 0xf) + 2)
				 / soc_info.spi_freq;

	pr_info("VCO freq %d MHz\n", vco_freq);
	pr_info("Memory freq: %d MHz, Lexra-Bus freq: %d MHz, SPI-freq: %d MHz, CPU-Freq: %d MHz\n",
		soc_info.mem_freq, soc_info.lx_freq, soc_info.spi_freq, soc_info.cpu_freq
	);
}

void prom_console_init(void)
{
	/* UART 16550A is initialized by the bootloader */
}

#ifdef CONFIG_EARLY_PRINTK
#define rtl838x_r8(reg)		__raw_readb(reg)
#define rtl838x_w8(val, reg)	__raw_writeb(val, reg)

void unregister_prom_console(void)
{

}

void disable_early_printk(void)
{

}

void prom_putchar(char c)
{
	unsigned int retry = 0;

	do {
		if (retry++ >= 30000) {
			/* Reset Tx FIFO */
			rtl838x_w8(TXRST | CHAR_TRIGGER_14, UART0_FCR);
			return;
		}
	} while ((rtl838x_r8(UART0_LSR) & LSR_THRE) == TxCHAR_AVAIL);

	/* Send Character */
	rtl838x_w8(c, UART0_THR);
}

char prom_getchar(void)
{
	return '\0';
}
#endif


const char *get_system_type(void)
{
	return soc_info.name;
}


void __init prom_free_prom_memory(void)
{

}

void __init device_tree_init(void)
{
	pr_info("%s called\r\n", __func__);
	if (!fdt_check_header(&__appended_dtb)) {
		fdt = &__appended_dtb;
		pr_info("Using appended Device Tree.\n");
	}
	initial_boot_params = (void *)fdt;
	unflatten_and_copy_device_tree();
}

static void __init prom_init_cmdline(void)
{
	int argc = fw_arg0;
	char **argv = (char **) KSEG1ADDR(fw_arg1);
	int i;

	arcs_cmdline[0] = '\0';

	for (i = 0; i < argc; i++) {
		char *p = (char *) KSEG1ADDR(argv[i]);

		if (CPHYSADDR(p) && *p) {
			strlcat(arcs_cmdline, p, sizeof(arcs_cmdline));
			strlcat(arcs_cmdline, " ", sizeof(arcs_cmdline));
		}
	}
	pr_info("Kernel command line: %s\n", arcs_cmdline);
}

/* Do basic initialization */
void __init prom_init(void)
{
	uint32_t model;

	pr_info("%s called\n", __func__);
	pr_info("C0 Status: %08x, cause %08x\n", read_c0_status(), read_c0_cause());

	soc_info.sw_base = RTL838X_SW_BASE;

	model = sw_r32(RTL838X_MODEL_NAME_INFO);
	pr_info("RTL838X model is %x\n", model);
	model = model >> 16 & 0xFFFF;

	if ((model != 0x8328) && (model != 0x8330) && (model != 0x8332)
	    && (model != 0x8380) && (model != 0x8382)) {
		model = sw_r32(RTL839X_MODEL_NAME_INFO);
		pr_info("RTL839X model is %x\n", model);
		model = model >> 16 & 0xFFFF;
	}

	if ((model & 0x8390) != 0x8390) {
		model = sw_r32(RTL93XX_MODEL_NAME_INFO);
		pr_info("RTL93XX model is %x\n", model);
		model = model >> 16 & 0xFFFF;
	}

	soc_info.id = model;

	switch (model) {
	case 0x8328:
		soc_info.name = "RTL8328";
		soc_info.family = RTL8328_FAMILY_ID;
		break;
	case 0x8332:
		soc_info.name = "RTL8332";
		soc_info.family = RTL8380_FAMILY_ID;
		break;
	case 0x8380:
		soc_info.name = "RTL8380";
		soc_info.family = RTL8380_FAMILY_ID;
		break;
	case 0x8382:
		soc_info.name = "RTL8382";
		soc_info.family = RTL8380_FAMILY_ID;
		break;
	case 0x8390:
		soc_info.name = "RTL8390";
		soc_info.family = RTL8390_FAMILY_ID;
		break;
	case 0x8391:
		soc_info.name = "RTL8391";
		soc_info.family = RTL8390_FAMILY_ID;
		break;
	case 0x8392:
		soc_info.name = "RTL8392";
		soc_info.family = RTL8390_FAMILY_ID;
		break;
	case 0x8393:
		soc_info.name = "RTL8393";
		soc_info.family = RTL8390_FAMILY_ID;
		break;
	case 0x9301:
		soc_info.name = "RTL9301";
		soc_info.family = RTL9300_FAMILY_ID;
		break;
	case 0x9302:
		soc_info.rev = sw_r32(RTL93XX_MODEL_NAME_INFO) & 0xf;
		switch (sw_r32(RTL93XX_MODEL_NAME_INFO) & 0xfffffff0) {
		case 0x93020810:
			soc_info.name = "RTL9302A 12x2.5G";
			break;
		case 0x93021010:
			soc_info.name = "RTL9302B 8x2.5G";
			break;
		case 0x93021810:
			soc_info.name = "RTL9302C 16x2.5G";
			break;
		case 0x93022010:
			soc_info.name = "RTL9302D 24x2.5G";
			break;
		case 0x93020800:
			soc_info.name = "RTL9302A";
			break;
		case 0x93021000:
			soc_info.name = "RTL9302B";
			break;
		case 0x93021800:
			soc_info.name = "RTL9302C";
			break;
		case 0x93022000:
			soc_info.name = "RTL9302D";
			break;
		case 0x93023001:
			soc_info.name = "RTL9302F";
			break;
		default:
			soc_info.name = "RTL9302";
		}
		soc_info.family = RTL9300_FAMILY_ID;
		break;
	case 0x9313:
		soc_info.name = "RTL9313";
		soc_info.family = RTL9310_FAMILY_ID;
		break;
	default:
		soc_info.name = "DEFAULT";
		soc_info.family = 0;
	}
	pr_info("SoC Type: %s\n", get_system_type());
	if (soc_info.rev)
		pr_info("SoC Revision %d\n", soc_info.rev);

	freq_detect();

	if (soc_info.family == RTL9300_FAMILY_ID || soc_info.family == RTL9310_FAMILY_ID)
		soc_info.timer_base = RTL93XX_TIMER0_BASE;
	else
		soc_info.timer_base = RTL838X_TIMER0_BASE;

	prom_init_cmdline();
}
