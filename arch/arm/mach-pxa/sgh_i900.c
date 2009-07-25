/**
 * Support for the PXA312 based Samsung SGH-i900 (Omnia)
 *
 * Copyright (C) 2009 Stefan Schmidt <stefan@datenfreihafen.org>
 *
 * Based on zylonite.c Copyright (C) 2006 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/pxafb.h>
#include <mach/zylonite.h>
#include <mach/mmc.h>
#include <mach/hardware.h>
#include <mach/pxafb.h>
#include <mach/zylonite.h>
#include <mach/mmc.h>

#include "devices.h"
#include "generic.h"

#define MAX_SLOTS	3
struct platform_mmc_slot sgh_i900_mmc_slot[MAX_SLOTS];

#if defined(CONFIG_FB_PXA) || defined(CONFIG_FB_PXA_MODULE)
static struct platform_pwm_backlight_data sgh_i900_backlight_data = {
	.pwm_id		= 3,
	.max_brightness	= 100,
	.dft_brightness	= 100,
	.pwm_period_ns	= 10000,
};

static struct platform_device sgh_i900_backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.parent = &pxa27x_device_pwm1.dev,
		.platform_data	= &sgh_i900_backlight_data,
	},
};

static struct pxafb_mode_info sgh_i900_mode = {
	.pixclock	= 96153,
	.xres		= 240,
	.yres		= 400,
	.bpp		= 16,
	.hsync_len	= 8,
	.left_margin	= 8,
	.right_margin	= 8,
	.vsync_len	= 4,
	.upper_margin	= 38,
	.lower_margin	= 38,
	.sync		= 0,
};

static struct pxafb_mach_info sgh_i900_lcd_info = {
	.num_modes		= 1,
	.lcd_conn		= LCD_COLOR_TFT_16BPP | LCD_PCLK_EDGE_FALL,
};

static void __init sgh_i900_init_lcd(void)
{
	platform_device_register(&sgh_i900_backlight_device);

	sgh_i900_lcd_info.modes = &sgh_i900_mode;

	set_pxa_fb_info(&sgh_i900_lcd_info);
}
#else
static inline void sgh_i900_init_lcd(void) {}
#endif

#if defined(CONFIG_MMC)
static int sgh_i900_mci_ro(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	return gpio_get_value(sgh_i900_mmc_slot[pdev->id].gpio_wp);
}

static int sgh_i900_mci_init(struct device *dev,
			     irq_handler_t sgh_i900_detect_int,
			     void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int err, cd_irq, gpio_cd, gpio_wp;

	cd_irq = gpio_to_irq(sgh_i900_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = sgh_i900_mmc_slot[pdev->id].gpio_cd;
	gpio_wp = sgh_i900_mmc_slot[pdev->id].gpio_wp;

	/*
	 * setup GPIO for SGH-i900 MMC controller
	 */
	err = gpio_request(gpio_cd, "mmc card detect");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	err = gpio_request(gpio_wp, "mmc write protect");
	if (err)
		goto err_request_wp;
	gpio_direction_input(gpio_wp);

	err = request_irq(cd_irq, sgh_i900_detect_int,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "MMC card detect", data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
				"can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}

	return 0;

err_request_irq:
	gpio_free(gpio_wp);
err_request_wp:
	gpio_free(gpio_cd);
err_request_cd:
	return err;
}

static void sgh_i900_mci_exit(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int cd_irq, gpio_cd, gpio_wp;

	cd_irq = gpio_to_irq(sgh_i900_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = sgh_i900_mmc_slot[pdev->id].gpio_cd;
	gpio_wp = sgh_i900_mmc_slot[pdev->id].gpio_wp;

	free_irq(cd_irq, data);
	gpio_free(gpio_cd);
	gpio_free(gpio_wp);
}

static struct pxamci_platform_data sgh_i900_mci_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.init		= sgh_i900_mci_init,
	.exit		= sgh_i900_mci_exit,
	.get_ro		= sgh_i900_mci_ro,
};

static struct pxamci_platform_data sgh_i900_mci2_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
};

static void __init sgh_i900_init_mmc(void)
{
	pxa_set_mci_info(&sgh_i900_mci_platform_data);
	pxa3xx_set_mci2_info(&sgh_i900_mci2_platform_data);
	pxa3xx_set_mci3_info(&sgh_i900_mci_platform_data);
}
#else
static inline void sgh_i900_init_mmc(void) {}
#endif

static void __init sgh_i900_init(void)
{
	sgh_i900_init_lcd();
	sgh_i900_init_mmc();
}

MACHINE_START(SGH_I900, "Samsung SGH-i900 (Omnia) phone")
	.phys_io	= 0x40000000,
	.boot_params	= 0xa0000100,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq	= pxa3xx_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= sgh_i900_init,
MACHINE_END
