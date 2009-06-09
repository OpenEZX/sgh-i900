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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/hardware.h>
#include <mach/audio.h>
#include <mach/pxafb.h>
#include <mach/sgh_i900.h>
#include <mach/mmc.h>
#include <mach/ohci.h>

#include "devices.h"
#include "generic.h"

#define MAX_SLOTS	3
struct platform_mmc_slot sgh_i900_mmc_slot[MAX_SLOTS];

int gpio_debug_led1;
int gpio_debug_led2;

int wm9713_irq;

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
static struct gpio_led sgh_i900_debug_leds[] = {
	[0] = {
		.name			= "sgh_i900:yellow:1",
		.default_trigger	= "heartbeat",
	},
	[1] = {
		.name			= "sgh_i900:yellow:2",
		.default_trigger	= "default-on",
	},
};

static struct gpio_led_platform_data sgh_i900_debug_leds_info = {
	.leds		= sgh_i900_debug_leds,
	.num_leds	= ARRAY_SIZE(sgh_i900_debug_leds),
};

static struct platform_device sgh_i900_device_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &sgh_i900_debug_leds_info,
	}
};

static void __init sgh_i900_init_leds(void)
{
	sgh_i900_debug_leds[0].gpio = gpio_debug_led1;
	sgh_i900_debug_leds[1].gpio = gpio_debug_led2;

	platform_device_register(&sgh_i900_device_leds);
}
#else
static inline void sgh_i900_init_leds(void) {}
#endif

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

static struct pxafb_mode_info toshiba_ltm035a776c_mode = {
	.pixclock		= 110000,
	.xres			= 240,
	.yres			= 480,
	.bpp			= 16,
	.hsync_len		= 4,
	.left_margin		= 6,
	.right_margin		= 4,
	.vsync_len		= 2,
	.upper_margin		= 2,
	.lower_margin		= 3,
	.sync			= FB_SYNC_VERT_HIGH_ACT,
};

static struct pxafb_mach_info sgh_i900_toshiba_lcd_info = {
	.num_modes		= 1,
	.lcd_conn		= LCD_COLOR_TFT_16BPP | LCD_PCLK_EDGE_FALL,
};

static void __init sgh_i900_init_lcd(void)
{
	platform_device_register(&sgh_i900_backlight_device);

	sgh_i900_toshiba_lcd_info.modes = &toshiba_ltm035a776c_mode;

	set_pxa_fb_info(&sgh_i900_toshiba_lcd_info);
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
	if (cpu_is_pxa310())
		pxa3xx_set_mci3_info(&sgh_i900_mci_platform_data);
}
#else
static inline void sgh_i900_init_mmc(void) {}
#endif

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct pxaohci_platform_data sgh_i900_ohci_info = {
	.port_mode	= PMM_PERPORT_MODE,
	.flags		= ENABLE_PORT1 | ENABLE_PORT2 |
			  POWER_CONTROL_LOW | POWER_SENSE_LOW,
};

static void __init sgh_i900_init_ohci(void)
{
	pxa_set_ohci_info(&sgh_i900_ohci_info);
}
#else
static inline void sgh_i900_init_ohci(void) {}
#endif /* CONFIG_USB_OHCI_HCD || CONFIG_USB_OHCI_HCD_MODULE */

static void __init sgh_i900_init(void)
{
	/* board-processor specific initialization */
	sgh_i900_pxa300_init();
	sgh_i900_pxa320_init();

	pxa_set_ac97_info(NULL);
	sgh_i900_init_lcd();
	sgh_i900_init_mmc();
	sgh_i900_init_leds();
	sgh_i900_init_ohci();
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
