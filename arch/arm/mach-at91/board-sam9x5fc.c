/*
 *  Board-specific setup code for the AT91SAM9x5 Evaluation Kit family
 *
 *  Copyright (C) 2010 Atmel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <mach/cpu.h>

#include <linux/input/tca8418_keypad.h>
#include <linux/kxtf9.h>

#include <video/atmel_lcdfb.h>
#include <media/soc_camera.h>
#include <media/atmel-isi.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/atmel_hlcdc.h>
#include <mach/at91sam9_smc.h>
#include <mach/at91_shdwc.h>

#include "sam9_smc.h"
#include "generic.h"
#include <mach/board-sam9x5.h>

static void __init ek_map_io(void)
{
	/* Initialize processor and DBGU */
	cm_map_io();

	/* USART0 on ttyS1. (Rx, Tx) */
	at91_register_uart(AT91SAM9X5_ID_USART0, 1, 0);

	/* USART1 on ttyS2. (Rx, Tx) */
	at91_register_uart(AT91SAM9X5_ID_USART1, 2, 0);

	/* USART2 on ttyS3. (Rx, Tx) */
	at91_register_uart(AT91SAM9X5_ID_USART2, 3, 0);
}

/*
 * USB Host port (OHCI)
 */
/* Port A is shared with gadget port & Port C is full-speed only */
static struct at91_usbh_data __initdata ek_usbh_fs_data = {
	.ports		= 3,

};

/*
 * USB HS Host port (EHCI)
 */
/* Port A is shared with gadget port */
static struct at91_usbh_data __initdata ek_usbh_hs_data = {
	.ports		= 2,
};


/*
 * USB HS Device port
 */
static struct usba_platform_data __initdata ek_usba_udc_data;


/*
 * MACB Ethernet devices
 */
static struct at91_eth_data __initdata ek_macb0_data = {
	.is_rmii	= 1,
};

static struct at91_eth_data __initdata ek_macb1_data = {
	.phy_irq_pin	= AT91_PIN_PC26,
	.is_rmii	= 1,
};


/*
 * MCI (SD/MMC)
 */
/* mci0 detect_pin is revision dependent */
static struct mci_platform_data __initdata mci0_data = {
	.slot[0] = {
		.bus_width	= 4,
		.wp_pin		= -1,
	},
};

static struct mci_platform_data __initdata mci1_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= AT91_PIN_PD14,
		.wp_pin		= -1,
	},
};

/*
 * LCD Controller
 */
#if defined(CONFIG_FB_ATMEL) || defined(CONFIG_FB_ATMEL_MODULE)
static struct fb_videomode at91_tft_vga_modes[] = {
	{
		.name           = "LG",
		.refresh	= 60,
		.xres		= 800,		.yres		= 480,
		.pixclock	= KHZ2PICOS(33260),

		.left_margin	= 88,		.right_margin	= 168,
		.upper_margin	= 8,		.lower_margin	= 37,
		.hsync_len	= 128,		.vsync_len	= 2,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs at91fb_default_monspecs = {
	.manufacturer	= "LG",
	.monitor        = "LB043WQ1",

	.modedb		= at91_tft_vga_modes,
	.modedb_len	= ARRAY_SIZE(at91_tft_vga_modes),
	.hfmin		= 15000,
	.hfmax		= 17640,
	.vfmin		= 57,
	.vfmax		= 67,
};

/* Default output mode is TFT 24 bit */
#define AT91SAM9X5_DEFAULT_LCDCFG5	(LCDC_LCDCFG5_MODE_OUTPUT_24BPP)

/* Driver datas */
static struct atmel_lcdfb_info __initdata ek_lcdc_data = {
	.lcdcon_is_backlight		= true,
	.alpha_enabled			= false,
	.default_bpp			= 16,
	/* Reserve enough memory for 32bpp */
	.smem_len			= 800 * 480 * 4,
	/* In 9x5 default_lcdcon2 is used for LCDCFG5 */
	.default_lcdcon2		= AT91SAM9X5_DEFAULT_LCDCFG5,
	.default_monspecs		= &at91fb_default_monspecs,
	.guard_time			= 9,
	.lcd_wiring_mode		= ATMEL_LCDC_WIRING_RGB,
};

#else
static struct atmel_lcdfb_info __initdata ek_lcdc_data;
#endif

/*
 * Touchscreen
 */
static struct at91_tsadcc_data ek_tsadcc_data = {
	.adc_clock		= 300000,
	.filtering_average	= 0x03,	/* averages 2^filtering_average ADC conversions */
	.pendet_debounce	= 0x08,
	.pendet_sensitivity	= 0x02,	/* 2 = set to default */
	.ts_sample_hold_time	= 0x0a,
};

#if defined(CONFIG_KEYBOARD_TCA8418)

static uint32_t fc_keymap[] = {
  KEY(0, 0, KEY_1),
  0
};

static struct matrix_keymap_data fc_key_data = {
  .keymap         = fc_keymap,
  .keymap_size    = ARRAY_SIZE(fc_keymap),
};


static struct tca8418_keypad_platform_data fc_keys_info = {
  .keymap_data    = &fc_key_data,
  .rows           = 4,
  .cols           = 3,
  .rep            = 1,
  .irq_is_gpio    = 0
};

#endif

static int fc_kxtf9_gpio_level(void)
{
	return at91_get_gpio_value(AT91_PIN_PC6);
}

struct kxtf9_platform_data fc_kxtf9_pdata = {
	.min_interval	= 2,
	.poll_interval	= 200,

	.g_range	= KXTF9_G_2G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,


	.data_odr_init		= ODR100,
	.ctrl_reg1_init		= RES_12BIT | KXTF9_G_2G | WUFE,
	.int_ctrl_init		= IEA | IEN,
	.tilt_timer_init	= 0x03,
	.engine_odr_init	= OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init		= 0x0A,
	.wuf_thresh_init	= 0x20,
	.tdt_timer_init		= 0x78,
	.tdt_h_thresh_init	= 0xB6,
	.tdt_l_thresh_init	= 0x1A,
	.tdt_tap_timer_init	= 0xA2,
	.tdt_total_timer_init	= 0x24,
	.tdt_latency_timer_init	= 0x28,
	.tdt_window_timer_init	= 0xA0,

	.gpio = fc_kxtf9_gpio_level,
	.gesture = 0,
	.sensitivity_low = {
		  0x50, 0xFF, 0xB8, 0x32, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_medium = {
		  0x50, 0xFF, 0x68, 0x32, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_high = {
		  0x78, 0xB6, 0x1A, 0xA2, 0x24, 0x28, 0xA0,
	},
};

/*
 * I2C Devices
 */
static struct i2c_board_info __initdata ek_i2c_devices[] = {
#if defined(CONFIG_KEYBOARD_TCA8418)
  {
		I2C_BOARD_INFO("tca8418_keypad", 0x34),
    .platform_data = &fc_keys_info,
		.irq = AT91_PIN_PB18,
		.flags = I2C_CLIENT_WAKE,
  },
#endif
	{
		I2C_BOARD_INFO("kxtf9", 0x0F),
		.platform_data = &fc_kxtf9_pdata,
		.irq = AT91_PIN_PC6,
		.flags = I2C_CLIENT_WAKE,
	},
};

static void __init ek_board_configure_pins(void)
{
  /* Port A is shared with gadget port */
	ek_usbh_fs_data.vbus_pin[1] = AT91_PIN_PD19;
	ek_usbh_hs_data.vbus_pin[1] = AT91_PIN_PD19;
	/* Port C is full-speed only */
	ek_usbh_fs_data.vbus_pin[2] = AT91_PIN_PD20;

	ek_usba_udc_data.vbus_pin = AT91_PIN_PB16;

	ek_macb0_data.phy_irq_pin = AT91_PIN_PB8;

#if defined(CONFIG_KEYBOARD_TCA8418)
	at91_set_gpio_input(AT91_PIN_PB18, 1);
#endif

	at91_set_gpio_input(AT91_PIN_PC6, 1);
}

static void __init ek_board_init(void)
{
	u32 cm_config;

	cm_board_init(&cm_config);
	ek_board_configure_pins();
	/* Serial */
	at91_add_device_serial();
	/* USB HS Host */
	at91_add_device_usbh_ohci(&ek_usbh_fs_data);
	at91_add_device_usbh_ehci(&ek_usbh_hs_data);
	/* USB HS Device */
	at91_add_device_usba(&ek_usba_udc_data);
	/* Ethernet */
	at91_add_device_eth(0, &ek_macb0_data);
	at91_add_device_eth(1, &ek_macb1_data);
	/* MMC0 */
	at91_add_device_mci(0, &mci0_data);
	/* I2C */
	if (cm_config & CM_CONFIG_I2C0_ENABLE)
		i2c_register_board_info(0,
				ek_i2c_devices, ARRAY_SIZE(ek_i2c_devices));
	else
		at91_add_device_i2c(0, ek_i2c_devices, ARRAY_SIZE(ek_i2c_devices));

  /*while(1) {
    at91_set_gpio_value(AT91_PIN_PA30, 0);
    at91_set_gpio_value(AT91_PIN_PA31, 0);

    at91_set_gpio_value(AT91_PIN_PA30, 1);
    at91_set_gpio_value(AT91_PIN_PA31, 1);
  }*/

	if (!cpu_is_at91sam9x25()) {
		/* LCD Controller */
		at91_add_device_lcdc(&ek_lcdc_data);
		/* Touch Screen */
		at91_add_device_tsadcc(&ek_tsadcc_data);
	}

	/* MMC1 */
	/* Conflict between SPI0, MCI1 and ISI pins.
	 * add MCI1 only if SPI0 and ISI are both disabled.
	 */
	if (!(cm_config & CM_CONFIG_SPI0_ENABLE))
		at91_add_device_mci(1, &mci1_data);

#if 0
	if (cpu_is_at91sam9x25() || cpu_is_at91sam9x35())
		/*
		 * open jumper/solderdrop JP11 to activate CAN0
		 *
		 * _note_: this will deactivate the debug uart
		 */
		at91_add_device_can(0, NULL);
#endif


	/* SSC (for WM8731) */
	//at91_add_device_ssc(AT91SAM9X5_ID_SSC, ATMEL_SSC_TX | ATMEL_SSC_RX);

	/* SMD */
	at91_add_device_smd();

	/* print conflict information */
	if (cm_config & CM_CONFIG_SPI0_ENABLE)
		printk(KERN_CRIT
			"AT91: SPI0 conflicts with MCI1 and ISI, disable MCI1 and ISI\n");
}

MACHINE_START(AT91SAM9X5FC, "Ahern AT91SAM9X5 Fuel Controller")
	/* Maintainer: Atmel */
/* XXX/ukl: can we drop .boot_params? */
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= ek_map_io,
	.init_irq	= cm_init_irq,
	.init_machine	= ek_board_init,
MACHINE_END
