/* adapted from linux/arch/arm/mach-msm/panel-samsungwvga-tl2786a.c
 *
 * Copyright (c) 2009 Google Inc.
 * Copyright (c) 2009 HTC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/leds.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include "msm_fb.h"

#define DEBUG_LCM
#ifdef DEBUG_LCM
#define LCMDBG(fmt, arg...)	printk(fmt, ## arg)
#else
#define LCMDBG(fmt, arg...)	{}
#endif

struct lcm_cmd {
	int reg;
	uint32_t val;
	unsigned delay;
};

#define LCM_CMD_SEQ	(struct lcm_cmd[])
#define LCM_REG_END	(-1)
#define LCM_CMD_END	{-1, 0, 0}


static struct lcm_cmd lcm_init_seq[] = {
		{ 0x31, 0x08, 0 },
		{ 0x32, 0x14, 0 },
		{ 0x30, 0x2 , 0 },
		{ 0x27, 0x1 , 0 },
		{ 0x12, 0x8 , 0 },
		{ 0x13, 0x8 , 0 },
		{ 0x15, 0x0 , 0 },
		{ 0x16, 0x02, 0 },
		{ 0x39, 0x44, 0 },
		{ 0x17, 0x22, 0 },
		{ 0x18, 0x33, 0 },
		{ 0x19, 0x3 , 0 },
		{ 0x1A, 0x1 , 0 },
		{ 0x22, 0xA4, 0 },
		{ 0x23, 0x0 , 0 },
		{ 0x26, 0xA0, 0 },
		{ 0x1D, 0xA0, 250 },
};

static struct lcm_cmd lcm_init_666seq[] = {
		{ 0x31, 0x08, 0 },
		{ 0x32, 0x14, 0 },
		{ 0x30, 0x2 , 0 },
		{ 0x27, 0x1 , 0 },
		{ 0x12, 0x8 , 0 },
		{ 0x13, 0x8 , 0 },
		{ 0x15, 0x0 , 0 },
		{ 0x16, 0x01, 0 },
		{ 0x16, 0x01, 0 },
		{ 0x39, 0x44, 0 },
		{ 0x17, 0x22, 0 },
		{ 0x18, 0x33, 0 },
		{ 0x19, 0x3 , 0 },
		{ 0x1A, 0x1 , 0 },
		{ 0x22, 0xA4, 0 },
		{ 0x23, 0x0 , 0 },
		{ 0x26, 0xA0, 0 },
		{ 0x1D, 0xA0, 250 },
};

//static struct lcm_cmd lcm_wakup_seq[] = {
//		{ 0x1d, 0xa0, 200 },
//		{ 0x14, 0x3 , 100 },
//};

static struct lcm_cmd lcm_standby_seq[] = {
		{0x14, 0x0, 100 },
		{0x1d, 0xa1, 200 },
};


#define OLED_GAMMA_TABLE_SIZE		(7 * 3)
static struct lcm_cmd samsung_oled_gamma_table[][OLED_GAMMA_TABLE_SIZE] = {
	/*level 10*/
	{
		{0x40, 0x0 , 0},
		{0x41, 0x3f, 0},
		{0x42, 0x40, 0},
		{0x43, 0x39, 0},
		{0x44, 0x32, 0},
		{0x45, 0x2e, 0},
		{0x46, 0xc , 0},
		{0x50, 0x0 , 0},
		{0x51, 0x0 , 0},
		{0x52, 0x0 , 0},
		{0x53, 0x00, 0},
		{0x54, 0x26, 0},
		{0x55, 0x2d, 0},
		{0x56, 0xb , 0},
		{0x60, 0x0 , 0},
		{0x61, 0x3f, 0},
		{0x62, 0x40, 0},
		{0x63, 0x38, 0},
		{0x64, 0x31, 0},
		{0x65, 0x2d, 0},
		{0x66, 0x12, 0},
	},

	/*level 40*/
	{
		{0x40, 0x0 , 0},
		{0x41, 0x3f, 0},
		{0x42, 0x3e, 0},
		{0x43, 0x2e, 0},
		{0x44, 0x2d, 0},
		{0x45, 0x28, 0},
		{0x46, 0x21, 0},
		{0x50, 0x0 , 0},
		{0x51, 0x0 , 0},
		{0x52, 0x0 , 0},
		{0x53, 0x21, 0},
		{0x54, 0x2a, 0},
		{0x55, 0x28, 0},
		{0x56, 0x20, 0},
		{0x60, 0x0 , 0},
		{0x61, 0x3f, 0},
		{0x62, 0x3e, 0},
		{0x63, 0x2d, 0},
		{0x64, 0x2b, 0},
		{0x65, 0x26, 0},
		{0x66, 0x2d, 0},
	},

	/*level 70*/
	{
		{0x40, 0x0 , 0},
		{0x41, 0x3f, 0},
		{0x42, 0x35, 0},
		{0x43, 0x2c, 0},
		{0x44, 0x2b, 0},
		{0x45, 0x26, 0},
		{0x46, 0x29, 0},
		{0x50, 0x0 , 0},
		{0x51, 0x0 , 0},
		{0x52, 0x0 , 0},
		{0x53, 0x25, 0},
		{0x54, 0x29, 0},
		{0x55, 0x26, 0},
		{0x56, 0x28, 0},
		{0x60, 0x0 , 0},
		{0x61, 0x3f, 0},
		{0x62, 0x34, 0},
		{0x63, 0x2b, 0},
		{0x64, 0x2a, 0},
		{0x65, 0x23, 0},
		{0x66, 0x37, 0},
	},

	/*level 100*/
	{
		{0x40, 0x0 , 0},
		{0x41, 0x3f, 0},
		{0x42, 0x30, 0},
		{0x43, 0x2a, 0},
		{0x44, 0x2b, 0},
		{0x45, 0x24, 0},
		{0x46, 0x2f, 0},
		{0x50, 0x0 , 0},
		{0x51, 0x0 , 0},
		{0x52, 0x0 , 0},
		{0x53, 0x25, 0},
		{0x54, 0x29, 0},
		{0x55, 0x24, 0},
		{0x56, 0x2e, 0},
		{0x60, 0x0 , 0},
		{0x61, 0x3f, 0},
		{0x62, 0x2f, 0},
		{0x63, 0x29, 0},
		{0x64, 0x29, 0},
		{0x65, 0x21, 0},
		{0x66, 0x3f, 0},
	},

	/*level 130*/
	{
		{0x40, 0x0 , 0},
		{0x41, 0x3f, 0},
		{0x42, 0x2e, 0},
		{0x43, 0x29, 0},
		{0x44, 0x2a, 0},
		{0x45, 0x23, 0},
		{0x46, 0x34, 0},
		{0x50, 0x0 , 0},
		{0x51, 0x0 , 0},
		{0x52, 0xa , 0},
		{0x53, 0x25, 0},
		{0x54, 0x28, 0},
		{0x55, 0x23, 0},
		{0x56, 0x33, 0},
		{0x60, 0x0 , 0},
		{0x61, 0x3f, 0},
		{0x62, 0x2d, 0},
		{0x63, 0x28, 0},
		{0x64, 0x27, 0},
		{0x65, 0x20, 0},
		{0x66, 0x46, 0},
	},

	/*level 160*/
	{
		{0x40, 0x0 , 0},
		{0x41, 0x3f, 0},
		{0x42, 0x2b, 0},
		{0x43, 0x29, 0},
		{0x44, 0x28, 0},
		{0x45, 0x23, 0},
		{0x46, 0x38, 0},
		{0x50, 0x0 , 0},
		{0x51, 0x0 , 0},
		{0x52, 0xb , 0},
		{0x53, 0x25, 0},
		{0x54, 0x27, 0},
		{0x55, 0x23, 0},
		{0x56, 0x37, 0},
		{0x60, 0x0 , 0},
		{0x61, 0x3f, 0},
		{0x62, 0x29, 0},
		{0x63, 0x28, 0},
		{0x64, 0x25, 0},
		{0x65, 0x20, 0},
		{0x66, 0x4b, 0},
	},

	/*level 190*/
	{
		{0x40, 0x0 , 0},
		{0x41, 0x3f, 0},
		{0x42, 0x29, 0},
		{0x43, 0x29, 0},
		{0x44, 0x27, 0},
		{0x45, 0x22, 0},
		{0x46, 0x3c, 0},
		{0x50, 0x0 , 0},
		{0x51, 0x0 , 0},
		{0x52, 0x10, 0},
		{0x53, 0x26, 0},
		{0x54, 0x26, 0},
		{0x55, 0x22, 0},
		{0x56, 0x3b, 0},
		{0x60, 0x0 , 0},
		{0x61, 0x3f, 0},
		{0x62, 0x28, 0},
		{0x63, 0x28, 0},
		{0x64, 0x24, 0},
		{0x65, 0x1f, 0},
		{0x66, 0x50, 0},
	},

	/*level 220*/
	{
		{0x40, 0x0 , 0},
		{0x41, 0x3f, 0},
		{0x42, 0x28, 0},
		{0x43, 0x28, 0},
		{0x44, 0x28, 0},
		{0x45, 0x20, 0},
		{0x46, 0x40, 0},
		{0x50, 0x0 , 0},
		{0x51, 0x0 , 0},
		{0x52, 0x11, 0},
		{0x53, 0x25, 0},
		{0x54, 0x27, 0},
		{0x55, 0x20, 0},
		{0x56, 0x3f, 0},
		{0x60, 0x0 , 0},
		{0x61, 0x3f, 0},
		{0x62, 0x27, 0},
		{0x63, 0x26, 0},
		{0x64, 0x26, 0},
		{0x65, 0x1c, 0},
		{0x66, 0x56, 0},
	},

	/*level 250*/
	{
		{0x40, 0x0 , 0},
		{0x41, 0x3f, 0},
		{0x42, 0x2a, 0},
		{0x43, 0x27, 0},
		{0x44, 0x27, 0},
		{0x45, 0x1f, 0},
		{0x46, 0x44, 0},
		{0x50, 0x0 , 0},
		{0x51, 0x0 , 0},
		{0x52, 0x17, 0},
		{0x53, 0x24, 0},
		{0x54, 0x26, 0},
		{0x55, 0x1f, 0},
		{0x56, 0x43, 0},
		{0x60, 0x0 , 0},
		{0x61, 0x3f, 0},
		{0x62, 0x2a, 0},
		{0x63, 0x25, 0},
		{0x64, 0x24, 0},
		{0x65, 0x1b, 0},
		{0x66, 0x5c, 0},
	},
};

#define SAMSUNG_OLED_NUM_LEVELS		ARRAY_SIZE(samsung_oled_gamma_table)

#define SAMSUNG_OLED_MIN_VAL		10
#define SAMSUNG_OLED_MAX_VAL		250
#define SAMSUNG_OLED_DEFAULT_VAL	(SAMSUNG_OLED_MIN_VAL +		\
					 (SAMSUNG_OLED_MAX_VAL -	\
					  SAMSUNG_OLED_MIN_VAL) / 2)

#define SAMSUNG_OLED_LEVEL_STEP		((SAMSUNG_OLED_MAX_VAL -	\
					  SAMSUNG_OLED_MIN_VAL) /	\
					 (SAMSUNG_OLED_NUM_LEVELS - 1))

#define SPI_CONFIG              (0x00000000)
#define SPI_IO_CONTROL          (0x00000004)
#define SPI_OPERATIONAL         (0x00000030)
#define SPI_ERROR_FLAGS_EN      (0x00000038)
#define SPI_ERROR_FLAGS         (0x00000034)
#define SPI_OUTPUT_FIFO         (0x00000100)
extern int panel_type;
extern int qspi_send(uint32_t id, uint8_t data);
static int lcm_write_cmd(uint32_t reg, uint32_t data)
{
	int ret = -1;

	ret = qspi_send(0x0, reg);
	if (ret)
		goto err_lcm_writeb;

	ret = qspi_send(0x1, data);
	if (ret)
		goto err_lcm_writeb;
	return 0;

err_lcm_writeb:
	printk(KERN_ERR "%s: Failure on sending SPI commands", __func__);
	return ret;
}

static int lcm_write_tb(struct lcm_cmd cmd_table[], unsigned size)
{
	int i;

	for (i = 0; i < size; i++) {
		lcm_write_cmd(cmd_table[i].reg, cmd_table[i].val);
		if (cmd_table[i].delay)
			msleep(cmd_table[i].delay);
	}
	return 0;
}

/* ---------------------------------------------------- */

static DEFINE_MUTEX(panel_lock);
static uint8_t new_val = SAMSUNG_OLED_DEFAULT_VAL;
static uint8_t last_val = SAMSUNG_OLED_DEFAULT_VAL;
static uint8_t table_sel_vals[] = { 0x43, 0x34 };
static int table_sel_idx;
static void (*panel_power_gpio)(int on);
static struct wake_lock panel_idle_lock;
static struct led_trigger *amoled_lcd_backlight;
static int panel_id = 0;

static void gamma_table_bank_select(void)
{
	lcm_write_cmd(0x39, table_sel_vals[table_sel_idx]);
	table_sel_idx ^= 1;
}

/*
 * Caller must make sure the spi is ready
 * */
static void amoled_set_gamma_val(int val)
{
	int i;
	int level;
	int frac;

	val = clamp(val, SAMSUNG_OLED_MIN_VAL, SAMSUNG_OLED_MAX_VAL);
	val = (val / 2) * 2;

	if (val < 31) {
		val = 20;
	}

	level = (val - SAMSUNG_OLED_MIN_VAL) / SAMSUNG_OLED_LEVEL_STEP;
	/*
		LCMDBG("set brightness = (%d, %d)\n", level, val);
	*/
	frac = (val - SAMSUNG_OLED_MIN_VAL) % SAMSUNG_OLED_LEVEL_STEP;

	for (i = 0; i < OLED_GAMMA_TABLE_SIZE; ++i) {
		unsigned int v1;
		unsigned int v2 = 0;
		u8 v;
		if (frac == 0) {
			v = samsung_oled_gamma_table[level][i].val;
		} else {
			v1 = samsung_oled_gamma_table[level][i].val;
			v2 = samsung_oled_gamma_table[level+1][i].val;
			v = (v1 * (SAMSUNG_OLED_LEVEL_STEP - frac) +
				 v2 * frac) / SAMSUNG_OLED_LEVEL_STEP;
		}
		lcm_write_cmd(samsung_oled_gamma_table[level][i].reg, v);
	}
	gamma_table_bank_select();
	last_val = val;
}

static void amoled_panel_power(int on)
{
        if (panel_power_gpio)
          (*panel_power_gpio)(on);
}

static int samsung_oled_panel_init(void)
{
	LCMDBG("%s()\n", __func__);

	amoled_panel_power(1);

	wake_lock(&panel_idle_lock);
	mutex_lock(&panel_lock);
	if (system_rev < 1)
		lcm_write_tb(lcm_init_seq, ARRAY_SIZE(lcm_init_seq));
	else
		lcm_write_tb(lcm_init_666seq, ARRAY_SIZE(lcm_init_666seq));
	gamma_table_bank_select();
	mutex_unlock(&panel_lock);
	wake_unlock(&panel_idle_lock);
	return 0;
}

static int amoled_panel_unblank(struct platform_device *pdev)
{
	LCMDBG("%s\n", __func__);


        if (samsung_oled_panel_init())
          printk(KERN_ERR "samsung_oled_panel_init failed\n");

	wake_lock(&panel_idle_lock);
	mutex_lock(&panel_lock);
	table_sel_idx = 0;
	gamma_table_bank_select();
	amoled_set_gamma_val(last_val);
	qspi_send(0, 0xef);
	qspi_send(1, 0xd0);
	qspi_send(1, 0xe8);
	lcm_write_cmd(0x14, 0x03);
	mutex_unlock(&panel_lock);
	wake_unlock(&panel_idle_lock);

	LCMDBG("%s: last_val = %d\n", __func__,last_val);
	led_trigger_event(amoled_lcd_backlight, LED_FULL);
	return 0;
}

static int amoled_panel_blank(struct platform_device *pdev)
{
	LCMDBG("%s\n", __func__);
	mutex_lock(&panel_lock);
	lcm_write_tb(lcm_standby_seq, ARRAY_SIZE(lcm_standby_seq));
	mutex_unlock(&panel_lock);
	amoled_panel_power(0);
	led_trigger_event(amoled_lcd_backlight, LED_OFF);
	return 0;
}

static struct msm_fb_panel_data amoled_panel_data = {
	.on = amoled_panel_unblank,
	.off = amoled_panel_blank,
};

static struct platform_device this_device = {
	.name	= "lcdc_panel",
	.id	= 1,
	.dev	= {
		.platform_data = &amoled_panel_data,
	},
};

void amoled_brightness_set(struct led_classdev *led_cdev,
			enum led_brightness val)
{
	led_cdev->brightness = val;

	mutex_lock(&panel_lock);
	new_val = val;
	amoled_set_gamma_val(new_val);
	mutex_unlock(&panel_lock);
}

static int amoled_panel_detect(void)
{
	return panel_type;
}

static struct led_classdev amoled_backlight_led = {
	.name = "lcd-backlight",
	.brightness = LED_FULL,
};

static int amoled_backlight_probe(struct platform_device *pdev)
{
	int rc;
	panel_id = amoled_panel_detect();

	amoled_backlight_led.brightness_set = amoled_brightness_set;

	rc = led_classdev_register(&pdev->dev, &amoled_backlight_led);
	if (rc)
		LCMDBG("backlight: failure on register led_classdev\n");
	return 0;
}

static struct platform_device amoled_backlight = {
	.name = "amoled-backlight",
};

static struct platform_driver amoled_backlight_driver = {
	.probe		= amoled_backlight_probe,
	.driver		= {
		.name	= "amoled-backlight",
		.owner	= THIS_MODULE,
	},
};

static int __init amoled_init_panel(void)
{
	int ret;

	printk(KERN_DEBUG "%s\n", __func__);

	/* set gpio to proper state in the beginning */
	if (panel_power_gpio)
		(*panel_power_gpio)(1);

	wake_lock_init(&panel_idle_lock, WAKE_LOCK_SUSPEND,
			"backlight_present");

	ret = platform_device_register(&amoled_backlight);
	if (ret)
		return ret;

	led_trigger_register_simple("lcd-backlight-gate",
			&amoled_lcd_backlight);
	return 0;
}

static int amoled_probe(struct platform_device *pdev)
{
	int rc = -EIO;
	struct msm_panel_common_pdata *lcdc_amoled_pdata;

	lcdc_amoled_pdata = pdev->dev.platform_data;
	panel_power_gpio = lcdc_amoled_pdata->panel_config_gpio;
	printk("Panel type = %d\n", amoled_panel_detect());

	rc = amoled_init_panel();
	if (rc)
		printk(KERN_ERR "%s fail %d\n", __func__, rc);

	return rc;
}

static struct platform_driver this_driver = {
	.probe = amoled_probe,
	.driver = { .name = "lcdc_tl2796a_wvga" },
};

static int __init amoled_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	if (msm_fb_detect_client("lcdc_tl2796a_wvga"))
		return 0;

	ret = platform_driver_register(&this_driver);
	if (ret) {
		pr_err("%s: driver register failed, rc=%d\n", __func__, ret);
		return ret;
	}

	pinfo = &amoled_panel_data.panel_info;
	pinfo->xres = 480;
	pinfo->yres = 800;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 24576000;
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;

	pinfo->lcdc.h_back_porch = 4;
	pinfo->lcdc.h_front_porch = 8;
	pinfo->lcdc.h_pulse_width = 4;
	pinfo->lcdc.v_back_porch = 6;
	pinfo->lcdc.v_front_porch = 8;
	pinfo->lcdc.v_pulse_width = 2;
	pinfo->lcdc.border_clr = 0;
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;

	ret = platform_device_register(&this_device);
	if (ret) {
		printk(KERN_ERR "%s not able to register the device\n",
			__func__);
		platform_driver_unregister(&this_driver);
	}

	return ret;
}

static int __init amoled_backlight_init(void)
{
	return platform_driver_register(&amoled_backlight_driver);
}

device_initcall(amoled_init);
module_init(amoled_backlight_init);
