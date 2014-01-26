/*
 * Copyright (C) 2008 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <mach/panel_id.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

#define write_client_reg(val, cmd) mddi_queue_register_write(cmd, val, FALSE, 0);

static DECLARE_WAIT_QUEUE_HEAD(renesas_vsync_wait);
/* MDDI Commands */
static struct wake_lock panel_idle_lock;

struct mddi_cmd {
        unsigned char cmd;
        unsigned delay;
        unsigned int *vals;
        unsigned len;
};
#define prm_size 20
#define LCM_CMD(_cmd, _delay, ...)                              \
{                                                               \
        .cmd = _cmd,                                            \
        .delay = _delay,                                        \
        .vals = (u32 []){__VA_ARGS__},                           \
        .len = sizeof((u32 []){__VA_ARGS__}) / sizeof(u32)        \
}

#define LCM_INT(a, b, c, d) a + (b << 8) + (c << 16) + (d << 24)
/*
  .vals = (u8 []){__VA_ARGS__},                                 
  .len = sizeof((u8 []){__VA_ARGS__}) / sizeof(u8)              
*/
#define DEFAULT_BRIGHTNESS 255
#define PWM_USER_DEF	 		143
#define PWM_USER_MIN			30
#define PWM_USER_DIM			 9
#define PWM_USER_MAX			255

#define PWM_HITACHI_DEF			174
#define PWM_HITACHI_MIN			 10
#define PWM_HITACHI_MAX			255

enum {
	GATE_ON = 1 << 0,
};
static struct renesas_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} renesas;

static struct mddi_cmd tear[] = {
  LCM_CMD(0x44, 0, 0x01, 0x90)
		};

static struct mddi_cmd saga_renesas_cmd[] = {
	LCM_CMD(0x2A, 0, 0x00, 0x00, 0x01, 0xDF),
	LCM_CMD(0x2B, 0, 0x00, 0x00, 0x03, 0x1F),
	LCM_CMD(0x36, 0, 0x00),
	LCM_CMD(0x3A, 0, 0x55),//set_pixel_format 0x66 for 18bit/pixel, 0x77 for 24bit/pixel
};
static struct mddi_cmd saga_renesas_backlight_blank_cmd[] = {
#if 0
  LCM_CMD(0xB9, 0, 0x00, 0x00, 0x00, 0x00,
          0x66, 0x00, 0x00, 0x00,
			 0x04, 0x00, 0x00, 0x00,//adjust PWM frequency to 10.91k .
          0x08, 0x00, 0x00, 0x00,),
#endif
  LCM_CMD(0xB0, 0, 0x04),
  LCM_CMD(0xB9, 0, 0x00, 0xff, 0x04, 0x08),

};
static struct mddi_cmd gama[] = {
           LCM_CMD(0xB0, 0, 0x04),
           LCM_CMD(0xC1, 0, 0x43,
                   0x31,
                                 0x00,
                                 0x21,
                                 0x21,
                                 0x32,
                                 0x12,
                                 0x28,
                                 0x4A,
                                 0x1E, 0x00, 0x00, 0x00,
                                 0xA5, 0x00, 0x00, 0x00,
                                 0x0F, 0x00, 0x00, 0x00,
                                 0x58, 0x00, 0x00, 0x00,
                                 0x21, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00
           ),
           LCM_CMD(0xC8, 0, 0x2D, 0x00, 0x00, 0x00,
                                 0x2F, 0x00, 0x00, 0x00,
                                 0x31, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x3E, 0x00, 0x00, 0x00,
                                 0x51, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x23, 0x00, 0x00, 0x00,
                                 0x16, 0x00, 0x00, 0x00,
                                 0x0B, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00,
                                 0x2D, 0x00, 0x00, 0x00,
                                 0x2F, 0x00, 0x00, 0x00,
                                 0x31, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x3E, 0x00, 0x00, 0x00,
                                 0x51, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x23, 0x00, 0x00, 0x00,
                                 0x16, 0x00, 0x00, 0x00,
                                 0x0B, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00
           ),
           LCM_CMD(0xC9, 0, 0x00, 0x00, 0x00, 0x00,
                                 0x0F, 0x00, 0x00, 0x00,
                                 0x18, 0x00, 0x00, 0x00,
                                 0x25, 0x00, 0x00, 0x00,
                                 0x33, 0x00, 0x00, 0x00,
                                 0x4D, 0x00, 0x00, 0x00,
                                 0x38, 0x00, 0x00, 0x00,
                                 0x25, 0x00, 0x00, 0x00,
                                 0x18, 0x00, 0x00, 0x00,
                                 0x11, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00,
                                 0x0F, 0x00, 0x00, 0x00,
                                 0x18, 0x00, 0x00, 0x00,
                                 0x25, 0x00, 0x00, 0x00,
                                 0x33, 0x00, 0x00, 0x00,
                                 0x4D, 0x00, 0x00, 0x00,
                                 0x38, 0x00, 0x00, 0x00,
                                 0x25, 0x00, 0x00, 0x00,
                                 0x18, 0x00, 0x00, 0x00,
                                 0x11, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00
           ),
           LCM_CMD(0xCA, 0, 0x27, 0x00, 0x00, 0x00,
                                 0x2A, 0x00, 0x00, 0x00,
                                 0x2E, 0x00, 0x00, 0x00,
                                 0x34, 0x00, 0x00, 0x00,
                                 0x3C, 0x00, 0x00, 0x00,
                                 0x51, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x24, 0x00, 0x00, 0x00,
                                 0x16, 0x00, 0x00, 0x00,
                                 0x0C, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00,
                                 0x27, 0x00, 0x00, 0x00,
                                 0x2A, 0x00, 0x00, 0x00,
                                 0x2E, 0x00, 0x00, 0x00,
                                 0x34, 0x00, 0x00, 0x00,
                                 0x3C, 0x00, 0x00, 0x00,
                                 0x51, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x24, 0x00, 0x00, 0x00,
                                 0x16, 0x00, 0x00, 0x00,
                                 0x0C, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00 ),
           LCM_CMD(0xD5, 0, 0x14, 0x00, 0x00, 0x00,
                                 0x14, 0x00, 0x00, 0x00
           ),
           LCM_CMD(0xB0, 0, 0x03, 0x00, 0x00, 0x00),
};

static void
do_renesas_cmd(struct mddi_cmd *cmd_table, ssize_t size)
{
	struct mddi_cmd *pcmd = NULL;
        int ret;
	for (pcmd = cmd_table; pcmd < cmd_table + size; pcmd++) {
          ret = mddi_host_register_multiwrite(pcmd->cmd, pcmd->vals, pcmd->len, TRUE, 0, MDDI_HOST_PRIM);
          if (ret != 0)
            printk(KERN_ERR "%s: failed multiwrite (%d)\n", __func__, ret);
          //write_client_reg(pcmd->vals, pcmd->cmd, pcmd->len);
          if (pcmd->delay)
            msleep(pcmd->delay);
	}
}

/* Backlight */

static enum led_brightness brightness_value = DEFAULT_BRIGHTNESS;

/* use one flag to have better backlight on/off performance */
static int saga_set_dim = 1;


static int
saga_shrink_pwm(int brightness, int user_def,
		int user_min, int user_max, int panel_def,
		int panel_min, int panel_max)
{
	if (brightness < PWM_USER_DIM) {
		return 0;
	}

	if (brightness < user_min) {
		return panel_min;
	}

	if (brightness > user_def) {
		brightness = (panel_max - panel_def) *
			(brightness - user_def) /
			(user_max - user_def) +
			panel_def;
	} else {
			brightness = (panel_def - panel_min) *
			(brightness - user_min) /
			(user_def - user_min) +
			panel_min;
	}

        return brightness;
}

static void
saga_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	unsigned int shrink_br = val;
	struct mddi_cmd *pcmd = saga_renesas_backlight_blank_cmd;
        return ;
	printk(KERN_DEBUG "set brightness = %d\n", val);
        printk(KERN_ERR "%s: brightness enter: %d\n", __func__, val);
	if (test_bit(GATE_ON, &renesas.status) == 0)
		return;
        printk(KERN_ERR "%s: brightness: test_bit passed\n", __func__);

	shrink_br = saga_shrink_pwm(val, PWM_USER_DEF,
				PWM_USER_MIN, PWM_USER_MAX, PWM_HITACHI_DEF,
				PWM_HITACHI_MIN, PWM_HITACHI_MAX);
        pcmd->vals[1] = shrink_br;

	mutex_lock(&renesas.lock);
	if (saga_set_dim == 1) {
		/* we dont need set dim again */
		saga_set_dim = 0;
	}
        //        write_client_reg(0x04, 0xB0);
        do_renesas_cmd(pcmd, 1);
        //        write_client_reg(0x03, 0xB0);
	brightness_value = val;
	mutex_unlock(&renesas.lock);
}

static enum led_brightness
saga_get_brightness(struct led_classdev *led_cdev)
{

	return brightness_value;
}


static void
saga_backlight_switch(int on)
{
	enum led_brightness val;
        printk(KERN_ERR "%s: %d\n", __func__, on);

	if (on != 0) {
		printk(KERN_DEBUG "turn on backlight\n");
		set_bit(GATE_ON, &renesas.status);
		val = renesas.lcd_backlight.brightness;
		/* LED core uses get_brightness for default value
		  If the physical layer is not ready, we should*/
		if (val == 0)
			val = DEFAULT_BRIGHTNESS;
                //		saga_set_brightness(&renesas.lcd_backlight, val);
		 /*set next backlight value with dim */
		//glacier_set_dim = 1;
	} else {
		do_renesas_cmd(saga_renesas_backlight_blank_cmd, ARRAY_SIZE(saga_renesas_backlight_blank_cmd));
		saga_set_brightness(&renesas.lcd_backlight, 0);
		clear_bit(GATE_ON, &renesas.status);
	}
}

static int
saga_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;

	mutex_init(&renesas.lock);
	renesas.lcd_backlight.name = "lcd-backlight";
	renesas.lcd_backlight.brightness_set = saga_set_brightness;
	renesas.lcd_backlight.brightness_get = saga_get_brightness;
	err = led_classdev_register(&pdev->dev, &renesas.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;
	return 0;

err_register_lcd_bl:
	led_classdev_unregister(&renesas.lcd_backlight);
	return err;
}


static int renesas_init(void)
{
	do_renesas_cmd(saga_renesas_cmd, ARRAY_SIZE(saga_renesas_cmd));
        return 0;
}

static int renesas_lcd_off(struct platform_device *pdev)
{
  printk(KERN_DEBUG "%s +\n", __func__);
  //  write_client_reg(0x04, 0xB0);
  write_client_reg(0x0, 0x28);
  saga_backlight_switch(LED_OFF);
  write_client_reg(0x0, 0xB8);
  //  write_client_reg(0x03, 0xB0);
  msleep(72);
        // uninit
        //	client_data->auto_hibernate(client_data, 0);
  write_client_reg(0x0, 0x10);
  msleep(72);
  //	client_data->auto_hibernate(client_data, 1); 
  //	return bridge_data->blank(bridge_data, client_data);
  return 0;
}

static int renesas_lcd_on(struct platform_device *pdev)
{
  printk(KERN_DEBUG "%s +\n", __func__);

  renesas_init();
          //	client_data->auto_hibernate(client_data, 0);

  //write_client_reg(0x04, 0xB0);

  write_client_reg(0x0, 0x11);
  msleep(125);
  //        do_renesas_cmd(gama, ARRAY_SIZE(gama));
  saga_backlight_switch(LED_FULL);
  write_client_reg(0x0, 0x29);
  do_renesas_cmd(tear, ARRAY_SIZE(tear));
  write_client_reg(0x0, 0x35);
  //  write_client_reg(0x03, 0xB0);
  
  //	client_data->auto_hibernate(client_data, 1);
  
  printk(KERN_DEBUG "%s -\n", __func__);
  return 0;
}

static struct platform_device renesas_backlight = {
	.name = "renesas_backlight",
};


static int __devinit renesas_probe(struct platform_device *pdev)
{
	int ret;
	struct msm_panel_common_pdata *mddi_renesas_pdata;
        printk(KERN_ERR "%s +\n", __func__);
        //        msm_fb_add_device(pdev);
	mddi_renesas_pdata = pdev->dev.platform_data;
	wake_lock_init(&panel_idle_lock, WAKE_LOCK_SUSPEND,
			"backlight_present");

	ret = platform_device_register(&renesas_backlight);
	if (ret)
		return ret;
        msm_fb_add_device(pdev);
	return 0;
}


static struct platform_driver this_driver = {
	.probe  = renesas_probe,
	.driver = {
		.name   = "mddi_renesas_R61408_wvga",
	},
};

static struct msm_fb_panel_data renesas_panel_data = {
	.on = renesas_lcd_on,
	.off = renesas_lcd_off,
};

static struct platform_device this_device = {
	.name   = "mddi_renesas_R61408_wvga",
	.id	= 0,
	.dev	= {
		.platform_data = &renesas_panel_data,
	}
};

static struct platform_driver saga_backlight_driver = {
	.probe = saga_backlight_probe,
	.driver = {
		.name = "renesas_backlight",
		.owner = THIS_MODULE,
	},
};

static int __init mddi_renesas_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;
	printk(KERN_ERR "%s + (%d/%d)\n", __func__, panel_type, PANEL_ID_SAG_HITACHI);
	if (panel_type != PANEL_ID_SAG_HITACHI)
		return -ENODEV;
	printk(KERN_ERR "%s +\n", __func__);

	u32 id;
	ret = msm_fb_detect_client("mddi_renesas_R61408_wvga");
	if (ret == -ENODEV)
		return 0;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	if (ret) {
		id = mddi_get_client_id();
		if (((id >> 16) != 0x4474) || ((id & 0xffff) == 0x8960))
			return 0;
	}
#endif

	ret = platform_driver_register(&this_driver);
	if (!ret) {
		pinfo = &renesas_panel_data.panel_info;
		pinfo->xres = 480;
		pinfo->yres = 800;
		MSM_FB_SINGLE_MODE_PANEL(pinfo);
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
                pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		pinfo->wait_cycle = 0;
                //		pinfo->mddi.is_type1 = TRUE;
		pinfo->bpp = 16;
		pinfo->fb_num = 2;
		pinfo->clk_rate = 153600000;
		pinfo->clk_min = 140000000;
		pinfo->clk_max = 160000000;
                //		pinfo->lcd.vsync_enable = TRUE;
                //		pinfo->lcd.refx100 = 6050;
		pinfo->lcd.v_back_porch = 23;
		pinfo->lcd.v_front_porch = 20;
		pinfo->lcd.v_pulse_width = 105;
                //		pinfo->lcd.hw_vsync_mode = TRUE;
                //		pinfo->lcd.vsync_notifier_period = 0;

		ret = platform_device_register(&this_device);
		if (ret)
			platform_driver_unregister(&this_driver);                     
	}

	return ret;
}

static int __init renesas_backlight_init(void)
{
	return platform_driver_register(&saga_backlight_driver);
}

device_initcall(mddi_renesas_init);
module_init(renesas_backlight_init);

