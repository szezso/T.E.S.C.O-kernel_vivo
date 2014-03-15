/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2011 HTC Corporation.
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
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>

#include "board-vivo.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4329";

/* bt on configuration */
static uint32_t vivo_bt_on_table[] = {

	/* BT_RTS */
	GPIO_CFG(VIVO_GPIO_BT_UART1_RTS,
				1,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA),
	/* BT_CTS */
	GPIO_CFG(VIVO_GPIO_BT_UART1_CTS,
				1,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	/* BT_RX */
	GPIO_CFG(VIVO_GPIO_BT_UART1_RX,
				1,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	/* BT_TX */
	GPIO_CFG(VIVO_GPIO_BT_UART1_TX,
				1,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA),

	/* BT_HOST_WAKE */
	GPIO_CFG(VIVO_GPIO_BT_HOST_WAKE,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_4MA),
	/* BT_CHIP_WAKE */
	GPIO_CFG(VIVO_GPIO_BT_CHIP_WAKE,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_4MA),

	/* BT_RESET_N */
	GPIO_CFG(VIVO_GPIO_BT_RESET_N,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_4MA),
	/* BT_SHUTDOWN_N */
	GPIO_CFG(VIVO_GPIO_BT_SHUTDOWN_N,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_4MA),
};

/* bt off configuration */
static uint32_t vivo_bt_off_table[] = {

	/* BT_RTS */
	GPIO_CFG(VIVO_GPIO_BT_UART1_RTS,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	/* BT_CTS */
	GPIO_CFG(VIVO_GPIO_BT_UART1_CTS,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	/* BT_RX */
	GPIO_CFG(VIVO_GPIO_BT_UART1_RX,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	/* BT_TX */
	GPIO_CFG(VIVO_GPIO_BT_UART1_TX,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),

	/* BT_RESET_N */
	GPIO_CFG(VIVO_GPIO_BT_RESET_N,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_4MA),
	/* BT_SHUTDOWN_N */
	GPIO_CFG(VIVO_GPIO_BT_SHUTDOWN_N,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_4MA),

	/* BT_HOST_WAKE */
	GPIO_CFG(VIVO_GPIO_BT_HOST_WAKE,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN,
				GPIO_CFG_4MA),
	/* BT_CHIP_WAKE */
	GPIO_CFG(VIVO_GPIO_BT_CHIP_WAKE,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_4MA),
};

static void config_bt_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[BT]%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void vivo_config_bt_on(void)
{
	printk(KERN_INFO "[BT]== R ON ==\n");

	/* set bt on configuration*/
	config_bt_table(vivo_bt_on_table,
				ARRAY_SIZE(vivo_bt_on_table));
	mdelay(2);

	/* BT_RESET_N */
	gpio_set_value(VIVO_GPIO_BT_RESET_N, 0);
	mdelay(1);

	/* BT_SHUTDOWN_N */
	gpio_set_value(VIVO_GPIO_BT_SHUTDOWN_N, 0);
	mdelay(5);

	/* BT_SHUTDOWN_N */
	gpio_set_value(VIVO_GPIO_BT_SHUTDOWN_N, 1);
	mdelay(1);

	/* BT_RESET_N */
	gpio_set_value(VIVO_GPIO_BT_RESET_N, 1);
	mdelay(2);

}

static void vivo_config_bt_off(void)
{
	/* BT_RESET_N */
	gpio_set_value(VIVO_GPIO_BT_RESET_N, 0);
	mdelay(1);

	/* BT_SHUTDOWN_N */
	gpio_set_value(VIVO_GPIO_BT_SHUTDOWN_N, 0);
	mdelay(1);

	/* set bt off configuration*/
	config_bt_table(vivo_bt_off_table,
				ARRAY_SIZE(vivo_bt_off_table));
	mdelay(2);

	/* BT_RTS */
	/* BT_CTS */
	/* BT_TX */
	/* BT_RX */

	/* BT_HOST_WAKE */

	/* BT_CHIP_WAKE */
	gpio_set_value(VIVO_GPIO_BT_CHIP_WAKE, 0);

	printk(KERN_INFO "[BT]== R OFF ==\n");
}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked)
		vivo_config_bt_on();
	else
		vivo_config_bt_off();

	return 0;
}

#if 0

static int bluetooth_set_power(void *data, bool blocked)
{
        if (!blocked) {
                gpio_direction_output(VIVO_GPIO_BT_RESET_N, 1);
                gpio_direction_output(VIVO_GPIO_BT_SHUTDOWN_N, 1);
        } else {
                gpio_direction_output(VIVO_GPIO_BT_SHUTDOWN_N, 0);
                gpio_direction_output(VIVO_GPIO_BT_RESET_N, 0);
        }
        return 0;
}
#endif

static struct rfkill_ops vivo_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int vivo_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true;  /* off */

	/* always turn on clock? */
	/* htc_wifi_bt_sleep_clk_ctl(CLK_ON, ID_BT); */
	mdelay(2);

#if 0
        rc = gpio_request(VIVO_GPIO_BT_RESET_N, "bt_reset");
        if (rc)
                goto err_gpio_reset;
        rc = gpio_request(VIVO_GPIO_BT_SHUTDOWN_N, "bt_shutdown");
        if (rc)
                goto err_gpio_shutdown;

#endif
	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&vivo_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
	return rc;

#if 0
err_rfkill_reg:
        rfkill_destroy(bt_rfk);
err_rfkill_alloc:
        gpio_free(VIVO_GPIO_BT_SHUTDOWN_N);
err_gpio_shutdown:
        gpio_free(VIVO_GPIO_BT_RESET_N);
err_gpio_reset:
        return rc;
#endif
}

static int vivo_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
#if 0
        gpio_free(VIVO_GPIO_BT_SHUTDOWN_N);
        gpio_free(VIVO_GPIO_BT_RESET_N);
#endif
	return 0;
}

static struct platform_driver vivo_rfkill_driver = {
	.probe = vivo_rfkill_probe,
	.remove = vivo_rfkill_remove,
	.driver = {
		.name = "vivo_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init vivo_rfkill_init(void)
{

	if (!machine_is_vivo())
		return 0;

	return platform_driver_register(&vivo_rfkill_driver);
}

static void __exit vivo_rfkill_exit(void)
{
	platform_driver_unregister(&vivo_rfkill_driver);
}

module_init(vivo_rfkill_init);
module_exit(vivo_rfkill_exit);
MODULE_DESCRIPTION("vivo rfkill");
MODULE_AUTHOR("htc_ssdbt <htc_ssdbt@htc.com>");
MODULE_LICENSE("GPL");
