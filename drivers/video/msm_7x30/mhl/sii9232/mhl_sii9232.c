/***************************************************************************
 *
 *   SiI9232 - MHL Transmitter Driver
 *
 * Copyright (C) 2011 SiliconImage, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *****************************************************************************/

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/fb.h>
#include <linux/msm_mdp.h>
#include <mach/msm_fb-7x30.h>
#include <mach/htc_mhl.h>
#include <mach/debug_display.h>
#ifdef CONFIG_HTC_HEADSET_MGR
#include <mach/htc_headset_mgr.h>
#endif
#include "../../hdmi_common.h"
#include "../../mdp_hw.h"
#include "mhl_tpi.h"
#include <mach/board.h>

#define DEBUG
#define HONEYCOMB_HDMI
#define MHL_RCP_KETEVENT

#ifdef MHL_RCP_KETEVENT
struct input_dev *input_dev;
#endif
bool sii9232_power_low = false;

#define SII9232_I2C_RETRY_COUNT 2

struct mhl_sii9232_info
{
	struct mhl_info info;
	struct hdmi_common_state_type hdmi_common;
	struct workqueue_struct *wq;
	struct wake_lock wake_lock;
	struct work_struct work;
	void (*power_switch)(int);
	int reset_pin;
	int intr_pin;
	int irq;
	bool isMHL;
	struct work_struct mhl_notifier_work;
#ifdef HONEYCOMB_HDMI
	struct switch_dev hpd_switch;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif //CONFIG_HAS_EARLYSUSPEND
};
extern  struct mdp_dtv_info *_dtv_info;
static struct mhl_sii9232_info *sii9232_info_ptr;
static struct  i2c_client *sii9232_i2c_client;
struct timer_list       poll_timer;
static void sii9232_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sii9232_irq_work, sii9232_irq_do_work);
static bool init = false;
static bool first_plug = false;
extern uint8_t txPowerState;
extern bool g_bI2CBusError;

static DEFINE_MUTEX(mhl_early_suspend_sem);
static bool g_bEnterEarlySuspend = false;
static bool g_bGotUsbBus = false;


// -----------------------------------------------------------------------------
//                         External routine declaration
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
//                         Common Routine Implementation
// -----------------------------------------------------------------------------
#ifdef CONFIG_USB_ACCESSORY_DETECT
static DEFINE_MUTEX(mhl_notify_sem);
void update_mhl_status(bool isMHL)
{
	struct mhl_sii9232_info *pInfo = sii9232_info_ptr;

	pInfo->isMHL = isMHL;
	if (!init) return;
	PR_DISP_INFO("%s: %d\n", __func__, isMHL);

	if( !isMHL ) g_bGotUsbBus = false;

	queue_work(pInfo->wq, &pInfo->mhl_notifier_work);
}
static void send_mhl_connect_notify(struct work_struct *w)
{
	static struct t_mhl_status_notifier *mhl_notifier;
	struct mhl_sii9232_info *pInfo = sii9232_info_ptr;

	if (!pInfo)
		return;

	PR_DISP_INFO("%s: %d\n", __func__, pInfo->isMHL);
	mutex_lock(&mhl_notify_sem);
	list_for_each_entry(mhl_notifier,
		&g_lh_mhl_detect_notifier_list,
		mhl_notifier_link) {
			if (mhl_notifier->func != NULL)
				mhl_notifier->func(pInfo->isMHL, false);
		}
	mutex_unlock(&mhl_notify_sem);
}

int mhl_detect_register_notifier(struct t_mhl_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&mhl_notify_sem);
	list_add(&notifier->mhl_notifier_link,
		&g_lh_mhl_detect_notifier_list);
	mutex_unlock(&mhl_notify_sem);
	return 0;
}
#endif

static int sii9232_get_intr_status(void)
{
	return gpio_get_value(sii9232_info_ptr->intr_pin);
}

static void sii9232_reset(void)
{
	gpio_set_value(sii9232_info_ptr->reset_pin, 0);
	msleep(100);
	gpio_set_value(sii9232_info_ptr->reset_pin, 1);
}

static void sii9232_video_switch(int on_off)
{
	_dtv_info->dtv_relay(_dtv_info, on_off);
}


static int sii9232_read_edid_block(int block, uint8_t *edid_buf)
{
	if (block > 3)
		return -1;
	memcpy(edid_buf, sii9232_info_ptr->info.edid_buf + block * 0x80, 0x80);

	return 0;
}

/* Change HDMI state */
static void sii9232_change_hdmi_state(int online)
{
	struct hdmi_common_state_type *hdmi_common;
	hdmi_common = &sii9232_info_ptr->hdmi_common;
        mutex_lock(&hdmi_common_state_hpd_mutex);
        hdmi_common->hpd_state = online;
        mutex_unlock(&hdmi_common_state_hpd_mutex);

        if (!hdmi_common->uevent_kobj)
                return;

        if (online)
                kobject_uevent(hdmi_common->uevent_kobj,
                        KOBJ_ONLINE);
        else
                kobject_uevent(hdmi_common->uevent_kobj,
                        KOBJ_OFFLINE);
#ifdef CONFIG_HTC_HEADSET_MGR
		switch_send_event(BIT_HDMI_CABLE, online);
		switch_send_event(BIT_HDMI_AUDIO, online);
#endif
#ifdef HONEYCOMB_HDMI
		switch_set_state(&sii9232_info_ptr->hpd_switch, online);
#endif
        PR_DISP_INFO("sii9232_uevent: hdmi_state(%d)\n", online);
}

static void sii9232_change_hdcp_state(int state)
{
	struct hdmi_common_state_type *hdmi_common;
	hdmi_common = &sii9232_info_ptr->hdmi_common;
        mutex_lock(&hdmi_common_state_hpd_mutex);
        hdmi_common->hdcp_active = state;
        mutex_unlock(&hdmi_common_state_hpd_mutex);

        if (!hdmi_common->uevent_kobj)
                return;

        if (state)
                kobject_uevent(hdmi_common->uevent_kobj,
                        KOBJ_ONLINE);
        else
                kobject_uevent(hdmi_common->uevent_kobj,
                        KOBJ_OFFLINE);
        PR_DISP_INFO("sii9232_uevent: hdcp_state %d\n", state);
}


static uint32_t sii9232_check_hdmi_sink(void)
{
	struct hdmi_common_state_type *hdmi_common;
	hdmi_common = &sii9232_info_ptr->hdmi_common;
	return hdmi_common->hdmi_sink;
}

static void sii9232_device_suspend(void)
{
	PR_DISP_INFO("%s\n", __func__);
	sii9232_power_low = true;

	if (sii9232_info_ptr->power_switch) {
		sii9232_info_ptr->power_switch(0);
	}
	// Reset the I2C bus error variable...
	g_bI2CBusError = false;
}



static void sii9232_device_wakeup(void)
{
	PR_DISP_INFO("%s\n", __func__);
	if (!init) {
		first_plug = true;
                return;
	}
	sii9232_power_low = false;
	if (sii9232_info_ptr->power_switch) {
		sii9232_info_ptr->power_switch(1);
	}
	tpi_hw_init(&sii9232_info_ptr->info);
	mod_timer(&poll_timer, jiffies + HZ/2);
}

void mhl_device_wakeup()
{
	PR_DISP_INFO("%s\n", __func__);

	// USB driver will call this routine to startup the MHL connection.
	// The "mhl_early_suspend_sem" mutex can avoid the race condition which "TPI_Poll()" routine still was running.
	// It can protect that only one thread try to control the MHL transmitter HW register.
	// Otherwise, it was possible that previous timer call-back didn't complete but another thread try to access the HW register also...
	mutex_lock(&mhl_early_suspend_sem);
	g_bGotUsbBus = true;
	sii9232_device_wakeup();
	mutex_unlock(&mhl_early_suspend_sem);
}

static void sii9232_send_keyevent(uint32_t key, uint32_t type)
{
#ifdef MHL_RCP_KETEVENT
        PR_DISP_INFO("CBUS key_event: %d\n", key);
	if (type == 0) {
		input_report_key(input_dev, key, 1);
        	input_report_key(input_dev, key, 0);
        	input_sync(input_dev);
	}
#endif
}

static void sii9232_irq_do_work(struct work_struct *work)
{
	mutex_lock(&mhl_early_suspend_sem);
	if( !g_bEnterEarlySuspend ){
		TPI_Poll();
		if (txPowerState == 0x03 || g_bI2CBusError) {
			#ifdef CONFIG_USB_ACCESSORY_DETECT
			update_mhl_status(false);
			#endif //CONFIG_USB_ACCESSORY_DETECT
			sii9232_device_suspend();
		}
		else {
			mod_timer(&poll_timer, jiffies + HZ/20);
		}
	}
	mutex_unlock(&mhl_early_suspend_sem);
}

static void poll_timer_func(unsigned long arg)
{
        struct mhl_sii9232_info *sii9232 = (struct mhl_sii9232_info *)arg;
	queue_work(sii9232->wq, &sii9232_irq_work);
}

static int sii9232_i2c_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
        int rc = 0;
        if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
                rc = -ENOTSUPP;
                goto probe_failure;
        }

        i2c_set_clientdata(client, sii9232_info_ptr);
        sii9232_i2c_client = client;

        return 0;

probe_failure:
        sii9232_i2c_client = NULL;
        return rc;
}

#ifdef MHL_RCP_KETEVENT
/* Sysfs method to input simulated coordinates */
static ssize_t write_keyevent(struct device *dev,
                              struct device_attribute *attr,
                              const char *buffer, size_t count)
{
        int key;

        /* parsing input data */
        sscanf(buffer, "%d", &key);


        PR_DISP_INFO("key_event: %d\n", key);

        /* Report key event */
        switch(key) {
       case 0:
                input_report_key(input_dev, KEY_HOME, 1);
                input_report_key(input_dev, KEY_HOME, 0);
               break;
       case 1:
                input_report_key(input_dev, KEY_UP, 1);
                input_report_key(input_dev, KEY_UP, 0);
               break;
       case 2:
                input_report_key(input_dev, KEY_DOWN, 1);
                input_report_key(input_dev, KEY_DOWN, 0);
               break;
       case 3:
                input_report_key(input_dev, KEY_LEFT, 1);
                input_report_key(input_dev, KEY_LEFT, 0);
               break;
       case 4:
                input_report_key(input_dev, KEY_RIGHT, 1);
                input_report_key(input_dev, KEY_RIGHT, 0);
               break;
       case 5:
                input_report_key(input_dev, KEY_ENTER, 1);
                input_report_key(input_dev, KEY_ENTER, 0);
		break;
       case 6:
                input_report_key(input_dev, KEY_SELECT, 1);
                input_report_key(input_dev, KEY_SELECT, 0);
		break;


       default:
                input_report_key(input_dev, KEY_OK, 1);
                input_report_key(input_dev, KEY_OK, 0);
		break;
	}
        input_sync(input_dev);

        return count;

}

/* Attach the sysfs write method */
static DEVICE_ATTR(rcp_event, 0644, NULL, write_keyevent);

#endif

static const struct i2c_device_id sii9232_i2c_id[] = {
	{MHL_SII9232_I2C_NAME, 0},
	{}
};

static struct i2c_driver sii9232_i2c_driver = {
	.id_table = sii9232_i2c_id,
	.probe = sii9232_i2c_probe,
	//.remove = sii9232_i2c_remove,
	.driver = {
		.name = MHL_SII9234_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sii9232_early_suspend(struct early_suspend *h)
{
	struct mhl_sii9232_info *pInfo = sii9232_info_ptr;

	printk(KERN_INFO "%s(%d)\n", __func__, pInfo->isMHL );

	// Enter the early suspend state...
	g_bEnterEarlySuspend = true;

	mutex_lock(&mhl_early_suspend_sem);
	if( pInfo->isMHL ){
		// Delete the TPI polling timer...
		del_timer(&poll_timer);
		// Suspend the device which still pluged in the MHL device.
		// Power down the MHL transmitter, otherwise it may crash after resume...
		sii9232_device_suspend();
	}
	mutex_unlock(&mhl_early_suspend_sem);
}

static void sii9232_late_resume(struct early_suspend *h)
{
	struct mhl_sii9232_info *pInfo = sii9232_info_ptr;

	printk(KERN_INFO "%s(isMHL=%d, g_bGotUsbBus=%d)\n", __func__, pInfo->isMHL, g_bGotUsbBus );

	if( pInfo->isMHL || g_bGotUsbBus ){
		// Suspend the device which still pluged in the MHL device.
		// We power down it in the early-suspend, now we need to re-initiate it...

		// Notify the HDMI demand for cable out...
		sii9232_change_hdmi_state(false);
		mhl_device_wakeup();

		// Power down the transmitter to enter the D2 mode...
		if (sii9232_info_ptr->power_switch) {
			sii9232_info_ptr->power_switch(0);
		}
		// Notify the calbe out...
		update_mhl_status(false);
	}

	// Leave the early suspend state...
	g_bEnterEarlySuspend = false;
}
#endif //CONFIG_HAS_EARLYSUSPEND

static int sii9232_probe(struct platform_device *pdev)
{
	struct mhl_sii9232_info *sii9232;
	struct mhl_platform_data *pdata;
	int ret = 0;


	ret = i2c_add_driver(&sii9232_i2c_driver);
        if (ret < 0 || sii9232_i2c_client == NULL) {
                ret = -ENOTSUPP;
            pr_err("sii9232_init FAILED: i2c add error rc=%d\n",
                   ret);				
                return ret;
    }

	sii9232 = (struct mhl_sii9232_info *) kzalloc(sizeof(struct mhl_sii9232_info), GFP_KERNEL);

	if (!sii9232) {
		pr_err("%s: alloc memory error!!\n", __func__);
		return -ENOMEM;
	}

	sii9232->info.i2c_client = sii9232_i2c_client;
	pdata = sii9232_i2c_client->dev.platform_data;

	if (!pdata) {
		pr_err("%s: Assign platform_data error!!\n", __func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}
	sii9232->irq = sii9232_i2c_client->irq;

	sii9232->reset_pin = pdata->gpio_reset;
	sii9232->intr_pin = pdata->gpio_intr;
	sii9232->power_switch = pdata->power_switch;
	sii9232->info.i2c_addr_tpi = pdata->i2c_addr_tpi;
	sii9232->info.i2c_addr_cbus = pdata->i2c_addr_cbus;		
	sii9232->info.reset_chip = sii9232_reset;
	sii9232->info.get_int_status = sii9232_get_intr_status;
	sii9232->info.video_switch = sii9232_video_switch; 
#ifdef CONFIG_USB_ACCESSORY_DETECT
	sii9232->info.cable_status_update = update_mhl_status;
#endif

	sii9232_info_ptr = sii9232;
	sii9232_info_ptr->hdmi_common.dev = &pdev->dev;
	
	/* Pin Config */
	gpio_request(sii9232->reset_pin, "mhl_sii9232_gpio_reset");
	gpio_direction_output(sii9232->reset_pin, 0);
	gpio_request(sii9232->intr_pin, "mhl_sii9232_gpio_intr");
	gpio_direction_input(sii9232->intr_pin);

	hdmi_common_state = &sii9232->hdmi_common;
	HDMI_SETUP_LUT(720x480p60_16_9);
	HDMI_SETUP_LUT(1280x720p60_16_9);
	hdmi_common_state->video_resolution = HDMI_VFRMT_1280x720p60_16_9;
	hdmi_common_state_create(&_dtv_info->fb_pdev);

	hdmi_common_state->read_edid_block = sii9232_read_edid_block;
	sii9232->info.read_edid =  hdmi_common_read_edid;
	sii9232->info.change_hdmi_state = sii9232_change_hdmi_state;
	sii9232->info.change_hdcp_state = sii9232_change_hdcp_state;
	sii9232->info.check_hdmi_sink = sii9232_check_hdmi_sink;
	sii9232->info.send_keyevent = sii9232_send_keyevent;
	sii9232->info.device_suspend = sii9232_device_suspend;
	sii9232->info.device_wakeup = sii9232_device_wakeup;

#ifdef HONEYCOMB_HDMI
	sii9232->hpd_switch.name = "hdmi";
	switch_dev_register(&sii9232->hpd_switch);
#endif
#ifdef CONFIG_USB_ACCESSORY_DETECT
	INIT_WORK(&sii9232->mhl_notifier_work, send_mhl_connect_notify);
#endif

	/* Power ON */
	if (sii9232->power_switch) {
		sii9232->power_switch(1);
	}
	/* device init to D3 */
	ret = tpi_hw_init(&sii9232->info);
	/* Power ON */
	if (sii9232->power_switch) {
		sii9232->power_switch(0);
	}



	if (ret < 0) {
		pr_err("%s: can't init\n", __func__);
		ret = -ENOMEM;
		goto err_init;
	}

	sii9232->wq = create_workqueue("mhl_sii9232_wq");

	if (!sii9232->wq)	{
		pr_err("%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_workqueue;
	}

	init_timer(&poll_timer);
	poll_timer.function = poll_timer_func;
	poll_timer.data = (unsigned long) sii9232;


	/* Create a sysfs node to read simulated coordinates */

#ifdef MHL_RCP_KETEVENT
	ret = device_create_file(&pdev->dev, &dev_attr_rcp_event);


	//gpio_set_value(sii9232_info_ptr->reset_pin, 0);

	input_dev = input_allocate_device();
	/* indicate that we generate key events */
	set_bit(EV_KEY, input_dev->evbit);
	/* indicate that we generate *any* key event */
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_ENTER, input_dev->keybit);
	set_bit(KEY_LEFT, input_dev->keybit);
	set_bit(KEY_UP, input_dev->keybit);
	set_bit(KEY_DOWN, input_dev->keybit);
	set_bit(KEY_RIGHT, input_dev->keybit);
	input_dev->name = "rcp_events";

	ret = input_register_device(input_dev);
	if (ret < 0)
		pr_err("MHL: can't register input devce\n");
#endif
	init = true;
	if (first_plug)
		mod_timer(&poll_timer, jiffies + HZ/20);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	sii9232->early_suspend.suspend = sii9232_early_suspend;
	sii9232->early_suspend.resume = sii9232_late_resume;
	sii9232->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	register_early_suspend(&sii9232->early_suspend);
	#endif //CONFIG_HAS_EARLYSUSPEND

	pr_debug("%s: Probe success!\n", __func__);
	return ret;

err_create_workqueue:
	gpio_free(sii9232->reset_pin);
	gpio_free(sii9232->intr_pin);
err_init:
err_platform_data_null:
	kfree(sii9232);
	return ret;
}

static int sii9232_remove(struct platform_device *pdev)
{
#ifdef HONEYCOMB_HDMI
	switch_dev_unregister(&sii9232_info_ptr->hpd_switch);
#endif
	gpio_free(sii9232_info_ptr->reset_pin);
	gpio_free(sii9232_info_ptr->intr_pin);
	destroy_workqueue(sii9232_info_ptr->wq);
	kfree(sii9232_info_ptr);
	sii9232_info_ptr = NULL;
	return 0;
}


static struct platform_driver this_driver = {
        .probe = sii9232_probe,
        .remove = sii9232_remove,
        .driver.name = "hdmi_msm",
};
#if 0
static int sii9232_add_dtv_device(struct device *dev,
				   struct class_interface *class_intf)
{
	/* might need locking if mulitple mdp devices */
	printk(KERN_INFO "%s\n", __func__);
	if (dtv_info)
		return 0;
	dtv_info = container_of(dev, struct mdp_dtv_info, dev);
	return platform_driver_register(&this_driver);
}

static void sii9232_remove_dtv_device(struct device *dev,
				       struct class_interface *class_intf)
{
	platform_driver_unregister(&this_driver);
	dtv_info = NULL;
}


static struct class_interface dtv_interface = {
	.add_dev = &sii9232_add_dtv_device,
	.remove_dev = &sii9232_remove_dtv_device,
};
#endif
static int __init sii9232_init(void)
{
	pr_err("%s\n", __func__);
	return platform_driver_register(&this_driver);
}

static void __exit sii9232_exit(void)
{
	platform_driver_unregister(&this_driver);
}
late_initcall(sii9232_init);
//module_init(sii9232_init);
//module_exit(sii9232_exit);
MODULE_DESCRIPTION("SiI9232 Driver");
MODULE_LICENSE("GPL");
