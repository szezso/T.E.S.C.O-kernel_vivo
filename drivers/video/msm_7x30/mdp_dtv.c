/* drivers/video/msm/mdp_dtv.c
 *
 * Copyright (c) 2009 Google Inc.
 * Copyright (c) 2009 QUALCOMM Incorporated
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
 * Author: Dima Zavin <dima@android.com>
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <linux/msm_mdp.h>
#include <mach/msm_fb-7x30.h>
#include <mach/debug_display.h>

#include "mdp_hw.h"
#include "mdp4.h"

#if 0
#define D(fmt, args...) PR_DISP_INFO("Dispaly: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

struct class *dtv_class;
struct mdp_dtv_info *_dtv_info;
static struct mdp_device *mdp_dev;
static struct mdp4_overlay_pipe *dtv_pipe;

static void mdp_dtv_relay(struct mdp_dtv_info *dtv, int on_off)
{
	if (on_off) {
		if (dtv->active)
			return;
		/* clear fb data */
		memset(dtv->fb_base, 0x00, dtv->fb_size);
		clk_enable(dtv->mdp_clk);
		clk_enable(dtv->tv_enc_clk);
		clk_enable(dtv->tv_dac_clk);
		clk_enable(dtv->hdmi_clk);
		if (dtv->mdp_tv_clk)
			clk_enable(dtv->mdp_tv_clk);
		mdp_writel(dtv->mdp, 1, MDP_DTV_EN);
		dtv->active = true;
	} else {
		if (!dtv->active)
			return;
		mdp_writel(dtv->mdp, 0, MDP_DTV_EN);
		if (dtv->mdp_tv_clk)
			clk_disable(dtv->mdp_tv_clk);
		clk_disable(dtv->hdmi_clk);
		clk_disable(dtv->tv_dac_clk);
		clk_disable(dtv->tv_enc_clk);
		clk_disable(dtv->mdp_clk);
		dtv->active = false;
	}
}

static void mdp_dtv_overlay_start(void *priv, uint32_t addr, uint32_t stride,
			   uint32_t width, uint32_t height, uint32_t x,
			   uint32_t y)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);

	struct mdp4_overlay_pipe *pipe;
	pipe = dtv_pipe;
	pipe->srcp0_addr = addr;

	if (mdp->dma_config_dirty)
	{
		if(mdp->dma_format == DMA_IBUF_FORMAT_RGB565) {
			pipe->src_format = MDP_RGB_565;
			pipe->srcp0_ystride = pipe->src_width * 2;
		} else if(mdp->dma_format == DMA_IBUF_FORMAT_XRGB8888) {
			pipe->src_format = MDP_RGBA_8888;
			pipe->srcp0_ystride = pipe->src_width * 4;
		}
		mdp4_overlay_format2pipe(pipe);
		mdp_writel(pipe->mdp, 0, MDP_DTV_EN);
		mdelay(30);
		mdp4_overlay_dmap_xy(pipe);
		mdp4_overlay_dmap_cfg(pipe, 1);
		mdp4_overlayproc_cfg(pipe);
		mdp4_overlay_rgb_setup(pipe);
		mdp4_overlay_reg_flush(pipe, 1); /* rgb1 and mixer0 */
		mdp_writel(pipe->mdp, 1, MDP_DTV_EN);
		mdp->dma_config_dirty = false;
	} else {
		mdp4_overlay_rgb_setup(pipe);
		mdp4_overlay_reg_flush(pipe, 1); /* rgb1 and mixer0 */
	}

}

static void mdp_dtv_reconfig_timing(struct mdp_dtv_info *dtv,
		struct msm_lcdc_timing *timing, struct msm_fb_data *fb_data)
{
	unsigned int hsync_period;
	unsigned int hsync_start_x;
	unsigned int hsync_end_x;
	unsigned int vsync_period;
	unsigned int display_vstart;
	unsigned int display_vend;
	unsigned int dtv_on;

	hsync_period = (timing->hsync_pulse_width + timing->hsync_back_porch +
			fb_data->xres + timing->hsync_front_porch);	
	hsync_start_x = (timing->hsync_pulse_width + timing->hsync_back_porch);
	hsync_end_x = hsync_start_x + fb_data->xres - 1;

	vsync_period = (timing->vsync_pulse_width + timing->vsync_back_porch +
			fb_data->yres + timing->vsync_front_porch);
	vsync_period *= hsync_period;

	display_vstart = timing->vsync_pulse_width + timing->vsync_back_porch;
	display_vstart *= hsync_period;
	display_vstart += timing->hsync_skew;

	display_vend = (timing->vsync_pulse_width + timing->vsync_back_porch +
			 fb_data->yres) * hsync_period;
	display_vend += timing->hsync_skew - 1;

	/* register values we pre-compute at init time from the timing
	 * information in the panel info */
	dtv->parms.hsync_ctl = (((hsync_period & 0xfff) << 16) |
				 (timing->hsync_pulse_width & 0xfff));
	dtv->parms.vsync_period = vsync_period & 0xffffff;
	dtv->parms.vsync_pulse_width = (timing->vsync_pulse_width *
					 hsync_period) & 0xffffff;

	dtv->parms.display_hctl = (((hsync_end_x & 0xfff) << 16) |
				    (hsync_start_x & 0xfff));
	dtv->parms.display_vstart = display_vstart & 0xffffff;
	dtv->parms.display_vend = display_vend & 0xffffff;
	dtv->parms.hsync_skew = timing->hsync_skew & 0xfff;
	dtv->parms.polarity = ((timing->hsync_act_low << 0) |
				(timing->vsync_act_low << 1) |
				(timing->den_act_low << 2));
	dtv->parms.clk_rate = timing->clk_rate;

	clk_enable(dtv->mdp_clk);
	clk_enable(dtv->tv_enc_clk);
	clk_enable(dtv->tv_dac_clk);
	clk_enable(dtv->hdmi_clk);
	if (dtv->mdp_tv_clk)
		clk_enable(dtv->mdp_tv_clk);

	/* get DTV I/F state */
	dtv_on = mdp_readl(dtv->mdp, MDP_DTV_EN);

	/* enforece to disable it */
	mdp_writel(dtv->mdp, 0, MDP_DTV_EN);
	mdelay(30);
	clk_set_rate(dtv->tv_src_clk, dtv->parms.clk_rate);

	/* write the dtv params */
	mdp_writel(dtv->mdp, dtv->parms.hsync_ctl, MDP_DTV_HSYNC_CTL);
	mdp_writel(dtv->mdp, dtv->parms.vsync_period, MDP_DTV_VSYNC_PERIOD);
	mdp_writel(dtv->mdp, dtv->parms.vsync_pulse_width,
		   MDP_DTV_VSYNC_PULSE_WIDTH);
	mdp_writel(dtv->mdp, dtv->parms.display_hctl, MDP_DTV_DISPLAY_HCTL);
	mdp_writel(dtv->mdp, dtv->parms.display_vstart,
		   MDP_DTV_DISPLAY_V_START);
	mdp_writel(dtv->mdp, dtv->parms.display_vend, MDP_DTV_DISPLAY_V_END);
	mdp_writel(dtv->mdp, dtv->parms.hsync_skew, MDP_DTV_HSYNC_SKEW);

	mdp_writel(dtv->mdp, 0, MDP_DTV_BORDER_CLR);
	mdp_writel(dtv->mdp, 0xf0000 | 0x80000000 , MDP_DTV_UNDERFLOW_CTL);
	mdp_writel(dtv->mdp, 0, MDP_DTV_ACTIVE_HCTL);
	mdp_writel(dtv->mdp, 0, MDP_DTV_ACTIVE_V_START);
	mdp_writel(dtv->mdp, 0, MDP_DTV_ACTIVE_V_END);
	mdp_writel(dtv->mdp, dtv->parms.polarity, MDP_DTV_CTL_POLARITY);

	if (dtv->mdp_tv_clk)
		clk_disable(dtv->mdp_tv_clk);
	clk_disable(dtv->hdmi_clk);
	clk_disable(dtv->tv_dac_clk);
	clk_disable(dtv->tv_enc_clk);
	clk_disable(dtv->mdp_clk);

	/* restore the DTV_EN if try to reconfig within DTV ON state */
	if (dtv_on)
		mdp_writel(dtv->mdp, 1, MDP_DTV_EN);
}

static int mdp_dtv_probe(struct platform_device *pdev)
{
	struct msm_lcdc_platform_data *pdata = pdev->dev.platform_data;
	struct mdp_dtv_info *dtv;
	int ret = 0;
	struct mdp4_overlay_pipe *pipe;
	int ptype;
	
	if (!pdata) {
		PR_DISP_ERR("%s: no DTV platform data found\n", __func__);
		return -EINVAL;
	}

	dtv = kzalloc(sizeof(struct mdp_dtv_info), GFP_KERNEL);
	if (!dtv)
		return -ENOMEM;

	/* We don't actually own the clocks, the mdp does. */
	dtv->mdp_clk = clk_get(mdp_dev->dev.parent, "mdp_clk");
	if (IS_ERR(dtv->mdp_clk)) {
		PR_DISP_ERR("%s: failed to get mdp_clk\n", __func__);
		ret = PTR_ERR(dtv->mdp_clk);
		goto err_get_mdp_clk;
	}

	dtv->tv_enc_clk = clk_get(mdp_dev->dev.parent, "tv_enc_clk");
	if (IS_ERR(dtv->tv_enc_clk)) {
		PR_DISP_ERR("%s: failed to get tv_enc_clk\n", __func__);
		ret = PTR_ERR(dtv->tv_enc_clk);
		goto err_get_tv_enc_clk;
	}

	dtv->tv_dac_clk = clk_get(mdp_dev->dev.parent, "tv_dac_clk");
	if (IS_ERR(dtv->tv_dac_clk)) {
		PR_DISP_ERR("%s: failed to get tv_dac_clk\n", __func__);
		ret = PTR_ERR(dtv->tv_dac_clk);
		goto err_get_tv_dac_clk;
	}

	dtv->hdmi_clk = clk_get(mdp_dev->dev.parent, "hdmi_clk");
	if (IS_ERR(dtv->hdmi_clk)) {
		PR_DISP_ERR("%s: failed to get hdmi_clk\n", __func__);
		ret = PTR_ERR(dtv->hdmi_clk);
		goto err_get_hdmi_clk;
	}
#ifdef CONFIG_ARCH_MSM8X60
	dtv->hdmi_clk = clk_get(mdp_dev->dev.parent, "mdp_tv_clk");
	if (IS_ERR(dtv->mdp_tv_clk)) {
		PR_DISP_ERR("%s: failed to get mdp_tv_clk\n", __func__);
		ret = PTR_ERR(dtv->mdp_tv_clk);
		goto err_get_mdp_tv_clk;
	}
#endif

	dtv->tv_src_clk = clk_get(mdp_dev->dev.parent, "tv_src_clk");
	if (IS_ERR(dtv->tv_src_clk)) {
		dtv->tv_src_clk = dtv->tv_enc_clk; /* Fallback to slave */
		PR_DISP_ERR("%s: tv_src_clk not available, using tv_enc_clk"
			" instead\n", __func__);
	}

	dtv->pdata = pdata;

	platform_set_drvdata(pdev, dtv);
	mdp_out_if_register(mdp_dev, MSM_DTV_INTERFACE, dtv, INTR_OVERLAY1_DONE,
			    mdp_dtv_overlay_start);
	dtv->mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	
	mdp_dtv_reconfig_timing(dtv, pdata->timing, pdata->fb_data);
	
	dtv->fb_start = pdata->fb_resource->start;
	
	dtv->fb_size = pdata->fb_resource->end - pdata->fb_resource->start + 1;

        dtv->fb_base = ioremap(dtv->fb_start, dtv->fb_size);

	if(dtv->mdp->mdp_dev.color_format)
		dtv->color_format = dtv->mdp->mdp_dev.color_format;
	else
		dtv->color_format = MSM_MDP_OUT_IF_FMT_RGB565;
	
	mdp_dtv_relay(dtv, 1);

	if (dtv_pipe == NULL) {
		ptype = mdp4_overlay_format2type(MDP_RGB_565);
		pipe = mdp4_overlay_pipe_alloc(ptype, false);
		if (!pipe)
			goto err_mdp4_overlay_pipe_alloc;
		pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;
		pipe->pipe_used = 1;
		pipe->mixer_num  = MDP4_MIXER1;
		pipe->src_format = MDP_RGB_565;
		mdp4_overlay_format2pipe(pipe);
		pipe->mdp = dtv->mdp;

		dtv_pipe = pipe; /* keep it */
	} else {
		pipe = dtv_pipe;
	}

	pipe->src_height = pdata->fb_data->yres;
	pipe->src_width = pdata->fb_data->xres;
	pipe->src_h = pdata->fb_data->yres;
	pipe->src_w = pdata->fb_data->xres;
	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->srcp0_addr = (uint32_t) dtv->fb_start;

	pipe->srcp0_ystride = pdata->fb_data->xres * 2;

 	mdp4_overlay_dmae_xy(pipe);     /* dma_e */
	mdp4_overlay_dmae_cfg(pipe, 0);

	mdp4_overlay_rgb_setup(pipe);
	mdp4_mixer_stage_up(pipe);
	mdp4_overlayproc_cfg(pipe);

	dtv->fb_panel_data.fb_data = pdata->fb_data;
	dtv->fb_panel_data.interface_type = MSM_DTV_INTERFACE;
	
	dtv->fb_pdev.name = "msm_panel";
	dtv->fb_pdev.id = pdata->fb_id;
	dtv->fb_pdev.resource = pdata->fb_resource;
	dtv->fb_pdev.num_resources = 1;
	dtv->fb_pdev.dev.platform_data = &dtv->fb_panel_data;
	dtv->dtv_relay = mdp_dtv_relay;
	dtv->dtv_reconfig_timing = mdp_dtv_reconfig_timing;
	mdp4_overlay_reg_flush(pipe, 1); /* rgb1 and mixer0 */
	
	mdp_dtv_relay(dtv, 0);

	ret = platform_device_register(&dtv->fb_pdev);
	if (ret) {
		PR_DISP_ERR("%s: Cannot register msm_panel pdev\n", __func__);
		goto err_plat_dev_reg;
	}

	_dtv_info = dtv;
	
	PR_DISP_INFO("%s: initialized\n", __func__);

	return 0;

err_plat_dev_reg:
err_mdp4_overlay_pipe_alloc:
	platform_set_drvdata(pdev, NULL);
#ifdef CONFIG_ARCH_MSM8X60
err_get_mdp_tv_clk:	
	clk_put(dtv->hdmi_clk);		
#endif
err_get_hdmi_clk:	
	clk_put(dtv->tv_dac_clk);
err_get_tv_dac_clk:
	clk_put(dtv->tv_enc_clk);
err_get_tv_enc_clk:
	clk_put(dtv->mdp_clk);
err_get_mdp_clk:
	kfree(dtv);
	return ret;
}

static int mdp_dtv_remove(struct platform_device *pdev)
{
	struct mdp_dtv_info *dtv = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	clk_put(dtv->hdmi_clk);
	clk_put(dtv->tv_dac_clk);
	clk_put(dtv->tv_enc_clk);
	clk_put(dtv->mdp_clk);	
	kfree(dtv);

	return 0;
}

static int mdp_dtv_suspend(struct platform_device *pdev,pm_message_t state)
{
	struct mdp_dtv_info *dtv = platform_get_drvdata(pdev);

	PR_DISP_INFO("%s: suspending\n", __func__);
	if (!dtv->active)
		return 0;

	mdp_writel(dtv->mdp, 0, MDP_DTV_EN);
	clk_disable(dtv->tv_enc_clk);
	clk_disable(dtv->tv_dac_clk);
	clk_disable(dtv->hdmi_clk);

	if (dtv->mdp_tv_clk)
		clk_disable(dtv->mdp_tv_clk);
	clk_disable(dtv->mdp_clk);

	return 0;
}

static int mdp_dtv_resume(struct platform_device *pdev)
{
	struct mdp_dtv_info *dtv = platform_get_drvdata(pdev);

	PR_DISP_INFO("%s: resuming\n", __func__);
	if (!dtv->active)
		return 0;
	
	clk_enable(dtv->mdp_clk);
  	clk_enable(dtv->tv_enc_clk);
	clk_enable(dtv->tv_dac_clk);
	clk_enable(dtv->hdmi_clk);
	if (dtv->mdp_tv_clk)
		clk_enable(dtv->mdp_tv_clk);

	mdp_writel(dtv->mdp, 1, MDP_LCDC_EN);
	return 0;
}
static struct platform_driver mdp_dtv_driver = {
	.probe = mdp_dtv_probe,
	.remove = mdp_dtv_remove,
	.suspend = mdp_dtv_suspend,
	.resume = mdp_dtv_resume,
	.driver = {
		.name	= "msm_mdp_dtv",
		.owner	= THIS_MODULE,
	},
};

static int mdp_dtv_add_mdp_device(struct device *dev,
				   struct class_interface *class_intf)
{
	/* might need locking if mulitple mdp devices */
	PR_DISP_INFO("%s\n", __func__);
	if (mdp_dev)
		return 0;
	mdp_dev = container_of(dev, struct mdp_device, dev);
	return platform_driver_register(&mdp_dtv_driver);
}

static void mdp_dtv_remove_mdp_device(struct device *dev,
				       struct class_interface *class_intf)
{
	/* might need locking if mulitple mdp devices */
	if (dev != &mdp_dev->dev)
		return;
	platform_driver_unregister(&mdp_dtv_driver);
	mdp_dev = NULL;
}

static struct class_interface mdp_dtv_interface = {
	.add_dev = &mdp_dtv_add_mdp_device,
	.remove_dev = &mdp_dtv_remove_mdp_device,
};

int register_dtv_client(struct class_interface *cint)
{
	if (!dtv_class) {
		PR_DISP_ERR("mdp_dtv: no dtv_class when registering dtv client\n");
		return -ENODEV;
	}
	cint->class = dtv_class;
	return class_interface_register(cint);
}

static int __init mdp_dtv_init(void)
{
	dtv_class = class_create(THIS_MODULE, "msm_mdp_dtv");
	if (IS_ERR(dtv_class)) {
		PR_DISP_ERR("Error creating dtv_class\n");
		return PTR_ERR(dtv_class);
	}
	return register_mdp_client(&mdp_dtv_interface);
}

module_init(mdp_dtv_init);
