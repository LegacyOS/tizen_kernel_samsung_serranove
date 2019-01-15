/* fb_led.c
 *
 * Copyright (C) 2015 Samsung Electronics Co.Ltd
 *
 * Eunchul Kim <chulspro.kim@samsung.com>
 * Taeheon Kim <th908.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/fb_led.h>

#define PANEL_NAME "fb_led"
#define MAX_FB 2

#define to_context(dev) platform_get_drvdata(to_platform_device(dev))

struct fb_led_ctx {
	struct fb_info *info;
	struct fb_panel_info *panel;
	struct fb_panel_ops *panel_ops;
	struct i2c_client *client;
	void *panel_ctx;
	unsigned long size;
};

static struct fb_panel_ops *led_ops;

void fb_led_panel_register(struct fb_panel_ops *ops)
{
	pr_debug("%s:ops[0x%x]\n", __func__, (int)ops);
	if (ops)
		led_ops = ops;
}

static int fb_led_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	switch (var->bits_per_pixel) {
	case 1:
		break;
	default:
		break;
	}

	return ret;
}

static int fb_led_set_par(struct fb_info *info)
{
	struct fb_led_ctx *ctx = info->par;
	struct fb_panel_info *panel = ctx->panel;
	struct fb_var_screeninfo *var = &info->var;

	pr_debug("%s:panel[%d,%d]var[%d,%d]\n", __func__,
		panel->timing.xres, panel->timing.yres,
		var->xres, var->yres);

	return 0;
}

static int fb_led_blank(int blank, struct fb_info *info)
{
	struct fb_led_ctx *ctx = info->par;
	struct fb_panel_ops *panel_ops = ctx->panel_ops;
	int ret = 0;

	pr_debug("%s:blank[%d]panel_ops[0x%x]\n", __func__,
		blank, (int)panel_ops);

	if (panel_ops && panel_ops->blank)
		ret = panel_ops->blank(ctx->client, ctx->panel_ctx, blank);

	return ret;
}

static int fb_led_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	struct fb_led_ctx *ctx = info->par;
	struct fb_panel_ops *panel_ops = ctx->panel_ops;
	int ret = 0;

	pr_debug("%s:yoffset[%d]\n", __func__, var->yoffset);

	if (!info->fix.smem_start) {
		pr_err("smem_start is null\n");
		return -EFAULT;
	}
	/* send fb buffer */
	if (panel_ops && panel_ops->pan_display)
		ret = panel_ops->pan_display(ctx->client, ctx->panel_ctx,
			(void *)info->screen_base + (var->yoffset * var->xres));

	return ret;
}

static int fb_led_ioctl(struct fb_info *info, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;

	pr_debug("%s:cmd[0x%x]\n", __func__, cmd);

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int fb_led_open(struct fb_info *info, int user)
{
	return 0;
}
static int fb_led_mmap(struct fb_info *info,
		struct vm_area_struct *vma)
{
	struct fb_led_ctx *ctx = info->par;
	struct i2c_client *client = ctx->client;

	pr_debug("%s\n", __func__);

	return dma_mmap_writecombine(&client->dev, vma,
		info->screen_base, info->fix.smem_start,
		info->fix.smem_len);
}

static struct fb_ops fb_led_ops = {
	.owner = THIS_MODULE,
	.fb_open = fb_led_open,
	.fb_check_var = fb_led_check_var,
	.fb_set_par = fb_led_set_par,
	.fb_blank = fb_led_blank,
	.fb_pan_display	= fb_led_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_ioctl = fb_led_ioctl,
	.fb_mmap = fb_led_mmap,
};

static int fb_led_configure_clocks(struct fb_led_ctx *ctx,
		struct device *dev)
{
	struct fb_panel_info *panel = ctx->panel;
	struct fb_videomode *timing = &panel->timing;
	int ret = 0;

	ctx->size = timing->xres * timing->yres * MAX_FB;

	pr_info("%s:pixclock[%d]size[%ld]\n", __func__,
		timing->pixclock, ctx->size);

	return ret;
}

static struct fb_led_pdata *of_get_pdata(struct device *dev)
{
	struct device_node *np;
	struct platform_device *dst_dev;

	np = of_parse_phandle(dev->of_node, "display", 0);
	if (!np) {
		dev_err(dev, "failed to get client node handle.\n");
		return NULL;
	}

	dst_dev = of_find_device_by_node(np);
	if (!dst_dev) {
		dev_err(dev, "failed to get src platform device.\n");
		return NULL;
	}

	return dst_dev->dev.platform_data;
}

static int fb_led_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct fb_panel_ops *panel_ops = led_ops;
	struct fb_led_pdata *pdata;
	struct fb_led_ctx *ctx;
	struct fb_info *info;
	dma_addr_t map_dma;
	int ret = -EINVAL;

	pr_info("%s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		pr_err("failed to check i2c\n");
		return -EIO;
	}

	pdata = devm_kzalloc(&client->dev,
				sizeof(struct fb_led_pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("%s: failed to allocate platform data\n", __func__);
		return -EINVAL;
	}

	pdata = of_get_pdata(&client->dev);
	if (!pdata) {
		pr_err("can't get client platform data\n");
		ret = -ENOENT;
		goto free_pdata;
	}
	/* register framebuffer */
	info = framebuffer_alloc(sizeof(struct fb_led_ctx),
		&client->dev);
	if (!info) {
		pr_err("failed to allocate framebuffer\n");
		ret = -ENOENT;
		goto free_pdata;
	}

	ctx = info->par;
	ctx->client = client;
	ctx->panel = pdata->panel;
	ctx->panel_ops = panel_ops;
	ctx->panel_ctx = to_context(pdata->panel_dev);

	/* register client */
	if (panel_ops && panel_ops->register_client)
		panel_ops->register_client(client, ctx->panel_ctx);

	ret = fb_led_configure_clocks(ctx, &client->dev);
	if (ret) {
		pr_err("failed to configure clocks\n");
		goto free_fb;
	}

	client->dev.coherent_dma_mask = 0xffffffffUL;
	info->screen_base = dma_alloc_writecombine(&client->dev, ctx->size,
		&map_dma, GFP_KERNEL);
	if (!info->screen_base) {
		pr_err("failed to allocate screen base\n");
		ret = -ENOMEM;
		goto free_fb;
	}

	pr_info("%s:map_dma[0x%x]screen_base[0x%x]\n", __func__,
		(unsigned int)map_dma, (unsigned int)info->screen_base);

	/* clear frame buffer */
	memset(info->screen_base, 0xff, ctx->size);
	info->fix.smem_start = map_dma;
	info->fix.smem_len = ctx->size;
	info->fix.ypanstep = 1;

	ctx->size = PAGE_ALIGN(ctx->size);

	/* setup the initial video mode from the window */
	fb_videomode_to_var(&info->var, &ctx->panel->timing);
	info->var.yres_virtual = info->var.yres * MAX_FB;

	/* setup ops */
	info->fbops = &fb_led_ops;

	ctx->info = info;

	ret = register_framebuffer(ctx->info);
	if (ret < 0) {
		pr_err("failed to register framebuffer\n");
		goto free_dma;
	}

	i2c_set_clientdata(ctx->client, ctx);

	pr_info("%s:register successfully\n", __func__);

	return 0;

free_pdata:
	devm_kfree(&client->dev, pdata);
free_dma:
	dma_free_writecombine(&client->dev, PAGE_ALIGN(info->fix.smem_len),
			info->screen_base, info->fix.smem_start);
free_fb:
	unregister_framebuffer(info);
	return ret;
}

static int fb_led_remove(struct i2c_client *client)
{
	struct fb_led_ctx *ctx = i2c_get_clientdata(client);
	struct fb_info *info = ctx->info;

	pr_debug("%s\n", __func__);

	if (info->screen_base)
		dma_free_writecombine(&client->dev,
				PAGE_ALIGN(info->fix.smem_len),
				info->screen_base, info->fix.smem_start);

	unregister_framebuffer(ctx->info);
	kfree(ctx);

	return 0;
}

static struct i2c_device_id fb_led_idtable[] = {
	{PANEL_NAME, 0},
	{ },
};

struct i2c_driver fb_led_driver = {
	.driver = {
		.name = PANEL_NAME,
		.owner = THIS_MODULE,
	},
	.id_table = fb_led_idtable,
	.probe = fb_led_probe,
	.remove = fb_led_remove,
};

static int __init fb_led_init(void)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

#ifdef CONFIG_FB_LED
	ret = platform_driver_register(&fb_led_panel_driver);
	if (ret < 0)
		goto out_i2c_panel;

	ret = i2c_add_driver(&fb_led_driver);
	if (ret < 0)
		goto out_i2c;
#endif

	return 0;

#ifdef CONFIG_FB_LED
out_i2c:
	platform_driver_unregister(&fb_led_panel_driver);
out_i2c_panel:
	return ret;
#endif
}

static void __exit fb_led_exit(void)
{
	pr_info("%s\n", __func__);

#ifdef CONFIG_FB_LED
	i2c_del_driver(&fb_led_driver);
	platform_driver_unregister(&fb_led_panel_driver);
#endif
}

module_init(fb_led_init);
module_exit(fb_led_exit);

MODULE_AUTHOR("Eunchul Kim <chulspro.kim@samsung.com>");
MODULE_DESCRIPTION("Samsung I2C Framebuffer driver");
MODULE_LICENSE("GPL");
