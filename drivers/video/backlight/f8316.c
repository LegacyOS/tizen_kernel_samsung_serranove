/*
 * Copyright (C) 2015 Samsung Electronics
 *
 * Eunchul Kim <chulspro.kim@samsung.com>
 * Taeheon Kim <th908.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/lcd.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/fb_led.h>
#include "f8316.h"

#define DOTLED_NAME "f8316"
struct panel_ctx {
	struct i2c_client *client;
	struct fb_panel_info *panel;
	struct lcd_platform_data *lcd_pdata;
	struct lcd_device *ld;
	struct regulator *vdd;
	int reset_gpio;
};
static u8 dot[21] = {
	0x1, 0xff, 0xff,
	0x1, 0xff, 0xff,
	0x0, 0x03, 0x80,
	0x0, 0x03, 0x80,
	0x0, 0x03, 0x80,
	0x0, 0x03, 0x80,
	0x0, 0x03, 0x80,
};
struct fb_panel_info panel;

static int panel_cmd_write(struct i2c_client *client, u8 reg, u8 data)
{
	int ret = 0;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0) {
		pr_err("%s:error[%d]\n", __func__, ret);
		return -EIO;
	}

	return ret;
}

static int panel_data_write(struct i2c_client *client, u8 reg,
		u8 length, const u8 *values)
{
	int i = 0;
	int ret = 0;

	while (i != length) {
		ret = i2c_smbus_write_byte_data(client, reg, values[i]);
		if (ret < 0) {
			pr_err("%s:error[%d][%d]\n", __func__, ret, i);
			break;
		}
		reg++;
		i++;
	}

	return ret;
}

static int panel_update(struct i2c_client *client,
					int mode, u8 *matrix)
{
	int ret = 0;
	int i, retry = 3;

	/*TODO*/
	for (i = 0; i < retry; i++) {
		ret = panel_data_write(client, REG_LED_BUF1,
					REG_BUF1_LEN, matrix);
		if (ret < 0) {
			pr_err("failed[%d]to update matrix retry[%d]\n",
					ret, i);
		} else {
			pr_info("Success update matrix\n");
			break;
		}
	}

	for (i = 0; i < retry; i++) {
		ret = panel_cmd_write(client, REG_LED_CTRL1,
			REG_LED_UPDATE | REG_LED_SLIDE | REG_LED_SLIDING2);
		if (ret < 0) {
			pr_err("failed[%d]to send CTRL regretry[%d]\n",
					ret, i);
		} else {
			pr_info("Success CTRL reg\n");
			break;
		}
	}

	return 0;
}

static int panel_power_on(struct panel_ctx *ctx, int power)
{
	int ret = 0;

	if (power) {
		ret = regulator_set_voltage(ctx->vdd, 3300000, 3300000);
		if (ret) {
			pr_err("%s: unable to set voltage for vdd_vreg, %d\n",
				__func__, ret);
			return ret;
		}

		ret = regulator_enable(ctx->vdd);
		if (ret) {
			pr_err("%s: unable to enable vdd, %d\n",
				__func__, ret);
			return ret;
		}
	} else {
		ret = regulator_disable(ctx->vdd);
		if (ret) {
			pr_err("%s: unable to enable vdd, %d\n",
				__func__, ret);
			return ret;
		}
	}

	msleep(20);

	return ret;
}

static void panel_reset(struct panel_ctx *ctx, bool on)
{
	gpio_direction_output(ctx->reset_gpio, on);
	msleep(50);
}

static int panel_get_power(struct lcd_device *ld)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int panel_set_power(struct lcd_device *ld, int power)
{
	pr_debug("%s:power[%d]\n", __func__, power);
	return 0;
}

static struct lcd_ops f8316_lcd_ops = {
	.get_power = panel_get_power,
	.set_power = panel_set_power,
};

static void panel_register_client(struct i2c_client *client,
		void *panel_ctx)
{
	struct panel_ctx *ctx = panel_ctx;

	pr_info("%s:client[0x%x]\n", __func__, (int)client);

	ctx->client = client;
}

static int panel_blank(struct i2c_client *client, void *panel_ctx, int blank)
{
	struct panel_ctx *ctx = panel_ctx;
	int ret = 0;

	pr_info("%s:blank[%d]\n", __func__, blank);

	switch (blank) {
	case FB_BLANK_UNBLANK:
	case FB_BLANK_NORMAL:
		/* regulator enable */
		panel_power_on(ctx, true);
		/* reset high */
		panel_reset(ctx, true);
		/* send sequence */

		break;
	case FB_BLANK_POWERDOWN:
		/* reset low */
		panel_reset(ctx, false);
		/* regulator disable */
		panel_power_on(ctx, false);

		break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int panel_pan_display(struct i2c_client *client,
		void *panel_ctx, void *kvaddr)
{
	struct panel_ctx *ctx = panel_ctx;
	struct fb_panel_info *panel = ctx->panel;
	u32 pitch = panel->timing.xres;

	pr_info("%s:xres[%d]yres[%d]pitch[%d]\n", __func__,
		panel->timing.xres, panel->timing.yres, pitch);

	/* Temp to drive led until Apps ready */
	panel_update(client, 1, dot);

	return 0;
}

static struct fb_panel_ops panel_ops = {
	.register_client = panel_register_client,
	.blank = panel_blank,
	.pan_display = panel_pan_display,
};

static int get_devtree_pdata(struct panel_ctx *ctx, struct device *dev)
{
	struct device_node *np;
	enum of_gpio_flags flags;
	int ret = 0;

	np = dev->of_node;
	if (!np) {
		ret = -ENODEV;
		pr_err("Failed to get devtree ret = [%d]\n", ret);
		return ret;
	}

	ctx->reset_gpio = of_get_named_gpio_flags(np,
		"gpio_reset", 0, &flags);
	if (ctx->reset_gpio > 0) {
		pr_info("gpio: reset = %d\n", ctx->reset_gpio);
	} else {
		pr_err("%s: failed to get reset gpio\n", __func__);
		ret = -EINVAL;
		return ret;
	}

	ctx->vdd = regulator_get(dev, "vdd");
	if (IS_ERR(ctx->vdd)) {
		pr_err("%s:Regulator get failed vdd ret=%d\n", __func__, ret);
		ret = -ENODEV;
		return ret;
	}

	return ret;
}


static int of_get_panel_data(struct fb_led_pdata *pdata, struct device *dev)
{
	struct device_node *np;
	struct device_node *timing_np;
	struct fb_panel_info *panel = pdata->panel;
	int ret = 0;

	np = dev->of_node;
	if (!np) {
		ret = -ENODEV;
		pr_err("Failed to get panel devtree ret = [%d]\n", ret);
		return ret;
	}
	ret = of_property_read_u32(np, "panel-width-mm", &panel->width_mm);
	if (ret)
		pr_err("Failed to read display size x\n");

	ret = of_property_read_u32(np, "panel-height-mm", &panel->height_mm);
	if (ret)
		pr_err("Failed to read display size y\n");

	timing_np = of_find_node_by_name(np, "display-timings");
	if (!timing_np) {
		pr_err("could not find display-timings node\n");
		return -ENODEV;
	} else {
		ret = of_property_read_u32(timing_np, "vactive",
				&panel->timing.xres);
		if (ret) {
			pr_err("Failed to read display max x\n");
			return -EINVAL;
		}

		ret = of_property_read_u32(timing_np, "hactive",
				&panel->timing.yres);
		if (ret) {
			pr_err("Failed to read display max x\n");
			return -EINVAL;
		}
	}

	pr_info("v[%d]h[%d]\n", panel->timing.xres, panel->timing.yres);

	return ret;
}


static int panel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_led_pdata *pdata;
	struct panel_ctx *ctx;
	int ret = -EINVAL;

	pdata = devm_kzalloc(dev, sizeof(struct fb_led_pdata),
				GFP_KERNEL);
	if (pdata == NULL) {
		pr_err("failed to allocate memory\n");
		return -ENOMEM;
	}
	pdata->panel = &panel;
	pdata->panel_dev = dev;
	pdev->dev.platform_data = pdata;
	ret = of_get_panel_data(pdata, dev);
	if (ret < 0)
		goto free_pdata;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		pr_err("no context memory\n");
		return -ENOMEM;
	}
	ctx->panel = pdata->panel;
	ctx->lcd_pdata = pdata->lcd_pdata;

	ret = get_devtree_pdata(ctx, dev);
	if (ret < 0)
		goto free_ctx;

	ctx->ld = lcd_device_register(DOTLED_NAME, dev, ctx,
					&f8316_lcd_ops);
	if (IS_ERR(ctx->ld)) {
		pr_err("failed to register lcd ops\n");
		ret = PTR_ERR(ctx->ld);
		goto free_ctx;
	}

	platform_set_drvdata(pdev, ctx);

	fb_led_panel_register(&panel_ops);

	/* Temp to drive led until Apps ready */
	panel_power_on(ctx, true);
	panel_reset(ctx, true);
	pr_info("%s: probed successfully\n", __func__);

	return 0;
free_ctx:
	kfree(ctx);
free_pdata:
	kfree(pdata);

	return ret;
}

static int panel_remove(struct platform_device *pdev)
{
	struct panel_ctx *ctx = platform_get_drvdata(pdev);

	pr_debug("%s\n", __func__);

	kfree(ctx);

	return 0;
}

struct of_device_id fb_led_of_match[] = {
	{ .compatible = "f8316-led",},
	{ },
};

struct platform_driver fb_led_panel_driver = {
	.probe		= panel_probe,
	.remove		= panel_remove,
	.driver		= {
		.name	= "f8316-led",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(fb_led_of_match),
	},
};

MODULE_AUTHOR("Tae-Heon Kim <th908.kim@samsung.com>");
MODULE_DESCRIPTION("Dot-Led display driver");
MODULE_LICENSE("GPL");
