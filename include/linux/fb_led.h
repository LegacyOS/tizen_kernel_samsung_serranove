/* fb_led.h
 *
 * Copyright (C) 2014 Samsung Electronics Co.Ltd
 * Authors:
 *	Eunchul Kim <chulspro.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef _FB_LED_H_
#define _FB_LED_H_

#include <linux/fb.h>

#ifdef __KERNEL__

/**
 * A structure for lcd panel information.
 *
 * @timing: default video mode for initializing
 * @width_mm: physical size of lcd width.
 * @height_mm: physical size of lcd height.
 * @self_refresh: self refresh rate of panel.
 */
struct fb_panel_info {
	struct fb_videomode timing;
	u32 width_mm;
	u32 height_mm;
	u32 self_refresh;
};

/**
 * Platform Specific Structure for FB based I2C.
 *
 * @panel_dev: default panel device info for initializing
 * @panel: default panel info for initializing
 * @lcd_pdata: default lcd platform data for initializing
 * @bpp: default bit per pixel
 */
struct fb_led_pdata {
	struct device *panel_dev;
	struct fb_panel_info *panel;
	struct lcd_platform_data *lcd_pdata;
	unsigned int bpp;
};

/**
 * A structure for lcd panel ops.
 *
 * @register_client: register i2c client and panel ctx.
 * @blank: panel on & off.
 * @pan_display: send framebuffer to panel.
 */
struct fb_panel_ops {
	void (*register_client)(struct i2c_client *client,
		void *panel_ctx);
	int (*blank)(struct i2c_client *client,
		void *panel_ctx, int blank);
	int (*pan_display)(struct i2c_client *client,
		void *panel_ctx, void *kvaddr);
};

void fb_led_panel_register(struct fb_panel_ops *ops);

extern struct platform_driver fb_led_panel_driver;
#endif /* _KERNEL_ */
#endif /* _FB_LED_H_ */

