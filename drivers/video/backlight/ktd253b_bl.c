/*
* ktd253b_bl.c - driver for KTD253b led driver chip
*
* Copyright  2014, Samsung Electronics Co. Ltd. All Rights Reserved.
*
* Contact: Hyoyoung Kim <hyway.kim@samsung.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2 of the License.
*
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/ktd2801_bl.h>
#include <linux/ktd253b_bl.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#ifdef CONFIG_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/firmware.h>

#include <linux/bootmem.h>
#include <linux/console.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/ioport.h>
#include <linux/leds.h>
#include <linux/memory.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/msm_mdp.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/proc_fs.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/sync.h>
#include <linux/sw_sync.h>
#include <linux/file.h>
#include <linux/kthread.h>
#include <mach/board.h>
#include <mach/memory.h>
#include <linux/qcom_iommu.h>
#include <linux/msm_iommu_domains.h>
#include <mach/msm_memtypes.h>

static DEFINE_SPINLOCK(bl_ctrl_lock);
#define BACKLIGHT_DEBUG 1
#if BACKLIGHT_DEBUG
#define BLDBG(fmt, args...) printk(fmt, ## args)
#else
#define BLDBG(fmt, args...)
#endif

#define KTD253B_BACKLIGHT_DEV_NAME	"smd-bd"
#define KTD253B_SLEEP_OUT_DELAY 65

/* #define HBM_BRIGHTNESS 101 */
#define KTD253B_MAX_BRIGHTNESS 100
#define KTD253B_MIN_BRIGHTNESS 0
#define KTD253B_DEFAULT_BRIGHTNESS 60
#define KTD253B_DIMMING_VALUE 31
#define KTD253B_MAX_BRIGHTNESS_IN_BLU 32 /* backlight-IC MAX VALUE */
#define DEFAULT_BRIGHTNESS 60

struct notifier_block fb_notif_ktd253b;

static int GPIO_BL_CTRL;
static int BL_brightness;
static int is_poweron = 1;
static int wakeup_brightness = DEFAULT_BRIGHTNESS;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend_BL;
#endif

static int lcd_brightness_ktd253b = -1;
struct brt_value brt_table_ktd253b[] = {
	{ 100,	3 },/* MAX */
	{ 98,	4 },
	{ 96,	5 },
	{ 93,	6 },
	{ 90,	7 },
	{ 87,	8 },
	{ 84,	9 },
	{ 81,	10 },
	{ 78,	11 },
	{ 75,	12 },
	{ 72,	13 },
	{ 69,	14 },
	{ 66,	15 },
	{ 63,	16 },
	{ 60,	17 },/* default */
	{ 56,	18 },
	{ 52,	19 },
	{ 48,	20 },
	{ 44,	21 },
	{ 40,	22 },
	{ 36,	23 },
	{ 32,	24 },
	{ 28,	25 },
	{ 24,	26 },
	{ 20,	27 },
	{ 16,	28 },
	{ 12,	29 },
	{ 8,	30 },
	{ 4,	31 },
	{ 0,	31 },/* dim */
};
#define NB_BRT_LEVEL_KTD253B (int)(sizeof(brt_table_ktd253b)/sizeof(struct brt_value))

#ifdef CONFIG_EARLYSUSPEND
static void ktd253b_backlight_early_suspend(struct early_suspend *h)
{
	is_poweron = 0;
	ktd253b_backlight_set_brightness(0);
	return;
}

static void ktd253b_backlight_late_resume(struct early_suspend *h)
{
	msleep(10);
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(100);
	ktd253b_backlight_set_brightness(wakeup_brightness);
	is_poweron = 1;
}
#else
static int ktd253b_backlight_suspend(void)
{
	BLDBG("[BACKLIGHT] ktd253b_backlight_suspend\n");

	is_poweron = 0;

	ktd253b_backlight_set_brightness(0);

	return 0;
}

static int ktd253b_backlight_resume(void)
{
	BLDBG("[BACKLIGHT] ktd253b_backlight_resume\n");

	msleep(KTD253B_SLEEP_OUT_DELAY);
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(100);
	ktd253b_backlight_set_brightness(wakeup_brightness);

	is_poweron = 1;

	return 0;
}

void ktd253b_backlight_shutdown(void)
{
	pr_debug("%s: starts\n", __func__);

	gpio_direction_output(GPIO_BL_CTRL, 0);
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int power;

	BLDBG("[BACKLIGHT][%s]: Event[%lu]\n", __func__, event);

	/* If we aren't interested in this event, skip it immediately ... */
	switch (event) {
	case FB_EVENT_BLANK:
		power = *(int *)evdata->data;
		switch (power) {
		case FB_BLANK_UNBLANK:
			ktd253b_backlight_resume();
			break;
		default:
			return 0;
		}
		break;
	case FB_EARLY_EVENT_BLANK:
		power = *(int *)evdata->data;
		switch (power) {
		case FB_BLANK_POWERDOWN:
			ktd253b_backlight_suspend();
			break;
		default:
			return 0;
		}
		break;
	case FB_EVENT_SUSPEND:
		ktd253b_backlight_shutdown();
		break;
	default:
		return 0;
	}

	return 0;
}

static int ktd253b_register_fb(void)
{
	memset(&fb_notif_ktd253b, 0, sizeof(fb_notif_ktd253b));
	fb_notif_ktd253b.notifier_call = fb_notifier_callback;

	return fb_register_client(&fb_notif_ktd253b);
}

static void ktd253b_unregister_fb(void)
{
	fb_unregister_client(&fb_notif_ktd253b);
}
#endif

void ktd253b_backlight_set_brightness(int level)
{
	int tune_level = 0;
	unsigned long flags = 0;

	if (level >0) {
		int i;
		for (i = 0; i < NB_BRT_LEVEL_KTD253B; i++) {
			if (level <= brt_table_ktd253b[i].level
				&& level > brt_table_ktd253b[i+1].level) {
				tune_level = brt_table_ktd253b[i].tune_level;
				break;
			}
		}
	}
	if(level <= KTD253B_MIN_BRIGHTNESS){
		tune_level = KTD253B_DIMMING_VALUE;
	}

	/*	BACKLIGHT is KTD253B model */

	if (level == 0 && is_poweron == 0) {
		gpio_set_value(GPIO_BL_CTRL, 0);
		mdelay(3);
		lcd_brightness_ktd253b= 0;
	} else {
		int pulse;

		if (unlikely(lcd_brightness_ktd253b < 0)) {
			int val = gpio_get_value(GPIO_BL_CTRL);
			if (val) {
				gpio_set_value(GPIO_BL_CTRL, 0);
				mdelay(3);
				BLDBG( "[BACKLIGHT]LCD Baklight init in boot time on kernel\n");
			}
			lcd_brightness_ktd253b = 0;
		}

		if (!lcd_brightness_ktd253b) {
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(100);
			lcd_brightness_ktd253b = KTD253B_MAX_BRIGHTNESS_IN_BLU;
		}

		pulse = (tune_level - lcd_brightness_ktd253b + KTD253B_MAX_BRIGHTNESS_IN_BLU)
					% KTD253B_MAX_BRIGHTNESS_IN_BLU;
		BLDBG( "[BACKLIGHT] ktd253b set_brightness : level(%d) tune (%d)pulse(%d)\n", level, tune_level,pulse);

		spin_lock_irqsave(&bl_ctrl_lock, flags);
		for (; pulse > 0; pulse--) {
			gpio_set_value(GPIO_BL_CTRL, 0);
			udelay(4);
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(4);
		}
		spin_unlock_irqrestore(&bl_ctrl_lock, flags);

		lcd_brightness_ktd253b = tune_level;
	}
	mdelay(1);
	return;
}

static int ktd253b_backlight_update_status(struct backlight_device *bl)
{
	wakeup_brightness = bl->props.brightness;

	if (is_poweron) {
		BLDBG("[BACKLIGHT] called ktd253b_backlight_set_brightness %d\n", wakeup_brightness);
		ktd253b_backlight_set_brightness(wakeup_brightness);
	}

	return 0;
}

static int ktd253b_backlight_get_brightness(struct backlight_device *bl)
{
	BLDBG("[BACKLIGHT] ktd253b_backlight_get_brightness\n");

	BL_brightness = bl->props.brightness;

	return BL_brightness;
}

static const struct backlight_ops ktd253b_backlight_ops = {
	.update_status	= ktd253b_backlight_update_status,
	.get_brightness	= ktd253b_backlight_get_brightness,
};

static int ktd253b_backlight_probe(struct platform_device *pdev)
{
	struct backlight_device *bl = NULL;
	struct backlight_properties props = {0,};
	int result = 0;

	int rc = 0;
	struct device_node *node = pdev->dev.of_node;
	GPIO_BL_CTRL = of_get_named_gpio(node, "sec,led-ctrl", 0);
	if (GPIO_BL_CTRL < 0) {
		BLDBG("[ERROR] of_get_named_gpio fail\n");
	} else {
		rc = gpio_request(GPIO_BL_CTRL, "BL_CTRL");
		if (rc)
			BLDBG("[ERROR] gpio_request failed \n");
	}

	BLDBG("[BACKLIGHT] ktd253b_backlight_probe\n");

	props.max_brightness = KTD253B_MAX_BRIGHTNESS;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register(KTD253B_BACKLIGHT_DEV_NAME, &pdev->dev, NULL,
					&ktd253b_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		result = PTR_ERR(bl);

		goto error_exit;
	}

	bl->props.max_brightness = KTD253B_MAX_BRIGHTNESS;
	bl->props.brightness = KTD253B_DEFAULT_BRIGHTNESS;

	platform_set_drvdata(pdev, bl);

#ifndef CONFIG_EARLYSUSPEND
	ktd253b_register_fb();
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend_BL.suspend = ktd253b_backlight_early_suspend;
	early_suspend_BL.resume  = ktd253b_backlight_late_resume;
	early_suspend_BL.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&early_suspend_BL);
#endif
	ktd253b_backlight_update_status(bl);

	return result;

error_exit:
	if (bl)
		backlight_device_unregister(bl);

	return result;
}

static int ktd253b_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	ktd253b_unregister_fb();
	backlight_device_unregister(bl);
	gpio_direction_output(GPIO_BL_CTRL, 0);

	return 0;
}

static const struct of_device_id ktd253b_dt_match[] = {
	{ .compatible = "smd-bd-ktd253b",},
	{}
};

static struct platform_driver ktd253b_backlight_driver = {
	.driver		= {
		.name	= "ktd253b",
		.owner	= THIS_MODULE,
		.of_match_table = ktd253b_dt_match,
	},
	.probe		= ktd253b_backlight_probe,
	.remove		= ktd253b_backlight_remove,
};

static int __init ktd253b_backlight_init(void)
{
	return platform_driver_register(&ktd253b_backlight_driver);
}
module_init(ktd253b_backlight_init);

static void __exit ktd253b_backlight_exit(void)
{
	platform_driver_unregister(&ktd253b_backlight_driver);
}
module_exit(ktd253b_backlight_exit);

MODULE_DESCRIPTION("KTDktd253b based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ktd253b-backlight");
