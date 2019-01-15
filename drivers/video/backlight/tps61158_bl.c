/*
* tps61158.c - driver for TPS61158 led driver chip
*
* Copyright  2015, Samsung Electronics Co. Ltd. All Rights Reserved.
*
* Contact: Kai Lv <kai.lv@samsung.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2 of the License.
*
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#ifdef CONFIG_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/of.h>
#include <linux/of_gpio.h>

static DEFINE_SPINLOCK(bl_ctrl_lock);

struct brt_value {
	int level;		/* Platform setting values */
	int tune_level;		/* Chip Setting values */
};

static void tps61158_backlight_set_brightness(int level);

#define EASY_SCALE

#ifdef EASY_SCALE
#define DIMMING_MAX_LEVEL 100
#define MIN_BRIGHTNESS 0
#define MAX_BRIGHTNESS DIMMING_MAX_LEVEL
#define DIMMING_VALUE 10
#define DEFAULT_BRIGHTNESS 30
#define DATA_LEN 5
#define DATA_START 10
#define EOS 10
#define LOW_BIT_L 20
#define LOW_BIT_H 5
#define HIGH_BIT_L 5
#define HIGH_BIT_H 20
#define ES_DELAY 200
#define ES_DETECT 800
#define SLEEP_OUT_DELAY 50
#endif				/* EASY_SCALE */

#define BACKLIGHT_DEV_NAME	"ti-bl"

static int GPIO_BL_CTRL;
static int BL_brightness;
static int is_poweron = 1;
static int wakeup_brightness = DEFAULT_BRIGHTNESS;

#ifdef EASY_SCALE
int addr_bit_map[8] = { 0, 1, 0, 1, 1, 0, 0, 0 };
#endif				/* EASY_SCALE */

struct notifier_block fb_notif;

#ifdef CONFIG_HAS_EARLYSUSPEND
struct early_suspend early_suspend_BL;
#endif

#ifdef EASY_SCALE
struct brt_value brt_table_tps[] = {
	{0, 5},			/* Max */
	{4, 6},
	{8, 7},
	{12, 8},
	{16, 9},
	{20, 10},
	{24, 11},
	{28, 12},
	{32, 13},
	{36, 14},
	{40, 15},
	{44, 16},
	{48, 17},
	{52, 18},
	{56, 19},
	{60, 20},
	{64, 21},
	{68, 22},
	{72, 23},
	{76, 24},
	{80, 25},
	{84, 26},
	{87, 27},
	{90, 28},
	{93, 29},
	{96, 30},
	{99, 31},
	{100, 31}
};
#endif

#define NB_BRT_LEVEL (int)(sizeof(brt_table_tps)/sizeof(struct brt_value))

#ifdef CONFIG_EARLYSUSPEND
static void tps61158_backlight_early_suspend(struct early_suspend *h)
{
	is_poweron = 0;
	tps61158_backlight_set_brightness(0);
#ifdef EASY_SCALE
	gpio_set_value(GPIO_BL_CTRL, 0);
#endif
	return;
}

static void tps61158_backlight_late_resume(struct early_suspend *h)
{
	unsigned long flags;

#ifdef EASY_SCALE
	spin_lock_irqsave(&bl_ctrl_lock, flags);
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(ES_DELAY);
	gpio_set_value(GPIO_BL_CTRL, 0);
	udelay(ES_DETECT);
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(2000);
	udelay(2000);
	spin_unlock_irqrestore(&bl_ctrl_lock, flags);
#endif

	tps61158_backlight_set_brightness(wakeup_brightness);
	is_poweron = 1;
}
#else
static int tps61158_backlight_suspend(void)
{
	pr_debug("[BACKLIGHT] tps61158_backlight_suspend\n");

	is_poweron = 0;

#ifdef EASY_SCALE
	gpio_set_value(GPIO_BL_CTRL, 0);
#endif

	return 0;
}

static int tps61158_backlight_resume(void)
{
	unsigned long flags;

	pr_debug("[BACKLIGHT] tps61158_backlight_resume\n");

	msleep(SLEEP_OUT_DELAY);

#ifdef EASY_SCALE
	spin_lock_irqsave(&bl_ctrl_lock, flags);
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(ES_DELAY);
	gpio_set_value(GPIO_BL_CTRL, 0);
	udelay(ES_DETECT);
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(2000);
	udelay(2000);
	spin_unlock_irqrestore(&bl_ctrl_lock, flags);
#endif

	tps61158_backlight_set_brightness(wakeup_brightness);

	is_poweron = 1;

	return 0;
}

void tps61158_backlight_shutdown(void)
{
	pr_debug("%s: starts\n", __func__);

	gpio_direction_output(GPIO_BL_CTRL, 0);
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int power;

	pr_debug("[%s]: Event[%lu]\n", __func__, event);

	/* If we aren't interested in this event, skip it immediately ... */
	switch (event) {
	case FB_EVENT_BLANK:
		power = *(int *)evdata->data;
		switch (power) {
		case FB_BLANK_UNBLANK:
			tps61158_backlight_resume();
			break;
		default:
			return 0;
		}
		break;
	case FB_EARLY_EVENT_BLANK:
		power = *(int *)evdata->data;
		switch (power) {
		case FB_BLANK_POWERDOWN:
			tps61158_backlight_suspend();
			break;
		default:
			return 0;
		}
		break;
	case FB_EVENT_SUSPEND:
		tps61158_backlight_shutdown();
		break;
	default:
		return 0;
	}

	return 0;
}

static int tps61158_register_fb(void)
{
	memset(&fb_notif, 0, sizeof(fb_notif));
	fb_notif.notifier_call = fb_notifier_callback;

	return fb_register_client(&fb_notif);
}

static void tps61158_unregister_fb(void)
{
	fb_unregister_client(&fb_notif);
}
#endif

#ifdef EASY_SCALE
void tps61158_backlight_set_brightness(int level)
{
	int i = 0;
	unsigned char brightness = 5;
	int bit_map[7] = { 0, };
	unsigned long flags = 0;

	if (level < 0) {
		pr_debug("%s: level[%d], Suspend!\n", __func__, level);
		gpio_set_value(GPIO_BL_CTRL, 0);
		return;
	}

	if (level < MIN_BRIGHTNESS) {
		brightness = DIMMING_VALUE;
	} else {
		for (i = 0; i < NB_BRT_LEVEL; ++i) {
			if (level > brt_table_tps[i].level
			    && level <= brt_table_tps[i + 1].level) {
				brightness = brt_table_tps[i].tune_level;
				break;
			}
		}
	}

	pr_debug("[LCD] %s: level[%d], brightness[%d]\n", __func__, level,
	      brightness);

	for (i = 0; i < 5; i++) {
		bit_map[i] = brightness & 0x01;
		brightness >>= 1;
	}
	bit_map[5] = 0;
	bit_map[6] = 0;
	spin_lock_irqsave(&bl_ctrl_lock, flags);

	/* start */
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(DATA_START);

	/* device address */
	for (i = 0; i < 8; i++) {
		if (addr_bit_map[i]) {
			gpio_set_value(GPIO_BL_CTRL, 0);
			udelay(HIGH_BIT_L);
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(HIGH_BIT_H);
		} else {
			gpio_set_value(GPIO_BL_CTRL, 0);
			udelay(LOW_BIT_L);
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(LOW_BIT_H);
		}
	}

	/* EOS */
	gpio_set_value(GPIO_BL_CTRL, 0);
	udelay(EOS);

	/* start */
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(DATA_START);

	/* RFA */
	gpio_set_value(GPIO_BL_CTRL, 0);
	udelay(LOW_BIT_L);
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(LOW_BIT_H);

	/* data */
	for (i = 6; i >= 0; i--) {
		if (bit_map[i]) {
			gpio_set_value(GPIO_BL_CTRL, 0);
			udelay(HIGH_BIT_L);
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(HIGH_BIT_H);
		} else {
			gpio_set_value(GPIO_BL_CTRL, 0);
			udelay(LOW_BIT_L);
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(LOW_BIT_H);
		}
	}

	/* EOS */
	gpio_set_value(GPIO_BL_CTRL, 0);
	udelay(EOS);
	gpio_set_value(GPIO_BL_CTRL, 1);

	spin_unlock_irqrestore(&bl_ctrl_lock, flags);
	return;
}
#endif

static int tps61158_backlight_update_status(struct backlight_device *bl)
{
	wakeup_brightness = bl->props.brightness;

	if (is_poweron) {
		pr_debug("[LCD] called tps61158_backlight_update_status %d\n",
		      wakeup_brightness);
		tps61158_backlight_set_brightness(wakeup_brightness);
	}

	return 0;
}

static int tps61158_backlight_get_brightness(struct backlight_device *bl)
{
	pr_debug("[BACKLIGHT] tps61158_backlight_get_brightness\n");

	BL_brightness = bl->props.brightness;

	return BL_brightness;
}

static const struct backlight_ops tps61158_backlight_ops = {
	.update_status = tps61158_backlight_update_status,
	.get_brightness = tps61158_backlight_get_brightness,
};

static int tps61158_backlight_probe(struct platform_device *pdev)
{
	struct backlight_device *bl = NULL;
	struct backlight_properties props = { 0, };
	unsigned long flags = 0;
	int result = 0;

	int rc = 0;
	struct device_node *node = NULL;

	node = pdev->dev.of_node;
	GPIO_BL_CTRL = of_get_named_gpio(node, "sec,led-ctrl", 0);
	if (GPIO_BL_CTRL < 0) {
		pr_debug("[ERROR] of_get_named_gpio fail\n");
	} else {
		rc = gpio_request(GPIO_BL_CTRL, "BL_CTRL");
		if (rc)
			pr_debug("[ERROR] gpio_request failed \n");
	}

	pr_debug("[BACKLIGHT] tps61158_backlight_probe\n");

	props.max_brightness = MAX_BRIGHTNESS;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register(BACKLIGHT_DEV_NAME, &pdev->dev, NULL,
				       &tps61158_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		result = PTR_ERR(bl);

		goto error_exit;
	}

	bl->props.max_brightness = MAX_BRIGHTNESS;
	bl->props.brightness = DEFAULT_BRIGHTNESS;

	platform_set_drvdata(pdev, bl);

#ifndef CONFIG_EARLYSUSPEND
	tps61158_register_fb();
#endif

#ifdef EASY_SCALE
	spin_lock_irqsave(&bl_ctrl_lock, flags);
	gpio_set_value(GPIO_BL_CTRL, 0);
	udelay(2000);
	udelay(2000);
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(ES_DELAY);
	gpio_set_value(GPIO_BL_CTRL, 0);
	udelay(ES_DETECT);
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(2000);
	udelay(2000);
	spin_unlock_irqrestore(&bl_ctrl_lock, flags);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend_BL.suspend = tps61158_backlight_early_suspend;
	early_suspend_BL.resume = tps61158_backlight_late_resume;
	early_suspend_BL.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&early_suspend_BL);
#endif
	tps61158_backlight_update_status(bl);

	return result;

 error_exit:
	if (bl)
		backlight_device_unregister(bl);

	return result;
}

static int tps61158_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	tps61158_unregister_fb();
	backlight_device_unregister(bl);
	gpio_direction_output(GPIO_BL_CTRL, 0);

	return 0;
}

static const struct of_device_id tps61158_dt_match[] = {
	{.compatible = "ti-bl-tps61158",},
	{}
};

static struct platform_driver tps61158_backlight_driver = {
	.driver = {
		   .name = "tps61158",
		   .owner = THIS_MODULE,
		   .of_match_table = tps61158_dt_match,
		   },
	.probe = tps61158_backlight_probe,
	.remove = tps61158_backlight_remove,
};

static int __init tps61158_backlight_init(void)
{
	return platform_driver_register(&tps61158_backlight_driver);
}

module_init(tps61158_backlight_init);

static void __exit tps61158_backlight_exit(void)
{
	platform_driver_unregister(&tps61158_backlight_driver);
}

module_exit(tps61158_backlight_exit);

MODULE_DESCRIPTION("TI tps61158 based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tps61158-backlight");
