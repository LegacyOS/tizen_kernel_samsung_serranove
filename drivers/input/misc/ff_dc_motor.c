/* Copyright (c) 2010-2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

#include "../../staging/android/timed_output.h"
#include <linux/mfd/sm5703_vibrator.h>

#define MOTOR_PIN_CHK 0

/**
 * struct ff_dc_vib - structure to hold vibrator data
 * @vib_input_dev: input device supporting force feedback
 * @work: work structure to set the vibration parameters
 * @dev: device supporting force feedback
 * @level: level of vibration to set in the chip
 * @motor_en: enable pin controlled GPIO of the main chip
 */
struct ff_dc_vib {
	struct input_dev *vib_input_dev;
	struct work_struct work;
	struct device *dev;
	int level;
	int motor_en;
};

/**
 * ff_dc_vib_set - handler to start/stop vibration
 * @vib: pointer to vibrator structure
 * @on: state to set
 */
static int ff_dc_vib_set(struct ff_dc_vib *vib, int on)
{
	pr_info("[VIB] %s, value[%d]\n", __func__, on);
#if MOTOR_PIN_CHK
	gpio_direction_output(vib->motor_en, on);
#endif
	sm5703_set_vibrator(sm_vib->vib_power, on);

	return 0;
}

/**
 * ff_dc_work - worker to set vibration level
 * @work: pointer to work_struct
 */
static void ff_dc_work(struct work_struct *work)
{
	struct ff_dc_vib *vib = container_of(work, struct ff_dc_vib, work);

	pr_info("[VIB] %s\n", __func__);

	if (vib->level)
		ff_dc_vib_set(vib, 1);
	else
		ff_dc_vib_set(vib, 0);
}

/**
 * ff_dc_vib_close - callback of input close callback
 * @dev: input device pointer
 *
 * Turns off the vibrator.
 */
static void ff_dc_vib_close(struct input_dev *dev)
{
	struct ff_dc_vib *vib = input_get_drvdata(dev);

	cancel_work_sync(&vib->work);
	if (vib->level)
		ff_dc_vib_set(vib, 0);
}

/**
 * ff_dc_vib_play_effect - function to handle vib effects.
 * @dev: input device pointer
 * @data: data of effect
 * @effect: effect to play
 *
 * Currently this driver supports only rumble effects.
 */
static int ff_dc_vib_play_effect(struct input_dev *dev, void *data,
				  struct ff_effect *effect)
{

	struct ff_dc_vib *vib = input_get_drvdata(dev);
	pr_info("[VIB] %s\n", __func__);
	vib->level = effect->u.rumble.strong_magnitude >> 8;
	if (!vib->level)
		vib->level = effect->u.rumble.weak_magnitude >> 9;

	schedule_work(&vib->work);

	return 0;
}

static int ff_dc_vib_probe(struct platform_device *pdev)
{
	struct ff_dc_vib *vib;
	struct input_dev *input_dev;
	int error;

	vib = kzalloc(sizeof(*vib), GFP_KERNEL);
#if MOTOR_PIN_CHK
	vib->motor_en = of_get_named_gpio(pdev->dev.of_node, "samsung,motor-en", 0);
	pr_info("[VIB] %s motor_en %d\n", __func__, vib->motor_en);

	if (!gpio_is_valid(vib->motor_en)) {
		pr_err("%s:%d, reset gpio not specified\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	if (gpio_request(vib->motor_en, "motor_en")) {
		pr_err("%s:%d, request not specified\n",
				__func__, __LINE__);
		return -EINVAL;
	}
#endif
	input_dev = input_allocate_device();
	if (!vib || !input_dev) {
		dev_err(&pdev->dev, "couldn't allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	INIT_WORK(&vib->work, ff_dc_work);
	vib->dev = &pdev->dev;
	vib->vib_input_dev = input_dev;

	input_dev->name = "ff_dc_vib_memless";
	input_dev->id.version = 1;
	input_dev->dev.parent = &pdev->dev;
	input_dev->close = ff_dc_vib_close;
	input_set_drvdata(input_dev, vib);
	input_set_capability(vib->vib_input_dev, EV_FF, FF_RUMBLE);

	error = input_ff_create_memless(input_dev, NULL,
					ff_dc_vib_play_effect);
	if (error) {
		dev_err(&pdev->dev,
			"couldn't register vibrator as FF device\n");
		goto err_free_mem;
	}

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "couldn't register input device\n");
		goto err_destroy_memless;
	}

	platform_set_drvdata(pdev, vib);
	return 0;

err_destroy_memless:
	input_ff_destroy(input_dev);
err_free_mem:
	input_free_device(input_dev);
	kfree(vib);

	return error;
}

static int ff_dc_vib_remove(struct platform_device *pdev)
{
	struct ff_dc_vib *vib = platform_get_drvdata(pdev);

	input_unregister_device(vib->vib_input_dev);
	kfree(vib);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ff_dc_vib_suspend(struct device *dev)
{
	struct ff_dc_vib *vib = dev_get_drvdata(dev);

	/* Turn off the vibrator */
	ff_dc_vib_set(vib, 0);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ff_dc_vib_pm_ops, ff_dc_vib_suspend, NULL);

static const struct of_device_id vib_motor_match[] = {
	{	.compatible = "ff-dc-vib",
	},
	{}
};

static struct platform_driver ff_dc_vib_driver = {
	.probe		= ff_dc_vib_probe,
	.remove		= ff_dc_vib_remove,
	.driver		= {
		.name	= "ff-dc-vib",
		.owner	= THIS_MODULE,
		.of_match_table = vib_motor_match,
		.pm	= &ff_dc_vib_pm_ops,
	},
};
module_platform_driver(ff_dc_vib_driver);

MODULE_ALIAS("platform:ff_dc_vib");
MODULE_DESCRIPTION("ff-dc vibrator driver based on ff-memless framework");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("yongz.zhang@samsung.com");
