/*
 * Samsung Tizen development team
 *
 * drivers/pinctrl/sec-pinctrl-dvs.c
 *
 * Drivers for appying gpio config prepare to samsung gpio debugging & verification.
 *
 * Copyright (C) 2015, Samsung Electronics.
 *
 * This program is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>
#include "core.h"

struct secgpio_pinctrl {
	struct device *dev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *default_state;
	struct pinctrl_state *sleep_state;
	int *gpio_no;
	int num_pinctrl;
};

struct secgpio_pinctrl pinctrl_config;

static int sec_pinctrl_dvs_remove(struct platform_device *pdev)
{
	return 0;
}

static int sec_pinctrl_suspend(struct device *dev)
{
	int err = pinctrl_select_state(pinctrl_config.pinctrl,
								pinctrl_config.sleep_state);
	if (err) {
		dev_err(dev,	"Can't select pinctrl sleep state\n");
	}

	return 0;
}

static int sec_pinctrl_resume(struct device *dev)
{
	int err = pinctrl_select_state(pinctrl_config.pinctrl,
								pinctrl_config.default_state);
	if (err) {
		dev_err(dev, "Can't select pinctrl default state\n");
	}

	return 0;
}

static struct of_device_id sec_pinctrl_dvs_match_table[] = {
	{ .compatible = "sec_pinctrl_dvs",},
	{},
};
MODULE_DEVICE_TABLE(of, sec_pinctrl_dvs_match_table);

static const struct dev_pm_ops sec_pinctrl_pm_ops= {
	.suspend	= sec_pinctrl_suspend,
	.resume	= sec_pinctrl_resume,
};

static int sec_pinctrl_parse_dt(struct device *dev)
{
	int ret = 0;

	pinctrl_config.pinctrl = devm_pinctrl_get(dev);

	if (!pinctrl_config.pinctrl) {
		pr_info("fail to devm_pinctrl_get pinctrl\n");
		goto out;
	}

	pinctrl_config.default_state =
		pinctrl_lookup_state(pinctrl_config.pinctrl, "default");

	if (IS_ERR_OR_NULL(pinctrl_config.default_state)) {
		dev_err(dev, "Failed to look up default state\n");
		ret = 1;
		goto out;
	}

	pinctrl_config.sleep_state =
		pinctrl_lookup_state(pinctrl_config.pinctrl, "sleep");

	if (IS_ERR_OR_NULL(pinctrl_config.sleep_state)) {
		dev_err(dev, "Failed to look up sleep state\n");
		ret = 1;
	}

out:
	return ret;

}

static int sec_pinctrl_dvs_probe(struct platform_device *pdev)
{
	int error = 0;

	/* pinctrl probe */
	if (!sec_pinctrl_parse_dt(&(pdev->dev))) {
		pr_info("Failed to parse dt in %s\n", __func__);
	}

	error = pinctrl_select_state(pinctrl_config.pinctrl,
								pinctrl_config.default_state);
	if (error) {
		dev_err(&pdev->dev,	"Can't select  pinctrl default state\n");
	}

	return 0;
}


static struct platform_driver sec_pinctrl_dvs = {
	.probe = sec_pinctrl_dvs_probe,
	.remove = sec_pinctrl_dvs_remove,
	.driver = {
		.name = "sec_pinctrl_dvs",
		.owner = THIS_MODULE,
		.pm = &sec_pinctrl_pm_ops,
		.of_match_table = sec_pinctrl_dvs_match_table,
	},
};

static int __init sec_pinctrl_dvs_init(void)
{
	int ret;
	ret = platform_driver_register(&sec_pinctrl_dvs);

	return ret;
}

static void __exit sec_pinctrl_dvs_exit(void)
{
	platform_driver_unregister(&sec_pinctrl_dvs);
}

postcore_initcall(sec_pinctrl_dvs_init);
module_exit(sec_pinctrl_dvs_exit);

MODULE_AUTHOR("sirano06.lee@samsung.com");
MODULE_DESCRIPTION("Apply gpio config for GPIO DVS");
MODULE_LICENSE("GPL");
