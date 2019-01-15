/*
 * clearwater-platform.c  -- ASoC Audio driver for CLEARWATER-compr-platform
 *
 * Author: Kwang-Hui Cho <kwanghui.cho@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include "clearwater.h"

static int clearwater_platform_probe(struct platform_device *pdev)
{
	int ret;

	ret = clearwater_asoc_platform_register(&pdev->dev);
	if (ret)
		dev_err(&pdev->dev, "Can't register clearwater-platform\n");
	else
		dev_info(&pdev->dev, "clearwater-platform success\n");

	return ret;
}

static int clearwater_platform_remove(struct platform_device *pdev)
{
	clearwater_asoc_platform_unregister(&pdev->dev);
	return 0;
}

static struct platform_driver clearwater_platform_driver = {
	.driver = {
		.name = "clearwater-platform",
		.owner = THIS_MODULE,
	},
	.probe = clearwater_platform_probe,
	.remove = clearwater_platform_remove,
};

module_platform_driver(clearwater_platform_driver);

MODULE_DESCRIPTION("ASoC Clearwater Compr Platform driver");
MODULE_AUTHOR("Kwang-Hui Cho <kwanghui.cho@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:clearwater-platform");
