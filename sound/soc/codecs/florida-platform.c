/*
 * florida-platform.c  -- ASoC Audio driver for CLEARWATER-compr-platform
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
#include "florida.h"

static int florida_platform_probe(struct platform_device *pdev)
{
	int ret;

	ret = florida_asoc_platform_register(&pdev->dev);
	if (ret)
		dev_err(&pdev->dev, "Can't register florida-platform\n");
	else
		dev_info(&pdev->dev, "florida-platform success\n");

	return ret;
}

static int florida_platform_remove(struct platform_device *pdev)
{
	florida_asoc_platform_unregister(&pdev->dev);
	return 0;
}

static struct platform_driver florida_platform_driver = {
	.driver = {
		.name = "florida-platform",
		.owner = THIS_MODULE,
	},
	.probe = florida_platform_probe,
	.remove = florida_platform_remove,
};

module_platform_driver(florida_platform_driver);

MODULE_DESCRIPTION("ASoC Florida Compr Platform driver");
MODULE_AUTHOR("Kwang-Hui Cho <kwanghui.cho@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:florida-platform");
