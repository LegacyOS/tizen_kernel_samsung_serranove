/*
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Authors:
 *	Joong-Mock Shin <jmock.shin@samsung.com>
 *	Taeheon Kim <th908.kim@samsung.com>
 *	Sangmin Lee <lsmin.lee@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#include <linux/input/hall_sensor.h>

static struct class *flip_class;
static struct device *hall_dev;


static void hall_sensor_work(struct work_struct *work)
{
	struct hall_sensor_driverdata *ddata =
		container_of(work, struct hall_sensor_driverdata,
				detect_dwork.work);
	char *envp[2];

	ddata->status = gpio_get_value(ddata->gpio_detect);

	/*  send uevent */
	envp[0] = "COVER";
	envp[1] = NULL;
	kobject_uevent_env(&hall_dev->kobj, KOBJ_CHANGE, envp);

	dev_info(&ddata->dev->dev, "[Hall] COVER: %s uevent is sent\n",
			ddata->status ? "OPEN" : "CLOSE");
}

static irqreturn_t hall_status_detect(int irq, void *dev_id)
{
	struct hall_sensor_driverdata *ddata = dev_id;

	ddata->status = gpio_get_value(ddata->gpio_detect);

	dev_info(&ddata->dev->dev,
		"[Hall] %s [%d]\n", __func__, ddata->status);

	cancel_delayed_work_sync(&ddata->detect_dwork);

	if(ddata->status) {
		wake_lock_timeout(&ddata->wake_lock, HZ / 20);
		schedule_delayed_work(&ddata->detect_dwork, HZ / 100);
	} else {
		wake_unlock(&ddata->wake_lock);
		schedule_delayed_work(&ddata->detect_dwork, 0);
	}

	return IRQ_HANDLED;
}

static ssize_t cover_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct hall_sensor_driverdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, 5, "%d\n", ddata->status);
}

static DEVICE_ATTR(cover_status, 0444, cover_status_show, NULL);

#ifdef CONFIG_OF
static int hall_sensor_parse_dt(struct device *dev,
			struct hall_sensor_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	/* gpio info */
	pdata->gpio_detect = of_get_named_gpio_flags(np, "hall,gpio_flip_cover",
				0, &pdata->irq_gpio_flags);
	return 0;
}

static struct of_device_id hall_sensor_of_match[] = {
	{ .compatible = "hall", },
	{ },
};
MODULE_DEVICE_TABLE(of, hall_sensor_of_match);
#else
static int hall_sensor_parse_dt(struct device *dev,
			struct hall_sensor_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int hall_sensor_probe(struct platform_device *pdev)
{
	struct hall_sensor_platform_data *pdata;
	struct hall_sensor_driverdata *ddata;
	int ret = 0;

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct hall_sensor_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&pdev->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = hall_sensor_parse_dt(&pdev->dev, pdata);
		if (ret) {
			dev_err(&pdev->dev, "Error hall_sensor_parse_dt\n");
			return ret;
		}
	} else {
		pdata = pdev->dev.platform_data;
		if (!pdata) {
			dev_err(&pdev->dev, "No hall_sensor platform data\n");
			return -EINVAL;
		}
	}

	ddata = kzalloc(sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->gpio_detect = pdata->gpio_detect;
	platform_set_drvdata(pdev, ddata);
	ddata->dev = pdev;

	ret = gpio_request(ddata->gpio_detect, "hall_sensor_irq");
	if (ret) {
		dev_err(&pdev->dev, "[HALL]%s:"\
			" unable to request hall_sensor_irq [%d]\n",\
			__func__, ddata->gpio_detect);
		return -EINVAL;
	}

	ret = gpio_direction_input(ddata->gpio_detect);
	if (ret) {
		dev_err(&pdev->dev,
			"[HALL]%s: unable to request input pin [%d]\n",
			__func__, ddata->gpio_detect);
		return -EINVAL;
	}

	ddata->hall_irq = gpio_to_irq(ddata->gpio_detect);

	wake_lock_init(&ddata->wake_lock, WAKE_LOCK_SUSPEND,
		"hall_wake_lock");

	flip_class = class_create(THIS_MODULE, "flip");
	if (IS_ERR(flip_class))
		goto out;

	hall_dev = device_create(flip_class, NULL, 0, NULL, "hall_ic");

	ret = device_create_file(hall_dev, &dev_attr_cover_status);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: device_create failed\n", __func__);
		goto err_device_create_file;
	}

	dev_set_drvdata(hall_dev, ddata);
	/* start workqueue */
	INIT_DELAYED_WORK(&ddata->detect_dwork, hall_sensor_work);

	ret = request_threaded_irq(ddata->hall_irq , NULL, hall_status_detect,
		IRQF_DISABLED | IRQF_TRIGGER_RISING |
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"hall_status", ddata);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request hall irq[%d] gpio %d\n",
		ddata->hall_irq , ddata->gpio_detect);
		goto out;
	} else {
		/* update the current status */
		schedule_delayed_work(&ddata->detect_dwork, HZ/2);
	}
	device_init_wakeup(&pdev->dev, true);

	dev_info(&pdev->dev, "%s done successfully\n", __func__);

	return 0;

err_device_create_file:
	device_remove_file(hall_dev, &dev_attr_cover_status);
out:
	wake_lock_destroy(&ddata->wake_lock);
	gpio_free(ddata->gpio_detect);
	kfree(ddata);
	return ret;
}

static int hall_sensor_remove(struct platform_device *pdev)
{
	struct hall_sensor_driverdata *ddata = platform_get_drvdata(pdev);

	flush_delayed_work(&ddata->detect_dwork);
	cancel_delayed_work_sync(&ddata->detect_dwork);
	wake_lock_destroy(&ddata->wake_lock);
	kfree(ddata);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int hall_sensor_resume(struct device *dev)
{
	struct hall_sensor_driverdata *ddata = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(ddata->hall_irq);

	return 0;
}

static int hall_sensor_suspend(struct device *dev)
{
	struct hall_sensor_driverdata *ddata = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(ddata->hall_irq);

	return 0;
}

static const struct dev_pm_ops hall_pm_ops = {
	.suspend = hall_sensor_suspend,
	.resume = hall_sensor_resume,
};

static struct platform_driver hall_sensor_driver = {
	.probe		= hall_sensor_probe,
	.remove		= hall_sensor_remove,
	.driver		= {
		.name	= "hall",
		.owner	= THIS_MODULE,
		.pm	= &hall_pm_ops,
		.of_match_table = hall_sensor_of_match
	},
};

static int __init hall_sensor_init(void)
{
	return platform_driver_register(&hall_sensor_driver);
}

static void __exit hall_sensor_exit(void)
{
	platform_driver_unregister(&hall_sensor_driver);
}

late_initcall(hall_sensor_init);
module_exit(hall_sensor_exit);

/* Module information */
MODULE_AUTHOR("Joong-Mock Shin <jmock.shin@samsung.com>");
MODULE_DESCRIPTION("Hall sensor driver");
MODULE_LICENSE("GPL");
