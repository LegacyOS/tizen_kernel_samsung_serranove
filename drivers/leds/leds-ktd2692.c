/*
 * LED driver - leds-ktd2692.c
 *
 * Copyright (C) 2013 Sunggeun Yim <sunggeun.yim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/pwm.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/leds-ktd2692.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

extern struct class *camera_class;	/*sys/class/camera */
struct device *ktd2692_dev;

struct ktd2692_platform_data *global_ktd2692data;
struct device *global_dev;

void ktd2692_setGpio(int onoff)
{
	if (onoff)
		__gpio_set_value(global_ktd2692data->flash_control, 1);
	else
		__gpio_set_value(global_ktd2692data->flash_control, 0);

}

void ktd2692_set_low_bit(void)
{
	__gpio_set_value(global_ktd2692data->flash_control, 0);
	udelay(T_L_LB);		/* 12ms */
	__gpio_set_value(global_ktd2692data->flash_control, 1);
	udelay(T_H_LB);		/* 4ms */
}

void ktd2692_set_high_bit(void)
{
	__gpio_set_value(global_ktd2692data->flash_control, 0);
	udelay(T_L_HB);		/* 4ms */

	__gpio_set_value(global_ktd2692data->flash_control, 1);
	udelay(T_H_HB);		/* 12ms */
}

static int ktd2692_set_bit(unsigned int bit)
{
	if (bit)
		ktd2692_set_high_bit();
	else
		ktd2692_set_low_bit();

	return 0;
}

static int ktd2692_write_data(unsigned data)
{
	int err = 0;
	unsigned int bit = 0;

	/* Data Start Condition */
	__gpio_set_value(global_ktd2692data->flash_control, 1);
	udelay(T_SOD);		/*15us */

	/* BIT 7 */
	bit = ((data >> 7) & 0x01);
	ktd2692_set_bit(bit);

	/* BIT 6 */
	bit = ((data >> 6) & 0x01);
	ktd2692_set_bit(bit);

	/* BIT 5 */
	bit = ((data >> 5) & 0x01);
	ktd2692_set_bit(bit);

	/* BIT 4 */
	bit = ((data >> 4) & 0x01);
	ktd2692_set_bit(bit);

	/* BIT 3 */
	bit = ((data >> 3) & 0x01);
	ktd2692_set_bit(bit);

	/* BIT 2 */
	bit = ((data >> 2) & 0x01);
	ktd2692_set_bit(bit);

	/* BIT 1 */
	bit = ((data >> 1) & 0x01);
	ktd2692_set_bit(bit);

	/* BIT 0 */
	bit = ((data >> 0) & 0x01);
	ktd2692_set_bit(bit);

	__gpio_set_value(global_ktd2692data->flash_control, 0);

	udelay(T_EOD_L);	/*4us */

	/* Data End Condition */
	__gpio_set_value(global_ktd2692data->flash_control, 1);
	udelay(T_EOD_H);

	return err;
}

/*
*	echo 0 > /sys/class/camera/irled/rear_flash disable irled
*	echo 1 > /sys/class/camera/irled/rear_flash enable movie mode
*	echo 2 > /sys/class/camera/irled/rear_flash enable flash mode
*/
ssize_t ktd2692_store(struct device *dev,
		      struct device_attribute *attr, const char *buf,
		      size_t count)
{
	int value = 0;
	int ret = 0;
	unsigned long flags = 0;

	if ((buf == NULL) || kstrtouint(buf, 10, &value))
		return count;

	global_ktd2692data->sysfs_input_data = value;

	if (value <= 0) {
		ret =
		    gpio_request(global_ktd2692data->flash_control,
				 "qcom,irled-gpio");
		if (ret) {
			LED_ERROR("Failed to requeset ktd2692_led_control\n");
		} else {
			LED_INFO("KTD2692-TORCH OFF. : E(%d)\n", value);

			global_ktd2692data->mode_status =
			    KTD2692_DISABLES_MOVIE_FLASH_MODE;
			spin_lock_irqsave(&global_ktd2692data->int_lock, flags);
			ktd2692_write_data(global_ktd2692data->mode_status |
					KTD2692_ADDR_MOVIE_FLASHMODE_CONTROL);
			spin_unlock_irqrestore(&global_ktd2692data->int_lock,
					       flags);

			ktd2692_setGpio(0);

			gpio_free(global_ktd2692data->flash_control);
			LED_INFO("KTD2692-TORCH OFF. : X(%d)\n", value);
		}

	} else {

		ret =
		    gpio_request(global_ktd2692data->flash_control,
				 "qcom,irled-gpio");
		if (ret) {
			LED_ERROR("Failed to requeset qcom,irled-gpio\n");
		} else {
			LED_INFO("KTD2692-TORCH ON. : E(%d)\n", value);
			gpio_direction_output(global_ktd2692data->flash_control,
					      0);

			spin_lock_irqsave(&global_ktd2692data->int_lock, flags);

			/* use the internel defualt setting */
			LED_INFO("set lvp votage: %0x\n",
				global_ktd2692data->LVP_Voltage |
				KTD2692_ADDR_LVP_SETTING);
			ktd2692_write_data(global_ktd2692data->LVP_Voltage |
					   KTD2692_ADDR_LVP_SETTING);


			/* use the internel defualt setting */
			ktd2692_write_data(global_ktd2692data->flash_timeout |
					   KTD2692_ADDR_FLASH_TIMEOUT_SETTING);

			if (value == 1) {
				LED_INFO("set movie_current_value: %0x\n",
					global_ktd2692data->movie_current_value
					| KTD2692_ADDR_MOVIE_CURRENT_SETTING);
				global_ktd2692data->mode_status =
				    KTD2692_ENABLE_MOVIE_MODE;
				ktd2692_write_data
				    (global_ktd2692data->movie_current_value |
				     KTD2692_ADDR_MOVIE_CURRENT_SETTING);
			} else {
				LED_INFO("set flash_current_value: %0x\n",
					global_ktd2692data->flash_current_value
					| KTD2692_ADDR_FLASH_CURRENT_SETTING);
				global_ktd2692data->mode_status =
				    KTD2692_ENABLE_FLASH_MODE;
				ktd2692_write_data
				    (global_ktd2692data->flash_current_value |
				     KTD2692_ADDR_FLASH_CURRENT_SETTING);
			}

			LED_INFO("set movie flash mode: %0x\n",
				global_ktd2692data->mode_status |
				KTD2692_ADDR_MOVIE_FLASHMODE_CONTROL);
			ktd2692_write_data(global_ktd2692data->mode_status |
					KTD2692_ADDR_MOVIE_FLASHMODE_CONTROL);
			spin_unlock_irqrestore(&global_ktd2692data->int_lock,
					       flags);

			gpio_free(global_ktd2692data->flash_control);
			LED_INFO("KTD2692-TORCH ON. : X(%d)\n", value);
		}
	}

	return count;
}

ssize_t ktd2692_show(struct device *dev,
		     struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(buf),
			"%d\n", global_ktd2692data->sysfs_input_data);
}

static DEVICE_ATTR(rear_flash, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
		   ktd2692_show, ktd2692_store);

/*
*	echo 0 ~ 15 > /sys/class/camera/irled/movie_current
*	to set current for movie mode
*
*/
ssize_t ktd2692_movie_current_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int value = 0;

	if ((buf == NULL) || kstrtouint(buf, 10, &value))
		return count;

	global_ktd2692data->movie_current_value = value;
	LED_INFO("movie_current_value. : X(%d)\n",
		 global_ktd2692data->movie_current_value);
	return count;
}

ssize_t ktd2692_movie_current_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(buf),
			"%d\n", global_ktd2692data->movie_current_value);
}

static DEVICE_ATTR(movie_current,
		   S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
		   ktd2692_movie_current_show, ktd2692_movie_current_store);

/*
*	echo 0 ~ 15 > /sys/class/camera/irled/flash_current
*	to set current for flash mode
*/
ssize_t ktd2692_flash_current_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int value = 0;

	if ((buf == NULL) || kstrtouint(buf, 10, &value))
		return count;

	global_ktd2692data->flash_current_value = value;
	LED_INFO("flash_current_value. : X(%d)\n",
		 global_ktd2692data->flash_current_value);
	return count;
}

ssize_t ktd2692_flash_current_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(buf),
			"%d\n", global_ktd2692data->flash_current_value);
}

static DEVICE_ATTR(flash_current,
		   S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
		   ktd2692_flash_current_show, ktd2692_flash_current_store);

/*
*	echo 0~7 > /sys/class/camera/irled/flash_timeout
*	to set flash_timeout
*/
ssize_t ktd2692_flash_timeout_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int value = 0;

	if ((buf == NULL) || kstrtouint(buf, 10, &value))
		return count;

	global_ktd2692data->flash_timeout = value;
	LED_INFO("flash_timeout. : X(%d)\n", global_ktd2692data->flash_timeout);
	return count;
}

ssize_t ktd2692_flash_timeout_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(buf),
			"%d\n", global_ktd2692data->flash_timeout);
}

static DEVICE_ATTR(flash_timeout,
		   S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
		   ktd2692_flash_timeout_show, ktd2692_flash_timeout_store);

static int ktd2692_parse_dt(struct device *dev,
			    struct ktd2692_platform_data *pdata)
{
	struct device_node *dnode = dev->of_node;
	int ret = 0;

	/* Defulat Value */
	pdata->LVP_Voltage = KTD2692_DISABLE_LVP;
	pdata->flash_timeout = KTD2692_DISABLE_TIMER;
	pdata->min_current_value = KTD2692_MIN_CURRENT_240mA;
	pdata->movie_current_value = KTD2692_MOVIE_CURRENT5;
	pdata->flash_current_value = KTD2692_FLASH_CURRENT6;
	pdata->mode_status = KTD2692_DISABLES_MOVIE_FLASH_MODE;

	/* get gpio */
	pdata->flash_control = of_get_named_gpio(dnode, "qcom,irled-gpio", 0);
	if (!gpio_is_valid(pdata->flash_control)) {
		dev_err(dev, "failed to get flash_control\n");
		ret = -1;
	}

	return ret;
}

static int ktd2692_probe(struct platform_device *pdev)
{
	struct ktd2692_platform_data *pdata;
	int ret = 0;

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&pdev->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = ktd2692_parse_dt(&pdev->dev, pdata);
		if (ret < 0)
			return -EFAULT;
	} else {
		pdata = pdev->dev.platform_data;
		if (pdata == NULL)
			return -EFAULT;
	}

	global_ktd2692data = pdata;
	global_dev = &pdev->dev;

	LED_INFO("KTD2692_LED Probe\n");

	ktd2692_dev = device_create(camera_class, NULL, 0, NULL, "irled");
	if (IS_ERR(ktd2692_dev))
		LED_ERROR("Failed to create device(flash)!\n");

	if (device_create_file(ktd2692_dev, &dev_attr_rear_flash) < 0) {
		LED_ERROR("failed to create device file, %s\n",
			  dev_attr_rear_flash.attr.name);
	}

	if (device_create_file(ktd2692_dev, &dev_attr_movie_current) < 0) {
		LED_ERROR("failed to create device file, %s\n",
			  dev_attr_movie_current.attr.name);
	}

	if (device_create_file(ktd2692_dev, &dev_attr_flash_current) < 0) {
		LED_ERROR("failed to create device file, %s\n",
			  dev_attr_flash_current.attr.name);
	}

	if (device_create_file(ktd2692_dev, &dev_attr_flash_timeout) < 0) {
		LED_ERROR("failed to create device file, %s\n",
			  dev_attr_flash_current.attr.name);
	}

	spin_lock_init(&pdata->int_lock);

	return 0;
}

static int ktd2692_remove(struct platform_device *pdev)
{
	device_remove_file(ktd2692_dev, &dev_attr_flash_timeout);
	device_remove_file(ktd2692_dev, &dev_attr_flash_current);
	device_remove_file(ktd2692_dev, &dev_attr_movie_current);
	device_remove_file(ktd2692_dev, &dev_attr_rear_flash);
	device_destroy(camera_class, 0);
	class_destroy(camera_class);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id ktd2692_dt_ids[] = {
	{.compatible = "qcom,camera-ktd2692-irled",},
	{},
};

/*MODULE_DEVICE_TABLE(of, ktd2692_dt_ids);*/
#endif

static struct platform_driver ktd2692_driver = {
	.driver = {
		   .name = ktd2692_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = ktd2692_dt_ids,
#endif
		   },
	.probe = ktd2692_probe,
	.remove = ktd2692_remove,
};

static int __init ktd2692_init(void)
{
	return platform_driver_register(&ktd2692_driver);
}

static void __exit ktd2692_exit(void)
{
	platform_driver_unregister(&ktd2692_driver);
}

module_init(ktd2692_init);
module_exit(ktd2692_exit);

MODULE_AUTHOR("sunggeun yim <sunggeun.yim@samsung.com.com>");
MODULE_DESCRIPTION("KTD2692 driver");
MODULE_LICENSE("GPL");
