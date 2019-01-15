/*
 * Copyright (C) 2015 Samsung Electronics
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
#include <linux/regulator/consumer.h>
#include "leds-f8316-svr.h"

#ifdef DEBUG_SEGMENT
static u8 dot[21] = {
	0x0, 0xf0, 0xf0, 0x00, 0x0f, 0x0f,
	0x0, 0xf0, 0xf0, 0x00, 0x0f, 0x0f,
	0x0, 0xf0, 0xf0, 0x00, 0x0f, 0x0f,
	0x0, 0xf0, 0xf0
};
#endif
struct i2c_dev_info {
	uint8_t dev_addr;
	struct i2c_client *client;
};
struct i2c_client *f8316_svr_led_client;

static int f8316_i2c_write(struct i2c_client *client, u8 reg, u8 data)
{
	int ret = 0;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0) {
		DEV_ERROR("error[%d]\n", ret);
		return -EIO;
	}

	return ret;
}

static int f8316_i2c_matrix_write(struct i2c_client *client, u8 reg,
		u8 length, const u8 *values)
{
	int i = 0;
	int ret = 0;

	while (i != length) {
		ret = i2c_smbus_write_byte_data(client, reg, values[i]);
		if (ret < 0) {
			DEV_ERROR("error[%d][%d]\n", ret, i);
			break;
		}
		reg = reg + 1;
		i++;
	}

	return ret;
}

static int f8316_parse_dt(struct f8316_platform_data *pdata)
{
	struct device_node *np = pdata->client->dev.of_node;

	/* gpio pins */
	pdata->reset_gpio = of_get_named_gpio_flags(np,
		"f8316,gpio_reset", 0, NULL);
	if (pdata->reset_gpio > 0)
		DEV_INFO("gpio: reset = %d\n", pdata->reset_gpio);

	return 0;
}

static int f8316_power_init(struct f8316_platform_data *pdata)
{
	int ret = 0;

	pr_info("f8316_power_init\n");

	pdata->vdd = regulator_get(&pdata->client->dev, "vdd");
	if (IS_ERR(pdata->vdd)) {
		ret = PTR_ERR(pdata->vdd);
		dev_err(&pdata->client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}
	ret = regulator_set_voltage(pdata->vdd, 3300000, 3300000);
	if (ret) {
		dev_err(&pdata->client->dev, "unable to set voltage for vdd_vreg, %d\n",
			ret);
		return ret;
	}

	ret = regulator_enable(pdata->vdd);
	if (ret) {
		dev_err(&pdata->client->dev, "unable to enable vdd, %d\n",
			ret);
		return ret;
	}

	return ret;
}

static int f8316_reset(struct f8316_platform_data *pdata)
{
	int ret = 0;

	pr_info("f8316 gpio init\n");

	ret = gpio_request_one(pdata->reset_gpio, GPIOF_OUT_INIT_HIGH,
			"8316_reset");
	if (ret) {
		DEV_ERROR(" failed to request gpio %d\n",
				pdata->reset_gpio);
		return ret;
	}

	msleep(50);

	return ret;
}

int update_svr_led_matrix(int mode, u8 *matrix)
{
	int ret = 0;
	int i, retry = 3;

	/*TODO*/
	for (i = 0; i < retry; i++) {
		ret = f8316_i2c_matrix_write(f8316_svr_led_client, REG_LED_BUF1,
					REG_BUF1_LEN, matrix);
		if (ret < 0) {
			DEV_ERROR("failed[%d]to update matrix retry[%d]\n",
					ret, i);
		} else {
			DEV_INFO("Success update matrix\n");
			break;
		}
	}

	for (i = 0; i < retry; i++) {
		ret = f8316_i2c_write(f8316_svr_led_client, REG_LED_CTRL1,
					REG_LED_UPDATE | REG_LED_SLIDE);
		if (ret < 0) {
			DEV_ERROR("failed[%d]to send CTRL regretry[%d]\n",
					ret, i);
		} else {
			DEV_INFO("Success CTRL reg\n");
			break;
		}
	}

	return 0;
}

static int f8316_led_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct f8316_platform_data *pdata;
	int ret = 0;

	pdata = devm_kzalloc(&client->dev,
				sizeof(struct f8316_platform_data), GFP_KERNEL);
	if (pdata == NULL) {
		DEV_ERROR("failed to allocate memory\n");
		return -ENOMEM;
	}

	pdata->client = client;
	i2c_set_clientdata(client, pdata);

	f8316_svr_led_client = client;

	f8316_parse_dt(pdata);

	ret = f8316_power_init(pdata);
	if (ret)
		DEV_ERROR("failed to initialize regulator\n");

	ret = f8316_reset(pdata);
	if (ret)
		DEV_ERROR("failed to initialize gpio\n");

	DEV_INFO("f8316 probed successfully %x\n", f8316_svr_led_client->addr);

#ifdef DEBUG_SEGMENT
	update_svr_led_matrix(0, dot);
#endif
	return 0;
}

static int f8316_led_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int f8316_led_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int f8316_led_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id f8316_led_id[] = {
	{ "f8316", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, f8316_led_id);

static struct i2c_driver f8316_led_i2c_driver = {
	.driver  = {
		.name  = "f8316",
		.owner  = THIS_MODULE,
	},
	.probe  = f8316_led_i2c_probe,
	.remove  = f8316_led_i2c_remove,
	.suspend = f8316_led_i2c_suspend,
	.resume = f8316_led_i2c_resume,
	.id_table  = f8316_led_id,
};

static int __init f8316_init(void)
{
	int ret;

	ret = i2c_add_driver(&f8316_led_i2c_driver);
	if (ret < 0) {
		DEV_ERROR("fail to init\n");
		return ret;
	}

	return 0;
}

static void __exit f8316_exit(void)
{
	i2c_del_driver(&f8316_led_i2c_driver);
}

module_init(f8316_init);
module_exit(f8316_exit);

MODULE_AUTHOR("Jung-Kee Kim <jungki45.kim@samsung.com>");
MODULE_DESCRIPTION("SVR-Led driver");
MODULE_LICENSE("GPL");
