/*
* rt8555_bl.c - driver for RT8555 backlight driver chip
*
* Copyright  2011, Samsung Electronics Co. Ltd. All Rights Reserved.
*
* Contact: Raghu Bankapur <rb.bankapur@samsung.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2 of the License.
*
*/

#include <linux/kernel.h>
#include <asm/unaligned.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/backlight.h>
#include <linux/notifier.h>
#include <linux/fb.h>

struct tuning_table {
	u8* table;
	u8 table_length;
};

struct brt_value {
	int level;			/* Platform setting values */
	int tune_level;			/* Chip Setting values */
};

struct backlight_platform_data {
	unsigned	int gpio_backlight_en;
	unsigned	int gpio_backlight_pwm;
	u32 en_gpio_flags;

	int gpio_sda;
	u32 sda_gpio_flags;

	int gpio_scl;
	u32 scl_gpio_flags;
};

struct backlight_info {
	struct i2c_client *client;
	struct backlight_platform_data	*pdata;
	struct tuning_table bl_ic_settings;
	struct tuning_table bl_control;
};

static void rt8555_backlight_set_brightness(int level);

#define MAX_BRIGHTNESS 100
#define MIN_BRIGHTNESS 1
#define DEFAULT_BRIGHTNESS 60
#define DIMMING_VALUE 20
#define SLEEP_OUT_DELAY 50

struct notifier_block fb_notifier;
struct backlight_info *bl_info;
static const char *bl_ic_name;
static int is_poweron = 1;
static int wakeup_brightness = DEFAULT_BRIGHTNESS;

struct brt_value brt_table_rt[] = {
	/* Maximum */
	{100, 255},
	{99, 242},
	{98, 222},
	{97, 212},
	{96, 202},
	{95, 197},
	{94, 192},
	{93, 187},
	{92, 182},
	{91, 177},
	{90, 172},
	{89, 168},
	{88, 165},
	{87, 162},
	{86, 159},
	{85, 156},
	{84, 153},
	{83, 150},
	{82, 147},
	{81, 144},
	{80, 141},
	{79, 138},
	{78, 135},
	{77, 132},
	{76, 129},
	{75, 126},
	{74, 123},
	{73, 120},
	{72, 117},
	{71, 114},
	/* Default */
	{70, 111},
	{69, 108},
	{68, 105},
	{67, 102},
	{66, 99},
	{65, 96},
	{64, 93},
	{63, 91},
	{62, 89},
	{61, 87},
	{60, 85},
	{59, 83},
	{58, 81},
	{57, 79},
	{56, 77},
	{55, 75},
	{54, 73},
	{53, 71},
	{52, 69},
	{51, 67},
	{50, 65},
	{49, 63},
	{48, 61},
	{47, 59},
	{46, 57},
	{45, 55},
	{44, 53},
	{43, 51},
	{42, 49},
	{41, 47},
	{40, 45},
	{39, 44},
	{38, 43},
	{37, 42},
	{36, 41},
	{35, 40},
	{34, 39},
	{33, 38},
	{32, 37},
	{31, 36},
	{30, 35},
	{29, 34},
	{28, 33},
	{27, 32},
	{26, 31},
	{25, 30},
	{24, 29},
	{23, 28},
	{22, 27},
	{21, 26},
	{20, 25},
	{19, 24},
	{18, 23},
	{17, 22},
	{16, 21},
	{15, 20},
	{14, 19},
	{13, 18},
	{12, 17},
	{11, 16},
	{10, 15},
	{9, 14},
	{8, 13},
	{7, 12},
	{6, 11},
	{5, 10},
	{4, 9},
	{3, 8},
	{2, 7},
	/* Minimum */
	{1, 6},
	{0, 5},
};

#define NB_BRT_LEVEL (int)(sizeof(brt_table_rt)/sizeof(struct brt_value))

static int rt8555_backlight_i2c_write(struct i2c_client *client,
		u8 reg,  u8 val, unsigned int len)
{
	int rc = 0;
	int retry = 3;
	u8 temp_val = val;

	while (retry--) {
		rc = i2c_smbus_write_i2c_block_data(client,
				reg, len, &temp_val);
		if (rc >= 0)
			return rc;

		dev_info(&client->dev, "%s: i2c transfer error. %d\n", __func__, rc);
	}

	return rc;
}

static void rt8555_backlight_control_i2c_dimming(void)
{
	struct backlight_info *info = bl_info;

	pr_debug("[BACKLIGHT] rt8555 backlight switch into i2c control mode\n");

	rt8555_backlight_i2c_write(info->client, info->bl_control.table[2], info->bl_control.table[3], 1);
}

static void rt8555_backlight_control_gpio(int enable)
{
	struct backlight_info *info = bl_info;

	if (!info) {
		pr_info("%s error bl_info", __func__);
		return ;
	}

	pr_info("%s :enable:[%d]\n", __func__, enable);

	if(enable) {
		if (gpio_is_valid(info->pdata->gpio_backlight_en))
			gpio_set_value(info->pdata->gpio_backlight_en, 1);
			msleep(1);
	} else {
		if (gpio_is_valid(info->pdata->gpio_backlight_en))
			gpio_set_value(info->pdata->gpio_backlight_en, 0);
			msleep(1);
	}
}

void rt8555_backlight_control_i2c(int scaled_level)
{
	struct backlight_info *info = bl_info;

	if (!info) {
		pr_info("%s error backlight_info", __func__);
		return ;
	}

	pr_info("%s : Backlight scaled_level = %d\n", __func__, scaled_level);

	if (!(info->bl_control.table)) {
		pr_info("%s : No backlight configuration :-", __func__);
		return ;
	}

	rt8555_backlight_i2c_write(info->client, info->bl_control.table[0], scaled_level, 8);
}

static int rt8555_backlight_suspend(void)
{
	pr_debug("[BACKLIGHT] rt8555_backlight_suspend\n");

	is_poweron = 0;

	rt8555_backlight_control_gpio(0);

	return 0;
}

static int rt8555_backlight_resume(void)
{
	pr_debug("[BACKLIGHT] rt8555_backlight_resume\n");

	rt8555_backlight_control_gpio(1);
	rt8555_backlight_control_i2c_dimming();
	rt8555_backlight_set_brightness(wakeup_brightness);

	is_poweron = 1;

	return 0;
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int power;

	pr_info("[%s]: Event[%lu]\n", __func__, event);

	/* If we aren't interested in this event, skip it immediately ... */
	switch (event) {
	case FB_EVENT_BLANK:
		power = *(int *)evdata->data;
		switch (power) {
		case FB_BLANK_UNBLANK:
			rt8555_backlight_resume();
			break;
		default:
			return 0;
		}
		break;
	case FB_EARLY_EVENT_BLANK:
		power = *(int *)evdata->data;
		switch (power) {
		case FB_BLANK_POWERDOWN:
			rt8555_backlight_suspend();
			break;
		default:
			return 0;
		}
		break;
	default:
		return 0;
	}

	return 0;
}

static int rt8555_register_fb(void)
{
	memset(&fb_notifier, 0, sizeof(fb_notifier));
	fb_notifier.notifier_call = fb_notifier_callback;

	return fb_register_client(&fb_notifier);
}

static void rt8555_unregister_fb(void)
{
	fb_unregister_client(&fb_notifier);
}

void rt8555_backlight_set_brightness(int level)
{
	int i = 0;
	int bl_brightness = 0;

	if (level < MIN_BRIGHTNESS)
		bl_brightness = DIMMING_VALUE;

	for (i = 0; i < NB_BRT_LEVEL; ++i) {
		if (level<= brt_table_rt[i].level
				&& level > brt_table_rt[i + 1].level) {
			bl_brightness = brt_table_rt[i].tune_level;
			break;
		}
	}

	rt8555_backlight_control_i2c(bl_brightness);
}

static int rt8555_backlight_update_status(struct backlight_device *bd)
{
	wakeup_brightness = bd->props.brightness;

	if (is_poweron) {
		rt8555_backlight_set_brightness(wakeup_brightness);
	}

	return 0;
}

static int rt8555_backlight_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static const struct backlight_ops rt8555_backlight_ops = {
	.update_status = rt8555_backlight_update_status,
	.get_brightness = rt8555_backlight_get_brightness
};

static void backlight_request_gpio(struct backlight_platform_data *pdata)
{
	int ret;
	if (gpio_is_valid(pdata->gpio_backlight_en)) {
		ret = gpio_request(pdata->gpio_backlight_en, "backlight_en");
		if (ret) {
			printk(KERN_ERR "%s: unable to request backlight_en [%d]\n",
				__func__, pdata->gpio_backlight_en);
			return ;
		}
	}
}

static int rt8555_parse_backlight_settings(struct device *dev,struct tuning_table* bl_tune, char *keystring)
{
	const char *data;
	int   len = 0;
	struct device_node *np = dev->of_node;

	data = of_get_property(np, keystring, &len);

	if (!data) {
		pr_info("%s:%d, Unable to read table %s ", __func__, __LINE__, keystring);
		return -EINVAL;
	} else
		pr_err("%s:Success to read table %s\n", __func__, keystring);

	if ((len % 2) != 0) {
		pr_err("%s:%d, Incorrect table entries for %s",
					__func__, __LINE__, keystring);
		return -EINVAL;
	}

	bl_tune->table= kzalloc(sizeof(char) * len, GFP_KERNEL);
	bl_tune->table_length = len;

	if (!bl_tune->table)
		return -ENOMEM;

	memcpy(bl_tune->table, data, len);

	return 0;

}

static int rt8555_backlight_parse_dt(struct device *dev,
			struct backlight_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->gpio_scl = of_get_named_gpio_flags(np, "backlight,scl-gpio",
				0, &pdata->scl_gpio_flags);
	pdata->gpio_sda = of_get_named_gpio_flags(np, "backlight,sda-gpio",
				0, &pdata->sda_gpio_flags);
	pdata->gpio_backlight_en = of_get_named_gpio_flags(np, "backlight-en-gpio",
				0, &pdata->en_gpio_flags);

	bl_ic_name = of_get_property(np, "backlight-ic-name", NULL);
	if (!bl_ic_name) {
		pr_info("%s:%d, Backlight IC name not specified\n",__func__, __LINE__);
	} else {
		pr_info("%s: Backlight IC Name = %s\n", __func__, bl_ic_name);
	}

	pr_info("%s gpio_scl : %d , gpio_sda : %d en : %d \n", __func__, pdata->gpio_scl, pdata->gpio_sda, pdata->gpio_backlight_en);

	return 0;
}

static int rt8555_backlight_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct backlight_platform_data *pdata;
	struct backlight_info *info;
	struct backlight_device *bd = NULL;
	int rc = 0;

	pr_info("%s", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		pr_err("[BACKLIGHT] failed to check i2c functionality!\n");
		return -EIO;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct backlight_platform_data),
				GFP_KERNEL);

		if (!pdata) {
			dev_info(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		rc = rt8555_backlight_parse_dt(&client->dev, pdata);

		if (rc)
			return rc;
	} else
		pdata = client->dev.platform_data;

	backlight_request_gpio(pdata);

	bl_info = info = kzalloc(sizeof(*info), GFP_KERNEL);

	if (!info) {
		dev_info(&client->dev, "%s: fail to memory allocation.\n", __func__);
		return -ENOMEM;
	}

	info->client = client;
	info->pdata = pdata;
	rt8555_parse_backlight_settings(&client->dev, &info->bl_control,"backlight-i2c-bl-control");

	bd = backlight_device_register("panel", &client->dev, NULL, &rt8555_backlight_ops, NULL);

	if (IS_ERR(bd)) {

		pr_info("backlight : failed to register device\n");
		return rc;
	}

	bd->props.max_brightness = MAX_BRIGHTNESS;
	bd->props.brightness = MAX_BRIGHTNESS;

	rt8555_register_fb();
	i2c_set_clientdata(client, info);
	rt8555_backlight_control_gpio(1);
	rt8555_backlight_update_status(bd);

	return rc;
}

static int rt8555_backlight_remove(struct i2c_client *client)
{
	rt8555_unregister_fb();

	return 0;
}

static const struct i2c_device_id rt8555_backlight_id[] = {
	{"rt8555", 0},
	{}
};

static struct of_device_id rt8555_backlight_match_table[] = {
	{.compatible = "richtek-bl-rt8555",},
	{},
};

static struct i2c_driver rt8555_backlight_driver = {
	.driver = {
		.name = "rt8555",
		.of_match_table = rt8555_backlight_match_table,
		   },
	.id_table = rt8555_backlight_id,
	.probe = rt8555_backlight_probe,
	.remove = rt8555_backlight_remove,
};

static int __init rt8555_backlight_init(void)
{

	int rc = 0;

	pr_info("%s", __func__);

	rc = i2c_add_driver(&rt8555_backlight_driver);
	if (rc) {
		printk(KERN_ERR "backlight_init registration failed. ret= %d\n", rc);
	}

	return rc;
}

module_init(rt8555_backlight_init);

static void __exit rt8555_backlight_exit(void)
{

	i2c_del_driver(&rt8555_backlight_driver);
}

module_exit(rt8555_backlight_exit);

MODULE_DESCRIPTION("RICHTEK rt8555 based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rt8555-backlight");

