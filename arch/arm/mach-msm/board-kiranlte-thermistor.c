/* board-kiran-thermistor.c
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/sec_thermistor.h>
#include <linux/qpnp/qpnp-adc.h>

static struct sec_therm_adc_table temp_table_ap[] = {
	/* adc, temperature */
	{25950,	900},
	{25995,	890},
	{26039,	880},
	{26084,	870},
	{26128,	860},
	{26173,	850},
	{26201,	840},
	{26230,	830},
	{26258,	820},
	{26287,	810},
	{26315,	800},
	{26401,	790},
	{26488,	780},
	{26574,	770},
	{26661,	760},
	{26747,	750},
	{26821,	740},
	{26895,	730},
	{26969,	720},
	{27043,	710},
	{27117,	700},
	{27200,	690},
	{27284,	680},
	{27367,	670},
	{27450,	660},
	{27534,	650},
	{27641,	640},
	{27749,	630},
	{27857,	620},
	{27965,	610},
	{28073,	600},
	{28171,	590},
	{28270,	580},
	{28369,	570},
	{28468,	560},
	{28567,	550},
	{28698,	540},
	{28830,	530},
	{28962,	520},
	{29094,	510},
	{29225,	500},
	{29380,	490},
	{29535,	480},
	{29690,	470},
	{29884,	460},
	{29999,	450},
	{30184,	440},
	{30369,	430},
	{30553,	420},
	{30738,	410},
	{30923,	400},
	{31104,	390},
	{31284,	380},
	{31465,	370},
	{31646,	360},
	{31827,	350},
	{32024,	340},
	{32221,	330},
	{32418,	320},
	{32616,	310},
	{32813,	300},
	{33017,	290},
	{33221,	280},
	{33424,	270},
	{33628,	260},
	{33832,	250},
	{34001,	240},
	{34252,	230},
	{34502,	220},
	{34753,	210},
	{34949,	200},
	{35168,	190},
	{35387,	180},
	{35607,	170},
	{35826,	160},
	{36045,	150},
	{36294,	140},
	{36543,	130},
	{36793,	120},
	{37042,	110},
	{37291,	100},
	{37451,	90},
	{37612,	80},
	{37772,	70},
	{37933,	60},
	{38093,	50},
	{38262,	40},
	{38431,	30},
	{38599,	20},
	{38768,	10},
	{38937,	0},
	{39100,	-10},
	{39263,	-20},
	{39426,	-30},
	{39590,	-40},
	{39753,	-50},
	{39888,	-60},
	{40022,	-70},
	{40157,	-80},
	{40291,	-90},
	{40426,	-100},
	{40549,	-110},
	{40673,	-120},
	{40796,	-130},
	{40920,	-140},
	{41043,	-150},
	{41141,	-160},
	{41240,	-170},
	{41338,	-180},
	{41436,	-190},
	{41535,	-200},
};

static struct sec_therm_adc_table temp_table_battery[] = {
	/* adc, temperature */
	{27281, 700},
	{27669, 650},
	{28178, 600},
	{28724, 550},
	{29342, 500},
	{30101, 450},
	{30912, 400},
	{31807, 350},
	{32823, 300},
	{33858, 250},
	{34950, 200},
	{36049, 150},
	{37054, 100},
	{38025, 50},
	{38239, 40},
	{38453, 30},
	{38667, 20},
	{38881, 10},
	{39096, 0},
	{39269, -10},
	{39442, -30},
	{39615, -40},
	{39965, -50},
	{40706, -100},
	{41223, -150},
	{41719, -200},
};

static struct sec_therm_adc_table temp_table_pam0[] = {
	/* adc, temperature */
	{27281, 700},
	{27669, 650},
	{28178, 600},
	{28724, 550},
	{29342, 500},
	{30101, 450},
	{30912, 400},
	{31807, 350},
	{32823, 300},
	{33858, 250},
	{34950, 200},
	{36049, 150},
	{37054, 100},
	{38025, 50},
	{38239, 40},
	{38453, 30},
	{38667, 20},
	{38881, 10},
	{39096, 0},
	{39269, -10},
	{39442, -30},
	{39615, -40},
	{39965, -50},
	{40706, -100},
	{41223, -150},
	{41719, -200},
};

static struct sec_therm_adc_table temp_table_default[] = {
	/* adc, temperature */
	{25950, 900},
	{26108, 850},
	{26515, 800},
	{26810, 750},
	{27203, 700},
	{27590, 650},
	{28085, 600},
	{28679, 550},
	{29343, 500},
	{30099, 450},
	{30962, 400},
	{31874, 350},
	{32865, 300},
	{33886, 250},
	{34996, 200},
	{36145, 150},
	{37148, 100},
	{38164, 50},
	{38432, 20},
	{38890, 0},
	{39156, -20},
	{39420, -40},
	{39650, -50},
	{40606, -100},
	{41393, -150},
	{41890, -200},
};

struct sec_therm_adc_info kiran_adc_list[] = {
	{
		.therm_id = SEC_THERM_AP,
		.name = "ap_therm",
		.adc_name = "MPP4",
		.adc_ch = P_MUX4_1_1,
	},
	{
		.therm_id = SEC_THERM_BATTERY,
		.name = "batt_therm",
		.adc_name = "BAT_THM",
		.adc_ch = LR_MUX1_BATT_THERM,
	},
};

static int sec_therm_temp_table_init(struct sec_therm_adc_info *adc_info)
{
	if (unlikely(!adc_info))
		return -EINVAL;

	switch (adc_info->therm_id) {
		case SEC_THERM_AP:
			adc_info->temp_table = temp_table_ap;
			adc_info->temp_table_size = ARRAY_SIZE(temp_table_ap);
			break;
		case SEC_THERM_BATTERY:
			adc_info->temp_table = temp_table_battery;
			adc_info->temp_table_size = ARRAY_SIZE(temp_table_battery);
			break;

		case SEC_THERM_PAM0:
			adc_info->temp_table = temp_table_pam0;
			adc_info->temp_table_size = ARRAY_SIZE(temp_table_pam0);
			break;

		case SEC_THERM_XO:
			adc_info->temp_table = temp_table_default;
			adc_info->temp_table_size = ARRAY_SIZE(temp_table_default);
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static int sec_therm_parse_dt(struct device_node *node,
			struct sec_therm_adc_info *adc_list)
{
	struct device_node *child = NULL;
	int i = 0, ret;

	for_each_child_of_node(node, child) {
		int therm_id, therm_adc_ch;
		const char *therm_name, *therm_adc_name;

		therm_name = child->name;
		if (!therm_name) {
			pr_err("%s: Failed to get thermistor name\n", __func__);
			return -EINVAL;
		}

		ret = of_property_read_u32(child, "sec,therm-id", &therm_id);
		if (ret) {
			pr_err("%s: Failed to get thermistor id\n", __func__);
			return ret;
		}

		therm_adc_name = of_get_property(child, "sec,therm-adc-name", NULL);
		if (!therm_adc_name) {
			pr_err("%s: Failed to get adc name\n", __func__);
			return -EINVAL;
		}

		ret = of_property_read_u32(child, "sec,therm-adc-ch", &therm_adc_ch);
		if (ret) {
			pr_err("%s: Failed to get thermistor adc channel\n", __func__);
			return ret;
		}

		pr_info("%s: name:%s, therm_id:%d, adc_name:%s, adc_ch:0x%x\n",
				__func__, therm_name, therm_id, therm_adc_name, therm_adc_ch);

		adc_list[i].name = therm_name;
		adc_list[i].therm_id = therm_id;
		adc_list[i].adc_name = therm_adc_name;
		adc_list[i].adc_ch = therm_adc_ch;
		i++;
	}

	return 0;
}

int sec_therm_adc_read(struct sec_therm_info *info, int therm_id, int *val)
{
	struct sec_therm_adc_info *adc_info = NULL;
	struct qpnp_vadc_result result;
	int i, ret = 0;

	if (unlikely(!info || !val))
		return -EINVAL;

	for (i = 0; i < info->adc_list_size; i++) {
		if (therm_id == info->adc_list[i].therm_id) {
			adc_info = &info->adc_list[i];
			break;
		}
	}

	if (!adc_info) {
		pr_err("%s: Failed to found therm_id %d\n", __func__, therm_id);
		return -EINVAL;
	}

	ret = qpnp_vadc_read(adc_info->adc_client, adc_info->adc_ch, &result);
	if (ret) {
		pr_err("%s: Failed to read adc channel 0x%02x (%d)\n",
				__func__, adc_info->adc_ch, ret);
		return -EINVAL;
	}

	*val = result.adc_code;
	return 0;
}

int sec_therm_adc_init(struct platform_device *pdev)
{
	struct sec_therm_info *info = platform_get_drvdata(pdev);
	struct sec_therm_adc_info *adc_list = NULL;
	int adc_list_size = 0;
	int i, ret = 0;

	/* device tree support */
	if (pdev->dev.of_node) {
		struct device_node *node = pdev->dev.of_node;
		struct device_node *child;

		for_each_child_of_node(node, child)
			adc_list_size++;

		if (adc_list_size <= 0) {
			pr_err("%s: No adc channel info\n", __func__);
			return -ENODEV;
		}

		adc_list = devm_kzalloc(&pdev->dev,
				sizeof(struct sec_therm_adc_info) * adc_list_size, GFP_KERNEL);
		if (!adc_list) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}

		ret = sec_therm_parse_dt(node, adc_list);
		if (ret) {
			pr_err("%s: Failed to parse dt (%d)\n", __func__, ret);
			goto err;
		}
	} else {
		adc_list = kiran_adc_list;
		adc_list_size = ARRAY_SIZE(kiran_adc_list);

		for (i = 0; i < adc_list_size; i++) {
			pr_info("%s: name:%s, therm_id:%d, adc_name:%s, adc_ch:0x%x\n",
					__func__,
					adc_list[i].name, adc_list[i].therm_id,
					adc_list[i].adc_name, adc_list[i].adc_ch);
		}
	}

	for (i = 0; i < adc_list_size; i++) {
		ret = sec_therm_temp_table_init(&adc_list[i]);
		if (ret) {
			pr_err("%s: Failed to init %d adc_temp_table\n",
					__func__, adc_list[i].therm_id);
			goto err;
		}

		adc_list[i].adc_client = qpnp_get_vadc(info->dev, adc_list[i].adc_name);
		if (IS_ERR_OR_NULL(adc_list[i].adc_client)) {
			pr_err("%s: Failed to get %d vadc (%ld)\n", __func__,
					adc_list[i].therm_id, PTR_ERR(adc_list[i].adc_client));
			goto err;
		}
	}

	info->adc_list = adc_list;
	info->adc_list_size = adc_list_size;

	return 0;

err:
	devm_kfree(&pdev->dev, adc_list);
	return ret;
}

void sec_therm_adc_exit(struct platform_device *pdev)
{
	struct sec_therm_info *info = platform_get_drvdata(pdev);

	if (!info)
		return;

	if (pdev->dev.of_node && info->adc_list)
		devm_kfree(&pdev->dev, info->adc_list);
}
