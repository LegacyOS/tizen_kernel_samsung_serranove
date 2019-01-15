
 /*
  * ZigBee driver
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

struct zigbee_platform_data {
	unsigned int reset;
	u32 reset_flags;
	struct regulator *zb_dv_3p3;
};

#define ZIGBEE_POWER_SAME_WITH_ZWAVE

#ifndef ZIGBEE_POWER_SAME_WITH_ZWAVE
static int zigbee_power(int on, struct platform_device *pdev)
{
	int ret;
	struct zigbee_platform_data *pdata = platform_get_drvdata(pdev);

	if(on) {
		ret = regulator_enable(pdata->zb_dv_3p3);
		if(ret)
			return ret;
		dev_info(&pdev->dev, "ZigBee_DV_3.3V ON\n");
	} else {
		if(regulator_is_enabled(pdata->zb_dv_3p3)) {
			regulator_disable(pdata->zb_dv_3p3);
			dev_info(&pdev->dev, "ZigBee_DV_3.3 OFF\n");
		}
	}

	regulator_put(pdata->zb_dv_3p3);

	return 0;
}
#endif

static int zigbee_parse_dt(struct device *dev, struct zigbee_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pr_info("ZigBee_parse_dt\n");

	pdata->reset = of_get_named_gpio(np, "zigbee,reset", 0);
	if (!gpio_is_valid(pdata->reset)) {
		dev_err(dev, "invalied reset\n");
		return -1;
	}

	return 0;
}

static int zigbee_probe(struct platform_device *pdev)
{

	struct device *dev = &pdev->dev;
	struct zigbee_platform_data *pdata;
	int ret = 0;

	pr_info("ZigBee_probe enter\n");

	pdata = devm_kzalloc(dev, sizeof(struct zigbee_platform_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	ret = zigbee_parse_dt(dev, pdata);
	if (ret)
		return ret;
	ret = devm_gpio_request(dev, pdata->reset, "zigbee_reset");
	if (ret) {
		dev_err(dev, "failed to get gpio reset\n");
		return ret;
	}

	ret = gpio_direction_output(pdata->reset, 1);
	if (ret)
		return ret;

#ifndef ZIGBEE_POWER_SAME_WITH_ZWAVE
	pdata->zb_dv_3p3 = devm_regulator_get(dev, "ZB_PMIC_3.3V");
	if (IS_ERR(pdata->zb_dv_3p3)) {
		dev_err(dev, "could not get lvs %ld\n", PTR_ERR(pdata->zb_dv_3p3));
		return PTR_ERR(pdata->zb_dv_3p3);
	}
#endif

	platform_set_drvdata(pdev, pdata);

#ifndef ZIGBEE_POWER_SAME_WITH_ZWAVE
	zigbee_power(1, pdev);
#endif
	pr_info("ZigBee_probe end\n");

	return 0;
}

static const struct of_device_id zigbee_dt_match[] = {
	{.compatible = "em357,zigbee"},
	{}
};

MODULE_DEVICE_TABLE(of, zigbee_dt_match);

static struct platform_driver zigbee_driver = {
	.probe 	 = zigbee_probe,
	.driver	 = {
		.name	 = "zigbee",
		.owner  = THIS_MODULE,
		.of_match_table = zigbee_dt_match,
	},
};
module_platform_driver(zigbee_driver);
MODULE_AUTHOR("Constant <constant.lee@samsung.com>");
MODULE_DESCRIPTION("ZigBee device driver for EM357");
MODULE_LICENSE("GPL v2");

