
 /*
  * Z-Wave driver
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/sd3503.h>

struct zwave_platform_data {
	unsigned int reset;
	unsigned int uart_tx;
	unsigned int uart_rx;
	unsigned int zwave_switch;
	u32 reset_flags;
	u32 uart_tx_flags;
	u32 uart_rx_flags;
	u32 zwave_switch_flags;
	struct regulator *zwave_dv_3p3;
};

static struct zwave_platform_data *zwave_pdata;

static int zwave_power(int on, struct platform_device *pdev, struct zwave_platform_data *pdata)
{
	int ret;

	if(on) {
		ret = regulator_enable(pdata->zwave_dv_3p3);
		if(ret)
			return ret;
		dev_info(&pdev->dev, "ZWAVE_DV_3.3V ON\n");
	} else {
		if(regulator_is_enabled(pdata->zwave_dv_3p3)) {
			regulator_disable(pdata->zwave_dv_3p3);
			dev_info(&pdev->dev, "ZWAVE_DV_3.3 OFF\n");
		}
	}

	regulator_put(pdata->zwave_dv_3p3);

	return 0;
}

static long zwave_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case ZWAVE_RESET_HIGH:
		pr_info("ZWAVE RESET_HIGH\n");
		gpio_set_value(zwave_pdata->reset, 1);
		break;
	case ZWAVE_RESET_LOW:
		pr_info("ZWAVE RESET_LOW\n");
		gpio_set_value(zwave_pdata->reset, 0);
		break;
	case ZWAVE_SWITCH_HIGH:
		pr_info("ZWAVE SWITCH_HIGH\n");
		gpio_set_value(zwave_pdata->zwave_switch, 1);
		break;
	case ZWAVE_SWITCH_LOW:
		pr_info("ZWAVE SWITCH_LOW\n");
		gpio_set_value(zwave_pdata->zwave_switch, 0);
		break;
	}
	return ret;
}

static int zwave_parse_dt(struct device *dev, struct zwave_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pr_info("ZWAVE_parse_dt\n");

	pdata->uart_tx = of_get_named_gpio(np, "zwave,tx", 0);

	if (!gpio_is_valid(pdata->uart_tx)) {
		dev_err(dev, "invalied uart_tx\n");
		return -1;
	}

	pdata->uart_rx = of_get_named_gpio(np, "zwave,rx", 0);
	if (!gpio_is_valid(pdata->uart_rx)) {
		dev_err(dev, "invalied uart_rx\n");
		return -1;
	}

	pdata->reset = of_get_named_gpio(np, "zwave,reset", 0);
	if (!gpio_is_valid(pdata->reset)) {
		dev_err(dev, "invalied reset\n");
		return -1;
	}

	pdata->zwave_switch = of_get_named_gpio(np, "zwave,switch", 0);
	if (!gpio_is_valid(pdata->reset)) {
		dev_err(dev, "invalied reset\n");
		return -1;
	}

	return 0;
}

static const struct file_operations zwave_fops = {
	.owner	= THIS_MODULE,
	.unlocked_ioctl	= zwave_ioctl,
};

static struct miscdevice zwave_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= ZWAVE_DRIVER_NAME,
	.fops	= &zwave_fops,
};

static int zwave_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;

	pr_info("ZWAVE_probe enter\n");

	zwave_pdata = devm_kzalloc(dev, sizeof(struct zwave_platform_data), GFP_KERNEL);

	if (!zwave_pdata) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	ret = zwave_parse_dt(dev, zwave_pdata);
	if (ret)
		return ret;

	ret = misc_register(&zwave_miscdev);
	if (ret) {
		dev_err(dev, "failed to register zwave miscdev\n");
		return ret;
	}

	pr_info("ZWAVE_misc register ok\n");
#ifdef ZWAVE_FIRMWARE_DOWNLOAD
	ret = devm_gpio_request(dev, zwave_pdata->uart_tx, "zwave_tx");
	if (ret) {
		dev_err(dev, "failed to get gpio zwave_tx\n");
		return ret;
	}

	ret = devm_gpio_request(dev, zwave_pdata->uart_rx, "zwave_rx");
	if (ret) {
		dev_err(dev, "failed to get gpio zwave_rx\n");
		return ret;
	}
#endif
	ret = devm_gpio_request(dev, zwave_pdata->reset, "zwave_reset");
	if (ret) {
		dev_err(dev, "failed to get gpio zwave_reset\n");
		return ret;
	}

    ret = devm_gpio_request(dev, zwave_pdata->zwave_switch, "zwave_switch");
    if (ret) {
            dev_err(dev, "failed to get gpio zwave_switch\n");
            return ret;
    }

#ifdef ZWAVE_FIRMWARE_DOWNLOAD
	ret = gpio_direction_input(zwave_pdata->uart_tx);
	if (ret) {
		return ret;
	}
	ret = gpio_direction_input(zwave_pdata->uart_rx);
	if (ret) {
		return ret;
	}
#endif

	ret =gpio_direction_output(zwave_pdata->reset, 1);
	if (ret) {
		return ret;
	}

	ret =gpio_direction_output(zwave_pdata->zwave_switch, 0);
	if (ret) {
		return ret;
	}

	zwave_pdata->zwave_dv_3p3 = devm_regulator_get(dev, "ZW_PMIC_3.3V");
	ret = IS_ERR(zwave_pdata->zwave_dv_3p3);
	if (ret) {
		dev_err(dev, "could not get lvs %ld\n", IS_ERR(zwave_pdata->zwave_dv_3p3));
		return ret;
	}

	zwave_power(1, pdev, zwave_pdata);
	pr_info("ZWAVE_probe end\n");

	return 0;
}

static const struct of_device_id zwave_dt_match[] = {
	{.compatible = "sd3503,zwave"},
	{}
};

MODULE_DEVICE_TABLE(of, zwave_dt_match);

static struct platform_driver zwave_driver = {
	.probe 	 = zwave_probe,
	.driver	 = {
		.name	 = "zwave",
		.owner  = THIS_MODULE,
		.of_match_table = zwave_dt_match,
	},
};
module_platform_driver(zwave_driver);
MODULE_AUTHOR("Constant <constant.lee@samsung.com>");
MODULE_DESCRIPTION("Z-wave device driver for SD3503");
MODULE_LICENSE("GPL v2");

