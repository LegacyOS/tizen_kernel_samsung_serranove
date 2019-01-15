/*
 * Bluetooth Broadcom GPIO and Low Power Mode control
 *
 *  Copyright (C) 2011 Samsung Electronics Co., Ltd.
 *  Copyright (C) 2011 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/wakelock.h>

#include <asm/mach-types.h>

#include <mach/gpiomux.h>
#include <linux/of_irq.h>

//#define BT_UART_CFG
#define BT_LPM_ENABLE

#define BASEGPIO 		902

#define BT_UART_RTS 		(BASEGPIO + 3)
#define BT_UART_CTS 		(BASEGPIO + 2)
#define BT_UART_RXD 		(BASEGPIO + 1)
#define BT_UART_TXD 		(BASEGPIO + 0)

#define BT_EN 			(BASEGPIO + 6)
#define BT_WAKE 		(BASEGPIO + 7)
#define BT_HOST_WAKE 		(BASEGPIO + 25)

#define GPIO_BT_UART_RTS 	BT_UART_RTS
#define GPIO_BT_UART_CTS 	BT_UART_CTS
#define GPIO_BT_UART_RXD 	BT_UART_RXD
#define GPIO_BT_UART_TXD 	BT_UART_TXD

#define GPIO_BT_HOST_WAKE 	BT_HOST_WAKE
#define GPIO_BT_WAKE 		BT_WAKE
#define GPIO_BT_EN 		BT_EN
#define GPIO_BT_WAKE 		BT_WAKE

static struct rfkill *bt_rfkill;

int get_gpio_hwrev(int gpio)
{
	return gpio;
}

#if defined(BT_UART_CFG) && defined(CONFIG_GPIO_MSM_V3)
static unsigned bt_uart_on_table[] = {
	GPIO_CFG(GPIO_BT_UART_RTS, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_8MA),
	GPIO_CFG(GPIO_BT_UART_CTS, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_8MA),
	GPIO_CFG(GPIO_BT_UART_RXD, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_8MA),
	GPIO_CFG(GPIO_BT_UART_TXD, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_8MA)
};

static unsigned bt_uart_off_table[] = {
	GPIO_CFG(GPIO_BT_UART_RTS, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_8MA),
	GPIO_CFG(GPIO_BT_UART_CTS, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_8MA),
	GPIO_CFG(GPIO_BT_UART_RXD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_8MA),
	GPIO_CFG(GPIO_BT_UART_TXD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_8MA)
};
#endif

#if defined(BT_LPM_ENABLE) && defined(CONFIG_GPIO_MSM_V3)
static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= -1, //GPIO_BT_HOST_WAKE,
		.end	= -1, //GPIO_BT_HOST_WAKE,
		.flags	= IORESOURCE_IO
	},
	{
		.name	= "gpio_ext_wake",
		.start	= -1, //FPGA_GPIO_BT_WAKE,
		.end	= -1, //FPGA_GPIO_BT_WAKE,
		.flags	= IORESOURCE_IO
	},
	{
		.name	= "host_wake",
		.start	= -1, //44,
		.end	= -1, //44,
		.flags	= IORESOURCE_IRQ
	}
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources
};

//static int gpio_rev_init(struct device *dev)
static void gpio_rev_init(void)
{

	bluesleep_resources[0].start = get_gpio_hwrev(GPIO_BT_HOST_WAKE);
	bluesleep_resources[0].end = get_gpio_hwrev(GPIO_BT_HOST_WAKE);
	bluesleep_resources[1].start = GPIO_BT_WAKE;
	bluesleep_resources[1].end = GPIO_BT_WAKE;
	bluesleep_resources[2].start = gpio_to_irq(GPIO_BT_HOST_WAKE);
	bluesleep_resources[2].end = gpio_to_irq(GPIO_BT_HOST_WAKE);

}
#endif

static int bcm4334_bt_rfkill_set_power(void *data, bool blocked)
{
	/* rfkill_ops callback. Turn transmitter on when blocked is false */
#if defined(BT_UART_CFG) && defined(CONFIG_GPIO_MSM_V3)
	int pin, rc = 0;
#endif
	if (!blocked) {
		pr_err("[BT] Bluetooth Power On.\n");
		gpio_direction_input(GPIO_BT_HOST_WAKE);
		if (gpio_direction_output(GPIO_BT_EN, 1))
			pr_err("[BT] failed to set BT_EN.\n");
		gpio_set_value(get_gpio_hwrev(GPIO_BT_WAKE), 1);
#if defined(BT_UART_CFG) && defined(CONFIG_GPIO_MSM_V3)
		for (pin = 0; pin < ARRAY_SIZE(bt_uart_on_table); pin++) {
			rc = gpio_tlmm_config(bt_uart_on_table[pin],
			      GPIO_CFG_ENABLE);
			if (rc < 0)
				pr_err("[BT] %s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_uart_on_table[pin], rc);
		}
#endif
	} else {
#if defined(BT_UART_CFG) && defined(CONFIG_GPIO_MSM_V3)
		for (pin = 0; pin < ARRAY_SIZE(bt_uart_off_table); pin++) {
			rc = gpio_tlmm_config(bt_uart_off_table[pin],
					      GPIO_CFG_ENABLE);
		if (rc < 0)
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				  __func__, bt_uart_off_table[pin], rc);
	}
#endif
		pr_err("[BT] Bluetooth Power Off.\n");
		gpio_direction_output(GPIO_BT_EN, 0);
		gpio_set_value(get_gpio_hwrev(GPIO_BT_WAKE), 0);
	}
	return 0;
}

static const struct rfkill_ops bcm4334_bt_rfkill_ops = {
	.set_block = bcm4334_bt_rfkill_set_power
};

static int bcm4334_bluetooth_probe(struct platform_device *pdev)
{
	int rc = 0;
#if defined(BT_UART_CFG) && defined(CONFIG_GPIO_MSM_V3)
	int pin = 0;
#endif
	rc = gpio_request(GPIO_BT_EN, "bcm_bt_en_gpio");
	if(rc) {
		pr_err("[BT] GPIO_BT_EN request failed.\n");
		return rc;
	}
	gpio_direction_output(get_gpio_hwrev(GPIO_BT_EN), 0);
	/* temporailiy set HOST_WAKE OUT direction until FPGA work finishs */
	/* if setting HOST_WAKE to NO PULL, BT would not be turned on. */
	/* By guideline of BRCM, it is needed to determine pull status */
#if !defined(BT_LPM_ENABLE) && defined(CONFIG_GPIO_MSM_V3)
	gpio_tlmm_config(GPIO_CFG(get_gpio_hwrev(GPIO_BT_HOST_WAKE), 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	gpio_set_value(get_gpio_hwrev(GPIO_BT_HOST_WAKE), 1);
#endif
#if defined(BT_UART_CFG) && defined(CONFIG_GPIO_MSM_V3)
	for (pin = 0; pin < ARRAY_SIZE(bt_uart_off_table); pin++) {
		rc = gpio_tlmm_config(bt_uart_off_table[pin], GPIO_CFG_ENABLE);
		if (rc < 0)
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, bt_uart_off_table[pin], rc);
	}
#endif
	rc = gpio_request(GPIO_BT_WAKE, "bcm_bt_wake_gpio");
	if(rc) {
		pr_err("[BT] GPIO_BT_WAKE request failed.\n");
		gpio_free(get_gpio_hwrev(BT_EN));
		return rc;
	}
	gpio_direction_output(get_gpio_hwrev(GPIO_BT_WAKE), 0);
	bt_rfkill = rfkill_alloc("bcm4334_bluetooth", &pdev->dev,
                RFKILL_TYPE_BLUETOOTH, &bcm4334_bt_rfkill_ops,
                NULL);
	if (unlikely(!bt_rfkill)) {
		pr_err("[BT] bt_rfkill alloc failed.\n");
		gpio_free(get_gpio_hwrev(GPIO_BT_WAKE));
		gpio_free(get_gpio_hwrev(BT_EN));
		return -ENOMEM;
	}
	rfkill_init_sw_state(bt_rfkill, 0);
	rc = rfkill_register(bt_rfkill);
	if (unlikely(rc)) {
		pr_err("[BT] bt_rfkill register failed.\n");
		rfkill_destroy(bt_rfkill);
		gpio_free(get_gpio_hwrev(BT_WAKE));
		gpio_free(get_gpio_hwrev(BT_EN));
		return rc;
	}
	rfkill_set_sw_state(bt_rfkill, true);
	return rc;
}

static int bcm4334_bluetooth_remove(struct platform_device *pdev)
{
	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);

	gpio_free(GPIO_BT_HOST_WAKE);
	gpio_free(GPIO_BT_WAKE);
	gpio_free(GPIO_BT_EN);

	return 0;
}

static struct platform_device bcm4334_bluetooth_platform_device = {
	.name		= "bcm4334_bluetooth",
	.id		= -1
};

static struct platform_driver bcm4334_bluetooth_platform_driver = {
	.probe = bcm4334_bluetooth_probe,
	.remove = bcm4334_bluetooth_remove,
	.driver = {
		   .name = "bcm4334_bluetooth",
		   .owner = THIS_MODULE
		   }
};

static int __init bcm4334_bluetooth_init(void)
{
	int ret = 0;
	ret = platform_device_register(&bcm4334_bluetooth_platform_device);
	if(ret) {
		pr_err("[BT]: register platform device failed \n");
		return ret;
	}
#if defined(BT_LPM_ENABLE) && defined(CONFIG_GPIO_MSM_V3)
	gpio_rev_init();
	if(platform_device_register(&msm_bluesleep_device))
		pr_err("[BT]: register platform device for LMP mode failed \n");
#endif
	ret = platform_driver_register(&bcm4334_bluetooth_platform_driver);
	if(ret)
		pr_err("[BT]: register platform driver failed \n");
	return ret;
}

static void __exit bcm4334_bluetooth_exit(void)
{
	platform_device_unregister(&bcm4334_bluetooth_platform_device);
#if defined(BT_LPM_ENABLE) && defined(CONFIG_GPIO_MSM_V3)
	platform_device_unregister(&msm_bluesleep_device);
#endif
	platform_driver_unregister(&bcm4334_bluetooth_platform_driver);
}

module_init(bcm4334_bluetooth_init);
module_exit(bcm4334_bluetooth_exit);

MODULE_ALIAS("platform:bcm4334");
MODULE_DESCRIPTION("bcm4334_bluetooth");
MODULE_LICENSE("GPL");
