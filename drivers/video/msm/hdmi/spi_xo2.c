/*
 * Driver for LATTICE XO2 SPI Controllers
 *
 * Copyright (C) 2015
 *
 * This driver is inspired by:
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of_gpio.h>

#include "spi_xo2_reg.h"

#define DRV_NAME	"spi-xo2"
#define DEBUG_XO2

/* SPI command */
unsigned char reset_cmd[] = {0x79, 0x00, 0x00, 0x00};
unsigned char enable_prgmmode_cmd[] = {0xC6, 0x00, 0x00, 0x00};
unsigned char erase_sram_cmd[] = {0x0E, 0x01, 0x00, 0x00};
unsigned char configure_sram_cmd[] = {0x7a, 0x00, 0x00, 0x00};
unsigned char exit_prgmmode_cmd[] = {0x26, 0x00, 0x00};
unsigned char checkID_cmd[] = {0xE0, 0x00, 0x00, 0x00};
unsigned char read_sram_status_cmd[] = {0x3C, 0x00, 0x00, 0x00};
unsigned char noop_cmd[] = {0xff, 0xff, 0xff, 0xff};
struct spi_device *spi_xo2;
uint32_t spi_readval;
static unsigned int GPIO_XO2_SPI_SN;

static int xo2_gpio_setting(void)
{
	int ret = 0;

	ret = gpio_request_one(GPIO_XO2_SPI_SN, GPIOF_OUT_INIT_HIGH, "SPI_CS");
	if (ret)
		pr_err("failed to request GPIO_XO2_SPI_SN\n");

	return ret;
}

static void xo2spi_write_block(unsigned char *p_buf, int size)
{
	int k = 0;
	unsigned char val = 0;
	int error = 0;

	for (k = 0; k < size; k++) {
		val = p_buf[k];
		error = spi_write(spi_xo2, &val, 1);
		if (unlikely(error))
			pr_err("SPI write error: %d\n", error);
	}
}

static void xo2spi_read_block(unsigned char *p_buf,
		int size, uint32_t *p_readval)
{
	int i = 0;
	unsigned char rxbuf = 0;
	uint32_t result = 0;
	int error = 0;

	xo2spi_write_block(p_buf, size);
	for (i = 0; i < 4; i++) {
		result = result << 8;
		error = spi_read(spi_xo2, &rxbuf, 1);
		if (unlikely(error))
			pr_err("SPI read error: %d\n", error);
		result = result + rxbuf;
	}
	*p_readval = result;  /* for 32bit data return*/
}

int fw_data_update(void)
{
	int result = 0;


	while(1) {
	gpio_direction_output(GPIO_XO2_SPI_SN, 0);
	usleep_range(10000, 11000);
	xo2spi_read_block(checkID_cmd,
			sizeof(checkID_cmd) / sizeof(unsigned char),
			&spi_readval);
	gpio_direction_output(GPIO_XO2_SPI_SN, 1);

	msleep(100);
	/*LCMXO2-2000HE ID : 0x012B3043*/
	pr_info("xo2 read id : 0x%x\n", spi_readval);
	if (spi_readval == 0x012B3043)
		break;
	}
	/*reset the device*/
	pr_info("xo2 reset the device\n");
	gpio_direction_output(GPIO_XO2_SPI_SN, 0);
	xo2spi_write_block(reset_cmd,
			sizeof(reset_cmd) / sizeof(unsigned char));
	gpio_direction_output(GPIO_XO2_SPI_SN, 1);

	/* enable SRAM programming*/
	gpio_direction_output(GPIO_XO2_SPI_SN, 0);
	pr_info("xo2 enable SRAM Programming\n");
	xo2spi_write_block(enable_prgmmode_cmd,
			sizeof(enable_prgmmode_cmd) / sizeof(unsigned char));
	gpio_direction_output(GPIO_XO2_SPI_SN, 1);

	/* Erase SRAM*/
	gpio_direction_output(GPIO_XO2_SPI_SN, 0);
	pr_info("xo2 Erase SRAM\n");
	xo2spi_write_block(erase_sram_cmd,
			sizeof(erase_sram_cmd) / sizeof(unsigned char));
	gpio_direction_output(GPIO_XO2_SPI_SN, 1);
	msleep(200);

	/* read SRAM status register*/
	gpio_direction_output(GPIO_XO2_SPI_SN, 0);
	xo2spi_read_block(read_sram_status_cmd,
			sizeof(read_sram_status_cmd) / sizeof(unsigned char),
			&spi_readval);
	pr_info("xo2 After erase SRAM - SRAM status  : 0x%x\n", spi_readval);
	gpio_direction_output(GPIO_XO2_SPI_SN, 1);

#ifdef DEBUG_XO2
	if (spi_readval & 0x3100) {
		/* 13bit 12bit 8bit check*/
		pr_err("xo2 SRAM ERASE FAIL  : 0x%x\n", spi_readval);
		msleep(200);
	} else
		pr_info("xo2 SRAM ERASE SUCCESS  : 0x%x\n", spi_readval);

#endif

	/* configure SRAM*/
	pr_info("xo2 Configure SRAM\n");
	gpio_direction_output(GPIO_XO2_SPI_SN, 0);
	xo2spi_write_block(configure_sram_cmd,
			sizeof(configure_sram_cmd) / sizeof(unsigned char));
	pr_info("xo2 Write data\n");
	xo2spi_write_block(spiword, sizeof(spiword) / sizeof(unsigned char));
	gpio_direction_output(GPIO_XO2_SPI_SN, 1);

	/* read SRAM status register*/
	gpio_direction_output(GPIO_XO2_SPI_SN, 0);
	xo2spi_read_block(read_sram_status_cmd,
			sizeof(read_sram_status_cmd) / sizeof(unsigned char),
			&spi_readval);
	pr_info("xo2 After configure SRAM - SRAM status  : 0x%x\n",
			spi_readval);
	gpio_direction_output(GPIO_XO2_SPI_SN, 1);

#ifdef DEBUG_XO2
	if (spi_readval & 0x100)
		pr_info("xo2 configure SRAM  SUCCESS  : 0x%x\n", spi_readval);
	else {
		pr_err("xo2 configure SRAM  FAIL  : 0x%x\n", spi_readval);
		return -EBUSY;
	}
#endif

	/* exit Programming mode*/
	pr_info("xo2 Exit programming mode\n");
	gpio_direction_output(GPIO_XO2_SPI_SN, 0);
	xo2spi_write_block(exit_prgmmode_cmd,
			sizeof(exit_prgmmode_cmd) / sizeof(unsigned char));
	gpio_direction_output(GPIO_XO2_SPI_SN, 1);
	gpio_direction_output(GPIO_XO2_SPI_SN, 0);
	xo2spi_write_block(noop_cmd, sizeof(noop_cmd) / sizeof(unsigned char));
	gpio_direction_output(GPIO_XO2_SPI_SN, 1);
	gpio_direction_output(GPIO_XO2_SPI_SN, 0);

	return result;
}

#ifdef CONFIG_OF
static int xo_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;

	/* gpio pins */
	GPIO_XO2_SPI_SN = of_get_named_gpio_flags(np, "cs-gpio",
		0, &flags);
	if (GPIO_XO2_SPI_SN < 0) {
		pr_err("[XO2]error for sn gpio\n");
		return GPIO_XO2_SPI_SN;
	}

	return 0;
}
#endif
static int xo2spi_probe(struct spi_device *spi)
{
	int ret;

	spi_xo2 = spi;
	pr_info("xo2_spi_probe() spi_xo2 : 0x%x\n", (unsigned int)spi_xo2);
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if (ret < 0) {
		pr_err("spi_setup() fail ret : %d\n", ret);
		return ret;
	}

	if (spi->dev.of_node)
		xo_parse_dt(&spi->dev);

	ret = xo2_gpio_setting();
	if (ret != 0) {
		pr_err("xo2_gpio_setting() fail\n");
		return ret;
	}

	fw_data_update();
	return 0;
}

static int xo2spi_remove(struct spi_device *spi)
{
	return 0;
}

unsigned long xo2_get_if_handle(void)
{
	pr_info("%s : spi_xo2 0x%p\n", __func__, spi_xo2);
	return (unsigned long)spi_xo2;
}
EXPORT_SYMBOL_GPL(xo2_get_if_handle);

static int xo2_suspend(struct device *dev)
{
	pr_info("%s\n", __func__);

	return 0;
}

static int xo2_resume(struct device *dev)
{

	fw_data_update();
	pr_info("%s\n", __func__);

	return 0;
}

static const struct dev_pm_ops xo2_pm_ops = {
	.suspend = xo2_suspend,
	.resume = xo2_resume
};

static const struct spi_device_id xo2_id[] = {
	{"xo2_spi", 0},
	{}
};
#ifdef CONFIG_OF
static struct of_device_id xo2spi_table[] = {
	{ .compatible = "lattice,xo2", },
	{ },
};
#else
#define xo2spi_idtable NULL
#endif
MODULE_DEVICE_TABLE(spi, ssp_id);

static struct spi_driver xo2spi_driver = {
	.id_table	= xo2_id,
	.probe	= xo2spi_probe,
	.remove	= xo2spi_remove,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "xo2_spi",
		.pm = &xo2_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = xo2spi_table
#endif
	},

};
static int __init xo2_spi_init(void)
{
	return spi_register_driver(&xo2spi_driver);
}

static void __exit xo2_spi_exit(void)
{
	spi_unregister_driver(&xo2spi_driver);
}

module_init(xo2_spi_init);
module_exit(xo2_spi_exit);
MODULE_AUTHOR("Lattice");
MODULE_DESCRIPTION("XO2 SPI Driver");
MODULE_LICENSE("GPL v2");
