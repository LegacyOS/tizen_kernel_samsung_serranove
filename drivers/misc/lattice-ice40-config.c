/*
 * Copyright (C) 2014 Samsung Electronics
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#define GPIO_LEVEL_LOW	0
#define GPIO_LEVEL_HIGH	1

struct lattice_ice40_pdata {
	const char *fw_name;
	int spi_sclk;
	int spi_miso;
	int spi_mosi;
	int spi_cs;
	int config_done;
	int config_reset;
	int fpga_en;
	struct device *dev;
	const struct firmware *firmware;
#ifdef USE_WORKQUEUE_WIP
	struct workqueue_struct *fw_download_wq;
	struct delayed_work	fw_dl_work;
#endif
	struct dentry *debug_root;
	long debug_mode;
};
static int setup_gpio(struct lattice_ice40_pdata *pdata,
					bool debug_mode, bool init_mode);

static int lattice_debug_open(struct inode *inode, struct file *file)
{
	struct lattice_ice40_pdata *pdata = NULL;

	file->f_mode &= ~FMODE_LSEEK;
	file->private_data = inode->i_private;

	pdata = (struct lattice_ice40_pdata *) file->private_data;
	dev_dbg(pdata->dev, "%s\n", __func__);
	return 0;
}

static int lattice_debug_release(struct inode *inode, struct file *file)
{
	struct lattice_ice40_pdata *pdata = NULL;
	pdata = (struct lattice_ice40_pdata *) file->private_data;
	dev_dbg(pdata->dev, "%s\n", __func__);
	return 0;
}

static int lattice_debug_read(struct file *file, char __user *buff,
					size_t count, loff_t *ppos)
{
	int len;
	char buf[16];
	struct lattice_ice40_pdata *pdata = file->private_data;

	dev_dbg(pdata->dev, "%s\n", __func__);

	if (!pdata)
		return -ENODEV;

	if (*ppos)
		return 0; /* end */

	len = snprintf(buf, sizeof(buf), "%ld\n", pdata->debug_mode);
	if (len < 0)
		return 0;

	if (copy_to_user(buff, buf, len))
		return -EFAULT;

	*ppos += len;

	return len;
}

static int lattice_debug_write(struct file *file, const char __user *buff,
					size_t count, loff_t *ppos)
{
	char buf[16];
	struct lattice_ice40_pdata *pdata = file->private_data;

	dev_dbg(pdata->dev, "%s\n", __func__);

	if (!pdata)
		return -ENODEV;

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, buff, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtol(buf, 0, &pdata->debug_mode) < 0) {
		dev_err(pdata->dev, "%s(): value convert err\n", __func__);
		return -EFAULT;
	} else {
		dev_info(pdata->dev, "%s: %ld\n", __func__, pdata->debug_mode);
	}

	if (pdata->debug_mode) {
		/* make all config gpio pins as input */
		setup_gpio(pdata, true, false); /* debug, non-init mode */
	} else {
		/* make config pins as norminal */
		setup_gpio(pdata, false, false); /* non-debug, non-init mode */
	}
	return count;
}

static const struct file_operations lattice_debug_fops = {
	.open = lattice_debug_open,
	.release = lattice_debug_release,
	.read = lattice_debug_read,
	.write = lattice_debug_write,
};

struct lattice_ice40_pdata * __init lattice_ice40_dt_to_pdata(
						struct platform_device *pdev)
{
	int ret;
	struct lattice_ice40_pdata *pdata = NULL;
	struct device_node *node = pdev->dev.of_node;

	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct lattice_ice40_pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "%s: no memory\n", __func__);
		return NULL;
	} else {
		pdata->spi_sclk = of_get_named_gpio(node,
						"lattice,spi-sclk-gpio", 0);
		if (pdata->spi_sclk < 0)
			dev_err(&pdev->dev, "Can't get spi sclk gpio\n");

		pdata->spi_miso = of_get_named_gpio(node,
						"lattice,spi-miso-gpio", 0);
		if (pdata->spi_miso < 0)
			dev_err(&pdev->dev, "Can't get spi miso gpio\n");

		pdata->spi_mosi = of_get_named_gpio(node,
						"lattice,spi-mosi-gpio", 0);
		if (pdata->spi_mosi < 0)
			dev_err(&pdev->dev, "Can't get spi mosi gpio\n");

		pdata->spi_cs = of_get_named_gpio(node,
						"lattice,spi-cs-gpio", 0);
		if (pdata->spi_cs < 0)
			dev_err(&pdev->dev, "Can't get spi cs gpio\n");

		pdata->config_done = of_get_named_gpio(node,
						"lattice,cdone-gpio", 0);
		if (pdata->config_done < 0)
			dev_err(&pdev->dev, "Can't get config done gpio\n");

		pdata->config_reset = of_get_named_gpio(node,
						"lattice,creset-gpio", 0);
		if (pdata->config_reset < 0)
			dev_err(&pdev->dev, "Can't get config reset gpio\n");

		pdata->fpga_en = of_get_named_gpio(node,
						"lattice,fpga-en-gpio", 0);
		if (pdata->fpga_en < 0)
			dev_err(&pdev->dev, "Can't get fgpa en gpio\n");

		ret = of_property_read_string(node,
					"lattice,firmware", &pdata->fw_name);
		if (ret < 0)
			dev_err(&pdev->dev, "Can't get fw_name\n");

		pdata->dev = &pdev->dev;
	}
	return pdata;
}

static void dump_pdata(struct lattice_ice40_pdata *pdata)
{
	dev_info(pdata->dev, "firmware [%s]\n", pdata->fw_name);
	dev_info(pdata->dev, "spi sclk [%d]\n", pdata->spi_sclk);
	dev_info(pdata->dev, "spi miso [%d]\n", pdata->spi_miso);
	dev_info(pdata->dev, "spi mosi [%d]\n", pdata->spi_mosi);
	dev_info(pdata->dev, "spi cs [%d]\n", pdata->spi_cs);
	dev_info(pdata->dev, "config reset [%d]\n", pdata->config_reset);
	dev_info(pdata->dev, "config done [%d]\n", pdata->config_done);
	dev_info(pdata->dev, "fgpa en [%d]\n", pdata->fpga_en);
}

#ifdef USE_WORKQUEUE_WIP
static void firmware_download_work(struct work_struct *work)
{
}
#endif

static void chip_power_enable(struct lattice_ice40_pdata *pdata, int enable)
{
	dev_info(pdata->dev, "%s %s\n", __func__, enable ? "ON" : "OFF");

	if (enable) {
		gpio_set_value(pdata->fpga_en, GPIO_LEVEL_HIGH);
		msleep(60);
	} else {
		gpio_set_value(pdata->fpga_en, GPIO_LEVEL_LOW);
		msleep(30);
	}
}

static void chip_reset(struct lattice_ice40_pdata *pdata)
{
	dev_info(pdata->dev, "%s\n", __func__);
	/* forced reset high */
	gpio_set_value(pdata->config_reset, GPIO_LEVEL_HIGH);
	msleep(20);

	/* set cs low */
	gpio_set_value_cansleep(pdata->spi_cs, GPIO_LEVEL_LOW);
	/* reset low & hold 30us */
	gpio_set_value(pdata->config_reset, GPIO_LEVEL_LOW);
	dev_info(pdata->dev, "%s: creset low\n", __func__);
	usleep_range(30, 50);

	/* reset high & hold 1ms*/
	gpio_set_value(pdata->config_reset, GPIO_LEVEL_HIGH);
	dev_info(pdata->dev, "%s: creset high\n", __func__);
	usleep_range(1000, 1300);
}

static int setup_gpio(struct lattice_ice40_pdata *pdata,
					bool debug_mode, bool init_mode)
{
	/* gpio requet here */
	int ret = 0;

	dev_info(pdata->dev, "%s: debug_mode(%d) init_mode(%d)\n",
					__func__, debug_mode, init_mode);

	if (pdata->spi_sclk > 0) {
		if (init_mode) {
			ret = gpio_request(pdata->spi_sclk, "FPGA_SPI_CLK");
			if (ret < 0) {
				dev_err(pdata->dev,
					"spi_clk request failed %d\n", ret);
				goto gpio_err;
			}
		}

		if (debug_mode)
			ret = gpio_direction_input(pdata->spi_sclk);
		else
			ret = gpio_direction_output(pdata->spi_sclk,
							GPIO_LEVEL_HIGH);
		if (ret < 0) {
			dev_err(pdata->dev,
				"spi_clk direction failed %d\n", ret);
			goto gpio_err;
		}
		/*
		ret = gpio_request_one(pdata->spi_sclk,
				GPIOF_OUT_INIT_HIGH, "FPGA_SPI_CLK");
		*/
	}

	if (pdata->spi_miso > 0) {
		if (init_mode) {
			ret = gpio_request(pdata->spi_miso, "FPGA_SPI_MISO");
			if (ret < 0) {
				dev_err(pdata->dev,
					"spi_miso request failed %d\n", ret);
				goto gpio_err;
			}
		}

		ret = gpio_direction_input(pdata->spi_miso);
		if (ret < 0) {
			dev_err(pdata->dev, "spi_miso failed %d\n", ret);
			goto gpio_err;
		}
		/*
		ret = gpio_request_one(pdata->spi_miso,
				GPIOF_IN, "FPGA_SPI_MISO");
		*/
	}

	if (pdata->spi_mosi > 0) {
		if (init_mode) {
			ret = gpio_request(pdata->spi_mosi, "FPGA_SPI_MOSI");
			if (ret < 0) {
				dev_err(pdata->dev,
					"spi_mosi request failed %d\n", ret);
				goto gpio_err;
			}
		}

		if (debug_mode)
			ret = gpio_direction_input(pdata->spi_mosi);
		else
			ret = gpio_direction_output(pdata->spi_mosi,
							GPIO_LEVEL_LOW);
		if (ret < 0) {
			dev_err(pdata->dev, "spi_mosi failed %d\n", ret);
			goto gpio_err;
		}
		/*
		ret = gpio_request_one(pdata->spi_mosi,
				GPIOF_OUT_INIT_LOW, "FPGA_SPI_MOSI");
		*/
	}

	if (pdata->spi_cs > 0) {
		if (init_mode) {
			ret = gpio_request(pdata->spi_cs, "FPGA_SPI_CS");
			if (ret < 0) {
				dev_err(pdata->dev,
					"spi_cs request failed %d\n", ret);
				goto gpio_err;
			}
		}

		if (debug_mode)
			ret = gpio_direction_input(pdata->spi_cs);
		else
			ret = gpio_direction_output(pdata->spi_cs,
							GPIO_LEVEL_HIGH);

		if (ret < 0) {
			dev_err(pdata->dev, "spi_cs failed %d\n", ret);
			goto gpio_err;
		}
		/*
		ret = gpio_request_one(pdata->spi_cs,
				GPIOF_OUT_INIT_HIGH, "FPGA_SPI_CS");
		*/
	}

	if (pdata->config_reset > 0) {
		if (init_mode) {
			ret = gpio_request(pdata->config_reset, "FPGA_CRESET");
			if (ret < 0) {
				dev_err(pdata->dev,
					"creset request failed %d\n", ret);
				goto gpio_err;
			}
		}

		if (debug_mode)
			ret = gpio_direction_input(pdata->config_reset);
		else
			ret = gpio_direction_output(pdata->config_reset,
							GPIO_LEVEL_HIGH);

		if (ret < 0) {
			dev_err(pdata->dev, "creset failed %d\n", ret);
			goto gpio_err;
		}
		/*
		ret = gpio_request_one(pdata->config_reset,
				GPIOF_OUT_INIT_HIGH, "FPGA_CRESET");
		*/
	}

	if (pdata->config_done > 0) {
		if (init_mode) {
			ret = gpio_request(pdata->config_done, "FPGA_CRESET");
			if (ret < 0) {
				dev_err(pdata->dev,
					"cdone request failed %d\n", ret);
				goto gpio_err;
			}
		}

		ret = gpio_direction_input(pdata->config_done);
		if (ret < 0) {
			dev_err(pdata->dev, "%s: cdone failed\n", __func__);
			goto gpio_err;
		}
		/*
		ret = gpio_request_one(pdata->config_done,
				GPIOF_IN, "FPGA_CRESET");
		*/
	}

	if (pdata->fpga_en > 0) {
		if (init_mode) {
			ret = gpio_request(pdata->fpga_en, "FPGA_EN");
			if (ret < 0) {
				dev_err(pdata->dev,
					"fpga_en request failed %d\n", ret);
				goto gpio_err;
			}
		}

		ret = gpio_direction_output(pdata->fpga_en, GPIO_LEVEL_HIGH);
		if (ret < 0) {
			dev_err(pdata->dev, "%s: fpga_en failed\n", __func__);
			goto gpio_err;
		}
		/*
		ret = gpio_request_one(pdata->fpga_en,
				GPIOF_OUT_INIT_HIGH, "FPGA_EN");
		*/
	}

gpio_err:
	return ret;
}

static int check_config_done(struct lattice_ice40_pdata *pdata)
{
	int retry;
	int level;

	for (retry = 0; retry < 3; retry++) {
		level = gpio_get_value(pdata->config_done);
		if (level != GPIO_LEVEL_HIGH) {
			dev_err(pdata->dev, "cdone is still low [%d][%d]\n",
								level, retry);
		} else {
			dev_info(pdata->dev, "config done!!\n");
			return 0;
		}
		msleep(20);
	}
	return -EIO;
}

static int __init lattice_ice40_probe(struct platform_device *pdev)
{
	struct lattice_ice40_pdata *pdata = NULL;
	int pos = 0, bit = 0;
	int ret;
	unsigned char spibit;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "%s no of_node\n", __func__);
		return -ENXIO;
	}
	pdata = lattice_ice40_dt_to_pdata(pdev);
	if (!pdata) {
		dev_err(&pdev->dev, "%s: no pdata\n", __func__);
		return -ENXIO;
	}
	dump_pdata(pdata);

#ifdef USE_WORKQUEUE_WIP
	pdata->wq_fw = create_singlethread_workqueue("fpga_fw");
	INIT_DELAYED_WORK(&pdata->dw_fw, firmware_download_work);
	queue_delayed_work(pdata->wq_fw, &pdata->dw_fw, msec_to_jiffies(20));
#endif
	setup_gpio(pdata, false, true); /* non-debug, init mode */
	chip_power_enable(pdata, 1);
	chip_reset(pdata);


	ret = request_firmware(&pdata->firmware, pdata->fw_name, &pdev->dev);
	if (ret != 0) {
		dev_warn(&pdev->dev, "Failed to request %s\n", pdata->fw_name);
		goto probe_out;
	}

	dev_info(&pdev->dev, "lattice: firmware size %d\n",
						pdata->firmware->size);

	for (pos = 0; pos < pdata->firmware->size; pos++) {
		if (pos <= 10) { /* debug */
			dev_info(&pdev->dev, "lattice: firmware 0x%02x\n",
						pdata->firmware->data[pos]);
		}
		spibit = pdata->firmware->data[pos];
		for (bit = 0; bit < 8; bit++) {
			gpio_set_value(pdata->spi_sclk,	GPIO_LEVEL_LOW);

			gpio_set_value(pdata->spi_mosi, !!(spibit & 0x80));

			gpio_set_value(pdata->spi_sclk, GPIO_LEVEL_HIGH);
			spibit <<= 1;
		}
	}

	for (pos = 0; pos < 50; pos++) {
		gpio_set_value(pdata->spi_sclk, GPIO_LEVEL_LOW);
		udelay(1);
		gpio_set_value(pdata->spi_sclk, GPIO_LEVEL_HIGH);
	}
	/*gpio_set_value(pdata->spi_cs, GPIO_LEVEL_HIGH);*/
	usleep_range(50, 70);

	ret = check_config_done(pdata);
	if (ret < 0) {
		dev_info(&pdev->dev, "firmware write has failed\n");
		goto probe_fw_out;
	}

	release_firmware(pdata->firmware);

	pdata->debug_mode = 0;
	pdata->debug_root = debugfs_create_dir("lattice", NULL);
	if (IS_ERR_OR_NULL(pdata->debug_root)) {
		dev_err(&pdev->dev, "can't create lattice debugfs\n");
		goto probe_out;
	}

	debugfs_create_file("debug_mode", 0644, pdata->debug_root,
			(void*) pdata, &lattice_debug_fops);

	return 0;
probe_fw_out:
	release_firmware(pdata->firmware);
probe_out:
	chip_power_enable(pdata, 0);
	return -ENODEV;
}

static struct of_device_id lattice_ice40_dt_match[] = {
	{
		.compatible = "lattice,ice40ul-640",
	},
	{}
};

static struct platform_driver lattice_ice40_driver = {
	.driver		= {
		.name	= "lattice-ice40",
		.owner	= THIS_MODULE,
		/*.pm		= &lattice_ice40_dev_pm_ops,*/
		.of_match_table = lattice_ice40_dt_match,
	},
	/*.remove		= lattice_ice40_remove,*/
};

static int __init lattice_ice40_init(void)
{
	return platform_driver_probe(&lattice_ice40_driver,
						lattice_ice40_probe);
}

module_init(lattice_ice40_init);

static void __exit lattice_ice40_exit(void)
{
	platform_driver_unregister(&lattice_ice40_driver);
}
module_exit(lattice_ice40_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kwang-Hui Cho <kwanghui.cho@samsung.com>");
MODULE_DESCRIPTION("Lattice FPGA driver");
