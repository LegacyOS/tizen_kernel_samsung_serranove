/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "m5mols.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_dt_util.h"
#include <linux/vmalloc.h>
#include <linux/firmware.h>

static struct yuv_ctrl m5mols_ctrl;
extern struct class *camera_class;
static struct m5mols_data *m5mols;

/* 8-bit write */
#define m5mols_writeb(c, g, b, v)	m5mols_write(c, 1, g, b, v)
/* 16-bit write */
#define m5mols_writew(c, g, b, v)	m5mols_write(c, 2, g, b, v)
/* 32-bit write */
#define m5mols_writel(c, g, b, v)	m5mols_write(c, 4, g, b, v)

#define m5mols_readb(c, g, b, v)	m5mols_read(c, 1, g, b, v)
#define m5mols_readw(c, g, b, v)	m5mols_read(c, 2, g, b, v)
#define m5mols_readl(c, g, b, v)	m5mols_read(c, 4, g, b, v)

static int32_t m5mols_shutter_set(uint32_t mode);

static int32_t m5mols_write(struct i2c_client *client,
	uint8_t len, uint8_t category, uint8_t byte, int32_t val)
{
	struct i2c_msg msg;
	uint8_t data[len + 4];
	int32_t i, err;

	if (len != 0x01 && len != 0x02 && len != 0x04)
		return -EINVAL;

	msg.addr = 0x1F;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	data[0] = msg.len;
	data[1] = 0x02; /* Write category parameters */
	data[2] = category;
	data[3] = byte;

	if (len == 0x01) {
		data[4] = val & 0xFF;
	} else if (len == 0x02) {
		data[4] = (val >> 8) & 0xFF;
		data[5] = val & 0xFF;
	} else {
		data[4] = (val >> 24) & 0xFF;
		data[5] = (val >> 16) & 0xFF;
		data[6] = (val >> 8) & 0xFF;
		data[7] = val & 0xFF;
	}

	CDBG("category %#x, byte %#x, value %#x", category, byte, val);

	for (i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	return err;
}

static int32_t m5mols_read(struct i2c_client *client,
	uint8_t len, uint8_t category, uint8_t byte, uint32_t *val)
{
	int32_t rc = 0;
	uint8_t tx_buf[5];
	uint8_t rx_buf[len + 1];

	struct i2c_msg msg = {
		.addr = 0x1f,
		.flags = 0,
		.len = 5,
		.buf = tx_buf,
	};
	*val = 0;

	tx_buf[0] = 0x05;
	tx_buf[1] = 0x01;
	tx_buf[2] = category;
	tx_buf[3] = byte;
	tx_buf[4] = len;

	rc = i2c_transfer(client->adapter, &msg, 1);
	if (likely(rc == 1)) {
		msg.flags = I2C_M_RD;
		msg.len = len + 1;
		msg.buf = rx_buf;
		rc = i2c_transfer(client->adapter, &msg, 1);
	} else {
		pr_err("[m5mols]%s: failed at category=0x%x, byte=0x%x\n",
			__func__, category,  byte);
		return -EIO;
	}

	if (likely(rc == 1)) {
		if (len == 1)
			*val = rx_buf[1];
		else if (len == 2)
			*(unsigned short *)val =
				be16_to_cpu(*(unsigned short *)(&rx_buf[1]));
		else
			*val = be32_to_cpu(*(unsigned int *)(&rx_buf[1]));

		return 0;
	}

	return -EIO;
}

static int32_t m5mols_mem_read(struct i2c_client *client,
	uint16_t len, uint32_t addr, uint8_t *val)
{
	struct i2c_msg msg;
	uint8_t data[8];
	uint8_t recv_data[len + 3];
	int32_t i, err = 0;

	if (!client->adapter)
		return -ENODEV;

	if (len < 1) {
		pr_err("[m5mols]%s: Data Length to read is out of range\n",
			__func__);
		return -EINVAL;
	}

	msg.addr = 0x1f;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = 0x03;
	data[2] = (addr >> 24) & 0xFF;
	data[3] = (addr >> 16) & 0xFF;
	data[4] = (addr >> 8) & 0xFF;
	data[5] = addr & 0xFF;
	data[6] = (len >> 8) & 0xFF;
	data[7] = len & 0xFF;

	for (i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1)
		return err;

	msg.flags = I2C_M_RD;
	msg.len = sizeof(recv_data);
	msg.buf = recv_data;
	for (i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1)
		return err;

	if (len != (recv_data[1] << 8 | recv_data[2]))
		pr_err("[m5mols]%s, expected length %d, but return length %d\n",
			__func__, len, recv_data[1] << 8 | recv_data[2]);

	memcpy(val, recv_data + 3, len);

	CDBG("read from offset 0x%x", addr);
	return err;

}

static int32_t m5mols_mem_write(struct i2c_client *client,
	uint8_t cmd, uint16_t len, uint32_t addr, uint8_t *val)
{
	struct i2c_msg msg;
	uint8_t data[len + 8];
	int32_t i;
	int32_t err = 0;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = 0x1F;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = cmd;
	data[2] = (addr >> 24) & 0xFF;
	data[3] = (addr >> 16) & 0xFF;
	data[4] = (addr >> 8) & 0xFF;
	data[5] = addr & 0xFF;
	data[6] = (len >> 8) & 0xFF;
	data[7] = len & 0xFF;
	memcpy(data + 2 + sizeof(addr) + sizeof(len), val, len);

	for (i = M5MO_I2C_VERIFY_RETRY; i; i--) {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	return err;
}

static int32_t m5mols_sensor_power_seq(uint8_t up_down)
{
	struct msm_sensor_fn_t *func_tbl = m5mols->msensor_ctrl->func_tbl;
	uint32_t rc = 0;

	if (!(func_tbl->sensor_power_up) || !(func_tbl->sensor_power_down)) {
		pr_err("[m5mols]%s, power seq does not exist\n", __func__);
		return -EFAULT;
	}

	if (up_down) {
		if ((!m5mols->power_count)) {
			rc = func_tbl->sensor_power_up(m5mols->msensor_ctrl);
			if (rc < 0)
				pr_err("[m5mols]%s, sensor power up is failed(%d)\n",
					__func__, rc);
		}
		m5mols->power_count++;
	} else {
		if (m5mols->power_count < 2) {
			rc = func_tbl->sensor_power_down(m5mols->msensor_ctrl);
			if (rc < 0)
				pr_err("[m5mols]%s, sensor power down is failed(%d)\n",
					__func__, rc);
		}
		m5mols->power_count--;
	}
	CDBG("up_down=%d, count=%d", up_down, m5mols->power_count);

	return rc;
}

static irqreturn_t m5mols_shutter_sense_irq_thread(int irq, void *ptr)
{
	int32_t rc = 0;
	CDBG("is called");

	rc = m5mols_shutter_set(SHUTTER_STANDBY);
	if (rc < 0)
		pr_err("[m5mols]%s, shutter mode set failed(%d)\n",
			__func__, rc);

	m5mols->shutter->issued = 1;
	wake_up_interruptible(&m5mols->shutter->wait);

	return IRQ_HANDLED;
}

static int m5mols_shutter_wait_interrupt(unsigned int timeout)
{
	if (wait_event_interruptible_timeout(m5mols->shutter->wait,
		m5mols->shutter->issued == 1, msecs_to_jiffies(timeout)) == 0) {
		pr_err("[m5mols]%s, time out\n", __func__);

		return -ETIMEDOUT;
	}
	m5mols->shutter->issued = 0;

	return 0;
}

static int32_t m5mols_check_interrupt(struct i2c_client *i2c_client)
{
	uint32_t read_data;
	int32_t err = -EINVAL;

	err = m5mols_readb(i2c_client, M5MO_CATEGORY_SYS,
		M5MO_SYS_INT_FACTOR, &read_data);

	CDBG("read data = 0x%x", read_data);

	msleep(50);
	read_data = 0;
	err = m5mols_readb(i2c_client, M5MO_CATEGORY_SYS,
		M5MO_SYS_INT_FACTOR, &read_data);

	CDBG("read data = 0x%x", read_data);

	return err;
}

static int32_t m5mols_check_manufacturer_id(struct i2c_client *client)
{
	int32_t i, err;
	uint8_t id = 0;
	uint32_t addr[] = {0x1000AAAA, 0x10005554, 0x1000AAAA};
	uint8_t val[3][2] = {
		[0] = {0x00, 0xAA},
		[1] = {0x00, 0x55},
		[2] = {0x00, 0x90},
	};
	uint8_t reset[] = {0x00, 0xF0};

	/* set manufacturer's ID read-mode */
	for (i = 0; i < 3; i++) {
		err = m5mols_mem_write(client, 0x06, 2, addr[i], val[i]);
		CHECK_ERR(err);
	}

	/* read manufacturer's ID */
	err = m5mols_mem_read(client, sizeof(id), 0x10000001, &id);
	CHECK_ERR(err);

	/* reset manufacturer's ID read-mode */
	err = m5mols_mem_write(client, 0x06, sizeof(reset), 0x10000000, reset);
	CHECK_ERR(err);

	CDBG(": %d", id);

	return id;
}

#if 0
static void m5mols_shutter_ldo_control(uint8_t enable)
{
	if (enable)
		gpio_set_value(m5mols->shutter->ldo_en, 1);
	else
		gpio_set_value(m5mols->shutter->ldo_en, 0);
}
#endif

static int32_t m5mols_shutter_set(uint32_t mode)
{
	int32_t rc = 0;

	switch (mode) {
	case SHUTTER_STANDBY:
		gpio_set_value(m5mols->shutter->in1_gpio, 0);
		gpio_set_value(m5mols->shutter->in2_gpio, 0);
		break;

	case SHUTTER_FORWARD:
		gpio_set_value(m5mols->shutter->in1_gpio, 1);
		gpio_set_value(m5mols->shutter->in2_gpio, 0);

		m5mols_shutter_wait_interrupt(M5MOLS_SHUTTER_TIMEOUT);
		break;

	case SHUTTER_REVERSE:
		gpio_set_value(m5mols->shutter->in1_gpio, 0);
		gpio_set_value(m5mols->shutter->in2_gpio, 1);

		m5mols_shutter_wait_interrupt(M5MOLS_SHUTTER_TIMEOUT);
		break;

	default:
		pr_err("[m5mols]%s, unknown mode(%d)\n", __func__, mode);
		rc = -EINVAL;
		break;
	}

	return rc;
}

#if 0
static void m5mols_irfilter_ldo_control(uint8_t enable)
{
	if (enable)
		gpio_set_value(m5mols->irfilter->ldo_en, 1);
	else
		gpio_set_value(m5mols->irfilter->ldo_en, 0);
}

static void m5mols_ir_filter_enable(int32_t enable)
{
	if (enable)
		gpio_set_value(m5mols->irfilter->filter_en, 1);
	else
		gpio_set_value(m5mols->irfilter->filter_en, 0);
}
#endif

static int32_t m5mols_set_ir_filter(int32_t mode)
{
	int32_t rc = 0;

	switch (mode) {
	case IR_FILTER_OPEN:
		gpio_set_value(m5mols->irfilter->filter_on, 0);
		gpio_set_value(m5mols->irfilter->filter_off, 0);
		break;
	case IR_FILTER_CW:
		gpio_set_value(m5mols->irfilter->filter_on, 1);
		gpio_set_value(m5mols->irfilter->filter_off, 0);
		break;
	case IR_FILTER_CCW:
		gpio_set_value(m5mols->irfilter->filter_on, 0);
		gpio_set_value(m5mols->irfilter->filter_off, 1);
		break;
	default:
		pr_err("[m5mols]%s, unknown mode(%d)\n", __func__, mode);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int32_t m5mols_set_ir_mode(int mode)
{
	struct i2c_client *client =
		m5mols->msensor_ctrl->sensor_i2c_client->client;
	int32_t rc = 0;
	CDBG(": %d", mode);

	if (mode) {
		/* m5mols_set_ir_filter(IR_FILTER_CW); */
		rc = m5mols_writeb(client,
			M5MO_CATEGORY_MON, M5MO_MON_IR_MODE, 0x01);
	} else {
		/* m5mols_set_ir_filter(IR_FILTER_CCW); */
		rc = m5mols_writeb(client,
			M5MO_CATEGORY_MON, M5MO_MON_IR_MODE, 0x00);
	}

	return rc;
}

static int32_t m5mols_get_fw_version(struct i2c_client *client, int8_t *version)
{
	int8_t val;
	int32_t rc;

	rc = m5mols_sensor_power_seq(1);
	if (rc < 0)
		goto exit;

	/* Set M5mols Pin */
	val = 0x7E;
	rc = m5mols_mem_write(client, 0x04, sizeof(val), 0x50000308, &val);
	CHECK_ERR(rc);

	rc = m5mols_mem_read(client, M5MOLS_VERSION_INFO_SIZE,
			M5MO_FLASH_BASE_ADDR + M5MO_VERSION_INFO_ADDR,
			version);

	rc = m5mols_sensor_power_seq(0);

exit:
	return rc;
}

static int32_t m5mols_program_fw(struct i2c_client *client, uint8_t *buf,
	uint32_t addr, uint32_t unit, uint32_t count, uint8_t id)
{
	uint32_t val;
	uint32_t intram_unit = SZ_2K;
	int32_t i, j, retries, err = 0;
	int32_t erase = 0x01;

	if (unit == SZ_64K && id != 0x01)
		erase = 0x04;

	CDBG("-- start -- (id:%d)", id);
	for (i = 0; i < count; i++) {
		/* Set Flash ROM memory address */
		CDBG("set addr : 0x%x (index = %d)", addr, i);
		err = m5mols_writel(client, M5MO_CATEGORY_FLASH,
				M5MO_FLASH_ADDR, addr);
		CHECK_ERR(err);

		/* Erase FLASH ROM entire memory */
		err = m5mols_writeb(client, M5MO_CATEGORY_FLASH,
				M5MO_FLASH_ERASE, erase);
		CHECK_ERR(err);

		CDBG("erase Flash ROM");
		/* Response while sector-erase is operating */
		retries = 0;
		do {
			msleep(50);
			err = m5mols_readb(client, M5MO_CATEGORY_FLASH,
					M5MO_FLASH_ERASE, &val);
			CHECK_ERR(err);
		} while (val == erase && retries++ < M5MO_I2C_VERIFY);

		/* Set FLASH ROM programming size */
		err = m5mols_writew(client, M5MO_CATEGORY_FLASH,
				M5MO_FLASH_BYTE, unit == SZ_64K ? 0 : unit);
		CHECK_ERR(err);
		msleep(20);

		/* Clear M-5MoLS internal RAM */
		err = m5mols_writeb(client, M5MO_CATEGORY_FLASH,
				M5MO_FLASH_RAM_CLEAR, 0x01);
		CHECK_ERR(err);
		msleep(10);

		/* Set Flash ROM programming address */
		err = m5mols_writel(client, M5MO_CATEGORY_FLASH,
				M5MO_FLASH_ADDR, addr);
		CHECK_ERR(err);

		/* Send programmed firmware */
		for (j = 0; j < unit; j += intram_unit) {
			err = m5mols_mem_write(client, 0x04, intram_unit,
					M5MO_INT_RAM_BASE_ADDR + j,
					buf + (i * unit) + j);
			CHECK_ERR(err);
			msleep(10);
		}

		/* Start Programming */
		err = m5mols_writeb(client, M5MO_CATEGORY_FLASH,
				M5MO_FLASH_WR, 0x01);
		CHECK_ERR(err);
		CDBG("start programming");
		msleep(50);

		/* Confirm programming has been completed */
		retries = 0;
		do {
			msleep(50);
			err = m5mols_readb(client, M5MO_CATEGORY_FLASH,
					M5MO_FLASH_WR, &val);
			CHECK_ERR(err);
		} while (val && retries++ < M5MO_I2C_VERIFY);

		/* Increase Flash ROM memory address */
		addr += unit;
	}

	CDBG("-- end --");
	return 0;
}

static int32_t m5mols_load_fw(struct i2c_client *client)
{
	const struct firmware *fw = NULL;
	struct device *dev = &client->adapter->dev;
	int8_t current_ver[M5MOLS_VERSION_INFO_SIZE + 1];
	int8_t phone_ver[M5MOLS_VERSION_INFO_SIZE + 1];
	struct file *fp;
	mm_segment_t old_fs;
	uint8_t *buf = NULL, val;
	long fsize, nread;
	loff_t fpos = 0;
	int32_t err, id;
	int32_t sdcard_fw = 1;

	/* power up */
	err = m5mols_sensor_power_seq(1);
	if (err < 0)
		goto exit;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	/* Check -1: SD Card */
#ifdef SDCARD_FW
	fp = filp_open(M5MO_FW_PATH_SDCARD, O_RDONLY, S_IRUSR);

	if (IS_ERR(fp)) {	/* if there is no firmware in SDCARD */
		pr_err("[m5mols]%s: failed to open %s\n", __func__,
			M5MO_FW_PATH_SDCARD);
		sdcard_fw = 0;
	} else {
		pr_err("[m5mols]%s: update from SD Card\n",
			__func__);
		goto request_fw;
	}
#endif

request_fw:
	if (sdcard_fw) {
		fsize = fp->f_path.dentry->d_inode->i_size;

		buf = vmalloc(fsize);
		if (!buf) {
			pr_err("[m5mols]%s: failed to allocate memory\n",
				__func__);
			goto out;
		}

		nread = vfs_read(fp, (char __user *)buf, fsize, &fpos);
		if (nread != fsize) {
			pr_err("[m5mols]%s: failed to read firmware file, nread %ld Bytes\n",
				__func__, nread);
			goto out;
		}
	} else {
		request_firmware(&fw, M5MO_FW_NAME, dev);
		buf = (uint8_t *)fw->data;
		memcpy(phone_ver, buf + M5MO_VERSION_INFO_ADDR,
			M5MOLS_VERSION_INFO_SIZE);
		phone_ver[6] = '\0';

		m5mols_get_fw_version(client, current_ver);
		current_ver[6] = '\0';
		pr_err("[m5mols]%s, current_ver=%s, phone_ver=%s\n", __func__,
			current_ver, phone_ver);

		if ((phone_ver[0] == 'P') && (phone_ver[1] == 'A')) {
			if ((phone_ver[2] < current_ver[2]) ||
				(phone_ver[3] < current_ver[3]) ||
				(phone_ver[4] < current_ver[4]) ||
				(phone_ver[5] <= current_ver[5])) {
				pr_err("[m5mols]%s, phone version is older than current verion\n",
					__func__);
				goto out;
			} else
				pr_err("[m5mols]%s: update from device firmware\n",
					__func__);
		} else {
			pr_err("[m5mols]%s, Maker or Sensor code is invalid\n",
				__func__);
			goto out;
		}
	}

	/* set pin */
	val = 0x7E;
	err = m5mols_mem_write(client, 0x04, sizeof(val), 0x50000308, &val);
	CHECK_ERR(err);

	id = m5mols_check_manufacturer_id(client);

	if (id < 0) {
		pr_err("[m5mols]%s, manufacturer_id, err %d", __func__, id);
		goto out;
	}

	/* select flash memory */
	err = m5mols_writeb(client, M5MO_CATEGORY_FLASH, M5MO_FLASH_SEL,
			id == 0x01 ? 0x00 : 0x01);
	CHECK_ERR(err);

	/* program FLSH ROM */
	err = m5mols_program_fw(client, buf, M5MO_FLASH_BASE_ADDR,
			SZ_64K, 16, id);
	CHECK_ERR(err);

out:
	CDBG("end");

	if (sdcard_fw) {
		filp_close(fp, current->files);
		if (buf != NULL)
			vfree(buf);
	} else
		release_firmware(fw);
	set_fs(old_fs);
exit:
	/* power down */
	err = m5mols_sensor_power_seq(0);

	return err;
}

static int32_t m5mols_dump_fw(struct i2c_client *client)
{
	struct file *fp;
	mm_segment_t old_fs;
	uint8_t *buf = NULL, val;
	uint32_t addr, unit, count, intram_unit = SZ_2K;
	int32_t i, j, err;

	err = m5mols_sensor_power_seq(1);
	if (err < 0)
		goto exit;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(M5MO_FW_DUMP_PATH,
		O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		pr_err("[m5mols]%s failed to open %s, err %ld\n",
			__func__, M5MO_FW_DUMP_PATH, PTR_ERR(fp));
		err = -ENOENT;
		goto out;
	}

	buf = kmalloc(intram_unit, GFP_KERNEL);
	if (!buf) {
		pr_err("[m5mols]%s, failed to allocate memory\n", __func__);
		err = -ENOMEM;
		goto out;
	}

	CDBG("start, file path %s\n", M5MO_FW_DUMP_PATH);

	/* set pin */
	val = 0x7E;
	err = m5mols_mem_write(client, 0x04, sizeof(val), 0x50000308, &val);

	if (err < 0) {
		pr_err("[m5mols]%s: set_pin : i2c falied, err %d\n",
			__func__, err);
		goto out;
	}
	addr = M5MO_FLASH_BASE_ADDR;
	unit = SZ_64K;
	count = 16;
	for (i = 0; i < count; i++) {
		for (j = 0; j < unit; j += intram_unit) {
			err = m5mols_mem_read(client, intram_unit,
				addr + (i * unit) + j, buf);
			if (err < 0) {
				pr_err("[m5mols]%s: i2c falied, err %d\n",
					__func__, err);
				goto out;
			}
			vfs_write(fp, buf, intram_unit, &fp->f_pos);
		}
	}
	CDBG("end\n");

out:
	if (buf != NULL)
		kfree(buf);

	if (!IS_ERR(fp))
		filp_close(fp, current->files);
	set_fs(old_fs);
exit:
	err = m5mols_sensor_power_seq(0);

	return err;
}

static int m5mols_firmware_update_mode(int mode)
{
	int   rc = 0;
	int8_t version[M5MOLS_VERSION_INFO_SIZE + 1] = "PAIA00";
	struct i2c_client *client =
		m5mols->msensor_ctrl->sensor_i2c_client->client;

	CDBG(": %d", mode);

	switch(mode)
	{
	case CAMERA_FIRMWARE_VERSION:
		CDBG("#### FIRMWARE VERSION READ ####");
		m5mols_get_fw_version(client, version);
		break;

	case CAMERA_FIRMWARE_UPDATE:
		CDBG("#### FIRMWARE UPDATE START ! ####");
		/* FIX ME, need to check it is necessary
		m5mo_reset_for_update(); */

		rc = m5mols_load_fw(client);
		if (rc == 0) {
			CDBG("#### FIRMWARE UPDATE SUCCEEDED ! ####");
		} else {
			CDBG("#### FIRMWARE UPDATE FAILED ! ####");
		}
		break;

	case CAMERA_FIRMWARE_DUMP:
		CDBG("#### FIRMWARE DUMP START ! ####");
		/* FIX ME, need to check it is necessary
		m5mo_reset_for_update(); */

		rc = m5mols_dump_fw(client);
		if (rc == 0) {
			CDBG("#### FIRMWARE DUMP SUCCEEDED ! ####");
		} else {
			CDBG("#### FIRMWARE DUMP FAILED ! ####");
		}
		break;

	}

	return 0;
}

static ssize_t m5mols_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t m5mols_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", NAME);
}

static ssize_t m5mols_fw_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int8_t version[M5MOLS_VERSION_INFO_SIZE + 1] = "PAIA00";
	struct i2c_client *client =
		m5mols->msensor_ctrl->sensor_i2c_client->client;

	m5mols_get_fw_version(client, version);
	CDBG("current isp fw version = %s", version);

	return snprintf(buf, PAGE_SIZE, "%s\n", version);
}

static ssize_t m5mols_fw_dump(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int32_t err = 0;
	struct i2c_client *client =
		m5mols->msensor_ctrl->sensor_i2c_client->client;

	err = m5mols_dump_fw(client);

	return snprintf(buf, PAGE_SIZE, "%s\n", (err ? "NG" : "OK"));
}

static ssize_t m5mols_fw_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client =
		m5mols->msensor_ctrl->sensor_i2c_client->client;

	m5mols_load_fw(client);

	return size;
}

static ssize_t m5mols_shutter_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int32_t rc = 0;
	int64_t mode;

	if (kstrtoll(buf, 10, &mode) < 0)
		return -EINVAL;

	CDBG("Shutter mode = %d", (uint32_t)mode);
	rc = m5mols_shutter_set((uint32_t) mode);
	if (rc < 0)
		pr_err("[m5mols]%s, failed shutter set\n", __func__);

	return size;
}

static ssize_t m5mols_ir_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int32_t rc = 0;
	int64_t mode;

	if (kstrtoll(buf, 10, &mode) < 0)
		return -EINVAL;

	CDBG("ir mode = %d", (uint32_t)mode);
	rc = m5mols_set_ir_mode((uint32_t) mode);
	if (rc < 0)
		pr_err("[m5mols]%s, failed set ir mode\n", __func__);

	return size;
}

static ssize_t m5mols_ir_filter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int32_t rc = 0;


	return snprintf(buf, PAGE_SIZE, "%d\n", rc);
}

static ssize_t m5mols_ir_filter_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int32_t rc = 0;
	int64_t mode;

	if (kstrtoll(buf, 10, &mode) < 0)
		return -EINVAL;

	CDBG("ir filter set to %d", (uint32_t)mode);

	rc = m5mols_set_ir_filter((uint32_t) mode);
	if (rc < 0)
		pr_err("[m5mols]%s, failed set ir mode\n", __func__);

	return size;
}

static DEVICE_ATTR(vendor, S_IRUGO, m5mols_vendor_show, NULL);
static DEVICE_ATTR(name, S_IRUGO, m5mols_name_show, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, m5mols_fw_version_show, NULL);
static DEVICE_ATTR(firmware, S_IRUGO | S_IWUSR | S_IWGRP,
	m5mols_fw_dump, m5mols_fw_store);
static DEVICE_ATTR(shutter, S_IWUSR | S_IWGRP,
	NULL, m5mols_shutter_store);
static DEVICE_ATTR(ir_mode, S_IWUSR | S_IWGRP,
	NULL, m5mols_ir_mode_store);
static DEVICE_ATTR(ir_filter, S_IRUGO | S_IWUSR | S_IWGRP,
	m5mols_ir_filter_show, m5mols_ir_filter_store);

static struct device_attribute *m5mols_attrs[] = {
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_fw_version,
	&dev_attr_firmware,
	&dev_attr_shutter,
	&dev_attr_ir_mode,
	&dev_attr_ir_filter,
	NULL,
};

static int32_t m5mols_set_interrupt(struct m5mols_data *data)
{
	int32_t rc = 0;

	data->shutter->sense_upper_irq =
		gpio_to_irq(data->shutter->sense_upper_gpio);

	rc = request_threaded_irq(data->shutter->sense_upper_irq, NULL,
				m5mols_shutter_sense_irq_thread,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"m5mols_shutter_sense_upper_irq", NULL);
	if (rc < 0)
		pr_err("[m5mols]%s, Failed to register shutter_sense_upper_irq(%d)\n",
				__func__, rc);

	data->shutter->sense_lower_irq =
		gpio_to_irq(data->shutter->sense_lower_gpio);

	rc = request_threaded_irq(data->shutter->sense_lower_irq, NULL,
				m5mols_shutter_sense_irq_thread,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"m5mols_shutter_sense_lower_irq", NULL);
	if (rc < 0)
		pr_err("[m5mols]%s, Failed to register shutter_sense_lower_irq(%d)\n",
				__func__, rc);

	return rc;
}

static int32_t m5mols_set_gpio(struct m5mols_data *data)
{
	int32_t rc = 0;

	/* MOTOR_LDO_EN */
	rc = gpio_request(data->shutter->ldo_en, "m5mols_shutter_ldo_en");
	if (rc < 0)
		pr_err("[m5mols]%s, unable to request m5mols_shutter_ldo_en[%d]\n",
			__func__, data->shutter->ldo_en);
	gpio_direction_output(data->shutter->ldo_en, 0);

	/* MOTOR_PWM_IN1 */
	rc = gpio_request(data->shutter->in1_gpio, "m5mols_shutter_in1");
	if (rc < 0)
		pr_err("[m5mols]%s, unable to request m5mols_shutter_in1[%d]\n",
			__func__, data->shutter->in1_gpio);
	gpio_direction_output(data->shutter->in1_gpio, 0);

	/* MOTOR_PWM_IN2 */
	rc = gpio_request(data->shutter->in2_gpio, "m5mols_shutter_in2");
	if (rc < 0)
		pr_err("[m5mols]%s, unable to request m5mols_shutter_in2[%d]\n",
			__func__, data->shutter->in2_gpio);
	gpio_direction_output(data->shutter->in2_gpio, 0);

	/* MOTOR_SENSE_UPPER */
	rc = gpio_request(data->shutter->sense_upper_gpio,
			"m5mols_shutter_sense_upper");
	if (rc < 0)
		pr_err("[m5mols]%s, unable to request m5mols_shutter_sense_upper[%d]\n",
			__func__, data->shutter->sense_upper_gpio);

	/* MOTOR_SENSE_LOWER */
	rc = gpio_request(data->shutter->sense_lower_gpio,
			"m5mols_shutter_sense_lower");
	if (rc < 0)
		pr_err("[m5mols]%s, unable to request m5mols_shutter_sense_lower[%d]\n",
			__func__, data->shutter->sense_lower_gpio);

	/* IR_FILTER_LDO_EN */
	rc = gpio_request(data->irfilter->ldo_en,
			"m5mols_irfilter_ldo_en");
	if (rc < 0)
		pr_err("[m5mols]%s, unable to request m5mols_irfilter_ldo_en[%d]\n",
			__func__, data->irfilter->ldo_en);
	gpio_direction_output(data->irfilter->ldo_en, 0);

	/* IR_FILTER_EN_1.8V */
	rc = gpio_request(data->irfilter->filter_en,
			"m5mols_irfilter_en");
	if (rc < 0)
		pr_err("[m5mols]%s, unable to request m5mols_irfilter_en[%d]\n",
			__func__, data->irfilter->filter_en);
	gpio_direction_output(data->irfilter->filter_en, 0);

	/* IR_FILTER_ON_1.8V */
	rc = gpio_request(data->irfilter->filter_on,
			"m5mols_irfilter_on");
	if (rc < 0)
		pr_err("[m5mols]%s, unable to request m5mols_irfilter_on[%d]\n",
			__func__, data->irfilter->filter_on);
	gpio_direction_output(data->irfilter->filter_on, 0);

	/* IR_FILTER_OFF_1.8V */
	rc = gpio_request(data->irfilter->filter_off,
			"m5mols_irfilter_off");
	if (rc < 0)
		pr_err("[m5mols]%s, unable to request m5mols_irfilter_off[%d]\n",
			__func__, data->irfilter->filter_off);
	gpio_direction_output(data->irfilter->filter_off, 0);

	/* Set interrupt */
	rc = m5mols_set_interrupt(data);
	if (rc < 0)
		pr_err("[m5mols]%s, interrupt set failed(%d)\n", __func__, rc);

	return rc;
}

static int32_t m5mols_parse_dt(struct m5mols_data *data)
{
	int32_t rc = 0;
	struct device_node *of_node = data->msensor_ctrl->of_node;

	/* Get gpio for Shutter */
	data->shutter->ldo_en = of_get_named_gpio(of_node,
		"m5mols,gpio-shutter_ldo_en", 0);
	if (data->shutter->ldo_en < 0) {
		pr_err("[m5mols]%s, getting gpio failed for shutter ldo en\n",
			__func__);
		return -ENODEV;
	}

	data->shutter->in1_gpio = of_get_named_gpio(of_node,
		"m5mols,gpio-shutter_in1", 0);
	if (data->shutter->in1_gpio < 0) {
		pr_err("[m5mols]%s, getting gpio failed for shutter in1\n",
			__func__);
		return -ENODEV;
	}

	data->shutter->in2_gpio = of_get_named_gpio(of_node,
		"m5mols,gpio-shutter_in2", 0);
	if (data->shutter->in2_gpio < 0) {
		pr_err("[m5mols]%s, getting gpio failed for shutter in2\n",
			__func__);
		return -ENODEV;
	}

	data->shutter->sense_upper_gpio = of_get_named_gpio(of_node,
		"m5mols,gpio-shutter_sense_upper_irq", 0);
	if (data->shutter->sense_upper_gpio < 0) {
		pr_err("[m5mols]%s, getting gpio failed for shutter sense upper\n",
				__func__);
		return -ENODEV;
	}

	data->shutter->sense_lower_gpio = of_get_named_gpio(of_node,
		"m5mols,gpio-shutter_sense_lower_irq", 0);
	if (data->shutter->sense_lower_gpio < 0) {
		pr_err("[m5mols]%s, getting gpio failed for shutter sense lower\n",
				__func__);
		return -ENODEV;
	}

	/* Get gpio for IR Filter */
	data->irfilter->ldo_en = of_get_named_gpio(of_node,
		"m5mols,gpio_irfilter_ldo_en", 0);
	if (data->irfilter->ldo_en < 0) {
		pr_err("[m5mols]%s, getting gpio failed for irfilter ldo en\n",
				__func__);
		return -ENODEV;
	}

	data->irfilter->filter_en = of_get_named_gpio(of_node,
		"m5mols,gpio_irfilter_en", 0);
	if (data->irfilter->filter_en < 0) {
		pr_err("[m5mols]%s, getting gpio failed for irfilter en\n",
				__func__);
		return -ENODEV;
	}

	data->irfilter->filter_on = of_get_named_gpio(of_node,
		"m5mols,gpio_irfilter_on", 0);
	if (data->irfilter->filter_on < 0) {
		pr_err("[m5mols]%s, getting gpio failed for irfilter on_gpio\n",
				__func__);
		return -ENODEV;
	}

	data->irfilter->filter_off = of_get_named_gpio(of_node,
		"m5mols,gpio_irfilter_off", 0);
	if (data->irfilter->filter_off < 0) {
		pr_err("[m5mols]%s, getting gpio failed for irfilter off_gpio\n",
				__func__);
		return -ENODEV;
	}

	return rc;
}

int32_t m5mols_sensor_init(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct m5mols_shuter *shutter;
	struct m5mols_irfilter *irfilter;
	int32_t rc = 0;
	int32_t i;

	CDBG("Enter");
	m5mols = kzalloc(sizeof(*m5mols), GFP_KERNEL);
	if (m5mols == NULL) {
		pr_err("[m5mols] %s, failed to allocate memory for m5mols\n",
				__func__);
		rc = -ENOMEM;
		goto m5mols_mem_alloc_fail;
	}

	shutter = kzalloc(sizeof(*shutter), GFP_KERNEL);
	if (shutter == NULL) {
		pr_err("[m5mols] %s, failed to allocate memory for shutter\n",
				__func__);
		rc = -ENOMEM;
		goto shutter_mem_alloc_fail;
	}

	irfilter = kzalloc(sizeof(*irfilter), GFP_KERNEL);
	if (irfilter == NULL) {
		pr_err("[m5mols] %s, failed to allocate memory for irfilter\n",
				__func__);
		rc = -ENOMEM;
		goto irfilter_mem_alloc_fail;
	}

	m5mols->msensor_ctrl = s_ctrl;
	m5mols->shutter = shutter;
	m5mols->irfilter = irfilter;

	rc = m5mols_parse_dt(m5mols);
	if (rc < 0)
		pr_err("[m5mols]%s, failed parse dt\n", __func__);
	else {
		rc = m5mols_set_gpio(m5mols);
		if (rc < 0)
			pr_err("[m5mols]%s, failed set gpio\n", __func__);
/*
		m5mols_shutter_ldo_control(1);
		m5mols_irfilter_ldo_control(1);
*/
	}
	m5mols->power_count = 0;

	init_waitqueue_head(&m5mols->shutter->wait);

	m5mols->m5mols_cam = device_create(camera_class, NULL,
						 1, NULL, "m5mols");
	if (IS_ERR(m5mols->m5mols_cam)) {
		pr_err("[m5mols]%s, Failed to create m5mols_cam device!\n",
			__func__);
		goto device_create_fail;
	}

	for (i = 0; m5mols_attrs[i] != NULL; i++) {
		if ((device_create_file(m5mols->m5mols_cam,
			m5mols_attrs[i])) < 0) {
			pr_err("[m5mols]%s, Failed device_create_file, attrs[%d]\n",
				__func__, i);
			goto device_create_fail;
		}
	}
	CDBG("Exit");

	return 0;

device_create_fail:
	free_irq(shutter->sense_upper_irq, shutter);
	free_irq(shutter->sense_lower_irq, shutter);
	gpio_free(shutter->sense_upper_gpio);
	gpio_free(shutter->sense_lower_gpio);
	kfree(irfilter);
irfilter_mem_alloc_fail:
	kfree(shutter);
shutter_mem_alloc_fail:
	kfree(m5mols);
m5mols_mem_alloc_fail:
	return rc;
}

int m5mols_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	const char *sensor_name = NULL;
	struct msm_camera_slave_info *slave_info = NULL;
	struct msm_camera_i2c_client *sensor_i2c_client = NULL;

	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;
	if (!sensor_i2c_client || !slave_info || !sensor_name) {
		pr_err("%s:%d failed: %p %p %p\n", __func__, __LINE__,
			sensor_i2c_client, slave_info, sensor_name);
		return -EINVAL;
	}

	CDBG("sensor_name =%s slaveid = 0x%X\n", sensor_name,
		sensor_i2c_client->cci_client->sid);

	return 0;
}

void m5mols_init_camera(struct msm_sensor_ctrl_t *s_ctrl)
{
	CDBG(" CFG_GET_SENSOR_INFO");
}

int32_t m5mols_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	int32_t rc = 0;
	int32_t i = 0;
	struct msm_camera_i2c_client *sensor_i2c_client =
						s_ctrl->sensor_i2c_client;

	mutex_lock(s_ctrl->msm_sensor_mutex);

	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		CDBG(" CFG_GET_SENSOR_INFO");
		memcpy(cdata->cfg.sensor_info.sensor_name,
		s_ctrl->sensordata->sensor_name,
		sizeof(cdata->cfg.sensor_info.sensor_name));

		cdata->cfg.sensor_info.session_id =
		s_ctrl->sensordata->sensor_info->session_id;

		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
			s_ctrl->sensordata->sensor_info->subdev_id[i];

		CDBG("sensor name %s", cdata->cfg.sensor_info.sensor_name);
		CDBG("session id %d", cdata->cfg.sensor_info.session_id);

		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("subdev_id[%d] %d", i,
			    cdata->cfg.sensor_info.subdev_id[i]);

		break;
	case CFG_SET_INIT_SETTING:
		CDBG("CFG_SET_INIT_SETTING");
		break;
	case CFG_SET_RESOLUTION:
		CDBG("CFG_SET_RESOLUTION");
		break;
	case CFG_SET_STOP_STREAM:
		if (m5mols_ctrl.streamon == 1) {
			CDBG("CFG_SET_STOP_STREAM");
			m5mols_ctrl.streamon = 0;
		}
		break;
	case CFG_SET_START_STREAM:
		CDBG(" CFG_SET_START_STREAM");
		switch (m5mols_ctrl.op_mode) {
		case  CAMERA_MODE_PREVIEW:
			CDBG(" CFG_SET_START_STREAM: Preview");
			rc = m5mols_writeb(sensor_i2c_client->client,
				M5MO_CATEGORY_SYS, M5MO_SYS_MODE,
				M5MO_MONITOR_MODE);
			if (rc < 0)
				pr_err("[m5mols]%s, send cmd fail(%d)\n",
					__func__, rc);
			msleep(50);
			m5mols_check_interrupt(sensor_i2c_client->client);
		break;
		case CAMERA_MODE_CAPTURE:
			CDBG("CFG_SET_START_STREAM: Capture");
			break;
		case CAMERA_MODE_RECORDING:
			CDBG("CFG_SET_START_STREAM: Recording");
			break;
		}
		m5mols_ctrl.streamon = 1;
		break;
	case CFG_SET_SLAVE_INFO:
		CDBG("CFG_SET_SLAVE_INFO");
		break;
	case CFG_WRITE_I2C_ARRAY:
		CDBG("CFG_WRITE_I2C_ARRAY");
		break;
	case CFG_WRITE_I2C_SEQ_ARRAY:
		CDBG("CFG_WRITE_I2C_SEQ_ARRAY");
		break;
	case CFG_POWER_UP:
		CDBG(" CFG_POWER_UP");
		m5mols_ctrl.streamon = 0;
		m5mols_ctrl.op_mode = CAMERA_MODE_INIT;
		m5mols_ctrl.prev_mode = CAMERA_MODE_INIT;
		m5mols_ctrl.settings.prev_resolution = MSM_SENSOR_RES_FULL;
		m5mols_ctrl.settings.resolution = MSM_SENSOR_RES_FULL;

/*		m5mols_shutter_set(SHUTTER_FORWARD); */

		rc = m5mols_sensor_power_seq(1);

		rc = m5mols_writeb(sensor_i2c_client->client,
			M5MO_CATEGORY_FLASH, M5MO_FLASH_CAM_START, 0x01);
		if (rc < 0)
			pr_err("[m5mols]%s, send cmd fail(%d)\n", __func__, rc);
		msleep(50);
		m5mols_check_interrupt(sensor_i2c_client->client);
		break;
	case CFG_POWER_DOWN:
		CDBG("CFG_POWER_DOWN");
		rc = m5mols_sensor_power_seq(0);

/*		m5mols_shutter_set(SHUTTER_REVERSE); */
		break;
	case CFG_SET_STOP_STREAM_SETTING:
		CDBG("CFG_SET_STOP_STREAM_SETTING");
		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

int32_t m5mols_sensor_native_control(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct ioctl_native_cmd *cam_info = (struct ioctl_native_cmd *)argp;
	int32_t rc = 0;

	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("cam_info values = %d : %d : %d : %d : %d\n",
		cam_info->mode, cam_info->address, cam_info->value_1,
		cam_info->value_2 , cam_info->value_3);

	switch (cam_info->mode) {
	case EXT_CAM_EV:
		m5mols_ctrl.settings.exposure = (cam_info->value_1);
		break;
	case EXT_CAM_WB:
		m5mols_ctrl.settings.wb = (cam_info->value_1);
		break;
	case EXT_CAM_METERING:
		m5mols_ctrl.settings.metering = (cam_info->value_1);
		break;
	case EXT_CAM_EFFECT:
		m5mols_ctrl.settings.effect = (cam_info->value_1);
		break;
	case EXT_CAM_SCENE_MODE:
		m5mols_ctrl.settings.scenemode = (cam_info->value_1);
		break;
	case EXT_CAM_SENSOR_MODE:
		m5mols_ctrl.prev_mode =	m5mols_ctrl.op_mode;
		m5mols_ctrl.op_mode = (cam_info->value_1);
		CDBG("EXT_CAM_SENSOR_MODE = %d", m5mols_ctrl.op_mode);
		break;
	case EXT_CAM_EXIF:
		/* sr352_get_exif(cam_info); */
		if (!copy_to_user((void *)argp,
					(const void *)&cam_info,
					sizeof(cam_info)))
			pr_err("copy failed");
		break;
	case EXT_CAM_SET_AE_AWB:
		CDBG("EXT_CAM_SET_AE_AWB lock[%d]\n", cam_info->value_1);
		m5mols_ctrl.settings.aeawblock = cam_info->value_1;
		break;
	case EXT_CAM_SET_FIRMWARE_UPDATE:
		rc = m5mols_firmware_update_mode(cam_info->value_1);
		break;
	case EXT_CAM_IR_MODE:
		rc = m5mols_set_ir_mode(cam_info->value_1);
		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}
