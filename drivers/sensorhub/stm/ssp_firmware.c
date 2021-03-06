/*
 *  Copyright (C) 2015, Samsung Electronics Co. Ltd. All Rights Reserved.
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
 */

#include "ssp_firmware.h"

struct stm32fwu_spi_cmd {
	u8 cmd;
	u8 xor_cmd;
	u8 ack_pad; /* Send this when waiting for an ACK */
	u8 reserved;
	int status; /* ACK or NACK (or error) */
	int timeout; /* This is number of retries */
	int ack_loops; /* Not used */
};

static int stm32fwu_spi_write(struct spi_device *spi,
			const u8 *buffer, ssize_t len);


unsigned int get_module_rev(struct ssp_data *data)
{
	return SSP_FIRMWARE_REVISION_STM;
}

static void stm32fwu_spi_send_ack( struct spi_device *spi, u8 SyncData )
{
	u8 syncb[2] = {0};
	syncb[0] = SyncData;
	stm32fwu_spi_write(spi, syncb, 1);
}

static int stm32fwu_spi_wait_for_ack(struct spi_device *spi,
		struct stm32fwu_spi_cmd *cmd, u8 dummy_bytes)
{
#ifdef CONFIG_SENSORS_SSP_STM32F401CEY6B
	static int check_spi_wait_cnt = 1;
#endif
	struct spi_message m;
	char tx_buf = 0x0;
	char rx_buf = 0x0;
	struct spi_transfer	t = {
		.tx_buf		= &tx_buf,
		.rx_buf		= &rx_buf,
		.len		= 1,
		.bits_per_word = 8,
	};
	int i = 0;
	int ret;
#ifdef CONFIG_SENSORS_SSP_STM32F401CEY6B
	dummy_bytes = BL_DUMMY;
#endif
#if SSP_STM_DEBUG
	ssp_infof("dummy byte = 0x%02hhx", dummy_bytes);
#endif
	while (i < cmd->timeout) {
		tx_buf = dummy_bytes;
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);

		ret = spi_sync(spi, &m);
#ifdef CONFIG_SENSORS_SSP_STM32F401CEY6B
		if (ret < 0) {
			dev_err(&spi->dev, "%s: spi error %d\n", __func__, ret);
			return ret;
		} else if ((rx_buf == BL_ACK) || (rx_buf == BL_NACK)) {
			/* ACK cmd set */
			stm32fwu_spi_send_ack(spi, BL_ACK);
			return (int)rx_buf;
		} else {
			/* Cross cmd set */
			tx_buf = rx_buf;
		}
		if (check_spi_wait_cnt % 20 == 0)
			usleep_range(1000, 1100);
		else
			usleep_range(1000, 1100);
		i++;
		check_spi_wait_cnt++;
#else
		if (ret < 0) {
			dev_err(&spi->dev, "%s: spi_sync error returned %d\n",
					__func__, ret);
			return ret;
		} else  if ((rx_buf == BL_ACK) || (rx_buf == BL_ACK2)) {
			cmd->ack_loops = i;
			return BL_ACK;
		} else if (rx_buf == BL_NACK) {
			return (int)rx_buf;
		}
		usleep_range(1000, 1100);
		i++;
#endif
	}
#if SSP_STM_DEBUG
	dev_err(&spi->dev, "%s: Timeout after %d loops\n",
			__func__, cmd->timeout);
#endif
	return -EIO;
}

static int stm32fwu_spi_send_cmd(struct spi_device *spi,
		struct stm32fwu_spi_cmd *cmd)
{
	u8 tx_buf[3] = {0,};
	u8 rx_buf[3] = {0,};
	u8 dummy_byte = 0;
	struct spi_message m;
	int ret;
#if BYTETOBYTE_USED
	int i;
	struct spi_transfer t[STM_MAX_BUFFER_SIZE];
	memset(t, 0, STM_MAX_BUFFER_SIZE * sizeof(struct spi_transfer));
#else
	struct spi_transfer	t = {
		.tx_buf		= tx_buf,
		.rx_buf		= rx_buf,
		.len		= 3,
		.bits_per_word = 8,
	};
#endif
	ssp_dbgf();

	spi_message_init(&m);
	tx_buf[0] = BL_SPI_SOF;
	tx_buf[1] = cmd->cmd;
	tx_buf[2] = cmd->xor_cmd;

#if BYTETOBYTE_USED
	for (i = 0; i < 3; i++) {
		t[i].tx_buf = &tx_buf[i];
		t[i].rx_buf = &rx_buf[i];
		t[i].len = 1;
		t[i].bits_per_word = 8;
		t[i].delay_usecs = BYTE_DELAY_WRITE;
		spi_message_add_tail(&t[i], &m);
	}
#else
	spi_message_add_tail(&t, &m);
#endif

	ret = spi_sync(spi, &m);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: spi error %d\n", __func__, ret);
		return ret;
	}

	dummy_byte = cmd->ack_pad;

	/* check for ack/nack and loop until found */
	ret = stm32fwu_spi_wait_for_ack(spi, cmd, dummy_byte);
	cmd->status = ret;

	if (ret != BL_ACK) {
		ssp_errf("Got NAK or Error %d", ret);
		return ret;
	}

	return ret;
}

static int stm32fwu_spi_write(struct spi_device *spi,
		const u8 *buffer, ssize_t len)
{
	int ret;
	u8 rx_buf[STM_MAX_BUFFER_SIZE] = {0,};
	struct spi_message m;
#if BYTETOBYTE_USED
	struct spi_transfer t[STM_MAX_BUFFER_SIZE];
	memset(t, 0, STM_MAX_BUFFER_SIZE * sizeof(struct spi_transfer));
	int i;
#else
	struct spi_transfer	t = {
		.tx_buf		= buffer,
		.rx_buf		= rx_buf,
		.len		= (unsigned int)len,
		.bits_per_word = 8,
	};
#endif
	spi_message_init(&m);
#if BYTETOBYTE_USED
	for (i = 0; i < len; i++) {
		t[i].tx_buf = &buffer[i];
		t[i].rx_buf = &rx_buf[i];
		t[i].len = 1;
		t[i].bits_per_word = 8;
		t[i].delay_usecs = BYTE_DELAY_WRITE;
		spi_message_add_tail(&t[i], &m);
	}
#else
	spi_message_add_tail(&t, &m);
#endif
	ret = spi_sync(spi, &m);

	if (ret < 0) {
		ssp_err("Error in %d spi_write()", ret);
		return ret;
	}

	return len;
}

static int send_addr(struct spi_device *spi, u32 fw_addr, int send_short)
{
	int res;
	int i = send_short;
	int len = SEND_ADDR_LEN - send_short;
	u8 header[SEND_ADDR_LEN];
	struct stm32fwu_spi_cmd dummy_cmd;
	dummy_cmd.timeout = DEF_ACKROOF_NUMBER;
	ssp_dbgf();

	header[0] = (u8)((fw_addr >> 24) & 0xFF);
	header[1] = (u8)((fw_addr >> 16) & 0xFF);
	header[2] = (u8)((fw_addr >> 8) & 0xFF);
	header[3] = (u8)(fw_addr & 0xFF);
	header[4] = header[0] ^ header[1] ^ header[2] ^ header[3];

	res = stm32fwu_spi_write(spi, &header[i], len);

	if (res <  len) {
		ssp_err("Error in sending address. Res %d", res);
		return (res > 0) ? -EIO : res;
	}

	res = stm32fwu_spi_wait_for_ack(spi, &dummy_cmd, BL_ACK);
	if (res != BL_ACK) {
		ssp_err("send_addr(): rcv_ack returned 0x%x", res);
		return res;
	}
	return 0;
}

static int fw_write_stm(struct spi_device *spi, u32 fw_addr,
		int len, const u8 *buffer)
{
	int res;
	struct stm32fwu_spi_cmd cmd;
	struct stm32fwu_spi_cmd dummy_cmd;
	int i;
	u8 xor = 0;
	u8 send_buff[STM_MAX_BUFFER_SIZE] = {0,};

	cmd.cmd = WMEM_COMMAND;
	cmd.xor_cmd = XOR_WMEM_COMMAND;
	cmd.timeout = DEF_ACKCMD_NUMBER;
	cmd.ack_pad = (u8)((fw_addr >> 24) & 0xFF);
	ssp_dbgf();
#if SSP_STM_DEBUG
	ssp_info("sending WMEM_COMMAND");
#endif

	if (len > STM_MAX_XFER_SIZE) {
		ssp_err("Can't send more than 256 bytes per transaction");
		return -EINVAL;
	}

	send_buff[0] = len - 1;
	memcpy(&send_buff[1], buffer, len);
	for (i = 0; i < (len + 1); i++)
		xor ^= send_buff[i];

	send_buff[len + 1] = xor;

	res = stm32fwu_spi_send_cmd(spi, &cmd);
	if (res != BL_ACK) {
		ssp_err("Error %d sending read_mem cmd", res);
		return res;
	}

#ifdef CONFIG_SENSORS_SSP_STM32F401CEY6B
	res = send_addr(spi, fw_addr, 0);
#else
	if (cmd.ack_loops > 0)
		res = send_addr(spi, fw_addr, 1);
	else
		res = send_addr(spi, fw_addr, 0);
#endif

	if (res != 0) {
		ssp_err("Error %d sending write_mem Address", res);
		return res;
	}

	res = stm32fwu_spi_write(spi, send_buff, len + 2);
	if (res <  len) {
		ssp_err("Error writing to flash. res = %d", res);
		return (res > 0) ? -EIO : res;
	}
	ssp_dbgf("2");

	dummy_cmd.timeout = DEF_ACKROOF_NUMBER;
	usleep_range(100, 150); /* Samsung added */
	res = stm32fwu_spi_wait_for_ack(spi, &dummy_cmd, BL_ACK);
	if (res == BL_ACK)
		return len;

	if (res == BL_NACK) {
		ssp_err("Got NAK waiting for WRITE_MEM to complete");
		return -EPROTO;
	}
	ssp_err("timeout waiting for ACK for WRITE_MEM command");
	return -ETIME;
}

static int load_ums_fw_bootmode(struct spi_device *spi, const char *pFn)
{
	const u8 *buff = NULL;
	char fw_path[BL_UMS_FW_PATH+1];
	unsigned int uFSize = 0, uNRead = 0;
	unsigned int uPos = 0;
	int iRet = SUCCESS;
	int remaining;
	int block = STM_MAX_XFER_SIZE;
	unsigned int fw_addr = STM_APP_ADDR;
	int retry_count = 0;
	int err_count = 0;
	int count = 0;
	struct file *fp = NULL;
	mm_segment_t old_fs = get_fs();

	ssp_info("ssp_load_ums_fw start!!");

	old_fs = get_fs();
	set_fs(get_ds());

	snprintf(fw_path, BL_UMS_FW_PATH, "/sdcard/ssp/%s", pFn);

	fp = filp_open(fw_path, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		iRet = ERROR;
		ssp_err("file %s open error", fw_path);
		goto err_open;
	}

	uFSize = (unsigned int)fp->f_path.dentry->d_inode->i_size;
	ssp_info("ssp_load_ums firmware size: %u", uFSize);

	buff = kzalloc((size_t)uFSize, GFP_KERNEL);
	if (!buff) {
		iRet = ERROR;
		ssp_err("fail to alloc buffer for fw");
		goto err_alloc;
	}

	uNRead = (unsigned int)vfs_read(fp, (char __user *)buff,
			(unsigned int)uFSize, &fp->f_pos);
	if (uNRead != uFSize) {
		iRet = ERROR;
		ssp_err("fail to read file %s (nread = %u)", fw_path, uNRead);
		goto err_fw_size;
	}
	remaining = uFSize;

	while (remaining > 0) {
		if (block > remaining)
			block = remaining;

		while (retry_count < 3) {
			iRet = fw_write_stm(spi, fw_addr, block, buff + uPos);
			if (iRet < block) {
				ssp_err("Err writing to addr 0x%08X", fw_addr);
				if (iRet < 0) {
					ssp_err("Error was %d", iRet);
				} else {
					ssp_err("Incomplete write of %d bytes",
							iRet);
					iRet = -EIO;
				}
				retry_count++;
				err_count++;
			} else {
				retry_count = 0;
				break;
			}
		}
		if (iRet < 0) {
			ssp_err("Writing MEM failed: %d, retry cont: %d",
					iRet, err_count);
			goto out;
		}
		remaining -= block;
		uPos += block;
		fw_addr += block;
#ifdef CONFIG_SENSORS_SSP_STM32F401CEY6B
		if (count++ == 50) {
			ssp_info("Updated %u bytes / %u bytes", uPos, uFSize);
			count = 0;
		}
#else
		if (count++ == 20) {
			ssp_info("Updated %u bytes / %u bytes", uPos, uFSize);
			count = 0;
		}
#endif
	}

	ssp_info("Firm up(UMS) success(%d bytes, retry %d)", uPos, err_count);
out:
err_fw_size:
	kfree(buff);
err_alloc:
	filp_close(fp, NULL);
err_open:
	set_fs(old_fs);

	return iRet;
}

static int fw_erase_stm(struct spi_device *spi)
{
	struct stm32fwu_spi_cmd cmd;
	struct stm32fwu_spi_cmd dummy_cmd;
	int ret;
	char buff[EXT_ER_DATA_LEN] = {0xff, 0xff, 0x00};
	ssp_dbgf();
	cmd.cmd = EXT_ER_COMMAND;
	cmd.xor_cmd = XOR_EXT_ER_COMMAND;
	cmd.timeout = DEF_ACKCMD_NUMBER;
#ifdef CONFIG_SENSORS_SSP_STM32F401CEY6B
	cmd.ack_pad = BL_DUMMY;
#else
	cmd.ack_pad = 0xFF;
#endif

	ret = stm32fwu_spi_send_cmd(spi, &cmd);
	if (ret != BL_ACK) {
		ssp_err("fw_erase failed - %d", ret);
		return ret;
	}

#ifdef CONFIG_SENSORS_SSP_STM32F401CEY6B
	ret = stm32fwu_spi_write(spi, buff, EXT_ER_DATA_LEN);
	if(ret < EXT_ER_DATA_LEN) {
		ssp_err("fw_erase write failed");
		return 0;
	}
#else
	if (cmd.ack_loops == 0)
		ret = stm32fwu_spi_write(spi, buff, EXT_ER_DATA_LEN);
	else
		ret = stm32fwu_spi_write(spi, buff, EXT_ER_DATA_LEN-1);

	if (ret < (EXT_ER_DATA_LEN - cmd.ack_loops))
		return -EPROTO;
#endif

	dummy_cmd.timeout = DEF_ACK_ERASE_NUMBER;

	ret = stm32fwu_spi_wait_for_ack(spi, &dummy_cmd, BL_ACK);
	if (ret == BL_ACK)
		return 0;
	else if (ret == BL_NACK)
		return -EPROTO;
	else
		return -ETIME;
}

static int load_kernel_fw_bootmode(struct spi_device *spi, const char *pFn)
{
	const struct firmware *fw = NULL;
	int remaining;
	unsigned int uPos = 0;
	unsigned int fw_addr = STM_APP_ADDR;
	int iRet;
	int block = STM_MAX_XFER_SIZE;
	int count = 0;
	int err_count = 0;
	int retry_count = 0;

	ssp_info("ssp_load_fw start!!");

	iRet = request_firmware(&fw, pFn, &spi->dev);
	if (iRet) {
		ssp_err("Unable to open firmware %s", pFn);
		return iRet;
	}

	remaining = fw->size;
	while (remaining > 0) {
		if (block > remaining)
			block = remaining;

		while (retry_count < 3) {
			iRet = fw_write_stm(spi, fw_addr, block, fw->data + uPos);
			if (iRet < block) {
				ssp_err("Err writing to addr 0x%08X", fw_addr);
				if (iRet < 0) {
					ssp_err("Error was %d", iRet);
				} else {
					ssp_err("Incomplete write of %d bytes",
							iRet);
					iRet = -EIO;
				}
				retry_count++;
				err_count++;
			} else {
				retry_count = 0;
				break;
			}
		}
		if (iRet < 0) {
			ssp_err("Writing MEM failed: %d, retry cont: %d",
					iRet, err_count);
			goto out_load_kernel;
		}

		remaining -= block;
		uPos += block;
		fw_addr += block;
		if (count++ == 20) {
			ssp_info("Updated %u bytes / %u bytes", uPos,
				(unsigned int)fw->size);
			count = 0;
		}
	}

	ssp_info("Firmware download is success(%d bytes, retry %d)",
			uPos, err_count);

out_load_kernel:
	release_firmware(fw);
	return iRet;
}

static int change_to_bootmode(struct ssp_data *data)
{
	int iCnt;
	int ret = SUCCESS;
	char syncb = BL_SPI_SOF;
	int ncount = 5;
	struct stm32fwu_spi_cmd dummy_cmd;
	ssp_dbgf();

	/* dummy_cmd.timeout = DEF_ACKCMD_NUMBER; */
	dummy_cmd.timeout = ncount;

	gpio_set_value_cansleep(data->rst, 0);
	usleep_range(4000, 4400);
	gpio_set_value_cansleep(data->rst, 1);
	usleep_range(45000, 47000);

	for (iCnt = 0; iCnt < 9; iCnt++) {
		gpio_set_value_cansleep(data->rst, 0);
		usleep_range(4000, 4400);
		gpio_set_value_cansleep(data->rst, 1);
		usleep_range(15000, 15500);
	}

	data->spi->mode = SPI_MODE_0;
	if (spi_setup(data->spi))
		ssp_err("failed to setup spi mode for boot");
	usleep_range(1000, 1100);

	msleep(30);

	while (ncount-- >= 0) {
		ret = stm32fwu_spi_write(data->spi, &syncb, 1);
#if SSP_STM_DEBUG
		ssp_info("stm32fwu_spi_write(sync byte) returned %d", ret);
#endif

#ifdef CONFIG_SENSORS_SSP_STM32F401CEY6B
		ret = stm32fwu_spi_wait_for_ack(data->spi, &dummy_cmd,
			BL_DUMMY);
#else
		ret = stm32fwu_spi_wait_for_ack(data->spi, &dummy_cmd, BL_ACK);
#endif
#if SSP_STM_DEBUG
		ssp_info("stm32fwu_spi_wait_for_ack returned %d (0x%x)",
			ret, ret);
#endif
		if (ret == BL_ACK)
			break;
	}
	return ret;
}


void toggle_mcu_reset(struct ssp_data *data)
{
	gpio_set_value_cansleep(data->rst, 0);

	usleep_range(1000, 1200);

	gpio_set_value_cansleep(data->rst, 1);
}

static int update_mcu_bin(struct ssp_data *data, int iBinType)
{
	int retry = BLMODE_RETRYCOUNT;
	int iRet = SUCCESS;
	struct stm32fwu_spi_cmd cmd;

	cmd.cmd = GO_COMMAND;
	cmd.xor_cmd = XOR_GO_COMMAND;
	cmd.timeout = 1000;
	cmd.ack_pad = (u8)((STM_APP_ADDR >> 24) & 0xFF);
	ssp_info("update_mcu_bin\n");

	/* 1. Start system boot mode */
	do {
		iRet = change_to_bootmode(data);
		ssp_info("bootmode %d retry: %d", iRet, 3 - retry);
	} while (retry-- > 0 && iRet != BL_ACK);

	if(iRet != BL_ACK) {
		ssp_errf("change_to_bootmode %d", iRet);
		return iRet;
	}

	/* 2. Flash erase all */
	iRet = fw_erase_stm(data->spi);
	if (iRet < 0) {
		ssp_errf("fw_erase_stm %d", iRet);
		return iRet;
	}

	switch (iBinType) {
	case KERNEL_BINARY:
		if (data->fw_name)
			iRet = load_kernel_fw_bootmode(data->spi, data->fw_name);
		else
			iRet = load_kernel_fw_bootmode(data->spi, BL_FW_NAME);
		break;
	case KERNEL_CRASHED_BINARY:
		iRet = load_kernel_fw_bootmode(data->spi,
				BL_CRASHED_FW_NAME);
		break;
	case UMS_BINARY:
		if (data->ums_fw_name)
			iRet = load_ums_fw_bootmode(data->spi, data->ums_fw_name);
		else
			iRet = load_ums_fw_bootmode(data->spi, BL_UMS_FW_NAME);
		break;
	default:
		ssp_err("binary type error!!");
	}

	/* STM : GO USER ADDR */
	stm32fwu_spi_send_cmd(data->spi, &cmd);
#ifdef CONFIG_SENSORS_SSP_STM32F401CEY6B
	send_addr(data->spi, STM_APP_ADDR, 0);
#else
	if (cmd.ack_loops > 0)
		send_addr(data->spi, STM_APP_ADDR, 1);
	else
		send_addr(data->spi, STM_APP_ADDR, 0);
#endif

	data->spi->mode = SPI_MODE_1;
	if (spi_setup(data->spi))
		ssp_err("failed to setup spi mode for app");
	usleep_range(1000, 1100);

	return iRet;
}

int forced_to_download_binary(struct ssp_data *data, int iBinType)
{
	int iRet = 0;
	int retry = 3;

	ssp_infof("mcu binany update!");

	ssp_enable(data, false);

	data->fw_dl_state = FW_DL_STATE_DOWNLOADING;
	ssp_infof("DL state = %d", data->fw_dl_state);

	data->spi->max_speed_hz = BOOT_SPI_HZ;
	if (spi_setup(data->spi))
		ssp_err("failed to setup spi for ssp_boot");
	do {
		ssp_info("%d try", 3 - retry);
		iRet = update_mcu_bin(data, iBinType);
	} while (retry -- > 0 && iRet < 0);

	data->spi->max_speed_hz = NORM_SPI_HZ;

	if (spi_setup(data->spi))
		ssp_err("failed to setup spi for ssp_norm");
	if (iRet < 0) {
		ssp_infof("update_mcu_bin failed!");
		goto out;
	}

	data->fw_dl_state = FW_DL_STATE_SYNC;
	ssp_infof("DL state = %d", data->fw_dl_state);
	ssp_enable(data, true);

	if (atomic_read(&data->apShutdownProgress)) {
		ssp_infof("AP shutdown is in progress, skip open cal data");
	} else {
		/* we should reload cal data after
		updating firmware on boooting */
#ifdef CONFIG_SENSORS_SSP_ACCELEROMETER_SENSOR
		accel_open_calibration(data);
#endif
#ifdef CONFIG_SENSORS_SSP_GYRO_SENSOR
		gyro_open_calibration(data);
#endif
#ifdef CONFIG_SENSORS_SSP_PRESSURE_SENSOR
		pressure_open_calibration(data);
#endif
#ifdef CONFIG_SENSORS_SSP_PROXIMITY_SENSOR
		proximity_open_calibration(data);
#endif
#ifdef CONFIG_SENSORS_SSP_HRM_SENSOR
		hrm_open_calibration(data);
#endif
	}

	data->fw_dl_state = FW_DL_STATE_DONE;
	ssp_infof("DL state = %d", data->fw_dl_state);

	iRet = SUCCESS;
out:
	return iRet;
}

int check_fwbl(struct ssp_data *data)
{

	unsigned int fw_revision;

	fw_revision = SSP_FIRMWARE_REVISION_STM;

	data->uCurFirmRev = get_firmware_rev(data);

	if ((data->uCurFirmRev == SSP_INVALID_REVISION)
			|| (data->uCurFirmRev == SSP_INVALID_REVISION2)) {
		data->uCurFirmRev = SSP_INVALID_REVISION;
		ssp_err("SSP_INVALID_REVISION");
		return FW_DL_STATE_NEED_TO_SCHEDULE;
	} else {
		if (data->uCurFirmRev != fw_revision) {
			ssp_info("MCU Firm Rev : Old = %8u, New = %8u",
					data->uCurFirmRev, fw_revision);

			return FW_DL_STATE_NEED_TO_SCHEDULE;
		}
		ssp_info("MCU Firm Rev : Old = %8u, New = %8u",
				data->uCurFirmRev, fw_revision);
	}

	return FW_DL_STATE_NONE;
}
