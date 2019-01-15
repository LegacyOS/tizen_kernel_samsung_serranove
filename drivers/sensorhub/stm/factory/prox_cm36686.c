/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
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
#include "../ssp.h"

#define	VENDOR		"CAPELLA"
#define	CHIP_ID		"CM36686"

#define CANCELATION_FILE_PATH	"/csa/sensor/prox_cal_data"

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/
#define CANCEL_HI_THD		34
#define CANCEL_LOW_THD	30

static ssize_t prox_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t prox_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_ID);
}

static ssize_t proximity_avg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
		data->buf[PROXIMITY_RAW].prox_raw[1],
		data->buf[PROXIMITY_RAW].prox_raw[2],
		data->buf[PROXIMITY_RAW].prox_raw[3]);
}

void set_proximity_threshold(struct ssp_data *data)
{
	int iRet = 0;
	struct ssp_msg *msg;

	if (!(data->uSensorState & (1 << PROXIMITY_SENSOR))) {
		ssp_infof("Skip this function!!!"\
			", proximity sensor is not connected(0x%x)",
			data->uSensorState);
		return;
	}

	msg= kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return;
	}
	msg->cmd = MSG2SSP_AP_SENSOR_PROXTHRESHOLD;
	msg->length = 4;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char*) kzalloc(4, GFP_KERNEL);
	msg->free_buffer = 1;

	ssp_infof("SENSOR_PROXTHRESHOL");

	msg->buffer[0] = (data->uProxHiThresh & 0xFF00) >> 8;
	msg->buffer[1] = (data->uProxHiThresh & 0x00FF);
	msg->buffer[2] = (data->uProxLoThresh & 0xFF00) >> 8;
	msg->buffer[3] = (data->uProxLoThresh & 0x00FF);

	iRet = ssp_spi_async(data, msg);

	if (iRet != SUCCESS) {
		ssp_errf("SENSOR_PROXTHRESHOLD CMD fail %d", iRet);
		return;
	}

	ssp_infof("Proximity Threshold - %u, %u",
		data->uProxHiThresh, data->uProxLoThresh);
}

static ssize_t proximity_avg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	char chTempbuf[4] = { 0 };
	int iRet;
	int64_t dEnable;
	struct ssp_data *data = dev_get_drvdata(dev);

	s32 dMsDelay = 20;
	memcpy(&chTempbuf[0], &dMsDelay, 4);

	iRet = kstrtoll(buf, 10, &dEnable);
	if (iRet < 0)
		return iRet;

	if (dEnable) {
		send_instruction(data, ADD_SENSOR, PROXIMITY_RAW, chTempbuf, 4);
		data->bProximityRawEnabled = true;
	} else {
		send_instruction(data, REMOVE_SENSOR, PROXIMITY_RAW,
			chTempbuf, 4);
		data->bProximityRawEnabled = false;
	}

	return size;
}

static u16 get_proximity_rawdata(struct ssp_data *data)
{
	u16 uRowdata = 0;
	char chTempbuf[9] = { 0 };

	s32 dMsDelay = 20;
	memcpy(&chTempbuf[0], &dMsDelay, 4);
	memcpy(&chTempbuf[4], &data->batchLatencyBuf[PROXIMITY_RAW], 4);
	chTempbuf[8] = data->batchOptBuf[PROXIMITY_RAW];

	if (data->bProximityRawEnabled == false) {
		send_instruction(data, ADD_SENSOR, PROXIMITY_RAW, chTempbuf, 9);
		msleep(200);
		uRowdata = data->buf[PROXIMITY_RAW].prox_raw[0];
		send_instruction(data, REMOVE_SENSOR, PROXIMITY_RAW,
			chTempbuf, 4);
	} else {
		uRowdata = data->buf[PROXIMITY_RAW].prox_raw[0];
	}

	return uRowdata;
}

static ssize_t proximity_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", get_proximity_rawdata(data));
}

static ssize_t proximity_raw_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", get_proximity_rawdata(data));
}

int proximity_open_calibration(struct ssp_data *data)
{
	int iRet = 0;
	mm_segment_t old_fs;
	struct file *cancel_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cancel_filp = filp_open(CANCELATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cancel_filp)) {
		iRet = PTR_ERR(cancel_filp);
		if (iRet != -ENOENT)
			ssp_errf("Can't open cancelation file");
		set_fs(old_fs);
		goto exit;
	}

	iRet = cancel_filp->f_op->read(cancel_filp,
		(u8 *)&data->uProxCanc, sizeof(unsigned int), &cancel_filp->f_pos);
	if (iRet != sizeof(u8)) {
		ssp_errf("Can't read the cancel data");
		iRet = -EIO;
	}

	ssp_infof("Prox Cancel=%d", data->uProxCanc);
	if (data->uProxCanc != 0) {
		/*If there is an offset cal data. */
		data->uProxHiThresh =
			data->uProxHiThresh_default + data->uProxCanc;
		data->uProxLoThresh =
			data->uProxLoThresh_default + data->uProxCanc;
	}

	ssp_info("proximity ps_canc = %d, ps_thresh hi-%d lo-%d",
		data->uProxCanc, data->uProxHiThresh, data->uProxLoThresh);

	filp_close(cancel_filp, current->files);
	set_fs(old_fs);

exit:
	return iRet;
}

static int proximity_store_cancelation(struct ssp_data *data, int iCalCMD)
{
	int iRet = 0;
	mm_segment_t old_fs;
	struct file *cancel_filp = NULL;

	if (iCalCMD) {
		data->uProxCanc = get_proximity_rawdata(data);
		data->uProxHiThresh =
			data->uProxHiThresh_default + data->uProxCanc;
		data->uProxLoThresh =
			data->uProxLoThresh_default + data->uProxCanc;
	} else {
		data->uProxHiThresh = data->uProxHiThresh_default;
		data->uProxLoThresh = data->uProxLoThresh_default;
		data->uProxCanc = 0;
	}

	if (iRet != ERROR)
		set_proximity_threshold(data);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cancel_filp = filp_open(CANCELATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY | O_SYNC, 0666);
	if (IS_ERR(cancel_filp)) {
		ssp_errf("Can't open cancelation file");
		set_fs(old_fs);
		iRet = PTR_ERR(cancel_filp);
		return iRet;
	}

	iRet = cancel_filp->f_op->write(cancel_filp, (u8 *)&data->uProxCanc,
		sizeof(unsigned int), &cancel_filp->f_pos);
	if (iRet != sizeof(unsigned int)) {
		ssp_errf("Can't write the cancel data to file");
		iRet = -EIO;
	}

	filp_close(cancel_filp, current->files);
	set_fs(old_fs);

	return iRet;
}

static ssize_t proximity_cancel_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_infof("uProxThresh : hi-%u lo-%u, uProxCanc = %u",
		data->uProxHiThresh, data->uProxLoThresh, data->uProxCanc);

	return snprintf(buf, PAGE_SIZE, "%u,%u,%u\n", data->uProxCanc,
		data->uProxHiThresh, data->uProxLoThresh);
}

static ssize_t proximity_cancel_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int iCalCMD = 0, iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "1")) /* calibrate cancelation value */
		iCalCMD = 1;
	else if (sysfs_streq(buf, "0")) /* reset cancelation value */
		iCalCMD = 0;
	else {
		ssp_errf("invalid value %d", *buf);
		return -EINVAL;
	}

	iRet = proximity_store_cancelation(data, iCalCMD);
	if (iRet < 0) {
		ssp_errf("proximity_store_cancelation() failed");
		return iRet;
	}

	ssp_infof("%u", iCalCMD);
	return size;
}

static ssize_t proximity_thresh_high_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_infof("uProxThresh = hi-%u, lo-%u",
		data->uProxHiThresh, data->uProxLoThresh);

	return snprintf(buf, PAGE_SIZE, "%u,%u\n", data->uProxHiThresh,
		data->uProxLoThresh);
}

static ssize_t proximity_thresh_high_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	u16 uNewThresh;
	int iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	iRet = kstrtou16(buf, 10, &uNewThresh);
	if (iRet < 0)
		ssp_errf("kstrtoint failed.(%d)", iRet);
	else {
		if(uNewThresh & 0xfc00)
			ssp_errf("allow 10bits.(%d)", uNewThresh);
		else {
			uNewThresh &= 0x03ff;
			data->uProxHiThresh = uNewThresh;
			set_proximity_threshold(data);
		}
	}

	ssp_infof("new prox threshold : hi - %u, lo - %u",
		data->uProxHiThresh, data->uProxLoThresh);

	return size;
}

static ssize_t proximity_thresh_low_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_infof("uProxThresh = hi - %u, lo - %u",
		data->uProxHiThresh, data->uProxLoThresh);

	return snprintf(buf, PAGE_SIZE, "%u,%u\n", data->uProxHiThresh,
		data->uProxLoThresh);
}

static ssize_t proximity_thresh_low_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	u16 uNewThresh;
	int iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	iRet = kstrtou16(buf, 10, &uNewThresh);
	if (iRet < 0)
		ssp_errf("kstrtoint failed.(%d)", iRet);
	else {
		if(uNewThresh & 0xfc00)
			ssp_errf("allow 10bits.(%d)", uNewThresh);
		else {
			uNewThresh &= 0x03ff;
			data->uProxLoThresh = uNewThresh;
			set_proximity_threshold(data);
		}
	}

	ssp_infof("new prox threshold : hi - %u, lo - %u",
		data->uProxHiThresh, data->uProxLoThresh);

	return size;
}

static ssize_t proximity_cancel_pass_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_infof("%u", data->uProxCalResult);
	return snprintf(buf, PAGE_SIZE, "%u\n", data->uProxCalResult);
}

static DEVICE_ATTR(vendor, S_IRUGO, prox_vendor_show, NULL);
static DEVICE_ATTR(name, S_IRUGO, prox_name_show, NULL);
static DEVICE_ATTR(state, S_IRUGO, proximity_state_show, NULL);
static DEVICE_ATTR(raw_data, S_IRUGO, proximity_raw_data_show, NULL);
static DEVICE_ATTR(prox_avg, S_IRUGO | S_IWUSR | S_IWGRP,
	proximity_avg_show, proximity_avg_store);
static DEVICE_ATTR(prox_cal, S_IRUGO | S_IWUSR | S_IWGRP,
	proximity_cancel_show, proximity_cancel_store);
static DEVICE_ATTR(thresh_high, S_IRUGO | S_IWUSR | S_IWGRP,
	proximity_thresh_high_show, proximity_thresh_high_store);
static DEVICE_ATTR(thresh_low, S_IRUGO | S_IWUSR | S_IWGRP,
	proximity_thresh_low_show, proximity_thresh_low_store);
static DEVICE_ATTR(prox_offset_pass, S_IRUGO, proximity_cancel_pass_show, NULL);

static struct device_attribute *prox_attrs[] = {
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_state,
	&dev_attr_raw_data,
	&dev_attr_prox_avg,
	&dev_attr_prox_cal,
	&dev_attr_thresh_high,
	&dev_attr_thresh_low,
	&dev_attr_prox_offset_pass,
	NULL,
};

void initialize_prox_factorytest(struct ssp_data *data)
{
	sensors_register(data->prox_device, data,
		prox_attrs, "proximity_sensor");
}

void remove_prox_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->prox_device, prox_attrs);
}
