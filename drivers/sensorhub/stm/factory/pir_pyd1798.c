/*
 *  Copyright (C) 2014, Samsung Electronics Co. Ltd. All Rights Reserved.
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
#define NAME	"PYD1798"
#define VENDOR	"PYRO-ELECTRIC"

void set_pir_threshold(struct ssp_data *data)
{
	int iRet = 0;
	struct ssp_msg *msg;
	u32 near_th = data->pir_standard_threshold;
	u32 far_th = data->pir_long_threshold;

	if (!(data->uSensorState & (1<<PIR_SENSOR))) {
		ssp_infof("Skip this function!!!"\
			", pir sensor is not connected(0x%x)",
			data->uSensorState);
		return;
	}

	msg= kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = MSG2SSP_AP_SENSOR_PIRTHRESHOLD;
	msg->length = 5;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char*) kzalloc(9, GFP_KERNEL);
	msg->free_buffer = 1;

	msg->buffer[0] = 0x00;
	msg->buffer[1] = (char)(near_th);
	msg->buffer[2] = ((char)(near_th >> 8) & 0xff);
	msg->buffer[3] = (char)(far_th);
	msg->buffer[4] = ((char)(far_th >> 8) & 0xff);

	iRet = ssp_spi_async(data, msg);

	if (iRet != SUCCESS) {
		ssp_errf("SENSOR_PIR_THRESHOLD CMD fail %d", iRet);
		return;
	}

	ssp_infof("pir Threshold - %u, %u", near_th, far_th);
}

static ssize_t pir_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", NAME);
}

static ssize_t pir_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t pir_standard_raw_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d %d\n",
			data->buf[PIR_SENSOR_RAW].near_temp,
			data->buf[PIR_SENSOR_RAW].near_adc);
}

static ssize_t pir_long_raw_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d %d\n",
			data->buf[PIR_SENSOR_RAW].far_temp,
			data->buf[PIR_SENSOR_RAW].far_adc);
}

static ssize_t pir_raw_data_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int64_t enable;
	u8 uBuf[9] = {0,};
	s32 dMsDelay = 10;

	if (kstrtoll(buf, 10, &enable) < 0)
		return -EINVAL;

	if (enable) {
		memcpy(&uBuf[0], &dMsDelay, 4);
		memcpy(&uBuf[4], &data->batchLatencyBuf[PIR_SENSOR_RAW], 4);
		uBuf[8] = data->batchOptBuf[PIR_SENSOR_RAW];

		send_instruction(data, ADD_SENSOR, PIR_SENSOR_RAW,
			uBuf, 9);
	} else
		send_instruction(data, REMOVE_SENSOR, PIR_SENSOR_RAW,
			uBuf, 4);

	return size;
}

static ssize_t pir_standard_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->pir_standard_threshold);
}

static ssize_t pir_standard_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int64_t new_threshold;

	if (kstrtoll(buf, 10, &new_threshold) < 0)
		return -EINVAL;

	data->pir_standard_threshold = (uint32_t)new_threshold;
	set_pir_threshold(data);

	return size;
}

static ssize_t pir_long_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->pir_long_threshold);
}

static ssize_t pir_long_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int64_t new_threshold;

	if (kstrtoll(buf, 10, &new_threshold) < 0)
		return -EINVAL;

	data->pir_long_threshold = (uint32_t)new_threshold;
	set_pir_threshold(data);

	return size;
}

static DEVICE_ATTR(name, S_IRUGO , pir_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO , pir_vendor_show, NULL);
static DEVICE_ATTR(raw_data_long, S_IRUGO | S_IWUSR | S_IWGRP,
	pir_long_raw_data_show, pir_raw_data_store);
static DEVICE_ATTR(raw_data_standard, S_IRUGO | S_IWUSR | S_IWGRP,
	pir_standard_raw_data_show, pir_raw_data_store);
static struct device_attribute dev_attr_standard_threshold
	= __ATTR(threshold, S_IRUGO | S_IWUSR | S_IWGRP,
	pir_standard_threshold_show, pir_standard_threshold_store);
static struct device_attribute dev_attr_long_threshold
	= __ATTR(threshold, S_IRUGO | S_IWUSR | S_IWGRP,
	pir_long_threshold_show, pir_long_threshold_store);

static struct device_attribute *pir_standard_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_raw_data_standard,
	&dev_attr_standard_threshold,
	NULL,
};

static struct device_attribute *pir_long_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_raw_data_long,
	&dev_attr_long_threshold,
	NULL,
};

void initialize_pir_factorytest(struct ssp_data *data)
{
	sensors_register(data->pir_standard_dev, data,
			pir_standard_attrs, "pir_standard_sensor");
	sensors_register(data->pir_long_dev,
			data, pir_long_attrs, "pir_long_sensor");
}
void remove_pir_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->pir_standard_dev, pir_standard_attrs);
	sensors_unregister(data->pir_long_dev, pir_long_attrs);
}
