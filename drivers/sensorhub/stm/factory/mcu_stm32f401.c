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

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#ifdef CONFIG_SENSORS_SSP_STM32F401CEY6B
#define MODEL_NAME		"STM32F401CEY6B"
#else
#define MODEL_NAME		"STM32F401CCY6B"
#endif
#define SMART_ALERT_MOTION	8

ssize_t mcu_revision_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "ST01%u,ST01%u\n", data->uCurFirmRev,
		get_module_rev(data));
}

ssize_t mcu_model_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", MODEL_NAME);
}

ssize_t mcu_update_kernel_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bSuccess = false;
	int iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_infof("mcu binany update!");

	iRet = forced_to_download_binary(data, UMS_BINARY);
	if (iRet == SUCCESS) {
		bSuccess = true;
		goto out;
	}

	iRet = forced_to_download_binary(data, KERNEL_BINARY);
	if (iRet == SUCCESS)
		bSuccess = true;
	else
		bSuccess = false;
out:
	return snprintf(buf, PAGE_SIZE, "%s\n", (bSuccess ? "OK" : "NG"));
}

ssize_t mcu_update_kernel_crashed_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bSuccess = false;
	int iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_infof("mcu binany update!");

	iRet = forced_to_download_binary(data, UMS_BINARY);
	if (iRet == SUCCESS) {
		bSuccess = true;
		goto out;
	}

	iRet = forced_to_download_binary(data, KERNEL_CRASHED_BINARY);
	if (iRet == SUCCESS)
		bSuccess = true;
	else
		bSuccess = false;
out:
	return snprintf(buf, PAGE_SIZE, "%s\n", (bSuccess ? "OK" : "NG"));
}

ssize_t mcu_update_ums_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bSuccess = false;
	int iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_infof("mcu binany update!");

	iRet = forced_to_download_binary(data, UMS_BINARY);
	if (iRet == SUCCESS)
		bSuccess = true;
	else
		bSuccess = false;

	return snprintf(buf, PAGE_SIZE, "%s\n", (bSuccess ? "OK" : "NG"));
}

ssize_t mcu_sensor_state(struct device *dev, struct device_attribute *attr,
		char *buf) {
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->uSensorState);
}

ssize_t mcu_reset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	reset_mcu(data);

	return snprintf(buf, PAGE_SIZE, "OK\n");
}

ssize_t mcu_dump_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	struct ssp_data *data = dev_get_drvdata(dev);
	int status = 1, iDelaycnt = 0;

	data->bDumping = true;
	set_big_data_start(data, BIG_TYPE_DUMP, 0);
	msleep(300);
	while (data->bDumping) {
		mdelay(10);
		if (iDelaycnt++ > 1000) {
			status = 0;
			break;
		}
	}
	return snprintf(buf, PAGE_SIZE, "%s\n", status ? "OK" : "NG");
}

static char buffer[FACTORY_DATA_MAX];

ssize_t mcu_factorytest_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int iRet = 0;
	struct ssp_msg *msg;

	if (sysfs_streq(buf, "1")) {
		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		msg->cmd = MCU_FACTORY;
		msg->length = 5;
		msg->options = AP2HUB_READ;
		msg->buffer = buffer;
		msg->free_buffer = 0;

		memset(msg->buffer, 0, 5);

		iRet = ssp_spi_async(data, msg);

	} else {
		ssp_errf("invalid value %d", *buf);
		return -EINVAL;
	}

	ssp_info("MCU Factory Test Start! - %d", iRet);

	return size;
}

ssize_t mcu_factorytest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bMcuTestSuccessed = false;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (data->bSspShutdown == true) {
		ssp_infof("MCU Bin is crashed");
		return snprintf(buf, PAGE_SIZE, "NG,NG,NG\n");
	}

	ssp_infof("MCU Factory Test Data : %u, %u, %u, %u, %u",
		buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);

		/* system clock, RTC, I2C Master, I2C Slave, externel pin */
	if ((buffer[0] == SUCCESS)
			&& (buffer[1] == SUCCESS)
			&& (buffer[2] == SUCCESS)
			&& (buffer[3] == SUCCESS)
			&& (buffer[4] == SUCCESS))
		bMcuTestSuccessed = true;

	ssp_infof("MCU Factory Test Result - %s, %s, %s\n", MODEL_NAME,
		(bMcuTestSuccessed ? "OK" : "NG"), "OK");

	return snprintf(buf, PAGE_SIZE, "%s,%s,%s\n", MODEL_NAME,
		(bMcuTestSuccessed ? "OK" : "NG"), "OK");
}

ssize_t mcu_sleep_factorytest_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int iRet = 0;
	struct ssp_msg *msg;

	if (sysfs_streq(buf, "1")) {
		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		msg->cmd = MCU_SLEEP_FACTORY;
		msg->length = FACTORY_DATA_MAX;
		msg->options = AP2HUB_READ;
		msg->buffer = buffer;
		msg->free_buffer = 0;

		iRet = ssp_spi_async(data, msg);

	} else {
		ssp_errf("invalid value %d", *buf);
		return -EINVAL;
	}

	ssp_infof("MCU Sleep Factory Test Start! - %d", 1);

	return size;
}

ssize_t mcu_sleep_factorytest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int iDataIdx, iSensorData = 0;
	struct ssp_data *data = dev_get_drvdata(dev);
	struct sensor_value *fsb;
	u16 chLength = 0;
	int ret = 0;
	
	fsb = kzalloc(sizeof(struct sensor_value)*SENSOR_MAX, GFP_KERNEL);

	memcpy(&chLength, buffer, 2);
	memset(fsb, 0, sizeof(struct sensor_value) * SENSOR_MAX);

	for (iDataIdx = 2; iDataIdx < chLength + 2;) {
		iSensorData = (int)buffer[iDataIdx++];

		if ((iSensorData < 0) ||
			(iSensorData >= (SENSOR_MAX - 1))) {
			ssp_errf("Mcu data frame error %d", iSensorData);
			goto exit;
		}
#ifdef SSP_USE_IIO_BATCH
		get_sensordata(data, (char *)buffer, &iDataIdx,
				iSensorData, &(fsb[iSensorData]));
#else
		data->get_sensor_data[iSensorData]((char *)buffer,
			&iDataIdx, &(fsb[iSensorData]));
#endif
	}

	fsb[PRESSURE_SENSOR].pressure -= data->iPressureCal;

exit:
	ssp_infof("Result");
#ifdef CONFIG_SENSORS_SSP_ACCELEROMETER_SENSOR
	ssp_infof("accel %d,%d,%d", fsb[ACCELEROMETER_SENSOR].x,
		fsb[ACCELEROMETER_SENSOR].y, fsb[ACCELEROMETER_SENSOR].z);
#endif
#ifdef CONFIG_SENSORS_SSP_GYRO_SENSOR
	ssp_infof("gyro %d,%d,%d", fsb[GYROSCOPE_SENSOR].x,
		fsb[GYROSCOPE_SENSOR].y, fsb[GYROSCOPE_SENSOR].z);
#endif
#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
	ssp_infof("mag %d,%d,%d", fsb[GEOMAGNETIC_SENSOR].cal_x,
		fsb[GEOMAGNETIC_SENSOR].cal_y, fsb[GEOMAGNETIC_SENSOR].cal_z);
#endif
#ifdef CONFIG_SENSORS_SSP_PRESSURE_SENSOR
	ssp_dbg("baro %d,%d", fsb[PRESSURE_SENSOR].pressure,
		fsb[PRESSURE_SENSOR].temperature);
#endif
#ifdef CONFIG_SENSORS_SSP_GESTURE_SENSOR
	ssp_infof("ges %d,%d,%d,%d", fsb[GESTURE_SENSOR].data[0],
		fsb[GESTURE_SENSOR].data[1], fsb[GESTURE_SENSOR].data[2],
		fsb[GESTURE_SENSOR].data[3]);
#endif
#ifdef CONFIG_SENSORS_SSP_LIGHT_SENSOR
#if defined(CONFIG_SENSORS_SSP_AL3320)
	ssp_infof("[SSP]: light %u,%u", fsb[LIGHT_SENSOR].raw_high,
		fsb[LIGHT_SENSOR].raw_low);
#elif defined(CONFIG_SENSORS_SSP_CM36686)
	ssp_infof("[SSP]: light %u,%u", fsb[LIGHT_SENSOR].als,
		fsb[LIGHT_SENSOR].white);
#else
	ssp_infof("[SSP]: light %u,%u,%u,%u", fsb[LIGHT_SENSOR].r,
		fsb[LIGHT_SENSOR].g, fsb[LIGHT_SENSOR].b, fsb[LIGHT_SENSOR].w);
#endif
#endif
#ifdef CONFIG_SENSORS_SSP_PROXIMITY_SENSOR
	ssp_infof("[SSP]: prox %u,%u", fsb[PROXIMITY_SENSOR].prox,
		fsb[PROXIMITY_SENSOR].prox_ex);
#endif
#ifdef CONFIG_SENSORS_SSP_TEMP_HUMID_SENSOR
	ssp_infof("[SSP]: temp %d,%d,%d", fsb[TEMPERATURE_HUMIDITY_SENSOR].x,
		fsb[TEMPERATURE_HUMIDITY_SENSOR].y,
		fsb[TEMPERATURE_HUMIDITY_SENSOR].z);
#endif
#ifdef CONFIG_SENSORS_SSP_HRM_SENSOR
	ssp_infof("[SSP]: hrm_raw %d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
		fsb[BIO_HRM_RAW].ch_a_sum, fsb[BIO_HRM_RAW].ch_a_x1,
		fsb[BIO_HRM_RAW].ch_a_x2, fsb[BIO_HRM_RAW].ch_a_y1,
		fsb[BIO_HRM_RAW].ch_a_y2, fsb[BIO_HRM_RAW].ch_b_sum,
		fsb[BIO_HRM_RAW].ch_b_x1, fsb[BIO_HRM_RAW].ch_b_x2,
		fsb[BIO_HRM_RAW].ch_b_y1, fsb[BIO_HRM_RAW].ch_b_y2);
	ssp_infof("[SSP]: hrm_lib %d,%d,%d", fsb[BIO_HRM_LIB].hr,
		fsb[BIO_HRM_LIB].rri, fsb[BIO_HRM_RAW].snr);
#endif
#ifdef CONFIG_SENSORS_SSP_UV_SENSOR
	ssp_infof("[SSP]: uv %u", fsb[UV_SENSOR].uv_raw);
#endif
#ifdef CONFIG_SENSORS_SSP_PIR_SENSOR
	ssp_infof("[SSP]: pir %u %u %u %d", fsb[PIR_SENSOR].near_temp,
		fsb[PIR_SENSOR].near_adc, fsb[PIR_SENSOR].far_temp,
		fsb[PIR_SENSOR].far_adc);
#endif

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
		%d,%d,%d,%d,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\
		%d,%u,%u,%u,%u,%u\n"
#ifdef CONFIG_SENSORS_SSP_ACCELEROMETER_SENSOR
		, fsb[ACCELEROMETER_SENSOR].x, fsb[ACCELEROMETER_SENSOR].y,
		fsb[ACCELEROMETER_SENSOR].z
#else
		, 0, 0, 0
#endif
#ifdef CONFIG_SENSORS_SSP_GYRO_SENSOR
		, fsb[GYROSCOPE_SENSOR].x, fsb[GYROSCOPE_SENSOR].y,
		fsb[GYROSCOPE_SENSOR].z
#else
		, 0, 0, 0
#endif
#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
		, fsb[GEOMAGNETIC_SENSOR].cal_x, fsb[GEOMAGNETIC_SENSOR].cal_y,
		fsb[GEOMAGNETIC_SENSOR].cal_z
#else
		, 0, 0, 0
#endif
#ifdef CONFIG_SENSORS_SSP_PRESSURE_SENSOR
		, fsb[PRESSURE_SENSOR].pressure,
		fsb[PRESSURE_SENSOR].temperature
#else
		, 0, 0
#endif
#ifdef CONFIG_SENSORS_SSP_GESTURE_SENSOR
		, fsb[GESTURE_SENSOR].data[0], fsb[GESTURE_SENSOR].data[1],
		fsb[GESTURE_SENSOR].data[2], fsb[GESTURE_SENSOR].data[3]
#else
		, 0, 0, 0, 0
#endif
#ifdef CONFIG_SENSORS_SSP_PROXIMITY_SENSOR
		, fsb[PROXIMITY_SENSOR].prox_ex
#else
		, 0
#endif
#ifdef CONFIG_SENSORS_SSP_LIGHT_SENSOR
#if defined(CONFIG_SENSORS_SSP_AL3320)
		, fsb[LIGHT_SENSOR].raw_high, fsb[LIGHT_SENSOR].raw_low, 0, 0
#elif defined(CONFIG_SENSORS_SSP_CM36686)
		, fsb[LIGHT_SENSOR].als, fsb[LIGHT_SENSOR].white, 0, 0
#else
		, fsb[LIGHT_SENSOR].r, fsb[LIGHT_SENSOR].g, fsb[LIGHT_SENSOR].b,
		fsb[LIGHT_SENSOR].w
#endif
#else
		, 0, 0, 0, 0
#endif
#ifdef CONFIG_SENSORS_SSP_HRM_SENSOR
		, fsb[BIO_HRM_RAW].ch_a_sum, fsb[BIO_HRM_RAW].ch_a_x1,
		fsb[BIO_HRM_RAW].ch_a_x2, fsb[BIO_HRM_RAW].ch_a_y1,
		fsb[BIO_HRM_RAW].ch_a_y2, fsb[BIO_HRM_RAW].ch_b_sum,
		fsb[BIO_HRM_RAW].ch_b_x1, fsb[BIO_HRM_RAW].ch_b_x2,
		fsb[BIO_HRM_RAW].ch_b_y1, fsb[BIO_HRM_RAW].ch_b_y2
		, fsb[BIO_HRM_LIB].hr, fsb[BIO_HRM_LIB].rri, fsb[BIO_HRM_RAW].snr
#else
		, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
#endif
#ifdef CONFIG_SENSORS_SSP_UV_SENSOR
		, fsb[UV_SENSOR].uv_raw
#else
		, 0
#endif
#ifdef CONFIG_SENSORS_SSP_PIR_SENSOR
		, fsb[PIR_SENSOR].near_temp, fsb[PIR_SENSOR].near_adc,
		fsb[PIR_SENSOR].far_temp, fsb[PIR_SENSOR].far_adc
#else
		, 0, 0, 0, 0
#endif
		);

	kfree(fsb);

	return ret;
}

#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
int ssp_charging_motion(struct ssp_data *data, int iEnable)
{
	u8 uBuf[2] = {0, 0};

	if (iEnable == 1) {
		send_instruction(data, ADD_LIBRARY,
			SMART_ALERT_MOTION, uBuf, 2);
	} else {
		send_instruction(data, REMOVE_LIBRARY,
			SMART_ALERT_MOTION, uBuf, 2);
	}

	return 0;
}

int ssp_parse_motion(struct ssp_data *data, char *dataframe, int start, int end)
{
	int length = end - start;
	char *buf = dataframe + start;;

	if (length != 4)
		return FAIL;

	if ((buf[0] == 1) && (buf[1] == 1) && (buf[2] == SMART_ALERT_MOTION)) {
		ssp_infof("LP MODE WAKEUP");
		queue_work(data->lpm_motion_wq, &data->work_lpm_motion);
		//report_key_event(data);
		return SUCCESS;
	}

	return FAIL;
}

static void lpm_motion_work_func(struct work_struct *work)
{
	struct ssp_data *data =
		container_of(work, struct ssp_data, work_lpm_motion);

	input_event(data->motion_input_dev, EV_KEY, KEY_HOMEPAGE, 1);
	input_sync(data->motion_input_dev);
	ssp_charging_motion(data, 0);

	msleep(10);

	input_event(data->motion_input_dev, EV_KEY, KEY_HOMEPAGE, 0);
	input_sync(data->motion_input_dev);
	ssp_charging_motion(data, 1);

	wake_lock_timeout(&data->ssp_wake_lock, 3 * HZ);

}

int intialize_lpm_motion(struct ssp_data *data)
{
	data->lpm_motion_wq = create_singlethread_workqueue("ssp_lpm_motion_wq");
	if (!data->lpm_motion_wq)
		return ERROR;

	INIT_WORK(&data->work_lpm_motion, lpm_motion_work_func);
	return SUCCESS;
}
#endif
