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

#include "ssp_data.h"

#ifdef SSP_USE_IIO_BATCH
/* will be enabled after iio implemented instead of input event*/
static void generate_data(struct ssp_data *data, struct sensor_value *sensorsdata,
						int sensor, u64 timestamp)
{
	u64 move_timestamp = data->lastTimestamp[sensor];
	if ((sensor != PROXIMITY_SENSOR) && (sensor != GESTURE_SENSOR)
		&& (sensor != STEP_DETECTOR) && (sensor != SIG_MOTION_SENSOR)
		&& (sensor != STEP_COUNTER)) {
		while ((move_timestamp * 10 + data->adDelayBuf[sensor] * 15) < (timestamp * 10)) {
			move_timestamp += data->adDelayBuf[sensor];
			sensorsdata->timestamp = move_timestamp;
			report_sensordata(data, sensor, sensorsdata);
		}
	}
}

static void get_timestamp(struct ssp_data *data, char *dataframe,
		int *index, struct sensor_value *sensorsdata,
		struct ssp_time_diff *sensortime, int sensor)
{
	if (sensortime->batch_mode == BATCH_MODE_RUN) {
		if (sensortime->batch_count == sensortime->batch_count_fixed) {
			if (sensortime->time_diff == data->adDelayBuf[sensor]) {
				generate_data(data, sensorsdata, sensor,
						(data->timestamp - data->adDelayBuf[sensor] * (sensortime->batch_count_fixed - 1)));
			}
			sensorsdata->timestamp = data->timestamp - ((sensortime->batch_count - 1) * sensortime->time_diff);
		} else {
			if (sensortime->batch_count > 1)
				sensorsdata->timestamp = data->timestamp - ((sensortime->batch_count - 1) * sensortime->time_diff);
			else
				sensorsdata->timestamp = data->timestamp;
		}
	} else {
		if (((sensortime->irq_diff * 10) > (data->adDelayBuf[sensor] * 18))
			&& ((sensortime->irq_diff * 10) < (data->adDelayBuf[sensor] * 100))) {
			generate_data(data, sensorsdata, sensor, data->timestamp);
		}
		sensorsdata->timestamp = data->timestamp;
	}
	*index += 4;
}

void get_sensordata(struct ssp_data *data, char *dataframe,
		int *index, int sensor, struct sensor_value *sensordata)
{
	memcpy(sensordata, dataframe + *index, data->data_len[sensor]);
	*index += data->data_len[sensor];
}
#else
static void get_timestamp(struct ssp_data *data, char *dataframe,
		int *index, struct sensor_value *sensorsdata)
{
	s32 otimestamp = 0;
	s64 ctimestamp = 0;

	memcpy(&otimestamp, dataframe + *index, 4);
	*index += 4;

	ctimestamp = (s64) otimestamp * 1000000;
	sensorsdata->timestamp = data->timestamp + ctimestamp;
}
#endif

#if defined(CONFIG_SENSORS_SSP_ACCELEROMETER_SENSOR) \
	|| defined(CONFIG_SENSORS_SSP_GYRO_SENSOR)
static void get_3axis_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 6);
	*iDataIdx += 6;
}
#endif

#ifdef CONFIG_SENSORS_SSP_GYRO_SENSOR
static void get_uncalib_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 12);
	*iDataIdx += 12;
}
#endif

#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
static void get_geomagnetic_uncaldata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 12);
	*iDataIdx += 12;
}

static void get_geomagnetic_rawdata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 6);
	*iDataIdx += 6;
}

static void get_geomagnetic_caldata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 7);
	*iDataIdx += 7;
}
#endif

#ifdef CONFIG_SENSORS_SSP_LIGHT_SENSOR
static void get_light_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
#if defined(CONFIG_SENSORS_SSP_AL3320)
	memcpy(sensorsdata, dataframe + *iDataIdx, 2);
	*iDataIdx += 2;
#elif defined(CONFIG_SENSORS_SSP_CM36686)
	memcpy(sensorsdata, dataframe + *iDataIdx, 4);
	*iDataIdx += 4;
#else
	memcpy(sensorsdata, dataframe + *iDataIdx, 8);
	*iDataIdx += 8;
#endif
}
#endif

#ifdef CONFIG_SENSORS_SSP_PRESSURE_SENSOR
static void get_pressure_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	s16 temperature = 0;
	memcpy(&sensorsdata->pressure, dataframe + *iDataIdx, 4);
	memcpy(&sensorsdata->temperature, dataframe + *iDataIdx + 4, 2);
	*iDataIdx += 6;
}
#endif

#ifdef CONFIG_SENSORS_SSP_GESTURE_SENSOR
static void get_gesture_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 20);
	*iDataIdx += 20;
}
#endif

#ifdef CONFIG_SENSORS_SSP_PROXIMITY_SENSOR
static void get_proximity_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	sensorsdata->prox = dataframe[*iDataIdx];
	memcpy(&sensorsdata->prox_ex, dataframe + *iDataIdx + 1, 2);
	*iDataIdx += 3;
}

static void get_proximity_rawdata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(&sensorsdata->prox_raw[0], dataframe + *iDataIdx, 2);
	*iDataIdx += 2;
}
#endif

#ifdef CONFIG_SENSORS_SSP_TEMP_HUMID_SENSOR
static void get_temp_humidity_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memset(&sensorsdata->data[2], 0, 2);
	memcpy(sensorsdata, dataframe + *iDataIdx, 5);
	*iDataIdx += 5;
}
#endif

#ifdef CONFIG_SENSORS_SSP_ROT_VECTOR_SENSOR
static void get_rot_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 17);
	*iDataIdx += 17;
}
#endif

#ifdef CONFIG_SENSORS_SSP_HRM_SENSOR
static void get_hrm_raw_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 40);
	*iDataIdx += 40;
}

static void get_hrm_raw_fac_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 64);
	*iDataIdx += 64;
}

static void get_hrm_lib_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 8);
	*iDataIdx += 8;
}
#endif

#ifdef CONFIG_SENSORS_SSP_UV_SENSOR
static void get_uv_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 1);
	*iDataIdx += 1;
}
#endif

#ifdef CONFIG_SENSORS_SSP_PIR_SENSOR
static void get_pir_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 1);
	*iDataIdx += 1;
}

static void get_pir_raw_sensordata(char *dataframe, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	memcpy(sensorsdata, dataframe + *iDataIdx, 8);
	*iDataIdx += 8;
}
#endif

int handle_big_data(struct ssp_data *data, char *dataframe, int *pDataIdx)
{
	u8 bigType = 0;
	struct ssp_big *big = kzalloc(sizeof(*big), GFP_KERNEL);
	big->data = data;
	bigType = dataframe[(*pDataIdx)++];
	memcpy(&big->length, dataframe + *pDataIdx, 4);
	*pDataIdx += 4;
	memcpy(&big->addr, dataframe + *pDataIdx, 4);
	*pDataIdx += 4;

	if (bigType >= BIG_TYPE_MAX) {
		kfree(big);
		return FAIL;
	}

	INIT_WORK(&big->work, data->ssp_big_task[bigType]);
	queue_work(data->debug_wq, &big->work);
	return SUCCESS;
}

void refresh_task(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct delayed_work *)work,
			struct ssp_data, work_refresh);

	if (data->bSspShutdown == true) {
		ssp_errf("ssp already shutdown");
		return;
	}

	wake_lock(&data->ssp_wake_lock);
	ssp_errf();
	data->uResetCnt++;

	if (initialize_mcu(data) > 0) {
		sync_sensor_state(data);
		ssp_sensorhub_report_notice(data, MSG2SSP_AP_STATUS_RESET);
		if (data->uLastAPState != 0)
			ssp_send_cmd(data, data->uLastAPState, 0);
		if (data->uLastResumeState != 0)
			ssp_send_cmd(data, data->uLastResumeState, 0);
		data->uTimeOutCnt = 0;
	} else
		data->uSensorState = 0;

	wake_unlock(&data->ssp_wake_lock);
}

int queue_refresh_task(struct ssp_data *data, int delay)
{
#if 0
	cancel_delayed_work_sync(&data->work_refresh);

	INIT_DELAYED_WORK(&data->work_refresh, refresh_task);
#endif
	queue_delayed_work(data->debug_wq, &data->work_refresh,
			msecs_to_jiffies(delay));
	return SUCCESS;
}

int parse_dataframe(struct ssp_data *data, char *dataframe, int frame_len)
{
	struct sensor_value sensorsdata;
#ifdef SSP_USE_IIO_BATCH
	struct ssp_time_diff sensortime;
#endif
	int sensor, index;
	u16 length = 0;
#ifdef CONFIG_SENSORS_SSP_GYRO_SENSOR
	s16 caldata[3] = { 0, };
#endif

	memset(&sensorsdata, 0, sizeof(sensorsdata));

	for (index = 0; index < frame_len;) {
		switch (dataframe[index++]) {
		case MSG2AP_INST_BYPASS_DATA:
			sensor = dataframe[index++];
			if ((sensor < 0) || (sensor >= SENSOR_MAX)) {
				ssp_errf("Mcu bypass dataframe err %d", sensor);
				return ERROR;
			}
#ifdef SSP_USE_IIO_BATCH
/* will be enabled after iio and batch mode implemented */
			memcpy(&length, dataframe + index, 2);
			index += 2;
			sensortime.batch_count = sensortime.batch_count_fixed = length;
			sensortime.batch_mode = length > 1 ? BATCH_MODE_RUN : BATCH_MODE_NONE;
			sensortime.irq_diff = data->timestamp - data->lastTimestamp[sensor];

			if (sensortime.batch_mode == BATCH_MODE_RUN) {
				if (data->reportedData[sensor] == true) {
					u64 time;
					sensortime.time_diff = div64_long((s64)(data->timestamp - data->lastTimestamp[sensor]), (s64)length);
					if (length > 8)
						time = data->adDelayBuf[sensor] * 18;
					else if (length > 4)
						time = data->adDelayBuf[sensor] * 25;
					else if (length > 2)
						time = data->adDelayBuf[sensor] * 50;
					else
						time = data->adDelayBuf[sensor] * 100;
					if ((sensortime.time_diff * 10) > time) {
						data->lastTimestamp[sensor] = data->timestamp - (data->adDelayBuf[sensor] * length);
						sensortime.time_diff = data->adDelayBuf[sensor];
					} else {
						time = data->adDelayBuf[sensor] * 18;
						if ((sensortime.time_diff * 10) > time)
							sensortime.time_diff = data->adDelayBuf[sensor];
					}
				} else {
					if (data->lastTimestamp[sensor] < (data->timestamp - (data->adDelayBuf[sensor] * length))) {
						data->lastTimestamp[sensor] = data->timestamp - (data->adDelayBuf[sensor] * length);
						sensortime.time_diff = data->adDelayBuf[sensor];
					} else
						sensortime.time_diff = div64_long((s64)(data->timestamp - data->lastTimestamp[sensor]), (s64)length);
				}
			} else {
				if (data->reportedData[sensor] == false)
					sensortime.irq_diff = data->adDelayBuf[sensor];
			}

			do {
				get_sensordata(data, dataframe, &index,
					sensor, &sensorsdata);

				get_timestamp(data, dataframe, &index, &sensorsdata, &sensortime, sensor);
				if (sensortime.irq_diff > 1000000)
					report_sensordata(data, sensor, &sensorsdata);
				else if ((sensor == PROXIMITY_SENSOR) || (sensor == PROXIMITY_RAW)
						|| (sensor == GESTURE_SENSOR) || (sensor == SIG_MOTION_SENSOR))
					report_sensordata(data, sensor, &sensorsdata);
				else
					ssp_errf("irq_diff is under 1msec (%d)", sensor);
				sensortime.batch_count--;
			} while ((sensortime.batch_count > 0) && (index < frame_len));

			if (sensortime.batch_count > 0)
				ssp_errf("batch count error (%d)", sensortime.batch_count);

			data->lastTimestamp[sensor] = data->timestamp;
			data->reportedData[sensor] = true;
#else
			data->get_sensor_data[sensor](dataframe, &index,
					&sensorsdata);
			get_timestamp(data, dataframe, &index, &sensorsdata);
			data->report_sensor_data[sensor](data, &sensorsdata);
#endif
			break;
		case MSG2AP_INST_DEBUG_DATA:
			sensor = print_mcu_debug(dataframe, &index, frame_len);
			if (sensor) {
				ssp_errf("Mcu debug dataframe err %d", sensor);
				return ERROR;
			}
			break;
		case MSG2AP_INST_LIBRARY_DATA:
			memcpy(&length, dataframe + index, 2);
			index += 2;
#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
			if (data->bLpModeEnabled == true)
				ssp_parse_motion(data, dataframe,
							index, index + length);
			else
				ssp_sensorhub_handle_data(data, dataframe,
					index, index + length);
#else
			ssp_sensorhub_handle_data(data, dataframe, index,
					index + length);
#endif
			index += length;
			break;
		case MSG2AP_INST_BIG_DATA:
			handle_big_data(data, dataframe, &index);
			break;
#ifdef CONFIG_SENSORS_SSP_META_DATA
		case MSG2AP_INST_META_DATA:
			sensorsdata.meta_data.what = dataframe[index++];
			sensorsdata.meta_data.sensor = dataframe[index++];
			report_meta_data(data, &sensorsdata);
			break;
#endif
		case MSG2AP_INST_TIME_SYNC:
			ssp_infof("time sync");
			data->bTimeSyncing = true;
			break;
		case MSG2AP_INST_RESET:
			ssp_infof("Reset MSG received from MCU");
			queue_refresh_task(data, 0);
			break;
#ifdef CONFIG_SENSORS_SSP_GYRO_SENSOR
		case MSG2AP_INST_GYRO_CAL:
			ssp_infof("Gyro caldata received from MCU");
			memcpy(caldata, dataframe + index, sizeof(caldata));
			wake_lock(&data->ssp_wake_lock);
			save_gyro_caldata(data, caldata);
			wake_unlock(&data->ssp_wake_lock);
			index += sizeof(caldata);
			break;
#endif
		case MSG2AP_INST_DUMP_DATA:
			debug_crash_dump(data, dataframe, frame_len);
			return SUCCESS;
			break;
		}
	}

	return SUCCESS;
}

#ifndef SSP_USE_IIO_BATCH
static void get_dummy_sensordata(char *pchRcvDataFrame, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	ssp_infof("not supported");
}

void report_dummy_data(struct ssp_data *data, struct sensor_value *value)
{
	ssp_infof("not supported");
}
#endif

void initialize_function_pointer(struct ssp_data *data)
{
#ifndef SSP_USE_IIO_BATCH
	int sensor_type;

	for (sensor_type = 0; sensor_type < SENSOR_MAX; sensor_type++) {
		data->get_sensor_data[sensor_type] = get_dummy_sensordata;
		data->report_sensor_data[sensor_type] = report_dummy_data;
	}

#ifdef CONFIG_SENSORS_SSP_ACCELEROMETER_SENSOR
	data->get_sensor_data[ACCELEROMETER_SENSOR] = get_3axis_sensordata;
	data->report_sensor_data[ACCELEROMETER_SENSOR] = report_acc_data;
#endif

#ifdef CONFIG_SENSORS_SSP_GYRO_SENSOR
	data->get_sensor_data[GYROSCOPE_SENSOR] = get_3axis_sensordata;
	data->get_sensor_data[GYRO_UNCALIB_SENSOR] = get_uncalib_sensordata;
	data->report_sensor_data[GYROSCOPE_SENSOR] = report_gyro_data;
	data->report_sensor_data[GYRO_UNCALIB_SENSOR] =
		report_uncalib_gyro_data;
#endif

#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
	data->get_sensor_data[GEOMAGNETIC_UNCALIB_SENSOR] =
		get_geomagnetic_uncaldata;
	data->get_sensor_data[GEOMAGNETIC_RAW] = get_geomagnetic_rawdata;
	data->get_sensor_data[GEOMAGNETIC_SENSOR] =
		get_geomagnetic_caldata;
	data->report_sensor_data[GEOMAGNETIC_UNCALIB_SENSOR] =
		report_mag_uncaldata;
	data->report_sensor_data[GEOMAGNETIC_RAW] = report_geomagnetic_raw_data;
	data->report_sensor_data[GEOMAGNETIC_SENSOR] =
		report_mag_data;
#endif

#ifdef CONFIG_SENSORS_SSP_LIGHT_SENSOR
	data->get_sensor_data[LIGHT_SENSOR] = get_light_sensordata;
	data->report_sensor_data[LIGHT_SENSOR] = report_light_data;
#endif

#ifdef CONFIG_SENSORS_SSP_PROXIMITY_SENSOR
	data->get_sensor_data[PROXIMITY_SENSOR] = get_proximity_sensordata;
	data->get_sensor_data[PROXIMITY_RAW] = get_proximity_rawdata;
	data->report_sensor_data[PROXIMITY_SENSOR] = report_prox_data;
	data->report_sensor_data[PROXIMITY_RAW] = report_prox_raw_data;
#endif

#ifdef CONFIG_SENSORS_SSP_PRESSURE_SENSOR
	data->get_sensor_data[PRESSURE_SENSOR] = get_pressure_sensordata;
	data->report_sensor_data[PRESSURE_SENSOR] = report_pressure_data;
#endif

#ifdef CONFIG_SENSORS_SSP_GESTURE_SENSOR
	data->get_sensor_data[GESTURE_SENSOR] = get_gesture_sensordata;
	data->report_sensor_data[GESTURE_SENSOR] = report_gesture_data;
#endif

#ifdef CONFIG_SENSORS_SSP_TEMP_HUMID_SENSOR
	data->get_sensor_data[TEMPERATURE_HUMIDITY_SENSOR] =
		get_temp_humidity_sensordata;
	data->report_sensor_data[TEMPERATURE_HUMIDITY_SENSOR] =
		report_temp_humidity_data;
#endif

#ifdef CONFIG_SENSORS_SSP_ROT_VECTOR_SENSOR
	data->get_sensor_data[ROTATION_VECTOR] = get_rot_sensordata;
	data->get_sensor_data[GAME_ROTATION_VECTOR] = get_rot_sensordata;
	data->report_sensor_data[ROTATION_VECTOR] = report_rot_data;
	data->report_sensor_data[GAME_ROTATION_VECTOR] = report_game_rot_data;
#endif

#ifdef CONFIG_SENSORS_SSP_HRM_SENSOR
	data->get_sensor_data[BIO_HRM_RAW] = get_hrm_raw_sensordata;
	data->get_sensor_data[BIO_HRM_RAW_FAC] =
		get_hrm_raw_fac_sensordata;
	data->get_sensor_data[BIO_HRM_LIB] = get_hrm_lib_sensordata;
	data->report_sensor_data[BIO_HRM_RAW] = report_hrm_raw_data;
	data->report_sensor_data[BIO_HRM_RAW_FAC] = report_hrm_raw_fac_data;
	data->report_sensor_data[BIO_HRM_LIB] = report_hrm_lib_data;
#endif

#ifdef CONFIG_SENSORS_SSP_UV_SENSOR
	data->get_sensor_data[UV_SENSOR] = get_uv_sensordata;
	data->report_sensor_data[UV_SENSOR] = report_uv_data;
#endif

#ifdef CONFIG_SENSORS_SSP_PIR_SENSOR
	data->get_sensor_data[PIR_SENSOR] = get_pir_sensordata;
	data->get_sensor_data[PIR_SENSOR_RAW] = get_pir_raw_sensordata;
		data->report_sensor_data[PIR_SENSOR] = report_pir_data;
	data->report_sensor_data[PIR_SENSOR_RAW] = report_pir_raw_data;
#endif
#endif
	data->ssp_big_task[BIG_TYPE_DUMP] = ssp_dump_task;
	data->ssp_big_task[BIG_TYPE_READ_LIB] = ssp_read_big_library_task;
}
