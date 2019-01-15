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
#include "ssp_input.h"

/*************************************************************************/
/* SSP Kernel -> HAL input evnet function                                */
/*************************************************************************/
#ifdef CONFIG_SENSORS_SSP_META_DATA
void report_meta_data(struct ssp_data *data, struct sensor_value *s)
{
	input_report_rel(data->meta_input_dev, REL_DIAL, s->meta_data.what);
	input_report_rel(data->meta_input_dev, REL_HWHEEL,
		s->meta_data.sensor + 1);
	input_sync(data->meta_input_dev);
}
#endif

#ifdef CONFIG_SENSORS_SSP_ACCELEROMETER_SENSOR
void report_acc_data(struct ssp_data *data, struct sensor_value *accdata)
{
	data->buf[ACCELEROMETER_SENSOR].x = accdata->x;
	data->buf[ACCELEROMETER_SENSOR].y = accdata->y;
	data->buf[ACCELEROMETER_SENSOR].z = accdata->z;

	input_report_rel(data->acc_input_dev, REL_X,
		data->buf[ACCELEROMETER_SENSOR].x);
	input_report_rel(data->acc_input_dev, REL_Y,
		data->buf[ACCELEROMETER_SENSOR].y);
	input_report_rel(data->acc_input_dev, REL_Z,
		data->buf[ACCELEROMETER_SENSOR].z);
	input_sync(data->acc_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("x=%d y=%d z=%d", accdata->x, accdata->y, accdata->z);
#endif
}
#endif

#ifdef CONFIG_SENSORS_SSP_GYRO_SENSOR
void report_gyro_data(struct ssp_data *data, struct sensor_value *gyrodata)
{
	long lTemp[3] = {0,};
	data->buf[GYROSCOPE_SENSOR].x = gyrodata->x;
	data->buf[GYROSCOPE_SENSOR].y = gyrodata->y;
	data->buf[GYROSCOPE_SENSOR].z = gyrodata->z;

	lTemp[0] = (long)data->buf[GYROSCOPE_SENSOR].x;
	lTemp[1] = (long)data->buf[GYROSCOPE_SENSOR].y;
	lTemp[2] = (long)data->buf[GYROSCOPE_SENSOR].z;

	input_report_rel(data->gyro_input_dev, REL_RX, lTemp[0]);
	input_report_rel(data->gyro_input_dev, REL_RY, lTemp[1]);
	input_report_rel(data->gyro_input_dev, REL_RZ, lTemp[2]);
	input_sync(data->gyro_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("x=%d y=%d z=%d", gyrodata->x, gyrodata->y, gyrodata->z);
#endif
}

void report_uncalib_gyro_data(struct ssp_data *data,
	struct sensor_value *gyrodata)
{
	data->buf[GYRO_UNCALIB_SENSOR].uncal_x = gyrodata->uncal_x;
	data->buf[GYRO_UNCALIB_SENSOR].uncal_y = gyrodata->uncal_y;
	data->buf[GYRO_UNCALIB_SENSOR].uncal_z = gyrodata->uncal_z;
	data->buf[GYRO_UNCALIB_SENSOR].offset_x = gyrodata->offset_x;
	data->buf[GYRO_UNCALIB_SENSOR].offset_y = gyrodata->offset_y;
	data->buf[GYRO_UNCALIB_SENSOR].offset_z = gyrodata->offset_z;

	input_report_rel(data->uncalib_gyro_input_dev, REL_RX,
		data->buf[GYRO_UNCALIB_SENSOR].uncal_x);
	input_report_rel(data->uncalib_gyro_input_dev, REL_RY,
		data->buf[GYRO_UNCALIB_SENSOR].uncal_y);
	input_report_rel(data->uncalib_gyro_input_dev, REL_RZ,
		data->buf[GYRO_UNCALIB_SENSOR].uncal_z);
	input_report_rel(data->uncalib_gyro_input_dev, REL_HWHEEL,
		data->buf[GYRO_UNCALIB_SENSOR].offset_x);
	input_report_rel(data->uncalib_gyro_input_dev, REL_DIAL,
		data->buf[GYRO_UNCALIB_SENSOR].offset_y);
	input_report_rel(data->uncalib_gyro_input_dev, REL_WHEEL,
		data->buf[GYRO_UNCALIB_SENSOR].offset_z);
	input_sync(data->uncalib_gyro_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("x=%d y=%d z=%d, offset:x=%d y=%d z=%d",
		gyrodata->uncal_x, gyrodata->uncal_y, gyrodata->uncal_z,
		gyrodata->offset_x, gyrodata->offset_y, gyrodata->offset_z);
#endif
}
#endif

#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
void report_geomagnetic_raw_data(struct ssp_data *data,
	struct sensor_value *magrawdata)
{
	data->buf[GEOMAGNETIC_RAW].x = magrawdata->x;
	data->buf[GEOMAGNETIC_RAW].y = magrawdata->y;
	data->buf[GEOMAGNETIC_RAW].z = magrawdata->z;
#ifdef SSP_DEBUG_LOG
	ssp_infof("x=%d y=%d z=%d",
		magrawdata->x, magrawdata->y, magrawdata->z);
#endif
}

void report_mag_data(struct ssp_data *data, struct sensor_value *magdata)
{
	data->buf[GEOMAGNETIC_SENSOR].cal_x = magdata->cal_x;
	data->buf[GEOMAGNETIC_SENSOR].cal_y = magdata->cal_y;
	data->buf[GEOMAGNETIC_SENSOR].cal_z = magdata->cal_z;
	data->buf[GEOMAGNETIC_SENSOR].accuracy = magdata->accuracy;

	input_report_rel(data->mag_input_dev, REL_RX,
		data->buf[GEOMAGNETIC_SENSOR].cal_x);
	input_report_rel(data->mag_input_dev, REL_RY,
		data->buf[GEOMAGNETIC_SENSOR].cal_y);
	input_report_rel(data->mag_input_dev, REL_RZ,
		data->buf[GEOMAGNETIC_SENSOR].cal_z);
	input_report_rel(data->mag_input_dev, REL_HWHEEL,
		data->buf[GEOMAGNETIC_SENSOR].accuracy + 1);
	input_sync(data->mag_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("x=%d y=%d z=%d, accuray=%d",
		magdata->cal_x, magdata->cal_y, magdata->cal_z,
		magdata->accuracy);
#endif
}

void report_mag_uncaldata(struct ssp_data *data, struct sensor_value *magdata)
{
	data->buf[GEOMAGNETIC_UNCALIB_SENSOR].uncal_x = magdata->uncal_x;
	data->buf[GEOMAGNETIC_UNCALIB_SENSOR].uncal_y = magdata->uncal_y;
	data->buf[GEOMAGNETIC_UNCALIB_SENSOR].uncal_z = magdata->uncal_z;
	data->buf[GEOMAGNETIC_UNCALIB_SENSOR].offset_x= magdata->offset_x;
	data->buf[GEOMAGNETIC_UNCALIB_SENSOR].offset_y= magdata->offset_y;
	data->buf[GEOMAGNETIC_UNCALIB_SENSOR].offset_z= magdata->offset_z;

	input_report_rel(data->uncal_mag_input_dev, REL_RX,
		data->buf[GEOMAGNETIC_UNCALIB_SENSOR].uncal_x);
	input_report_rel(data->uncal_mag_input_dev, REL_RY,
		data->buf[GEOMAGNETIC_UNCALIB_SENSOR].uncal_y);
	input_report_rel(data->uncal_mag_input_dev, REL_RZ,
		data->buf[GEOMAGNETIC_UNCALIB_SENSOR].uncal_z);
	input_report_rel(data->uncal_mag_input_dev, REL_HWHEEL,
		data->buf[GEOMAGNETIC_UNCALIB_SENSOR].offset_x);
	input_report_rel(data->uncal_mag_input_dev, REL_DIAL,
		data->buf[GEOMAGNETIC_UNCALIB_SENSOR].offset_y);
	input_report_rel(data->uncal_mag_input_dev, REL_WHEEL,
		data->buf[GEOMAGNETIC_UNCALIB_SENSOR].offset_z);
	input_sync(data->uncal_mag_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("x=%d y=%d z=%d, offset:x=%d y=%d z=%d",
		magdata->uncal_x, magdata->uncal_y, magdata->uncal_z,
		magdata->offset_x, magdata->offset_y, magdata->offset_z);
#endif
}
#endif

#ifdef CONFIG_SENSORS_SSP_PRESSURE_SENSOR
void report_pressure_data(struct ssp_data *data, struct sensor_value *predata)
{
	int temp[3] = {0, };
	data->buf[PRESSURE_SENSOR].pressure =
		predata->pressure - data->iPressureCal;
	data->buf[PRESSURE_SENSOR].temperature = predata->temperature;

	temp[0] = data->buf[PRESSURE_SENSOR].pressure;
	temp[1] = data->buf[PRESSURE_SENSOR].temperature;
	temp[2] = data->sealevelpressure;

	/* pressure */
	input_report_rel(data->pressure_input_dev, REL_HWHEEL, temp[0]);
	/* sealevel */
	input_report_rel(data->pressure_input_dev, REL_DIAL, temp[2]);
	/* temperature */
	input_report_rel(data->pressure_input_dev, REL_WHEEL, temp[1]);
	input_sync(data->pressure_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("pressure=%d, temp=%d, sealevel=%d\n", __func__,
		temp[0], temp[1], temp[2]);
#endif
}
#endif

#ifdef CONFIG_SENSORS_SSP_GESTURE_SENSOR
void report_gesture_data(struct ssp_data *data, struct sensor_value *gesdata)
{
	int i = 0;
	for (i = 0; i < 20; i++) {
		data->buf[GESTURE_SENSOR].data[i] = gesdata->data[i];
	}

	input_report_abs(data->gesture_input_dev,
		ABS_X, data->buf[GESTURE_SENSOR].data[0]);
	input_report_abs(data->gesture_input_dev,
		ABS_Y, data->buf[GESTURE_SENSOR].data[1]);
	input_report_abs(data->gesture_input_dev,
		ABS_Z, data->buf[GESTURE_SENSOR].data[2]);
	input_report_abs(data->gesture_input_dev,
		ABS_RX, data->buf[GESTURE_SENSOR].data[3]);
	input_report_abs(data->gesture_input_dev,
		ABS_RY, data->buf[GESTURE_SENSOR].data[4]);
	input_report_abs(data->gesture_input_dev,
		ABS_RZ, data->buf[GESTURE_SENSOR].data[5]);
	input_report_abs(data->gesture_input_dev,
		ABS_THROTTLE, data->buf[GESTURE_SENSOR].data[6]);
	input_report_abs(data->gesture_input_dev,
		ABS_RUDDER, data->buf[GESTURE_SENSOR].data[7]);
	input_report_abs(data->gesture_input_dev,
		ABS_WHEEL, data->buf[GESTURE_SENSOR].data[8]);
	input_report_abs(data->gesture_input_dev,
		ABS_GAS, data->buf[GESTURE_SENSOR].data[9]);
	input_report_abs(data->gesture_input_dev,
		ABS_BRAKE, data->buf[GESTURE_SENSOR].data[10]);
	input_report_abs(data->gesture_input_dev,
		ABS_HAT0X, data->buf[GESTURE_SENSOR].data[11]);
	input_report_abs(data->gesture_input_dev,
		ABS_HAT0Y, data->buf[GESTURE_SENSOR].data[12]);
	input_report_abs(data->gesture_input_dev,
		ABS_HAT1X, data->buf[GESTURE_SENSOR].data[13]);
	input_report_abs(data->gesture_input_dev,
		ABS_HAT1Y, data->buf[GESTURE_SENSOR].data[14]);
	input_report_abs(data->gesture_input_dev,
		ABS_HAT2X, data->buf[GESTURE_SENSOR].data[15]);
	input_report_abs(data->gesture_input_dev,
		ABS_HAT2Y, data->buf[GESTURE_SENSOR].data[16]);
	input_report_abs(data->gesture_input_dev,
		ABS_HAT3X, data->buf[GESTURE_SENSOR].data[17]);
	input_report_abs(data->gesture_input_dev,
		ABS_HAT3Y, data->buf[GESTURE_SENSOR].data[18]);
	input_report_abs(data->gesture_input_dev,
		ABS_PRESSURE, data->buf[GESTURE_SENSOR].data[19]);

	input_sync(data->gesture_input_dev);
}
#endif

#ifdef CONFIG_SENSORS_SSP_LIGHT_SENSOR
void report_light_data(struct ssp_data *data, struct sensor_value *lightdata)
{
#if defined(CONFIG_SENSORS_SSP_AL3320)
	u16 raw_lux = ((lightdata->raw_high << 8) | lightdata->raw_low);

	data->buf[LIGHT_SENSOR].raw_high = lightdata->raw_high;
	data->buf[LIGHT_SENSOR].raw_low = lightdata->raw_low;

	data->lux = light_get_lux(raw_lux);
	input_report_rel(data->light_input_dev, REL_RX, data->lux + 1);
	input_sync(data->light_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("lux=%d", data->lux);
#endif
#elif defined(CONFIG_SENSORS_SSP_CM36686)
	data->lux = light_get_lux(lightdata->als);

	data->buf[LIGHT_SENSOR].als = lightdata->als;
	data->buf[LIGHT_SENSOR].white = lightdata->white;

	input_report_rel(data->light_input_dev, REL_RX,
		data->lux + 1);
	input_report_rel(data->light_input_dev, REL_RY,
		data->buf[LIGHT_SENSOR].white + 1);
	input_sync(data->light_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("als=%d, white=%d, lux=%d",
		lightdata->als, lightdata->white, data->lux);
#endif
#else
	data->buf[LIGHT_SENSOR].r = lightdata->r;
	data->buf[LIGHT_SENSOR].g = lightdata->g;
	data->buf[LIGHT_SENSOR].b = lightdata->b;
	data->buf[LIGHT_SENSOR].w = lightdata->w;

	input_report_rel(data->light_input_dev, REL_HWHEEL,
		data->buf[LIGHT_SENSOR].r + 1);
	input_report_rel(data->light_input_dev, REL_DIAL,
		data->buf[LIGHT_SENSOR].g + 1);
	input_report_rel(data->light_input_dev, REL_WHEEL,
		data->buf[LIGHT_SENSOR].b + 1);
	input_report_rel(data->light_input_dev, REL_MISC,
		data->buf[LIGHT_SENSOR].w + 1);
	input_sync(data->light_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("r=%d, g=%d, b=%d, w=%d",
		lightdata->r, lightdata->g, lightdata->b, lightdata->w);
#endif
#endif
}
#endif

#ifdef CONFIG_SENSORS_SSP_PROXIMITY_SENSOR
void report_prox_data(struct ssp_data *data, struct sensor_value *proxdata)
{
	ssp_infof("Proximity Sensor Detect : %u, raw : %u",
		proxdata->prox, proxdata->prox_ex);

	data->buf[PROXIMITY_SENSOR].prox = proxdata->prox;
	data->buf[PROXIMITY_SENSOR].prox_ex = proxdata->prox_ex;

	input_report_abs(data->prox_input_dev, ABS_DISTANCE,
		(!proxdata->prox));
	input_sync(data->prox_input_dev);

	wake_lock_timeout(&data->ssp_wake_lock, 3 * HZ);
}

void report_prox_raw_data(struct ssp_data *data,
	struct sensor_value *proxrawdata)
{
	if (data->uFactoryProxAvg[0]++ >= PROX_AVG_READ_NUM) {
		data->uFactoryProxAvg[2] /= PROX_AVG_READ_NUM;
		data->buf[PROXIMITY_RAW].prox_raw[1] =
			(u16)data->uFactoryProxAvg[1];
		data->buf[PROXIMITY_RAW].prox_raw[2] =
			(u16)data->uFactoryProxAvg[2];
		data->buf[PROXIMITY_RAW].prox_raw[3] =
			(u16)data->uFactoryProxAvg[3];

		data->uFactoryProxAvg[0] = 0;
		data->uFactoryProxAvg[1] = 0;
		data->uFactoryProxAvg[2] = 0;
		data->uFactoryProxAvg[3] = 0;
	} else {
		data->uFactoryProxAvg[2] += proxrawdata->prox_raw[0];

		if (data->uFactoryProxAvg[0] == 1)
			data->uFactoryProxAvg[1] = proxrawdata->prox_raw[0];
		else if (proxrawdata->prox_raw[0] < data->uFactoryProxAvg[1])
			data->uFactoryProxAvg[1] = proxrawdata->prox_raw[0];

		if (proxrawdata->prox_raw[0] > data->uFactoryProxAvg[3])
			data->uFactoryProxAvg[3] = proxrawdata->prox_raw[0];
	}

	data->buf[PROXIMITY_RAW].prox_raw[0] = proxrawdata->prox_raw[0];

#ifdef SSP_DEBUG_LOG
	ssp_infof("raw=%u", proxrawdata->prox_raw[0]);
#endif
}
#endif

#ifdef CONFIG_SENSORS_SSP_ROT_VECTOR_SENSOR
void report_rot_data(struct ssp_data *data, struct sensor_value *rotdata)
{
	int rot_buf[5];
	data->buf[ROTATION_VECTOR].quat_a = rotdata->quat_a;
	data->buf[ROTATION_VECTOR].quat_b = rotdata->quat_b;
	data->buf[ROTATION_VECTOR].quat_c = rotdata->quat_c;
	data->buf[ROTATION_VECTOR].quat_d = rotdata->quat_d;
	data->buf[ROTATION_VECTOR].acc_rot = rotdata->acc_rot;

	rot_buf[0] = rotdata->quat_a;
	rot_buf[1] = rotdata->quat_b;
	rot_buf[2] = rotdata->quat_c;
	rot_buf[3] = rotdata->quat_d;
	rot_buf[4] = rotdata->acc_rot;

	input_report_rel(data->rot_input_dev, REL_X, rot_buf[0]);
	input_report_rel(data->rot_input_dev, REL_Y, rot_buf[1]);
	input_report_rel(data->rot_input_dev, REL_Z, rot_buf[2]);
	input_report_rel(data->rot_input_dev, REL_RX, rot_buf[3]);
	input_report_rel(data->rot_input_dev, REL_RY, rot_buf[4] + 1);
	input_sync(data->rot_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("quat=[%d %d %d %d], acc_rot=%d",
		rotdata->quat_a, rotdata->quat_b, rotdata->quat_c,
		rotdata->quat_d, rotdata->acc_rot);
#endif
}

void report_game_rot_data(struct ssp_data *data, struct sensor_value *grotdata)
{
	int rot_buf[5];
	data->buf[GAME_ROTATION_VECTOR].quat_a = grotdata->quat_a;
	data->buf[GAME_ROTATION_VECTOR].quat_b = grotdata->quat_b;
	data->buf[GAME_ROTATION_VECTOR].quat_c = grotdata->quat_c;
	data->buf[GAME_ROTATION_VECTOR].quat_d = grotdata->quat_d;
	data->buf[GAME_ROTATION_VECTOR].acc_rot = grotdata->acc_rot;

	rot_buf[0] = grotdata->quat_a;
	rot_buf[1] = grotdata->quat_b;
	rot_buf[2] = grotdata->quat_c;
	rot_buf[3] = grotdata->quat_d;
	rot_buf[4] = grotdata->acc_rot;

	input_report_rel(data->game_rot_input_dev, REL_X, rot_buf[0]);
	input_report_rel(data->game_rot_input_dev, REL_Y, rot_buf[1]);
	input_report_rel(data->game_rot_input_dev, REL_Z, rot_buf[2]);
	input_report_rel(data->game_rot_input_dev, REL_RX, rot_buf[3]);
	input_report_rel(data->game_rot_input_dev, REL_RY, rot_buf[4] + 1);
	input_sync(data->game_rot_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("quat=[%d %d %d %d], acc_rot=%d",
		grotdata->quat_a, grotdata->quat_b, grotdata->quat_c,
		grotdata->quat_d, grotdata->acc_rot);
#endif
}
#endif

#ifdef CONFIG_SENSORS_SSP_HRM_SENSOR
void report_hrm_raw_data(struct ssp_data *data, struct sensor_value *hrmdata)
{
	data->buf[BIO_HRM_RAW].ch_a_sum = hrmdata->ch_a_sum;
	data->buf[BIO_HRM_RAW].ch_a_x1 = hrmdata->ch_a_x1;
	data->buf[BIO_HRM_RAW].ch_a_x2 = hrmdata->ch_a_x2;
	data->buf[BIO_HRM_RAW].ch_a_y1 = hrmdata->ch_a_y1;
	data->buf[BIO_HRM_RAW].ch_a_y2 = hrmdata->ch_a_y2;
	data->buf[BIO_HRM_RAW].ch_b_sum = hrmdata->ch_b_sum;
	data->buf[BIO_HRM_RAW].ch_b_x1 = hrmdata->ch_b_x1;
	data->buf[BIO_HRM_RAW].ch_b_x2 = hrmdata->ch_b_x2;
	data->buf[BIO_HRM_RAW].ch_b_y1 = hrmdata->ch_b_y1;
	data->buf[BIO_HRM_RAW].ch_b_y2 = hrmdata->ch_b_y2;

	input_report_rel(data->hrm_raw_input_dev, REL_X,
		data->buf[BIO_HRM_RAW].ch_a_sum + 1);
	input_report_rel(data->hrm_raw_input_dev, REL_Y,
		data->buf[BIO_HRM_RAW].ch_a_x1 + 1);
	input_report_rel(data->hrm_raw_input_dev, REL_Z,
		data->buf[BIO_HRM_RAW].ch_a_x2 + 1);
	input_report_rel(data->hrm_raw_input_dev, REL_RX,
		data->buf[BIO_HRM_RAW].ch_a_y1 + 1);
	input_report_rel(data->hrm_raw_input_dev, REL_RY,
		data->buf[BIO_HRM_RAW].ch_a_y2 + 1);
	input_report_rel(data->hrm_raw_input_dev, REL_RZ,
		data->buf[BIO_HRM_RAW].ch_b_sum + 1);
	input_report_rel(data->hrm_raw_input_dev, REL_HWHEEL,
		data->buf[BIO_HRM_RAW].ch_b_x1 + 1);
	input_report_rel(data->hrm_raw_input_dev, REL_DIAL,
		data->buf[BIO_HRM_RAW].ch_b_x2 + 1);
	input_report_rel(data->hrm_raw_input_dev, REL_WHEEL,
		data->buf[BIO_HRM_RAW].ch_b_y1 + 1);
	input_report_rel(data->hrm_raw_input_dev, REL_MISC,
		data->buf[BIO_HRM_RAW].ch_b_y2 + 1);
	input_sync(data->hrm_raw_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("ch_a=[%d %d %d %d %d], ch_b=[%d %d %d %d %d]",
		hrmdata->ch_a_sum, hrmdata->ch_a_x1,
		hrmdata->ch_a_x2, hrmdata->ch_a_y1,
		hrmdata->ch_a_y2, hrmdata->ch_b_sum,
		hrmdata->ch_b_x1, hrmdata->ch_b_x2,
		hrmdata->ch_b_y1, hrmdata->ch_b_y2);
#endif
}

void report_hrm_raw_fac_data(struct ssp_data *data, struct sensor_value *hrmdata)
{
	data->buf[BIO_HRM_RAW_FAC].frequency = hrmdata->frequency;
	data->buf[BIO_HRM_RAW_FAC].green_dc_level =
		hrmdata->green_dc_level;
	data->buf[BIO_HRM_RAW_FAC].green_high_dc_level =
		hrmdata->green_high_dc_level;
	data->buf[BIO_HRM_RAW_FAC].green_mid_dc_level =
		hrmdata->green_mid_dc_level;
	data->buf[BIO_HRM_RAW_FAC].red_dc_levet = hrmdata->red_dc_levet;
	data->buf[BIO_HRM_RAW_FAC].ir_dc_level = hrmdata->ir_dc_level;
	data->buf[BIO_HRM_RAW_FAC].noise_level = hrmdata->noise_level;
	data->buf[BIO_HRM_RAW_FAC].oscRegValue = hrmdata->oscRegValue;
	memcpy(data->buf[BIO_HRM_RAW_FAC].adc_offset,
		hrmdata->adc_offset, sizeof(hrmdata->adc_offset));

#ifdef SSP_DEBUG_LOG
	ssp_infof("[%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
		hrmdata->frequency, hrmdata->green_dc_level,
		hrmdata->green_high_dc_level, hrmdata->green_mid_dc_level,
		hrmdata->red_dc_levet, hrmdata->ir_dc_level,
		hrmdata->noise_level,
		hrmdata->adc_offset[0], hrmdata->adc_offset[1],
		hrmdata->adc_offset[2], hrmdata->adc_offset[3],
		hrmdata->adc_offset[4], hrmdata->adc_offset[5],
		hrmdata->adc_offset[6], hrmdata->adc_offset[7]);
#endif
}

void report_hrm_lib_data(struct ssp_data *data, struct sensor_value *hrmdata)
{
	data->buf[BIO_HRM_LIB].hr = hrmdata->hr;
	data->buf[BIO_HRM_LIB].rri = hrmdata->rri;
	data->buf[BIO_HRM_LIB].snr = hrmdata->snr;

	input_report_rel(data->hrm_lib_input_dev, REL_X, data->buf[BIO_HRM_LIB].hr + 1);
	input_report_rel(data->hrm_lib_input_dev, REL_Y, data->buf[BIO_HRM_LIB].rri + 1);
	input_report_rel(data->hrm_lib_input_dev, REL_Z, data->buf[BIO_HRM_LIB].snr + 1);
	input_sync(data->hrm_lib_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("hr=%d rri=%d snr=%d",
		hrmdata->hr, hrmdata->rri, hrmdata->snr);
#endif
}
#endif

#ifdef CONFIG_SENSORS_SSP_UV_SENSOR
void report_uv_data(struct ssp_data *data, struct sensor_value *uvdata)
{
	data->buf[UV_SENSOR].uv_raw = uvdata->uv_raw;

	input_report_rel(data->uv_input_dev, REL_MISC,
		data->buf[UV_SENSOR].uv_raw + 1);
	input_sync(data->uv_input_dev);

#ifdef SSP_DEBUG_LOG
	ssp_infof("uv raw=%d", uvdata->uv_raw);
#endif
}
#endif

#ifdef CONFIG_SENSORS_SSP_TEMP_HUMID_SENSOR
void report_temp_humidity_data(struct ssp_data *data,
	struct sensor_value *temp_humi_data)
{
	data->buf[TEMPERATURE_HUMIDITY_SENSOR].x = temp_humi_data->x;
	data->buf[TEMPERATURE_HUMIDITY_SENSOR].y = temp_humi_data->y;
	data->buf[TEMPERATURE_HUMIDITY_SENSOR].z = temp_humi_data->z;

	/* Temperature */
	input_report_rel(data->temp_humi_input_dev, REL_HWHEEL,
		data->buf[TEMPERATURE_HUMIDITY_SENSOR].x);
	/* Humidity */
	input_report_rel(data->temp_humi_input_dev, REL_DIAL,
		data->buf[TEMPERATURE_HUMIDITY_SENSOR].y);
	input_sync(data->temp_humi_input_dev);
	if (data->buf[TEMPERATURE_HUMIDITY_SENSOR].z)
		wake_lock_timeout(&data->ssp_wake_lock, 2 * HZ);

#ifdef SSP_DEBUG_LOG
	ssp_infof("temp=%d, humidity=%d",
		temp_humi_data->x, temp_humi_data->y);
#endif

}

void report_bulk_comp_data(struct ssp_data *data)
{
	input_report_rel(data->temp_humi_input_dev, REL_WHEEL,
		data->bulk_buffer->len);
	input_sync(data->temp_humi_input_dev);
}
#endif

#ifdef CONFIG_SENSORS_SSP_PIR_SENSOR
void report_pir_data(struct ssp_data *data, struct sensor_value *pir)
{
	data->buf[PIR_SENSOR].event = pir->event;

	switch (data->buf[PIR_SENSOR].event) {
	case 0x01:
		input_report_rel(data->pir_standard_input_dev, REL_MISC, 1);
		input_sync(data->pir_standard_input_dev);
		break;
	case 0x02:
		input_report_rel(data->pir_long_input_dev, REL_MISC, 1);
		input_sync(data->pir_long_input_dev);
		break;
	case 0x03:
		input_report_rel(data->pir_standard_input_dev, REL_MISC, 1);
		input_sync(data->pir_standard_input_dev);
		input_report_rel(data->pir_long_input_dev, REL_MISC, 1);
		input_sync(data->pir_long_input_dev);
		break;
	}
#ifdef SSP_DEBUG_LOG
	ssp_infof("pir event=%d", pir->event);
#endif
}

void report_pir_raw_data(struct ssp_data *data, struct sensor_value *pir)
{
	data->buf[PIR_SENSOR_RAW].near_temp = pir->near_temp;
	data->buf[PIR_SENSOR_RAW].near_adc = pir->near_adc;
	data->buf[PIR_SENSOR_RAW].far_temp = pir->far_temp;
	data->buf[PIR_SENSOR_RAW].far_adc = pir->far_adc;

#ifdef SSP_DEBUG_LOG
	ssp_infof("near: %d, %d / far: %d, %d]", pir->near_temp, pir->near_adc,
		pir->far_temp, pir->far_adc);
#endif
}
#endif

void remove_event_symlink(struct ssp_data *data)
{
	if (data->meta_input_dev)
		sensors_remove_symlink(data->meta_input_dev);
	if (data->pir_long_input_dev)
		sensors_remove_symlink(data->pir_long_input_dev);
	if (data->pir_standard_input_dev)
		sensors_remove_symlink(data->pir_standard_input_dev);
	if (data->uv_input_dev)
		sensors_remove_symlink(data->uv_input_dev);
	if (data->hrm_lib_input_dev)
		sensors_remove_symlink(data->hrm_lib_input_dev);
	if (data->hrm_raw_input_dev)
		sensors_remove_symlink(data->hrm_raw_input_dev);
	if (data->game_rot_input_dev)
		sensors_remove_symlink(data->game_rot_input_dev);
	if (data->rot_input_dev)
		sensors_remove_symlink(data->rot_input_dev);
	if (data->temp_humi_input_dev)
		sensors_remove_symlink(data->temp_humi_input_dev);
	if (data->prox_input_dev)
		sensors_remove_symlink(data->prox_input_dev);
	if (data->light_input_dev)
		sensors_remove_symlink(data->light_input_dev);
	if (data->gesture_input_dev)
		sensors_remove_symlink(data->gesture_input_dev);
	if (data->pressure_input_dev)
		sensors_remove_symlink(data->pressure_input_dev);
	if (data->uncalib_gyro_input_dev)
		sensors_remove_symlink(data->uncalib_gyro_input_dev);
	if (data->uncal_mag_input_dev)
		sensors_remove_symlink(data->uncal_mag_input_dev);
	if (data->mag_input_dev)
		sensors_remove_symlink(data->mag_input_dev);
	if (data->gyro_input_dev)
		sensors_remove_symlink(data->gyro_input_dev);
	if (data->acc_input_dev)
		sensors_remove_symlink(data->acc_input_dev);
}

int initialize_event_symlink(struct ssp_data *data)
{
	int iRet = 0;

	if (data->acc_input_dev) {
		iRet = sensors_create_symlink(data->acc_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->gyro_input_dev) {
		iRet = sensors_create_symlink(data->gyro_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->mag_input_dev) {
		iRet = sensors_create_symlink(data->mag_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->uncal_mag_input_dev) {
		iRet = sensors_create_symlink(data->uncal_mag_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->uncalib_gyro_input_dev) {
		iRet = sensors_create_symlink(data->uncalib_gyro_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->pressure_input_dev) {
		iRet = sensors_create_symlink(data->pressure_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->gesture_input_dev) {
		iRet = sensors_create_symlink(data->gesture_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->light_input_dev) {
		iRet = sensors_create_symlink(data->light_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->prox_input_dev) {
		iRet = sensors_create_symlink(data->prox_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->temp_humi_input_dev) {
		iRet = sensors_create_symlink(data->temp_humi_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->rot_input_dev) {
		iRet = sensors_create_symlink(data->rot_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->game_rot_input_dev) {
		iRet = sensors_create_symlink(data->game_rot_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->hrm_raw_input_dev) {
		iRet = sensors_create_symlink(data->hrm_raw_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->hrm_lib_input_dev) {
		iRet = sensors_create_symlink(data->hrm_lib_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->uv_input_dev) {
		iRet = sensors_create_symlink(data->uv_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->pir_standard_input_dev) {
		iRet = sensors_create_symlink(data->pir_standard_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}
	if (data->pir_long_input_dev) {
		iRet = sensors_create_symlink(data->pir_long_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}

	if (data->meta_input_dev) {
		iRet = sensors_create_symlink(data->meta_input_dev);
		if (iRet < 0)
			goto err_sysfs_create_link;
	}

	return SUCCESS;

err_sysfs_create_link:
	remove_event_symlink(data);
	ssp_errf("could not create event symlink");

	return FAIL;
}

void remove_input_dev(struct ssp_data *data)
{
	if (data->meta_input_dev) {
		input_unregister_device(data->meta_input_dev);
		ssp_infof("unregister meta input device");
	}
	if (data->pir_long_input_dev) {
		input_unregister_device(data->pir_long_input_dev);
		ssp_infof("unregister pir long input device");
	}
	if (data->pir_standard_input_dev) {
		input_unregister_device(data->pir_standard_input_dev);
		ssp_infof("unregister pir standard input device");
	}
	if (data->uv_input_dev) {
		input_unregister_device(data->uv_input_dev);
		ssp_infof("unregister uv input device");
	}
	if (data->hrm_lib_input_dev) {
		input_unregister_device(data->hrm_lib_input_dev);
		ssp_infof("unregister hrm lib input device");
	}
	if (data->hrm_raw_input_dev) {
		input_unregister_device(data->hrm_raw_input_dev);
		ssp_infof("unregister hrm raw input device");
	}
	if (data->motion_input_dev) {
		input_unregister_device(data->motion_input_dev);
		ssp_infof("unregister motion input device");
	}
	if (data->game_rot_input_dev) {
		input_unregister_device(data->game_rot_input_dev);
		ssp_infof("unregister game_rot input device");
	}
	if (data->rot_input_dev) {
		input_unregister_device(data->rot_input_dev);
		ssp_infof("unregister rot input device");
	}
	if (data->temp_humi_input_dev) {
		input_unregister_device(data->temp_humi_input_dev);
		ssp_infof("unregister temp_humid input device");
	}
	if (data->prox_input_dev) {
		input_unregister_device(data->prox_input_dev);
		ssp_infof("unregister prox input device");
	}
	if (data->light_input_dev) {
		input_unregister_device(data->light_input_dev);
		ssp_infof("unregister light input device");
	}
	if (data->gesture_input_dev) {
		input_unregister_device(data->gesture_input_dev);
		ssp_infof("unregister gesture input device");
	}
	if (data->pressure_input_dev) {
		input_unregister_device(data->pressure_input_dev);
		ssp_infof("unregister pressure input device");
	}
	if (data->uncalib_gyro_input_dev) {
		input_unregister_device(data->uncalib_gyro_input_dev);
		ssp_infof("unregister uncal gyro input device");
	}
	if (data->uncal_mag_input_dev) {
		input_unregister_device(data->uncal_mag_input_dev);
		ssp_infof("unregister uncal mag input device");
	}
	if (data->mag_input_dev) {
		input_unregister_device(data->mag_input_dev);
		ssp_infof("unregister mag input device");
	}
	if (data->gyro_input_dev) {
		input_unregister_device(data->gyro_input_dev);
		ssp_infof("unregister gyro input device");
	}
	if (data->acc_input_dev) {
		input_unregister_device(data->acc_input_dev);
		ssp_infof("unregister acc input device");
	}
}

int initialize_input_dev(struct ssp_data *data)
{
	int iRet = 0;

#ifdef CONFIG_SENSORS_SSP_ACCELEROMETER_SENSOR
	data->acc_input_dev = input_allocate_device();
	if (data->acc_input_dev == NULL)
		goto err_initialize_input_dev;

	data->acc_input_dev->name = "accelerometer_sensor";
	input_set_capability(data->acc_input_dev, EV_REL, REL_X);
	input_set_capability(data->acc_input_dev, EV_REL, REL_Y);
	input_set_capability(data->acc_input_dev, EV_REL, REL_Z);

	iRet = input_register_device(data->acc_input_dev);
	if (iRet < 0) {
		input_free_device(data->acc_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->acc_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_GYRO_SENSOR
	data->gyro_input_dev = input_allocate_device();
	if (data->gyro_input_dev == NULL)
		goto err_initialize_input_dev;

	data->gyro_input_dev->name = "gyro_sensor";
	input_set_capability(data->gyro_input_dev, EV_REL, REL_RX);
	input_set_capability(data->gyro_input_dev, EV_REL, REL_RY);
	input_set_capability(data->gyro_input_dev, EV_REL, REL_RZ);

	iRet = input_register_device(data->gyro_input_dev);
	if (iRet < 0) {
		input_free_device(data->gyro_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->gyro_input_dev, data);

	data->uncalib_gyro_input_dev = input_allocate_device();
	if (data->uncalib_gyro_input_dev == NULL)
		goto err_initialize_input_dev;

	data->uncalib_gyro_input_dev->name = "uncal_gyro_sensor";
	input_set_capability(data->uncalib_gyro_input_dev, EV_REL, REL_RX);
	input_set_capability(data->uncalib_gyro_input_dev, EV_REL, REL_RY);
	input_set_capability(data->uncalib_gyro_input_dev, EV_REL, REL_RZ);
	input_set_capability(data->uncalib_gyro_input_dev, EV_REL, REL_HWHEEL);
	input_set_capability(data->uncalib_gyro_input_dev, EV_REL, REL_DIAL);
	input_set_capability(data->uncalib_gyro_input_dev, EV_REL, REL_WHEEL);
	iRet = input_register_device(data->uncalib_gyro_input_dev);
	if (iRet < 0) {
		input_free_device(data->uncalib_gyro_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->uncalib_gyro_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
	data->mag_input_dev = input_allocate_device();
	if (data->mag_input_dev == NULL)
		goto err_initialize_input_dev;

	data->mag_input_dev->name = "geomagnetic_sensor";
	input_set_capability(data->mag_input_dev, EV_REL, REL_RX);
	input_set_capability(data->mag_input_dev, EV_REL, REL_RY);
	input_set_capability(data->mag_input_dev, EV_REL, REL_RZ);
	input_set_capability(data->mag_input_dev, EV_REL, REL_HWHEEL);

	iRet = input_register_device(data->mag_input_dev);
	if (iRet < 0) {
		input_free_device(data->mag_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->mag_input_dev, data);


	data->uncal_mag_input_dev = input_allocate_device();
	if (data->uncal_mag_input_dev == NULL)
		goto err_initialize_input_dev;

	data->uncal_mag_input_dev->name = "uncal_geomagnetic_sensor";
	input_set_capability(data->uncal_mag_input_dev, EV_REL, REL_RX);
	input_set_capability(data->uncal_mag_input_dev, EV_REL, REL_RY);
	input_set_capability(data->uncal_mag_input_dev, EV_REL, REL_RZ);
	input_set_capability(data->uncal_mag_input_dev, EV_REL, REL_HWHEEL);
	input_set_capability(data->uncal_mag_input_dev, EV_REL, REL_DIAL);
	input_set_capability(data->uncal_mag_input_dev, EV_REL, REL_WHEEL);

	iRet = input_register_device(data->uncal_mag_input_dev);
	if (iRet < 0) {
		input_free_device(data->uncal_mag_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->uncal_mag_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_PRESSURE_SENSOR
	data->pressure_input_dev = input_allocate_device();
	if (data->pressure_input_dev == NULL)
		goto err_initialize_input_dev;

	data->pressure_input_dev->name = "pressure_sensor";
	input_set_capability(data->pressure_input_dev, EV_REL, REL_HWHEEL);
	input_set_capability(data->pressure_input_dev, EV_REL, REL_DIAL);
	input_set_capability(data->pressure_input_dev, EV_REL, REL_WHEEL);

	iRet = input_register_device(data->pressure_input_dev);
	if (iRet < 0) {
		input_free_device(data->pressure_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->pressure_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_GESTURE_SENSOR
	data->gesture_input_dev = input_allocate_device();
	if (data->gesture_input_dev == NULL)
		goto err_initialize_input_dev;

	data->gesture_input_dev->name = "gesture_sensor";
	input_set_capability(data->gesture_input_dev, EV_REL, REL_HWHEEL);
	input_set_capability(data->gesture_input_dev, EV_REL, REL_DIAL);
	input_set_capability(data->gesture_input_dev, EV_REL, REL_WHEEL);

	iRet = input_register_device(data->gesture_input_dev);
	if (iRet < 0) {
		input_free_device(data->gesture_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->gesture_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_LIGHT_SENSOR
	data->light_input_dev = input_allocate_device();
	if (data->light_input_dev == NULL)
		goto err_initialize_input_dev;

	data->light_input_dev->name = "light_sensor";
#if defined(CONFIG_SENSORS_SSP_AL3320)
	input_set_capability(data->light_input_dev, EV_REL, REL_RX);
#elif defined(CONFIG_SENSORS_SSP_CM36686)
	input_set_capability(data->light_input_dev, EV_REL, REL_RX);
	input_set_capability(data->light_input_dev, EV_REL, REL_RY);
#else
	input_set_capability(data->light_input_dev, EV_REL, REL_HWHEEL);
	input_set_capability(data->light_input_dev, EV_REL, REL_DIAL);
	input_set_capability(data->light_input_dev, EV_REL, REL_WHEEL);
	input_set_capability(data->light_input_dev, EV_REL, REL_MISC);
#endif

	iRet = input_register_device(data->light_input_dev);
	if (iRet < 0) {
		input_free_device(data->light_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->light_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_PROXIMITY_SENSOR
	data->prox_input_dev = input_allocate_device();
	if (data->prox_input_dev == NULL)
		goto err_initialize_input_dev;

	data->prox_input_dev->name = "proximity_sensor";
	input_set_capability(data->prox_input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(data->prox_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	iRet = input_register_device(data->prox_input_dev);
	if (iRet < 0) {
		input_free_device(data->prox_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->prox_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_TEMP_HUMID_SENSOR
	data->temp_humi_input_dev = input_allocate_device();
	if (data->temp_humi_input_dev == NULL)
		goto err_initialize_input_dev;

	data->temp_humi_input_dev->name = "temp_humidity_sensor";
	input_set_capability(data->temp_humi_input_dev, EV_REL, REL_HWHEEL);
	input_set_capability(data->temp_humi_input_dev, EV_REL, REL_DIAL);
	input_set_capability(data->temp_humi_input_dev, EV_REL, REL_WHEEL);

	iRet = input_register_device(data->temp_humi_input_dev);
	if (iRet < 0) {
		input_free_device(data->temp_humi_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->temp_humi_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_ROT_VECTOR_SENSOR
	data->rot_input_dev = input_allocate_device();
	if (data->rot_input_dev == NULL)
		goto err_initialize_input_dev;

	data->rot_input_dev->name = "rot_sensor";
	input_set_capability(data->rot_input_dev, EV_REL, REL_X);
	input_set_capability(data->rot_input_dev, EV_REL, REL_Y);
	input_set_capability(data->rot_input_dev, EV_REL, REL_Z);
	input_set_capability(data->rot_input_dev, EV_REL, REL_RX);
	input_set_capability(data->rot_input_dev, EV_REL, REL_RY);

	iRet = input_register_device(data->rot_input_dev);
	if (iRet < 0) {
		input_free_device(data->rot_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->rot_input_dev, data);

	data->game_rot_input_dev = input_allocate_device();
	if (data->game_rot_input_dev == NULL)
		goto err_initialize_input_dev;

	data->game_rot_input_dev->name = "game_rot_sensor";
	input_set_capability(data->game_rot_input_dev, EV_REL, REL_X);
	input_set_capability(data->game_rot_input_dev, EV_REL, REL_Y);
	input_set_capability(data->game_rot_input_dev, EV_REL, REL_Z);
	input_set_capability(data->game_rot_input_dev, EV_REL, REL_RX);
	input_set_capability(data->game_rot_input_dev, EV_REL, REL_RY);

	iRet = input_register_device(data->game_rot_input_dev);
	if (iRet < 0) {
		input_free_device(data->game_rot_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->game_rot_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
	data->motion_input_dev = input_allocate_device();
	if (data->motion_input_dev == NULL)
		goto err_initialize_input_dev;

	data->motion_input_dev->name = "LPM_MOTION";
	input_set_capability(data->motion_input_dev, EV_KEY, KEY_HOMEPAGE);

	iRet = input_register_device(data->motion_input_dev);
	if (iRet < 0) {
		input_free_device(data->motion_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->motion_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_HRM_SENSOR
	data->hrm_raw_input_dev = input_allocate_device();
	if (data->hrm_raw_input_dev == NULL)
		goto err_initialize_input_dev;

	data->hrm_raw_input_dev->name = "hrm_raw_sensor";
	input_set_capability(data->hrm_raw_input_dev, EV_REL, REL_X);
	input_set_capability(data->hrm_raw_input_dev, EV_REL, REL_Y);
	input_set_capability(data->hrm_raw_input_dev, EV_REL, REL_Z);
	input_set_capability(data->hrm_raw_input_dev, EV_REL, REL_RX);
	input_set_capability(data->hrm_raw_input_dev, EV_REL, REL_RY);
	input_set_capability(data->hrm_raw_input_dev, EV_REL, REL_RZ);
	input_set_capability(data->hrm_raw_input_dev, EV_REL, REL_HWHEEL);
	input_set_capability(data->hrm_raw_input_dev, EV_REL, REL_DIAL);
	input_set_capability(data->hrm_raw_input_dev, EV_REL, REL_WHEEL);
	input_set_capability(data->hrm_raw_input_dev, EV_REL, REL_MISC);

	iRet = input_register_device(data->hrm_raw_input_dev);
	if (iRet < 0) {
		input_free_device(data->hrm_raw_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->hrm_raw_input_dev, data);

	data->hrm_lib_input_dev = input_allocate_device();
	if (data->hrm_lib_input_dev == NULL)
		goto err_initialize_input_dev;

	data->hrm_lib_input_dev->name = "hrm_lib_sensor";
	input_set_capability(data->hrm_lib_input_dev, EV_REL, REL_X);
	input_set_capability(data->hrm_lib_input_dev, EV_REL, REL_Y);
	input_set_capability(data->hrm_lib_input_dev, EV_REL, REL_Z);

	iRet = input_register_device(data->hrm_lib_input_dev);
	if (iRet < 0) {
		input_free_device(data->hrm_lib_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->hrm_lib_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_UV_SENSOR
	data->uv_input_dev = input_allocate_device();
	if (data->uv_input_dev == NULL)
		goto err_initialize_input_dev;

	data->uv_input_dev->name = "uv_sensor";
	input_set_capability(data->uv_input_dev, EV_REL, REL_MISC);

	iRet = input_register_device(data->uv_input_dev);
	if (iRet < 0) {
		input_free_device(data->uv_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->uv_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_PIR_SENSOR
	data->pir_standard_input_dev = input_allocate_device();
	if (data->pir_standard_input_dev == NULL)
		goto err_initialize_input_dev;

	data->pir_standard_input_dev->name = "pir_standard_sensor";
	input_set_capability(data->pir_standard_input_dev, EV_REL, REL_MISC);

	iRet = input_register_device(data->pir_standard_input_dev);
	if (iRet < 0) {
		input_free_device(data->pir_standard_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->pir_standard_input_dev, data);

	data->pir_long_input_dev = input_allocate_device();
	if (data->pir_long_input_dev == NULL)
		goto err_initialize_input_dev;

	data->pir_long_input_dev->name = "pir_long_sensor";
	input_set_capability(data->pir_long_input_dev, EV_REL, REL_MISC);

	iRet = input_register_device(data->pir_long_input_dev);
	if (iRet < 0) {
		input_free_device(data->pir_long_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->pir_long_input_dev, data);
#endif

#ifdef CONFIG_SENSORS_SSP_META_DATA
	data->meta_input_dev = input_allocate_device();
	if (data->meta_input_dev == NULL)
		goto err_initialize_input_dev;

	data->meta_input_dev->name = "meta_event";
	input_set_capability(data->meta_input_dev, EV_REL, REL_HWHEEL);
	input_set_capability(data->meta_input_dev, EV_REL, REL_DIAL);
	iRet = input_register_device(data->meta_input_dev);
	if (iRet < 0) {
		input_free_device(data->meta_input_dev);
		goto err_initialize_input_dev;
	}
	input_set_drvdata(data->meta_input_dev, data);
#endif

	return SUCCESS;

err_initialize_input_dev:
	remove_input_dev(data);

	return ERROR;
}
