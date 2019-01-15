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

#ifndef __SSP_INPUT_H__
#define __SSP_INPUT_H__

#include "ssp.h"

#define META_EVENT		0
#define META_TIMESTAMP		0

#define PROX_AVG_READ_NUM	80

struct sensor_value;

#ifdef CONFIG_SENSORS_SSP_META_DATA
void report_meta_data(struct ssp_data *, struct sensor_value *);
#endif

#ifdef CONFIG_SENSORS_SSP_ACCELEROMETER_SENSOR
void report_acc_data(struct ssp_data *, struct sensor_value *);
#endif

#ifdef CONFIG_SENSORS_SSP_GYRO_SENSOR
void report_gyro_data(struct ssp_data *, struct sensor_value *);
void report_uncalib_gyro_data(struct ssp_data *, struct sensor_value *);
#endif

#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
void report_mag_data(struct ssp_data *, struct sensor_value *);
void report_mag_uncaldata(struct ssp_data *, struct sensor_value *);
void report_geomagnetic_raw_data(struct ssp_data *, struct sensor_value *);
#endif

#ifdef CONFIG_SENSORS_SSP_PRESSURE_SENSOR
void report_pressure_data(struct ssp_data *, struct sensor_value *);
#endif

#ifdef CONFIG_SENSORS_SSP_GESTURE_SENSOR
void report_gesture_data(struct ssp_data *, struct sensor_value *);
#endif

#ifdef CONFIG_SENSORS_SSP_LIGHT_SENSOR
void report_light_data(struct ssp_data *, struct sensor_value *);
#endif

#ifdef CONFIG_SENSORS_SSP_PROXIMITY_SENSOR
void report_prox_data(struct ssp_data *, struct sensor_value *);
void report_prox_raw_data(struct ssp_data *, struct sensor_value *);
#endif

#ifdef CONFIG_SENSORS_SSP_ROT_VECTOR_SENSOR
void report_rot_data(struct ssp_data *, struct sensor_value *);
void report_game_rot_data(struct ssp_data *, struct sensor_value *);
#endif

#ifdef CONFIG_SENSORS_SSP_TEMP_HUMID_SENSOR
void report_temp_humidity_data(struct ssp_data *, struct sensor_value *);
void report_bulk_comp_data(struct ssp_data *data);
#endif

#ifdef CONFIG_SENSORS_SSP_HRM_SENSOR
void report_hrm_raw_data(struct ssp_data *, struct sensor_value *);
void report_hrm_raw_fac_data(struct ssp_data *, struct sensor_value *);
void report_hrm_lib_data(struct ssp_data *, struct sensor_value *);
#endif

#ifdef CONFIG_SENSORS_SSP_UV_SENSOR
void report_uv_data(struct ssp_data *, struct sensor_value *);
#endif

#ifdef CONFIG_SENSORS_SSP_PIR_SENSOR
void report_pir_data(struct ssp_data *, struct sensor_value *);
void report_pir_raw_data(struct ssp_data *, struct sensor_value *);
#endif
int initialize_input_dev(struct ssp_data *);
void remove_input_dev(struct ssp_data *);

#endif
