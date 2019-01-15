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
#include <linux/kernel.h>
#include "../ssp.h"

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define VENDOR		"INVENSENSE"
#define CHIP_ID		"MPU6050"

#define CALIBRATION_FILE_PATH	"/csa/sensor/gyro_cal_data"
#define VERBOSE_OUT 1
#define CALIBRATION_DATA_AMOUNT	20
#define DEF_GYRO_FULLSCALE	2000
#define DEF_GYRO_SENS	(32768 / DEF_GYRO_FULLSCALE)
#define DEF_BIAS_LSB_THRESH_SELF	(20 * DEF_GYRO_SENS)
#define DEF_RMS_LSB_TH_SELF (5 * DEF_GYRO_SENS)
#define DEF_RMS_THRESH	((DEF_RMS_LSB_TH_SELF) * (DEF_RMS_LSB_TH_SELF))
#define DEF_SCALE_FOR_FLOAT (1000)
#define DEF_RMS_SCALE_FOR_RMS (10000)
#define DEF_SQRT_SCALE_FOR_RMS (100)

static ssize_t gyro_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t gyro_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_ID);
}

int gyro_open_calibration(struct ssp_data *data)
{
	int iRet = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0660);
	if (IS_ERR(cal_filp)) {
		set_fs(old_fs);
		iRet = PTR_ERR(cal_filp);

		data->gyrocal.x = 0;
		data->gyrocal.y = 0;
		data->gyrocal.z = 0;

		return iRet;
	}

	iRet = cal_filp->f_op->read(cal_filp, (char *)&data->gyrocal,
		sizeof(data->gyrocal), &cal_filp->f_pos);
	if (iRet != sizeof(data->gyrocal))
		iRet = -EIO;

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	ssp_info("open gyro calibration %d, %d, %d",
		data->gyrocal.x, data->gyrocal.y, data->gyrocal.z);
	return iRet;
}

static int save_gyro_caldata(struct ssp_data *data, s16 *iCalData)
{
	int iRet = 0;
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;

	data->gyrocal.x = iCalData[0];
	data->gyrocal.y = iCalData[1];
	data->gyrocal.z = iCalData[2];

	ssp_infof("do gyro calibrate %d, %d, %d",
		data->gyrocal.x, data->gyrocal.y, data->gyrocal.z);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0660);
	if (IS_ERR(cal_filp)) {
		ssp_errf("Can't open calibration file");
		set_fs(old_fs);
		iRet = PTR_ERR(cal_filp);
		return -EIO;
	}

	iRet = cal_filp->f_op->write(cal_filp, (char *)&data->gyrocal,
		sizeof(data->gyrocal), &cal_filp->f_pos);
	if (iRet != sizeof(data->gyrocal)) {
		pr_err("[SSP]: %s - Can't write gyro cal to file\n", __func__);
		iRet = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return iRet;
}

static ssize_t gyro_power_off(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssp_infof("E");

	return snprintf(buf, PAGE_SIZE, "%d\n", 1);
}

static ssize_t gyro_power_on(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssp_infof("E");

	return snprintf(buf, PAGE_SIZE, "%d\n", 1);
}

static ssize_t gyro_get_temp(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char chTempBuf[2] = { 0, 10};
	unsigned char reg[2];
	short temperature = 0;
	int iDelayCnt = 0, iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	data->uFactorydataReady = 0;
	memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

	iRet = send_instruction(data, FACTORY_MODE, GYROSCOPE_TEMP_FACTORY,
		chTempBuf, 2);

	while (!(data->uFactorydataReady & (1 << GYROSCOPE_TEMP_FACTORY))
		&& (iDelayCnt++ < 150)
		&& (iRet == SUCCESS))
		msleep(20);

	if ((iDelayCnt >= 150) || (iRet != SUCCESS)) {
		pr_err("[SSP]: %s - Gyro Temp Timeout!!\n", __func__);
		goto exit;
	}
	reg[0] = data->uFactorydata[1];
	reg[1] = data->uFactorydata[0];
	temperature = (short) (((reg[0]) << 8) | reg[1]);
	temperature = (((temperature + 521) / 340) + 35);
	ssp_infof("%d", temperature);
exit:
	return snprintf(buf, PAGE_SIZE, "%d\n", temperature);
}


u32 mpu6050_selftest_sqrt(u32 sqsum)
{
	u32 sq_rt;
	u32 g0, g1, g2, g3, g4;
	u32 seed;
	u32 next;
	u32 step;

	g4 = sqsum / 100000000;
	g3 = (sqsum - g4 * 100000000) / 1000000;
	g2 = (sqsum - g4 * 100000000 - g3 * 1000000) / 10000;
	g1 = (sqsum - g4 * 100000000 - g3 * 1000000 - g2 * 10000) / 100;
	g0 = (sqsum - g4 * 100000000 - g3 * 1000000 - g2 * 10000 - g1 * 100);

	next = g4;
	step = 0;
	seed = 0;
	while (((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = seed * 10000;
	next = (next - (seed * step)) * 100 + g3;

	step = 0;
	seed = 2 * seed * 10;
	while (((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 1000;
	next = (next - seed * step) * 100 + g2;
	seed = (seed + step) * 10;
	step = 0;
	while (((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 100;
	next = (next - seed * step) * 100 + g1;
	seed = (seed + step) * 10;
	step = 0;

	while (((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 10;
	next = (next - seed * step) * 100 + g0;
	seed = (seed + step) * 10;
	step = 0;

	while (((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step;

	return sq_rt;
}


static ssize_t gyro_selftest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char chTempBuf[2] = { 3, 200};
	u8 initialized = 0;
	int i = 0, j = 0, total_count = 0, ret_val = 0;
	long avg[3] = {0,}, rms[3] = {0,};
	int gyro_bias[3] = {0,}, gyro_rms[3] = {0,};
	s16 iCalData[3] = {0,};
	char a_name[3][2] = { "X", "Y", "Z" };
	int iDelayCnt = 0, iRet = 0;
	int dps_rms[3] = { 0, };
	u32 temp = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	data->uFactorydataReady = 0;
	memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

	iRet = send_instruction(data, FACTORY_MODE, GYROSCOPE_FACTORY,
		chTempBuf, 2);

	while (!(data->uFactorydataReady & (1 << GYROSCOPE_FACTORY))
		&& (iDelayCnt++ < 150)
		&& (iRet == SUCCESS))
		msleep(20);

	if ((iDelayCnt >= 150) || (iRet != SUCCESS)) {
		ssp_errf("Gyro Selftest Timeout!!");
		goto exit;
	}
	for (i = 0; i < 29; i++)
		ssp_errf("uFactorydata[%d] = 0x%x", i, data->uFactorydata[i]);

	initialized = data->uFactorydata[0];
	total_count = (int)((data->uFactorydata[4] << 24) +
				(data->uFactorydata[3] << 16) +
				(data->uFactorydata[2] << 8) +
				data->uFactorydata[1]);
	avg[0] = (long)((data->uFactorydata[8] << 24) +
				(data->uFactorydata[7] << 16) +
				(data->uFactorydata[6] << 8) +
				data->uFactorydata[5]);
	avg[1] = (long)((data->uFactorydata[12] << 24) +
				(data->uFactorydata[11] << 16) +
				(data->uFactorydata[10] << 8) +
				data->uFactorydata[9]);
	avg[2] = (long)((data->uFactorydata[16] << 24) +
				(data->uFactorydata[15] << 16) +
				(data->uFactorydata[14] << 8) +
				data->uFactorydata[13]);
	rms[0] = (long)((data->uFactorydata[20] << 24) +
				(data->uFactorydata[19] << 16) +
				(data->uFactorydata[18] << 8) +
				data->uFactorydata[17]);
	rms[1] = (long)((data->uFactorydata[24] << 24) +
				(data->uFactorydata[23] << 16) +
				(data->uFactorydata[22] << 8) +
				data->uFactorydata[21]);
	rms[2] = (long)((data->uFactorydata[28] << 24) +
				(data->uFactorydata[27] << 16) +
				(data->uFactorydata[26] << 8) +
				data->uFactorydata[25]);
	ssp_info("init: %d, total cnt: %d", initialized, total_count);
	ssp_info("avg %+8ld %+8ld %+8ld (LSB)", avg[0], avg[1], avg[2]);
	ssp_info("rms %+8ld %+8ld %+8ld (LSB)", rms[0], rms[1], rms[2]);
/*
	avg[0] /= total_count;
	avg[1] /= total_count;
	avg[2] /= total_count;
*/
	ssp_info("bias : %+8ld %+8ld %+8ld (LSB)", avg[0], avg[1], avg[2]);

	gyro_bias[0] = (avg[0] * DEF_SCALE_FOR_FLOAT) / DEF_GYRO_SENS;
	gyro_bias[1] = (avg[1] * DEF_SCALE_FOR_FLOAT) / DEF_GYRO_SENS;
	gyro_bias[2] = (avg[2] * DEF_SCALE_FOR_FLOAT) / DEF_GYRO_SENS;
	iCalData[0] = (s16)avg[0];
	iCalData[1] = (s16)avg[1];
	iCalData[2] = (s16)avg[2];

	if (VERBOSE_OUT) {
		ssp_info("abs bias : %+8d.%03d %+8d.%03d %+8d.%03d (dps)",
			(int)abs(gyro_bias[0]) / DEF_SCALE_FOR_FLOAT,
			(int)abs(gyro_bias[0]) % DEF_SCALE_FOR_FLOAT,
			(int)abs(gyro_bias[1]) / DEF_SCALE_FOR_FLOAT,
			(int)abs(gyro_bias[1]) % DEF_SCALE_FOR_FLOAT,
			(int)abs(gyro_bias[2]) / DEF_SCALE_FOR_FLOAT,
			(int)abs(gyro_bias[2]) % DEF_SCALE_FOR_FLOAT);
	}
	for (j = 0; j < 3; j++) {
		if (abs(avg[j]) > DEF_BIAS_LSB_THRESH_SELF) {
			ssp_err("Gyro bias (%ld) exceeded threshold "
				"(threshold = %d LSB)", a_name[j],
				avg[j], DEF_BIAS_LSB_THRESH_SELF);
			ret_val |= 1 << (3 + j);
		}
	}
	/* 3rd, check RMS for dead gyros
	   If any of the RMS noise value returns zero,
	   then we might have dead gyro or FIFO/register failure,
	   the part is sleeping, or the part is not responsive */
	if (rms[0] == 0 || rms[1] == 0 || rms[2] == 0)
		ret_val |= 1 << 6;

	if (VERBOSE_OUT) {
		ssp_info("RMS ^ 2 : %+8ld %+8ld %+8ld",
			(long)rms[0] / total_count,
			(long)rms[1] / total_count, (long)rms[2] / total_count);
	}

	for (j = 0; j < 3; j++) {
		if (rms[j] / total_count > DEF_RMS_THRESH) {
			ssp_err("%s-Gyro rms (%ld) exceeded threshold "
				"(threshold = %d LSB)", a_name[j],
				rms[j] / total_count, DEF_RMS_THRESH);
			ret_val |= 1 << (7 + j);
		}
	}

	for (i = 0; i < 3; i++) {
		if (rms[i] > 10000) {
			temp =
			    ((u32) (rms[i] / total_count)) *
			    DEF_RMS_SCALE_FOR_RMS;
		} else {
			temp =
			    ((u32) (rms[i] * DEF_RMS_SCALE_FOR_RMS)) /
			    total_count;
		}
		if (rms[i] < 0)
			temp = 1 << 31;

		dps_rms[i] = mpu6050_selftest_sqrt(temp) / DEF_GYRO_SENS;

		gyro_rms[i] =
		    dps_rms[i] * DEF_SCALE_FOR_FLOAT / DEF_SQRT_SCALE_FOR_RMS;
	}

	ssp_info("RMS : %+8d.%03d	 %+8d.%03d  %+8d.%03d (dps)",
		(int)abs(gyro_rms[0]) / DEF_SCALE_FOR_FLOAT,
		(int)abs(gyro_rms[0]) % DEF_SCALE_FOR_FLOAT,
		(int)abs(gyro_rms[1]) / DEF_SCALE_FOR_FLOAT,
		(int)abs(gyro_rms[1]) % DEF_SCALE_FOR_FLOAT,
		(int)abs(gyro_rms[2]) / DEF_SCALE_FOR_FLOAT,
		(int)abs(gyro_rms[2]) % DEF_SCALE_FOR_FLOAT);

	if (!ret_val) {
		ssp_err("ret_val == 0");
		save_gyro_caldata(data, iCalData);
	} else {
		ssp_err("ret_val != 0");
		data->gyrocal.x = 0;
		data->gyrocal.y = 0;
		data->gyrocal.z = 0;
		/*initialized = -1;*/
	}

exit:
	ssp_infof("Gyro Selftest - %d,"
		"%d.%03d,%d.%03d,%d.%03d,"
		"%d.%03d,%d.%03d,%d.%03d,"
		"%d,%d,%d",
		ret_val,
		(int)abs(gyro_bias[0]/1000),
		(int)abs(gyro_bias[0])%1000,
		(int)abs(gyro_bias[1]/1000),
		(int)abs(gyro_bias[1])%1000,
		(int)abs(gyro_bias[2]/1000),
		(int)abs(gyro_bias[2])%1000,
		gyro_rms[0]/1000,
		(int)abs(gyro_rms[0])%1000,
		gyro_rms[1]/1000,
		(int)abs(gyro_rms[1])%1000,
		gyro_rms[2]/1000,
		(int)abs(gyro_rms[2])%1000,
		(int)(total_count/3),
		(int)(total_count/3),
		(int)(total_count/3));

	return snprintf(buf, PAGE_SIZE, "%d,"
		"%d.%03d,%d.%03d,%d.%03d,"
		"%d.%03d,%d.%03d,%d.%03d,"
		"%d,%d,%d\n",
		ret_val,
		(int)abs(gyro_bias[0]/1000),
		(int)abs(gyro_bias[0])%1000,
		(int)abs(gyro_bias[1]/1000),
		(int)abs(gyro_bias[1])%1000,
		(int)abs(gyro_bias[2]/1000),
		(int)abs(gyro_bias[2])%1000,
		gyro_rms[0]/1000,
		(int)abs(gyro_rms[0])%1000,
		gyro_rms[1]/1000,
		(int)abs(gyro_rms[1])%1000,
		gyro_rms[2]/1000,
		(int)abs(gyro_rms[2])%1000,
		(int)(total_count/3),
		(int)(total_count/3),
		(int)(total_count/3));
}

static ssize_t gyro_selftest_dps_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int iNewDps = 0;
	int iDelayCnt = 0, iRet = 0;
	char chTempBuf[2] = { 0, 10 };

	struct ssp_data *data = dev_get_drvdata(dev);

	sscanf(buf, "%d", &iNewDps);

	if (iNewDps == GYROSCOPE_DPS250)
		chTempBuf[0] = 0;
	else if (iNewDps == GYROSCOPE_DPS500)
		chTempBuf[0] = 1;
	else if (iNewDps == GYROSCOPE_DPS2000)
		chTempBuf[0] = 2;
	else {
		chTempBuf[0] = 1;
		iNewDps = GYROSCOPE_DPS500;
	}

	data->uFactorydataReady = 0;
	memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

	iRet = send_instruction(data, FACTORY_MODE, GYROSCOPE_DPS_FACTORY,
		chTempBuf, 2);

	while (!(data->uFactorydataReady & (1 << GYROSCOPE_DPS_FACTORY))
		&& (iDelayCnt++ < 150)
		&& (iRet == SUCCESS))
		msleep(20);

	if ((iDelayCnt >= 150) || (iRet != SUCCESS)) {
		ssp_errf("Gyro Selftest DPS Timeout!!");
		goto exit;
	}

	mdelay(5);

	if (data->uFactorydata[0] != SUCCESS) {
		ssp_errf("Gyro Selftest DPS Error!!");
		goto exit;
	}

	data->uGyroDps = (unsigned int)iNewDps;
	ssp_infof("%u dps stored", data->uGyroDps);
exit:
	return count;
}

static ssize_t gyro_selftest_dps_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->uGyroDps);
}

static DEVICE_ATTR(name, S_IRUGO, gyro_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, gyro_vendor_show, NULL);
static DEVICE_ATTR(power_off, S_IRUGO, gyro_power_off, NULL);
static DEVICE_ATTR(power_on, S_IRUGO, gyro_power_on, NULL);
static DEVICE_ATTR(temperature, S_IRUGO, gyro_get_temp, NULL);
static DEVICE_ATTR(selftest, S_IRUGO, gyro_selftest_show, NULL);
static DEVICE_ATTR(selftest_dps, S_IRUGO | S_IWUSR | S_IWGRP,
	gyro_selftest_dps_show, gyro_selftest_dps_store);

static struct device_attribute *gyro_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_selftest,
	&dev_attr_power_on,
	&dev_attr_power_off,
	&dev_attr_temperature,
	&dev_attr_selftest_dps,
	NULL,
};

void initialize_gyro_factorytest(struct ssp_data *data)
{
	sensors_register(data->gyro_device, data, gyro_attrs, "gyro_sensor");
}

void remove_gyro_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->gyro_device, gyro_attrs);
}
