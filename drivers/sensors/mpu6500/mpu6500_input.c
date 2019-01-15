/*
	$License:
	Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	$
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/pagemap.h>
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <linux/poll.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#include "../sensors_core.h"
#include "mpu6500_input.h"
#ifdef CONFIG_SENSORS_MPU6500_LP
#include "mpu6500_lp.h"
#endif

#include "./mpu6500_selftest.h"

#define ACC_CAL_PATH	"/csa/sensor/accel_cal_data"
#define GYRO_CAL_PATH	"/csa/sensor/gyro_cal_data"


static const __s8 position_map[][3][3] = {
	{{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1} }, /* 0 top/lower-right */
	{{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1} }, /* 1 top/lower-left */
	{{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1} }, /* 2 top/upper-left */
	{{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1} }, /* 3 top/upper-right */
	{{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1} }, /* 4 bottom/lower-right */
	{{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1} }, /* 5 bottom/lower-left */
	{{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1} }, /* 6 bottom/upper-left */
	{{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1} }, /* 7 bottom/upper-right*/
};

#define LOG_RESULT_LOCATION(x) {\
	printk(KERN_ERR "%s:%s:%d result=%d\n", \
		__FILE__, __func__, __LINE__, x);\
}

#define CHECK_RESULT(x) {\
		result = x;\
		if (unlikely(result)) \
			LOG_RESULT_LOCATION(result);\
}

struct mpu6500_acc {
	s16 x;
	s16 y;
	s16 z;
};

struct motion_int_data {
	unsigned char pwr_mnt[2];
	unsigned char cfg;
	unsigned char accel_cfg;
	unsigned char int_cfg;
	unsigned char smplrt_div;
	unsigned char accel_cfg2;
	bool is_set;
};

struct mpu6500_input_data {
	struct i2c_client *client;
	struct motion_int_data mot_data;
	struct mutex mutex;
	struct mpu6k_input_platform_data *pdata;
	struct input_dev *accel_input_dev;
	struct input_dev *gyro_input_dev;
	struct device *accel_sensor_device;
	struct device *gyro_sensor_device;
#ifdef CONFIG_SENSORS_MPU6500_POLLING
	struct delayed_work accel_work;
	struct delayed_work gyro_work;
#endif
	int position;
	atomic_t accel_enable;
	atomic_t accel_delay;
	atomic_t gyro_enable;
	atomic_t gyro_delay;
	atomic_t irq_is_set;
#ifdef CONFIG_SENSORS_MPU6500_LP
	atomic_t lp_so;		/*low power screen orientation */
	atomic_t lp_bc;		/*low power brightness controller */
	atomic_t pedo;		/*pedometer*/
	unsigned int stepcounter;
	unsigned int is_firmware_loaded;
#endif
	unsigned char gyro_pwr_mgnt[2];
	unsigned char int_pin_cfg;
	u16 enabled_sensors;
	u16 sleep_sensors;
	int current_delay;
	int gyro_bias[3];
	s16 acc_cal[3];

	struct wake_lock reactive_wake_lock;
	int dynamic_threshold[2];
	int movement_recog_flag;
	atomic_t interrupt_state;
	unsigned long motion_recg_st_time;
	struct mpu6500_acc pre_acc;
	/* DTB */
	int acc_int;
	int irq;
};

struct mpu6500_input_cfg {
	int dummy;
};

struct mpu6500_input_data *gb_mpu_data;

static int mpu6500_input_activate_devices(struct mpu6500_input_data *data,
					  int sensors, bool enable);
static int mpu6500_input_resume_accel(struct mpu6500_input_data *data);
static int mpu6500_input_suspend_accel(struct mpu6500_input_data *data);

int mpu6500_i2c_write(struct i2c_client *i2c_client,
		      unsigned int len, unsigned char *data)
{
	struct i2c_msg msgs[1];
	int res;

	if (unlikely(NULL == data || NULL == i2c_client))
		return -EINVAL;

	msgs[0].addr = i2c_client->addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = (unsigned char *)data;
	msgs[0].len = len;

	res = i2c_transfer(i2c_client->adapter, msgs, 1);
	if (unlikely(res < 1))
		return res;
	else
		return 0;
}

int mpu6500_i2c_read(struct i2c_client *i2c_client,
		     unsigned int len, unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (unlikely(NULL == data || NULL == i2c_client))
		return -EINVAL;

	msgs[0].addr = i2c_client->addr;
	msgs[0].flags = I2C_M_RD;
	msgs[0].buf = data;
	msgs[0].len = len;

	res = i2c_transfer(i2c_client->adapter, msgs, 1);
	if (unlikely(res < 1))
		return res;
	else
		return 0;
}

int mpu6500_i2c_write_single_reg(struct i2c_client *i2c_client,
				 unsigned char reg, unsigned char value)
{

	unsigned char data[2];

	data[0] = reg;
	data[1] = value;

	return mpu6500_i2c_write(i2c_client, 2, data);
}

int mpu6500_i2c_read_reg(struct i2c_client *i2c_client,
			 unsigned char reg, unsigned int len,
			 unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (unlikely(NULL == data || NULL == i2c_client))
		return -EINVAL;

	msgs[0].addr = i2c_client->addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;

	msgs[1].addr = i2c_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = len;

	res = i2c_transfer(i2c_client->adapter, msgs, 2);
	if (unlikely(res < 1))
		return res;
	else
		return 0;
}

int mpu6500_i2c_read_fifo(struct i2c_client *i2c_client,
			  unsigned short length, unsigned char *data)
{
	int result;
	unsigned short bytes_read = 0;
	unsigned short this_len;

	while (bytes_read < length) {
		this_len = length - bytes_read;

		result =
		    mpu6500_i2c_read_reg(i2c_client, MPUREG_FIFO_R_W, this_len,
					 &data[bytes_read]);
		if (result) {
			mpu_err("i2c read failed(%d)\n", result);
			return result;
		}

		bytes_read += this_len;
	}

	return 0;
}

int mpu6500_i2c_memory_write(struct i2c_client *i2c_client,
			     unsigned short mem_addr, unsigned int len,
			     unsigned char const *data)
{
	unsigned char bank[2];
	unsigned char addr[2];
	unsigned char buf[513];

	struct i2c_msg msgs[3];
	int res;

	if (!data || !i2c_client)
		return -EINVAL;

	if (len >= (sizeof(buf) - 1))
		return -ENOMEM;

	bank[0] = MPUREG_BANK_SEL;
	bank[1] = mem_addr >> 8;

	addr[0] = MPUREG_MEM_START_ADDR;
	addr[1] = mem_addr & 0xFF;

	buf[0] = MPUREG_MEM_R_W;
	memcpy(buf + 1, data, len);

	/* write message */
	msgs[0].addr = i2c_client->addr;
	msgs[0].flags = 0;
	msgs[0].buf = bank;
	msgs[0].len = sizeof(bank);

	msgs[1].addr = i2c_client->addr;
	msgs[1].flags = 0;
	msgs[1].buf = addr;
	msgs[1].len = sizeof(addr);

	msgs[2].addr = i2c_client->addr;
	msgs[2].flags = 0;
	msgs[2].buf = (unsigned char *)buf;
	msgs[2].len = len + 1;

	res = i2c_transfer(i2c_client->adapter, msgs, 3);
	if (res != 3) {
		if (res >= 0)
			res = -EIO;
		return res;
	} else {
		return 0;
	}
}

int mpu6500_i2c_memory_read(struct i2c_client *i2c_client,
			    unsigned short mem_addr, unsigned int len,
			    unsigned char const *data)
{
	unsigned char bank[2];
	unsigned char addr[2];
	unsigned char buf;

	struct i2c_msg msgs[4];
	int res;

	if (!data || !i2c_client)
		return -EINVAL;

	bank[0] = MPUREG_BANK_SEL;
	bank[1] = mem_addr >> 8;

	addr[0] = MPUREG_MEM_START_ADDR;
	addr[1] = mem_addr & 0xFF;

	buf = MPUREG_MEM_R_W;

	/* write message */
	msgs[0].addr = i2c_client->addr;
	msgs[0].flags = 0;
	msgs[0].buf = bank;
	msgs[0].len = sizeof(bank);

	msgs[1].addr = i2c_client->addr;
	msgs[1].flags = 0;
	msgs[1].buf = addr;
	msgs[1].len = sizeof(addr);

	msgs[2].addr = i2c_client->addr;
	msgs[2].flags = 0;
	msgs[2].buf = &buf;
	msgs[2].len = 1;

	msgs[3].addr = i2c_client->addr;
	msgs[3].flags = I2C_M_RD;
	msgs[3].buf = (void *)data;
	msgs[3].len = len;

	res = i2c_transfer(i2c_client->adapter, msgs, 4);
	if (res != 4) {
		if (res >= 0)
			res = -EIO;
		return res;
	} else {
		return 0;
	}

}

static void mpu6500_proc_msleep(unsigned int msecs,
				struct hrtimer_sleeper *sleeper, int sigs)
{
	enum hrtimer_mode mode = HRTIMER_MODE_REL;
	int state = sigs ? TASK_INTERRUPTIBLE : TASK_UNINTERRUPTIBLE;

	hrtimer_init(&sleeper->timer, CLOCK_MONOTONIC, mode);
	sleeper->timer._softexpires = ktime_set(0, msecs * NSEC_PER_MSEC);
	hrtimer_init_sleeper(sleeper, current);

	do {
		set_current_state(state);
		hrtimer_start(&sleeper->timer,
			sleeper->timer._softexpires, mode);
		if (sleeper->task)
			schedule();
		hrtimer_cancel(&sleeper->timer);
		mode = HRTIMER_MODE_ABS;
	} while (sleeper->task && !(sigs && signal_pending(current)));
}

void mpu6500_msleep(unsigned int msecs)
{
	struct hrtimer_sleeper sleeper;

	mpu6500_proc_msleep(msecs, &sleeper, 0);
}

static int mpu6500_input_set_mode(struct mpu6500_input_data *data, u8 mode)
{
	int err = 0;

	if (mode == MPU6500_MODE_SLEEP) {
		err = mpu6500_input_activate_devices(data,
					       MPU6500_SENSOR_ACCEL |
					       MPU6500_SENSOR_GYRO
#ifdef CONFIG_SENSORS_MPU6500_LP
					       | MPU6500_SENSOR_LPSO
					       | MPU6500_SENSOR_LPBC
#endif
					       , false);
	}

	if (mode == MPU6500_MODE_NORMAL) {
		if (atomic_read(&data->accel_enable))
			err = mpu6500_input_activate_devices(data,
					MPU6500_SENSOR_ACCEL, true);
		if (atomic_read(&data->gyro_enable))
			err = mpu6500_input_activate_devices(data,
					MPU6500_SENSOR_GYRO, true);
#ifdef CONFIG_SENSORS_MPU6500_LP
		if (atomic_read(&data->lp_so))
			err = mpu6500_input_activate_devices(data,
					MPU6500_SENSOR_LPSO, true);
		if (atomic_read(&data->lp_bc))
			err = mpu6500_input_activate_devices(data,
					MPU6500_SENSOR_LPBC, true);

#endif
	}

	return err;
}

static void mpu6500_apply_orientation(struct mpu6500_input_data *data,
	s16 raw[3], s16 orient_raw[3])
{
	int i = 0, j = 0;
	s32 value = 0;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++)
			value += position_map[data->position][i][j] * raw[j];

		/*trim the edge */
		if (value > 32767)
			value = 32767;
		else if (value < -32768)
			value = -32768;

		orient_raw[i] = (s16) value;
		value = 0;
	}
}

static int mpu6500_input_read_accel_raw_data(struct mpu6500_input_data *data,
	struct mpu6500_acc *acc_raw_data)
{
	u8 regs[6] = {0, };
	s16 raw[3] = {0,}, orien_raw[3] = {0,};
	int result;

	result = mpu6500_i2c_read_reg(data->client,
				MPUREG_ACCEL_XOUT_H, 6, regs);

	if (result)
		mpu_err("i2c read failed(%d)\n", result);
	raw[0] = ((s16) ((s16) regs[0] << 8)) | regs[1];
	raw[1] = ((s16) ((s16) regs[2] << 8)) | regs[3];
	raw[2] = ((s16) ((s16) regs[4] << 8)) | regs[5];

	mpu6500_apply_orientation(data, raw, orien_raw);

	acc_raw_data->x = orien_raw[0];
	acc_raw_data->y = orien_raw[1];
	acc_raw_data->z = orien_raw[2];

	return result;
}

static int mpu6500_input_read_accel_data(struct mpu6500_input_data *data,
	struct mpu6500_acc *acc_data)
{
	int result = 0;
	static int count;
	struct mpu6500_acc acc_raw_data;

	if (!(data->enabled_sensors & MPU6500_SENSOR_ACCEL)) {
		mpu6500_input_resume_accel(data);
		usleep_range(10000, 11000);
	}
	result = mpu6500_input_read_accel_raw_data(data, &acc_raw_data);

	/* apply calibration data*/
	acc_data->x = acc_raw_data.x - data->acc_cal[0];
	acc_data->y = acc_raw_data.y - data->acc_cal[1];
	acc_data->z = acc_raw_data.z - data->acc_cal[2];

	if ((atomic_read(&data->accel_delay) * count >= 1500) &&
		(abs(data->pre_acc.x-acc_data->x) >= 4000 ||
		abs(data->pre_acc.y-acc_data->y) >= 4000 ||
		abs(data->pre_acc.z-acc_data->z) >= 4000)) {
		mpu_info("accel data => x = %d, y = %d, z = %d\n",
			acc_data->x, acc_data->y, acc_data->z);

		data->pre_acc.x = acc_data->x;
		data->pre_acc.y = acc_data->y;
		data->pre_acc.z = acc_data->z;
		count = 0;
	} else
		count++;

	if (!(data->enabled_sensors & MPU6500_SENSOR_ACCEL)) {
		mpu6500_input_suspend_accel(data);
		usleep_range(10000, 11000);
	}

	return result;
}

static void mpu6500_input_report_accel_xyz(struct mpu6500_input_data *data)
{
	int result;
	struct mpu6500_acc acc_data;

	result = mpu6500_input_read_accel_data(data, &acc_data);

	input_report_rel(data->accel_input_dev, REL_X, acc_data.x);
	input_report_rel(data->accel_input_dev, REL_Y, acc_data.y);
	input_report_rel(data->accel_input_dev, REL_Z, acc_data.z);

	input_sync(data->accel_input_dev);
}

static void mpu6500_input_report_gyro_xyz(struct mpu6500_input_data *data)
{
	u8 regs[6] = {0, };
	s16 raw[3], orien_raw[3];
	static int count;
	int result;

	result = mpu6500_i2c_read_reg(data->client,
				MPUREG_GYRO_XOUT_H, 6, regs);
	if (result)
		mpu_err("i2c read failed(%d)\n", result);

	raw[0] =
	    (((s16) ((s16) regs[0] << 8)) | regs[1]) - (s16) data->gyro_bias[0];
	raw[1] =
	    (((s16) ((s16) regs[2] << 8)) | regs[3]) - (s16) data->gyro_bias[1];
	raw[2] =
	    (((s16) ((s16) regs[4] << 8)) | regs[5]) - (s16) data->gyro_bias[2];

	mpu6500_apply_orientation(data, raw, orien_raw);

	if (atomic_read(&data->gyro_delay) * count >= 3000) {
		mpu_info("count=%d, gyro_data => x = %d, y = %d, z = %d\n",
			count, orien_raw[0], orien_raw[1], orien_raw[2]);
		count = 0;
	} else
		count++;

	input_report_rel(data->gyro_input_dev, REL_RX, orien_raw[0]);
	input_report_rel(data->gyro_input_dev, REL_RY, orien_raw[1]);
	input_report_rel(data->gyro_input_dev, REL_RZ, orien_raw[2]);

	input_sync(data->gyro_input_dev);
}

#ifdef CONFIG_SENSORS_MPU6500_LP
static void mpu6500_input_report_fifo_data(struct mpu6500_input_data *data)
{
	unsigned char val[3] = {0, };
	unsigned char fifo_data[256] = { 0 };
	unsigned short fifo_count = 0;
#ifndef CONFIG_SENSORS_MPU6500_POLLING
	short accel[3] = { 0 }, gyro[3] = { 0 };
	int i = 0;
	short orien_raw[3];
#endif
	unsigned int event = 0;
	unsigned int source = 0;
	u8 so_data ;
	u8 bc_event;
	bool need_accel_sync = false;
#ifndef CONFIG_SENSORS_MPU6500_POLLING
	bool need_gyro_sync = false;
#endif

	mpu6500_i2c_read_reg(data->client, MPUREG_FIFO_COUNTH, 2, val);

	fifo_count = (unsigned short)(val[0] << 8) | val[1];

	if (fifo_count > 0) {
		mpu6500_i2c_read_fifo(data->client, 16, fifo_data);

#ifndef CONFIG_SENSORS_MPU6500_POLLING
		for (i = 0; i < 3; i++) {
			accel[i] = be16_to_cpup((__be16 *) (&fifo_data[i * 2]));
			gyro[i] =
			    be16_to_cpup((__be16 *) (&fifo_data[i * 2 + 6]));
		}

		if (data->enabled_sensors & MPU6500_SENSOR_ACCEL) {
			mpu6500_apply_orientation(data, accel, orien_raw);
			input_report_rel(data->accel_input_dev, REL_X,
					orien_raw[0] - data->acc_cal[0]);
			input_report_rel(data->accel_input_dev, REL_Y,
					orien_raw[1] - data->acc_cal[1]);
			input_report_rel(data->accel_input_dev, REL_Z,
					orien_raw[2] - data->acc_cal[2]);

			/* apply calibration data*/
			orien_raw[0] -= data->acc_cal[0];
			orien_raw[1] -= data->acc_cal[1];
			orien_raw[2] -= data->acc_cal[2];

			mpu_info("accel_raw=> x = %d, y = %d, z = %d\n",
				orien_raw[0], orien_raw[1], orien_raw[2]);

			need_accel_sync = true;
		}

		if (data->enabled_sensors & MPU6500_SENSOR_GYRO) {
			mpu6500_apply_orientation(data, gyro, orien_raw);

			input_report_rel(data->gyro_input_dev,
						REL_RX, orien_raw[0]);
			input_report_rel(data->gyro_input_dev,
						REL_RY, orien_raw[1]);
			input_report_rel(data->gyro_input_dev,
						REL_RZ, orien_raw[2]);
			mpu_info("gyro_raw=> x = %d, y = %d, z = %d\n",
				orien_raw[0], orien_raw[1], orien_raw[2]);

			need_gyro_sync = true;
		}
#endif
		event = (unsigned int)(be32_to_cpup
					((unsigned int *)&fifo_data[12]));
		source = ((event >> 16) & 0xff);
		if (source & INT_SRC_DISPLAY_ORIENT) {
			so_data = ((DMP_MASK_DIS_ORIEN & (event & 0xff)) >>
							DMP_DIS_ORIEN_SHIFT);

			if (data->enabled_sensors & MPU6500_SENSOR_LPSO) {
				mpu_info("DISPLAY_ORIENTATION : %d\n", so_data);
				input_report_rel(data->accel_input_dev,
					REL_MISC, so_data + 1);
				need_accel_sync = true;
			}
		}

		if (source & INT_SRC_SCREEN_BRIGHTNESS) {
			if (data->enabled_sensors & MPU6500_SENSOR_LPBC) {
				bc_event = (event >> 24) % 256;
				mpu_info("BRIGHTNESS CONTROLLER : %d\n",
					bc_event);
				input_report_rel(data->accel_input_dev,
					REL_HWHEEL, bc_event + 1);
				need_accel_sync = true;
			}
		}

		if (need_accel_sync)
			input_sync(data->accel_input_dev);
#ifndef CONFIG_SENSORS_MPU6500_POLLING
		if (need_gyro_sync)
			input_sync(data->gyro_input_dev);
#endif
	}
}

#endif

static irqreturn_t mpu6500_input_irq_thread(int irq, void *dev)
{
	struct mpu6500_input_data *data = (struct mpu6500_input_data *)dev;
	struct motion_int_data *mot_data = &data->mot_data;
	unsigned char reg = 0;
	unsigned long timediff = 0;
	int result = 0;

	result = mpu6500_i2c_read_reg(data->client, MPUREG_INT_STATUS, 1, &reg);
	if (result) {
		mpu_err("i2c_read err= %d\n", result);
		goto done;
	}

	if (!mot_data->is_set) {
#ifdef CONFIG_SENSORS_MPU6500_LP
		if (LP_NEEDS_FIFO(data->enabled_sensors))
			mpu6500_input_report_fifo_data(data);
		else {
#ifndef CONFIG_SENSORS_MPU6500_POLLING
			if (data->enabled_sensors & MPU6500_SENSOR_ACCEL)
				mpu6500_input_report_accel_xyz(data);
			if (data->enabled_sensors & MPU6500_SENSOR_GYRO)
				mpu6500_input_report_gyro_xyz(data);
#endif
		}
#else
#ifndef CONFIG_SENSORS_MPU6500_POLLING
		if (data->enabled_sensors & MPU6500_SENSOR_ACCEL)
			mpu6500_input_report_accel_xyz(data);

		if (data->enabled_sensors & MPU6500_SENSOR_GYRO)
			mpu6500_input_report_gyro_xyz(data);
#endif
#endif
	} else {
		timediff = jiffies_to_msecs(jiffies -
				data->motion_recg_st_time);

		/* ignore motion interrupt happened in 100ms
		   to skip intial erronous interrupt */
		if (timediff < 150 &&
			(data->movement_recog_flag != REACTIVE_FACTORY)) {
			mpu_err("timediff = %ld msec\n", timediff);
			goto done;
		}
		if (reg & BIT_MOT_EN ||
			(data->movement_recog_flag == REACTIVE_FACTORY)) {
			/* handle motion recognition */
			atomic_set(&data->interrupt_state, true);
			mpu_info("motion interrupt happened\n");
			/* disable motion int */
			mpu6500_i2c_write_single_reg(data->client,
				MPUREG_INT_ENABLE, mot_data->int_cfg);

			wake_lock_timeout(&data->reactive_wake_lock,
				5 * HZ);
		}
	}
done:
	return IRQ_HANDLED;
}

static int mpu6500_input_set_fsr(struct mpu6500_input_data *data, int fsr)
{
	unsigned char fsr_mask;
	int result;
	unsigned char reg = 0;

	if (fsr <= 2000) {
		fsr_mask = 0x00;
	} else if (fsr <= 4000) {
		fsr_mask = 0x08;
	} else if (fsr <= 8000) {
		fsr_mask = 0x10;
	} else {		/* fsr = [8001, oo) */
		fsr_mask = 0x18;
	}

	result = mpu6500_i2c_read_reg(data->client,
				MPUREG_ACCEL_CONFIG, 1, &reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	result = mpu6500_i2c_write_single_reg(data->client,
				MPUREG_ACCEL_CONFIG, reg | fsr_mask);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}

	return result;
}

static int mpu6500_input_set_fp_mode(struct mpu6500_input_data *data)
{
	unsigned char b;

	/* Resetting the cycle bit and LPA wake up freq */
	mpu6500_i2c_read_reg(data->client, MPUREG_PWR_MGMT_1, 1, &b);
	b &= ~BIT_CYCLE & ~BIT_PD_PTAT;
	mpu6500_i2c_write_single_reg(data->client, MPUREG_PWR_MGMT_1, b);
	mpu6500_i2c_read_reg(data->client, MPUREG_PWR_MGMT_2, 1, &b);
	b &= ~BITS_LPA_WAKE_CTRL;
	mpu6500_i2c_write_single_reg(data->client, MPUREG_PWR_MGMT_2, b);
	/* Resetting the duration setting for fp mode */
	b = (unsigned char)10 / ACCEL_MOT_DUR_LSB;
	mpu6500_i2c_write_single_reg(data->client,
				     MPUREG_ACCEL_INTEL_ENABLE, b);

	return 0;
}

static int mpu6500_input_set_odr(struct mpu6500_input_data *data, int odr)
{
	int result;
	unsigned char b = 0;
	struct motion_int_data *mot_data = &data->mot_data;

#ifdef CONFIG_SENSORS_MPU6500_LP
	if (IS_LP_ENABLED(data->enabled_sensors))
		odr = MIN(odr, 4);
#endif

	b = (unsigned char)(odr);

	if (mot_data->is_set) {
		mot_data->smplrt_div = b;
		result = 0;
		goto done;
	}

	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_SMPLRT_DIV, b));

	mpu6500_i2c_read_reg(data->client, MPUREG_PWR_MGMT_1, 1, &b);
	b &= BIT_CYCLE;
	if (b == BIT_CYCLE) {
		mpu_info("Accel LP - > FP mode.\n ");
		mpu6500_input_set_fp_mode(data);
	}
#ifdef CONFIG_SENSORS_MPU6500_LP
	if (IS_LP_ENABLED(data->enabled_sensors))
		mpu6500_lp_set_delay(data->client, data->current_delay);
#endif

done:
	return result;
}

static int
mpu6500_input_set_motion_interrupt(struct mpu6500_input_data *data,
				bool enable, int mode)
{
	struct motion_int_data *mot_data = &data->mot_data;
	unsigned char reg = 0;

	atomic_set(&data->interrupt_state, false);

	if (enable) {
		if (!mot_data->is_set) {
			mpu6500_i2c_read_reg(data->client,
					     MPUREG_PWR_MGMT_1, 2,
					     mot_data->pwr_mnt);
			mpu6500_i2c_read_reg(data->client, MPUREG_CONFIG, 1,
					     &mot_data->cfg);
			mpu6500_i2c_read_reg(data->client, MPUREG_ACCEL_CONFIG,
					     1, &mot_data->accel_cfg);
			mpu6500_i2c_read_reg(data->client, MPUREG_INT_ENABLE, 1,
					     &mot_data->int_cfg);
			mpu6500_i2c_read_reg(data->client, MPUREG_SMPLRT_DIV, 1,
					     &mot_data->smplrt_div);
			mpu6500_i2c_read_reg(data->client,
					MPUREG_ACCEL_CONFIG2, 1,
					&mot_data->accel_cfg2);
		}

		/* 1) initialize  */
		mpu6500_i2c_read_reg(data->client, MPUREG_INT_STATUS, 1, &reg);

		/* Power up the chip and clear the cycle bit. Full power */
		reg = 0x01;
		mpu6500_i2c_write_single_reg(data->client, MPUREG_PWR_MGMT_1,
					     reg);

		mdelay(50);

		/* 2) LPF */
		if (mode == REACTIVE_FACTORY)
			reg = 0x0; /*260Hz LPF */
		else
			reg = 0x1; /*184Hz LPF */
		mpu6500_i2c_write_single_reg(data->client, MPUREG_CONFIG, reg);

		reg = 0x0; /* Clear Accel Config. */
		mpu6500_i2c_write_single_reg(data->client,
			MPUREG_ACCEL_CONFIG, reg);

		/* set LPM*/
		mpu6500_i2c_write_single_reg(data->client, MPUREG_ACCEL_CONFIG2,
			BIT_ACCEL_FCHOICE_B);

		/* 3. set motion thr & dur */
		if (mode == REACTIVE_FACTORY)
			reg = 0x41; /* motion & drdy enable */
		else
			reg = 0x40; /* motion interrupt enable */
		mpu6500_i2c_write_single_reg(data->client,
					MPUREG_INT_ENABLE, reg);

		reg = 4;	/* 3.91 Hz (low power accel odr) */
		mpu6500_i2c_write_single_reg(data->client,
			MPUREG_LP_ACCEL_ODR, reg);

		reg = 0xC0; /* Motion Duration =1 ms */
		mpu6500_i2c_write_single_reg(data->client,
			MPUREG_ACCEL_INTEL_CTRL, reg);

		/* 5) set motion thr & lp odr */
		if (mode == REACTIVE_FACTORY)
			reg = 0x0;
		else
			reg = MIN(MAX_THRESHOLD,
					(MAX(data->dynamic_threshold[0],
						data->dynamic_threshold[1]))/4);
		mpu6500_i2c_write_single_reg(data->client, MPUREG_WOM_THR, reg);

		if (mode != REACTIVE_FACTORY) {
			/* 4. setup the lower power mode for PWM-2 register */
			reg = mot_data->pwr_mnt[1];
			reg |= (BITS_LPA_WAKE_20HZ); /* the freq of wakeup */
			/* put gyro in standby. */
			reg |= (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG);
			reg &= ~(BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA);
			mpu6500_i2c_write_single_reg(data->client,
				MPUREG_PWR_MGMT_2, reg);
			/* Auto selects the best available clock source */
			reg = 0x1;
			/* Set the cycle bit to be 1. LP MODE */
			reg |= BIT_CYCLE;
			reg &= ~BIT_PD_PTAT; /* Clear the temp disp bit. */
			mpu6500_i2c_write_single_reg(data->client,
				MPUREG_PWR_MGMT_1, reg);
		}

		data->motion_recg_st_time = jiffies;
	} else {
		if (mot_data->is_set) {
			mpu6500_i2c_write_single_reg(data->client,
						     MPUREG_PWR_MGMT_1,
						     mot_data->pwr_mnt[0]);
			mdelay(50);
			mpu6500_i2c_write_single_reg(data->client,
						     MPUREG_PWR_MGMT_2,
						     mot_data->pwr_mnt[1]);

			mpu6500_i2c_write_single_reg(data->client,
						     MPUREG_CONFIG,
						     mot_data->cfg);
			mpu6500_i2c_write_single_reg(data->client,
						     MPUREG_ACCEL_CONFIG2,
						     mot_data->accel_cfg2);
			mpu6500_i2c_write_single_reg(data->client,
						     MPUREG_ACCEL_INTEL_ENABLE,
						     0x0);
			mpu6500_i2c_write_single_reg(data->client,
						     MPUREG_ACCEL_CONFIG,
						     mot_data->accel_cfg);

			mpu6500_i2c_write_single_reg(data->client,
						     MPUREG_INT_ENABLE,
						     mot_data->int_cfg);
			reg = 0xff;	/* Motion Duration =1 ms */
			mpu6500_i2c_write_single_reg(data->client,
						     MPUREG_WOM_THR, reg);
			mpu6500_i2c_read_reg(data->client, MPUREG_INT_STATUS, 1,
					     &reg);
			mpu6500_i2c_write_single_reg(data->client,
						     MPUREG_SMPLRT_DIV,
						     mot_data->smplrt_div);

			if (atomic_read(&data->accel_enable)) {
				mpu_info("Enable accerometer sensor\n");
				mpu6500_input_activate_devices(data,
						MPU6500_SENSOR_ACCEL, true);
			}
		}
	}
	mot_data->is_set = enable;

	return 0;
}

static int
mpu6500_input_set_irq(struct mpu6500_input_data *data, unsigned char irq)
{
	int result;

	if (irq) {
		CHECK_RESULT(mpu6500_i2c_write_single_reg
			     (data->client, MPUREG_INT_PIN_CFG,
			      data->int_pin_cfg | BIT_BYPASS_EN));
	}

	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_INT_ENABLE, irq));

	return result;
}

static int mpu6500_input_suspend_accel(struct mpu6500_input_data *data)
{
	unsigned char reg = 0;
	int result;

	CHECK_RESULT(mpu6500_i2c_read_reg
		     (data->client, MPUREG_PWR_MGMT_2, 1, &reg));

	reg |= (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA);
	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_PWR_MGMT_2, reg));

	return result;
}

static int mpu6500_input_resume_accel(struct mpu6500_input_data *data)
{
	int result = 0;
	unsigned char reg = 0;

	CHECK_RESULT(mpu6500_i2c_read_reg
		     (data->client, MPUREG_PWR_MGMT_1, 1, &reg));

	if (reg & BIT_SLEEP) {
		CHECK_RESULT(mpu6500_i2c_write_single_reg(data->client,
							  MPUREG_PWR_MGMT_1,
							  reg & ~BIT_SLEEP));
	}

	usleep_range(2000, 2100);

	CHECK_RESULT(mpu6500_i2c_read_reg
		     (data->client, MPUREG_PWR_MGMT_2, 1, &reg));

	reg &= ~(BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA);
	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_PWR_MGMT_2, reg));

	/* settings */

	/*----- LPF configuration  : 41hz ---->*/
	reg = MPU_FILTER_41HZ;
	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_ACCEL_CONFIG2, reg));
	/*<----- LPF configuration  : 41hz ---- */

	CHECK_RESULT(mpu6500_i2c_read_reg
		     (data->client, MPUREG_ACCEL_CONFIG, 1, &reg));
	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_ACCEL_CONFIG, reg | 0x0));
	CHECK_RESULT(mpu6500_input_set_fsr(data, 2000));

	return result;
}

static int
mpu6500_input_actiave_accel(struct mpu6500_input_data *data, bool enable)
{
	int result = 0;

	if (enable) {
		result = mpu6500_input_resume_accel(data);
		if (result) {
			LOG_RESULT_LOCATION(result);
			return result;
		} else {
			data->enabled_sensors |= MPU6500_SENSOR_ACCEL;
		}
	} else {
#ifdef CONFIG_SENSORS_MPU6500_LP
		if (!LP_NEEDS_ACCEL(data->enabled_sensors))
			result = mpu6500_input_suspend_accel(data);
#else
		result = mpu6500_input_suspend_accel(data);
#endif
		if (result == 0)
			data->enabled_sensors &= ~MPU6500_SENSOR_ACCEL;
	}

	return result;
}

static int mpu6500_input_suspend_gyro(struct mpu6500_input_data *data)
{
	int result = 0;

	CHECK_RESULT(mpu6500_i2c_read_reg
		     (data->client, MPUREG_PWR_MGMT_1, 2, data->gyro_pwr_mgnt));

	data->gyro_pwr_mgnt[1] |= (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG);

	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_PWR_MGMT_2, data->gyro_pwr_mgnt[1]));

	return result;
}

static int mpu6500_input_resume_gyro(struct mpu6500_input_data *data)
{
	int result = 0;
	unsigned regs[2] = { 0, };

	CHECK_RESULT(mpu6500_i2c_read_reg
		     (data->client, MPUREG_PWR_MGMT_1, 2, data->gyro_pwr_mgnt));

	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_PWR_MGMT_1,
		      data->gyro_pwr_mgnt[0] & ~BIT_SLEEP));

	data->gyro_pwr_mgnt[1] &= ~(BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG);

	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_PWR_MGMT_2, data->gyro_pwr_mgnt[1]));

	regs[0] = MPU_FS_500DPS << 3;	/* 2000dps */
	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_GYRO_CONFIG, regs[0]));

	regs[0] = MPU_FILTER_41HZ | 0x18;
	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_CONFIG, regs[0]));

	return result;
}

static int
mpu6500_input_activate_gyro(struct mpu6500_input_data *data, bool enable)
{
	int result;
	if (enable) {
		result = mpu6500_input_resume_gyro(data);
		if (result) {
			LOG_RESULT_LOCATION(result);
			return result;
		} else {
			data->enabled_sensors |= MPU6500_SENSOR_GYRO;
		}
	} else {
		result = mpu6500_input_suspend_gyro(data);
		if (result == 0)
			data->enabled_sensors &= ~MPU6500_SENSOR_GYRO;
	}

	return result;
}

static int mpu6500_set_delay(struct mpu6500_input_data *data)
{
	int result = 0;
	int delay = 200;
	unsigned char irq = 0;
	struct motion_int_data *mot_data = &data->mot_data;

	if (data->enabled_sensors & MPU6500_SENSOR_ACCEL) {
		delay = MIN(delay, atomic_read(&data->accel_delay));
#ifdef CONFIG_SENSORS_MPU6500_POLLING
		irq = 0x00;
#else
		irq = BIT_RAW_RDY_EN;
#endif
	}

	if (data->enabled_sensors & MPU6500_SENSOR_GYRO) {
		delay = MIN(delay, atomic_read(&data->gyro_delay));
#ifdef CONFIG_SENSORS_MPU6500_POLLING
		irq = 0x00;
#else
		irq = BIT_RAW_RDY_EN;
#endif
	}
#ifdef CONFIG_SENSORS_MPU6500_LP
	if (LP_NEEDS_FIFO(data->enabled_sensors))
		irq = BIT_DMP_INT_EN;
#endif

	data->current_delay = delay;

	CHECK_RESULT(mpu6500_input_set_odr(data, data->current_delay));
	if (!mot_data->is_set)
		CHECK_RESULT(mpu6500_input_set_irq(data, irq));
	return result;
}

#ifdef CONFIG_SENSORS_MPU6500_LP
static int mpu6500_input_activate_lp_func(struct mpu6500_input_data *data,
			       u16 sensor, bool enable)
{
	int result = 0;
	if (enable) {
		/* enable screen orientation */
		if (!(data->enabled_sensors & MPU6500_SENSOR_ACCEL))
			result = mpu6500_input_resume_accel(data);

		if (!result) {
			if (sensor & MPU6500_SENSOR_LPBC)
				result = mpu6500_lp_active_bc_interrupt
					(data->client, enable);

			if (sensor & MPU6500_SENSOR_PEDO)
				mpu6500_lp_ped_set_septctrl(data->client,
							data->stepcounter);

			if (IS_LP_ENABLED(sensor)
			    || IS_LP_ENABLED(data->enabled_sensors))
				result = mpu6500_lp_activate_lpfunc(
					data->client,
					enable,
					data->current_delay,
					LP_NEEDS_FIFO(data->enabled_sensors));
		}

		if (result) {
			LOG_RESULT_LOCATION(result);
			return result;
		} else
			data->enabled_sensors |= LP_MASK(sensor);
	} else {
		if (sensor & MPU6500_SENSOR_LPBC)
			result =
			mpu6500_lp_active_bc_interrupt(data->client, enable);

		if (sensor & MPU6500_SENSOR_PEDO)
			data->stepcounter =
				mpu6500_lp_ped_get_septctrl(data->client);

		if (!result)
			data->enabled_sensors &= ~LP_MASK(sensor);

		if (!IS_LP_ENABLED(data->enabled_sensors))
			result = mpu6500_lp_activate_lpfunc(data->client,
						enable, data->current_delay, 0);

		if (!(data->enabled_sensors & MPU6500_SENSOR_ACCEL) &&
		    !LP_MASK(data->enabled_sensors))
			result = mpu6500_input_suspend_accel(data);
	}

	return result;
}
#endif

static int
mpu6500_input_activate_devices(struct mpu6500_input_data *data,
			       int sensors, bool enable)
{
	int result = 0;
	unsigned char reg = 0;
	struct motion_int_data *mot_data = &data->mot_data;
#ifdef CONFIG_SENSORS_MPU6500_LP
	__s8 orientation[9] = {0, };
#endif

	if (sensors & MPU6500_SENSOR_ACCEL)
		CHECK_RESULT(mpu6500_input_actiave_accel(data, enable));

	if (sensors & MPU6500_SENSOR_GYRO)
		CHECK_RESULT(mpu6500_input_activate_gyro(data, enable));

#ifdef CONFIG_SENSORS_MPU6500_LP
	if (IS_LP_ENABLED(sensors)) {
		if (!data->is_firmware_loaded && enable) {
			memcpy(orientation, position_map[data->position],
					sizeof(orientation));
			if (mpu6500_lp_init_dmp(data->client, orientation))
				mpu_err("firmware loading failed!!!\n");
			else
				data->is_firmware_loaded = 1;
		}
		result = mpu6500_input_activate_lp_func(data, sensors, enable);
	}

	if (data->enabled_sensors &
	    (MPU6500_SENSOR_ACCEL | MPU6500_SENSOR_GYRO))
#ifdef CONFIG_SENSORS_MPU6500_POLLING
		mpu6500_lp_set_interrupt_on_gesture_event(data->client, true);
#else
		mpu6500_lp_set_interrupt_on_gesture_event(data->client, false);
#endif
	else if (IS_LP_ENABLED(data->enabled_sensors))
		mpu6500_lp_set_interrupt_on_gesture_event(data->client, true);
#endif

	if (data->enabled_sensors) {
		CHECK_RESULT(mpu6500_set_delay(data));
	} else {
		CHECK_RESULT(mpu6500_input_set_irq(data, 0x0));

		CHECK_RESULT(mpu6500_i2c_read_reg(data->client,
						  MPUREG_PWR_MGMT_1, 1,
						  &reg));

		if (!(reg & BIT_SLEEP))
			CHECK_RESULT(mpu6500_i2c_write_single_reg
				     (data->client, MPUREG_PWR_MGMT_1,
				      reg | BIT_SLEEP));

		if (data->movement_recog_flag != REACTIVE_OFF &&
						!mot_data->is_set) {
			usleep_range(5000, 5100);
			mpu6500_input_set_motion_interrupt(data, true, false);
		}
	}

	return result;
}

static int
mpu6500_input_gyro_config(struct mpu6500_input_data *data)
{
	#define MPU6500_GYRO_CFG 	0x49
	#define MPU6500_REG_BANK_SEL 0x76
	#define MPU6500_CFG_SET_BIT	0x20

	int result = 0;
	u8 d = 0, cfg = 0;

	CHECK_RESULT(mpu6500_i2c_read_reg
		(data->client, MPU6500_REG_BANK_SEL, 1, &cfg));
	CHECK_RESULT(mpu6500_i2c_write_single_reg
		(data->client, MPU6500_REG_BANK_SEL,
			cfg | MPU6500_CFG_SET_BIT));
	CHECK_RESULT(mpu6500_i2c_read_reg
		(data->client, MPU6500_GYRO_CFG, 1, &d));
	d |= 1;
	CHECK_RESULT(mpu6500_i2c_write_single_reg
		(data->client, MPU6500_GYRO_CFG, d));
	CHECK_RESULT(mpu6500_i2c_write_single_reg
		(data->client, MPU6500_REG_BANK_SEL, cfg));

	return result;
}

static int mpu6500_input_initialize(struct mpu6500_input_data *data,
			 const struct mpu6500_input_cfg *cfg)
{
	int result;

	data->int_pin_cfg = BIT_INT_ANYRD_2CLEAR;
	data->current_delay = 200;
	data->enabled_sensors = 0;

	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_PWR_MGMT_1, BIT_H_RESET));
	msleep(30);

	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (data->client, MPUREG_INT_PIN_CFG,
		      data->int_pin_cfg | BIT_BYPASS_EN));

	CHECK_RESULT(mpu6500_input_gyro_config(data));

#ifdef CONFIG_SENSORS_MPU6500_LP
	data->stepcounter = 0;
	data->is_firmware_loaded = 0;
#endif

	return mpu6500_input_set_mode(data, MPU6500_MODE_SLEEP);
}

static int mpu6500_input_accel_open_calibration(struct mpu6500_input_data *data)
{
	struct file *cal_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(ACC_CAL_PATH,
		O_RDONLY, S_IRUGO | S_IWUSR | S_IWGRP);
	if (IS_ERR(cal_filp)) {
		mpu_err("Can't open calibration file\n");
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		goto done;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)&data->acc_cal,
			3 * sizeof(s16), &cal_filp->f_pos);
	if (err != 3 * sizeof(s16)) {
		mpu_err("Can't read the cal data from file\n");
		err = -EIO;
	}

	mpu_info("(%d,%d,%d)\n", data->acc_cal[0],
			data->acc_cal[1], data->acc_cal[2]);

	filp_close(cal_filp, current->files);
done:
	set_fs(old_fs);
	return err;
}

static int mpu6500_input_accel_do_calibrate(struct mpu6500_input_data *data,
			int enable)
{
	struct file *cal_filp;
	int sum[3] = { 0, };
	int err;
	int i;
	struct mpu6500_acc acc_xyz;
	mm_segment_t old_fs;

	old_fs = get_fs();

	if (!(data->enabled_sensors & MPU6500_SENSOR_ACCEL)) {
		mpu6500_input_resume_accel(data);
		usleep_range(10000, 11000);
	}

	for (i = 0; i < ACC_CAL_TIME; i++) {
		err = mpu6500_input_read_accel_raw_data(data, &acc_xyz);
		if (err < 0) {
			mpu_err("mpu6500_input_read_accel_raw_data() "\
				"failed in the %dth loop\n", i);
			goto done;
		}
		usleep_range(10000, 11000);
		sum[0] += acc_xyz.x/ACC_CAL_DIV;
		sum[1] += acc_xyz.y/ACC_CAL_DIV;
		sum[2] += acc_xyz.z/ACC_CAL_DIV;
	}

	if (!(data->enabled_sensors & MPU6500_SENSOR_ACCEL))
		mpu6500_input_suspend_accel(data);

	if (enable) {
		data->acc_cal[0] =
			(sum[0] / ACC_CAL_TIME) * ACC_CAL_DIV;
		data->acc_cal[1] =
			(sum[1] / ACC_CAL_TIME) * ACC_CAL_DIV;

		if (sum[2] >= 0)
			data->acc_cal[2] =
				((sum[2] / ACC_CAL_TIME) - ACC_IDEAL) *
				ACC_CAL_DIV;
		else
			data->acc_cal[2] =
				((sum[2] / ACC_CAL_TIME) + ACC_IDEAL) *
				ACC_CAL_DIV;
	} else {
		data->acc_cal[0] = 0;
		data->acc_cal[1] = 0;
		data->acc_cal[2] = 0;
	}

	mpu_info("cal data (%d,%d,%d)\n", data->acc_cal[0],
				data->acc_cal[1], data->acc_cal[2]);

	set_fs(KERNEL_DS);

	cal_filp = filp_open(ACC_CAL_PATH,
			O_CREAT | O_TRUNC | O_WRONLY,
			S_IRUGO | S_IWUSR | S_IWGRP);
	if (IS_ERR(cal_filp)) {
		mpu_err("Can't open calibration file\n");
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		goto done;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&data->acc_cal, 3 * sizeof(s16),
			&cal_filp->f_pos);
	if (err != 3 * sizeof(s16)) {
		mpu_err("Can't write the cal data to file\n");
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
done:
	set_fs(old_fs);
	return err;
}

static int mpu6500_input_gyro_open_calibration(struct mpu6500_input_data *data)
{
	struct file *cal_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(GYRO_CAL_PATH,
		O_RDONLY, S_IRUGO | S_IWUSR | S_IWGRP);
	if (IS_ERR(cal_filp)) {
		mpu_err("Can't open calibration file\n");
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		goto done;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)&data->gyro_bias, 3 * sizeof(int),
			&cal_filp->f_pos);
	if (err != 3 * sizeof(int)) {
		mpu_err("Can't read the cal data from file\n");
		err = -EIO;
	}

	mpu_info("(%d,%d,%d)\n", data->gyro_bias[0],
				data->gyro_bias[1], data->gyro_bias[2]);

	filp_close(cal_filp, current->files);
done:
	set_fs(old_fs);
	return err;
}

static int mpu6500_input_gyro_do_calibrate(struct mpu6500_input_data *data)
{
	struct file *cal_filp;
	int err;
	mm_segment_t old_fs;

	mpu_info("cal data (%d,%d,%d)\n", data->gyro_bias[0],
				data->gyro_bias[1], data->gyro_bias[2]);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(GYRO_CAL_PATH,
			O_CREAT | O_TRUNC | O_WRONLY,
			S_IRUGO | S_IWUSR | S_IWGRP);
	if (IS_ERR(cal_filp)) {
		mpu_err("Can't open calibration file\n");
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		goto done;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&data->gyro_bias, 3 * sizeof(int),
			&cal_filp->f_pos);
	if (err != 3 * sizeof(int)) {
		mpu_err("Can't write the cal data to file\n");
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
done:
	set_fs(old_fs);
	return err;
}

static int mpu6500_input_accel_enable(struct mpu6500_input_data *data,
						int enable)
{
	int err = 0;

	mpu6500_input_accel_open_calibration(data);

	mpu_info("enable = %d\n", enable);
	mutex_lock(&data->mutex);
#ifdef CONFIG_SENSORS_MPU6500_POLLING
	if (enable && !atomic_read(&data->accel_enable)) {
		err = mpu6500_input_activate_devices(data,
				MPU6500_SENSOR_ACCEL, true);
		schedule_delayed_work(&data->accel_work,
			msecs_to_jiffies(5));
	}
	if (!enable && atomic_read(&data->accel_enable)) {
		cancel_delayed_work_sync(&data->accel_work);
		err = mpu6500_input_activate_devices(data,
				MPU6500_SENSOR_ACCEL, false);
	}
#else
	err = mpu6500_input_activate_devices(data, MPU6500_SENSOR_ACCEL,
					   (enable == 1) ? true : false);
#endif
	atomic_set(&data->accel_enable, (int)enable);
	mutex_unlock(&data->mutex);

	return err;
}

static ssize_t
mpu6500_input_accel_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&data->accel_enable));
}

static ssize_t
mpu6500_input_accel_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	int value = 0;

	if (kstrtoint(buf, 10, &value))
		return -EINVAL;

	mpu6500_input_accel_enable(data, (int)value);

	return count;
}

static ssize_t
mpu6500_input_accel_delay_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&data->accel_delay)*1000000);

}

static ssize_t
mpu6500_input_accel_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	u64 value;

	if (kstrtoll(buf, 10, &value))
		return -EINVAL;

	mpu_info("old delay = %d[msec], new delay = %llu[nsec]\n",
			atomic_read(&data->accel_delay), value);

	mutex_lock(&data->mutex);
	atomic_set(&data->accel_delay, (int)value/1000000);
	mpu6500_set_delay(data);
	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t
mpu6500_input_gyro_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&data->gyro_enable));
}

static ssize_t
mpu6500_input_gyro_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	struct motion_int_data *mot_data = &data->mot_data;
	int value;

	if (kstrtoint(buf, 10, &value))
		return -EINVAL;

	mpu6500_input_gyro_open_calibration(data);
	mpu_info("enable = %d\n", value);

	mutex_lock(&data->mutex);
#ifdef CONFIG_SENSORS_MPU6500_POLLING
	if (value && !atomic_read(&data->gyro_enable)) {
		mpu6500_input_activate_devices(data,
			MPU6500_SENSOR_GYRO, true);
		schedule_delayed_work(&data->gyro_work,
			msecs_to_jiffies(5));
	}
	if (!value && atomic_read(&data->gyro_enable)) {
		cancel_delayed_work_sync(&data->gyro_work);
		mpu6500_input_activate_devices(data,
			MPU6500_SENSOR_GYRO, false);
	}
#else
	mpu6500_input_activate_devices(data, MPU6500_SENSOR_GYRO,
				       (value == 1) ? true : false);
#endif
	/* backup the MPUREG_PWR_MGMT_2 register*/
	if (mot_data->is_set)
		mot_data->pwr_mnt[1] = data->gyro_pwr_mgnt[1];

	atomic_set(&data->gyro_enable, value);
	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t
mpu6500_input_gyro_delay_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&data->gyro_delay)*1000000);
}

static ssize_t
mpu6500_input_gyro_delay_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	u64 value;

	if (kstrtoll(buf, 10, &value))
		return -EINVAL;

	mpu_info("old delay = %d[msec], new delay = %llu[nsec]\n",
			atomic_read(&data->gyro_delay), value);

	mutex_lock(&data->mutex);
	atomic_set(&data->gyro_delay, (int)value/1000000);
	mpu6500_set_delay(data);
	mutex_unlock(&data->mutex);

	return count;
}


#ifdef CONFIG_SENSORS_MPU6500_LP
static ssize_t mpu6500_input_lp_so_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	int value;

	if (kstrtoint(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&data->mutex);

	mpu_info("enable = %d, old value:%d\n", value,
				atomic_read(&data->lp_so));
	if (value && !atomic_read(&data->lp_so))
		mpu6500_input_activate_devices(data,
				MPU6500_SENSOR_LPSO, true);
	else if (!value && atomic_read(&data->lp_so))
		mpu6500_input_activate_devices(data,
				MPU6500_SENSOR_LPSO, false);
	atomic_set(&data->lp_so, (int)value);

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mpu6500_input_lp_so_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&data->lp_so));
}

static ssize_t mpu6500_input_lp_bc_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	int value;

	if (kstrtoint(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&data->mutex);

	mpu_info("enable = %d, old value:%d\n", value,
				atomic_read(&data->lp_bc));
	if (value && !atomic_read(&data->lp_bc))
		mpu6500_input_activate_devices(data,
				MPU6500_SENSOR_LPBC, true);
	else if (!value && atomic_read(&data->lp_bc))
		mpu6500_input_activate_devices(data,
				MPU6500_SENSOR_LPBC, false);
	atomic_set(&data->lp_bc, (int)value);

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mpu6500_input_lp_bc_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&data->lp_bc));
}

static ssize_t mpu6500_input_pedo_enable_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	int value;

	if (kstrtoint(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&data->mutex);

	mpu_info("enable = %d, old value:%d\n", value,
				atomic_read(&data->pedo));
	if (value && !atomic_read(&data->pedo))
		mpu6500_input_activate_devices(data,
				MPU6500_SENSOR_PEDO, true);
	else if (!value && atomic_read(&data->pedo))
		mpu6500_input_activate_devices(data,
				MPU6500_SENSOR_PEDO, false);
	atomic_set(&data->pedo, (int)value);

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mpu6500_input_pedo_enable_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&data->pedo));
}

static ssize_t mpu6500_input_pedo_cnt_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	int value;

	if (kstrtoint(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&data->mutex);

	mpu6500_lp_ped_set_septctrl(data->client, (unsigned int)value);
	data->stepcounter = (unsigned int)value;

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mpu6500_input_pedo_cnt_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);

	if (data->enabled_sensors & MPU6500_SENSOR_PEDO)
		data->stepcounter = mpu6500_lp_ped_get_septctrl(data->client);

	return sprintf(buf, "%d\n", data->stepcounter);
}

static DEVICE_ATTR(lpso_enable, 0666,
				mpu6500_input_lp_so_show,
				mpu6500_input_lp_so_store);
static DEVICE_ATTR(lpbc_enable, 0666,
				mpu6500_input_lp_bc_show,
				mpu6500_input_lp_bc_store);
static DEVICE_ATTR(pedo_enable, 0666,
				mpu6500_input_pedo_enable_show,
				mpu6500_input_pedo_enable_store);
static DEVICE_ATTR(pedo_count, 0666,
				mpu6500_input_pedo_cnt_show,
				mpu6500_input_pedo_cnt_store);
#endif

static struct device_attribute dev_attr_acc_enable =
__ATTR(enable, 0666,
		mpu6500_input_accel_enable_show,
		mpu6500_input_accel_enable_store);
static struct device_attribute dev_attr_acc_poll_delay =
__ATTR(poll_delay, 0666,
		mpu6500_input_accel_delay_show,
		mpu6500_input_accel_delay_store);

static struct device_attribute dev_attr_gyro_enable =
__ATTR(enable, 0666,
		mpu6500_input_gyro_enable_show,
		mpu6500_input_gyro_enable_store);
static struct device_attribute dev_attr_gyro_poll_delay =
__ATTR(poll_delay, 0666,
		mpu6500_input_gyro_delay_show,
		mpu6500_input_gyro_delay_store);

static struct attribute *accel_attributes[] = {
	&dev_attr_acc_enable.attr,
	&dev_attr_acc_poll_delay.attr,
#ifdef CONFIG_SENSORS_MPU6500_LP
	&dev_attr_lpso_enable.attr,
	&dev_attr_lpbc_enable.attr,
	&dev_attr_pedo_enable.attr,
	&dev_attr_pedo_count.attr,
#endif
	NULL
};

static struct attribute *gyro_attributes[] = {
	&dev_attr_gyro_enable.attr,
	&dev_attr_gyro_poll_delay.attr,
	NULL
};

static struct attribute_group accel_attribute_group = {
	.attrs = accel_attributes
};

static struct attribute_group gyro_attribute_group = {
	.attrs = gyro_attributes
};

static ssize_t mpu6500_input_accel_raw_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	struct mpu6500_acc acc_data;

	mpu6500_input_read_accel_data(data, &acc_data);

	return sprintf(buf, "%d, %d, %d\n", acc_data.x, acc_data.y, acc_data.z);
}

static ssize_t mpu6500_input_accel_calibration_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	int err;

	err = mpu6500_input_accel_open_calibration(data);
	if (err < 0)
		mpu_err("mpu6500_input_accel_open_calibration() failed\n");

	if (!data->acc_cal[0] && !data->acc_cal[1] && !data->acc_cal[2])
		err = -1;

	return sprintf(buf, "%d %d %d %d\n",
		err, data->acc_cal[0], data->acc_cal[1], data->acc_cal[2]);
}

static ssize_t mpu6500_input_accel_calibration_store(struct device *dev,
				struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	int err;
	int enable;

	if (kstrtoint(buf, 10, &enable))
		return -EINVAL;

	err = mpu6500_input_accel_do_calibrate(data, enable);
	if (err < 0)
		mpu_err("accel_do_calibrate() failed\n");

	return size;
}

static ssize_t mpu6500_input_accel_reactive_enable_show(struct device *dev,
					struct device_attribute
						*attr, char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);

	if (atomic_read(&data->interrupt_state))
		mpu_info("Read interrupt state\n");

	return sprintf(buf, "%d\n", atomic_read(&data->interrupt_state));
}

static ssize_t mpu6500_input_accel_reactive_enable_store(struct device *dev,
					struct device_attribute
						*attr, const char *buf,
							size_t count)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	int reactive_mode = -1, err;

	err = kstrtoint(buf, 10, &reactive_mode);
	if (err < 0) {
		mpu_err("kstrtoint failed\n");
		return  err;
	}
	if (reactive_mode >= REACTIVE_MAX) {
		mpu_err("invalid value %d\n", *buf);
		return -EINVAL;
	}

	switch (reactive_mode) {
	case REACTIVE_OFF:
		if (data->movement_recog_flag != REACTIVE_OFF) {
			mpu_info("reactive alert is off.\n");
			if (device_may_wakeup(&data->client->dev))
				disable_irq_wake(data->irq);
#ifndef CONFIG_SENSORS_MPU6500_LP
			disable_irq(data->irq);
#endif
			mutex_lock(&data->mutex);
			mpu6500_input_set_motion_interrupt(data,
				false, reactive_mode);
			data->movement_recog_flag = REACTIVE_OFF;
			mutex_unlock(&data->mutex);
		}
		break;
	case REACTIVE_ON:
	case REACTIVE_FACTORY:
		if (data->movement_recog_flag == REACTIVE_OFF) {
			mpu_info("reactive alert is on[mode:%d]\n",
							reactive_mode);

			mutex_lock(&data->mutex);
			mpu6500_input_set_motion_interrupt(data,
						true, reactive_mode);
			data->movement_recog_flag = reactive_mode;
			mutex_unlock(&data->mutex);
#ifndef CONFIG_SENSORS_MPU6500_LP
			enable_irq(data->irq);
#endif
			if (device_may_wakeup(&data->client->dev))
				enable_irq_wake(data->irq);
		}
		break;
	}

	return count;
}

static ssize_t mpu6500_input_accel_reactive_threshold_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d %d\n", data->dynamic_threshold[0],
					data->dynamic_threshold[1]);
}

static ssize_t mpu6500_input_accel_reactive_threshold_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	char *str, *threshold;
	int err = 0;

	str = (char *)buf;

	threshold = strsep(&str, " ");
	err = kstrtoint(threshold, 10, &data->dynamic_threshold[0]);
	if (err < 0)
		mpu_err("kstrtoint failed : %d\n", err);

	threshold = strsep(&str, " ");
	err = kstrtoint(threshold, 10, &data->dynamic_threshold[1]);
	if (err < 0)
		mpu_err("kstrtoint failed : %d\n", err);

	return count;
}

static ssize_t mpu6500_power_on(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;

	mpu_info("this_client = %d\n", (int)gb_mpu_data->client);
	count = sprintf(buf, "%d\n",
		(gb_mpu_data->client != NULL ? 1 : 0));

	return count;
}

static ssize_t mpu6500_get_temp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;
	short temperature = 0;
	unsigned char regs[2] = {0, };
	int result;

	CHECK_RESULT(mpu6500_i2c_read_reg
		(gb_mpu_data->client, MPUREG_TEMP_OUT_H, 2, regs));

	temperature = (short) (((regs[0]) << 8) | regs[1]);
	temperature = (((temperature + 521) / 340) + 35);

	mpu_info("read temperature = %d\n", temperature);

	count = sprintf(buf, "%d\n", temperature);

	return count;
}

static ssize_t accel_vendor_show(struct device *dev,
				struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%s\n", "INVENSENSE");
}

static ssize_t accel_name_show(struct device *dev,
				struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%s\n", "MPU6500");
}

static ssize_t
mpu6500_input_gyro_self_test_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct mpu6500_input_data *data = dev_get_drvdata(dev);
	int scaled_gyro_bias[3] = { 0 };
	int scaled_gyro_rms[3] = { 0 };
	int packet_count[3] = { 0 };
	int gyro_ratio[3] = { 0 }, accel_ratio[3] = { 0 };
	int result = 0;
	int hw_result = 0;

	mutex_lock(&data->mutex);
	hw_result = mpu6500_hw_self_check(data->client,
				gyro_ratio, accel_ratio, MPU6500_HWST_GYRO);

	result = mpu6500_selftest_run(data->client,
				      packet_count,
				      scaled_gyro_bias,
				      scaled_gyro_rms, data->gyro_bias);
	if (!result) {
		/* store calibration to file */
		mpu6500_input_gyro_do_calibrate(data);
		if (scaled_gyro_rms[0] == 0)
			scaled_gyro_rms[0] = 1;
		if (scaled_gyro_rms[1] == 0)
			scaled_gyro_rms[1] = 1;
		if (scaled_gyro_rms[2] == 0)
			scaled_gyro_rms[2] = 1;
	} else {
		data->gyro_bias[0] = 0;
		data->gyro_bias[1] = 0;
		data->gyro_bias[2] = 0;
		result = -1;
	}
	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d,"\
		       "%d.%03d,%d.%03d,%d.%03d,"
		       "%d.%03d,%d.%03d,%d.%03d,"
		       "%d.%01d,%d.%01d,%d.%01d,"
		       "%d,%d,%d\n",
		       result | hw_result,
		       (int)abs(scaled_gyro_bias[0] / 1000),
		       (int)abs(scaled_gyro_bias[0]) % 1000,
		       (int)abs(scaled_gyro_bias[1] / 1000),
		       (int)abs(scaled_gyro_bias[1]) % 1000,
		       (int)abs(scaled_gyro_bias[2] / 1000),
		       (int)abs(scaled_gyro_bias[2]) % 1000,
		       scaled_gyro_rms[0] / 1000,
		       (int)abs(scaled_gyro_rms[0]) % 1000,
		       scaled_gyro_rms[1] / 1000,
		       (int)abs(scaled_gyro_rms[1]) % 1000,
		       scaled_gyro_rms[2] / 1000,
		       (int)abs(scaled_gyro_rms[2]) % 1000,
		       (int)abs(gyro_ratio[0]/10),
		       (int)abs(gyro_ratio[0])%10,
		       (int)abs(gyro_ratio[1]/10),
		       (int)abs(gyro_ratio[1])%10,
		       (int)abs(gyro_ratio[2]/10),
		       (int)abs(gyro_ratio[2])%10,
		       packet_count[0], packet_count[1], packet_count[2]);
}

static DEVICE_ATTR(raw_data, 0666, mpu6500_input_accel_raw_data_show, NULL);
static DEVICE_ATTR(calibration, 0666,
		mpu6500_input_accel_calibration_show,
		mpu6500_input_accel_calibration_store);
static DEVICE_ATTR(reactive_alert, 0666,
		mpu6500_input_accel_reactive_enable_show,
		mpu6500_input_accel_reactive_enable_store);
static DEVICE_ATTR(reactive_threshold, 0664,
		mpu6500_input_accel_reactive_threshold_show,
		mpu6500_input_accel_reactive_threshold_store);
static DEVICE_ATTR(power_on, 0666, mpu6500_power_on, NULL);
static DEVICE_ATTR(temperature, 0666, mpu6500_get_temp, NULL);
static DEVICE_ATTR(selftest, 0666, mpu6500_input_gyro_self_test_show, NULL);
static DEVICE_ATTR(vendor, 0666, accel_vendor_show, NULL);
static DEVICE_ATTR(name, 0666, accel_name_show, NULL);

static struct device_attribute *accel_sensor_attrs[] = {
	&dev_attr_raw_data,
	&dev_attr_calibration,
	&dev_attr_reactive_alert,
	&dev_attr_reactive_threshold,
	&dev_attr_vendor,
	&dev_attr_name,
	NULL,
};

static struct device_attribute *gyro_sensor_attrs[] = {
	&dev_attr_power_on,
	&dev_attr_temperature,
	&dev_attr_selftest,
	&dev_attr_vendor,
	&dev_attr_name,
	NULL,
};

static int mpu6500_input_register_input_device
			(struct mpu6500_input_data *data)
{
	int error = 0;

	data->accel_input_dev = input_allocate_device();
	if (!data->accel_input_dev) {
		error = -ENOMEM;
		goto done;
	}

	data->accel_input_dev->name = "accelerometer_sensor";
	data->accel_input_dev->id.bustype = BUS_I2C;
	input_set_drvdata(data->accel_input_dev, data);

	input_set_capability(data->accel_input_dev, EV_REL, REL_X);
	input_set_capability(data->accel_input_dev, EV_REL, REL_Y);
	input_set_capability(data->accel_input_dev, EV_REL, REL_Z);
	input_set_capability(data->accel_input_dev, EV_REL, REL_MISC);
	input_set_capability(data->accel_input_dev, EV_REL, REL_HWHEEL);

	error = input_register_device(data->accel_input_dev);
	if (error) {
		input_free_device(data->accel_input_dev);
		goto done;
	}

	error = sysfs_create_group(&data->accel_input_dev->dev.kobj,
		&accel_attribute_group);
	if (error) {
		input_free_device(data->accel_input_dev);
		goto done;
	}

	data->gyro_input_dev = input_allocate_device();
	if (!data->gyro_input_dev) {
		error = -ENOMEM;
		goto done;
	}

	data->gyro_input_dev->name = "gyro_sensor";
	data->gyro_input_dev->id.bustype = BUS_I2C;
	input_set_drvdata(data->gyro_input_dev, data);

	input_set_capability(data->gyro_input_dev, EV_REL, REL_RX);
	input_set_capability(data->gyro_input_dev, EV_REL, REL_RY);
	input_set_capability(data->gyro_input_dev, EV_REL, REL_RZ);

	error = input_register_device(data->gyro_input_dev);
	if (error) {
		input_free_device(data->gyro_input_dev);
		goto done;
	}

	error = sysfs_create_group(&data->gyro_input_dev->dev.kobj,
		&gyro_attribute_group);
	if (error) {
		input_free_device(data->gyro_input_dev);
		goto done;
	}

	wake_lock_init(&data->reactive_wake_lock, WAKE_LOCK_SUSPEND,
		"reactive_wake_lock");
	mutex_init(&data->mutex);

	atomic_set(&data->accel_enable, 0);
	atomic_set(&data->accel_delay, 200);
	atomic_set(&data->gyro_enable, 0);
	atomic_set(&data->gyro_delay, 200);
#ifdef CONFIG_SENSORS_MPU6500_LP
	atomic_set(&data->lp_so, 0);
	atomic_set(&data->lp_bc, 0);
	atomic_set(&data->pedo, 0);
#endif
	atomic_set(&data->interrupt_state, 0);
	data->movement_recog_flag = REACTIVE_OFF;
	data->dynamic_threshold[0] = DYNAMIC_THRESHOLD;
	data->dynamic_threshold[1] = DYNAMIC_THRESHOLD;

done:
	return error;
}

#ifdef CONFIG_SENSORS_MPU6500_POLLING
static void mpu6500_work_func_acc(struct work_struct *work)
{
	struct mpu6500_input_data *data =
		container_of((struct delayed_work *)work,
			struct mpu6500_input_data, accel_work);

	mpu6500_input_report_accel_xyz(data);

	if (atomic_read(&data->accel_delay) < 60) {
		usleep_range(atomic_read(&data->accel_delay) * 1000,
			atomic_read(&data->accel_delay) * 1100);
		schedule_delayed_work(&data->accel_work, 0);
	} else {
		schedule_delayed_work(&data->accel_work,
			msecs_to_jiffies(
			atomic_read(&data->accel_delay)));
	}
}

static void mpu6500_work_func_gyro(struct work_struct *work)
{
	struct mpu6500_input_data *data =
		container_of((struct delayed_work *)work,
			struct mpu6500_input_data, gyro_work);

	mpu6500_input_report_gyro_xyz(data);

	if (atomic_read(&data->gyro_delay) < 60) {
		usleep_range(atomic_read(&data->gyro_delay) * 1000,
			atomic_read(&data->gyro_delay) * 1100);
		schedule_delayed_work(&data->gyro_work, 0);
	} else {
		schedule_delayed_work(&data->gyro_work,
			msecs_to_jiffies(
			atomic_read(&data->gyro_delay)));
	}
}
#endif

static int mpu6500_input_setup_pin(struct mpu6500_input_data *data)
{
	int ret;

	ret = gpio_request(data->acc_int, "mpu6500_accel_int");
	if (ret < 0) {
		mpu_err("gpio %d request failed (%d)\n", data->acc_int, ret);
		goto exit;
	}

	ret = gpio_direction_input(data->acc_int);
	if (ret < 0) {
		mpu_err("failed to set gpio %d as input (%d)\n", data->acc_int, ret);
		goto exit_int_gpio;
	}

	data->irq = gpio_to_irq(data->acc_int);

	if (ret < 0) {
		pr_err("request_irq(%d) fail for gpio %d (%d)\n", data->irq, data->acc_int, ret);
		goto exit_int_gpio;
	}

	ret = request_threaded_irq(data->irq, NULL, mpu6500_input_irq_thread,
					 IRQF_TRIGGER_RISING | IRQF_ONESHOT, "mpu6500_int", data);
	if (ret < 0) {
		mpu_err("irq request failed %d, error %d\n", data->irq , ret);
		goto exit_int_gpio;
	}

#ifndef CONFIG_SENSORS_MPU6500_LP
	disable_irq(data->irq);
#endif

exit_int_gpio:
	gpio_free(data->acc_int);
exit:
	return ret;

}

#ifdef CONFIG_OF
static int mpu6500_input_parse_dt(struct device *dev,
			struct  mpu6500_input_data *data)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;

	data->acc_int = of_get_named_gpio_flags(np,
		"mpu6500,acc_int-gpio", 0, &flags);
	if (data->acc_int < 0) {
		mpu_err("of_node error %d (acc_int-gpio)\n", data->acc_int);
		return -ENODEV;
	}

	if (of_property_read_u32(np, "mpu6500,position", &data->position))
		data->position = 2;
	mpu_info("acc chip position is %d\n", data->position);

	return 0;
}
#else
static int mpu6500_input_parse_dt(struct device *dev,
			struct  mpu6500_input_data *data)
{
	return -ENODEV;
}
#endif

static int mpu6500_input_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	const struct mpu6500_input_cfg *cfg;
	struct mpu6500_input_data *data;
	int error = 0;
	unsigned char whoami = 0;

	mpu_info("is start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	data = kzalloc(sizeof(struct mpu6500_input_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (client->dev.of_node) {
		error = mpu6500_input_parse_dt(&client->dev, data);
		if (error) {
			mpu_err("Fail to parse DT\n");
			goto parse_dt_failed;
		}
	} else {
		mpu_err("of_node error\n");
		goto parse_dt_failed;
	}

	gb_mpu_data = data;
	data->client = client;
	i2c_set_clientdata(client, data);

	error = mpu6500_i2c_read_reg(client, MPUREG_WHOAMI, 1, &whoami);
	if (error < 0) {
		error = -ENOMEM;
		mpu_err("failed : threre is no such device.\n");
		goto chip_auth_failed;
	} else {
		if (whoami != 0x70) {
			error = -ENXIO;
			mpu_info("Chip id is 0x%x, MPU-6500M found\n", whoami);
			goto chip_auth_failed;
		}
	}

	error = sensors_register(data->accel_sensor_device,
		data, accel_sensor_attrs,
			"accelerometer_sensor");
	if (error) {
		mpu_err("cound not regist accelerometer sensor device(%d)\n",
			error);
		goto acc_sensor_register_failed;
	}

	error = sensors_register(data->gyro_sensor_device,
		data, gyro_sensor_attrs,
			"gyro_sensor");
	if (error) {
		mpu_err("cound not register gyro sensor device(%d).\n", error);
		goto gyro_sensor_register_failed;
	}

#ifdef CONFIG_SENSORS_MPU6500_POLLING
		INIT_DELAYED_WORK(&data->accel_work, mpu6500_work_func_acc);
		INIT_DELAYED_WORK(&data->gyro_work, mpu6500_work_func_gyro);
#endif

	error = mpu6500_input_initialize(data, cfg);
	if (error) {
		mpu_err("sensor ininitilzisation is failed\n");
		goto sensor_initialize_failed;
	}

	error = mpu6500_input_register_input_device(data);
	if (error) {
		mpu_err("input regisit is failed\n");
		goto input_register_device_failed;
	}

	error = mpu6500_input_setup_pin(data);
	if (error) {
		mpu_err("setup pin is failed\n");
		goto pin_setup_failed;
	}

	device_init_wakeup(&data->client->dev, 1);


	mpu_info("mpu6500_input_probe success\n");

	return 0;

pin_setup_failed:
	input_unregister_device(data->gyro_input_dev);
	input_unregister_device(data->accel_input_dev);
input_register_device_failed:
	wake_lock_destroy(&data->reactive_wake_lock);
sensor_initialize_failed:
#ifdef CONFIG_SENSORS_MPU6500_POLLING
	cancel_delayed_work(&data->gyro_work);
	cancel_delayed_work(&data->accel_work);
#endif
	sensors_unregister(data->gyro_sensor_device, gyro_sensor_attrs);
gyro_sensor_register_failed:
	sensors_unregister(data->accel_sensor_device, accel_sensor_attrs);
acc_sensor_register_failed:
chip_auth_failed:
parse_dt_failed:
	kfree(data);
	return error;
}

static int mpu6500_input_remove(struct i2c_client *client)
{
	struct mpu6500_input_data *data = i2c_get_clientdata(client);

	free_irq(data->irq, data);

	input_unregister_device(data->gyro_input_dev);
	input_unregister_device(data->accel_input_dev);
#ifdef CONFIG_SENSORS_MPU6500_POLLING
	cancel_delayed_work(&data->gyro_work);
	cancel_delayed_work(&data->accel_work);
#endif
	sensors_unregister(data->gyro_sensor_device, gyro_sensor_attrs);
	sensors_unregister(data->accel_sensor_device, accel_sensor_attrs);

	wake_lock_destroy(&data->reactive_wake_lock);
	kfree(data);

	return 0;
}

static int mpu6500_input_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mpu6500_input_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_SENSORS_MPU6500_POLLING
	if (atomic_read(&data->accel_enable))
		cancel_delayed_work_sync(&data->accel_work);
	if (atomic_read(&data->gyro_enable))
		cancel_delayed_work_sync(&data->gyro_work);
#endif

	if (data->movement_recog_flag == REACTIVE_OFF) {
#ifndef CONFIG_SENSORS_MPU6500_POLLING
		disable_irq_wake(data->irq);
		disable_irq(data->irq);
#endif
		if (atomic_read(&data->accel_enable) ||
			atomic_read(&data->gyro_enable))
			mpu6500_input_set_mode(data, MPU6500_MODE_SLEEP);
	}

	return 0;
}

static int mpu6500_input_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mpu6500_input_data *data = i2c_get_clientdata(client);

	if (data->movement_recog_flag == REACTIVE_OFF) {
#ifndef CONFIG_SENSORS_MPU6500_POLLING
		enable_irq(data->irq);
		enable_irq_wake(data->irq);
#endif
		if (atomic_read(&data->accel_enable) ||
			atomic_read(&data->gyro_enable))
			mpu6500_input_set_mode(data, MPU6500_MODE_NORMAL);
	}

#ifdef CONFIG_SENSORS_MPU6500_POLLING
	if (atomic_read(&data->accel_enable))
		schedule_delayed_work(&data->accel_work, 0);
	if (atomic_read(&data->gyro_enable))
		schedule_delayed_work(&data->gyro_work, 0);
#endif
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id mpu6500_match_table[] = {
	{ .compatible = "invensense,mpu6500",},
	{},
};
#else
#define mpu6500_match_table NULL
#endif

static const struct i2c_device_id mpu6500_input_id[] = {
	{MPU6500_INPUT_DRIVER, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mpu6500_input_id);

static const struct dev_pm_ops mpu6500_dev_pm_ops = {
	.suspend = mpu6500_input_suspend,
	.resume = mpu6500_input_resume,
};

static struct i2c_driver mpu6500_input_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = MPU6500_INPUT_DRIVER,
		   .of_match_table = mpu6500_match_table,
		   .pm = &mpu6500_dev_pm_ops,
		   },
	.class = I2C_CLASS_HWMON,
	.id_table = mpu6500_input_id,
	.probe = mpu6500_input_probe,
	.remove = mpu6500_input_remove,
};

static int __init mpu6500_init(void)
{
	int result = i2c_add_driver(&mpu6500_input_driver);

	mpu_info("is done\n");
	return result;
}

static void __exit mpu6500_exit(void)
{
	mpu_info("is called\n");

	i2c_del_driver(&mpu6500_input_driver);
}

MODULE_AUTHOR("Tae-Soo Kim <tskim@invensense.com>");
MODULE_DESCRIPTION("MPU6500 driver");
MODULE_LICENSE("GPL");

module_init(mpu6500_init);
module_exit(mpu6500_exit);
