#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/firmware.h>

#include <linux/sensor/mpu6500_input.h>
#include "mpu6500_lp.h"
#include "mpu6500_dmpkey.h"

#define CFG_FIFO_ON_EVENT       (2649)
#define CFG_ORIENT_IRQ_1        (2459)
#define CFG_DISPLAY_ORIENT_INT  (1634)
#define CFG_SCREEN_BRIGHTNESS_INT_1 (1345)
#define CFG_SCREEN_BRIGHTNESS_INT_2 (1390)
#define D_0_22                  (22+512)
#define D_1_74                  (256 + 74)
#define D_1_232                 (256 + 232)
#define D_1_250                 (256 + 250)
#define D_PEDSTD_STEPCTR        (768 + 0x60)
#define D_PEDSTD_TIMECTR        (964)

#define CFG_6                   (2711)
#define CFG_15                  (2686)
#define CFG_27                  (2700)

#define FCFG_1                  (1062)
#define FCFG_2                  (1066)

#define FCFG_3                  (1110)
#define FCFG_7                  (1076)

#define INV_FIRMWARE_FILE "inv_mpu6500_dmp.bin"
#define INV_FIRMWARE_HEADER "DMP"
#define INV_FIRMWARE_HEADER_SIZE (7+4) /* Header+size */

#define LOG_RESULT_LOCATION(x) {\
	printk(KERN_ERR "%s:%s:%d result=%d\n",\
	__FILE__, __func__, __LINE__, x);\
}

#define CHECK_RESULT(x) {\
	result = x;\
	if (unlikely(result))\
		LOG_RESULT_LOCATION(result);\
}

static int mpu6500_lp_load_firmware(struct i2c_client *i2c_client)
{
	const struct firmware *fw = NULL;
	int bank, write_size;
	unsigned int uPos = 0;
	int result = 0;
	unsigned short memaddr;

	result = request_firmware(&fw, INV_FIRMWARE_FILE, &i2c_client->dev);
	if (result) {
		mpu_err("Unable to open firmware %s\n", INV_FIRMWARE_FILE);
		goto request_fw_failed;
	}

	if (memcmp(INV_FIRMWARE_HEADER,
			fw->data, strlen(INV_FIRMWARE_HEADER))) {
		mpu_err("firmware header mismatched\n");
		result = -1;
		goto identify_failed;
	}

	uPos += INV_FIRMWARE_HEADER_SIZE;
	/* Write and verify memory */
	for (bank = 0; fw->size > uPos; bank++) {
		if (fw->size > MPU_MEM_BANK_SIZE)
			write_size = MPU_MEM_BANK_SIZE;
		else
			write_size = fw->size;

		memaddr = ((bank << 8) | 0x00);

		result = mpu6500_i2c_memory_write(i2c_client,
					memaddr, write_size, fw->data+uPos);
		if (result) {
			mpu_err("firmware load failed(%d)\n", result);
			goto identify_failed;
		}

		uPos += write_size;
	}
	mpu_info("DMP firmware loading is done\n");

identify_failed:
	release_firmware(fw);
request_fw_failed:

	return result;
}

static int mpu6500_lp_verify_firmware(struct i2c_client *i2c_client)
{
	return 0;
}

int mpu6500_lp_set_interrupt_on_gesture_event(struct i2c_client *i2c_client,
					      bool on)
{
	unsigned char result;
	const unsigned char regs_on[] = { DINADA, DINADA, DINAB1, DINAB9,
		DINAF3, DINA8B, DINAA3, DINA91,
		DINAB6, DINADA, DINAB4, DINADA
	};
	const unsigned char regs_off[] = { 0xd8, 0xd8, 0xb1, 0xb9, 0xf3, 0x8b,
		0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9
	};
	/*For some reason DINAC4 is defined as 0xb8,
	   but DINBC4 is not defined. */
	const unsigned char regs_end[] = { DINAFE, DINAF2, DINAAB, 0xc4,
		DINAAA, DINAF1, DINADF, DINADF
	};
	if (on)
		/*Sets the DMP to send an interrupt and put a FIFO packet
		   in the FIFO if and only if a tap/orientation event
		   just occurred */
		result =
		    mpu6500_i2c_memory_write(i2c_client, CFG_FIFO_ON_EVENT,
					     ARRAY_SIZE(regs_on), regs_on);
	else
		/*Sets the DMP to send an interrupt and put a FIFO packet
		   in the FIFO at the rate specified by the FIFO div.
		   see inv_set_fifo_div in hw_setup.c to set the FIFO div. */
		result =
		    mpu6500_i2c_memory_write(i2c_client, CFG_FIFO_ON_EVENT,
					     ARRAY_SIZE(regs_off), regs_off);
	if (result)
		return result;

	result =
	    mpu6500_i2c_memory_write(i2c_client, CFG_6, ARRAY_SIZE(regs_end),
				     regs_end);
	return result;
}

static int mpu6500_lp_send_sensor_data(struct i2c_client *i2c_client,
				       unsigned short elements)
{
	int result;
	unsigned char regs[] = { DINAA0 + 3, DINAA0 + 3, DINAA0 + 3,
		DINAA0 + 3, DINAA0 + 3, DINAA0 + 3,
		DINAA0 + 3, DINAA0 + 3, DINAA0 + 3,
		DINAA0 + 3
	};

	if (elements & MPU6500_ELEMENT_1)
		regs[0] = DINACA;
	if (elements & MPU6500_ELEMENT_2)
		regs[4] = DINBC4;
	if (elements & MPU6500_ELEMENT_3)
		regs[5] = DINACC;
	if (elements & MPU6500_ELEMENT_4)
		regs[6] = DINBC6;
	if ((elements & MPU6500_ELEMENT_5) || (elements & MPU6500_ELEMENT_6) ||
	    (elements & MPU6500_ELEMENT_7)) {
		regs[1] = DINBC0;
		regs[2] = DINAC8;
		regs[3] = DINBC2;
	}
	result =
	    mpu6500_i2c_memory_write(i2c_client, CFG_15, ARRAY_SIZE(regs),
				     regs);
	return result;
}

static int mpu6500_lp_send_interrupt_word(struct i2c_client *i2c_client)
{
	const unsigned char regs[] = { DINA20 };
	unsigned char result;

	result =
	    mpu6500_i2c_memory_write(i2c_client, CFG_27, ARRAY_SIZE(regs),
				     regs);
	return result;
}

static int mpu6500_lp_set_orient_interrupt_dmp(struct i2c_client *i2c_client,
					       bool on)
{
	/*Turn on the display orientation interrupt in the DMP */
	int result;
	unsigned char regs[] = { 0xd8 };

	if (on)
		regs[0] = 0xd9;
	result =
	    mpu6500_i2c_memory_write(i2c_client, CFG_DISPLAY_ORIENT_INT, 1,
				     regs);
	return result;
}

static int mpu6500_lp_set_fifo_div(struct i2c_client *i2c_client,
				   unsigned short fifo_rate)
{
	unsigned char regs[2];
	int result = 0;
	/*For some reason DINAC4 is defined as 0xb8, but DINBC4 is not */
	const unsigned char regs_end[12] = { DINAFE, DINAF2, DINAAB, 0xc4,
		DINAAA, DINAF1, DINADF, DINADF,
		0xbb, 0xaf, DINADF, DINADF
	};

	regs[0] = (unsigned char)((fifo_rate >> 8) & 0xff);
	regs[1] = (unsigned char)(fifo_rate & 0xff);
	result =
	    mpu6500_i2c_memory_write(i2c_client, D_0_22, ARRAY_SIZE(regs),
				     regs);
	if (result)
		return result;

	/*Modify the FIFO handler to reset the tap/orient interrupt flags */
	/* each time the FIFO handler runs */
	result =
	    mpu6500_i2c_memory_write(i2c_client, CFG_6, ARRAY_SIZE(regs_end),
				     regs_end);

	return result;
}

static unsigned short mpu6500_lp_row_to_scale(const signed char *row)
{
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;

	return b;
}

static unsigned short mpu6500_lp_orientation_to_scalar(__s8 orientation[9])
{
	unsigned short scalar;

	scalar = mpu6500_lp_row_to_scale(orientation);
	scalar |= mpu6500_lp_row_to_scale(orientation + 3) << 3;
	scalar |= mpu6500_lp_row_to_scale(orientation + 6) << 6;

	return scalar;
}

static int mpu6500_lp_gyro_dmp_cal(struct i2c_client *i2c_client,
				   __s8 orientation[9])
{
	int inv_gyro_orient;
	unsigned char regs[3];
	int result;

	unsigned char tmpD = DINA4C;
	unsigned char tmpE = DINACD;
	unsigned char tmpF = DINA6C;

	inv_gyro_orient = mpu6500_lp_orientation_to_scalar(orientation);

	if ((inv_gyro_orient & 3) == 0)
		regs[0] = tmpD;
	else if ((inv_gyro_orient & 3) == 1)
		regs[0] = tmpE;
	else if ((inv_gyro_orient & 3) == 2)
		regs[0] = tmpF;
	if ((inv_gyro_orient & 0x18) == 0)
		regs[1] = tmpD;
	else if ((inv_gyro_orient & 0x18) == 0x8)
		regs[1] = tmpE;
	else if ((inv_gyro_orient & 0x18) == 0x10)
		regs[1] = tmpF;
	if ((inv_gyro_orient & 0xc0) == 0)
		regs[2] = tmpD;
	else if ((inv_gyro_orient & 0xc0) == 0x40)
		regs[2] = tmpE;
	else if ((inv_gyro_orient & 0xc0) == 0x80)
		regs[2] = tmpF;

	result = mpu6500_i2c_memory_write(i2c_client, FCFG_1, 3, regs);
	if (result)
		return result;

	if (inv_gyro_orient & 4)
		regs[0] = DINA36 | 1;
	else
		regs[0] = DINA36;
	if (inv_gyro_orient & 0x20)
		regs[1] = DINA56 | 1;
	else
		regs[1] = DINA56;
	if (inv_gyro_orient & 0x100)
		regs[2] = DINA76 | 1;
	else
		regs[2] = DINA76;

	result =
	    mpu6500_i2c_memory_write(i2c_client, FCFG_3, ARRAY_SIZE(regs),
				     regs);

	return result;

}

static int mpu6500_lp_accel_dmp_cal(struct i2c_client *i2c_client,
				    __s8 orientation[9])
{
	int inv_accel_orient;
	int result;
	unsigned char regs[3];
	const unsigned char tmp[3] = { DINA0C, DINAC9, DINA2C };
	inv_accel_orient = mpu6500_lp_orientation_to_scalar(orientation);

	regs[0] = tmp[inv_accel_orient & 3];
	regs[1] = tmp[(inv_accel_orient >> 3) & 3];
	regs[2] = tmp[(inv_accel_orient >> 6) & 3];
	result = mpu6500_i2c_memory_write(i2c_client, FCFG_2, 3, regs);
	if (result)
		return result;

	regs[0] = DINA26;
	regs[1] = DINA46;
	regs[2] = DINA66;
	if (inv_accel_orient & 4)
		regs[0] |= 1;
	if (inv_accel_orient & 0x20)
		regs[1] |= 1;
	if (inv_accel_orient & 0x100)
		regs[2] |= 1;
	result =
	    mpu6500_i2c_memory_write(i2c_client, FCFG_7, ARRAY_SIZE(regs),
				     regs);

	return result;

}

int mpu6500_lp_active_bc_interrupt(struct i2c_client *i2c_client, bool on)
{
	int result;
	u8 regs[1] = { 0xde };

	if (on)
		regs[0] = 0x08;
	result =
	    mpu6500_i2c_memory_write(i2c_client, CFG_SCREEN_BRIGHTNESS_INT_1, 1,
				     regs);
	result =
	    mpu6500_i2c_memory_write(i2c_client, CFG_SCREEN_BRIGHTNESS_INT_2, 1,
				     regs);

	return result;
}

int mpu6500_lp_init_dmp(struct i2c_client *i2c_client, __s8 orientation[9])
{
	int result = 0;
	unsigned char mgmt1 = 0;

	mpu6500_i2c_read_reg(i2c_client, MPUREG_PWR_MGMT_1, 1, &mgmt1);

	if (mgmt1 & BIT_SLEEP) {
		CHECK_RESULT(mpu6500_i2c_write_single_reg
			     (i2c_client, MPUREG_PWR_MGMT_1, 0x0));
	}

	CHECK_RESULT(mpu6500_lp_load_firmware(i2c_client));
	CHECK_RESULT(mpu6500_lp_verify_firmware(i2c_client));

	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (i2c_client, MPUREG_DMP_CFG_1, DMP_START_ADDR >> 8));
	CHECK_RESULT(mpu6500_i2c_write_single_reg
		     (i2c_client, MPUREG_DMP_CFG_2, DMP_START_ADDR & 0xff));

	CHECK_RESULT(mpu6500_lp_send_sensor_data
		     (i2c_client, MPU6500_GYRO_ACC_MASK));
	CHECK_RESULT(mpu6500_lp_send_interrupt_word(i2c_client));

	CHECK_RESULT(mpu6500_lp_set_orient_interrupt_dmp(i2c_client, true));

	CHECK_RESULT(mpu6500_lp_accel_dmp_cal(i2c_client, orientation));
	CHECK_RESULT(mpu6500_lp_gyro_dmp_cal(i2c_client, orientation));

	CHECK_RESULT(mpu6500_i2c_write_single_reg
				 (i2c_client, MPUREG_PWR_MGMT_1, mgmt1));

	return result;
}

int mpu6500_lp_activate_lpfunc(struct i2c_client *i2c_client,
			       bool enable, int current_delay, bool enable_fifo)
{
	int result = 0;
	unsigned char reg = 0;

	if (enable) {
		if (!result)
			result =
			    mpu6500_lp_set_delay(i2c_client, current_delay);

		if (!result)
			result =
			    mpu6500_i2c_write_single_reg(i2c_client,
							 MPUREG_USER_CTRL,
							 BIT_DMP_RST |
							 BIT_FIFO_RST);

		if (!result) {
			reg = BIT_DMP_EN;
			if (enable_fifo)
				reg |= BIT_FIFO_EN;

			result =
			    mpu6500_i2c_write_single_reg(i2c_client,
							 MPUREG_USER_CTRL,
							 reg);
		}

	} else {
		if (!result)
			result =
			    mpu6500_i2c_write_single_reg(i2c_client,
							 MPUREG_USER_CTRL, 0x0);
	}

	return result;
}

int mpu6500_lp_set_delay(struct i2c_client *i2c_client, int current_delay)
{
	int result = 0;
	unsigned short divider = 0;
	unsigned char sample_divider = 0;
	unsigned short fifo_divider = 0;

	divider = (short)(current_delay) - 1;

	if (divider > 4) {
		sample_divider = 4;
		fifo_divider = (unsigned short)((current_delay) / 5) - 1;
	} else {
		sample_divider = divider;
		fifo_divider = 0;
	}

	result = mpu6500_lp_set_fifo_div(i2c_client, fifo_divider);

	return result;
}

int mpu6500_lp_ped_set_septctrl(struct i2c_client *i2c_client,
			unsigned int counter)
{
	int result = 0;
	unsigned int data = counter;
	cpu_to_be32s(&data);
	result =
	    mpu6500_i2c_memory_write(i2c_client, D_PEDSTD_STEPCTR, sizeof(data),
				     (u8 *)&data);
	return result;
}

int mpu6500_lp_ped_set_timectrl(struct i2c_client *i2c_client,
			unsigned int time)
{
	int result = 0;
	unsigned int data = time;
	cpu_to_be32s(&data);
	result =
	    mpu6500_i2c_memory_write(i2c_client, D_PEDSTD_TIMECTR, sizeof(data),
				     (u8 *)&data);
	return result;
}

unsigned int mpu6500_lp_ped_get_septctrl(struct i2c_client *i2c_client)
{
	unsigned int data = 0;
	int result = 0;
	char regs[4] = {0, };
	result =
	    mpu6500_i2c_memory_read(i2c_client, D_PEDSTD_STEPCTR, 4,
				     (u8 *)regs);
	data = be32_to_cpup((int *)regs);
	return data;
}

unsigned int mpu6500_lp_ped_get_timectrl(struct i2c_client *i2c_client)
{
	unsigned int data = 0;
	int result = 0;
	char regs[4] = {0, };
	result =
	    mpu6500_i2c_memory_read(i2c_client, D_PEDSTD_TIMECTR, 4,
				     (u8 *)regs);
	data = be32_to_cpup((int *)regs);
	return data;
}

