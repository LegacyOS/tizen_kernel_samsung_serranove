/*
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * Author: Haibo Jeff Zhong <hzhong@qualcomm.com>
 *
 * All source code in this file is licensed under the following license
 * except where indicated.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 *
 */


#ifndef SEC_M5MO_H
#define SEC_M5MO_H
#include "msm_sensor.h"
#include "msm_sensor_driver.h"

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) printk("[m5mols] %s : %d : " fmt "\n",  __func__, __LINE__, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif
#define VENDOR "FUJITSU"
#define NAME "M5MOLS"

#define CHECK_ERR(x)   \
		do {\
		if ((x) < 0) { \
			printk("i2c falied, err %d\n", x); \
			x = -1; \
			return x; \
				}	\
		} while (0)
#define M5MO_I2C_VERIFY		100
#define M5MO_I2C_VERIFY_RETRY		200
#define M5MO_I2C_RETRY		5

#define SDCARD_FW
#ifdef SDCARD_FW
#define M5MO_FW_PATH_SDCARD "/opt/usr/media/RS_M5LS.bin"
#endif
#define M5MO_FW_NAME				"RS_M5LS.bin"
#define M5MO_FW_DUMP_PATH		"/opt/usr/media/RS_M5LS_dump.bin"
#define M5MO_FLASH_BASE_ADDR	0x10000000
#define M5MO_INT_RAM_BASE_ADDR	0x68000000
#define M5MO_VERSION_INFO_SIZE	21
/*0x16fee0 offset in binary file */
#define M5MO_VERSION_INFO_ADDR	0xFFF00

/* Category */
#define M5MO_CATEGORY_SYS	0x00
#define M5MO_CATEGORY_PARM	0x01
#define M5MO_CATEGORY_MON	0x02
#define M5MO_CATEGORY_AE	0x03
#define M5MO_CATEGORY_WB	0x06
#define M5MO_CATEGORY_EXIF	0x07
#define M5MO_CATEGORY_FD	0x09
#define M5MO_CATEGORY_LENS	0x0A
#define M5MO_CATEGORY_CAPPARM	0x0B
#define M5MO_CATEGORY_CAPCTRL	0x0C
#define M5MO_CATEGORY_TEST	0x0D
#define M5MO_CATEGORY_ADJST	0x0E
#define M5MO_CATEGORY_FLASH	0x0F	/* F/W update */

/* M5MO_CATEGORY_SYS: 0x00 */
#define M5MO_SYS_PJT_CODE	0x01
#define M5MO_SYS_VER_FW		0x02
#define M5MO_SYS_VER_HW		0x04
#define M5MO_SYS_VER_PARAM	0x06
#define M5MO_SYS_VER_AWB	0x08
#define M5MO_SYS_USER_VER	0x0A
#define M5MO_SYS_MODE		0x0B
#define M5MO_SYS_ESD_INT	0x0E
#define M5MO_SYS_INT_FACTOR	0x10
#define M5MO_SYS_INT_EN		0x11
#define M5MO_SYS_ROOT_EN	0x12

/* M5MO_CATEGORY_PARAM: 0x01 */
#define M5MO_PARM_OUT_SEL	0x00
#define M5MO_PARM_MON_SIZE	0x01
#define M5MO_PARM_EFFECT	0x0B
#define M5MO_PARM_FLEX_FPS	0x31
#define M5MO_PARM_HDMOVIE	0x32
#define M5MO_PARM_HDR_MON	0x39
#define M5MO_PARM_HDR_MON_OFFSET_EV	0x3A
#define M5MO_PARM_MIPI_DATA_TYPE	0x3C

/* M5MO_CATEGORY_MON: 0x02 */
#define M5MO_MON_ZOOM		0x01
#define M5MO_MON_IR_MODE		0x04
#define M5MO_MON_MON_REVERSE	0x05
#define M5MO_MON_MON_MIRROR	0x06
#define M5MO_MON_SHOT_REVERSE	0x07
#define M5MO_MON_SHOT_MIRROR	0x08
#define M5MO_MON_CFIXB		0x09
#define M5MO_MON_CFIXR		0x0A
#define M5MO_MON_COLOR_EFFECT	0x0B
#define M5MO_MON_CHROMA_LVL	0x0F
#define M5MO_MON_EDGE_LVL	0x11
#define M5MO_MON_TONE_CTRL	0x25

/* M5MO_CATEGORY_AE: 0x03 */
#define M5MO_AE_LOCK		0x00
#define M5MO_AE_MODE		0x01
#define M5MO_AE_ISOSEL		0x05
#define M5MO_AE_FLICKER		0x06
#define M5MO_AE_EP_MODE_MON	0x0A
#define M5MO_AE_EP_MODE_CAP	0x0B
#define M5MO_AE_ONESHOT_MAX_EXP	0x36
#define M5MO_AE_INDEX		0x38

/* M5MO_CATEGORY_WB: 0x06 */
#define M5MO_AWB_LOCK		0x00
#define M5MO_WB_AWB_MODE	0x02
#define M5MO_WB_AWB_MANUAL	0x03

/* M5MO_CATEGORY_EXIF: 0x07 */
#define M5MO_EXIF_EXPTIME_NUM	0x00
#define M5MO_EXIF_EXPTIME_DEN	0x04
#define M5MO_EXIF_TV_NUM	0x08
#define M5MO_EXIF_TV_DEN	0x0C
#define M5MO_EXIF_BV_NUM	0x18
#define M5MO_EXIF_BV_DEN	0x1C
#define M5MO_EXIF_EBV_NUM	0x20
#define M5MO_EXIF_EBV_DEN	0x24
#define M5MO_EXIF_ISO		0x28
#define M5MO_EXIF_FLASH		0x2A

/* M5MO_CATEGORY_FD: 0x09 */
#define M5MO_FD_CTL		0x00
#define M5MO_FD_SIZE		0x01
#define M5MO_FD_MAX		0x02

/* M5MO_CATEGORY_LENS: 0x0A */
#define M5MO_LENS_AF_MODE	0x01
#define M5MO_LENS_AF_START	0x02
#define M5MO_LENS_AF_STATUS	0x03

#define M5MO_LENS_AF_MODE_SELECT	0x05
#define M5MO_LENS_AF_UPBYTE_STEP	0x06
#define M5MO_LENS_AF_LOWBYTE_STEP	0x07
#define M5MO_LENS_AF_CAL_DATA_READ	0x0C
#define M5MO_LENS_AF_CAL_DATA		0x13
#define M5MO_LENS_AF_CAL	0x1D
#define M5MO_LENS_AF_TOUCH_POSX	0x30
#define M5MO_LENS_AF_TOUCH_POSY	0x32

/* M5MO_CATEGORY_CAPPARM: 0x0B */
#define M5MO_CAPPARM_YUVOUT_MAIN	0x00
#define M5MO_CAPPARM_MAIN_IMG_SIZE	0x01
#define M5MO_CAPPARM_YUVOUT_PREVIEW	0x05
#define M5MO_CAPPARM_PREVIEW_IMG_SIZE	0x06
#define M5MO_CAPPARM_YUVOUT_THUMB	0x0A
#define M5MO_CAPPARM_THUMB_IMG_SIZE	0x0B
#define M5MO_CAPPARM_JPEG_SIZE_MAX	0x0F
#define M5MO_CAPPARM_JPEG_RATIO		0x17
#define M5MO_CAPPARM_MCC_MODE		0x1D
#define M5MO_CAPPARM_WDR_EN		0x2C
#define M5MO_CAPPARM_LIGHT_CTRL		0x40
#define M5MO_CAPPARM_FLASH_CTRL		0x41
#define M5MO_CAPPARM_JPEG_RATIO_OFS	0x34
#define M5MO_CAPPARM_THUMB_JPEG_MAX	0x3C
#define M5MO_CAPPARM_AFB_CAP_EN		0x53
#define M5MO_CAPPARM_FLASH_LOWTEMP	0x21

/* M5MO_CATEGORY_CAPCTRL: 0x0C */
#define M5MO_CAPCTRL_CAP_MODE	0x00
#define M5MO_CAPCTRL_CAP_FRM_COUNT 0x02
#define M5MO_CAPCTRL_FRM_SEL	0x06
#define M5MO_CAPCTRL_TRANSFER	0x09
#define M5MO_CAPCTRL_IMG_SIZE	0x0D
#define M5MO_CAPCTRL_THUMB_SIZE	0x11

/* M5MO_CATEGORY_ADJST: 0x0E */
#define M5MO_ADJST_AWB_RG_H	0x3C
#define M5MO_ADJST_AWB_RG_L	0x3D
#define M5MO_ADJST_AWB_BG_H	0x3E
#define M5MO_ADJST_AWB_BG_L	0x3F

/* M5MO_CATEGORY_FLASH: 0x0F */
#define M5MO_FLASH_ADDR		0x00
#define M5MO_FLASH_BYTE		0x04
#define M5MO_FLASH_ERASE	0x06
#define M5MO_FLASH_WR		0x07
#define M5MO_FLASH_RAM_CLEAR	0x08
#define M5MO_FLASH_CAM_START	0x12
#define M5MO_FLASH_SEL		0x13

/* M5MO_CATEGORY_TEST:	0x0D */
#define M5MO_TEST_OUTPUT_YCO_TEST_DATA		0x1B
#define M5MO_TEST_ISP_PROCESS			0x59

/* M5MO Sensor Mode */
#define M5MO_SYSINIT_MODE	0x0
#define M5MO_PARMSET_MODE	0x1
#define M5MO_MONITOR_MODE	0x2
#define M5MO_STILLCAP_MODE	0x3

#define M5MOLS_SHUTTER_TIMEOUT 1000
#define M5MOLS_VERSION_INFO_SIZE 6

enum {
	SHUTTER_STANDBY = 0,
	SHUTTER_FORWARD,
	SHUTTER_REVERSE,
};

enum {
	IR_FILTER_OPEN = 0,
	IR_FILTER_CW,
	IR_FILTER_CCW,
};

struct m5mols_shuter {
	int ldo_en;
	int in1_gpio;
	int in2_gpio;
	int sense_upper_gpio;
	int sense_lower_gpio;
	int sense_upper_irq;
	int sense_lower_irq;
	wait_queue_head_t wait;
	unsigned int issued;
};

struct m5mols_irfilter {
	int ldo_en;
	int filter_en;
	int filter_on;
	int filter_off;
};

struct m5mols_data {
	struct msm_sensor_ctrl_t *msensor_ctrl;
	struct device *m5mols_cam;
	struct m5mols_shuter *shutter;
	struct m5mols_irfilter *irfilter;
	int power_count;
};

int32_t m5mols_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp);
int32_t m5mols_sensor_native_control(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp);
int m5mols_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl);
int m5mo_dump_fw(struct i2c_client *client);
int32_t m5mols_sensor_init(struct msm_sensor_ctrl_t *s_ctrl);

#endif /* SEC_M5MO_H */
