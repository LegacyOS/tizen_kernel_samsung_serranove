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

#ifndef __SSP_PRJ_H__
#define __SSP_PRJ_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/rtc.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/ssp_platformdata.h>
#include "factory/ssp_factory.h"
#include "factory/ssp_mcu.h"
#include "ssp_sensorlist.h"
#include "ssp_data.h"
#include "ssp_debug.h"
#include "ssp_dev.h"
#include "ssp_firmware.h"

#ifdef SSP_USE_IIO_BATCH
#include "ssp_iio.h"
#else
#include "ssp_input.h"
#endif
/*
#include "ssp_misc.h"
*/
#include "ssp_sensorhub.h"
#include "ssp_spi.h"
#include "ssp_sysfs.h"
#include "sensors_core.h"

#undef CONFIG_SEC_DEBUG
#define CONFIG_SEC_DEBUG	0

#define DEFUALT_POLLING_DELAY	(200 * NSEC_PER_MSEC)
#define DATA_PACKET_SIZE	960

/* AP -> SSP Instruction */
#define MSG2SSP_INST_BYPASS_SENSOR_ADD		0xA1
#define MSG2SSP_INST_BYPASS_SENSOR_REMOVE	0xA2
#define MSG2SSP_INST_REMOVE_ALL			0xA3
#define MSG2SSP_INST_CHANGE_DELAY		0xA4
#define MSG2SSP_INST_LIBRARY_ADD		0xB1
#define MSG2SSP_INST_LIBRARY_REMOVE		0xB2
#define MSG2SSP_INST_LIB_NOTI			0xB4
#define MSG2SSP_INST_LIB_DATA			0xC1

#define MSG2SSP_AP_FUSEROM			0X01

#define MSG2SSP_AP_MCU_SET_BARO_CAL		0x8E
#define MSG2SSP_AP_MCU_SET_GYRO_CAL		0xCD
#define MSG2SSP_AP_MCU_SET_ACCEL_CAL		0xCE
#define MSG2SSP_AP_MCU_SET_HRM_OSC_REG		0xCF
#define MSG2SSP_AP_STATUS_SHUTDOWN		0xD0
#define MSG2SSP_AP_STATUS_WAKEUP		0xD1
#define MSG2SSP_AP_STATUS_SLEEP			0xD2
#define MSG2SSP_AP_STATUS_RESUME		0xD3
#define MSG2SSP_AP_STATUS_SUSPEND		0xD4
#define MSG2SSP_AP_STATUS_RESET			0xD5
#define MSG2SSP_AP_STATUS_POW_CONNECTED	0xD6
#define MSG2SSP_AP_STATUS_POW_DISCONNECTED	0xD7
#define MSG2SSP_AP_TEMPHUMIDITY_CAL_DONE	0xDA
#define MSG2SSP_AP_MCU_SET_DUMPMODE		0xDB
#define MSG2SSP_AP_MCU_BATCH_FLUSH		0xDD
#define MSG2SSP_AP_MCU_BATCH_COUNT		0xDF

#ifdef CONFIG_SENSORS_MULTIPLE_GLASS_TYPE
#define MSG2SSP_AP_GLASS_TYPE             0xEC
#endif

#define MSG2SSP_AP_WHOAMI			0x0F
#define MSG2SSP_AP_FIRMWARE_REV			0xF0
#define MSG2SSP_AP_SENSOR_FORMATION		0xF1
#define MSG2SSP_AP_SENSOR_PROXTHRESHOLD		0xF2
#define MSG2SSP_AP_SENSOR_BARCODE_EMUL		0xF3
#define MSG2SSP_AP_SENSOR_SCANNING		0xF4
#define MSG2SSP_AP_SET_MAGNETIC_HWOFFSET	0xF5
#define MSG2SSP_AP_GET_MAGNETIC_HWOFFSET	0xF6
#define MSG2SSP_AP_SENSOR_GESTURE_CURRENT	0xF7
#define MSG2SSP_AP_GET_THERM			0xF8
#define MSG2SSP_AP_GET_BIG_DATA			0xF9
#define MSG2SSP_AP_SET_BIG_DATA			0xFA
#define MSG2SSP_AP_START_BIG_DATA		0xFB
#define MSG2SSP_AP_GET_UV_DEVICE_ID		0xFC
#define MSG2SSP_AP_SENSOR_PIRTHRESHOLD	0xFC
#define MSG2SSP_AP_SET_MAGNETIC_STATIC_MATRIX	0xFD
#define MSG2SSP_AP_SENSOR_TILT			0xEA
#define MSG2SSP_AP_MCU_SET_TIME			0xFE
#define MSG2SSP_AP_MCU_GET_TIME			0xFF
#define MSG2SSP_AP_MOBEAM_DATA_SET		0x31
#define MSG2SSP_AP_MOBEAM_REGISTER_SET		0x32
#define MSG2SSP_AP_MOBEAM_COUNT_SET		0x33
#define MSG2SSP_AP_MOBEAM_START			0x34
#define MSG2SSP_AP_MOBEAM_STOP			0x35
#define MSG2SSP_AP_GEOMAG_LOGGING		0x36
#define MSG2SSP_AP_SENSOR_LPF			0x37
#define MSG2SSP_AP_IRDATA_SEND			0x38
#define MSG2SSP_AP_IRDATA_SEND_RESULT		0x39
#define MSG2SSP_AP_PROX_GET_TRIM		0x40

#define SUCCESS					1
#define FAIL					0
#define ERROR					-1

#define ssp_dbg(format, ...) do { \
	pr_debug("[SSP] " format "\n", ##__VA_ARGS__); \
	} while (0)

#define ssp_info(format, ...) do { \
	pr_info("[SSP] " format "\n", ##__VA_ARGS__); \
	} while (0)

#define ssp_err(format, ...) do { \
	pr_err("[SSP] " format "\n", ##__VA_ARGS__); \
	} while (0)

#define ssp_dbgf(format, ...) do { \
	pr_debug("[SSP] %s: " format "\n", __func__, ##__VA_ARGS__); \
	} while (0)

#define ssp_infof(format, ...) do { \
	pr_info("[SSP] %s: " format "\n", __func__, ##__VA_ARGS__); \
	} while (0)

#define ssp_errf(format, ...) do { \
	pr_err("[SSP] %s: " format "\n", __func__, ##__VA_ARGS__); \
	} while (0)

/* SSP Binary Type */
enum {
	KERNEL_BINARY = 0,
	KERNEL_CRASHED_BINARY,
	UMS_BINARY,
};

/* temphumidity sensor*/
#define MAX_COMP_BUFF	60
struct shtc1_buffer {
	u16 batt[MAX_COMP_BUFF];
	u16 chg[MAX_COMP_BUFF];
	s16 temp[MAX_COMP_BUFF];
	u16 humidity[MAX_COMP_BUFF];
	u16 baro[MAX_COMP_BUFF];
	u16 gyro[MAX_COMP_BUFF];
	char len;
};

/*
 * SENSOR_DELAY_SET_STATE
 * Check delay set to avoid sending ADD instruction twice
 */
enum {
	INITIALIZATION_STATE = 0,
	NO_SENSOR_STATE,
	ADD_SENSOR_STATE,
	RUNNING_SENSOR_STATE,
};

/* Firmware download STATE */
enum {
	FW_DL_STATE_FAIL = -1,
	FW_DL_STATE_NONE = 0,
	FW_DL_STATE_NEED_TO_SCHEDULE,
	FW_DL_STATE_SCHEDULED,
	FW_DL_STATE_DOWNLOADING,
	FW_DL_STATE_SYNC,
	FW_DL_STATE_DONE,
};

/* SSP_INSTRUCTION_CMD */
enum {
	REMOVE_SENSOR = 0,
	ADD_SENSOR,
	CHANGE_DELAY,
	GO_SLEEP,
	REMOVE_LIBRARY,
	ADD_LIBRARY,
};

/* SENSOR_TYPE */
enum {
	ACCELEROMETER_SENSOR = 0,		/* [0]=1 */
	GYROSCOPE_SENSOR,				/* [1]=2 */
	GEOMAGNETIC_UNCALIB_SENSOR,	/* [2]=4 */
	GEOMAGNETIC_RAW,				/* [3]=8 */
	GEOMAGNETIC_SENSOR,			/* [4]=16 */
	PRESSURE_SENSOR,				/* [5]=32 */
	GESTURE_SENSOR,				/* [6]=64 */
	PROXIMITY_SENSOR,				/* [7]=128 */
	TEMPERATURE_HUMIDITY_SENSOR,/* [8]=256 */
	LIGHT_SENSOR,					/* [9]=512 */
	PROXIMITY_RAW,				/* [10]=1024 */
	ORIENTATION_SENSOR,			/* [11]=2048 */
	STEP_DETECTOR = 12,			/* [12]=4096 */
	SIG_MOTION_SENSOR,			/* [13]=8192 */
	GYRO_UNCALIB_SENSOR,			/* [14]=16384 */
	GAME_ROTATION_VECTOR = 15,	/* [15]=32768 */
	ROTATION_VECTOR,				/* [16]=65536 */
	STEP_COUNTER,					/* [17]=131072 */
	BIO_HRM_RAW = 18,				/* [18]=262144 */
	BIO_HRM_RAW_FAC,				/* [19]=524288 */
	BIO_HRM_LIB,					/* [20]=1048576 */
	TILT_MOTION,					/* [21]=2097152 */
	UV_SENSOR,						/* [22]=4194304 */
	PIR_SENSOR,						/* [23]=8388608 */
	PIR_SENSOR_RAW,				/* [24]=16777216 */
	META_SENSOR,
	SENSOR_MAX, /*  = 25 */
};

enum {
	AP2HUB_READ = 0,
	AP2HUB_WRITE,
	HUB2AP_WRITE,
	AP2HUB_READY,
	AP2HUB_RETURN
};

enum {
	BIG_TYPE_DUMP = 0,
	BIG_TYPE_READ_LIB,
	BIG_TYPE_MAX,
};

extern struct class *sensors_event_class;

struct sensor_value {
	union {
		struct { /* accel, gyro, mag */
			s16 x;
			s16 y;
			s16 z;
			u32 gyro_dps;
		} __attribute__((__packed__));
		struct {		/*calibrated mag, gyro*/
			s16 cal_x;
			s16 cal_y;
			s16 cal_z;
			u8 accuracy;
		} __attribute__((__packed__));
		struct {		/*uncalibrated mag, gyro*/
			s16 uncal_x;
			s16 uncal_y;
			s16 uncal_z;
			s16 offset_x;
			s16 offset_y;
			s16 offset_z;
		} __attribute__((__packed__));
		struct {		/* rotation vector */
			s32 quat_a;
			s32 quat_b;
			s32 quat_c;
			s32 quat_d;
			u8 acc_rot;
		} __attribute__((__packed__));
#if defined(CONFIG_SENSORS_SSP_AL3320)
		struct {		/* light sensor */
			u8 raw_high;
			u8 raw_low;
		} __attribute__((__packed__));
#elif defined(CONFIG_SENSORS_SSP_CM36686)
		struct {
			u16 als;
			u16 white;
		} __attribute__((__packed__));
#else
		struct {		/* light sensor */
			u16 r;
			u16 g;
			u16 b;
			u16 w;
			u8 a_time;
			u8 a_gain;

		} __attribute__((__packed__));
#endif
		struct { /* pressure */
			s32 pressure;
			s16 temperature;
			s32 pressure_cal;
			s32 pressure_sealevel;
		} __attribute__((__packed__));
		struct { /* proximity */
			u8 prox;
			u16 prox_ex;
		} __attribute__((__packed__));
		struct { /* proximity raw */
			u16 prox_raw[4];
		};
		struct {
			u32 ch_a_sum;
			u32 ch_a_x1;
			u32 ch_a_x2;
			u32 ch_a_y1;
			u32 ch_a_y2;
			u32 ch_b_sum;
			u32 ch_b_x1;
			u32 ch_b_x2;
			u32 ch_b_y1;
			u32 ch_b_y2;
		} __attribute__((__packed__));
		struct {
			u32 frequency;
			u32 green_dc_level;
			u32 green_high_dc_level;
			u32 green_mid_dc_level;
			u32 red_dc_levet;
			u32 ir_dc_level;
			u32 noise_level;
			u32 adc_offset[8];
			u32 oscRegValue;
		} __attribute__((__packed__));
		struct {
			s16 hr;
			s16 rri;
			s32 snr;
		} __attribute__((__packed__));
		struct meta_data_event { /* meta data */
			s32 what;
			s32 sensor;
		} __attribute__((__packed__)) meta_data;
		u8 uv_raw;
		u8 data[20];
		struct { /* humi/temp sensor */
			u16 temp;
			u16 humi;
			u8 time;
		} __attribute__((__packed__));
		struct { /* pir raw sensor */
			u16 near_temp;
			u16 near_adc;
			u16 far_temp;
			u16 far_adc;
		} __attribute__((__packed__));
		u8 event; /* pir sensor*/
	};
	u64 timestamp;
} __attribute__((__packed__));

#ifdef CONFIG_SAMSUNG_LPM_MODE
extern int poweroff_charging;
#endif

struct calibraion_data {
	s16 x;
	s16 y;
	s16 z;
};

struct hrm_cal_data {
	u32 osc_reg;
	u32 green_70ma;
	u32 green_250ma;
	u32 red_70ma;
	u32 ir_70ma;
};

struct ssp_msg {
	u8 cmd;
	u16 length;
	u16 options;
	u32 data;

	struct list_head list;
	struct completion *done;
	char *buffer;
	u8 free_buffer;
	bool *dead_hook;
	bool dead;
} __attribute__((__packed__));

enum {
	BATCH_MODE_NONE = 0,
	BATCH_MODE_RUN,
};

struct ssp_time_diff {
	u16 batch_count;
	u16 batch_mode;
	u64 time_diff;
	u64 irq_diff;
	u16 batch_count_fixed;
};

struct ssp_data {
#ifdef SSP_USE_IIO_BATCH
	char name[SENSOR_MAX][SENSOR_NAME_MAX_LEN];
	bool enable[SENSOR_MAX];
	int data_len[SENSOR_MAX];
	int report_len[SENSOR_MAX];
	struct iio_dev *indio_devs[SENSOR_MAX];
	struct iio_chan_spec indio_channels[SENSOR_MAX];
	struct device *devices[SENSOR_MAX];
#else
	struct input_dev *acc_input_dev;
	struct input_dev *gyro_input_dev;
	struct input_dev *motion_input_dev;
	struct input_dev *mag_input_dev;
	struct input_dev *uncal_mag_input_dev;
	struct input_dev *uncalib_gyro_input_dev;
	struct input_dev *pressure_input_dev;
	struct input_dev *gesture_input_dev;
	struct input_dev *rot_input_dev;
	struct input_dev *game_rot_input_dev;
	struct input_dev *light_input_dev;
	struct input_dev *prox_input_dev;
	struct input_dev *temp_humi_input_dev;
	struct input_dev *hrm_raw_input_dev;
	struct input_dev *hrm_lib_input_dev;
	struct input_dev *uv_input_dev;
	struct input_dev *pir_standard_input_dev;
	struct input_dev *pir_long_input_dev;
	struct input_dev *meta_input_dev;
#endif
	struct sensor_value buf[SENSOR_MAX];
	struct spi_device *spi;
	struct wake_lock ssp_wake_lock;
	struct timer_list debug_timer;
	struct workqueue_struct *debug_wq;
	struct workqueue_struct *lpm_motion_wq;
	struct work_struct work_debug;
	struct work_struct work_lpm_motion;

	struct calibraion_data accelcal;
	struct calibraion_data gyrocal;
	struct hrm_cal_data hrmcal;
	struct device *mcu_device;
	struct device *acc_device;
	struct device *gyro_device;
	struct device *mag_device;
	struct device *prs_device;
	struct device *prox_device;
	struct device *light_device;
	struct device *ges_device;
	struct device *temphumidity_device;
	struct device *hrm_device;
	struct device *uv_device;
	struct device *pir_standard_dev;
	struct device *pir_long_dev;
	struct miscdevice batch_io_device;

	int ap_rev;

	struct delayed_work work_firmware;
	struct delayed_work work_refresh;
	struct miscdevice shtc1_device;

	bool bSspShutdown;
	bool bAccelAlert;
	bool bProximityRawEnabled;
	bool bGeomagneticRawEnabled;
	bool bMcuDumpMode;
	bool bBinaryChashed;
	bool bProbeIsDone;
	bool bDumping;
	bool bLpModeEnabled;
	bool bTimeSyncing;

	unsigned int uProxCanc;
	unsigned int uCrosstalk;
	unsigned int uProxCalResult;
	unsigned int uProxHiThresh;
	unsigned int uProxLoThresh;
	unsigned int uProxHiThresh_default;
	unsigned int uProxLoThresh_default;
	unsigned int uIr_Current;
	unsigned char uFuseRomData[3];
	unsigned char uMagCntlRegData;
	char *pchLibraryBuf;
	char chLcdLdi[2];
	int iIrq;
	int iLibraryLength;
	int aiCheckStatus[SENSOR_MAX];
	atomic_t eol_enable;
	u32 lux;

	unsigned int uComFailCnt;
	unsigned int uResetCnt;
	unsigned int uTimeOutCnt;
	unsigned int uIrqCnt;
	unsigned int uDumpCnt;

	unsigned int uSensorState;
	unsigned int uCurFirmRev;
	unsigned int uFactoryProxAvg[4];
	char uLastResumeState;
	char uLastAPState;

	s32 iPressureCal;
	int sealevelpressure;

	atomic_t aSensorEnable;
	atomic_t apShutdownProgress;
	int64_t adDelayBuf[SENSOR_MAX];
	s32 batchLatencyBuf[SENSOR_MAX];
	s8 batchOptBuf[SENSOR_MAX];
	u64 lastTimestamp[SENSOR_MAX];
	bool reportedData[SENSOR_MAX];

	struct ssp_sensorhub_data *hub_data;

	int fw_dl_state;
	unsigned char pdc_matrix[PDC_SIZE];

	char *comp_engine_ver;
	char *comp_engine_ver2;
	struct mutex cp_temp_adc_lock;
	struct mutex bulk_temp_read_lock;
	struct shtc1_buffer *bulk_buffer;
	struct mutex comm_mutex;
	struct mutex pending_mutex;

	int accel_position;
	int mag_position;
	u8 mag_matrix_size;
	u8 *mag_matrix;
	u32 pir_standard_threshold;
	u32 pir_long_threshold;

	const char *fw_name;
	const char *ums_fw_name;

	int mcu_int1;
	int mcu_int2;
	int ap_int;
	int rst;

	struct list_head pending_list;
	int (*set_mcu_reset)(int);
	void (*get_sensor_data[SENSOR_MAX])(char *, int *,
		struct sensor_value *);
	void (*report_sensor_data[SENSOR_MAX])(struct ssp_data *,
		struct sensor_value *);
	int (*check_lpmode)(void);
	void (*ssp_big_task[BIG_TYPE_MAX])(struct work_struct *);
	u64 timestamp;
	struct file *realtime_dump_file;
	int total_dump_size;
};

struct ssp_big {
	struct ssp_data* data;
	struct work_struct work;
	u32 length;
	u32 addr;
};

#endif
