#ifndef __SR352_H__
#define __SR352_H__

#include "msm_sensor.h"
#include "msm_sensor_driver.h"

#undef DEBUG_LEVEL_HIGH
#undef CDBG

#define DEBUG_LEVEL_HIGH
#ifdef DEBUG_LEVEL_HIGH
#define CDBG(fmt, args...)	printk("[SR352] %s : %d : " fmt "\n",   __FUNCTION__, __LINE__, ##args)
#endif

int32_t sr352_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp);
int32_t sr352_sensor_native_control(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp);
void sr352_set_default_settings(void);

int sr352_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl);

#endif	//__SR352_H__
