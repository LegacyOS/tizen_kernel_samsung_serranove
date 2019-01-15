#ifndef __HDMI_TX_H__
#define __HDMI_TX_H__

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/async.h>


#define AUX_ERR  1
#define AUX_OK   0


#define LOG_TAG "[SiI9136]"


#define GPIO_HDMI_INT	155
#define GPIO_HDMI_RESET_N	156

#if 1
#define DEV_DBG_ERROR(fmt, args...)         pr_info(KERN_ERR "===== [SiI9136 Exception] [%s][%d] :: "fmt, __func__, __LINE__, ##args);
#define DEV_DBG(fmt, args...) pr_info(KERN_DEBUG "[SiI9136][%s] :: " fmt, __func__, ##args);
#else
#define SII_DEV_DBG_ERROR(fmt, args...)
#define SII_DEV_DBG(fmt, args...)
#endif



struct sii9136_platform_data {
	struct i2c_client *client;
	struct hrtimer timer;
	struct delayed_work work;
	unsigned int reset_gpio;
	unsigned int irq_gpio;
};

typedef struct
{
    unsigned short vertical_lines_total;
    unsigned short horizontal_pixels_total;
    unsigned short vertical_lines_active;
    unsigned short horizontal_pixels_active;
    unsigned char  color_depth;
	unsigned char  hdcp_authenticated;
} VideoFormat_t;

#endif
