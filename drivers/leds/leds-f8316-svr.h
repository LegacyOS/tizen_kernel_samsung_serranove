#ifndef __DOT_LED_H__
#define __DOT_LED_H__

#define DEV_ERROR(fmt, args...) pr_err("%s :: "fmt, __func__, ##args);
#define DEV_INFO(fmt, args...) pr_info("[%s] :: " fmt, __func__, ##args);

/* F8316 REG*/
#define REG_LED_BUF1	0x10
#define REG_LED_BUF2	0x25
#define REG_LED_CTRL1	0x05
#define REG_LED_CTRL2	0x06

#define REG_BUF1_LEN	0x15
#define REG_BUF2_LEN	0x15

/* F8316 COMMAND */
#define REG_LED_UPDATE	1 << 7
#define REG_LED_CLEAR	1 << 6
#define REG_LED_SLIDE	1 << 4
#define REG_LED_STOP	1 << 7


struct f8316_platform_data {
	struct i2c_client *client;
	struct regulator *vdd;
	unsigned int reset_gpio;
};
int update_matrix(int mode, unsigned char *matrix);
#endif
