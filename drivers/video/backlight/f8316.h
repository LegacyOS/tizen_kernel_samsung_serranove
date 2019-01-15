#ifndef __DOT_LED_H__
#define __DOT_LED_H__

/* F8316 REG*/
#define REG_LED_BUF1	0x10
#define REG_LED_BUF2	0x25
#define REG_LED_CTRL1	0x05
#define REG_LED_CTRL2	0x06

#define REG_BUF1_LEN	0x15
#define REG_BUF2_LEN	0x15

/* F8316 COMMAND */
#define REG_LED_UPDATE	(1 << 7)
#define REG_LED_CLEAR	(1 << 6)
#define REG_LED_SLIDE	(1 << 4)
#define REG_LED_STOP	(1 << 7)
#define REG_LED_SLIDING1	(1 << 0)
#define REG_LED_SLIDING2	(1 << 1)
#endif
