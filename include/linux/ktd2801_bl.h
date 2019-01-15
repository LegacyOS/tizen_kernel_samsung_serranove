 /*
 * Copyright 2014 Samsung Electronics Co.Ltd.All rights reserved.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

 #ifndef __LINUX_KTD2801A_BL_H
 #define __LINUX_KTD2801A_BL_H

struct brt_value {
	int level;				/* Platform setting values */
	int tune_level;			/* Chip Setting values */
};

void ktd2801_backlight_set_brightness(int level);
 #endif
