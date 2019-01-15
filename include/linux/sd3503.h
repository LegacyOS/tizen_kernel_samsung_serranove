/*
 * Zwave SD3503
 *
 * Copyright (C) 2015 Samsung Electronics Co.Ltd
 * Author: SangSu Lee <constant.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 */
#define ZWAVE_DRIVER_NAME		"sd3503"
 /* ioctl */
#define SD3503_MAGIC		'Z'
#define ZWAVE_RESET_HIGH		_IOW(SD3503_MAGIC, 0, unsigned int)
#define ZWAVE_RESET_LOW			_IOW(SD3503_MAGIC, 1, unsigned int)
#define ZWAVE_SWITCH_HIGH		_IOW(SD3503_MAGIC, 2, unsigned int)
#define ZWAVE_SWITCH_LOW		_IOW(SD3503_MAGIC, 3, unsigned int)
