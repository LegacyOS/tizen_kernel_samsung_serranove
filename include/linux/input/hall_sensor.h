/*
 * include/linux/input/hall_sensor.h
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Authors:
 *	Joong-Mock Shin <jmock.shin@samsung.com>
 *	Taeheon Kim <th908.kim@samsung.com>
 *	Sangmin Lee <lsmin.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

struct hall_sensor_driverdata {
	int gpio_detect;
	int hall_irq;
	bool status;
	struct delayed_work detect_dwork;
	struct platform_device *dev;
	struct wake_lock wake_lock;
};

struct hall_sensor_platform_data {
	int gpio_detect;
	unsigned int irq_gpio_flags;
};

#endif
