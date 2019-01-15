/*
 * drivers/motor/sm5703_vibrator.h
 *
 * Header of SiliconMitus SM5703 Vibrator Driver
 *
 * Copyright (C) 2015 SiliconMitus
 * Author: SW Jung
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef SM5703_VIBRATOR_H
#define SM5703_VIBRATOR_H

struct sm5703_vib {
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	struct work_struct work;
	struct workqueue_struct *queue;

	struct regulator *vib_power;
	const char *power_name;
	int power_volt;
	int state;
	int timeout;
	struct mutex lock;
};

extern struct sm5703_vib *sm_vib;
extern void sm5703_set_vibrator(struct regulator *reg_vib, int on);

#endif // SM5703_VIBRATOR_H
