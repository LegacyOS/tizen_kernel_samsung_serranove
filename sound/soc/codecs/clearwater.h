/*
 * clearwater.h  --  ALSA SoC Audio driver for Florida-class codecs
 *
 * Copyright 2012 Wolfson Microelectronics plc
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _CLEARWATER_H
#define _CLEARWATER_H

#include "arizona.h"

#define CLEARWATER_FLL1        1
#define CLEARWATER_FLL2        2
#define CLEARWATER_FLL1_REFCLK 3
#define CLEARWATER_FLL2_REFCLK 4
#define CLEARWATER_FLL3	   5
#define CLEARWATER_FLL3_REFCLK 6

#ifdef CONFIG_SND_SOC_MSM8X16_ARIZONA
int clearwater_asoc_platform_register(struct device *dev);
void clearwater_asoc_platform_unregister(struct device *dev);
int clearwater_speaker_set_mute(struct snd_soc_codec *codec, int onoff);
#endif

#endif
