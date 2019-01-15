#ifndef __LINUX_TPA2026D2_H
#define __LINUX_TPA2026D2_H

#include <sound/soc.h>

#define TPA6130A2_REG_AGC_1		0x01
#define TPA6130A2_REG_AGC_5		0x05
#define TPA6130A2_REG_AGC_6		0x06
#define TPA6130A2_REG_AGC_7		0x07

int tpa2026d2_add_controls(struct snd_soc_codec *codec);
void tpa2026d2_spk_power(int power);

#endif
