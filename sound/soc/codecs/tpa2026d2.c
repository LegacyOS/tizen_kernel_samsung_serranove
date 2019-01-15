/*****************************************************************************
 *  Copyright 2001 - 2012 Broadcom Corporation.  All rights reserved.
 *
 *  Unless you and Broadcom execute a separate written software license
 *  agreement governing use of this software, this software is licensed to you
 *  under the terms of the GNU General Public License version 2, available at
 *  http://www.gnu.org/licenses/old-license/gpl-2.0.html (the "GPL").
 *
 *  Notwithstanding the above, under no circumstances may you combine this
 *  software in any way with any other Broadcom software provided under a
 *  license other than the GPL, without Broadcom's express prior written
 *  consent.
 *
 *****************************************************************************/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <sound/control.h>
#include <sound/tlv.h>

#include "tpa2026d2.h"

struct tpa2026d2 {
	int power_gpio;
	struct mutex mutex;
};

static int tpa2026d2_get_enum(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
static int tpa2026d2_put_enum(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
static int tpa2026d2_get_volsw(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);
static int tpa2026d2_put_volsw(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);


static struct i2c_client *tpa2026d2_client;

static const unsigned int tpa2026d2_fixed_gain_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0, 30, TLV_DB_SCALE_ITEM(0, 100, 0),
	36, 63, TLV_DB_SCALE_ITEM(-2800, 100, 0),
};

static DECLARE_TLV_DB_SCALE(tpa2026d2_output_limter_level_tlv, -650, 50, 0);
static DECLARE_TLV_DB_SCALE(tpa2026d2_max_gain_tlv, 1800, 100, 0);

static const char * const tpa2026d2_spk_lr_ng_ctl_text[] = {
		"DISABLE", "ENABLE", };
static const char * const tpa2026d2_sws_ctl_text[] = {
				"ENABLE", "DISABLE", };

static const struct soc_enum tpa2026d2_ic_ctl_enum[] = {
	SOC_ENUM_SINGLE(TPA6130A2_REG_AGC_1, 6,
			ARRAY_SIZE(tpa2026d2_spk_lr_ng_ctl_text), tpa2026d2_spk_lr_ng_ctl_text),
	SOC_ENUM_SINGLE(TPA6130A2_REG_AGC_1, 7,
			ARRAY_SIZE(tpa2026d2_spk_lr_ng_ctl_text), tpa2026d2_spk_lr_ng_ctl_text),
	SOC_ENUM_SINGLE(TPA6130A2_REG_AGC_1, 0,
			ARRAY_SIZE(tpa2026d2_spk_lr_ng_ctl_text), tpa2026d2_spk_lr_ng_ctl_text),
	SOC_ENUM_SINGLE(TPA6130A2_REG_AGC_1, 5,
			ARRAY_SIZE(tpa2026d2_sws_ctl_text), tpa2026d2_sws_ctl_text),
};

static const char * const tpa2026d2_output_limiter_switch_text[] = {
		"ENABLE", "DISABLE", };


static const struct soc_enum tpa2026d2_output_limiter_switch_enum =
	SOC_ENUM_SINGLE(TPA6130A2_REG_AGC_6, 5,
		ARRAY_SIZE(tpa2026d2_output_limiter_switch_text), tpa2026d2_output_limiter_switch_text);

static const char * const tpa2026d2_noisegate_threshold_text[] = {
		"1", "4", "10", "20"};

static const struct soc_enum tpa2026d2_noisegate_threshold_enum =
	SOC_ENUM_SINGLE(TPA6130A2_REG_AGC_6, 0,
		ARRAY_SIZE(tpa2026d2_noisegate_threshold_text), tpa2026d2_noisegate_threshold_text);


static const char * const tpa2026d2_compression_ratio_text[] = {
		"1:1", "2:1", "4:1", "8:1"};

static const struct soc_enum tpa2026d2_compression_ratio_enum =
	SOC_ENUM_SINGLE(TPA6130A2_REG_AGC_7, 0,
		ARRAY_SIZE(tpa2026d2_compression_ratio_text), tpa2026d2_compression_ratio_text);



static const struct snd_kcontrol_new tpa2026d2_controls[] = {
	/*reg 1*/
	SOC_ENUM_EXT("TPA2026D2 Spk Left Switch", tpa2026d2_ic_ctl_enum[0],
		tpa2026d2_get_enum, tpa2026d2_put_enum),
	SOC_ENUM_EXT("TPA2026D2 Spk Right Switch", tpa2026d2_ic_ctl_enum[1],
		tpa2026d2_get_enum, tpa2026d2_put_enum),
	SOC_ENUM_EXT("TPA2026D2 Noise Gate Switch", tpa2026d2_ic_ctl_enum[2],
		tpa2026d2_get_enum, tpa2026d2_put_enum),
	SOC_ENUM_EXT("TPA2026D2 SWS Switch", tpa2026d2_ic_ctl_enum[3],
		tpa2026d2_get_enum, tpa2026d2_put_enum),

	/*reg 5*/
	SOC_SINGLE_EXT_TLV("TPA2026D2 Fixed Gain Control",
		       TPA6130A2_REG_AGC_5, 0, 0x3f, 0,
		       tpa2026d2_get_volsw, tpa2026d2_put_volsw,
		       tpa2026d2_fixed_gain_tlv),

	/*reg 6*/
	SOC_ENUM_EXT("TPA2026D2 Output Limiter Switch", tpa2026d2_output_limiter_switch_enum,
		tpa2026d2_get_enum, tpa2026d2_put_enum),
	SOC_ENUM_EXT("TPA2026D2 NoiseGate Threshold", tpa2026d2_noisegate_threshold_enum,
		tpa2026d2_get_enum, tpa2026d2_put_enum),

	SOC_SINGLE_EXT_TLV("TPA2026D2 Output Limiter Level Control",
		       TPA6130A2_REG_AGC_6, 0, 0x1f, 0,
		       tpa2026d2_get_volsw, tpa2026d2_put_volsw,
		       tpa2026d2_output_limter_level_tlv),

	/*reg 7*/
	SOC_SINGLE_EXT_TLV("TPA2026D2 Max Gain Control",
		       TPA6130A2_REG_AGC_7, 4, 0x0C, 0,
		       tpa2026d2_get_volsw, tpa2026d2_put_volsw,
		       tpa2026d2_max_gain_tlv),

	SOC_ENUM_EXT("TPA2026D2 Compression Ratio", tpa2026d2_compression_ratio_enum,
		tpa2026d2_get_enum, tpa2026d2_put_enum),
};


static int tpa2026d2_i2c_read(int reg)
{
	int val ;

	BUG_ON(tpa2026d2_client == NULL);

	val = i2c_smbus_read_byte_data(tpa2026d2_client, reg);
	if (val < 0) {
		dev_err(&tpa2026d2_client->dev, "Read failed\n");
	}
	return val;
}


static int tpa2026d2_i2c_write(int reg, u8 value)
{
	int val ;

	BUG_ON(tpa2026d2_client == NULL);
	val = i2c_smbus_write_byte_data(tpa2026d2_client, reg, value);
	if (val < 0) {
		dev_err(&tpa2026d2_client->dev, "Write failed\n");
		return val;
	}

	return val;
}


static int tpa2026d2_get_enum(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct tpa2026d2 *data;

	BUG_ON(tpa2026d2_client == NULL);
	data = i2c_get_clientdata(tpa2026d2_client);

	mutex_lock(&data->mutex);

	ucontrol->value.enumerated.item[0] =
		(tpa2026d2_i2c_read(e->reg) >> e->shift_l) & e->mask;

	mutex_unlock(&data->mutex);
	return 0;
}


static int tpa2026d2_put_enum(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct tpa2026d2 *data;
	unsigned int val;
	unsigned int val_reg;
	val = ucontrol->value.enumerated.item[0] ;
	BUG_ON(tpa2026d2_client == NULL);
	data = i2c_get_clientdata(tpa2026d2_client);

	mutex_lock(&data->mutex);

	val_reg = tpa2026d2_i2c_read(e->reg);
	if (((val_reg >> e->shift_l) & e->mask) == val) {
		mutex_unlock(&data->mutex);
		return 0;
	}

	val_reg &= ~(e->mask << e->shift_l);
	val_reg |= val << e->shift_l;

	tpa2026d2_i2c_write(e->reg, val_reg);

	mutex_unlock(&data->mutex);
	return 1;
}


static int tpa2026d2_get_volsw(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct tpa2026d2 *data;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;

	BUG_ON(tpa2026d2_client == NULL);
	data = i2c_get_clientdata(tpa2026d2_client);

	mutex_lock(&data->mutex);

	ucontrol->value.integer.value[0] =
		(tpa2026d2_i2c_read(reg) >> shift) & mask;

	if (invert)
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];

	mutex_unlock(&data->mutex);
	return 0;
}

static int tpa2026d2_put_volsw(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct tpa2026d2 *data;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned int val = (ucontrol->value.integer.value[0] & mask);
	unsigned int val_reg;

	BUG_ON(tpa2026d2_client == NULL);
	data = i2c_get_clientdata(tpa2026d2_client);

	if (invert) {
		val = max - val;
	}

	mutex_lock(&data->mutex);

	val_reg = tpa2026d2_i2c_read(reg);
	if (((val_reg >> shift) & mask) == val) {
		mutex_unlock(&data->mutex);
		return 0;
	}

	val_reg &= ~(mask << shift);
	val_reg |= val << shift;

	tpa2026d2_i2c_write(reg, val_reg);

	mutex_unlock(&data->mutex);

	return 1;
}


void tpa2026d2_spk_power(int power)
{
	struct tpa2026d2 *data;
	BUG_ON(tpa2026d2_client == NULL);

	data = i2c_get_clientdata(tpa2026d2_client);
	mutex_lock(&data->mutex);

	if (power) {
		gpio_set_value(data->power_gpio, 1);
	} else {
		gpio_set_value(data->power_gpio, 0);
	}
	mutex_unlock(&data->mutex);
}

int tpa2026d2_add_controls(struct snd_soc_codec *codec)
{
	struct tpa2026d2 *data;
	BUG_ON(tpa2026d2_client == NULL);

	data = i2c_get_clientdata(tpa2026d2_client);

	return snd_soc_add_codec_controls(codec, tpa2026d2_controls,
						ARRAY_SIZE(tpa2026d2_controls));
}

static int tpa2026d2_i2c_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
{
	struct tpa2026d2 *t_data;
	int ret = 0;
	t_data = kzalloc(sizeof(struct tpa2026d2), GFP_KERNEL);

	if (t_data == NULL) {
		pr_err("%s failed to alloc mem.\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	tpa2026d2_client = i2c;
	i2c_set_clientdata(i2c, t_data);

	mutex_init(&t_data->mutex);

	t_data->power_gpio = of_get_named_gpio(i2c->dev.of_node,
					       "tpa2026d2,sdz-gpio", 0);
	if (t_data->power_gpio < 0) {
		pr_err("%s, getting gpio failed\n", __func__);
		ret = ENODEV;
		goto err;
	}
	gpio_direction_output(t_data->power_gpio, 0);
	return ret;
 err:
	kfree(t_data);
	tpa2026d2_client = NULL;
	return ret;
}

static int tpa2026d2_i2c_remove(struct i2c_client *i2c)
{

	struct tpa2026d2 *t_data;
	if (tpa2026d2_client == NULL) {
		return -ENODEV;
	}
	t_data = i2c_get_clientdata(tpa2026d2_client);
	kfree(t_data);
	tpa2026d2_client = NULL;
	return 0;
}

static const struct i2c_device_id tpa2026d2_i2c_id[] = {
	{"tpa2026d2", 0},
};

MODULE_DEVICE_TABLE(i2c, tpa2026d2_i2c_id);

static struct i2c_driver tpa2026d2_i2c_driver = {
	.driver = {
		   .name = "tpa2026d2",
		   .owner = THIS_MODULE,
		   },
	.probe = tpa2026d2_i2c_probe,
	.remove = tpa2026d2_i2c_remove,
	.id_table = tpa2026d2_i2c_id,
};

module_i2c_driver(tpa2026d2_i2c_driver);

MODULE_DESCRIPTION("I2C support for TPA2026D2");
MODULE_LICENSE("GPL");
