/*
 * sgtl5000.c  --  SGTL5000 ALSA SoC Audio driver
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * modification information
 * ------------------------
 * 2009/08/02 : merge L2.6.28_4.4.0_SS_Jul2009_source.
 *               check cpu_is_mx25().
 * 2009/09/06 : check suspend status at sgtl5000_dap_enable().
 *               suspend -> jack/change -> resume(->Interrupt -> resume-func)
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "sgtl5000.h"

extern void gpio_audio_sgtl_clk(int isEnable);

struct sgtl5000_priv {
	int sysclk;
	int master;
	int fmt;
	int rev;
	int lrclk;
};

/* suspend status */
static int suspend = 0;

/* platform device date */
static struct platform_device *pdevice = NULL;

/* headphone jack status */
static int hp_status;

static int sgtl5000_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level);

#define SGTL5000_MAX_CACHED_REG SGTL5000_DAP_AVC_DECAY
static u16 sgtl5000_regs[(SGTL5000_MAX_CACHED_REG >> 1) + 1];

#ifdef CONFIG_MACH_MX51_ERDOS
#define DAC_VOL_LIMIT	0x3c
#define DAC_VOL_MAX		0x9d
#define DAC_VOL_MIN		0x00
#define DAC_REG_MAX		0xfc
#define SPK_VOL_LIMIT	0x00
#define SPK_VOL_MAX		0x11
#define SPK_VOL_MIN		0x00
#define SPK_REG_MAX		0x1F
#define HP_VOL_LIMIT	0x0d
#define HP_VOL_MAX		0x2c
#define HP_VOL_MIN		0x00
#define HP_REG_MAX		0x7F
#else
#define DAC_VOL_LIMIT	0x3c
#define DAC_VOL_MAX		0xfc
#define DAC_VOL_MIN		0x00
#endif /* CONFIG_MACH_MX51_ERDOS */

static void volume_adjust_mono(unsigned int *value)
{
	unsigned int volume = *value;
	unsigned int vol_l = volume & 0x00ff;
	unsigned int vol_r = ( (volume & 0xff00) >>8 );

	volume = (vol_l + vol_r)/2;
	volume |= ( (vol_l + vol_r)/2 << 8);

	*value = (volume & 0xffff);
}

static void volume_fixes(unsigned int reg, unsigned int *value)
{
	unsigned int volume = *value;
	
	switch(reg){
	case SGTL5000_CHIP_DAC_VOL:
		volume += DAC_VOL_LIMIT;
		volume += DAC_VOL_LIMIT << 8;
		if(hp_status == 0){
			volume_adjust_mono(&volume);
		}
		if ( (volume & 0x00ff) >= DAC_VOL_MAX){
			volume = (volume & 0xff00) | DAC_REG_MAX;
		}
		if ( ( (volume & 0xff00) >> 8) >= DAC_VOL_MAX){
			volume = (volume & 0x00ff) | (DAC_REG_MAX << 8);
		}
		*value = (volume & 0xffff);
		break;

	case SGTL5000_CHIP_LINE_OUT_VOL:
		volume += SPK_VOL_LIMIT;
		volume += SPK_VOL_LIMIT << 8;
		if(hp_status == 0){
			volume_adjust_mono(&volume);
		}
		if ( (volume & 0x00ff) >= SPK_VOL_MAX){
			volume = (volume & 0xff00) | SPK_REG_MAX;
		}
		if ( ( (volume & 0xff00) >> 8) >= SPK_VOL_MAX){
			volume = (volume & 0x00ff) | (SPK_REG_MAX << 8);
		}
		*value = (volume & 0xffff);
		break;

	case SGTL5000_CHIP_ANA_HP_CTRL:
		volume += HP_VOL_LIMIT;
		volume += HP_VOL_LIMIT << 8;
		if ( (volume & 0x00ff) >= HP_VOL_MAX){
			volume = (volume & 0xff00) | HP_REG_MAX;
		}
		if ( ( (volume & 0xff00) >> 8) >= HP_VOL_MAX){
			volume = (volume & 0x00ff) | (HP_REG_MAX << 8);
		}
		*value = (volume & 0xffff);
		break;
	default:
		break;
	}
}

static unsigned int sgtl5000_read_reg_cache(struct snd_soc_codec *codec,
					    unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	unsigned int offset = reg >> 1;
	if (offset >= ARRAY_SIZE(sgtl5000_regs))
		return -EINVAL;
	return cache[offset];
}

static unsigned int sgtl5000_hw_read(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	struct i2c_client *client = codec->control_data;
	int i2c_ret;
	u16 value;
	u8 buf0[2], buf1[2];
	u16 addr = client->addr;
	u16 flags = client->flags;
	struct i2c_msg msg[2] = {
		{addr, flags, 2, buf0},
		{addr, flags | I2C_M_RD, 2, buf1},
	};

	buf0[0] = (reg & 0xff00) >> 8;
	buf0[1] = reg & 0xff;

	/* SYS_MCLK enable */
	gpio_audio_sgtl_clk(1);

	i2c_ret = i2c_transfer(client->adapter, msg, 2);
	if (i2c_ret < 0) {
		pr_err("%s: read reg error : reg=%x\n", __func__, reg);
		return 0;
	}

	if (codec->bias_level != SND_SOC_BIAS_ON &&
		codec->bias_level != SND_SOC_BIAS_PREPARE &&
		suspend != 1){
		/* SYS_MCLK disable */
		gpio_audio_sgtl_clk(0);
	}

	value = buf1[0] << 8 | buf1[1];

	pr_debug("r r:%02x,v:%04x\n", reg, value);
	return value;
}

static unsigned int sgtl5000_read(struct snd_soc_codec *codec, unsigned int reg)
{
	if ((reg == SGTL5000_CHIP_ID) ||
	    (reg == SGTL5000_CHIP_ADCDAC_CTRL) ||
	    (reg == SGTL5000_CHIP_ANA_STATUS) ||
	    (reg > SGTL5000_MAX_CACHED_REG))
		return sgtl5000_hw_read(codec, reg);
	else
		return sgtl5000_read_reg_cache(codec, reg);
}

static inline void sgtl5000_write_reg_cache(struct snd_soc_codec *codec,
					    u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	unsigned int offset = reg >> 1;
	if (offset < ARRAY_SIZE(sgtl5000_regs))
		cache[offset] = value;
}

static int sgtl5000_write(struct snd_soc_codec *codec, unsigned int reg,
			  unsigned int value)
{
	struct i2c_client *client = codec->control_data;
	u16 addr = client->addr;
	u16 flags = client->flags;
	u8 buf[4];
	int i2c_ret;
	struct i2c_msg msg = { addr, flags, 4, buf };

	sgtl5000_write_reg_cache(codec, reg, value);
	if (reg == SGTL5000_CHIP_DAC_VOL || 
		reg == SGTL5000_CHIP_LINE_OUT_VOL ||
		reg == SGTL5000_CHIP_ANA_HP_CTRL){
		volume_fixes(reg, &value);
	}
	pr_debug("w r:%02x,v:%04x\n", reg, value);
	buf[0] = (reg & 0xff00) >> 8;
	buf[1] = reg & 0xff;
	buf[2] = (value & 0xff00) >> 8;
	buf[3] = value & 0xff;

	/* SYS_MCLK enable */
	gpio_audio_sgtl_clk(1);

	i2c_ret = i2c_transfer(client->adapter, &msg, 1);
	if (i2c_ret < 0) {
		pr_err("%s: write reg error : R%02d = 0x%04x\n",
		       __func__, reg, value);
		return -EIO;
	}

	if (codec->bias_level != SND_SOC_BIAS_ON &&
		codec->bias_level != SND_SOC_BIAS_PREPARE &&
		suspend != 1){
		/* SYS_MCLK disable */
		gpio_audio_sgtl_clk(0);
	}

	return i2c_ret;
}

static void sgtl5000_sync_reg_cache(struct snd_soc_codec *codec)
{
	int reg;
	for (reg = 0; reg <= SGTL5000_MAX_CACHED_REG; reg += 2)
		sgtl5000_write_reg_cache(codec, reg,
					 sgtl5000_hw_read(codec, reg));
}

static int sgtl5000_restore_reg(struct snd_soc_codec *codec, unsigned int reg)
{
	unsigned int cached_val, hw_val;

	cached_val = sgtl5000_read_reg_cache(codec, reg);
	hw_val = sgtl5000_hw_read(codec, reg);

	if (hw_val != cached_val ||
		reg == SGTL5000_CHIP_DAC_VOL ||
		reg == SGTL5000_CHIP_LINE_OUT_VOL ||
		reg == SGTL5000_CHIP_ANA_HP_CTRL){
		return sgtl5000_write(codec, reg, cached_val);
	}

	return 0;
}

static int all_reg[] = {
	SGTL5000_CHIP_ID,
	SGTL5000_CHIP_DIG_POWER,
	SGTL5000_CHIP_CLK_CTRL,
	SGTL5000_CHIP_I2S_CTRL,
	SGTL5000_CHIP_SSS_CTRL,
	SGTL5000_CHIP_ADCDAC_CTRL,
	SGTL5000_CHIP_DAC_VOL,
	SGTL5000_CHIP_PAD_STRENGTH,
	SGTL5000_CHIP_ANA_ADC_CTRL,
	SGTL5000_CHIP_ANA_HP_CTRL,
	SGTL5000_CHIP_ANA_CTRL,
	SGTL5000_CHIP_LINREG_CTRL,
	SGTL5000_CHIP_REF_CTRL,
	SGTL5000_CHIP_MIC_CTRL,
	SGTL5000_CHIP_LINE_OUT_CTRL,
	SGTL5000_CHIP_LINE_OUT_VOL,
	SGTL5000_CHIP_ANA_POWER,
	SGTL5000_CHIP_PLL_CTRL,
	SGTL5000_CHIP_CLK_TOP_CTRL,
	SGTL5000_CHIP_ANA_STATUS,
	SGTL5000_CHIP_SHORT_CTRL,
	SGTL5000_CHIP_ANA_TEST2,
	SGTL5000_DAP_CTRL,
	SGTL5000_DAP_MAIN_CHAN,
	SGTL5000_DAP_MIX_CHAN,
};

#ifdef DEBUG
static void dump_reg(struct snd_soc_codec *codec)
{
	int i, reg;
	printk(KERN_DEBUG "dump begin\n");
	for (i = 0; i < 21; i++) {
		reg = sgtl5000_read(codec, all_reg[i]);
		printk(KERN_DEBUG "d r %04x, v %04x\n", all_reg[i], reg);
	}
	printk(KERN_DEBUG "dump end\n");
}
#else
static void dump_reg(struct snd_soc_codec *codec)
{
}
#endif

static int dac_mux_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = widget->codec;
	unsigned int reg;

	if (ucontrol->value.enumerated.item[0]) {
		reg = sgtl5000_read(codec, SGTL5000_CHIP_CLK_TOP_CTRL);
		reg |= SGTL5000_INT_OSC_EN;
		sgtl5000_write(codec, SGTL5000_CHIP_CLK_TOP_CTRL, reg);

		if (codec->bias_level != SND_SOC_BIAS_ON) {
			sgtl5000_set_bias_level(codec, SND_SOC_BIAS_PREPARE);
			snd_soc_dapm_put_enum_double(kcontrol, ucontrol);
			sgtl5000_set_bias_level(codec, SND_SOC_BIAS_ON);
		} else
			snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_CTRL);
		reg &= ~(SGTL5000_LINE_OUT_MUTE | SGTL5000_HP_MUTE);
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_CTRL, reg);
	} else {
		reg = sgtl5000_read(codec, SGTL5000_CHIP_CLK_TOP_CTRL);
		reg &= ~SGTL5000_INT_OSC_EN;
		sgtl5000_write(codec, SGTL5000_CHIP_CLK_TOP_CTRL, reg);

		snd_soc_dapm_put_enum_double(kcontrol, ucontrol);
		sgtl5000_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	}
	return 0;
}

static const char *adc_mux_text[] = {
	"MIC_IN", "LINE_IN"
};

static const char *dac_mux_text[] = {
	"DAC", "LINE_IN"
};

static const struct soc_enum adc_enum =
SOC_ENUM_SINGLE(SGTL5000_CHIP_ANA_CTRL, 2, 2, adc_mux_text);

static const struct soc_enum dac_enum =
SOC_ENUM_SINGLE(SGTL5000_CHIP_ANA_CTRL, 6, 2, dac_mux_text);

static const struct snd_kcontrol_new adc_mux =
SOC_DAPM_ENUM("ADC Mux", adc_enum);

static const struct snd_kcontrol_new dac_mux = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "DAC Mux",
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE
	    | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	.info = snd_soc_info_enum_double,
	.get = snd_soc_dapm_get_enum_double,
	.put = dac_mux_put,
	.private_value = (unsigned long)&dac_enum,
};

static const struct snd_soc_dapm_widget sgtl5000_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("LINE_IN"),
	SND_SOC_DAPM_INPUT("MIC_IN"),

	SND_SOC_DAPM_OUTPUT("HP_OUT"),
	SND_SOC_DAPM_OUTPUT("LINE_OUT"),

	SND_SOC_DAPM_PGA("HP", SGTL5000_CHIP_ANA_POWER, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LO", SGTL5000_CHIP_ANA_POWER, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("ADC Mux", SND_SOC_NOPM, 0, 0, &adc_mux),
	SND_SOC_DAPM_MUX("DAC Mux", SND_SOC_NOPM, 0, 0, &dac_mux),

	SND_SOC_DAPM_ADC("ADC", "Capture", SGTL5000_CHIP_DIG_POWER, 6, 0),
	SND_SOC_DAPM_DAC("DAC", "Playback", SGTL5000_CHIP_DIG_POWER, 5, 0),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"ADC Mux", "LINE_IN", "LINE_IN"},
	{"ADC Mux", "MIC_IN", "MIC_IN"},
	{"ADC", NULL, "ADC Mux"},
	{"DAC Mux", "DAC", "DAC"},
	{"DAC Mux", "LINE_IN", "LINE_IN"},
	{"LO", NULL, "DAC"},
	{"HP", NULL, "DAC Mux"},
	{"LINE_OUT", NULL, "LO"},
	{"HP_OUT", NULL, "HP"},
};

static int sgtl5000_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, sgtl5000_dapm_widgets,
				  ARRAY_SIZE(sgtl5000_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

#if 0 /* 2009/07/13 Interface of DAC is changed to a general interface by NSS  */

static int dac_info_volsw(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = DAC_VOL_MIN;
	uinfo->value.integer.max = DAC_VOL_MAX - DAC_VOL_LIMIT;
	return 0;
}

static int dac_get_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg, l, r;

	reg = sgtl5000_read(codec, SGTL5000_CHIP_DAC_VOL);
	l = (reg & SGTL5000_DAC_VOL_LEFT_MASK) >> SGTL5000_DAC_VOL_LEFT_SHIFT;
	r = (reg & SGTL5000_DAC_VOL_RIGHT_MASK) >> SGTL5000_DAC_VOL_RIGHT_SHIFT;
	l = l < DAC_VOL_LIMIT ? DAC_VOL_LIMIT : l;
	l = l > DAC_VOL_MAX ? DAC_VOL_MAX : l;
	r = r < DAC_VOL_LIMIT ? DAC_VOL_LIMIT : r;
	r = r > DAC_VOL_MAX ? DAC_VOL_MAX : r;
	l = DAC_VOL_MAX - l;
	r = DAC_VOL_MAX - r;

	ucontrol->value.integer.value[0] = l;
	ucontrol->value.integer.value[1] = r;

	return 0;
}

static int dac_put_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg, l, r;

	l = ucontrol->value.integer.value[0];
	r = ucontrol->value.integer.value[1];

	l = l < DAC_VOL_MIN ? DAC_VOL_MIN : l;
	l = l > DAC_VOL_MAX - DAC_VOL_LIMIT ? DAC_VOL_MAX - DAC_VOL_LIMIT : l;
	r = r < DAC_VOL_MIN ? DAC_VOL_MIN : r;
	r = r > DAC_VOL_MAX - DAC_VOL_LIMIT ? DAC_VOL_MAX - DAC_VOL_LIMIT : r;
	l = DAC_VOL_MAX - l;
	r = DAC_VOL_MAX - r;

	reg = l << SGTL5000_DAC_VOL_LEFT_SHIFT |
	    r << SGTL5000_DAC_VOL_RIGHT_SHIFT;

	sgtl5000_write(codec, SGTL5000_CHIP_DAC_VOL, reg);

	return 0;
}

#endif

static const char *mic_gain_text[] = {
	"0dB", "20dB", "30dB", "40dB"
};

static const char *adc_m6db_text[] = {
	"No Change", "Reduced by 6dB"
};

static const struct soc_enum mic_gain =
SOC_ENUM_SINGLE(SGTL5000_CHIP_MIC_CTRL, 0, 4, mic_gain_text);

static const struct soc_enum adc_m6db =
SOC_ENUM_SINGLE(SGTL5000_CHIP_ANA_ADC_CTRL, 8, 2, adc_m6db_text);

static const struct snd_kcontrol_new sgtl5000_snd_controls[] = {
	SOC_ENUM("MIC GAIN", mic_gain),
	SOC_DOUBLE("Capture Volume", SGTL5000_CHIP_ANA_ADC_CTRL, 0, 4, 0xf, 0),
	SOC_ENUM("Capture Vol Reduction", adc_m6db),

#if 0 /* 2009/07/13 Interface of DAC is changed to a general interface by NSS  */
	{.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = "Playback Volume",
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
	 SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = dac_info_volsw,
	 .get = dac_get_volsw,
	 .put = dac_put_volsw,
	 },
#else
	SOC_DOUBLE("Playback Volume", SGTL5000_CHIP_DAC_VOL, 0, 8, 
			DAC_VOL_MAX - DAC_VOL_LIMIT, 1),
#endif
#ifdef CONFIG_MACH_MX51_ERDOS
	SOC_DOUBLE("Speaker Volume", SGTL5000_CHIP_LINE_OUT_VOL, 0, 8, 
			SPK_VOL_MAX - SPK_VOL_LIMIT, 1),
	SOC_DOUBLE("Headphone Volume", SGTL5000_CHIP_ANA_HP_CTRL, 0, 8, 
			HP_VOL_MAX - HP_VOL_LIMIT, 1),
#else
	SOC_DOUBLE("Speaker Volume", SGTL5000_CHIP_LINE_OUT_VOL, 0, 8, 0x1f, 1),
	SOC_DOUBLE("Headphone Volume", SGTL5000_CHIP_ANA_HP_CTRL, 0, 8, 0x7f, 1),
#endif /* CONFIG_MACH_MX51_ERDOS */
};

static int sgtl5000_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(sgtl5000_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&sgtl5000_snd_controls[i],
					       codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

static int sgtl5000_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg1, reg2;

	reg1 = sgtl5000_read(codec, SGTL5000_CHIP_ANA_CTRL);
	reg2 = sgtl5000_read(codec, SGTL5000_CHIP_ADCDAC_CTRL);

	if (mute) {
		reg1 |= SGTL5000_LINE_OUT_MUTE;
		reg1 |= SGTL5000_HP_MUTE;
		reg2 |= SGTL5000_DAC_MUTE_LEFT;
		reg2 |= SGTL5000_DAC_MUTE_RIGHT;
	} else {
		reg1 &= ~SGTL5000_LINE_OUT_MUTE;
		reg1 &= ~SGTL5000_HP_MUTE;
		reg2 &= ~SGTL5000_DAC_MUTE_LEFT;
		reg2 &= ~SGTL5000_DAC_MUTE_RIGHT;
	}

	sgtl5000_write(codec, SGTL5000_CHIP_ANA_CTRL, reg1);
	sgtl5000_write(codec, SGTL5000_CHIP_ADCDAC_CTRL, reg2);
	if (!mute)
		dump_reg(codec);
	return 0;
}

void sgtl5000_dap_enable(int jack, int event)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdevice);
	struct snd_soc_codec *codec = socdev->codec;
	static int enable = 0;
	u16 reg;
	int i;

	/*
	 * check suspend(already complete resume)
	 */
	if (suspend != 0) {
		return;
	}

	hp_status = jack;

	/* Restore refs first in same order as in sgtl5000_init */
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_LINREG_CTRL);
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_ANA_POWER);
	msleep(10);
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_REF_CTRL);
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_LINE_OUT_CTRL);

	/* Restore everythine else */
	for (i = 1; i < sizeof(all_reg) / sizeof(int); i++)
		sgtl5000_restore_reg(codec, all_reg[i]);

	if (enable == event){
		return;
	}
	enable = event;

	if (enable == 0){
		/* Select data source I2S_IN for DAC */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_SSS_CTRL);
		reg &= ~SGTL5000_DAC_SEL_MASK;
		reg |= (SGTL5000_DAC_SEL_I2S_IN << SGTL5000_DAC_SEL_SHIFT);
		sgtl5000_write(codec, SGTL5000_CHIP_SSS_CTRL, reg);

		/* DAP power off */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_DIG_POWER);
		reg &= ~SGTL5000_DAP_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, reg);

		/* disable DAP */
		reg = 0;
		sgtl5000_write(codec, SGTL5000_DAP_CTRL, reg);
	} else {
		/* enable DAP */
		reg = sgtl5000_read(codec, SGTL5000_DAP_CTRL);
		reg = SGTL5000_DAP_EN
			| SGTL5000_DAP_MIX_EN;
		sgtl5000_write(codec, SGTL5000_DAP_CTRL, reg);

		/* DAP power on */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_DIG_POWER);
		reg |= SGTL5000_DAP_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, reg);

		/* Select data source DAP for DAC */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_SSS_CTRL);
		reg &= ~SGTL5000_DAC_SEL_MASK;
		reg |= (SGTL5000_DAC_SEL_DAP << SGTL5000_DAC_SEL_SHIFT);
		sgtl5000_write(codec, SGTL5000_CHIP_SSS_CTRL, reg);
	}

	return;
}
EXPORT_SYMBOL_GPL(sgtl5000_dap_enable);

static int sgtl5000_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sgtl5000_priv *sgtl5000 = codec->private_data;
	u16 i2sctl = 0;
	pr_debug("%s:fmt=%08x\n", __func__, fmt);
	sgtl5000->master = 0;
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		i2sctl |= SGTL5000_I2S_MASTER;
		sgtl5000->master = 1;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
		return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		i2sctl |= SGTL5000_I2S_MODE_PCM;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		i2sctl |= SGTL5000_I2S_MODE_PCM;
		i2sctl |= SGTL5000_I2S_LRALIGN;
		break;
	case SND_SOC_DAIFMT_I2S:
		i2sctl |= SGTL5000_I2S_MODE_I2S_LJ;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		i2sctl |= SGTL5000_I2S_MODE_RJ;
		i2sctl |= SGTL5000_I2S_LRPOL;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		i2sctl |= SGTL5000_I2S_MODE_I2S_LJ;
		i2sctl |= SGTL5000_I2S_LRALIGN;
		break;
	default:
		return -EINVAL;
	}
	sgtl5000->fmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	/* Clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
	case SND_SOC_DAIFMT_NB_IF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
	case SND_SOC_DAIFMT_IB_NF:
		i2sctl |= SGTL5000_I2S_SCLK_INV;
		break;
	default:
		return -EINVAL;
	}
	sgtl5000_write(codec, SGTL5000_CHIP_I2S_CTRL, i2sctl);

	return 0;
}

static int sgtl5000_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				   int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sgtl5000_priv *sgtl5000 = codec->private_data;

	switch (clk_id) {
	case SGTL5000_SYSCLK:
		sgtl5000->sysclk = freq;
		break;
	case SGTL5000_LRCLK:
		sgtl5000->lrclk = freq;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void sgtl5000_pcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int reg;

	reg = sgtl5000_read(codec, SGTL5000_CHIP_DIG_POWER);
	reg &= ~(SGTL5000_I2S_IN_POWERUP | SGTL5000_I2S_OUT_POWERUP);
	sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, reg);

	reg = sgtl5000_read(codec, SGTL5000_CHIP_I2S_CTRL);
	reg &= ~SGTL5000_I2S_MASTER;
	sgtl5000_write(codec, SGTL5000_CHIP_I2S_CTRL, reg);
}

/*
 * Set PCM DAI bit size and sample rate.
 * input: params_rate, params_fmt
 */
static int sgtl5000_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct sgtl5000_priv *sgtl5000 = codec->private_data;
	int channels = params_channels(params);
	int clk_ctl = 0;
	int pll_ctl = 0;
	int i2s_ctl;
	int div2 = 0;
	int reg;

	if (!sgtl5000->sysclk) {
		pr_err("%s: set sysclk first!\n", __func__);
		return -EFAULT;
	}

	/* rev 1 does not support mono playback */
	if (sgtl5000->rev != 0x00) {
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_TEST2);
		if (channels == 1)
			reg |= SGTL5000_MONO_DAC;
		else
			reg &= ~SGTL5000_MONO_DAC;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_TEST2, reg);
	}

	switch (sgtl5000->lrclk) {
	case 32000:
		clk_ctl |= SGTL5000_SYS_FS_32k << SGTL5000_SYS_FS_SHIFT;
		break;
	case 44100:
		clk_ctl |= SGTL5000_SYS_FS_44_1k << SGTL5000_SYS_FS_SHIFT;
		break;
	case 48000:
		clk_ctl |= SGTL5000_SYS_FS_48k << SGTL5000_SYS_FS_SHIFT;
		break;
	case 96000:
		clk_ctl |= SGTL5000_SYS_FS_96k << SGTL5000_SYS_FS_SHIFT;
		break;
	default:
		pr_err("%s: sample rate %d not supported\n", __func__,
		       sgtl5000->lrclk);
		return -EFAULT;
	}

#if 0	/* SGTL5000 rev1 has a IC bug to prevent switching to MCLK from PLL. */
	if (fs * 256 == sgtl5000->sysclk)
		clk_ctl |= SGTL5000_MCLK_FREQ_256FS << SGTL5000_MCLK_FREQ_SHIFT;
	else if (fs * 384 == sgtl5000->sysclk && fs != 96000)
		clk_ctl |= SGTL5000_MCLK_FREQ_384FS << SGTL5000_MCLK_FREQ_SHIFT;
	else if (fs * 512 == sgtl5000->sysclk && fs != 96000)
		clk_ctl |= SGTL5000_MCLK_FREQ_512FS << SGTL5000_MCLK_FREQ_SHIFT;
	else
#endif
	{
		if (!sgtl5000->master) {
			pr_err("%s: PLL not supported in slave mode\n",
			       __func__);
			return -EINVAL;
		}
		clk_ctl |= SGTL5000_MCLK_FREQ_PLL << SGTL5000_MCLK_FREQ_SHIFT;
	}

	if ((clk_ctl & SGTL5000_MCLK_FREQ_MASK) == SGTL5000_MCLK_FREQ_PLL) {
		u64 out, t;
		unsigned int in, int_div, frac_div;
		if (sgtl5000->sysclk > 17000000) {
			div2 = 1;
			in = sgtl5000->sysclk / 2;
		} else {
			div2 = 0;
			in = sgtl5000->sysclk;
		}
		if (sgtl5000->lrclk == 44100)
			out = 180633600;
		else
			out = 196608000;
		t = do_div(out, in);
		int_div = out;
		t *= 2048;
		do_div(t, in);
		frac_div = t;
		pll_ctl = int_div << SGTL5000_PLL_INT_DIV_SHIFT |
		    frac_div << SGTL5000_PLL_FRAC_DIV_SHIFT;
	}

	i2s_ctl = sgtl5000_read(codec, SGTL5000_CHIP_I2S_CTRL);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (sgtl5000->fmt == SND_SOC_DAIFMT_RIGHT_J)
			return -EINVAL;
		i2s_ctl |= SGTL5000_I2S_DLEN_16 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_32FS <<
		    SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		i2s_ctl |= SGTL5000_I2S_DLEN_20 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_64FS <<
		    SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		i2s_ctl |= SGTL5000_I2S_DLEN_24 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_64FS <<
		    SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		if (sgtl5000->fmt == SND_SOC_DAIFMT_RIGHT_J)
			return -EINVAL;
		i2s_ctl |= SGTL5000_I2S_DLEN_32 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_64FS <<
		    SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	pr_debug("fs=%d,clk_ctl=%d,pll_ctl=%d,i2s_ctl=%d,div2=%d\n",
		 sgtl5000->lrclk, clk_ctl, pll_ctl, i2s_ctl, div2);

	if ((clk_ctl & SGTL5000_MCLK_FREQ_MASK) == SGTL5000_MCLK_FREQ_PLL) {
		sgtl5000_write(codec, SGTL5000_CHIP_PLL_CTRL, pll_ctl);
		reg = sgtl5000_read(codec, SGTL5000_CHIP_CLK_TOP_CTRL);
		if (div2)
			reg |= SGTL5000_INPUT_FREQ_DIV2;
		else
			reg &= ~SGTL5000_INPUT_FREQ_DIV2;
		sgtl5000_write(codec, SGTL5000_CHIP_CLK_TOP_CTRL, reg);
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		reg |= SGTL5000_PLL_POWERUP | SGTL5000_VCOAMP_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
	}
	sgtl5000_write(codec, SGTL5000_CHIP_CLK_CTRL, clk_ctl);
	sgtl5000_write(codec, SGTL5000_CHIP_I2S_CTRL, i2s_ctl);
	reg = sgtl5000_read(codec, SGTL5000_CHIP_DIG_POWER);
	reg |= SGTL5000_I2S_IN_POWERUP | SGTL5000_I2S_OUT_POWERUP;
	sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, reg);
	return 0;
}

static int sgtl5000_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level)
{
	u16 reg, ana_pwr;
	pr_debug("dapm level %d\n", level);
	switch (level) {
	case SND_SOC_BIAS_ON:		/* full On */
		if (codec->bias_level == SND_SOC_BIAS_ON)
			break;
		codec->bias_level = level;

		/* SYS_MCLK enable */
		gpio_audio_sgtl_clk(1);

		/* must power up hp/line out before vag & dac to
		   avoid pops. */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		reg |= SGTL5000_VAG_POWERUP;
		reg |= SGTL5000_REFTOP_POWERUP;
		reg |= SGTL5000_DAC_POWERUP;
		reg |= SGTL5000_ADC_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
		msleep(400);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_MIC_CTRL);
		reg &= ~SGTL5000_BIAS_R_MASK;
		reg |= SGTL5000_BIAS_R_4k << SGTL5000_BIAS_R_SHIFT;
		sgtl5000_write(codec, SGTL5000_CHIP_MIC_CTRL, reg);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_CTRL);
		reg |= SGTL5000_HP_ZCD_EN;
		reg |= SGTL5000_ADC_ZCD_EN;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_CTRL, reg);
		break;

	case SND_SOC_BIAS_PREPARE:	/* partial On */
		if (codec->bias_level == SND_SOC_BIAS_PREPARE)
			break;
		codec->bias_level = level;

		/* SYS_MCLK enable */
		gpio_audio_sgtl_clk(1);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_MIC_CTRL);
		reg &= ~SGTL5000_BIAS_R_MASK;
		reg |= SGTL5000_BIAS_R_off;
		sgtl5000_write(codec, SGTL5000_CHIP_MIC_CTRL, reg);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_CTRL);
		reg &= ~SGTL5000_HP_ZCD_EN;
		reg &= ~SGTL5000_ADC_ZCD_EN;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_CTRL, reg);

		/* must power up hp/line out before vag & dac to
		   avoid pops. */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		reg &= ~SGTL5000_VAG_POWERUP;
		reg |= SGTL5000_REFTOP_POWERUP;
		reg |= SGTL5000_DAC_POWERUP;
		reg |= SGTL5000_ADC_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
		msleep(400);

		break;

	case SND_SOC_BIAS_STANDBY:	/* Off, with power */
		/* SYS_MCLK disable */
		gpio_audio_sgtl_clk(0);
		break;

	case SND_SOC_BIAS_OFF:	/* Off, without power */
		/* must power down hp/line out after vag & dac to
		   avoid pops. */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		ana_pwr = reg;
		reg &= ~SGTL5000_VAG_POWERUP;
		reg &= ~SGTL5000_REFTOP_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
		msleep(600);

		reg &= ~SGTL5000_HP_POWERUP;
		reg &= ~SGTL5000_LINE_OUT_POWERUP;
		reg &= ~SGTL5000_DAC_POWERUP;
		reg &= ~SGTL5000_ADC_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);

		/* save ANA POWER register value for resume */
		sgtl5000_write_reg_cache(codec, SGTL5000_CHIP_ANA_POWER,
					 ana_pwr);
		break;
	}
	codec->bias_level = level;
	return 0;
}

#define SGTL5000_RATES (SNDRV_PCM_RATE_32000 |\
		      SNDRV_PCM_RATE_44100 |\
		      SNDRV_PCM_RATE_48000 |\
		      SNDRV_PCM_RATE_96000)

#define SGTL5000_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE)

struct snd_soc_dai sgtl5000_dai = {
	.name = "SGTL5000",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = SGTL5000_RATES,
		     .formats = SGTL5000_FORMATS,
		     },
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = SGTL5000_RATES,
		    .formats = SGTL5000_FORMATS,
		    },
	.ops = {
		.shutdown = sgtl5000_pcm_shutdown,
		.hw_params = sgtl5000_pcm_hw_params,
		},
	.dai_ops = {
		    .digital_mute = sgtl5000_digital_mute,
		    .set_fmt = sgtl5000_set_dai_fmt,
		    .set_sysclk = sgtl5000_set_dai_sysclk}
};
EXPORT_SYMBOL_GPL(sgtl5000_dai);

static int sgtl5000_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	suspend = 1;

	sgtl5000_set_bias_level(codec, SND_SOC_BIAS_OFF);

	gpio_audio_sgtl_clk(1);

	return 0;
}

static int sgtl5000_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	unsigned int i;

	/* Restore refs first in same order as in sgtl5000_init */
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_LINREG_CTRL);
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_ANA_POWER);
	msleep(10);
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_REF_CTRL);
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_LINE_OUT_CTRL);

	/* Restore everythine else */
	for (i = 1; i < sizeof(all_reg) / sizeof(int); i++)
		sgtl5000_restore_reg(codec, all_reg[i]);

	sgtl5000_write(codec, SGTL5000_DAP_CTRL, 0);

	/* Bring the codec back up to standby first to minimise pop/clicks */
	sgtl5000_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	sgtl5000_set_bias_level(codec, codec->suspend_bias_level);

	suspend = 0;

	return 0;
}

/*
 * initialise the SGTL5000 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int sgtl5000_init(struct snd_soc_device *socdev)
{
	struct sgtl5000_platform_data *plat = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *client = codec->control_data;
	struct sgtl5000_priv *sgtl5000 = codec->private_data;
	u16 reg, ana_pwr, lreg_ctrl, ref_ctrl, lo_ctrl, short_ctrl, sss;
	int vag;
	unsigned int val;
	int ret = 0;

	val = sgtl5000_read(codec, SGTL5000_CHIP_ID);
	if (((val & SGTL5000_PARTID_MASK) >> SGTL5000_PARTID_SHIFT) !=
	    SGTL5000_PARTID_PART_ID) {
		pr_err("Device with ID register %x is not a SGTL5000\n", val);
		return -ENODEV;
	}

	sgtl5000->rev = (val & SGTL5000_REVID_MASK) >> SGTL5000_REVID_SHIFT;
	dev_info(&client->dev, "SGTL5000 revision %d\n", sgtl5000->rev);

	/* For Rev2 or higher, CODEC will copy left channel data to right.
	   For Rev1, set playback channels_min to 2. */
	if (sgtl5000->rev == 0x00)	/* if chip is rev 1 */
		sgtl5000_dai.playback.channels_min = 2;

	codec->name = "SGTL5000";
	codec->owner = THIS_MODULE;
	codec->read = sgtl5000_read_reg_cache;
	codec->write = sgtl5000_write;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = sgtl5000_set_bias_level;
	codec->dai = &sgtl5000_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(sgtl5000_regs);
	codec->reg_cache_step = 2;
	codec->reg_cache = (void *)&sgtl5000_regs;
	if (codec->reg_cache == NULL) {
		dev_err(&client->dev, "Failed to allocate register cache\n");
		return -ENOMEM;
	}

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(&client->dev, "failed to create pcms\n");
		return ret;
	}

	sgtl5000_write(codec, SGTL5000_CHIP_DAC_VOL, 0x0000);
	sgtl5000_write(codec, SGTL5000_CHIP_LINE_OUT_VOL, 0x0000);
	sgtl5000_write(codec, SGTL5000_CHIP_ANA_HP_CTRL, 0x0000);

	sgtl5000_sync_reg_cache(codec);

	/* reset value */
	ana_pwr = SGTL5000_DAC_STERO |
	    SGTL5000_LINREG_SIMPLE_POWERUP |
	    SGTL5000_STARTUP_POWERUP |
	    SGTL5000_ADC_STERO | SGTL5000_REFTOP_POWERUP;
	lreg_ctrl = 0;
	ref_ctrl = 0;
	lo_ctrl = 0;
	short_ctrl = 0;
	sss = (SGTL5000_DAC_SEL_I2S_IN << SGTL5000_DAC_SEL_SHIFT)
		| (SGTL5000_DAP_SEL_I2S_IN << SGTL5000_DAP_SEL_SHIFT)
		| (SGTL5000_DAP_MIX_SEL_I2S_IN << SGTL5000_DAP_MIX_SEL_SHIFT)
		| SGTL5000_DAP_MIX_LRSWAP;

	/* workaround for rev 0x11: use vddd linear regulator */
	if (!plat->vddd || (sgtl5000->rev >= 0x11)) {
		/* set VDDD to 1.2v */
		lreg_ctrl |= 0x8 << SGTL5000_LINREG_VDDD_SHIFT;
		/* power internal linear regulator */
		ana_pwr |= SGTL5000_LINEREG_D_POWERUP;
	} else {
		/* turn of startup power */
		ana_pwr &= ~SGTL5000_STARTUP_POWERUP;
		ana_pwr &= ~SGTL5000_LINREG_SIMPLE_POWERUP;
	}
	if (plat->vddio < 3100 && plat->vdda < 3100) {
		/* Enable VDDC charge pump */
		ana_pwr |= SGTL5000_VDDC_CHRGPMP_POWERUP;
	}
	if (plat->vddio >= 3100 && plat->vdda >= 3100) {
		/* VDDC use VDDIO rail */
		lreg_ctrl |= SGTL5000_VDDC_ASSN_OVRD;
		if (plat->vddio >= 3100)
			lreg_ctrl |= SGTL5000_VDDC_MAN_ASSN_VDDIO <<
			    SGTL5000_VDDC_MAN_ASSN_SHIFT;
	}
	/* If PLL is powered up (such as on power cycle) leave it on. */
	reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
	ana_pwr |= reg & (SGTL5000_PLL_POWERUP | SGTL5000_VCOAMP_POWERUP);

	/* set ADC/DAC ref voltage to vdda/2 */
	vag = plat->vdda / 2;
	if (vag <= SGTL5000_ANA_GND_BASE)
		vag = 0;
	else if (vag >= SGTL5000_ANA_GND_BASE + SGTL5000_ANA_GND_STP *
		 (SGTL5000_ANA_GND_MASK >> SGTL5000_ANA_GND_SHIFT))
		vag = SGTL5000_ANA_GND_MASK >> SGTL5000_ANA_GND_SHIFT;
	else
		vag = (vag - SGTL5000_ANA_GND_BASE) / SGTL5000_ANA_GND_STP;
	ref_ctrl |= vag << SGTL5000_ANA_GND_SHIFT;

	/* set line out ref voltage to vddio/2 */
	vag = plat->vddio / 2;
	if (vag <= SGTL5000_LINE_OUT_GND_BASE)
		vag = 0;
	else if (vag >= SGTL5000_LINE_OUT_GND_BASE + SGTL5000_LINE_OUT_GND_STP *
		 SGTL5000_LINE_OUT_GND_MAX)
		vag = SGTL5000_LINE_OUT_GND_MAX;
	else
		vag = (vag - SGTL5000_LINE_OUT_GND_BASE) /
		    SGTL5000_LINE_OUT_GND_STP;
	lo_ctrl |= vag << SGTL5000_LINE_OUT_GND_SHIFT;

	/* enable small pop */
	ref_ctrl |= SGTL5000_SMALL_POP;

	/* Controls the output bias current for the lineout */
	lo_ctrl |= (SGTL5000_LINE_OUT_CURRENT_360u << SGTL5000_LINE_OUT_CURRENT_SHIFT);

	/* set short detect */
	/* keep default */

	/* set routing */
	/* keep default, bypass DAP */

	sgtl5000_write(codec, SGTL5000_CHIP_LINREG_CTRL, lreg_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, ana_pwr);
	msleep(10);

	/* For rev 0x11, if vddd linear reg has been enabled, we have
	   to disable simple reg to get proper VDDD voltage.  */
	if ((ana_pwr & SGTL5000_LINEREG_D_POWERUP) && (sgtl5000->rev >= 0x11)) {
		ana_pwr &= ~SGTL5000_LINREG_SIMPLE_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, ana_pwr);
		msleep(10);
	}

	sgtl5000_write(codec, SGTL5000_CHIP_REF_CTRL, ref_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_LINE_OUT_CTRL, lo_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_SHORT_CTRL, short_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_SSS_CTRL, sss);
	sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, 0);

	reg = SGTL5000_DAC_VOL_RAMP_EN |
	    SGTL5000_DAC_MUTE_RIGHT | SGTL5000_DAC_MUTE_LEFT;
	sgtl5000_write(codec, SGTL5000_CHIP_ADCDAC_CTRL, reg);

	if (cpu_is_mx25())
		sgtl5000_write(codec, SGTL5000_CHIP_PAD_STRENGTH, 0x01df);
	else
	sgtl5000_write(codec, SGTL5000_CHIP_PAD_STRENGTH, 0x015f);

	reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_ADC_CTRL);
	reg &= ~SGTL5000_ADC_VOL_M6DB;
	reg &= ~(SGTL5000_ADC_VOL_LEFT_MASK | SGTL5000_ADC_VOL_RIGHT_MASK);
	reg |= (0xf << SGTL5000_ADC_VOL_LEFT_SHIFT)
	    | (0xf << SGTL5000_ADC_VOL_RIGHT_SHIFT);
	sgtl5000_write(codec, SGTL5000_CHIP_ANA_ADC_CTRL, reg);

	reg = SGTL5000_LINE_OUT_MUTE
		| SGTL5000_HP_MUTE
		| SGTL5000_ADC_MUTE;
	sgtl5000_write(codec, SGTL5000_CHIP_ANA_CTRL, reg);

	sgtl5000_write(codec, SGTL5000_CHIP_MIC_CTRL, 0);
	sgtl5000_write(codec, SGTL5000_CHIP_CLK_TOP_CTRL, 0);

	/* disable DAP */
	sgtl5000_write(codec, SGTL5000_DAP_CTRL, 0);
	/* initialize DAP */
	sgtl5000_write(codec, SGTL5000_DAP_MAIN_CHAN, 0x8000);
	sgtl5000_write(codec, SGTL5000_DAP_MIX_CHAN,  0x8000);

	sgtl5000_add_controls(codec);
	sgtl5000_add_widgets(codec);

	sgtl5000_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "sgtl5000: failed to register card\n");
		snd_soc_free_pcms(socdev);
		snd_soc_dapm_free(socdev);
		return ret;
	}

	return 0;
}

static struct snd_soc_device *sgtl5000_socdev;

static int sgtl5000_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = sgtl5000_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int ret;

	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = sgtl5000_init(socdev);
	if (ret < 0)
		dev_err(&i2c->dev, "Device initialisation failed\n");

	return ret;
}

static const struct i2c_device_id sgtl5000_id[] = {
	{"sgtl5000-i2c", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, sgtl5000_id);

static struct i2c_driver sgtl5000_i2c_driver = {
	.driver = {
		   .name = "sgtl5000-i2c",
		   .owner = THIS_MODULE,
		   },
	.probe = sgtl5000_i2c_probe,
	.id_table = sgtl5000_id,
};

static int sgtl5000_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct sgtl5000_priv *sgtl5000;
	int ret = 0;
	pdevice = pdev;

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	sgtl5000 = kzalloc(sizeof(struct sgtl5000_priv), GFP_KERNEL);
	if (sgtl5000 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = sgtl5000;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	sgtl5000_socdev = socdev;

	ret = i2c_add_driver(&sgtl5000_i2c_driver);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't add i2c driver\n");
		kfree(codec->private_data);
		kfree(codec);
	}

	return ret;
}

/* power down chip */
static int sgtl5000_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		sgtl5000_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	i2c_del_driver(&sgtl5000_i2c_driver);
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_sgtl5000 = {
	.probe = sgtl5000_probe,
	.remove = sgtl5000_remove,
	.suspend = sgtl5000_suspend,
	.resume = sgtl5000_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_sgtl5000);

MODULE_DESCRIPTION("ASoC SGTL5000 driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
