/* revert
 * linux/sound/soc/codecs/tlv320aic31xx.c
 *
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1   ASoC driver support    Mistral             14-04-2010
 *
 * Rev 0.2   Updated based Review Comments Mistral      29-06-2010
 *
 * Rev 0.3   Updated for Codec Family Compatibility     12-07-2010
 *
 * Rev 0.4   Ported to 2.6.35 kernel
 *
 * Rev 0.5   Updated the aic31xx_power_up(), aic31xx_power_down() and
 *           aic31xx_mute_codec() functions to enable and disable the ADC 
 *           related registers.
 */
 
/******************************************************************************
 * INCLUDE HEADER FILES
 *****************************************************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/regulator/consumer.h>
#include "tlv320aic31xx.h"

/* These are local compile flags used to enable/disable ADC and
 * DAC specific portions of the code-base. In the default
 * Operating condition, we need both of these below flags to 
 * to be defined. 
 */
#define ADC_EN
//#undef ADC_EN
#define DAC_EN
//#undef DAC_EN


/* The following flags are used to enable/disable the DAPM support 
 * for the driver 
 */

//#define DRIVER_DAPM_SUPPORT
#undef DRIVER_DAPM_SUPPORT


/******************************************************************************
 * GLOBAL VARIABLES
 *****************************************************************************/

static struct i2c_client *tlv320aic31xx_client;
struct regulator *audio_regulator;
static struct i2c_board_info tlv320aic31xx_hwmon_info = {
        I2C_BOARD_INFO("tlv320aic3110", 0x18),
};

/* Used to maintain the Register Access control*/
static u8 aic31xx_reg_ctl;

static int aic31xx_power_down(struct snd_soc_codec *codec);
static int aic31xx_power_up(struct snd_soc_codec *codec);
static int aic31xx_mute_codec (struct snd_soc_codec *codec, int mute);
static unsigned int aic31xx_read (struct snd_soc_codec *codec, unsigned int reg);

#define SOC_SINGLE_N(xname, xreg, xshift, xmax, xinvert)\
{\
    .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
    .info = n_control_info, .get = n_control_get,\
    .put = n_control_put, \
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
    .private_value =  ((unsigned long)&(struct soc_mixer_control)) \
    {.reg = xreg, .shift = xshift, .rshift = xshift, .max = xmax, \
    .invert = xinvert} }




#define SOC_SINGLE_AIC31XX(xname) {					\
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,			\
			.name = xname,					\
			.info = __new_control_info,			\
			.get = __new_control_get,			\
			.put = __new_control_put,			\
			.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,	\
			}

#define SOC_DOUBLE_R_AIC31XX(xname, reg_left, reg_right, xshift, xmax, xinvert) \
	{								\
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,			\
			.name = (xname),				\
			.info = snd_soc_info_volsw_2r,			\
			.get = snd_soc_get_volsw_2r_aic31xx,		\
			.put = snd_soc_put_volsw_2r_aic31xx,		\
			.private_value =				\
			(unsigned long) &(struct soc_mixer_control) {	\
			.reg = reg_left,				\
			.rreg = reg_right,				\
			.shift = xshift,				\
			.max = 	xmax,					\
			.invert = xinvert				\
			}						\
	}

#if 1
/*
 *----------------------------------------------------------------------------
 * Function : n_control_info
 * Purpose  : This function is to initialize data for new control required to
 *            program the AIC3110 registers.
 *           
 *----------------------------------------------------------------------------
 */
static unsigned int n_control_info(struct snd_kcontrol *kcontrol,
                  struct snd_ctl_elem_info *uinfo)
{
    struct soc_mixer_control *mc =
        (struct soc_mixer_control *)kcontrol->private_value;
    int max = mc->max;
    unsigned int shift = mc->shift;
    unsigned int rshift = mc->rshift;

    if (max == 1)
        uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    else
        uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

    uinfo->count = shift == rshift ? 1 : 2;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = max;
    return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : n_control_get
 * Purpose  : This function is to read data of new control for
 *            program the AIC3110 registers.
 *           
 *----------------------------------------------------------------------------
 */
static unsigned int n_control_get(struct snd_kcontrol *kcontrol,
                 struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    u32 val, reg;
    unsigned short mask, shift;   
    struct soc_mixer_control *mc =
        (struct soc_mixer_control *)kcontrol->private_value;
    reg = mc->reg;
    if (!strcmp(kcontrol->id.name, "ADC COARSE GAIN")) {
        mask = 0xFF;
        shift = 0;
        val = aic31xx_read(codec,reg);
        ucontrol->value.integer.value[0] =
            (val >=40) ? (val - 40) : (val);
    }
      
    return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put
 * Purpose  : new_control_put is called to pass data from user/application to
 *            the driver.
 *
 *----------------------------------------------------------------------------
 */
static unsigned int n_control_put(struct snd_kcontrol *kcontrol,
                 struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    struct soc_mixer_control *mc =
        (struct soc_mixer_control *)kcontrol->private_value;
    u8 val, val_mask;
    int reg,err;
    unsigned int invert = mc->invert;
    int max = mc->max;
    DBG("n_control_put\n");
    reg = mc->reg;
    val = ucontrol->value.integer.value[0];
    if(invert) {
        val = max - val;
    }
    if (!strcmp(kcontrol->id.name, "ADC COARSE GAIN")) {
        val = (val >= 0) ? (val +40) : (40);
        val_mask = 0xFF;   
    }
    if ((err = snd_soc_update_bits_locked(codec, reg, val_mask, val)) < 0) {
        printk("Error while updating bits\n");
        return err;
    }
   
    return 0;
}

#endif

/* 
 * Global Var aic31xx_reg
 * 
 * Used to maintain a cache of Page 0 and 1 Register values.
 */
#if defined(AIC3110_CODEC_SUPPORT)
static const u8 aic31xx_reg[AIC31xx_CACHEREGNUM] = {
	/* Page 0 Registers */
	0x00,
	0x00, 0x12, 0x00, 0x00, 0x11, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0x01, 0x01, 0x80, 0x00, 0x00,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x55,
	0x55, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x14, 0x0c, 0x00, 0x00,
	0x00, 0x6f, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0xee, 0x10, 0xd8,
	0x7e, 0xe3, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x10, 0x32, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x12, 0x02,
	/* Page 1 Registers */
	0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#elif defined(AIC3100_CODEC_SUPPORT)
static const u8 aic31xx_reg[AIC31XX_CACHEREGNUM] = {	/* Page 0
							 HPL_DRIVER  Registers */
	0x00, 0x00, 0x12, 0x00, 0x00, 0x11, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0x01, 0x01, 0x80, 0x00, 0x00,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x55,
	0x55, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x14, 0x0c, 0x00, 0x00,
	0x00, 0x6f, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0xee, 0x10, 0xd8,
	0x7e, 0xe3, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x10, 0x32, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x12, 0x02,
	/* Page 1 Registers */
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#endif

/*
 * Function : snd_soc_get_volsw_2r_aic31xx
 *
 * Purpose : Callback to get the value of a double mixer control that spans
 * two registers.
 */
int snd_soc_get_volsw_2r_aic31xx (struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = mc->reg;
	int reg2 = mc->rreg;
	int mask;
	int shift;
	unsigned short val, val2;

	printk (KERN_INFO "##snd_soc_get_volsw_2r_aic31xx (%s)\n", kcontrol->id.name);

	if (strcmp(kcontrol->id.name, "DAC Playback Volume") == 0) {
		mask = 0xFF;
		shift = 0;
	} else if (strcmp(kcontrol->id.name, "ADC Capture Volume") == 0) {
		mask = 0x7F;
		shift = 0;
	} else {
		printk("Invalid kcontrol name\n");
		return -1;
	}
        printk (KERN_INFO "##REG %d %d Mask %x SHIFT %x\n", reg, reg2, mask, shift);

	val = (snd_soc_read(codec, reg) >> shift) & mask;
	val2 = (snd_soc_read(codec, reg2) >> shift) & mask;

	if (strcmp(kcontrol->id.name, "DAC Playback Volume") == 0) {
		ucontrol->value.integer.value[0] = (val <= 48) ? (val + 127) :
			(val - 129);
		ucontrol->value.integer.value[1] = (val2 <= 48) ?
			(val2 + 127) : (val2 - 129);
	} else if (strcmp(kcontrol->id.name, "ADC Capture Volume") == 0) {
		ucontrol->value.integer.value[0] = (val <= 38) ? (val + 25) :
			(val - 103);
		ucontrol->value.integer.value[1] = (val2 <= 38) ?
			(val2 + 25) : (val2 - 103);
	}

	return 0;
}

/*
 * snd_soc_put_volsw_2r_aic31xx
 *
 * Callback to set the value of a double mixer control that spans two
 * registers.
 */
int snd_soc_put_volsw_2r_aic31xx (struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = mc->reg;
	int reg2 = mc->rreg;
	int err;
	unsigned short val, val2, val_mask;

	DBG("snd_soc_put_volsw_2r_aic31xx (%s)\n", kcontrol->id.name);

	val = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

	if (strcmp(kcontrol->id.name, "DAC Playback Volume") == 0) {
		val = (val >= 127) ? (val - 127) : (val + 129);
		val2 = (val2 >= 127) ? (val2 - 127) : (val2 + 129);
		val_mask = 0xFF;	/* 8
					   bits */
	} else if (strcmp(kcontrol->id.name, "ADC Capture Volume") == 0) {
		val = (val >= 25) ? (val - 25) : (val + 103);
		val2 = (val2 >= 25) ? (val2 - 25) : (val2 + 103);
		val_mask = 0x7F;	/* 7 bits */
	} else {
		printk("Invalid control name\n");
		return -1;
	}

	if ((err = snd_soc_update_bits(codec, reg, val_mask, val)) < 0) {
		printk("Error while updating bits\n");
		return err;
	}

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
	return err;
}

/*
 * aic31xx_change_page
 *
 * This function is to switch between page 0 and page 1.
 */
int aic31xx_change_page (struct snd_soc_codec *codec, u8 new_page)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 data[2];

	if (aic31xx == NULL) {
		printk("Codec Private member NULL..\n");
	}
        data[0] = 0;
	data[1] = new_page;

	printk (KERN_INFO "### Changing to Page %d Codec Priv structure %x\n", 
			new_page, (unsigned int) aic31xx);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk("Error in changing page to 1\n");
		return -1;
	}
	aic31xx->page_no = new_page;
	return 0;
}

/*
 * aic31xx_write_reg_cache
 * This function is to write aic31xx register cache
 */
static inline void aic31xx_write_reg_cache (struct snd_soc_codec
					    *codec, u16 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= AIC31xx_CACHEREGNUM) {
		return;
	}
	cache[reg] = value;
}

/*
 * aic31xx_write
 *
 * This function is to write to the aic31xx register space.
 */
static int aic31xx_write (struct snd_soc_codec *codec, unsigned int reg,
			  unsigned int value)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 data[2];
	u8 page;

	page = reg / 128;
	data[AIC31XX_REG_OFFSET_INDEX] = reg % 128;

	if (aic31xx->page_no != page) {
		aic31xx_change_page(codec, page);
	}

	/* data is * D15..D8 aic31xx register offset * D7...D0
	   register data */
	data[AIC31XX_REG_DATA_INDEX] = value & AIC31XX_8BITS_MASK;

/*	printk (KERN_INFO "w 18 %02x %02x CntrlData %x\n", 
                data[AIC31XX_REG_OFFSET_INDEX], value,
		codec->control_data ); */
#if defined(EN_REG_CACHE)
	if ((page == 0) || (page == 1) || (page == 2)) {
		aic31xx_write_reg_cache(codec, reg, value);
	}
#endif
	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk("Error in i2c write\n");
		return -EIO;
	}
	return 0;
}

/*
 * aic31xx_read
 *
 * This function is to read the aic31xx register space.
 */
static unsigned int aic31xx_read (struct snd_soc_codec *codec, unsigned int reg)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 value;
	u8 page = reg / 128;

	reg = reg % 128;

	if (aic31xx->page_no != page) {
		aic31xx_change_page(codec, page);
	}

	i2c_master_send(codec->control_data, (char *)&reg, 1);
	i2c_master_recv(codec->control_data, &value, 1);

	/* printk (KERN_INFO "r 18 %02x %02x\n", reg, value); */

	return value;
}


/*
 *----------------------------------------------------------------------------
 * Function : debug_print_registers
 * Purpose  : Debug routine to dump all the Registers of Page 0
 *
 *----------------------------------------------------------------------------
 */
void debug_print_registers (struct snd_soc_codec *codec)
{
	int i;
	u8 data;

    printk (KERN_INFO "### Page 0 Regs from 0 to 95\n");

	for (i = 0; i < 95; i++) {
		data = (u8) aic31xx_read(codec, i);
		printk(KERN_INFO "reg = %d val = %x\n", i, data);
	}
        printk (KERN_INFO "### Page 1 Regs from 30 to 52\n");

	for (i = 158; i < 180; i++) {
		data = (u8) aic31xx_read(codec, i);
		printk(KERN_INFO "reg = %d val = %x\n", (i%128), data);
	} 
}

/*
 *----------------------------------------------------------------------------
 * Function : aic31xx_headset_speaker_path
 * Purpose  : This function is to check for the presence of Headset and
 *            configure the Headphone of the Class D Speaker driver
 *            Registers appropriately.
 *
 *----------------------------------------------------------------------------
 */
int aic31xx_headset_speaker_path (struct snd_soc_codec *codec, int gpio_status)

{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 headset_new_status;
	u8 interrupt_status1;

	/* Read the status of the Headset Detection Register */
	headset_new_status = aic31xx_read (codec, HEADSET_DETECT);

	/* Read the Interrupt Status Registers of Page 0 46 */
    interrupt_status1 = aic31xx_read (codec, DAC_INTR_STATUS);
    interrupt_status1 = aic31xx_read (codec, DAC_INTR_STATUS);

    printk(KERN_INFO "Headset Status: Prev 0x%x Current 0x%x IntrFlags 0x%x GPIO 0x%x\n", 
	aic31xx->headset_current_status, headset_new_status,
                interrupt_status1, gpio_status);
	
	/*if (headset_new_status != aic31xx->headset_current_status) {*/
	if (headset_new_status != gpio_status) {
	
		aic31xx->headset_connected = headset_new_status & BIT5;
                aic31xx->headset_connected = gpio_status; 
	    
		if (aic31xx->playback_status == 1) {
		
     		/* If codec was not powered up, power up the same. */
			if(aic31xx->headset_connected) {
				printk (KERN_INFO "headset inserted and headset path Activated\n");
	                        aic31xx_write(codec, HEADPHONE_DRIVER, HP_DRIVER_ON); //  ON
	                        aic31xx_write(codec, CLASSD_SPEAKER_AMP, SPK_DRV_OFF); // OFF
				} 
			else {
				printk (KERN_INFO "headset removed and headset path "
	                               "Deactivated\n");
	                        aic31xx_write(codec, HEADPHONE_DRIVER ,HP_DRIVER_OFF); // OFF
	                        aic31xx_write(codec, CLASSD_SPEAKER_AMP ,SPK_DRV_ON ); //ON
	     }
	                /* We will force the aic31xx->mute to 1 to ensure that the
	                 * following function executes completely.
	                 */
	     aic31xx->mute = 1;
	     
		 /* Now unmute the appropriate Codec sections with Volume Ramping */
	     aic31xx_mute_codec (codec, 0);
	    }
       else{
           printk(KERN_INFO "NO Active Playback Record request...\n");
		   }
	}
	else {
		printk (KERN_INFO "aic31x_headset_speaker_path: No updates from previous status...\n");
	}
	/* Update the current Status Flag */
	aic31xx->headset_current_status = headset_new_status;
  return 0;
}


/*
 * __new_control_info
 *
 * This function is to initialize data for new control required to * program the
 * AIC31xx registers.
 */
static int __new_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 65535;

	return 0;
}

/*
 * __new_control_get
 *
 * This function is to read data of new control for program the AIC31xx
 * registers.
 */
static int __new_control_get(struct snd_kcontrol *kcontrol, struct
			     snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val;

	val = aic31xx_read(codec, aic31xx_reg_ctl);
	ucontrol->value.integer.value[0] = val;

	return 0;
}

/*
 * __new_control_put
 *
 * __new_control_put is called to pass data from user/application to the
 * driver.
 */
static int __new_control_put (struct snd_kcontrol *kcontrol, struct
			      snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	u32 data_from_user = ucontrol->value.integer.value[0];
	u8 data[2];

	aic31xx_reg_ctl = data[0] = (u8) ((data_from_user & 0xFF00) >> 8);
	data[1] = (u8) ((data_from_user & 0x00FF));

	if (!data[0]) {
		aic31xx->page_no = data[1];
	}

	printk("reg = %d val = %x\n", data[0], data[1]);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk("Error in i2c write\n");
		return -EIO;
	}

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic31xx_power_up
 * Purpose  : This function powers up the codec.
 *
 *----------------------------------------------------------------------------
 */
static int aic31xx_power_up (struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 value;
	u8 counter;

	printk (KERN_INFO "##++ aic31xx_power_up aic31xx->master=%d, aic31xx->power_status=%d, "
            "aic31xx->headset_connected=%d\n",
            aic31xx->master, aic31xx->power_status, aic31xx->headset_connected);

	printk (KERN_INFO "##aic31xx power up .................\n");
	/* all power is driven by DAPM system */
	if (aic31xx->master && (aic31xx->power_status != 1) ) {

#ifdef DAC_EN
      
          /* Switch ON Left and Right DACs */
		value = aic31xx_read (codec, DAC_CHN_REG);
		aic31xx_write (codec, DAC_CHN_REG, (value | ENABLE_DAC_CHN));

        /* Check for the DAC FLAG register to know if the DAC is
           really powered up */
        counter = 0;
        do {
                mdelay(10);
                value = aic31xx_read (codec, DAC_FLAG_1);
                counter++;
                printk (KERN_INFO "##DACEn Poll\r\n");
        } while ((counter < 20) && ((value & 0x88) == 0));
#endif
            printk (KERN_INFO "##-- aic31xx_power_up\n");

		/* Check whether the Headset or Speaker Driver needs Power Up */
		if (aic31xx->headset_connected) {
                        /* It is observed that turning ON Speaker, helps reduce
                           the pop up noise */
                        /* Switch ON the Class_D Speaker Amplifier */
                        value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
                        aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value | 0x80));

                        /* Switch ON Left and Right Headphone Drivers */
                        value = aic31xx_read (codec, HEADPHONE_DRIVER);
                        aic31xx_write (codec, HEADPHONE_DRIVER, (value | 0xC0)); 

                        /* Check for the DAC FLAG Register to know if the Left
                           Output Driver is powered up */
                        counter = 0;
                        do {
                                mdelay (10);
                                value = aic31xx_read (codec, DAC_FLAG_1);
                                counter++;
                                printk (KERN_INFO "##HPL Poll..\n");
                        } while ((value & 0x22) == 0);
                        printk (KERN_INFO "##HPL Power up Iterations %d\r\n", counter);

                        printk (KERN_INFO "##HPR Power Up Iterations %d\n", counter);

		} else {
                        /* Left Analog Speaker Volume update */
                        aic31xx_write (codec, L_ANLOG_VOL_2_SPL, 0x80);

                        aic31xx_write (codec, R_ANLOG_VOL_2_SPR, 0x80);

                        /* Switch ON the Class_D Speaker Amplifier */
                        value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
                        aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value | 0xC0));
                        mdelay(50);
                }

#ifdef ADC_EN	
	/* ADC Channel Power Up */
		value = aic31xx_read (codec, ADC_DIG_MIC);
		aic31xx_write (codec, ADC_DIG_MIC, (value | 0x80));

        /* Check for the ADC FLAG register to know if the ADC is
           really powered up */
        counter = 0;
        do {
                mdelay(10);
                value = aic31xx_read (codec, ADC_FLAG);
                counter++;
                printk (KERN_INFO "##ADC En Poll\r\n");
        } while ((counter < 40) && ((value & 0x40) == 0));

		printk (KERN_INFO "##-- aic31xx_power_up\n");

    /* Power Up control of MICBIAS */

	value = aic31xx_read (codec, MICBIAS_CTRL);
	aic31xx_write (codec, MICBIAS_CTRL, (value | (BIT1 | BIT0)));

#endif
	aic31xx->power_status = 1;

	}

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic31xx_power_down
 * Purpose  : This function powers down the codec.
 *
 *----------------------------------------------------------------------------
 */
static int aic31xx_power_down (struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	volatile u8 value;
        volatile u32 counter;

	printk (KERN_INFO "##++ aic31xx_power_down aic31xx->master=%d\n", aic31xx->master);

	if (aic31xx->master && (aic31xx->power_status != 0)) {

		/* Check whether the Headset or Speaker Driver needs Power Down */
		if(aic31xx->headset_connected) {

                        /* Switch off the Head phone Drivers */
                        value = aic31xx_read (codec, HEADPHONE_DRIVER);
                        aic31xx_write (codec, HEADPHONE_DRIVER, (value & ~0xC0)); /* 0xCC */

                        /* Now first check if the HPR is fully powered down */
                        counter = 0;
                        do {
                                mdelay(5);
                                value = aic31xx_read (codec, DAC_FLAG_1);
                                counter++;
                        }while ((counter < 100) && ((value & 0x22) != 0));
                        printk (KERN_INFO "##HPHONE RIGHT DRIVER Power Down. counter %d\r\n",
                             counter);
                        printk (KERN_INFO "##HPHONE LEFT DRIVER Power Down counter %d \r\n",
                            counter);

                        value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
                        aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value & ~0x80));

		} else {
			/* Switch OFF the Class_D Speaker Amplifier */
			value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
			aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value & ~0x80));

                        /* Left and Right Speaker Analog Volume is muted */
                        aic31xx_write (codec, L_ANLOG_VOL_2_SPL, 0x7F);

                        aic31xx_write (codec, R_ANLOG_VOL_2_SPR, 0x7F);
                        
	        }

#ifdef DAC_EN
                /* Switch OFF Left and Right DACs */

		value = aic31xx_read (codec, DAC_CHN_REG);
		aic31xx_write (codec, DAC_CHN_REG, (value & ~ENABLE_DAC_CHN));

                /* Check for the DAC FLAG register to know if the DAC is really
                   powered down */
                counter = 0;
                do {
                        mdelay(10);
                        value = aic31xx_read (codec, DAC_FLAG_1);
                        counter++;
                } while ((counter < 100) && ((value & 0x88) != 0));
                printk (KERN_INFO "##Left and Right DAC off Counter %d\r\n", counter);
#endif
#ifdef ADC_EN
		/* ADC Channel Power down */
		value = aic31xx_read (codec, ADC_DIG_MIC);
		aic31xx_write (codec, ADC_DIG_MIC, (value & ~BIT7));

        /* Check for the ADC FLAG register to know if the ADC is
           really powered down */
        counter = 0;
        do {
                mdelay(10);
                value = aic31xx_read (codec, ADC_FLAG);
                counter++;
                printk (KERN_INFO "##ADC En Poll\r\n");
        } while ((counter < 50) && ((value & 0x40) != 0));

            printk (KERN_INFO "##-- aic31xx_power_down\n");

    /* Power down control of MICBIAS */

		value = aic31xx_read (codec, MICBIAS_CTRL);
		aic31xx_write (codec, MICBIAS_CTRL, (value & ~(BIT1 | BIT0)));
#endif

 		aic31xx->power_status = 0;
	}

	printk (KERN_INFO "##-- aic31xx_power_down\n");

	return 0;
}




/*
 * aic31xx_set_bias_level
 *
 * This function is to get triggered when dapm events occurs.
 */
static int aic31xx_set_bias_level (struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 value;
 
        if (level == codec->dapm->bias_level) {
		printk(KERN_INFO "##aic31xx_set_bias_level. Current and previous levels same...\n");
	        return 0;
        }      

	printk (KERN_INFO "###aic31xx_set_bias_level New Level %d\n", level);

	switch (level) {	/* full On */
	case SND_SOC_BIAS_ON:	/* all power is driven by DAPM system */
                printk(KERN_INFO "###aic31xx_set_bias_level BIAS_ON\n");
		aic31xx_power_up (codec);
		break;

	case SND_SOC_BIAS_PREPARE:     /* partial On */
                printk(KERN_INFO "###aic31xx_set_bias_level BIAS_PREPARE\n");
		if (aic31xx->master) {	/* Switch on PLL */
			value = aic31xx_read(codec, CLK_REG_2);
			aic31xx_write(codec, CLK_REG_2, (value | ENABLE_PLL));

			/* Switch on NDAC Divider */
			value = aic31xx_read(codec, NDAC_CLK_REG);
			aic31xx_write(codec, NDAC_CLK_REG, value | ENABLE_NDAC);

			/* Switch on MDAC Divider */
			value = aic31xx_read(codec, MDAC_CLK_REG);
			aic31xx_write(codec, MDAC_CLK_REG, value | ENABLE_MDAC);

			/* Switch on NADC Divider */
			value =	aic31xx_read(codec, NADC_CLK_REG);
			aic31xx_write(codec, NADC_CLK_REG, value | ENABLE_MDAC);

			/* Switch on MADC Divider */
			value =	aic31xx_read(codec, MADC_CLK_REG);
			aic31xx_write(codec, MADC_CLK_REG, value | ENABLE_MDAC);

			/* Switch on BCLK_N Divider */
			value =	aic31xx_read(codec, BCLK_N_VAL);
			aic31xx_write(codec, BCLK_N_VAL, value | ENABLE_BCLK);
		} else {	/* Switch on PLL */
			value = aic31xx_read(codec, CLK_REG_2);
			aic31xx_write(codec, CLK_REG_2, (value | ENABLE_PLL));

			/* Switch on NDAC Divider */
			value = aic31xx_read(codec, NDAC_CLK_REG);
			aic31xx_write(codec, NDAC_CLK_REG, value | ENABLE_NDAC);

			/* Switch on MDAC Divider */
			value =	aic31xx_read(codec, MDAC_CLK_REG);
			aic31xx_write(codec, MDAC_CLK_REG, value | ENABLE_MDAC);

			/* Switch on NADC Divider */
			value = aic31xx_read(codec, NADC_CLK_REG);
			aic31xx_write(codec, NADC_CLK_REG, value | ENABLE_MDAC);

			/* Switch on MADC Divider */
			value =	aic31xx_read(codec, MADC_CLK_REG);
			aic31xx_write(codec, MADC_CLK_REG, value | ENABLE_MDAC);

			/* Switch on BCLK_N Divider */
			value =	aic31xx_read(codec, BCLK_N_VAL);
			aic31xx_write(codec, BCLK_N_VAL, value | ENABLE_BCLK);
		}

		break;

		/* Off, with power */
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
                printk(KERN_INFO "###aic31xx_set_bias_level BIAS_STANDBY or OFF\n");

		aic31xx_power_down (codec);
		/*
		 * all power is driven by DAPM system, so output power is safe
		 * if bypass was set
		 */
		if (aic31xx->master) {	/* Switch off PLL */
			value = aic31xx_read(codec, CLK_REG_2);
			aic31xx_write(codec, CLK_REG_2, (value & ~ENABLE_PLL));

			/* Switch off NDAC Divider */
			value = aic31xx_read(codec, NDAC_CLK_REG);
			aic31xx_write(codec, NDAC_CLK_REG, value &
				      ~ENABLE_NDAC);

			/* Switch off MDAC Divider */
			value = aic31xx_read(codec, MDAC_CLK_REG);
			aic31xx_write(codec, MDAC_CLK_REG, value &
				      ~ENABLE_MDAC);

			/* Switch off NADC Divider */
			value = aic31xx_read(codec, NADC_CLK_REG);
			aic31xx_write(codec, NADC_CLK_REG, value &
				      ~ENABLE_NDAC);

			/* Switch off MADC Divider */
			value = aic31xx_read(codec, MADC_CLK_REG);
			aic31xx_write(codec, MADC_CLK_REG, value &
				      ~ENABLE_MDAC);
			value = aic31xx_read(codec, BCLK_N_VAL);

			/* Switch off BCLK_N Divider */
			aic31xx_write(codec, BCLK_N_VAL, value & ~ENABLE_BCLK);
		}
		break;
	}
	codec->dapm->bias_level = level;
	return 0;
}


/* The updated aic31xx_divs Array for the KCI board having 19.2 Mhz
 * Master Clock Input coming from the FSREF2_CLK pin of OMAP4 
 */
static const struct aic31xx_rate_divs aic31xx_divs[] = {
	/*
	 * mclk, rate, p_val, pll_j, pll_d, dosr, ndac, mdac, aosr, nadc, madc,
	 * blck_N, codec_speficic_initializations
	 */
	/* 8k rate */
	{19200000, 8000, 1, 5, 1200, 256, 24, 2, 128, 48, 2, 8},
	{19200000, 8000, 1, 5, 1200, 256, 24, 2, 0, 24, 2, 8},
	/* 11.025k rate */
	{19200000, 11025, 1, 4, 4100, 256, 15, 2, 128, 30, 2, 8},
	{19200000, 11025, 1, 4, 4100, 256, 15, 2, 0, 15, 2, 8},
	/* 12K rate */
	{19200000, 12000, 1, 4, 8000, 256, 15, 2, 128, 30, 2, 8},
	{19200000, 12000, 1, 4, 8000, 256, 15, 2, 0, 15, 2, 8},
	/* 16k rate */
	{19200000, 16000, 1, 5, 1200, 256, 12, 2, 128, 24, 2, 8},
	{19200000, 16000, 1, 5, 1200, 256, 12, 2, 0, 12, 2, 8},
	/* 22.05k rate */
	{19200000, 22050, 1, 4, 7040, 256, 8, 2, 128, 16, 2, 8},
	{19200000, 22050, 1, 4, 7040, 256, 8, 2, 0, 8, 2, 8},
	/* 24k rate */
	{19200000, 24000, 1, 5, 1200, 256, 8, 2, 128, 16, 2, 8},
	{19200000, 24000, 1, 5, 1200, 256, 8, 2, 0, 8, 2, 8},
	/* 32k rate */
	{19200000, 32000, 1, 5, 1200, 256, 6, 2, 128, 12, 2, 8},
	{19200000, 32000, 1, 5, 1200, 256, 6, 2, 0, 6, 2, 8},
	/* 44.1k rate */
	{19200000, 44100, 1, 4, 7040, 256, 4, 2, 128, 8, 2, 8},
	{19200000, 44100, 1, 4, 7040, 256, 4, 2, 0, 4, 2, 8},
	/* 48k rate */
	{19200000, 48000, 1, 5, 1200, 256, 4, 2, 128, 8, 2, 8},
	{19200000, 48000, 1, 5, 1200, 256, 4, 2, 0, 4, 2, 8},
	/*96k rate */
	{19200000, 96000, 1, 5, 1200, 256, 2, 2, 128, 4, 2, 8},
	{19200000, 96000, 1, 5, 1200, 256, 2, 2, 0, 2, 2, 8},
	/*192k */
	{19200000, 192000, 1, 5, 1200, 256, 2, 1, 128, 4, 1, 16},
	{19200000, 192000, 1, 5, 1200, 256, 2, 1, 0, 2, 1, 16},
};


/*
 * aic31xx_get_divs
 *
 * This function is to get required divisor from the "aic31xx_divs" table.
 */
static inline int aic31xx_get_divs (int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(aic31xx_divs); i++) {
		if ((aic31xx_divs[i].rate == rate) &&
		    (aic31xx_divs[i].mclk == mclk)) {
			return i;
		}
	}
	printk("Master clock and sample rate is not supported\n");
	return -EINVAL;
}

/**
 * aic31xx_hw_params
 *
 * This function is to set the hardware parameters for AIC31xx.
 *  The functions set the sample rate and audio serial data word length.
 */
static int aic31xx_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int i;
	u8 data;

        aic31xx_power_down (codec);
        codec->dapm->bias_level = 2;

	printk (KERN_INFO "###aic31xx_hw_params System Clock %x\n", aic31xx->sysclk);
	i = aic31xx_get_divs(aic31xx->sysclk, params_rate(params));

	 if (i < 0) {
		 printk("sampling rate not supported\n");
		 return i;
	 }
	DBG("###aic31xx_hw_params Sampling Rate %d\n", params_rate(params));


	 /* We will fix R value to 1 and will make P & J=K.D as
	    varialble *//* Setting P & R values */
	 aic31xx_write(codec, CLK_REG_2, ((aic31xx_divs[i].p_val << 4) | 0x01)); 

	 /* J value */
	 aic31xx_write(codec, CLK_REG_3, aic31xx_divs[i].pll_j);

	 /* MSB & LSB for D value */
	 aic31xx_write(codec, CLK_REG_4, (aic31xx_divs[i].pll_d >> 8));
	 aic31xx_write(codec, CLK_REG_5, (aic31xx_divs[i].pll_d &
					  AIC31XX_8BITS_MASK));

	 /* NDAC divider value */
	 aic31xx_write(codec, NDAC_CLK_REG, aic31xx_divs[i].ndac);

	 /* MDAC divider value */
	 aic31xx_write(codec, MDAC_CLK_REG, aic31xx_divs[i].mdac);

	 /* DOSR MSB & LSB values */ aic31xx_write(codec, DAC_OSR_MSB,
						   aic31xx_divs[i].dosr >> 8);
	 aic31xx_write(codec, DAC_OSR_LSB,
		       aic31xx_divs[i].dosr & AIC31XX_8BITS_MASK);

	 /* NADC divider value */ aic31xx_write(codec, NADC_CLK_REG,
						aic31xx_divs[i].nadc);

	 /* MADC divider value */ aic31xx_write(codec, MADC_CLK_REG,
						aic31xx_divs[i].madc);

	 /* AOSR value */ aic31xx_write(codec, ADC_OSR_REG,
					aic31xx_divs[i].aosr);

	 /* BCLK N divider */ aic31xx_write(codec, BCLK_N_VAL,
					    aic31xx_divs[i].blck_N);

         printk(KERN_INFO "### Writing NDAC %d MDAC %d NADC %d MADC %d DOSR %d AOSR %d\n",
			aic31xx_divs[i].ndac,aic31xx_divs[i].mdac,
			aic31xx_divs[i].nadc, aic31xx_divs[i].madc,
			aic31xx_divs[i].dosr, aic31xx_divs[i].aosr);

	 data = aic31xx_read(codec, INTERFACE_SET_REG_1);

	 data = data & ~(3 << 4);

	 switch (params_format(params)) {
	 case SNDRV_PCM_FORMAT_S16_LE:
		 break;
	 case SNDRV_PCM_FORMAT_S20_3LE:
		 data |= (AIC31XX_WORD_LEN_20BITS << DATA_LEN_SHIFT);
		 break;
	 case SNDRV_PCM_FORMAT_S24_LE:
		 data |= (AIC31XX_WORD_LEN_24BITS << DATA_LEN_SHIFT);
		 break;
	 case SNDRV_PCM_FORMAT_S32_LE:
		 data |= (AIC31XX_WORD_LEN_32BITS << DATA_LEN_SHIFT);
		 break;
	 }

	 aic31xx_write(codec, INTERFACE_SET_REG_1, data);
         
	 return 0;
 }

/*
 *----------------------------------------------------------------------------
 * Function : aic31xx_mute_codec
 * Purpose  : This function is to mute or unmute the left and right DAC

 *
 *----------------------------------------------------------------------------
 */
static int aic31xx_mute_codec (struct snd_soc_codec *codec, int mute)
{
	u8       dac_reg;
	volatile u8 value;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	volatile u16 time_out_counter;

	printk (KERN_INFO "##+ new aic31xx_mute_codec %d (current state is %d, headset_connected=%d) \n",
               mute, aic31xx->mute, aic31xx->headset_connected);

	dac_reg = aic31xx_read(codec, DAC_MUTE_CTRL_REG);

	/* Also update the global Playback Status Flag. This is required for
           biquad update. */
	if ((mute) && (aic31xx->mute != 1)) {
		aic31xx->playback_status = 0;
                if (aic31xx->headset_connected) {

                        /* Switch ON the Class_D Speaker Amplifier */
			value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
			aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value | 0x80));

                        /* Page 47 of the datasheets requires unmuting HP and
                           Speaker drivers first */
                        /* MUTE the Headphone Left and Right */
                        value = aic31xx_read (codec, HPL_DRIVER);
                        aic31xx_write (codec, HPL_DRIVER, (value & ~0x04));

                        value = aic31xx_read (codec, HPR_DRIVER);
                        aic31xx_write (codec, HPR_DRIVER, (value & ~0x04));
                        printk (KERN_INFO "##MUTED the HPL and HPR DRIVER REGS\n");

                    /* Bring the HP Analog Volume Control Registers back to default value */
                    value = aic31xx_read (codec, R_ANLOG_VOL_2_HPR);
                    while (value < 0x9F) {
                            value++;
                            aic31xx_write (codec, R_ANLOG_VOL_2_HPR, value);
                            mdelay(2);
                            aic31xx_write (codec, L_ANLOG_VOL_2_HPL, value);
                            mdelay(2);
                    }

                    printk (KERN_INFO "Moved R_ANLOG_VOL_2_HPR to %d\r\n", (value & 0x7F));

                } else {
                        /* MUTE THE Class-D Speaker Driver */
                        value = aic31xx_read (codec, SPL_DRIVER);
                        aic31xx_write (codec, SPL_DRIVER, (value & ~BIT2));

                        printk (KERN_INFO "##SPL MUTE REGS\n");

                        /* MUTE THE Class-D Speaker Driver */
                        value = aic31xx_read (codec, SPR_DRIVER);
                        aic31xx_write (codec, SPR_DRIVER, (value & ~BIT2));

                        printk (KERN_INFO "##SPR MUTE REGS\n");

                }

		printk (KERN_INFO "muting DAC .................\n");

		aic31xx_write (codec, DAC_MUTE_CTRL_REG, (dac_reg | MUTE_ON));

		printk (KERN_INFO "##DAC MUTE Completed..\r\n");

		/* Change the DACL and DACR volumes values to lowest value */
		aic31xx_write (codec, LDAC_VOL, 0x81);
		aic31xx_write (codec, RDAC_VOL, 0x81);
		time_out_counter = 0;
		do {
				mdelay(5);
				/* Poll the DAC_FLAG register Page 0 38 for the DAC MUTE
				   Operation Completion Status */
				value = aic31xx_read (codec, DAC_FLAG_2);
				time_out_counter++;
		} while ((time_out_counter < 20) && ((value & 0x11) == 0));
		printk (KERN_INFO "##DAC Vol Poll Completed counter  %d regval %x\r\n",
			time_out_counter, value);

		/* Mute the ADC channel */

		 value = aic31xx_read(codec, ADC_FGA);
		 aic31xx_write(codec, ADC_FGA, (value | BIT7));

		aic31xx->mute = 1;
	} else if ((!mute) && (aic31xx->mute != 0)) {
			aic31xx->playback_status = 1;

			/* Unmuting the ADC channel */

			value = aic31xx_read(codec, ADC_FGA);
			aic31xx_write(codec, ADC_FGA, (value & ~BIT7));


		  /* We will enable the DAC UNMUTE first and finally the
		   Headphone UNMUTE to avoid pops */
			if (aic31xx->headset_connected) {
				/*Read the contents of the Page 0 Reg 63 DAC Data-Path
				 Setup Register. Just retain the upper two bits and
				 lower two bits
				*/
				value = (aic31xx_read(codec, DAC_CHN_REG) & 0xC3);
				aic31xx_write(codec, DAC_CHN_REG, (value | LDAC_2_LCHN | RDAC_2_RCHN));

                	/* Restore the values of the DACL and DACR */
                	aic31xx_write (codec, LDAC_VOL, 0xFC);
                	aic31xx_write (codec, RDAC_VOL, 0xFC);

			time_out_counter = 0;
                	do {
                	        mdelay (5);
                	        value = aic31xx_read (codec, DAC_FLAG_2);
                	        time_out_counter ++;
                	} while ((time_out_counter < 100) && ((value & 0x11) == 0));

			printk (KERN_INFO "Changed DAC Volume back counter %d.\n", time_out_counter);

	                aic31xx_write (codec, DAC_MUTE_CTRL_REG, (dac_reg & ~MUTE_ON));

	                printk (KERN_INFO "##DAC UNMUTED ...\n");

                    /* Bring the HP Analog Volume Control Registers back to default value */
                    value = aic31xx_read (codec, R_ANLOG_VOL_2_HPR);
                    while (value > 0x80) {
                            value--;
                            aic31xx_write (codec, R_ANLOG_VOL_2_HPR, value);
                            mdelay(2);
                            aic31xx_write (codec, L_ANLOG_VOL_2_HPL, value);
                            mdelay(2);
                    }

                    printk (KERN_INFO "Moved R_ANLOG_VOL_2_HPR to %d\r\n", (value & 0x7F));

                    /* Page 47 of the datasheets requires unmuting HP and
                       Speaker drivers first */
                    /* UNMUTE the Headphone Left and Right */
                    value = aic31xx_read (codec, HPL_DRIVER);
                    aic31xx_write (codec, HPL_DRIVER, (value | 0x04));

                    value = aic31xx_read (codec, HPR_DRIVER);
                    aic31xx_write (codec, HPR_DRIVER, (value | 0x04));
                    printk (KERN_INFO "##UNMUTED the HPL and HPR DRIVER REGS\n");


                    /* Switch OFF the Class_D Speaker Amplifier */
                    value = aic31xx_read (codec, CLASSD_SPEAKER_AMP);
                    aic31xx_write (codec, CLASSD_SPEAKER_AMP, (value & ~0x80));


                } else {
                        /*Read the contents of the Page 0 Reg 63 DAC Data-Path
                         Setup Register. Just retain the upper two bits and
                         lower two bits
                        */
                        value = (aic31xx_read(codec, DAC_CHN_REG) & 0xC3);
                        aic31xx_write(codec, DAC_CHN_REG,
                                      (value | LDAC_2_LCHN | RDAC_2_RCHN));

                	/* Restore the values of the DACL and DACR */
                	aic31xx_write (codec, LDAC_VOL, 0xFF);
                	aic31xx_write (codec, RDAC_VOL, 0xFF);

			time_out_counter = 0;
                	do {
                	        mdelay (5);
                	        value = aic31xx_read (codec, DAC_FLAG_2);
                	        time_out_counter ++;
                	} while ((time_out_counter < 100) && ((value & 0x11) == 0));

			printk (KERN_INFO "##Changed DAC Volume back counter %d.\n", time_out_counter);

	                aic31xx_write (codec, DAC_MUTE_CTRL_REG, (dac_reg & ~MUTE_ON));

	                printk (KERN_INFO "##DAC UNMUTED ...\n");
                        /* UNMUTE THE Left Class-D Speaker Driver */
                        value = aic31xx_read (codec, SPL_DRIVER);
                        aic31xx_write (codec, SPL_DRIVER, (value | BIT2));

                        printk (KERN_INFO "##SPL UNMUTE REGS\n");


                        /* UNMUTE THE Right Class-D Speaker Driver */
                        value = aic31xx_read (codec, SPR_DRIVER);
                        aic31xx_write (codec, SPR_DRIVER, (value | BIT2));

                        printk (KERN_INFO "##SPR UNMUTE REGS\n");

                }

        	aic31xx->power_status = 1;
        	aic31xx->mute = 0;
        }

	printk (KERN_INFO "##-aic31xx_mute_codec %d\n", mute);

	return 0;
}

/*
 * aic31xx_mute
 * This function is to mute or unmute the left and right DAC
 */
static int aic31xx_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 dac_reg;

	printk (KERN_INFO "###aic31xx_mute Flag %x\n", mute);

#if 0
	dac_reg = aic31xx_read(codec, DAC_MUTE_CTRL_REG) & ~MUTE_ON;
	if (mute)
		aic31xx_write(codec, DAC_MUTE_CTRL_REG, dac_reg | MUTE_ON);
	else
		aic31xx_write(codec, DAC_MUTE_CTRL_REG, dac_reg);

#else
	aic31xx_mute_codec (codec, mute);

#endif
      /*  debug_print_registers (codec); */

	return 0;
}

/*
 * aic31xx_set_dai_sysclk
 *
 * This function is to the DAI system clock
 */
static int aic31xx_set_dai_sysclk (struct snd_soc_dai *codec_dai,
				   int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	DBG("###aic31xx_set_dai_sysclk SysClk %x\n", freq);
	switch (freq) {
	case AIC31XX_FREQ_12000000:
	case AIC31XX_FREQ_24000000:
        case AIC31XX_FREQ_19200000:
		aic31xx->sysclk = freq;
		return 0;
	}
	printk("Invalid frequency to set DAI system clock\n");
	return -EINVAL;
}

/*
 * aic31xx_set_dai_fmt
 *
 * This function is to set the DAI format
 */
static int aic31xx_set_dai_fmt (struct snd_soc_dai *codec_dai,
				unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 iface_reg;

	printk (KERN_INFO "###aic31xx_set_dai_fmt %x\n", fmt);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic31xx->master = 1;
		iface_reg |= BIT_CLK_MASTER | WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic31xx->master = 0;
		iface_reg &= ~(BIT_CLK_MASTER | WORD_CLK_MASTER);
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		aic31xx->master = 0;
		iface_reg |= BIT_CLK_MASTER;
		iface_reg &= ~(WORD_CLK_MASTER);
		break;
	default:
		printk("Invalid DAI master/slave interface\n");
		return -EINVAL;
	}
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= (AIC31XX_DSP_MODE << AUDIO_MODE_SHIFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg |= (AIC31XX_RIGHT_JUSTIFIED_MODE << AUDIO_MODE_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= (AIC31XX_LEFT_JUSTIFIED_MODE << AUDIO_MODE_SHIFT);
		break;
	default:
		printk("Invalid DAI interface format\n");
		return -EINVAL;
	}

	aic31xx_write(codec, INTERFACE_SET_REG_1, iface_reg);
        printk (KERN_INFO "##-aic31xx_set_dai_fmt Master %d\n", aic31xx->master);

	return 0;
}

/* Left Output Mixer */
static const struct snd_kcontrol_new
left_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("DAC_L To Left Mixer Amp switch",
			DAC_MIX_CTRL, 6, 1, 0),
	SOC_DAPM_SINGLE("DAC_L To HPL Driver switch",
			DAC_MIX_CTRL, 7, 1, 0),
	SOC_DAPM_SINGLE("MIC1LP To Left Mixer Amp switch",
			DAC_MIX_CTRL, 5, 1, 0),
	SOC_DAPM_SINGLE("MIC1RP To Left Mixer Amp switch",
			DAC_MIX_CTRL, 4, 1, 0),
};

/* Right Output Mixer - Valid only for AIC31xx,3110,3100 */
static const struct
snd_kcontrol_new right_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("DAC_R To Right Mixer Amp switch",
			DAC_MIX_CTRL, 2, 1, 0),
	SOC_DAPM_SINGLE("DAC_R To HPR Driver switch", DAC_MIX_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("MIC1RP To Right Mixer Amp switch",
			DAC_MIX_CTRL, 1, 1, 0),
};
#ifdef DRIVER_DAPM_SUPPORT
static const struct snd_soc_dapm_widget aic31xx_dapm_widgets[] = {
	/* DACs */
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", DAC_CHN_REG, 7, 0),
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", DAC_CHN_REG, 6, 0),
	/* ADC */
	SND_SOC_DAPM_ADC("ADC", "Capture", ADC_DIG_MIC, 7, 0),
	/* Output Mixers */
	SND_SOC_DAPM_MIXER("Left Output Mixer", SND_SOC_NOPM, 0, 0,
			   &left_output_mixer_controls[0],
			   ARRAY_SIZE(left_output_mixer_controls)),
	SND_SOC_DAPM_MIXER("Right Output Mixer", SND_SOC_NOPM, 0, 0,
			   &right_output_mixer_controls[0],
			   ARRAY_SIZE(right_output_mixer_controls)),
	/* Analog selection */
	SND_SOC_DAPM_DAC("Left Analog Vol to HPL", "LAV to HPL",
			 L_ANLOG_VOL_2_HPL, 7, 0),
	SND_SOC_DAPM_DAC("Right Analog Vol to HPR", "RAV to HPR",
			 R_ANLOG_VOL_2_HPR, 7, 0),
#ifndef AIC3100_CODEC_SUPPORT
	SND_SOC_DAPM_DAC ("Left Analog Vol to SPL", "LAV to SPL",
			  L_ANLOG_VOL_2_SPL, 7, 0),
	SND_SOC_DAPM_DAC("Right Analog Vol to SPR", "RAV to SPR",
			 R_ANLOG_VOL_2_SPR, 7, 0),
#endif

#ifdef AIC3100_CODEC_SUPPORT
	SND_SOC_DAPM_DAC ("Left Analog Vol to SPL", "LAV to SPL",
			  L_ANLOG_VOL_2_SPL, 7, 0),
#endif /* Output drivers */
	SND_SOC_DAPM_DAC ("HPL Driver", "HPL Powerup",
			  HEADPHONE_DRIVER, 7, 0),
	SND_SOC_DAPM_DAC ("HPR Driver", "HPR Powerup",
			  HEADPHONE_DRIVER, 6, 0),

#ifndef AIC3100_CODEC_SUPPORT /*For AIC31xx and AIC3110 as it is
				stereo both left and right channel class-D can
				be powered up/down */
	SND_SOC_DAPM_DAC ("SPL Class - D", "SPL Powerup",
			  CLASSD_SPEAKER_AMP, 7, 0),
	SND_SOC_DAPM_DAC ("SPR Class - D", "SPR Powerup",
			  CLASSD_SPEAKER_AMP, 6, 0),
#endif

#ifdef AIC3100_CODEC_SUPPORT /*For AIC3100 as is mono only left
			       channel class-D can be powered up/down */
	SND_SOC_DAPM_DAC("SPL Class - D","SPL Powerup",
			 CLASSD_SPEAKER_AMP, 7, 0),
#endif

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_OUTPUT("SPL"),
#ifndef AIC3100_CODEC_SUPPORT
	SND_SOC_DAPM_OUTPUT("SPR"),
#endif

	/* Inputs */
	SND_SOC_DAPM_INPUT("EXTMIC"),
	SND_SOC_DAPM_INPUT("INTMIC"),

};

static const struct snd_soc_dapm_route
aic31xx_audio_map[] = {
	/* Left Output */
	{"Left Output Mixer", "DAC_L To Left Mixer Amp switch", "Left DAC"},
	{"Left Output Mixer", "DAC_L To HPL Driver switch", "Left DAC"},
	{"Left Output Mixer", "MIC1LP To Left Mixer Amp switch ", "Left DAC"},
	{"Left Output Mixer", "MIC1RP To Left Mixer Amp switch", "Left DAC"},

	/* Right Output */
	{"Right Output Mixer", "DAC_R To Right Mixer Amp switch", "Right DAC"},
	{"Right Output Mixer", "DAC_R To HPR Driver switch", "Right DAC"},
	{"Right Output Mixer", "MIC1RP To Right Mixer Amp switch", "Right DAC"},

	/* HPL path */
	{"Left Analog Vol to HPL", NULL, "Left Output Mixer"},
	{"HPL Driver", NULL, "Left Analog Vol to HPL"},
	{"HPL", NULL, "HPL Driver"},

	/* HPR path */
	{"Right Analog Vol to HPR", NULL, "Right Output Mixer"},
	{"HPR Driver", NULL, "Right Analog Vol to HPR"},
	{"HPR", NULL, "HPR Driver"},

	/* SPK L path */
	{"Left Analog Vol to SPL", NULL, "Left Output Mixer"},
	{"SPL Class - D", NULL, "Left Analog Vol to SPL"},
	{"SPL", NULL, "SPL Class - D"},

#ifndef AIC3100_CODEC_SUPPORT /* SPK R path */
	{"Right Analog Vol to SPR", NULL, "Right Output Mixer"},
	{"SPR Class - D", NULL, "Right Analog Vol to SPR"},
	{"SPR", NULL, "SPR Class - D"},
#endif
	{"ADC", NULL, "EXTMIC"},
	{"ADC", NULL, "INTMIC"},
};


/*
 * aic31xx_add_widgets
 *
 * adds all the ASoC Widgets identified by aic31xx_snd_controls array. This
 * routine will be invoked * during the Audio Driver Initialization.
 */
static int aic31xx_add_widgets (struct snd_soc_codec *codec)
{
	printk (KERN_INFO "###aic31xx_add_widgets \n");

/*	snd_soc_dapm_new_controls(codec->dapm, aic31xx_dapm_widgets,
				  ARRAY_SIZE(aic31xx_dapm_widgets));


	snd_soc_dapm_add_routes(codec->dapm, aic31xx_audio_map,
				ARRAY_SIZE(aic31xx_audio_map));

	snd_soc_dapm_new_widgets(codec->dapm);*/
	return 0;
}
#endif

static const char *dac_mute[] = {"Unmute", "Mute"};
static const char *adc_mute[] = {"Unmute", "Mute"};
static const char *dacvolume_extra[] = {"L & R Ind Vol", "LVol = RVol",
					"RVol = LVol"};
static const char *dacvolume_control[] = {"control register", "pin"};
static const char *dacsoftstep_control[] = {"1 step / sample",
					    "1 step / 2 sample", "disabled"};

static const char *beep_generator[] = {"Disabled", "Enabled"};

static const char *micbias_voltage[] = {"off", "2 V", "2.5 V", "AVDD"};
static const char *dacleftip_control[] = {"off", "left data",
					  "right data", "(left + right) / 2"};
static const char *dacrightip_control[] =
{ "off", "right data", "left data", "(left+right)/2" };

static const char *dacvoltage_control[] = {"1.35 V", "5 V ", "1.65 V", "1.8 V"};
static const char *headset_detection[] = {"Disabled", "Enabled"};
static const char *drc_enable[] = {"Disabled", "Enabled"};
static const char *mic1lp_enable[] = {"off", "10 k", "20 k", "40 k"};
static const char *mic1rp_enable[] = {"off", "10 k", "20 k", "40 k"};
static const char *mic1lm_enable[] = {"off", "10 k", "20 k", "40 k"};
static const char *cm_enable[] = {"off","10 k", "20 k", "40 k"};
static const char *mic_enable[] = {"Gain controlled by D0 - D6", "0 db Gain"};
static const char *mic1_enable[] = {"floating", "connected to CM internally"};


/* Creates an array of the Single Ended Widgets */
static const struct soc_enum aic31xx_enum[] = {
	SOC_ENUM_SINGLE(DAC_MUTE_CTRL_REG, 3, 2, dac_mute),
	SOC_ENUM_SINGLE(DAC_MUTE_CTRL_REG, 2, 2, dac_mute),
	SOC_ENUM_SINGLE(DAC_MUTE_CTRL_REG, 0, 3, dacvolume_extra),
	SOC_ENUM_SINGLE(PIN_VOL_CTRL, 7, 2, dacvolume_control),
	SOC_ENUM_SINGLE(DAC_CHN_REG, 0, 3, dacsoftstep_control),
	SOC_ENUM_SINGLE(LEFT_BEEP_GEN, 7, 2, beep_generator),
	SOC_ENUM_SINGLE(MICBIAS_CTRL, 0, 4, micbias_voltage),
	SOC_ENUM_SINGLE(DAC_CHN_REG, 4, 4, dacleftip_control),
	SOC_ENUM_SINGLE(DAC_CHN_REG, 2, 4, dacrightip_control),
	SOC_ENUM_SINGLE(HEADPHONE_DRIVER, 3, 4, dacvoltage_control),
	SOC_ENUM_SINGLE(HEADSET_DETECT, 7, 2, headset_detection),
	SOC_ENUM_DOUBLE(DRC_CTL_REG_1, 6, 5, 2, drc_enable),
	SOC_ENUM_SINGLE(MIC_GAIN, 6, 4, mic1lp_enable),
	SOC_ENUM_SINGLE(MIC_GAIN, 4, 4, mic1rp_enable),
	SOC_ENUM_SINGLE(MIC_GAIN, 2, 4, mic1lm_enable),
	SOC_ENUM_SINGLE(MIC_PGA, 7, 2, mic_enable),
	SOC_ENUM_SINGLE(ADC_IP_SEL, 6, 4, cm_enable),
	SOC_ENUM_SINGLE(ADC_IP_SEL, 4, 4, mic1lm_enable),
	SOC_ENUM_SINGLE(CM_SET, 7, 2, mic1_enable),
	SOC_ENUM_SINGLE(CM_SET, 6, 2, mic1_enable),
	SOC_ENUM_SINGLE(CM_SET, 5, 2, mic1_enable),
	SOC_ENUM_SINGLE(ADC_FGA, 7, 2, adc_mute),
};




static const struct snd_kcontrol_new aic31xx_snd_controls[] = {
	/* DAC Volume Control */
	SOC_DOUBLE_R_AIC31XX("DAC Playback Volume", LDAC_VOL, RDAC_VOL, 0, 0xAf, 0),
	/* DAC mute control */
	SOC_ENUM("Left DAC Mute", aic31xx_enum[LMUTE_ENUM]),
	SOC_ENUM("Right DAC Mute", aic31xx_enum[RMUTE_ENUM]),
	/* DAC volume Extra control */
	SOC_ENUM("DAC volume Extra control", aic31xx_enum[DACEXTRA_ENUM]),
	/* DAC volume Control register/pin control */
	SOC_ENUM("DAC volume Control register/pin",
		 aic31xx_enum[DACCONTROL_ENUM]),
	/* DAC Volume soft stepping control */
	SOC_ENUM("DAC Volume soft stepping", aic31xx_enum[SOFTSTEP_ENUM]),
	/* HP driver mute control */
	SOC_DOUBLE_R("HP driver mute", HPL_DRIVER, HPR_DRIVER, 2, 2, 0),

	/* SP driver mute control */
	SOC_DOUBLE_R("SP driver mute", SPL_DRIVER, SPR_DRIVER, 2, 2, 0),
	
	/* ADC FINE GAIN */
	SOC_SINGLE("ADC FINE GAIN", ADC_FGA, 4, 4, 1),

	/* ADC COARSE GAIN */
	SOC_SINGLE("ADC COARSE GAIN", ADC_CGA, 0, 64, 0),

	/* ADC MIC PGA GAIN */
	SOC_SINGLE("ADC MIC_PGA GAIN", MIC_PGA, 0, 119, 0),

	/* HP driver Volume Control */
	SOC_DOUBLE_R ("HP driver Volume(0 = 0 dB, 9 = 9 dB)",
		      HPL_DRIVER, HPR_DRIVER, 3, 0x09, 0),

	/* Left DAC input selection control */
	SOC_ENUM("Left DAC input selection", aic31xx_enum[DACLEFTIP_ENUM]),
	/* Right DAC input selection control */
	SOC_ENUM("Right DAC input selection", aic31xx_enum[DACRIGHTIP_ENUM]),

	/* Beep generator Enable/Disable control */
	SOC_ENUM("Beep generator Enable / Disable", aic31xx_enum[BEEP_ENUM]),
	/* Beep generator Volume Control */
	SOC_DOUBLE_R("Beep generator Volume Control(0 = -61 dbdB, 63 = 2 dB)",
		     LEFT_BEEP_GEN, RIGHT_BEEP_GEN, 0, 0x3F, 1),
	/* Beep Length MSB control */
	SOC_SINGLE("Beep Length MSB", BEEP_LENGTH_MSB, 0, 255, 0),
	/* Beep Length MID control */
	SOC_SINGLE("Beep Length MID", BEEP_LENGTH_MID, 0, 255, 0),
	/* Beep Length LSB control */
	SOC_SINGLE("Beep Length LSB", BEEP_LENGTH_LSB, 0, 255, 0),
	/* Beep Sin(x) MSB control */
	SOC_SINGLE("Beep Sin(x) MSB", BEEP_SINX_MSB, 0, 255, 0),
	/* Beep Sin(x) LSB control */
	SOC_SINGLE("Beep Sin(x) LSB", BEEP_SINX_LSB, 0, 255, 0),
	/* Beep Cos(x) MSB control */
	SOC_SINGLE("Beep Cos(x) MSB", BEEP_COSX_MSB, 0, 255, 0),
	/* Beep Cos(x) LSB control */
	SOC_SINGLE("Beep Cos(x) LSB", BEEP_COSX_LSB, 0, 255, 0),

	/* Mic Bias voltage */
	SOC_ENUM("Mic Bias Voltage", aic31xx_enum[MICBIAS_ENUM]),

	/* DAC Processing Block Selection */
	SOC_SINGLE("DAC Processing Block Selection(0 <->25)",
		    DAC_PRB_SEL_REG, 0, 0x19, 0),
	/* ADC Processing Block Selection */
	SOC_SINGLE("ADC Processing Block Selection(0 <->25)",
		   ADC_PRB_SEL_REG, 0, 0x12, 0),

	/* Throughput of 7-bit vol ADC for pin control */
	SOC_SINGLE("Throughput of 7 - bit vol ADC for pin",
		   PIN_VOL_CTRL, 0, 0x07, 0),

	/* Audio gain control (AGC) */
	SOC_SINGLE("Audio Gain Control(AGC)", AGC_CTRL_1, 7, 0x01, 0),
	/* AGC Target level control */
	SOC_SINGLE("AGC Target Level Control", AGC_CTRL_1, 4, 0x07, 1),
	/* AGC Maximum PGA applicable */
	SOC_SINGLE("AGC Maximum PGA Control", AGC_CTRL_3, 0, 0x77, 0),
	/* AGC Attack Time control */
	SOC_SINGLE("AGC Attack Time control", AGC_CTRL_4, 3, 0x1F, 0),
	/* AGC Attac Time Multiply factor */
	SOC_SINGLE("AGC_ATC_TIME_MULTIPLIER", AGC_CTRL_4, 0, 8, 0),
	/* AGC Decay Time control */
	SOC_SINGLE("AGC Decay Time control", AGC_CTRL_5, 3, 0x1F, 0),
	/* AGC Decay Time Multiplier */
	SOC_SINGLE("AGC_DECAY_TIME_MULTIPLIER", AGC_CTRL_5, 0, 8, 0),
	/* AGC HYSTERISIS */
        SOC_SINGLE("AGC_HYSTERISIS", AGC_CTRL_2, 6, 3, 0),
	/* AGC Noise Threshold */
	SOC_SINGLE("AGC_NOISE_THRESHOLD", AGC_CTRL_2, 1, 32, 1),
	/* AGC Noise Bounce control */
	SOC_SINGLE("AGC Noice bounce control", AGC_CTRL_6, 0, 0x1F, 0),
	/* AGC Signal Bounce control */
	SOC_SINGLE("AGC Signal bounce control", AGC_CTRL_7, 0, 0x0F, 0),

	/* HP Output common-mode voltage control */
	SOC_ENUM("HP Output common - mode voltage control",
		 aic31xx_enum[VOLTAGE_ENUM]),

	/* Headset detection Enable/Disable control */
	SOC_ENUM("Headset detection Enable / Disable", aic31xx_enum[HSET_ENUM]),

	/* DRC Enable/Disable control */
	SOC_ENUM("DRC Enable / Disable", aic31xx_enum[DRC_ENUM]),
	/* DRC Threshold value control */
	SOC_SINGLE("DRC Threshold value(0 = -3 db, 7 = -24 db)",
		   DRC_CTL_REG_1, 2, 0x07, 0),
	/* DRC Hysteresis value control */
	SOC_SINGLE("DRC Hysteresis value(0 = 0 db, 3 = 3 db)",
		   DRC_CTL_REG_1, 0, 0x03, 0),
	/* DRC Hold time control */
	SOC_SINGLE("DRC hold time", DRC_CTL_REG_2, 3, 0x0F, 0),
	/* DRC attack rate control */ SOC_SINGLE ("DRC attack rate",
						  DRC_CTL_REG_3, 4, 0x0F, 0),
	/* DRC decay rate control */
	SOC_SINGLE("DRC decay rate", DRC_CTL_REG_3, 0, 0x0F, 0),
	/* MIC1LP selection for ADC I/P P-terminal */
	SOC_ENUM("MIC1LP selection for ADC I/P P - terminal",
		 aic31xx_enum[MIC1LP_ENUM]),
	/* MIC1RP selection for ADC I/P P-terminal */
	SOC_ENUM("MIC1RP selection for ADC I/P P - terminal",
		 aic31xx_enum[MIC1RP_ENUM]),
	/* MIC1LM selection for ADC I/P P-terminal */
	SOC_ENUM("MIC1LM selection for ADC I/P P - terminal",
		 aic31xx_enum[MIC1LM_ENUM]),
	/* CM selection for ADC I/P M-terminal */
	SOC_ENUM("CM selection for ADC IP M - terminal",
		 aic31xx_enum[CM_ENUM]),
	/* MIC1LM selection for ADC I/P M-terminal */
	SOC_ENUM("MIC1LM selection for ADC I/P M - terminal",
		 aic31xx_enum[MIC1LMM_ENUM]),
	/* MIC PGA Setting */
	SOC_ENUM("MIC PGA Setting", aic31xx_enum[MIC_ENUM]),
	/* MIC1LP CM Setting */
	SOC_ENUM("MIC1LP CM Setting", aic31xx_enum[MIC1_ENUM]),
	/* MIC1RP CM Setting */
	SOC_ENUM("MIC1RP CM Setting", aic31xx_enum[MIC2_ENUM]),
	/* MIC1LP CM Setting */
	SOC_ENUM("MIC1LM CM Setting", aic31xx_enum[MIC3_ENUM]),
	/* ADC mute control */
	SOC_ENUM("ADC Mute", aic31xx_enum[ADCMUTE_ENUM]),

#ifdef AIC3110_CODEC_SUPPORT 
/* For AIC3110 output is stereo so we are using	SOC_DOUBLE_R macro */
			    
	/* SP Class-D driver output stage gain Control */
	SOC_DOUBLE_R ("SP Class - D driver Volume(0 = 6 dB, 4 = 24 dB)",
		      SPL_DRIVER, SPR_DRIVER, 3, 0x04, 0),
#endif

#ifdef AIC3100_CODEC_SUPPORT
	/* SP Class-D driver output stage gain Control */
	SOC_SINGLE("SP Class - D driver Volume(0 = 6 dB, 4 = 24 dB)",
		   SPL_DRIVER,3,0x04,0),
#endif

	/* HP Analog Gain Volume Control */
	SOC_DOUBLE_R("HP Analog Gain Volume(0 = 0 dB, 127 = -78.3 dB)",
		     L_ANLOG_VOL_2_HPL, R_ANLOG_VOL_2_HPR, 0, 0x7F, 1),

#ifdef AIC3110_CODEC_SUPPORT
	/* SP Analog Gain Volume Control */
	SOC_DOUBLE_R("SP Analog Gain Volume(0 = 0 dB, 127 = -78.3 dB)",
		     L_ANLOG_VOL_2_SPL, R_ANLOG_VOL_2_SPR, 0, 0x7F, 1),
#endif

#ifdef AIC3100_CODEC_SUPPORT
	/* SP Analog Gain Volume Control */
	SOC_SINGLE("SP Analog Gain Volume(0 = 0 dB, 127 = -78.3 dB)",
		   L_ANLOG_VOL_2_SPL,0,0x7F,1),
#endif
	/* Program Registers */
	SOC_SINGLE_AIC31XX ("Program Registers"),
};

/*
 * The global Register Initialization sequence Array. During the Audio
 * Driver initialization, this array will be utilized to perform the
 * default initialization of the audio Driver.
 */
static const struct
aic31xx_configs aic31xx_reg_init[] = {
	/* Reset */
	{RESET, 1},
	{RESET, 0},
	/* Clock settings */
	{CLK_REG_1, CODEC_MUX_VALUE},
	{INTERFACE_SET_REG_1, BCLK_DIR_CTRL},
	{INTERFACE_SET_REG_2, DAC_MOD_CLK_2_BDIV_CLKIN},
       // {INTERFACE_SET_REG_2, ADC_MOD_CLK_2_BDIV_CLKIN},
        {INTERFACE_SET_REG_4, 0x10}, /*old value is 0x12 */
        {INTERFACE_SET_REG_5, 0x00}, /*old value is 0x10 */
	{DOUT_CTRL, 0x12}, /* old value is 0x02 */
	/* The below configurations are added for testing the DOUT Pin. They need to be removed in final version */
        //{CLK_MUX_REG, 0x07},
        //{CLK_MVAL_REG, 0x81},
 
	/* DAC Channels PWR UP */
	{DAC_CHN_REG, RDAC_2_RCHN | LDAC_2_LCHN}, /*Mistral: setting
						    [0][63] to reset value 0x14 (RDAC_2_RCHN | LDAC_2_LCHN)=> DAC
						    power=off DAC delta path =left, vol=soft stepping per sample
						    period...(31xx).*/ /* Unmute DAC channels */
	{DAC_MUTE_CTRL_REG, 0x0C}, /* DAC are muted by
				     default...[0][64]...So we manually unmute*/

	/* DAC volume setting */
	{LDAC_VOL, DEFAULT_VOL}, /* DAC volume set to default value i.e 0db gain...[0][65]...*/

	{RDAC_VOL, DEFAULT_VOL}, /* Only applicable for 31xx...3120
				    does not have this register [0][66]*/
	{HEADSET_DETECT, 0x8F},  /* Headset detection enabled by default and Debounce 
                                  * programmed to 128 ms for Headset Detection and
                                  * 32ms for Headset button-press Detection */
	{L_ANLOG_VOL_2_HPL, 0x00}, /* Set volume of left Analog HPL to 0db attenuation
				     [1][36]*/ 
	{R_ANLOG_VOL_2_HPR, 0x00}, /*Only applicable for 31xx...3120 does not have this register [1][37]*/
	{L_ANLOG_VOL_2_SPL, 0x00}, /*Only applicable for 31xx...3120 does not have this register [1][37]*/
	{R_ANLOG_VOL_2_SPR, 0x00}, /*Only applicable for 31xx...3120 does not have this register [1][37]*/

/* Headset Detect setting */
	{INTL_CRTL_REG_1, 0xC0},
	{GPIO_CRTL_REG_1, 0x14},  // In case we need to configure ADC_WCLK, please use 0x1c
	{HEADSET_DETECT, 0x82},
	{MICBIAS_CTRL, 0x0B},

		/* ADC Setting */
	{ADC_DIG_MIC, 0x00},		/*this value will be altered duing power up
                                          Currently configured to use Digital Microphone Input */
	{CM_SET, 0x20}, /* previous value was 0x01 */

	{AGC_CTRL_1, 0x00}, /* Previous value was 0xA0, now agc disabled */
	{AGC_CTRL_2, 0xFE},
	{AGC_CTRL_3, 0x50},
	{AGC_CTRL_4, 0xA8},
	{AGC_CTRL_5, 0x00},
	{AGC_CTRL_6, 0x00},

	{ADC_PRB_SEL_REG, 0x04},

	/* DAC Output Mixer Setting */
	{DAC_MIX_CTRL, RDAC_2_RAMP | LDAC_2_LAMP}, /*For aic31xx this is applicable...enabling DAC
						     routing through mixer amplifier individually for left & right
						     DAC..[1][35]... */
        /* Coonfigure the Headphone Driver POP Removal Settings */ 
	{HP_POP_CTRL, (HP_POWER_UP_15_3_MSEC | HP_DRIVER_3_9_MS | CM_VOLTAGE_FROM_AVDD)},
	/* mute HP Driver */
	{HPL_DRIVER, 0x00}, /* Unmuting
				    HP_left(31xx)/HP(3120)...in [1][40]*/
	{HPR_DRIVER, 0x00}, /*Unmuting HP_right(31xx)..not present in 3120...[1][41]*/
	{SPL_DRIVER, 0x00},
	{SPR_DRIVER, 0x00},
 
	/* ADC Channel Fine Gain */
	{ADC_FGA, 0x80},
	/* ADC channel Coarse Gain */
	{ADC_CGA, 0x68}, /* 0x40 for 0db */
	/* ON the KC1 board MIC1LM has been configured */
	{MIC_GAIN, 0x04},
	{ADC_IP_SEL, 0x10},

	/* Initial Default configuration for ADC PGA */

	{MIC_PGA, 0x4F}, /* old value is 0x3c */	 
	
};

/*
 * aic31xx_probe
 *
 * This is first driver function called by the SoC core driver.
 */
static int aic31xx_probe (struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx;
        struct i2c_adapter *adapter;

	int ret = 0, i, size =0;

	printk(KERN_INFO "aic31xx_probe: AIC31xx Audio Codec %s\n", AIC31XX_VERSION);

        /*codec->control_data = dev_get_drvdata (codec->dev->parent); */
        adapter = i2c_get_adapter(3);
        if (!adapter) {
                printk(KERN_INFO "can't get i2c adapter\n");
                ret = -ENODEV;
                return (ret);
        }
        printk(KERN_INFO "i2c_get_adapter success. Creating a new i2c client device..\n");

        tlv320aic31xx_client = i2c_new_device(adapter, &tlv320aic31xx_hwmon_info);
        if (!tlv320aic31xx_client) {
                printk(KERN_INFO "can't add i2c device\n");
                ret = -ENODEV;
                return (ret);
        }
        printk (KERN_INFO "i2c_device Pntr %x\n", tlv320aic31xx_client);

        codec->control_data = (void *)tlv320aic31xx_client;

        printk(KERN_INFO "Codec CntrlData %x \n", codec->control_data);
 
        aic31xx = kzalloc (sizeof (struct aic31xx_priv), GFP_KERNEL);

        if (aic31xx == NULL) {
            printk (KERN_INFO "aic31xx_probe kzalloc for Codec Private failed..\n");
            return (-ENOMEM);
        }
        printk (KERN_INFO "aic31xx_probe: Codec Private allocation fine...\n");

        snd_soc_codec_set_drvdata (codec, aic31xx);
  
	codec->hw_write = (hw_write_t) i2c_master_send;

	aic31xx->page_no = 0;
        aic31xx->power_status = 0;
        aic31xx->mute = 1;
        aic31xx->headset_connected = 0;
        aic31xx->playback_status = 0;
        aic31xx->headset_current_status = 0;

        printk (KERN_INFO "Writing default values to Codec Regs..\n");
     

	for (i = 0;
	     i < sizeof(aic31xx_reg_init)/sizeof(struct aic31xx_configs); 
             /* i < 3; */
	     i++) {
		aic31xx_write(codec, aic31xx_reg_init[i].reg_offset,
			      aic31xx_reg_init[i].reg_val);
	}
        /* debug_print_registers (codec); */
        printk (KERN_INFO "Switching the COdec to STANDBY State...\n");

	/* off, with power on */
	aic31xx_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	size = ARRAY_SIZE(aic31xx_snd_controls);
	ret=snd_soc_add_controls(codec, aic31xx_snd_controls,
			     ARRAY_SIZE(aic31xx_snd_controls));
	
	printk(KERN_INFO " snd_soc_add_controls: ARRAY SIZE : %d \n", size);
//	aic31xx_add_widgets(codec); 


        mutex_init(&codec->mutex);


	return ret;
}

/*
 * aic31xx_remove
 *
 * This function is used to unregister the Driver.
 */
static int aic31xx_remove (struct snd_soc_codec *codec)
{
        printk (KERN_INFO "aic31xx_remove ...\n");

	/* power down chip */
	if (codec->control_data)
		aic31xx_set_bias_level(codec, SND_SOC_BIAS_OFF);

#ifdef NO_PCMS
	snd_soc_free_pcms(socdev);
#endif
	snd_soc_dapm_free(codec->dapm);
	kfree(snd_soc_codec_get_drvdata(codec));

	return 0;
}

/*
 * aic31xx_suspend
 * This function is to suspend the AIC31xx driver.
 */
static int aic31xx_suspend (struct snd_soc_codec *codec,
			    pm_message_t state)
{
	printk (KERN_INFO "###aic31xx_suspend\n");
	aic31xx_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

/*
 * aic31xx_resume
 * This function is to resume the AIC31xx driver.
 */
static int aic31xx_resume (struct snd_soc_codec *codec)
{
	int i; u8 data[2];
	u8 *cache =  codec->reg_cache;

	printk (KERN_INFO "###aic31xx_resume\n");
	aic31xx_change_page(codec, 0);

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE (aic31xx_reg); i++) {
		data[0] = i % 128; data[1] = cache[i];
		codec->hw_write(codec->control_data, data, 2);
	}

	aic31xx_change_page(codec, 0);
	aic31xx_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

/**
 * @struct snd_soc_codec_dev_aic31xx
 *
 * This structure is soc audio codec device sturecute which pointer * to basic
 * functions aic31xx_probe(), aic31xx_remove(), aic31xx_suspend() and
 * aic31xx_resume()
 */
struct snd_soc_codec_driver soc_codec_dev_aic31xx = {
	.probe = aic31xx_probe,
	.remove = aic31xx_remove,
	.suspend = aic31xx_suspend,
	.resume = aic31xx_resume,
	.read = aic31xx_read,
	.write = aic31xx_write,
	.set_bias_level = aic31xx_set_bias_level,
	.reg_cache_size = sizeof(aic31xx_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = aic31xx_reg,
};

/**
 * @struct tlv320aic31xx_dai_ops
 *
 * The DAI Operations Structure which contains the call-back function
 * Addresses used by the ALSA Core Layer when operating with this Codec
 * Driver.
 */
static struct snd_soc_dai_ops tlv320aic31xx_dai_ops = {
	.hw_params = aic31xx_hw_params,
	.digital_mute = aic31xx_mute,
	.set_sysclk = aic31xx_set_dai_sysclk,
	.set_fmt = aic31xx_set_dai_fmt,
};

/**
 * @struct tlv320aic31xx_dai
 *
 * SoC Codec DAI driver which has DAI capabilities viz., playback and
 * capture, DAI runtime information viz. state of DAI and pop wait
 * state, and DAI private data.  The AIC31xx rates ranges from 8k to
 * 192k The PCM bit format supported are 16, 20, 24 and 32 bits
 */
struct snd_soc_dai_driver tlv320aic31xx_dai[] = {
{
	.name = "tlv320aic3110",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC31XX_RATES,
		.formats = AIC31XX_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC31XX_RATES,
		.formats = AIC31XX_FORMATS,
	},
	.ops =  &tlv320aic31xx_dai_ops,
},
};


/*
 * tlv320aic31xx_codec_probe
 * This function is invoked by the soc_probe of the ALSA Core Layer during the execution of
 * Core Layer Initialization. This function is used to register the Audio Codec with the
 * ALSA Core Layer using the snd_soc_register_codec API.
 */
static int __devinit tlv320aic31xx_codec_probe (struct platform_device *pdev)
{

       int ret;
       int err;

       printk(KERN_INFO "Came to tlv320aic31xx_codec_probe...\n");
       audio_regulator = regulator_get(NULL, "audio-pwr");
       if (IS_ERR(audio_regulator)) {
           printk("%s: regulator_get error\n",__func__);
           //goto err0;
       }
       err = regulator_set_voltage(audio_regulator, REGU_MIN_VOL, REGU_MAX_VOL);
       if (err) {
           printk("%s: regulator_set 3V error\n",__func__);
           //goto err0;
       }
       regulator_enable(audio_regulator);


       
       ret =  snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_aic31xx, tlv320aic31xx_dai, ARRAY_SIZE(tlv320aic31xx_dai));

       printk(KERN_INFO "snd_soc_register_codec returned %d \n", ret);

       return (ret); 
}

/*
 * aic31xx_codec_remove
 * This function is to unregister the Audio Codec from the ALSA Core Layer.
 */
static int __devexit aic31xx_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
        regulator_disable(audio_regulator);
        regulator_put(audio_regulator);
	return 0;
}

/*
 * @struct tlv320aic31xx_i2c_driver
 *
 * Platform Driver structure used to describe the Driver structure.
 */
static struct platform_driver tlv320aic31xx_i2c_driver = {
     .driver = {
          .name = "tlv320aic3110-codec",
          .owner = THIS_MODULE,
     },
     .probe = tlv320aic31xx_codec_probe,
     .remove = __devexit_p(aic31xx_codec_remove),
};

/*
 * tlv320aic31xx_init
 * This function is as MODULE_INIT Routine for the Codec Driver.
 */
static int __init tlv320aic31xx_init (void)
{
	int ret;
	printk(KERN_INFO "Came to Codec DRiver Init routine...\n");

        ret = platform_driver_register (&tlv320aic31xx_i2c_driver);

	if (ret != 0) {
		printk(KERN_INFO "Failed to register TLV320AIC31xx I2C driver: "
		       "%d\n", ret);
                return ret;
	}

	printk("tlv320aic31xx_init success !!! \n");

       return ret;
}
module_init(tlv320aic31xx_init);

/*
 * tlv320aic31aic31xx_exit
 * This function is as MODULE_EXIT Routine for the Codec Driver.
 */
static void __exit tlv320aic31xx_exit (void)
{
       printk(KERN_INFO "tlv320aic3xx_exit....\n");

       platform_driver_unregister (&tlv320aic31xx_i2c_driver); 
}

module_exit(tlv320aic31xx_exit);

MODULE_DESCRIPTION("ASoC TLV320AIC3100 codec driver");
MODULE_AUTHOR("Ravindra <ravindra@mistralsolutions.com>");
MODULE_LICENSE("GPL");
