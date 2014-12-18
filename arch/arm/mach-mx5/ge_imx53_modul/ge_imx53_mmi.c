/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 *           Melchior FRANZ, Ginzinger electronic systems GmbH
 * @Descr: ge_imx-board specific code for ge_imx53_mmi
 * @TODO:
 *	* make microphone work
 *	* implement wlan/bt functionality
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 ***********************************************************************/
/***********************************************************************
 *  @History:
 *	2012KW22 - melchior.franz:
 *		* adaptation for first hardware version
 *	2012KW12 - manfred.schlaegl:
 *		* ipu interface 24bit but only 18 lines used:
 *			termination on unused ipu-display-lines added (emc)
 *		* updated mapping from mmi-schematic 23.03.2012
 *			https://development.ginzinger.com/armlinux/wiki/imx53_mmi
 *	2012KW04 - manfred.schlaegl: 
 *		* do graphics init only, if this board-type is active
 *		* begin implementation based on ge_imx53_devboard
 *
 *  ge_imx53_board configuration:
 *	mandatory base number: 0x0004xxxx
 *	0x0000 display ET0350G0DM6, ETM0350G0DH6
 *	0x0001 display ETQ570G0DM6, ETQ570G2DM6, ETMQ570G2DH6
 *	0x0002 display ET0430G0DM6, ETM0430G0DH6
 *	0x0003 display ETV570G0DMU, ETMV570G2DHU, ETV570G2DMU
 *	0x0004 display ET0500G0DM6
 *	0x0005 display ET0700G0DM6, ETM0700G0DH6
 *	0x0080 no touch capability
 *
 ***********************************************************************/

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/input/edt-ft5x06.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/leds_pwm.h>
#include <linux/ge_switch.h>
#include <linux/ge_variables.h>
#include <sound/tlv320aic3x.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/board-ge_imx53_modul.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <asm/mach/flash.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mxc_dvfs.h>
#include <mach/iomux-mx53.h>
#include <mach/i2c.h>
#include <mach/mxc_iim.h>
#include <mach/mmc.h>

#include "../crm_regs.h"
#include "../devices.h"
#include "../usb.h"

#include "ge_imx53_iomux.h"
#include "ge_imx53_mmi.h"
#include "ge_imx53_gpio_keys.h"

/* extension specific GPIOs */
#define GPIO_EXT_0		(2*32 + 16)	/* GPIO_3_16 - EIM_D16 */
#define GPIO_EXT_1		(2*32 + 17)	/* GPIO_3_17 - EIM_D17 */

/* resistive touch interrupt */
#define GPIO_RES_INT		(2*32 + 11)	/* GPIO_3_11 - EIM_DA11 */
/* capacitive (0)/resistive (1) selector */
#define GPIO_CAP_RES_SELECT	(2*32 + 18)	/* GPIO_3_18 - EIM_D18 */

/* lcd type */
#define GPIO_LCD_TYPE		(2*32 + 26)	/* GPIO_3_26 - EIM_D26 */
/* lcd enable */
#define GPIO_LCD_EN		(2*32 + 27)	/* GPIO_3_27 - EIM_D27 */

/* edt-display */
#define GPIO_CAP_RST		(2*32 + 20)	/* GPIO_3_20 - EIM_D20 */
#define GPIO_CAP_WAKE		(2*32 + 29)	/* GPIO_3_29 - EIM_D29 */
/* capacitive touch interrupt */
#define GPIO_CAPA_TOUCH_IRQ	(1*32 + 31)	/* GPIO_2_31 - EIM_EB3 */

/* usb host - usbh1 - oc/pwr */
#define GPIO_USBH1_OC		(2*32 + 30)	/* GPIO_3_30 - EIM_D30 */
#define GPIO_USBH1_PWR		(2*32 + 31)	/* GPIO_3_31 - EIM_D31 */
#define GPIO_HUB_POWER		(2*32 + 25)	/* GPIO_3_25 - EIM_D25 */

/* usb otg - otg - oc/pwr */
#define GPIO_USBOTG_OC		(2*32 + 21)	/* GPIO_3_21 - EIM_D21 */
#define GPIO_USBOTG_PWR		(2*32 + 22)	/* GPIO_3_22 - EIM_D22 */

/* sd2 cd */
#define GPIO_SD2_CD		(0*32 + 4)	/* GPIO_1_4 - GPIO_4 */

/* audio enable */
#define GPIO_AUDIO_EN		(3*32 + 14)	/* GPIO_4_14 - KEY_COL4 */

/* 
 * wireless module
 */
#define GPIO_WLAN_IRQ		(3*32 + 15)	/* GPIO_4_15 - KEY_ROW4 */
#define GPIO_BT_EN		(2*32 + 23)	/* GPIO_3_23 - EIM_D23 */
#define GPIO_WLAN_EN		(2*32 + 24)	/* GPIO_3_24 - EIM_D24 */

/*
 * spi2
 */
#define GPIO_CSPI2_CS0		(4*32 + 29)	/* GPIO_5_29 - CSI0_DAT11 */

/* leds */
#define GPIO_STAT_LED		(5*32 + 5)	/* GPIO_6_5 - CSI0_DAT19 */
#define GPIO_KEY_LED_0		(4*32 + 24)	/* GPIO_5_24 - CSI0_DAT6 */
#define GPIO_KEY_LED_1		(4*32 + 23)	/* GPIO_5_23 - CSI0_DAT5 */
#define GPIO_KEY_LED_2		(6*32 + 3)	/* GPIO_7_3 - PATA_DIOR */
#define GPIO_KEY_LED_3		(6*32 + 2)	/* GPIO_7_2 - PATA_INTRQ */
#define GPIO_KEY_LED_4		(2*32 + 12)	/* GPIO_3_12 - EIM_DA12 */
#define GPIO_KEY_LED_5		(2*32 + 14)	/* GPIO_3_14 - EIM_DA14 */
#define GPIO_KEY_LED_6		(2*32 + 15)	/* GPIO_3_15 - EIM_DA15 */
#define GPIO_KEY_LED_7		(4*32 + 19)	/* GPIO_5_19 - CSI0_MCLK */

/* buttons */
#define GPIO_BUT_0		(4*32 + 31)	/* GPIO_5_31 - CSI0_DAT13 */
#define GPIO_BUT_1		(4*32 + 30)	/* GPIO_5_30 - CSI0_DAT12 */
#define GPIO_BUT_2		(4*32 + 21)	/* GPIO_5_21 - CSI0_VSYNC */
#define GPIO_BUT_3		(4*32 + 20)	/* GPIO_5_20 - CSI0_DATA_EN */
#define GPIO_BUT_4		(4*32 + 22)	/* GPIO_5_22 - CSI0_DAT4 */
#define GPIO_BUT_5		(4*32 + 25)	/* GPIO_5_25 - CSI0_DAT7 */
#define GPIO_BUT_6		(2*32 + 19)	/* GPIO_3_19 - EIM_D19 */
#define GPIO_BUT_7		(4*32 + 18)	/* GPIO_5_18 - CSI0_PIXCLK */

/* switches */
#define GPIO_DISPLAY_CONFIG_0	(0*32 + 5)	/* GPIO_1_5 - GPIO_5 */
#define GPIO_DISPLAY_CONFIG_1	(0*32 + 7)	/* GPIO_1_7 - GPIO_7 */
#define GPIO_DISPLAY_CONFIG_2	(0*32 + 8)	/* GPIO_1_8 - GPIO_8 */
#define GPIO_DISPLAY_CONFIG_3   (0*32 + 2)	/* GPIO_1_2 - GPIO_2 */

#define GPIO_HW_VERSION_0	(5*32 + 2)	/* GPIO_6_2 - CSI0_DAT16 */
#define GPIO_HW_VERSION_1	(5*32 + 3)	/* GPIO_6_3 - CSI0_DAT17 */
#define GPIO_HW_VERSION_2	(5*32 + 4)	/* GPIO_6_4 - CSI0_DAT18 */

#define GPIO_MOD_KENNUNG_0	(3*32 + 20)	/* GPIO_4_20 - DI0_PIN4 */
#define GPIO_MOD_KENNUNG_1	(2*32 + 13)	/* GPIO_3_13 - EIM_DA13 */


/*
 * pad config
 */
static iomux_v3_cfg_t ge_imx53_mmi_pads[] = {
	/*
	 * usb host - usbh1
	 */
	/* <AltMode Name="ALT6" BallNumber="W4" BallName="EIM_D30" PowerGroup="WEIM_SEC" Signal="USBOH3_USBH1_OC" IsExcluded="false" /> */
	(_MX53_PAD_EIM_D30__GPIO3_30|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),		/* gpio: GPIO_USBH1_OC */
	/* <AltMode Name="ALT6" BallNumber="W5" BallName="EIM_D31" PowerGroup="WEIM_SEC" Signal="USBOH3_USBH1_PWR" IsExcluded="false" /> */
	MX53_PAD_EIM_D31__GPIO3_31,			/* gpio: GPIO_USBH1_PWR */
	/* <AltMode Name="ALT1" BallNumber="W3" BallName="EIM_D25" PowerGroup="WEIM_SEC" Signal="UART1_DSR" IsExcluded="false" /> */
	MX53_PAD_EIM_D25__GPIO3_25,			/* gpio: GPIO_HUB_POWER */

	/*
	 * usb otg
	 */
	/* <AltMode Name="ALT6" BallNumber="V3" BallName="EIM_D21" PowerGroup="WEIM_SEC" Signal="USBOH3_USBOTG_OC" IsExcluded="false" /> */
	(_MX53_PAD_EIM_D21__GPIO3_21|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),		/* gpio: GPIO_USBOTG_OC */
	/* <AltMode Name="ALT6" BallNumber="W2" BallName="EIM_D22" PowerGroup="WEIM_SEC" Signal="USBOH3_USBOTG_PWR" IsExcluded="false" /> */
	MX53_PAD_EIM_D22__GPIO3_22,			/* gpio: GPIO_USBOTG_PWR */

	/*
	 * i2c2 (Temp-Sensor)
	 */
	/* <AltMode Name="ALT4" BallNumber="F6" BallName="KEY_COL3" PowerGroup="KEYPAD" Signal="I2C2_SCL" Comment="Temp-Sensor" IsExcluded="false" /> */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	/* <AltMode Name="ALT4" BallNumber="D4" BallName="KEY_ROW3" PowerGroup="KEYPAD" Signal="I2C2_SDA" Comment="Temp-Sensor" IsExcluded="false" /> */
	MX53_PAD_KEY_ROW3__I2C2_SDA,

	/*
	 * i2c3 (Extension)
	 */
	/* <AltMode Name="ALT2" BallNumber="A6" BallName="GPIO_3" PowerGroup="GPIO" Signal="I2C3_SCL" Comment="Extension" IsExcluded="false" /> */
	MX53_PAD_GPIO_3__I2C3_SCL,
	/* <AltMode Name="ALT2" BallNumber="B6" BallName="GPIO_6" PowerGroup="GPIO" Signal="I2C3_SDA" Comment="Extension" IsExcluded="false" /> */
	MX53_PAD_GPIO_6__I2C3_SDA,

	/*
	 * can1
	 */
	/* <AltMode Name="ALT2" BallNumber="D5" BallName="KEY_ROW2" PowerGroup="KEYPAD" Signal="CAN1_RXCAN" IsExcluded="false" /> */
	MX53_PAD_KEY_ROW2__CAN1_RXCAN,
	/* <AltMode Name="ALT2" BallNumber="C4" BallName="KEY_COL2" PowerGroup="KEYPAD" Signal="CAN1_TXCAN" IsExcluded="false" /> */
	MX53_PAD_KEY_COL2__CAN1_TXCAN,

	/*
	 * audio / i2s
	 */
	/* <AltMode Name="ALT1" BallNumber="E5" BallName="KEY_COL4" PowerGroup="" Signal="GPIO4_GPIO[14]" IsExcluded="false" /> */
	MX53_PAD_KEY_COL4__GPIO4_14,			/* AUDIO_EN */
	/* <AltMode Name="ALT2" BallNumber="C5" BallName="KEY_COL0" PowerGroup="KEYPAD" Signal="AUDMUX_AUD5_TXC" Comment="Audio" IsExcluded="false" /> */
	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,		/* I2S_SCLK - IMX53_I2SSCLK - BCLK */
	/* <AltMode Name="ALT2" BallNumber="B3" BallName="KEY_ROW0" PowerGroup="KEYPAD" Signal="AUDMUX_AUD5_TXD" Comment="Audio" IsExcluded="false" /> */
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,		/* I2S_DOUT - IMX53_I2SDOUT - DOUT */
	/* <AltMode Name="ALT2" BallNumber="E7" BallName="KEY_COL1" PowerGroup="KEYPAD" Signal="AUDMUX_AUD5_TXFS" Comment="Audio" IsExcluded="false" /> */
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,		/* I2S_LRCLK - IMX53_I2SLRCLK - WCLK */
	/* <AltMode Name="ALT2" BallNumber="D6" BallName="KEY_ROW1" PowerGroup="KEYPAD" Signal="AUDMUX_AUD5_RXD" Comment="Audio" IsExcluded="false" /> */
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,		/* I2S_DIN - IMX53_I2SDIN - DIN */
	/* <AltMode Name="ALT3" BallNumber="C8" BallName="GPIO_0" PowerGroup="GPIO" Signal="CCM_SSI_EXT1_CLK" Comment="Audio" IsExcluded="false" /> */
	MX53_PAD_GPIO_0__CCM_SSI_EXT1_CLK,		/* SYS_MCLK - IMX53_MCLK - MCLK - 1.SSI_EXT1_CLK is 19MHz for camera and codec */

	/*
	 * wireless
	 */
	/* <AltMode Name="ALT1" BallNumber="E6" BallName="KEY_ROW4" PowerGroup="" Signal="GPIO4_GPIO[15]" IsExcluded="false" /> */
	(_MX53_PAD_KEY_ROW4__GPIO4_15|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),	/* gpio: GPIO_WLAN_IRQ */
	/* <AltMode Name="ALT1" BallNumber="Y1" BallName="EIM_D23" PowerGroup="" Signal="GPIO3_GPIO[23]" IsExcluded="false" /> */
	(_MX53_PAD_EIM_D23__GPIO3_23|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* gpio: GPIO_BT_EN */
	/* <AltMode Name="ALT1" BallNumber="Y2" BallName="EIM_D24" PowerGroup="" Signal="GPIO3_GPIO[24]" IsExcluded="false" /> */
	(_MX53_PAD_EIM_D24__GPIO3_24|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* gpio: GPIO_WLAN_EN */
	/* <AltMode Name="ALT1" BallNumber="D2" BallName="PI0_PIN4" PowerGroup="" Signal="" IsExcluded="false" /> */
	(_MX53_PAD_DI0_PIN4__GPIO4_20|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),		/* gpio: GPIO_MOD_KENNUNG_0 */
	/* <AltMode Name="ALT1" BallNumber="AC7" BallName="PI0_PIN4" PowerGroup="" Signal="" IsExcluded="false" /> */
	(_MX53_PAD_EIM_DA13__GPIO3_13|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),		/* gpio: GPIO_MOD_KENNUNG_1 */

	/* sd1 - sdio for wlan */
	/* <AltMode Name="ALT0" BallNumber="E16" BallName="SD1_CLK" PowerGroup="SD1" Signal="ESDHC1_CLK" IsExcluded="false" /> */
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	/* <AltMode Name="ALT0" BallNumber="F18" BallName="SD1_CMD" PowerGroup="SD1" Signal="ESDHC1_CMD" IsExcluded="false" /> */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	/* <AltMode Name="ALT0" BallNumber="A20" BallName="SD1_DATA0" PowerGroup="SD1" Signal="ESDHC1_DAT0" IsExcluded="false" /> */
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	/* <AltMode Name="ALT0" BallNumber="C17" BallName="SD1_DATA1" PowerGroup="SD1" Signal="ESDHC1_DAT1" IsExcluded="false" /> */
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	/* <AltMode Name="ALT0" BallNumber="F17" BallName="SD1_DATA2" PowerGroup="SD1" Signal="ESDHC1_DAT2" IsExcluded="false" /> */
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	/* <AltMode Name="ALT0" BallNumber="F16" BallName="SD1_DATA3" PowerGroup="SD1" Signal="ESDHC1_DAT3" IsExcluded="false" /> */
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,

	/*
	 * sd2 - mmc-card (without cd)
	 */
	/* <AltMode Name="ALT0" BallNumber="E14" BallName="SD2_CLK" PowerGroup="SD2" Signal="ESDHC2_CLK" IsExcluded="false" /> */
	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	/* <AltMode Name="ALT0" BallNumber="C15" BallName="SD2_CMD" PowerGroup="SD2" Signal="ESDHC2_CMD" IsExcluded="false" /> */
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	/* <AltMode Name="ALT0" BallNumber="D13" BallName="SD2_DATA0" PowerGroup="SD2" Signal="ESDHC2_DAT0" IsExcluded="false" /> */
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
	/* <AltMode Name="ALT0" BallNumber="C14" BallName="SD2_DATA1" PowerGroup="SD2" Signal="ESDHC2_DAT1" IsExcluded="false" /> */
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	/* <AltMode Name="ALT0" BallNumber="D14" BallName="SD2_DATA2" PowerGroup="SD2" Signal="ESDHC2_DAT2" IsExcluded="false" /> */
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	/* <AltMode Name="ALT0" BallNumber="E13" BallName="SD2_DATA3" PowerGroup="SD2" Signal="ESDHC2_DAT3" IsExcluded="false" /> */
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
	/* <AltMode Name="ALT6" BallNumber="D8" BallName="GPIO_4" PowerGroup="GPIO" Signal="ESDHC2_CD" IsExcluded="false" /> */
	(_MX53_PAD_GPIO_4__GPIO1_4|MUX_PAD_CTRL(MX53_47K_PULLUP_PAD_CTRL)),	/* gpio: GPIO_SD2_CD - 47 K pullup */

	/*
	 * ECSPI 2 (Extension)
	 */
	/* <AltMode Name="ALT3" BallNumber="R5" BallName="CSI0_DAT10" PowerGroup="IPU_CSI" Signal="ECSPI2_MISO" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT10__ECSPI2_MISO,
	/* <AltMode Name="ALT2" BallNumber="Y7" BallName="EIM_CS1" PowerGroup="WEIM_MAIN__0" Signal="ECSPI2_MOSI" IsExcluded="false" /> */
	MX53_PAD_EIM_CS1__ECSPI2_MOSI,
	/* <AltMode Name="ALT3" BallNumber="T1" BallName="CSI0_DAT8" PowerGroup="IPU_CSI" Signal="ECSPI2_SCLK" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT8__ECSPI2_SCLK,
	/* <AltMode Name="ALT1" BallNumber="T2" BallName="CSI0_DAT11" PowerGroup="IPU_CSI" Signal="ECSPI2_SS0" IsExcluded="false" /> */
	(_MX53_PAD_CSI0_DAT11__GPIO5_29|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)), /* gpio */

	/*
	 * Switches
	 */
	/* <AltMode Name="ALT1" BallNumber="A5" BallName="GPIO_5" PowerGroup="GPIO" Signal="GPIO1_GPIO[5]" IsExcluded="false" /> */
	(_MX53_PAD_GPIO_5__GPIO1_5|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),		/* gpio: GPIO_DISPLAY_CONFIG_0 */
	/* <AltMode Name="ALT1" BallNumber="A4" BallName="GPIO_7" PowerGroup="GPIO" Signal="GPIO1_GPIO[7]" IsExcluded="false" /> */
	(_MX53_PAD_GPIO_7__GPIO1_7|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),		/* gpio: GPIO_DISPLAY_CONFIG_1 */
	/* <AltMode Name="ALT1" BallNumber="B5" BallName="GPIO_8" PowerGroup="GPIO" Signal="GPIO1_GPIO[8]" IsExcluded="false" /> */
	(_MX53_PAD_GPIO_8__GPIO1_8|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),		/* gpio: GPIO_DISPLAY_CONFIG_2 */
	/* <AltMode Name="ALT1" BallNumber="C7" BallName="GPIO_2" PowerGroup="GPIO" Signal="GPIO1_GPIO[2]" IsExcluded="false" /> */
	(_MX53_PAD_GPIO_2__GPIO1_2|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),		/* gpio: GPIO_DISPLAY_CONFIG_3 */

	/* <AltMode Name="ALT1" BallNumber="T4" BallName="CSI0_DAT16" PowerGroup="IPU_CSI" Signal="GPIO6_2" IsExcluded="false" /> */
	(_MX53_PAD_CSI0_DAT16__GPIO6_2|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),	/* gpio: GPIO_HW_VERSION_0 */
	/* <AltMode Name="ALT1" BallNumber="T5" BallName="CSI0_DAT17" PowerGroup="IPU_CSI" Signal="GPIO6_3" IsExcluded="false" /> */
	(_MX53_PAD_CSI0_DAT17__GPIO6_3|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),	/* gpio: GPIO_HW_VERSION_1 */
	/* <AltMode Name="ALT1" BallNumber="U3" BallName="CSI0_DAT18" PowerGroup="IPU_CSI" Signal="GPIO6_4" IsExcluded="false" /> */
	(_MX53_PAD_CSI0_DAT18__GPIO6_4|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),	/* gpio: GPIO_HW_VERSION_2 */

	/*
	 * LEDs
	 */
	/* <AltMode Name="ALT1" BallNumber="U4" BallName="CSI0_DAT19" PowerGroup="IPU_CSI" Signal="GPIO6_5" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT19__GPIO6_5,			/* gpio: GPIO_STAT_LED */
	/* <AltMode Name="ALT1" BallNumber="R6" BallName="CSI0_DAT6" PowerGroup="IPU_CSI" Signal="ECSPI1_MISO" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT6__GPIO5_24,			/* gpio: GPIO_KEY_LED_0 */
	/* <AltMode Name="ALT1" BallNumber="R2" BallName="CSI0_DAT5" PowerGroup="IPU_CSI" Signal="ECSPI1_MOSI" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT5__GPIO5_23,			/* gpio: GPIO_KEY_LED_1 */
	/* <AltMode Name="ALT1" BallNumber="K3" BallName="PATA_DIOR" PowerGroup="PATA__0" Signal="UART2_RTS" IsExcluded="false" /> */
	MX53_PAD_PATA_DIOR__GPIO7_3,			/* gpio: GPIO_KEY_LED_2 */
	/* <AltMode Name="ALT1" BallNumber="K5" BallName="PATA_INTRQ" PowerGroup="PATA__0" Signal="UART2_CTS" IsExcluded="false" /> */
	MX53_PAD_PATA_INTRQ__GPIO7_2,			/* gpio: GPIO_KEY_LED_3 */
	/* <AltMode Name="ALT1" BallNumber="V10" BallName="EIM_DA12" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[12]" IsExcluded="false" /> */
	MX53_PAD_EIM_DA12__GPIO3_12,			/* gpio: GPIO_KEY_LED_4 */
	/* <AltMode Name="ALT1" BallNumber="Y10" BallName="EIM_DA14" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[14]" IsExcluded="false" /> */
	MX53_PAD_EIM_DA14__GPIO3_14,			/* gpio: GPIO_KEY_LED_5 */
	/* <AltMode Name="ALT1" BallNumber="AA9" BallName="EIM_DA15" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[15]" IsExcluded="false" /> */
	MX53_PAD_EIM_DA15__GPIO3_15,			/* gpio: GPIO_KEY_LED_6 */
	/* <AltMode Name="ALT1" BallNumber="P2" BallName="CSI0_MCLK" PowerGroup="IPU_CSI" Signal="CSI0_HSYNC" IsExcluded="false" /> */
	MX53_PAD_CSI0_MCLK__GPIO5_19,			/* gpio: GPIO_KEY_LED_7 */

	/*
	 * Keypad
	 */
	/* <AltMode Name="ALT1" BallNumber="T6" BallName="CSI0_DAT13" PowerGroup="IPU_CSI" Signal="UART4_RXD_MUX" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT13__GPIO5_31,			/* gpio: GPIO_BUT_0 */
	/* <AltMode Name="ALT1" BallNumber="T3" BallName="CSI0_DAT12" PowerGroup="IPU_CSI" Signal="UART4_TXD_MUX" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT12__GPIO5_30,			/* gpio: GPIO_BUT_1 */
	/* <AltMode Name="ALT1" BallNumber="P4" BallName="CSI0_VSYNC" PowerGroup="IPU_CSI" Signal="CSI0_VSYNC" IsExcluded="false" /> */
	MX53_PAD_CSI0_VSYNC__GPIO5_21,			/* gpio: GPIO_BUT_2 */
	/* <AltMode Name="ALT1" BallNumber="P3" BallName="CSI0_DATA_EN" PowerGroup="IPU_CSI" Signal="CSI0_DATA_EN" IsExcluded="false" /> */
	MX53_PAD_CSI0_DATA_EN__GPIO5_20,		/* gpio: GPIO_BUT_3 */
	/* <AltMode Name="ALT1" BallNumber="R1" BallName="CSI0_DAT4" PowerGroup="IPU_CSI" Signal="ECSPI1_SCLK" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT4__GPIO5_22,			/* gpio: GPIO_BUT_4 */
	/* <AltMode Name="ALT1" BallNumber="R3" BallName="CSI0_DAT7" PowerGroup="IPU_CSI" Signal="ECSPI1_SS0" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT7__GPIO5_25,			/* gpio: GPIO_BUT_5 */
	/* <AltMode Name="ALT1" BallNumber="V2" BallName="EIM_D19" PowerGroup="WEIM_SEC" Signal="ECSPI1_SS1" IsExcluded="false" /> */
	MX53_PAD_EIM_D19__GPIO3_19,			/* gpio: GPIO_BUT_6 */
	/* <AltMode Name="ALT1" BallNumber="P1" BallName="CSI0_PIXCLK" PowerGroup="IPU_CSI" Signal="CSI0_PIXCLK" IsExcluded="false" /> */
	MX53_PAD_CSI0_PIXCLK__GPIO5_18,

	/* <AltMode Name="ALT1" BallNumber="V4" BallName="EIM_D27" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[27]" Comment="LCD_EN" IsExcluded="false" /> */
	(_MX53_PAD_EIM_D27__GPIO3_27|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* gpio: GPIO_LCD_EN */
	/* <AltMode Name="ALT1" BallNumber="U6" BallName="EIM_D16" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[16]" IsExcluded="false" /> */
	MX53_PAD_EIM_D16__GPIO3_16,			/* gpio: GPIO_EXT_0 */
	/* <AltMode Name="ALT1" BallNumber="U5" BallName="EIM_D17" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[17]" IsExcluded="false" /> */
	MX53_PAD_EIM_D17__GPIO3_17,			/* gpio: GPIO_EXT_1 */
	/* <AltMode Name="ALT1" BallNumber="V1" BallName="EIM_D18" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[18]" IsExcluded="false" /> */
	MX53_PAD_EIM_D18__GPIO3_18,			/* gpio: GPIO_CAP_RES_SELECT */
	/* <AltMode Name="ALT1" BallNumber="W1" BallName="EIM_D20" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[20]" IsExcluded="false" /> */
	//MX53_PAD_EIM_D20__GPIO3_20,			/* gpio: GPIO_EVM_RST */
	(_MX53_PAD_EIM_D20__GPIO3_20|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* gpio: GPIO_EVM_RST */
	/* <AltMode Name="ALT1" BallNumber="AA2" BallName="EIM_D29" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[29]" IsExcluded="false" /> */
	(_MX53_PAD_EIM_D29__GPIO3_29|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* gpio: GPIO_CAP_WAKE */
	/* <AltMode Name="ALT1" BallNumber="Y4" BallName="EIM_EB3" PowerGroup="" Signal="GPIO03_GPIO[31]" IsExcluded="false" /> */
	(_MX53_PAD_EIM_EB3__GPIO2_31|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),	/* gpio: GPIO_CAPA_TOUCH_IRQ */

	/* <AltMode Name="ALT1" BallNumber="AC6" BallName="EIM_DA11" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[11]" IsExcluded="false" />
	 * used as touch-pen-down interrupt for resistive touch internal pulldown because of level-interrupt
	 */
	(_MX53_PAD_EIM_DA11__GPIO3_11|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* gpio: GPIO_RES_INT */
	/* <AltMode Name="ALT1" BallNumber="V5" BallName="EIM_D26" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[26]" Comment="LCD_TYPE" IsExcluded="false" /> */
	(_MX53_PAD_EIM_D26__GPIO3_26|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),		/* gpio: GPIO_LCD_TYPE */


	/*
	 * IPU (24bit interface -> 18 lines used)
	 */
	/* <AltMode Name="ALT0" BallNumber="H4" BallName="DI0_DISP_CLK" PowerGroup="IPU_LCD__1" Signal="IPU_DI0_DISP_CLK" IsExcluded="false" /> */
	(_MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK|MUX_PAD_CTRL(MX53_DISPLAY_CTRL_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="E4" BallName="DI0_PIN15" PowerGroup="IPU_LCD__1" Signal="IPU_DI0_PIN15" Comment="DRDY/DV siehe Datenblatt Tabelle 59" IsExcluded="false" /> */
	(_MX53_PAD_DI0_PIN15__IPU_DI0_PIN15|MUX_PAD_CTRL(MX53_DISPLAY_CTRL_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="D3" BallName="DI0_PIN2" PowerGroup="IPU_LCD__1" Signal="IPU_DI0_PIN2" Comment="HSYNC siehe Datenblatt Tabelle 59" IsExcluded="false" /> */
	(_MX53_PAD_DI0_PIN2__IPU_DI0_PIN2|MUX_PAD_CTRL(MX53_DISPLAY_CTRL_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="C2" BallName="DI0_PIN3" PowerGroup="IPU_LCD__1" Signal="IPU_DI0_PIN3" Comment="VSYNC siehe Datenblatt Tabelle 59" IsExcluded="false" /> */
	(_MX53_PAD_DI0_PIN3__IPU_DI0_PIN3|MUX_PAD_CTRL(MX53_DISPLAY_CTRL_PAD_CTRL)),

	/* <AltMode Name="ALT0" BallNumber="J5" BallName="DISP0_DAT0" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[0]" IsExcluded="false" /> */
	//(_MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	(_MX53_PAD_DISP0_DAT0__GPIO4_21|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),	/* unused -> disabled for emc */
	/* <AltMode Name="ALT0" BallNumber="J4" BallName="DISP0_DAT1" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[1]" IsExcluded="false" /> */
	//(_MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	(_MX53_PAD_DISP0_DAT1__GPIO4_22|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* unused -> disabled for emc */

	/* <AltMode Name="ALT0" BallNumber="G3" BallName="DISP0_DAT10" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[10]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="H5" BallName="DISP0_DAT11" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[11]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="H1" BallName="DISP0_DAT12" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[12]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="E1" BallName="DISP0_DAT13" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[13]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="F2" BallName="DISP0_DAT14" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[14]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="F3" BallName="DISP0_DAT15" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[15]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),

	/* <AltMode Name="ALT0" BallNumber="D1" BallName="DISP0_DAT16" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[16]" IsExcluded="false" /> */
	//(_MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	(_MX53_PAD_DISP0_DAT16__GPIO5_10|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* unused -> disabled for emc */
	/* <AltMode Name="ALT0" BallNumber="F5" BallName="DISP0_DAT17" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[17]" IsExcluded="false" /> */
	//(_MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	(_MX53_PAD_DISP0_DAT17__GPIO5_11|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* unused -> disabled for emc */

	/* <AltMode Name="ALT0" BallNumber="G4" BallName="DISP0_DAT18" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[18]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="G5" BallName="DISP0_DAT19" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[19]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="F4" BallName="DISP0_DAT20" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[20]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="C1" BallName="DISP0_DAT21" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[21]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="E3" BallName="DISP0_DAT22" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[22]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="C3" BallName="DISP0_DAT23" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[23]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),

	/* <AltMode Name="ALT0" BallNumber="G6" BallName="DISP0_DAT8" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[8]" IsExcluded="false" /> */
	//(_MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	(_MX53_PAD_DISP0_DAT8__GPIO4_29|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* unused -> disabled for emc */
	/* <AltMode Name="ALT0" BallNumber="E2" BallName="DISP0_DAT9" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[9]" IsExcluded="false" /> */
	//(_MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	(_MX53_PAD_DISP0_DAT9__GPIO4_30|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* unused -> disabled for emc */

	/* <AltMode Name="ALT0" BallNumber="H2" BallName="DISP0_DAT2" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[2]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="F1" BallName="DISP0_DAT3" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[3]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="G2" BallName="DISP0_DAT4" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[4]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="H3" BallName="DISP0_DAT5" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[5]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="G1" BallName="DISP0_DAT6" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[6]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="H6" BallName="DISP0_DAT7" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[7]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),

	/*
	 * PWM1 (backlight)
	 */
	/* <AltMode Name="ALT4" BallNumber="E8" BallName="GPIO_9" PowerGroup="GPIO" Signal="PWM1_PWMO" IsExcluded="false" /> */
	MX53_PAD_GPIO_9__PWM1_PWMO,

	/*
	 * UART2 (RPC/console; /dev/ttymxc1)
	 */
	/* <AltMode Name="ALT3" BallNumber="K4" BallName="PATA_BUFFER_EN" PowerGroup="PATA__0" Signal="UART2_RXD_MUX" Comment="bootfähig (lt. Hrn Matt)" IsExcluded="false" /> */
	/* already done in module-config (u-boot) */
	/* MX53_PAD_PATA_BUFFER_EN__UART2_DCE_RXD_MUX, */
	/* <AltMode Name="ALT3" BallNumber="J1" BallName="PATA_DMARQ" PowerGroup="PATA__0" Signal="UART2_TXD_MUX" IsExcluded="false" /> */
	/* already done in module-config (u-boot) */
	/* MX53_PAD_PATA_DMARQ__UART2_DCE_TXD_MUX, */

	/*
	 * UART3 (HCI)
	 */
	/* <AltMode Name="ALT4" BallNumber="L3" BallName="PATA_DA_1" PowerGroup="PATA__0" Signal="UART3_CTS" IsExcluded="false" /> */
	MX53_PAD_PATA_DA_1__UART3_DCE_RTS,	/* pad is named CTS, but in DCE-Mode function is RTS */
	/* <AltMode Name="ALT4" BallNumber="L4" BallName="PATA_DA_2" PowerGroup="PATA__0" Signal="UART3_RTS" IsExcluded="false" /> */
	MX53_PAD_PATA_DA_2__UART3_DCE_CTS,	/* pad is named RTS, but in DCE-Mode function is CTS */
	/* <AltMode Name="ALT4" BallNumber="L5" BallName="PATA_CS_0" PowerGroup="PATA__0" Signal="UART3_TXD_MUX" IsExcluded="false" /> */
	MX53_PAD_PATA_CS_0__UART3_DCE_TXD_MUX,
	/* <AltMode Name="ALT4" BallNumber="L2" BallName="PATA_CS_1" PowerGroup="PATA__0" Signal="UART3_RXD_MUX" IsExcluded="false" /> */
	MX53_PAD_PATA_CS_1__UART3_DCE_RXD_MUX,

	/*
	 * UART5 (RS232/TTL; /dev/ttymxc4)
	 */
	/* <AltMode Name="ALT2" BallNumber="U1" BallName="CSI0_DAT14" PowerGroup="IPU_CSI" Signal="UART5_TXD_MUX" Comment="Doppelbelegt mit Kamera (CSIO)" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT14__UART5_DCE_TXD_MUX,
	/* <AltMode Name="ALT2" BallNumber="U2" BallName="CSI0_DAT15" PowerGroup="IPU_CSI" Signal="UART5_RXD_MUX" Comment="Doppelbelegt mit Kamera (CSIO)" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT15__UART5_DCE_RXD_MUX,
};


static int hw_version = 0;


/*
 * sd1/2 read write-protect
 */
static int ge_imx53_mmi_sdhc_write_protect(struct device *dev)
{
	int ret = 0;

	if (to_platform_device(dev)->id == 0) {
		/* mmc0: wlan-sdio - never protected */
		ret = 0;
	} else {
		/* mmc1: no support for wp on micro-sd */
		ret = 0;
	}
	return ret;
}

/*
 * sd1/2 read card-detect
 */
static unsigned int ge_imx53_mmi_sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;
	if (to_platform_device(dev)->id == 0) {
		/* mmc0: wlan-sdio (always connected??? - TODO) */
		ret = 0;
	} else {
		ret = gpio_get_value(GPIO_SD2_CD);	/*hw low?**/
	}
	return ret;
}

/*
 * sd1/2 driver data
 */
static struct mxc_mmc_platform_data ge_imx53_mmi_mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
			| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = ge_imx53_mmi_sdhc_get_card_det_status,
	.wp_status = ge_imx53_mmi_sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

static struct mxc_mmc_platform_data ge_imx53_mmi_mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
			| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = ge_imx53_mmi_sdhc_get_card_det_status,
	.wp_status = ge_imx53_mmi_sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

/*
 * usbh1 power
 */
static void ge_imx53_mmi_usbh1_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(GPIO_USBH1_PWR, 1);
	else
		gpio_set_value(GPIO_USBH1_PWR, 0);
}

/*
 * usbotg power
 */
static void ge_imx53_mmi_usbotg_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(GPIO_USBOTG_PWR, 1);
	else
		gpio_set_value(GPIO_USBOTG_PWR, 0);
}

/*
 * audio
 */
struct aic3x_pdata ge_imx53_mmi_tlv320aic3x_platform_data = {
	.gpio_reset = GPIO_AUDIO_EN,
};

/* glue (soc <-> codec) */
static struct platform_device ge_imx53_mmi_tlv320aic3101_device = {
	.name = "ge_imx53_tlv320aic3101",
};

/* soc ssi (soc) */
static struct mxc_audio_platform_data ge_imx53_mmi_tlv320aic31xx_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
};
/* dummy data for ssi -> neccessary for imx-pcm */
static struct mxc_audio_platform_data ge_imx53_mmi_ssi_data = {
};

/* init */
static int __init ge_imx53_mmi_init_audio(void)
{
	/* register ssi */
	mxc_register_device(&mxc_ssi1_device, &ge_imx53_mmi_ssi_data);
	mxc_register_device(&mxc_ssi2_device, &ge_imx53_mmi_ssi_data);

	mxc_register_device(&ge_imx53_mmi_tlv320aic3101_device,
			&ge_imx53_mmi_tlv320aic31xx_data);
	return 0;
}

/*
 * i2c2
 */
static struct imxi2c_platform_data ge_imx53_mmi_i2c2_data = {
	.bitrate = 400000,
};

static struct i2c_board_info ge_imx53_mmi_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
		.platform_data = &ge_imx53_mmi_tlv320aic3x_platform_data,
	},
	{
		I2C_BOARD_INFO("lm73", 0x4C),
	},
};

/*
 * touchscreen
 */
/* ar1020 i2c resistive touchscreen driver */
static struct i2c_board_info ge_imx53_mmi_ar1020_i2c2_board_info = {
	I2C_BOARD_INFO("ar1020", 0x4d),	/* use emtrion-based driver */
	.irq = gpio_to_irq(GPIO_RES_INT),
};

/* edt-ft5x06-i2c capacitive touchscreen driver */
static struct edt_ft5x06_platform_data ge_imx53_mmi_ft5x06_data = {
	.reset_pin = GPIO_CAP_RST,
	.irq_pin = GPIO_CAPA_TOUCH_IRQ,
};

static struct i2c_board_info ge_imx53_mmi_ft5x06_i2c2_board_info = {
	I2C_BOARD_INFO("edt-ft5x06", 0x38),
	.platform_data = &ge_imx53_mmi_ft5x06_data,
};

/*
 * i2c3
 */
static struct imxi2c_platform_data ge_imx53_mmi_i2c3_data = {
	.bitrate = 400000,
};

static struct i2c_board_info ge_imx53_mmi_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("mcp7941x", 0x6f),
	},
};

/*
 * ecspi2
 */
/* workaround for ecspi chipselect pin may not keep correct level when idle */
static void ge_imx53_mmi_spi_chipselect_active(int cspi_mode, int status,
		int chipselect)
{
	/* ignore unsupported spi-interface */
	if (cspi_mode != 1)
		return;

	if (chipselect == 1)
		gpio_set_value(GPIO_CSPI2_CS0, 0);
	else
		gpio_set_value(GPIO_CSPI2_CS0, 1);
}

static void ge_imx53_mmi_spi_chipselect_inactive(int cspi_mode, int status,
		int chipselect)
{
	/* ignore unsupported spi-interface */
	if (cspi_mode != 1)
		return;

	if (chipselect == 1)
		gpio_set_value(GPIO_CSPI2_CS0, 1);
	else
		gpio_set_value(GPIO_CSPI2_CS0, 0);
}

static struct mxc_spi_master ge_imx53_mmi_mxcspi2_data = {
	.maxchipselect = 1,
	.spi_version = 23,
	.chipselect_active = ge_imx53_mmi_spi_chipselect_active,
	.chipselect_inactive = ge_imx53_mmi_spi_chipselect_inactive,
};

/*
 * can1
 */
/* based on emtrion-settings */
static struct flexcan_platform_data ge_imx53_mmi_can1_data = {
	.core_reg = NULL,
	.io_reg = NULL,
	.root_clk_id = "lp_apm", /* lp_apm is 24MHz */
	.xcvr_enable = NULL,
	.br_clksrc = 0,
	.br_rjw = 2,
	.br_presdiv = 3,
	.br_propseg = 2,
	.br_pseg1 = 3,
	.br_pseg2 = 3,
	.bcc = 1,
	.srx_dis = 1,
	.boff_rec = 1,
	.ext_msg = 1,
	.std_msg = 1,
};

/*
 * BACKLIGHT: PWM 1
 * no frequency control from userspace
 */
static struct platform_pwm_backlight_data ge_imx53_mmi_pwm1_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 1000,
	/* .dft_brightness is set in ge_imx53_mmi_init_fb */
//	.pwm_period_ns =  400000,	/* 2.5 KHz */
//	.pwm_period_ns =  100000,	/* 10 KHz */
	.pwm_period_ns =  1000000,	/* 1 KHz */
};

/*
 * LED
 */
static struct gpio_led ge_imx53_mmi_leds[] = {
	{
		.name = "stat_led",
		.gpio = GPIO_STAT_LED,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},
	{
		.name = "key_led_0",
		.gpio = GPIO_KEY_LED_0,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},
	{
		.name = "key_led_1",
		.gpio = GPIO_KEY_LED_1,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},
	{
		.name = "key_led_2",
		.gpio = GPIO_KEY_LED_2,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},
	{
		.name = "key_led_3",
		.gpio = GPIO_KEY_LED_3,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},
	{
		.name = "key_led_4",
		.gpio = GPIO_KEY_LED_4,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},
	{
		.name = "key_led_5",
		.gpio = GPIO_KEY_LED_5,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},
	{
		.name = "key_led_6",
		.gpio = GPIO_KEY_LED_6,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},
	{
		.name = "key_led_7",
		.gpio = GPIO_KEY_LED_7,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},
	{
		.name = "lcd_en",
		.gpio = GPIO_LCD_EN,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = "backlight",
	},
	{
		.name = "hub_power",
		.gpio = GPIO_HUB_POWER,
		.active_low = 0,
		.retain_state_suspended = 1,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},
	{
		.name = "cap_wake",
		.gpio = GPIO_CAP_WAKE,
		.active_low = 0,
		/* do not put touch-controller to sleep during system-suspend (wake on touch) */
		.retain_state_suspended = 1,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = NULL,
	},
};

static struct gpio_led_platform_data ge_imx53_mmi_led_data = {
	.num_leds	= ARRAY_SIZE(ge_imx53_mmi_leds),
	.leds		= ge_imx53_mmi_leds,
};

static struct platform_device ge_imx53_mmi_led_device = {
	.name = "leds-gpio",
};

/*
 * Keys
 */
#define DEBOUNCE_INTERVAL 50 // ms
static struct gpio_keys_button ge_imx53_mmi_keys[] = {
	{
		.desc = "keypad_button_0",
		.gpio = GPIO_BUT_0,
		.code = BTN_0,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "keypad_button_1",
		.gpio = GPIO_BUT_1,
		.code = BTN_1,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "keypad_button_2",
		.gpio = GPIO_BUT_2,
		.code = BTN_2,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "keypad_button_3",
		.gpio = GPIO_BUT_3,
		.code = BTN_3,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "keypad_button_4",
		.gpio = GPIO_BUT_4,
		.code = BTN_4,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "keypad_button_5",
		.gpio = GPIO_BUT_5,
		.code = BTN_5,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "keypad_button_6",
		.gpio = GPIO_BUT_6,
		.code = BTN_6,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "keypad_button_7",
		.gpio = GPIO_BUT_7,
		.code = BTN_7,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "usbh1_overcurrent",
		.gpio = GPIO_USBH1_OC,
		.code = BTN_A,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 0,
		.can_disable = 0,
	},
	{
		.desc = "usbotg_overcurrent",
		.gpio = GPIO_USBOTG_OC,
		.code = BTN_B,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 0,
		.can_disable = 0,
	},
};

static unsigned int ge_imx53_mmi_read_key_state(struct gpio_keys_button *btn, int size)
{
	int i, val;
	unsigned int ret = 0;
	const int bits_per_int = 32;
	if (size > bits_per_int) {
		printk(KERN_WARNING "%s: only %d gpio keys supported\n", __func__, bits_per_int);
		size = bits_per_int;
	}
	for (i = 0; i < size; i++, btn++) {
		// assume buttons that aren't debounced are virtual (e.g. USB overcurrent)
		if (!btn->debounce_interval)
			continue;

		gpio_request(btn->gpio, __func__);
		gpio_direction_input(btn->gpio);
		val = gpio_get_value(btn->gpio);
		if (!!val ^ btn->active_low)
			ret |= 1 << i;
		gpio_free(btn->gpio);
	}
	return ret;
}

/*
 * Switches
 */
struct ge_gpio_pin ge_imx53_mmi_display_config_map[] = {
	{
		.gpio = GPIO_DISPLAY_CONFIG_0,
		.mask = BIT(0),
		.active_low = 1,
	},
	{
		.gpio = GPIO_DISPLAY_CONFIG_1,
		.mask = BIT(1),
		.active_low = 1,
	},
	{
		.gpio = GPIO_DISPLAY_CONFIG_2,
		.mask = BIT(2),
		.active_low = 1,
	},
	{
		.gpio = GPIO_DISPLAY_CONFIG_3,
		.mask = BIT(3),
		.active_low = 1,
	},
};

struct ge_gpio_pin ge_imx53_mmi_hw_version_map[] = {
	{
		.gpio = GPIO_HW_VERSION_0,
		.mask = BIT(0),
		.active_low = 0,
	},
	{
		.gpio = GPIO_HW_VERSION_1,
		.mask = BIT(1),
		.active_low = 0,
	},
	{
		.gpio = GPIO_HW_VERSION_2,
		.mask = BIT(2),
		.active_low = 0,
	},
};

struct ge_gpio_pin ge_imx53_mmi_module_id_map[] = {
	{
		.gpio = GPIO_MOD_KENNUNG_0,
		.mask = BIT(0),
		.active_low = 1,
	},
	{
		.gpio = GPIO_MOD_KENNUNG_1,
		.mask = BIT(1),
		.active_low = 1,
	},
};

static struct ge_gpio_switch ge_imx53_mmi_switches[] = {
	{
		.name = "hex_dip_switch",
		.pins = ge_imx53_mmi_display_config_map,
		.num_pins = ARRAY_SIZE(ge_imx53_mmi_display_config_map),
	},
	{
		.name = "hw_version",
		.pins = ge_imx53_mmi_hw_version_map,
		.num_pins = ARRAY_SIZE(ge_imx53_mmi_hw_version_map),
	},
	{
		.name = "wireless_module_id",
		.pins = ge_imx53_mmi_module_id_map,
		.num_pins = ARRAY_SIZE(ge_imx53_mmi_module_id_map),
	},
};

static struct ge_gpio_switch_platform_data ge_imx53_mmi_switch_data = {
	.num_switches = ARRAY_SIZE(ge_imx53_mmi_switches),
	.switches = ge_imx53_mmi_switches,
};

static struct platform_device ge_imx53_mmi_switch_device = {
       .name = "switches-gpio",
};

static int ge_imx53_mmi_read_switch_value(struct ge_gpio_pin *pins, int size)
{
	int i, val, ret = 0;
	for (i = 0; i < size; i++, pins++) {
		gpio_request(pins->gpio, __func__);
		gpio_direction_input(pins->gpio);
		val = gpio_get_value(pins->gpio);
		if (!!val ^ pins->active_low)
			ret |= pins->mask;
		gpio_free(pins->gpio);
	}
	return ret;
}

/*
 * GRAPHICS
 */

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data ge_imx53_mmi_vpu_data = {
//	.iram_enable = true,	/* TODO */
//	.iram_size = 0x14000,	/* TODO */
	.reset = mx5_vpu_reset,
};

/* for lvds */
static struct ldb_platform_data ge_imx53_mmi_ldb_data = {
	.lvds_bg_reg = "VAUDIO",
	.ext_ref = 1,
};

static struct fb_videomode ge_imx53_mmi_video_modes[] = {
	{ // [0] ET0350G0DM6, ETM0350G0DH6
		.name = "ET0350_320x240",
		//.refresh = 60,
		.xres = 320,
		.yres = 240,
		.pixclock = 155915,
		.left_margin = 68,
		.right_margin = 20,
		.upper_margin = 18,
		.lower_margin = 4,
		.hsync_len = 1,
		.vsync_len = 1,
		.sync = 0,		/* FB_SYNC_HOR_HIGH_ACT, FB_SYNC_VERT_HIGH_ACT, FB_SYNC_CLK_LAT_FALL */
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
	{ // [1] ETQ570G0DM6, ETQ570G2DM6, ETMQ570G2DH6
		.name = "ET0570_320x240",
		//.refresh = 60,
		.xres = 320,
		.yres = 240,
		.pixclock = 155322,
		.left_margin = 38,
		.right_margin = 20,
		.upper_margin = 15,
		.lower_margin = 5,
		.hsync_len = 30,
		.vsync_len = 3,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
	{ // [2] ET0430G0DM6, ETM0430G0DH6
		.name = "ET0430_480x272",
		//.refresh = 60,
		.xres = 480,
		.yres = 272,
		.pixclock = 111000,
		.left_margin = 2,
		.right_margin = 2,
		.upper_margin = 2,
		.lower_margin = 2,
		.hsync_len = 41,
		.vsync_len = 10,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
	{ // [3] ETV570G0DMU, ETMV570G2DHU, ETV570G2DMU
		.name = "ET0570_640x480",
		//.refresh = 60,
		.xres = 640,
		.yres = 480,
		.pixclock = 39683,
		.left_margin = 144,
		.right_margin = 16,
		.upper_margin = 35,
		.lower_margin = 10,
		.hsync_len = 30,
		.vsync_len = 3,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
	{ // [4] ET0500G0DM6
		.name = "ET0500_800x480",
		//.refresh = 60,
		.xres = 800,
		.yres = 480,
		.pixclock = 32612,
		.left_margin = 100,
		.right_margin = 0,
		.upper_margin = 36,
		.lower_margin = 4,
		.hsync_len = 128,
		.vsync_len = 2,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
	{ // [5] ET0700G0DM6, ETM0700G0DH6
		.name = "ET0700_800x480",
		//.refresh = 60,
		.xres = 800,
		.yres = 480,
		.pixclock = 30352,
		.left_margin = 216,
		.right_margin = 40,
		.upper_margin = 36,
		.lower_margin = 4,
		.hsync_len = 128,
		.vsync_len = 2,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
};

static struct mxc_fb_platform_data ge_imx53_mmi_mxcfb_data = {
	/* .mode_str set to fb name in ge_imx53_mmi_init_fb */
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.default_bpp = 32,
	.mode = ge_imx53_mmi_video_modes,
	.num_modes = ARRAY_SIZE(ge_imx53_mmi_video_modes),
};

extern struct resource ge_imx53_modul_mxcfb_resources[];
static int __init ge_imx53_mmi_init_fb(void)
{
	int display_type;

	if (GE_IMX_BOARD_TYPE() != GE_IMX_BOARD_TYPE_GE_IMX53_MMI)
		return 0; // check necessary because of device_initcall()

	display_type = GE_IMX_BOARD_VERSION() & 0x0f;
	if (display_type >= ARRAY_SIZE(ge_imx53_mmi_video_modes)) {
		pr_err("%s - bad display type %i, using 0 (supported range 0..%i)\n",
				__func__, display_type, ARRAY_SIZE(ge_imx53_mmi_video_modes) - 1);
		display_type = 0;
	}

	ge_imx53_mmi_mxcfb_data.mode_str = (char *)ge_imx53_mmi_video_modes[display_type].name;
	pr_info("%s - display_type=%i (\"%s\")\n", __func__, display_type,
			ge_imx53_mmi_mxcfb_data.mode_str);

	/* use di0 */
	mxc_fb_devices[0].num_resources = 1;//ARRAY_SIZE(mxcfb_resources);
	mxc_fb_devices[0].resource = ge_imx53_modul_mxcfb_resources;
	mxc_register_device(&mxc_fb_devices[0], &ge_imx53_mmi_mxcfb_data);

	/* register backlight-pwm: inverted brightness control */
	ge_imx53_mmi_pwm1_backlight_data.dft_brightness = 0;
	mxc_register_device(&mxc_pwm1_backlight_device,
			&ge_imx53_mmi_pwm1_backlight_data);

	/* set initial enable and free after (see leds) */
	gpio_request(GPIO_LCD_EN, "lcd_en");
	gpio_direction_output(GPIO_LCD_EN, 1);
	gpio_free(GPIO_LCD_EN);

	return 0;
}
device_initcall(ge_imx53_mmi_init_fb);

static bool touch_is_capacitive;
static int __init ge_imx53_mmi_init_touchscreen(void)
{
	bool no_touch = GE_IMX_BOARD_VERSION() & 0x80;

	if (no_touch)
		return 0;

	gpio_request(GPIO_CAP_RST, "cap_rst");
	gpio_direction_input(GPIO_CAP_RST);
	touch_is_capacitive = gpio_get_value(GPIO_CAP_RST);

	gpio_request(GPIO_CAP_RES_SELECT, "cap_res_select");
	gpio_direction_output(GPIO_CAP_RES_SELECT, touch_is_capacitive ? 0 : 1);
	gpio_free(GPIO_CAP_RES_SELECT);

	if (touch_is_capacitive) { // ft5x06 based
		i2c_register_board_info(1, &ge_imx53_mmi_ft5x06_i2c2_board_info, 1);
		gpio_direction_output(GPIO_CAP_RST, 1);

		gpio_request(GPIO_CAP_WAKE, "cap_wake");
		gpio_direction_output(GPIO_CAP_WAKE, 1);
		gpio_free(GPIO_CAP_WAKE);

	} else { // resistive; ar1020 based
		i2c_register_board_info(1, &ge_imx53_mmi_ar1020_i2c2_board_info, 1);

		gpio_request(GPIO_RES_INT, "touch");
		gpio_direction_input(GPIO_RES_INT);
		set_irq_type(gpio_to_irq(GPIO_RES_INT), IRQ_TYPE_EDGE_RISING);
		enable_irq_wake(gpio_to_irq(GPIO_RES_INT));
		gpio_free(GPIO_RES_INT);
	}

	gpio_free(GPIO_CAP_RST);
	return 0;
}

/*
 * LS ModFlex TiWi R2 Bluetooth/WLAN transceiver
 */
static int ge_imx53_mmi_init_wireless(void)
{
	pr_info("%s - wlan/bluetooth LS TiWi R2\n", __func__);
	gpio_request(GPIO_BT_EN, "bt_en");
	gpio_direction_output(GPIO_BT_EN, 1);

	gpio_request(GPIO_WLAN_EN, "wlan_en");
	gpio_direction_output(GPIO_WLAN_EN, 1);

	gpio_request(GPIO_WLAN_IRQ, "wlan_irq");
	gpio_direction_input(GPIO_WLAN_IRQ);
	set_irq_type(gpio_to_irq(GPIO_WLAN_IRQ), IRQ_TYPE_EDGE_FALLING);
	enable_irq_wake(gpio_to_irq(GPIO_WLAN_IRQ));
	return 0;
}

/*
 * Variables export
 */
static unsigned int ge_imx53_display_type_get(void)
{
	return GE_IMX_BOARD_VERSION() & 0x0f;
}

static unsigned int ge_imx53_touch_type_get(void)
{
	if (GE_IMX_BOARD_VERSION() & 0x80)
		return 0;
	else
		return touch_is_capacitive ? 2 : 1;
}

static unsigned int ge_imx53_initial_key_state_get(void)
{
	static unsigned int initial_key_state;
	static bool initialized = false;
	if (!initialized) {
		initial_key_state = ge_imx53_mmi_read_key_state(ge_imx53_mmi_keys,
				ARRAY_SIZE(ge_imx53_mmi_keys));
		initialized = true;
	}
	return initial_key_state;
}

static struct ge_variable ge_imx53_variables_map[] = {
	{
		.name = "display_type",
		.get_value = ge_imx53_display_type_get,
	},
	{
		.name = "touch_type",
		.get_value = ge_imx53_touch_type_get,
	},
	{
		.name = "initial_key_state",
		.get_value = ge_imx53_initial_key_state_get,
	},
};

static struct ge_variables_platform_data ge_imx53_mmi_variables_data = {
	.num_variables = ARRAY_SIZE(ge_imx53_variables_map),
	.variables = ge_imx53_variables_map,
};

static struct platform_device ge_imx53_mmi_variables_device = {
	.name = "ge_variables",
	.id = 1,
};

/*
 * peripheral init
 */
static void __init ge_imx53_mmi_periph_init(void)
{
	int module_id;

	/*
	 * mux settings
	 */
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_mmi_pads,
			ARRAY_SIZE(ge_imx53_mmi_pads));

	/*
	 * read switches
	 */
	module_id = ge_imx53_mmi_read_switch_value(ge_imx53_mmi_module_id_map,
			ARRAY_SIZE(ge_imx53_mmi_module_id_map));
	if (module_id == 3)
		ge_imx53_mmi_init_wireless();

	hw_version = ge_imx53_mmi_read_switch_value(ge_imx53_mmi_hw_version_map,
			ARRAY_SIZE(ge_imx53_mmi_hw_version_map));

	/*
	 * USBH1
	 */
	/* USBH1_PWR */
	gpio_request(GPIO_USBH1_PWR, "usbh1-pwr");
	gpio_direction_output(GPIO_USBH1_PWR, 0);
	/* USBH1 register */
	mx5_set_host1_vbus_func(ge_imx53_mmi_usbh1_driver_vbus);
	mx5_usbh1_init();

	/*
	 * USBOTG
	 */
	/* USBOTG_PWR */
	gpio_request(GPIO_USBOTG_PWR, "otg-pwr");
	gpio_direction_output(GPIO_USBOTG_PWR, 0);
	/* USBOTG register */
	mx5_set_otghost_vbus_func(ge_imx53_mmi_usbotg_driver_vbus);
	mx5_usb_dr_init();

	/*
	 * SD1 - sdio for wlan
	 */
	mxc_register_device(&mxcsdhc1_device, &ge_imx53_mmi_mmc1_data);

	/*
	 * SD2 - mmc-card (without cd)
	 */
	/* SD2 CD GPIO */
	gpio_request(GPIO_SD2_CD, "sdhc2-cd");
	gpio_direction_input(GPIO_SD2_CD);
	/* SD2 CD IRQ */
	mxcsdhc2_device.resource[2].start = gpio_to_irq(GPIO_SD2_CD);
	mxcsdhc2_device.resource[2].end = gpio_to_irq(GPIO_SD2_CD);
	/* SD2 register */
	mxc_register_device(&mxcsdhc2_device, &ge_imx53_mmi_mmc2_data);

	/*
	 * I2C2
	 */
	mxc_register_device(&mxci2c_devices[1], &ge_imx53_mmi_i2c2_data);
	i2c_register_board_info(1, ge_imx53_mmi_i2c2_board_info,
			ARRAY_SIZE(ge_imx53_mmi_i2c2_board_info));

	/*
	 * I2C3
	 */
	mxc_register_device(&mxci2c_devices[2], &ge_imx53_mmi_i2c3_data);
	i2c_register_board_info(2, ge_imx53_mmi_i2c3_board_info,
			ARRAY_SIZE(ge_imx53_mmi_i2c3_board_info));

	/*
	 * ECSPI2
	 */
	mxc_register_device(&mxcspi2_device, &ge_imx53_mmi_mxcspi2_data);

	/*
	 * CAN1
	 */
	mxc_register_device(&mxc_flexcan0_device, &ge_imx53_mmi_can1_data);

	/*
	 * AUDIO
 	 */
	ge_imx53_mmi_init_audio();

	/*
	 * PWM
	 */
	mxc_register_device(&mxc_pwm1_device, NULL);

	/*
	 * Variables export
	 */
	mxc_register_device(&ge_imx53_mmi_variables_device, &ge_imx53_mmi_variables_data);

	/*
	 * LEDS
	 */
	mxc_register_device(&ge_imx53_mmi_led_device,
			&ge_imx53_mmi_led_data);

	/*
	 * Keys
	 */
	ge_imx53_gpio_keys_add(ge_imx53_mmi_keys,
			ARRAY_SIZE(ge_imx53_mmi_keys));

	/*
	 * Switches
	 */
	mxc_register_device(&ge_imx53_mmi_switch_device,
			&ge_imx53_mmi_switch_data);

	/*
	 * GRAPHICS
	 */
	mxc_register_device(&mxc_ldb_device, &ge_imx53_mmi_ldb_data);
	mxc_register_device(&mxcvpu_device, &ge_imx53_mmi_vpu_data);
	mxc_register_device(&gpu_device, &gpu_data);

	ge_imx53_mmi_init_touchscreen();
}

/*
 * board init
 */
void __init ge_imx53_mmi_init(void)
{
	pr_info("Ginzinger imx53_mmi (0x%X) Version 0x%X\n", GE_IMX_BOARD_TYPE(), GE_IMX_BOARD_VERSION());

	ge_imx53_mmi_periph_init();
}
