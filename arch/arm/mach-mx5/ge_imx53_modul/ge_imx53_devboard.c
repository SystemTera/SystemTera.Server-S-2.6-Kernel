/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 * @Descr: ge_imx-board specific code for ge_imx53_devboard
 * @TODO:
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
 *	2012KW11 - manfred.schlaegl:
 *		* support for GPIO6_GPIO16_PWREXT_EN
 *			see: https://development.ginzinger.com/armlinux/wiki/ManfredsLogs12KW11-2
 *	2012KW07 - melchior.franz:
 *		* beeper: use pwm-beeper instead of pwm-leds driver
 *	2012KW07 - manfred.schlaegl: 
 *		* lcd_type over ge_switch implemented
 *	2012KW04 - manfred.schlaegl:
 *		* removed obsolete todos
 *		* do graphics init only, if this board-type is active
 *		* include ge_imx53 iomux-definitions
 *		* format for tab-size 8
 *		* serial interface UART1 in DTE-Mode (RX/TX swapped!) 
 *			other serial interfaces in DCE-Mode (RTS/CTS swapped!)
 *		* use corrected mux-settings for serial interfaces
 *		* 47K pullup for SDx CD and WP
 *	2011KW50 - manfred.schlaegl:
 *		* board-version bits 8-11 determines "sound-card"
 *		 	(0 .. none, 1 .. tlv320aic3110, 2 .. tlv320aic3101)
 *		* board-version bits 12-16 determines IPU_PIX_FMT 
 *		 	(0 .. 18bit, else 24bit)
 * 		* set default software-bpp of framebuffer to 32 instead of 16bit
 *		* platform-data may be kept valid while running system (mmc-issues)
 *			removed structure attribute __initdata
 *	2011KW49 - manfred.schlaegl:
 *		* audio: ssi_ext_clk for TLV320AICXXXX activated
 *		* audio: support for TLV320AIC3101 or TLV320AIC3110 based
 *			sound-cards (see defines below) (EXPERIMENTAL)
 *		* support for codec ge_imx53_tlv320aic3101
 *		* audio: dummy-platform-data for sii-device 
 *			(NULL-Pointer exception instead)
 *	2011KW48 - manfred.schlaegl:
 *		* pulldown for touch-pendown-interrupt (because of level-irq in driver)
 *		* structure attributes (__initdata)
 *		* resistive i2c-touchscreen support (ar1020-emtrion)
 *		* graphics support (framebuffer)
 *	2011KW47 - manfred.schlaegl:
 *		* comment correction
 *	2011KW46 - manfred.schlaegl:
 *		* stat_led as led-gpio
 *		* pwm1,2 ok (pwm1 .. backlight, pwm2 .. leds-pwm)
 *		* uart1,3,4,5 ok (tx/rx)
 * 		* usbotg - host ok
 *		* usbh1 ok
 *		* implemented iomux
 *		* begin audio
 *		* support for can1/2 tested
 *		* support for tlv320aic31 (only i2c)
 *		* support for rtc ds1337 tested
 *		* support for i2c3 tested
 * 		* support for i2c2 tested
 *	2011KW45 - manfred.schlaegl:
 *		* support for sd1 and sd2
 *		* sd2 tested
 *		* cleanup
 *		* support for usbh1 and usbotg
 *		* usbotg as host tested
 *	2011KW44 - manfred.schlaegl: 
 *		* rename modul and boards
 *	2011KW43 - manfred.schlaegl: 
 *		* begin implementation
 * *  @TODO:
 *	* test lcd_type over ge_switch
 ***********************************************************************/

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
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

#include"ge_imx53_iomux.h"
#include"ge_imx53_devboard.h"

/* PWREXT_EN - GPIO6_GPIO16_PWREXT_EN */
#define GPIO_PWREXT_EN		(5*32 + 16)	/* GPIO_6_16 */

/* usb host - usbh1 - oc/pwr */
#define GPIO_USBH1_OC		(2*32 + 30)	/* GPIO_3_30 - EIM_D30 (TODO: gpio or function) */
#define GPIO_USBH1_PWR		(2*32 + 31)	/* GPIO_3_31 - EIM_D31 (TODO: gpio or function) */

/* usb otg - otg - oc/pwr */
#define GPIO_USBOTG_OC		(2*32 + 21)	/* GPIO_3_21 - EIM_D21 (TODO: gpio or function) */
#define GPIO_USBOTG_PWR		(2*32 + 22)	/* GPIO_3_22 - EIM_D22 (TODO: gpio or function) */

/* sd1 cs/wp */
#define GPIO_SD1_CD		(2*32 + 13)	/* GPIO_3_13 - EIM_DA13 */
#define GPIO_SD1_WP		(3*32 + 20)	/* GPIO_4_20 - DI0_PIN4 */

/* sd2 cs/wp */
#define GPIO_SD2_CD		(0*32 + 4)	/* GPIO_1_4 - GPIO_4 */
#define GPIO_SD2_WP		(0*32 + 2)	/* GPIO_1_2 - GPIO_2 */

/* stat_led */
#define GPIO_STAT_LED		(2*32 + 18)	/* GPIO_3_18 - EIM_D18 */

/* lcd type */
#define GPIO_LCD_TYPE		(2*32 + 26)	/* GPIO_3_26 - EIM_D26 */
/* lcd enable */
#define GPIO_LCD_EN		(2*32 + 27)	/* GPIO_3_27 - EIM_D27 */

/* lcd enable */
#define GPIO_TOUCH		(2*32 + 11)	/* GPIO_3_11 - EIM_DA11 */

/*
 * pad config
 */
static iomux_v3_cfg_t ge_imx53_devboard_pads[] = {

	/*
	 * usb host - usbh1
	 */
	/* <AltMode Name="ALT6" BallNumber="W4" BallName="EIM_D30" PowerGroup="WEIM_SEC" Signal="USBOH3_USBH1_OC" IsExcluded="false" /> */
//	MX53_PAD_EIM_D30__USBOH3_USBH1_OC,		/* function */
	MX53_PAD_EIM_D30__GPIO3_30,			/* gpio: GPIO_USBH1_OC */

	/* <AltMode Name="ALT6" BallNumber="W5" BallName="EIM_D31" PowerGroup="WEIM_SEC" Signal="USBOH3_USBH1_PWR" IsExcluded="false" /> */
//	MX53_PAD_EIM_D31__USBOH3_USBH1_PWR,		/* function */
	MX53_PAD_EIM_D31__GPIO3_31,			/* gpio: GPIO_USBH1_PWR */

	/*
	 * usb otg - 
	 */
	/* <AltMode Name="ALT6" BallNumber="V3" BallName="EIM_D21" PowerGroup="WEIM_SEC" Signal="USBOH3_USBOTG_OC" IsExcluded="false" /> */
//	MX53_PAD_EIM_D21__USBOH3_USBOTG_OC,		/* function */
	MX53_PAD_EIM_D21__GPIO3_21,			/* gpio: GPIO_USBOTG_OC */
	/* <AltMode Name="ALT6" BallNumber="W2" BallName="EIM_D22" PowerGroup="WEIM_SEC" Signal="USBOH3_USBOTG_PWR" IsExcluded="false" /> */
//	MX53_PAD_EIM_D22__USBOH3_USBOTG_PWR,		/* function */
	MX53_PAD_EIM_D22__GPIO3_22,			/* gpio: GPIO_USBOTG_PWR */

	/*
	 * sd1
	 */
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
	/* <AltMode Name="ALT1" BallNumber="AC7" BallName="EIM_DA13" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[13]" Comment="SD1_CD als GPIO verwendbar, sonst unauflösbarer konflikt mit PWM2" IsExcluded="false" /> */
	(_MX53_PAD_EIM_DA13__GPIO3_13|MUX_PAD_CTRL(MX53_47K_PULLUP_PAD_CTRL)),	/* gpio: GPIO_SD1_CD - 47 K pullup */
	/* <AltMode Name="ALT3" BallNumber="D2" BallName="DI0_PIN4" PowerGroup="IPU_LCD__1" Signal="ESDHC1_WP" IsExcluded="false" /> */
//	MX53_PAD_DI0_PIN4__ESDHC1_WP,			/* function */
	(_MX53_PAD_DI0_PIN4__GPIO4_20|MUX_PAD_CTRL(MX53_47K_PULLUP_PAD_CTRL)),	/* gpio: GPIO_SD1_WP - 47 K pullup */

	/*
	 * sd2
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
//	MX53_PAD_GPIO_4__ESDHC2_CD,			/* function */
	(_MX53_PAD_GPIO_4__GPIO1_4|MUX_PAD_CTRL(MX53_47K_PULLUP_PAD_CTRL)),	/* gpio: GPIO_SD2_CD - 47 K pullup */
	/* <AltMode Name="ALT6" BallNumber="C7" BallName="GPIO_2" PowerGroup="GPIO" Signal="ESDHC2_WP" IsExcluded="false" /> */
//	MX53_PAD_GPIO_2__ESDHC2_WP,			/* function */
	(_MX53_PAD_GPIO_2__GPIO1_2|MUX_PAD_CTRL(MX53_47K_PULLUP_PAD_CTRL)),	/* gpio: GPIO_SD1_WP - 47 K pullup */

	/*
	 * i2c2
	 */
	/* <AltMode Name="ALT4" BallNumber="F6" BallName="KEY_COL3" PowerGroup="KEYPAD" Signal="I2C2_SCL" Comment="Audio" IsExcluded="false" /> */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	/* <AltMode Name="ALT4" BallNumber="D4" BallName="KEY_ROW3" PowerGroup="KEYPAD" Signal="I2C2_SDA" Comment="Audio" IsExcluded="false" /> */
	MX53_PAD_KEY_ROW3__I2C2_SDA,

	/*
	 * i2c3
	 */
	/* <AltMode Name="ALT2" BallNumber="A6" BallName="GPIO_3" PowerGroup="GPIO" Signal="I2C3_SCL" IsExcluded="false" /> */
	MX53_PAD_GPIO_3__I2C3_SCL,
	/* <AltMode Name="ALT2" BallNumber="B6" BallName="GPIO_6" PowerGroup="GPIO" Signal="I2C3_SDA" IsExcluded="false" /> */
	MX53_PAD_GPIO_6__I2C3_SDA,


	/*
	 * can1
	 */
	/* <AltMode Name="ALT2" BallNumber="D5" BallName="KEY_ROW2" PowerGroup="KEYPAD" Signal="CAN1_RXCAN" IsExcluded="false" /> */
	MX53_PAD_KEY_ROW2__CAN1_RXCAN,
	/* <AltMode Name="ALT2" BallNumber="C4" BallName="KEY_COL2" PowerGroup="KEYPAD" Signal="CAN1_TXCAN" IsExcluded="false" /> */
	MX53_PAD_KEY_COL2__CAN1_TXCAN,

	/*
	 * can2
	 */
	/* <AltMode Name="ALT2" BallNumber="E6" BallName="KEY_ROW4" PowerGroup="KEYPAD" Signal="CAN2_RXCAN" IsExcluded="false" /> */
	MX53_PAD_KEY_ROW4__CAN2_RXCAN,	
	/* <AltMode Name="ALT2" BallNumber="E5" BallName="KEY_COL4" PowerGroup="KEYPAD" Signal="CAN2_TXCAN" IsExcluded="false" /> */
	MX53_PAD_KEY_COL4__CAN2_TXCAN,


	/*
	 * audio / i2s
	 */
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
	 * ECSPI1
	 */
	/* <AltMode Name="ALT3" BallNumber="R6" BallName="CSI0_DAT6" PowerGroup="IPU_CSI" Signal="ECSPI1_MISO" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT6__ECSPI1_MISO,
	/* <AltMode Name="ALT3" BallNumber="R2" BallName="CSI0_DAT5" PowerGroup="IPU_CSI" Signal="ECSPI1_MOSI" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT5__ECSPI1_MOSI,
	/* <AltMode Name="ALT3" BallNumber="R1" BallName="CSI0_DAT4" PowerGroup="IPU_CSI" Signal="ECSPI1_SCLK" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT4__ECSPI1_SCLK,
	/* <AltMode Name="ALT3" BallNumber="R3" BallName="CSI0_DAT7" PowerGroup="IPU_CSI" Signal="ECSPI1_SS0" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT7__ECSPI1_SS0,
	/* <AltMode Name="ALT4" BallNumber="V2" BallName="EIM_D19" PowerGroup="WEIM_SEC" Signal="ECSPI1_SS1" IsExcluded="false" /> */
	MX53_PAD_EIM_D19__ECSPI1_SS1,

	/*
	 * ECSPI 2
	 */
	/* <AltMode Name="ALT3" BallNumber="R5" BallName="CSI0_DAT10" PowerGroup="IPU_CSI" Signal="ECSPI2_MISO" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT10__ECSPI2_MISO,
	/* <AltMode Name="ALT2" BallNumber="Y7" BallName="EIM_CS1" PowerGroup="WEIM_MAIN__0" Signal="ECSPI2_MOSI" IsExcluded="false" /> */
	MX53_PAD_EIM_CS1__ECSPI2_MOSI,
	/* <AltMode Name="ALT3" BallNumber="T1" BallName="CSI0_DAT8" PowerGroup="IPU_CSI" Signal="ECSPI2_SCLK" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT8__ECSPI2_SCLK,
	/* <AltMode Name="ALT3" BallNumber="T2" BallName="CSI0_DAT11" PowerGroup="IPU_CSI" Signal="ECSPI2_SS0" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT11__ECSPI2_SS0,


	/*
	 * GPIO
	 */
	/* <AltMode Name="ALT1" BallNumber="A5" BallName="GPIO_5" PowerGroup="GPIO" Signal="GPIO1_GPIO[5]" IsExcluded="false" /> */
	MX53_PAD_GPIO_5__GPIO1_5,
	/* <AltMode Name="ALT1" BallNumber="A4" BallName="GPIO_7" PowerGroup="GPIO" Signal="GPIO1_GPIO[7]" IsExcluded="false" /> */
	MX53_PAD_GPIO_7__GPIO1_7,
	/* <AltMode Name="ALT1" BallNumber="B5" BallName="GPIO_8" PowerGroup="GPIO" Signal="GPIO1_GPIO[8]" IsExcluded="false" /> */
	MX53_PAD_GPIO_8__GPIO1_8,

	/* <AltMode Name="ALT1" BallNumber="AC6" BallName="EIM_DA11" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[11]" IsExcluded="false" />
	 * used as touch-pen-down interrupt for resistive touch internal pulldown because of level-interrupt
	 */
	(_MX53_PAD_EIM_DA11__GPIO3_11|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),
	/* <AltMode Name="ALT1" BallNumber="V10" BallName="EIM_DA12" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[12]" IsExcluded="false" /> */
	MX53_PAD_EIM_DA12__GPIO3_12,
	/* <AltMode Name="ALT1" BallNumber="Y10" BallName="EIM_DA14" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[14]" IsExcluded="false" /> */
	MX53_PAD_EIM_DA14__GPIO3_14,
	/* <AltMode Name="ALT1" BallNumber="AA9" BallName="EIM_DA15" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[15]" IsExcluded="false" /> */
	MX53_PAD_EIM_DA15__GPIO3_15,
	/* <AltMode Name="ALT1" BallNumber="U6" BallName="EIM_D16" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[16]" IsExcluded="false" /> */
	MX53_PAD_EIM_D16__GPIO3_16,
	/* <AltMode Name="ALT1" BallNumber="U5" BallName="EIM_D17" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[17]" IsExcluded="false" /> */
	MX53_PAD_EIM_D17__GPIO3_17,
	/* <AltMode Name="ALT1" BallNumber="V1" BallName="EIM_D18" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[18]" IsExcluded="false" /> */
	MX53_PAD_EIM_D18__GPIO3_18,
	/* <AltMode Name="ALT1" BallNumber="W1" BallName="EIM_D20" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[20]" IsExcluded="false" /> */
	MX53_PAD_EIM_D20__GPIO3_20,
	/* <AltMode Name="ALT1" BallNumber="V5" BallName="EIM_D26" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[26]" Comment="LCD_TYPE" IsExcluded="false" /> */
	MX53_PAD_EIM_D26__GPIO3_26,
	/* <AltMode Name="ALT1" BallNumber="V4" BallName="EIM_D27" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[27]" Comment="LCD_EN" IsExcluded="false" /> */
	MX53_PAD_EIM_D27__GPIO3_27,
	/* <AltMode Name="ALT1" BallNumber="AA2" BallName="EIM_D29" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[29]" IsExcluded="false" /> */
	MX53_PAD_EIM_D29__GPIO3_29,


	/*
	 * IPU
	 */
	/* <AltMode Name="ALT0" BallNumber="T4" BallName="CSI0_DAT16" PowerGroup="IPU_CSI" Signal="IPU_CSI0_D[16]" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT16__IPU_CSI0_D_16,
	/* <AltMode Name="ALT0" BallNumber="T5" BallName="CSI0_DAT17" PowerGroup="IPU_CSI" Signal="IPU_CSI0_D[17]" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT17__IPU_CSI0_D_17,
	/* <AltMode Name="ALT0" BallNumber="U3" BallName="CSI0_DAT18" PowerGroup="IPU_CSI" Signal="IPU_CSI0_D[18]" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT18__IPU_CSI0_D_18,
	/* <AltMode Name="ALT0" BallNumber="U4" BallName="CSI0_DAT19" PowerGroup="IPU_CSI" Signal="IPU_CSI0_D[19]" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT19__IPU_CSI0_D_19,
	/* <AltMode Name="ALT0" BallNumber="P3" BallName="CSI0_DATA_EN" PowerGroup="IPU_CSI" Signal="IPU_CSI0_DATA_EN" IsExcluded="false" /> */
	MX53_PAD_CSI0_DATA_EN__IPU_CSI0_DATA_EN,
	/* <AltMode Name="ALT0" BallNumber="P2" BallName="CSI0_MCLK" PowerGroup="IPU_CSI" Signal="IPU_CSI0_HSYNC" IsExcluded="false" /> */
	MX53_PAD_CSI0_MCLK__IPU_CSI0_HSYNC,
	/* <AltMode Name="ALT0" BallNumber="P1" BallName="CSI0_PIXCLK" PowerGroup="IPU_CSI" Signal="IPU_CSI0_PIXCLK" IsExcluded="false" /> */
	MX53_PAD_CSI0_PIXCLK__IPU_CSI0_PIXCLK,
	/* <AltMode Name="ALT0" BallNumber="P4" BallName="CSI0_VSYNC" PowerGroup="IPU_CSI" Signal="IPU_CSI0_VSYNC" IsExcluded="false" /> */
	MX53_PAD_CSI0_VSYNC__IPU_CSI0_VSYNC,
	/* <AltMode Name="ALT0" BallNumber="H4" BallName="DI0_DISP_CLK" PowerGroup="IPU_LCD__1" Signal="IPU_DI0_DISP_CLK" IsExcluded="false" /> */
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	/* <AltMode Name="ALT0" BallNumber="E4" BallName="DI0_PIN15" PowerGroup="IPU_LCD__1" Signal="IPU_DI0_PIN15" Comment="DRDY/DV siehe Datenblatt Tabelle 59" IsExcluded="false" /> */
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	/* <AltMode Name="ALT0" BallNumber="D3" BallName="DI0_PIN2" PowerGroup="IPU_LCD__1" Signal="IPU_DI0_PIN2" Comment="HSYNC siehe Datenblatt Tabelle 59" IsExcluded="false" /> */
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	/* <AltMode Name="ALT0" BallNumber="C2" BallName="DI0_PIN3" PowerGroup="IPU_LCD__1" Signal="IPU_DI0_PIN3" Comment="VSYNC siehe Datenblatt Tabelle 59" IsExcluded="false" /> */
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
	/* <AltMode Name="ALT0" BallNumber="J5" BallName="DISP0_DAT0" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[0]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	/* <AltMode Name="ALT0" BallNumber="J4" BallName="DISP0_DAT1" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[1]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	/* <AltMode Name="ALT0" BallNumber="G3" BallName="DISP0_DAT10" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[10]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	/* <AltMode Name="ALT0" BallNumber="H5" BallName="DISP0_DAT11" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[11]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	/* <AltMode Name="ALT0" BallNumber="H1" BallName="DISP0_DAT12" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[12]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	/* <AltMode Name="ALT0" BallNumber="E1" BallName="DISP0_DAT13" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[13]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	/* <AltMode Name="ALT0" BallNumber="F2" BallName="DISP0_DAT14" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[14]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	/* <AltMode Name="ALT0" BallNumber="F3" BallName="DISP0_DAT15" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[15]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	/* <AltMode Name="ALT0" BallNumber="D1" BallName="DISP0_DAT16" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[16]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	/* <AltMode Name="ALT0" BallNumber="F5" BallName="DISP0_DAT17" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[17]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
	/* <AltMode Name="ALT0" BallNumber="G4" BallName="DISP0_DAT18" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[18]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	/* <AltMode Name="ALT0" BallNumber="G5" BallName="DISP0_DAT19" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[19]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	/* <AltMode Name="ALT0" BallNumber="H2" BallName="DISP0_DAT2" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[2]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	/* <AltMode Name="ALT0" BallNumber="F4" BallName="DISP0_DAT20" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[20]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	/* <AltMode Name="ALT0" BallNumber="C1" BallName="DISP0_DAT21" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[21]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	/* <AltMode Name="ALT0" BallNumber="E3" BallName="DISP0_DAT22" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[22]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	/* <AltMode Name="ALT0" BallNumber="C3" BallName="DISP0_DAT23" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[23]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
	/* <AltMode Name="ALT0" BallNumber="F1" BallName="DISP0_DAT3" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[3]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	/* <AltMode Name="ALT0" BallNumber="G2" BallName="DISP0_DAT4" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[4]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	/* <AltMode Name="ALT0" BallNumber="H3" BallName="DISP0_DAT5" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[5]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	/* <AltMode Name="ALT0" BallNumber="G1" BallName="DISP0_DAT6" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[6]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	/* <AltMode Name="ALT0" BallNumber="H6" BallName="DISP0_DAT7" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[7]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	/* <AltMode Name="ALT0" BallNumber="G6" BallName="DISP0_DAT8" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[8]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	/* <AltMode Name="ALT0" BallNumber="E2" BallName="DISP0_DAT9" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[9]" IsExcluded="false" /> */
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,

	/* <AltMode Name="ALT1" BallNumber="AB16" BallName="LVDS0_CLK_N" PowerGroup="LVDS0" Signal="LDB_LVDS0_CLK_N" IsExcluded="false" /> */
	/* <AltMode Name="ALT1" BallNumber="AC16" BallName="LVDS0_CLK_P" PowerGroup="LVDS0" Signal="LDB_LVDS0_CLK_P" IsExcluded="false" /> */
	MX53_PAD_LVDS0_CLK_P__LDB_LVDS0_CLK,
	/* <AltMode Name="ALT1" BallNumber="Y17" BallName="LVDS0_TX0_N" PowerGroup="LVDS0" Signal="LDB_LVDS0_TX0_N" IsExcluded="false" /> */
	/* <AltMode Name="ALT1" BallNumber="AA17" BallName="LVDS0_TX0_P" PowerGroup="LVDS0" Signal="LDB_LVDS0_TX0_P" IsExcluded="false" /> */
	MX53_PAD_LVDS0_TX0_P__LDB_LVDS0_TX0,
	/* <AltMode Name="ALT1" BallNumber="AB17" BallName="LVDS0_TX1_N" PowerGroup="LVDS0" Signal="LDB_LVDS0_TX1_N" IsExcluded="false" /> */
	/* <AltMode Name="ALT1" BallNumber="AC17" BallName="LVDS0_TX1_P" PowerGroup="LVDS0" Signal="LDB_LVDS0_TX1_P" IsExcluded="false" /> */
	MX53_PAD_LVDS0_TX1_P__LDB_LVDS0_TX1,
	/* <AltMode Name="ALT1" BallNumber="Y16" BallName="LVDS0_TX2_N" PowerGroup="LVDS0" Signal="LDB_LVDS0_TX2_N" IsExcluded="false" /> */
	/* <AltMode Name="ALT1" BallNumber="AA16" BallName="LVDS0_TX2_P" PowerGroup="LVDS0" Signal="LDB_LVDS0_TX2_P" IsExcluded="false" /> */
	MX53_PAD_LVDS0_TX2_P__LDB_LVDS0_TX2,
	/* <AltMode Name="ALT1" BallNumber="AB15" BallName="LVDS0_TX3_N" PowerGroup="LVDS0" Signal="LDB_LVDS0_TX3_N" IsExcluded="false" /> */
	/* <AltMode Name="ALT1" BallNumber="AC15" BallName="LVDS0_TX3_P" PowerGroup="LVDS0" Signal="LDB_LVDS0_TX3_P" IsExcluded="false" /> */
	MX53_PAD_LVDS0_TX3_P__LDB_LVDS0_TX3,
	/* <AltMode Name="ALT1" BallNumber="AA13" BallName="LVDS1_CLK_N" PowerGroup="LVDS1" Signal="LDB_LVDS1_CLK_N" IsExcluded="false" /> */
	/* <AltMode Name="ALT1" BallNumber="Y13" BallName="LVDS1_CLK_P" PowerGroup="LVDS1" Signal="LDB_LVDS1_CLK_P" IsExcluded="false" /> */
	MX53_PAD_LVDS1_CLK_P__LDB_LVDS1_CLK,
	/* <AltMode Name="ALT1" BallNumber="AC14" BallName="LVDS1_TX0_N" PowerGroup="LVDS1" Signal="LDB_LVDS1_TX0_N" IsExcluded="false" /> */
	/* <AltMode Name="ALT1" BallNumber="AB14" BallName="LVDS1_TX0_P" PowerGroup="LVDS1" Signal="LDB_LVDS1_TX0_P" IsExcluded="false" /> */
	MX53_PAD_LVDS1_TX0_P__LDB_LVDS1_TX0,
	/* <AltMode Name="ALT1" BallNumber="AC13" BallName="LVDS1_TX1_N" PowerGroup="LVDS1" Signal="LDB_LVDS1_TX1_N" IsExcluded="false" /> */
	/* <AltMode Name="ALT1" BallNumber="AB13" BallName="LVDS1_TX1_P" PowerGroup="LVDS1" Signal="LDB_LVDS1_TX1_P" IsExcluded="false" /> */
	MX53_PAD_LVDS1_TX1_P__LDB_LVDS1_TX1,
	/* <AltMode Name="ALT1" BallNumber="AC12" BallName="LVDS1_TX2_N" PowerGroup="LVDS1" Signal="LDB_LVDS1_TX2_N" IsExcluded="false" /> */
	/* <AltMode Name="ALT1" BallNumber="AB12" BallName="LVDS1_TX2_P" PowerGroup="LVDS1" Signal="LDB_LVDS1_TX2_P" IsExcluded="false" /> */
	MX53_PAD_LVDS1_TX2_P__LDB_LVDS1_TX2,
	/* <AltMode Name="ALT1" BallNumber="AA12" BallName="LVDS1_TX3_N" PowerGroup="LVDS1" Signal="LDB_LVDS1_TX3_N" IsExcluded="false" /> */
	/* <AltMode Name="ALT1" BallNumber="Y12" BallName="LVDS1_TX3_P" PowerGroup="LVDS1" Signal="LDB_LVDS1_TX3_P" IsExcluded="false" /> */
	MX53_PAD_LVDS1_TX3_P__LDB_LVDS1_TX3,


	/*
	 * PWM1
	 */
	/* <AltMode Name="ALT4" BallNumber="E8" BallName="GPIO_9" PowerGroup="GPIO" Signal="PWM1_PWMO" IsExcluded="false" /> */
	MX53_PAD_GPIO_9__PWM1_PWMO,

	/*
	 * PWM2
	 */
	/* <AltMode Name="ALT4" BallNumber="B7" BallName="GPIO_1" PowerGroup="GPIO" Signal="PWM2_PWMO" IsExcluded="false" /> */
	MX53_PAD_GPIO_1__PWM2_PWMO,

	/*
	 * UART1
	 */
	/* <AltMode Name="ALT3" BallNumber="K2" BallName="PATA_RESET_B" PowerGroup="PATA__0" Signal="UART1_CTS" IsExcluded="false" /> */
	MX53_PAD_PATA_RESET_B__UART1_DTE_CTS,
	/* <AltMode Name="ALT3" BallNumber="Y1" BallName="EIM_D23" PowerGroup="WEIM_SEC" Signal="UART1_DCD" IsExcluded="false" /> */
	MX53_PAD_EIM_D23__UART1_DTE_DCD,
	/* <AltMode Name="ALT7" BallNumber="W3" BallName="EIM_D25" PowerGroup="WEIM_SEC" Signal="UART1_DSR" IsExcluded="false" /> */
	MX53_PAD_EIM_D25__UART1_DTE_DSR,
	/* <AltMode Name="ALT7" BallNumber="Y2" BallName="EIM_D24" PowerGroup="WEIM_SEC" Signal="UART1_DTR" IsExcluded="false" /> */
	MX53_PAD_EIM_D24__UART1_DTE_DTR,
	/* <AltMode Name="ALT3" BallNumber="Y4" BallName="EIM_EB3" PowerGroup="WEIM_SEC" Signal="UART1_RI" IsExcluded="false" /> */
	MX53_PAD_EIM_EB3__UART1_DTE_RI,
	/* <AltMode Name="ALT3" BallNumber="K1" BallName="PATA_IORDY" PowerGroup="PATA__0" Signal="UART1_RTS" IsExcluded="false" /> */
	MX53_PAD_PATA_IORDY__UART1_DTE_RTS,
	/* <AltMode Name="ALT3" BallNumber="J2" BallName="PATA_DMACK" PowerGroup="PATA__0" Signal="UART1_RXD_MUX" Comment="bootfähig bei diesem Mapping? nein" IsExcluded="false" /> */
	MX53_PAD_PATA_DMACK__UART1_DTE_TXD_MUX,	/* pad is named RXD, but in DTE-Mode function is TXD */
	/* <AltMode Name="ALT3" BallNumber="J3" BallName="PATA_DIOW" PowerGroup="PATA__0" Signal="UART1_TXD_MUX" IsExcluded="false" /> */
	MX53_PAD_PATA_DIOW__UART1_DTE_RXD_MUX,	/* pad is named TXD, but in DTE-Mode function is RXD */

	/*
	 * UART2
	 */
	/* <AltMode Name="ALT3" BallNumber="K5" BallName="PATA_INTRQ" PowerGroup="PATA__0" Signal="UART2_CTS" IsExcluded="false" /> */
	MX53_PAD_PATA_INTRQ__UART2_DCE_RTS,	/* pad is named CTS, but in DCE-Mode function is CTS */
	/* <AltMode Name="ALT3" BallNumber="K3" BallName="PATA_DIOR" PowerGroup="PATA__0" Signal="UART2_RTS" IsExcluded="false" /> */
	MX53_PAD_PATA_DIOR__UART2_DCE_CTS,	/* pad is named RTS, but in DCE-Mode function is CTS */
	/* <AltMode Name="ALT3" BallNumber="K4" BallName="PATA_BUFFER_EN" PowerGroup="PATA__0" Signal="UART2_RXD_MUX" Comment="bootfähig (lt. Hrn Matt)" IsExcluded="false" /> */
	/* already done in module-config (u-boot) */
	/* MX53_PAD_PATA_BUFFER_EN__UART2_DCE_RXD_MUX, */
	/* <AltMode Name="ALT3" BallNumber="J1" BallName="PATA_DMARQ" PowerGroup="PATA__0" Signal="UART2_TXD_MUX" IsExcluded="false" /> */
	/* already done in module-config (u-boot) */
	/* MX53_PAD_PATA_DMARQ__UART2_DCE_TXD_MUX, */

	/*
	 * UART3
	 */
	/* <AltMode Name="ALT4" BallNumber="L3" BallName="PATA_DA_1" PowerGroup="PATA__0" Signal="UART3_CTS" IsExcluded="false" /> */
	MX53_PAD_PATA_DA_1__UART3_DCE_RTS,	/* pad is named CTS, but in DCE-Mode function is CTS */
	/* <AltMode Name="ALT4" BallNumber="L4" BallName="PATA_DA_2" PowerGroup="PATA__0" Signal="UART3_RTS" IsExcluded="false" /> */
	MX53_PAD_PATA_DA_2__UART3_DCE_CTS,	/* pad is named RTS, but in DCE-Mode function is CTS */
	/* <AltMode Name="ALT4" BallNumber="L5" BallName="PATA_CS_0" PowerGroup="PATA__0" Signal="UART3_TXD_MUX" IsExcluded="false" /> */
	MX53_PAD_PATA_CS_0__UART3_DCE_TXD_MUX,
	/* <AltMode Name="ALT4" BallNumber="L2" BallName="PATA_CS_1" PowerGroup="PATA__0" Signal="UART3_RXD_MUX" IsExcluded="false" /> */
	MX53_PAD_PATA_CS_1__UART3_DCE_RXD_MUX,

	/*
	 * UART4
	 */
	/* <AltMode Name="ALT2" BallNumber="T3" BallName="CSI0_DAT12" PowerGroup="IPU_CSI" Signal="UART4_TXD_MUX" Comment="Doppelbelegt mit Kamera (CSIO)" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT12__UART4_DCE_TXD_MUX,
	/* <AltMode Name="ALT2" BallNumber="T6" BallName="CSI0_DAT13" PowerGroup="IPU_CSI" Signal="UART4_RXD_MUX" Comment="Doppelbelegt mit Kamera (CSIO)" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT13__UART4_DCE_RXD_MUX,

	/*
	 * UART5
	 */
	/* <AltMode Name="ALT2" BallNumber="U1" BallName="CSI0_DAT14" PowerGroup="IPU_CSI" Signal="UART5_TXD_MUX" Comment="Doppelbelegt mit Kamera (CSIO)" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT14__UART5_DCE_TXD_MUX,
	/* <AltMode Name="ALT2" BallNumber="U2" BallName="CSI0_DAT15" PowerGroup="IPU_CSI" Signal="UART5_RXD_MUX" Comment="Doppelbelegt mit Kamera (CSIO)" IsExcluded="false" /> */
	MX53_PAD_CSI0_DAT15__UART5_DCE_RXD_MUX,
};

/*
 * sd1/2 read write-protect
 */
static int ge_imx53_devboard_sdhc_write_protect(struct device *dev)
{
	int ret = 0;

	if (to_platform_device(dev)->id == 0) {
		ret = gpio_get_value(GPIO_SD1_WP);
	} else {
		ret = gpio_get_value(GPIO_SD2_WP);
	}
	return ret;
}

/*
 * sd1/2 read card-detect
 */
static unsigned int ge_imx53_devboard_sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;
	if (to_platform_device(dev)->id == 0) {
		ret = gpio_get_value(GPIO_SD1_CD);
	} else {
		ret = gpio_get_value(GPIO_SD2_CD);
	}
	return ret;
}

/*
 * sd1/2 driver data
 */
static struct mxc_mmc_platform_data ge_imx53_devboard_mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = ge_imx53_devboard_sdhc_get_card_det_status,
	.wp_status = ge_imx53_devboard_sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

static struct mxc_mmc_platform_data ge_imx53_devboard_mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = ge_imx53_devboard_sdhc_get_card_det_status,
	.wp_status = ge_imx53_devboard_sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

/*
 * usbh1 power
 */
static void ge_imx53_devboard_usbh1_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(GPIO_USBH1_PWR, 1);
	else
		gpio_set_value(GPIO_USBH1_PWR, 0);
}

/*
 * usbotg power
 */
static void ge_imx53_devboard_usbotg_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(GPIO_USBOTG_PWR, 1);
	else
		gpio_set_value(GPIO_USBOTG_PWR, 0);
}

/*
 * i2c2
 */
static struct imxi2c_platform_data ge_imx53_devboard_i2c2_data = {
	.bitrate = 400000,
};

static struct i2c_board_info ge_imx53_devboard_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("ds1307", 0x68),
	},
	{
		I2C_BOARD_INFO("ar1020", 0x4d),	/* use emtrion-based driver */
//		I2C_BOARD_INFO("ar1020_i2c", 0x4d),	/* use microchip driver */
		.irq = gpio_to_irq(GPIO_TOUCH),
	},
};

/*
 * i2c3
 */
static struct imxi2c_platform_data ge_imx53_devboard_i2c3_data = {
	.bitrate = 400000,
};

static struct i2c_board_info ge_imx53_devboard_i2c3_board_info[] = {
};


/*
 * can1
 */
/* based on emtrion-settings */
static struct flexcan_platform_data ge_imx53_devboard_can1_data = {
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
 * can2
 */
/* based on emtrion-settings */
static struct flexcan_platform_data ge_imx53_devboard_can2_data = {
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
 * audio
 */
/*
 * tlv320aic3101 based (used on modified devboard)
 */
/* codec (i2c) */
static struct i2c_board_info ge_imx53_devboard_tlv320aic3101_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
};
/* glue (soc <-> codec) */
static struct platform_device ge_imx53_devboard_tlv320aic3101_device = {
	.name = "ge_imx53_tlv320aic3101",
};

/*
 * tlv320aic3110 based (used on devboard)
 */
/* codec (i2c) */
static struct i2c_board_info ge_imx53_devboard_tlv320aic3110_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("tlv320aic3110", 0x18),
	},
};
/* glue (soc <-> codec) */
static struct platform_device ge_imx53_devboard_tlv320aic3110_device = {
	.name = "ge_imx53_tlv320aic3110",
};

/* soc ssi (soc) */
static struct mxc_audio_platform_data ge_imx53_devboard_tlv320aic31xx_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
};
/* dummy data for ssi -> neccessary for imx-pcm */
static struct mxc_audio_platform_data ge_imx53_devboard_ssi_data = {
};

/* init */
static int __init ge_imx53_devboard_init_audio(void)
{
	int audio_codec_sel;

	/* determine which audio-drivers should be loaded by board-version */
	audio_codec_sel=(GE_IMX_BOARD_VERSION()&0x0f00)>>8;
	if(audio_codec_sel==0) {
		pr_info("%s - audio_codec_sel=%i - no audio codec\n",__FUNCTION__,audio_codec_sel);
	} else {
		/* register ssi */
		mxc_register_device(&mxc_ssi1_device, &ge_imx53_devboard_ssi_data);
		mxc_register_device(&mxc_ssi2_device, &ge_imx53_devboard_ssi_data);

		if(audio_codec_sel==1) {
			pr_info("%s - audio_codec_sel=%i - tlv320aic3110\n",__FUNCTION__,audio_codec_sel);
			i2c_register_board_info(1,ge_imx53_devboard_tlv320aic3110_i2c2_board_info,1);
			mxc_register_device(&ge_imx53_devboard_tlv320aic3110_device, 
				&ge_imx53_devboard_tlv320aic31xx_data);
		} else {
			pr_info("%s - audio_codec_sel=%i - tlv320aic3101\n",__FUNCTION__,audio_codec_sel);
			i2c_register_board_info(1,ge_imx53_devboard_tlv320aic3101_i2c2_board_info,1);
			mxc_register_device(&ge_imx53_devboard_tlv320aic3101_device, 
				&ge_imx53_devboard_tlv320aic31xx_data);
		}
	}
	return 0;
}

/*
 * BACKLIGHT: PWM 1
 * no frequency control from userspace
 */
static struct platform_pwm_backlight_data ge_imx53_devboard_pwm1_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 1000,
	.dft_brightness = 1000,
	.pwm_period_ns =  400000,
};

/*
 * PWM-Beeper
 */
static struct platform_device ge_imx53_devboard_beeper_device = {
	.name = "pwm-beeper",
};

/*
 * LED
 */
static struct gpio_led ge_imx53_devboard_leds[] = {
	{
		.name = "pwrext_en",
		.gpio = GPIO_PWREXT_EN,
		.active_low = 0,
		.retain_state_suspended = 1,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = NULL,
//		.default_trigger = "mmc1",
	},
	{
		.name = "stat_led",
		.gpio = GPIO_STAT_LED,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = NULL,
//		.default_trigger = "mmc1",
	},
	{
		.name = "lcd_en",
		.gpio = GPIO_LCD_EN,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
//		.default_trigger = NULL,
		.default_trigger = "backlight",
	}
};

static struct gpio_led_platform_data ge_imx53_devboard_led_data = {
	.num_leds	= ARRAY_SIZE(ge_imx53_devboard_leds),
	.leds		= ge_imx53_devboard_leds,
};

static struct platform_device ge_imx53_devboard_led_device = {
	.name = "leds-gpio",
};

/*
 * GRAPHICS
 * TODO
 */

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data ge_imx53_modul_vpu_data = {
//	.iram_enable = true,	/* TODO */
//	.iram_size = 0x14000,	/* TODO */
	.reset = mx5_vpu_reset,
};

/* for lvds */
static struct ldb_platform_data ge_imx53_modul_ldb_data = {
	.lvds_bg_reg = "VAUDIO",
	.ext_ref = 1,
};

static struct fb_videomode ge_imx53_modul_video_modes[] = {
	{
		.name="URT_UMSH_8089MD_3T",
		.refresh=60,		/* Hz */
		.xres=640,
		.yres=480,
		.pixclock=7,		/* ps ? */	/* 26MHz -> 60Hz */
		.left_margin=144,
		.right_margin=16,
		.upper_margin=35,
		.lower_margin=10,
		.hsync_len=30,
		.vsync_len=5,
		.sync=0,			/* FB_SYNC_HOR_HIGH_ACT, FB_SYNC_VERT_HIGH_ACT, FB_SYNC_CLK_LAT_FALL */
		.vmode=FB_VMODE_NONINTERLACED,
		.flag=0,
	},
	{
		.name="EDT_ET057007DHU",
		.refresh=60,
		.xres=640,
		.yres=480,
		.pixclock=7,
		.left_margin=144,
		.right_margin=16,
		.upper_margin=37,
		.lower_margin=8,
		.hsync_len=30,
		.vsync_len=3,
		.sync=0,
		.vmode=FB_VMODE_NONINTERLACED,
		.flag=0,
	},
};

static struct mxc_fb_platform_data ge_imx53_modul_mxcfb_data[] = {
	{
		.mode_str = "URT_UMSH_8089MD_3T",
		/* .interface_pix_fmt is set in ge_imx53_devboard_init_fb */
		.default_bpp = 32,
		.mode = ge_imx53_modul_video_modes,
		.num_modes = ARRAY_SIZE(ge_imx53_modul_video_modes),
	},
	{
		.mode_str = "EDT_ET057007DHU",
		/* .interface_pix_fmt is set in ge_imx53_devboard_init_fb */
		.default_bpp = 32,
		.mode = ge_imx53_modul_video_modes,
		.num_modes = ARRAY_SIZE(ge_imx53_modul_video_modes),
	},
};

extern struct resource ge_imx53_modul_mxcfb_resources[];
static int __init ge_imx53_devboard_init_fb(void)
{
	int lcd_type;
	int ipu_pix_fmt;

	/* 
	 * abort if not this board-type
	 */
	if(GE_IMX_BOARD_TYPE()!=GE_IMX_BOARD_TYPE_GE_IMX53_DEVBOARD)
		return 0;

	/* get type */
	gpio_request(GPIO_LCD_TYPE, "lcd_type");
	gpio_direction_input(GPIO_LCD_TYPE);
	lcd_type=gpio_get_value(GPIO_LCD_TYPE) ? 1 : 0;
	pr_info("%s - lcd_type=%i - \"%s\"\n",__FUNCTION__,lcd_type,ge_imx53_modul_mxcfb_data[lcd_type].mode_str);
	gpio_free(GPIO_LCD_TYPE);

	/* get pix format from board-version */
	ipu_pix_fmt=(GE_IMX_BOARD_VERSION()&0xf000)>>12;
	if(ipu_pix_fmt==0) {
		pr_info("%s - ipu_pix_fmt=%i - IPU_PIX_FMT_RGB666\n",__FUNCTION__,ipu_pix_fmt);
		ge_imx53_modul_mxcfb_data[lcd_type].interface_pix_fmt=IPU_PIX_FMT_RGB666;
	} else {
		pr_info("%s - ipu_pix_fmt=%i - IPU_PIX_FMT_RGB24\n",__FUNCTION__,ipu_pix_fmt);
		ge_imx53_modul_mxcfb_data[lcd_type].interface_pix_fmt=IPU_PIX_FMT_RGB24;
	}

	/* use di0 */
	mxc_fb_devices[0].num_resources = 1;//ARRAY_SIZE(mxcfb_resources);
	mxc_fb_devices[0].resource = ge_imx53_modul_mxcfb_resources;
	mxc_register_device(&mxc_fb_devices[0], &ge_imx53_modul_mxcfb_data[lcd_type]);

	/* set initial enable and free after (see leds) */
	gpio_request(GPIO_LCD_EN, "lcd_en");
	gpio_direction_output(GPIO_LCD_EN, 1);
	gpio_free(GPIO_LCD_EN);

	return 0;
}
device_initcall(ge_imx53_devboard_init_fb);

/*
 * Switches
 */

/* lcd_type */
static struct ge_gpio_pin ge_imx53_devboard_lcd_type_switch_map[] = {
	{ 
		.gpio=GPIO_LCD_TYPE,
		.mask=1<<0,
		.active_low=0
	},
};

static struct ge_gpio_switch ge_imx53_devboard_switches[] = {
	{
		.name = "lcd_type",
		.pins = ge_imx53_devboard_lcd_type_switch_map,
		.num_pins = ARRAY_SIZE(ge_imx53_devboard_lcd_type_switch_map),
	},
};

static struct ge_gpio_switch_platform_data ge_imx53_devboard_switch_data = {
	.num_switches   = ARRAY_SIZE(ge_imx53_devboard_switches),
	.switches       = ge_imx53_devboard_switches,
};

static struct platform_device ge_imx53_devboard_switch_device = {
       .name = "switches-gpio",
};

/*
 * peripheral init
 */
static void __init ge_imx53_devboard_periph_init(void)
{
	/* 
	 * mux settings 
	 */
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_devboard_pads,
				ARRAY_SIZE(ge_imx53_devboard_pads));

	/*
	 * USBH1
	 */
	/* USBH1_OC */
	gpio_request(GPIO_USBH1_OC, "usbh1-oc");
	gpio_direction_input(GPIO_USBH1_OC);
	/* USBH1_PWR */
	gpio_request(GPIO_USBH1_PWR, "usbh1-pwr");
	gpio_direction_output(GPIO_USBH1_PWR, 0);
	/* USBH1 register */
	mx5_set_host1_vbus_func(ge_imx53_devboard_usbh1_driver_vbus);
	mx5_usbh1_init();

	/*
	 * USBOTG
	 */
	/* USBOTG_OC */
	gpio_request(GPIO_USBOTG_OC, "otg-oc");
	gpio_direction_input(GPIO_USBOTG_OC);
	/* USBOTG_PWR */
	gpio_request(GPIO_USBOTG_PWR, "otg-pwr");
	gpio_direction_output(GPIO_USBOTG_PWR, 0);
	/* USBOTG register */
	mx5_set_otghost_vbus_func(ge_imx53_devboard_usbotg_driver_vbus);
	mx5_usb_dr_init();

	/*
	 * SD1
	 */
	/* SD1 CD GPIO */
	gpio_request(GPIO_SD1_CD, "sdhc1-cd");
	gpio_direction_input(GPIO_SD1_CD);
	/* SD1 WP GPIO */
	gpio_request(GPIO_SD1_WP, "sdhc1-wp");
	gpio_direction_input(GPIO_SD1_WP);
	/* SD1 CD IRQ */
	mxcsdhc1_device.resource[2].start = gpio_to_irq(GPIO_SD1_CD);
	mxcsdhc1_device.resource[2].end = gpio_to_irq(GPIO_SD1_CD);
	/* SD1 register */
	mxc_register_device(&mxcsdhc1_device, &ge_imx53_devboard_mmc1_data);

	/*
	 * SD2
	 */
	/* SD2 CD GPIO */
	gpio_request(GPIO_SD2_CD, "sdhc2-cd");
	gpio_direction_input(GPIO_SD2_CD);
	/* SD2 WP GPIO */
	gpio_request(GPIO_SD2_WP, "sdhc2-wp");
	gpio_direction_input(GPIO_SD2_WP);
	/* SD2 CD IRQ */
	mxcsdhc2_device.resource[2].start = gpio_to_irq(GPIO_SD2_CD);
	mxcsdhc2_device.resource[2].end = gpio_to_irq(GPIO_SD2_CD);
	/* SD2 register */
	mxc_register_device(&mxcsdhc2_device, &ge_imx53_devboard_mmc2_data);

	/*
	 * I2C2
	 */
	mxc_register_device(&mxci2c_devices[1], &ge_imx53_devboard_i2c2_data);
	i2c_register_board_info(1, ge_imx53_devboard_i2c2_board_info,
				ARRAY_SIZE(ge_imx53_devboard_i2c2_board_info));

	/*
	 * I2C3
	 */
	mxc_register_device(&mxci2c_devices[2], &ge_imx53_devboard_i2c3_data);
	i2c_register_board_info(2, ge_imx53_devboard_i2c3_board_info,
				ARRAY_SIZE(ge_imx53_devboard_i2c3_board_info));

	/*
	 * CAN1
	 */
	mxc_register_device(&mxc_flexcan0_device, &ge_imx53_devboard_can1_data);

	/*
	 * CAN2
	 */
	mxc_register_device(&mxc_flexcan1_device, &ge_imx53_devboard_can2_data);

	/*
	 * AUDIO
 	 */
	ge_imx53_devboard_init_audio();

	/*
	 * PWM
	 */
	mxc_register_device(&mxc_pwm1_device, NULL);
	mxc_register_device(&mxc_pwm2_device, NULL);

	/*
	 * Backlight: PWM1
	 */
	mxc_register_device(&mxc_pwm1_backlight_device,
		&ge_imx53_devboard_pwm1_backlight_data);

	/*
	 * PWM-Beeper: PWM2
	 */
	mxc_register_device(&ge_imx53_devboard_beeper_device, (void *)1UL);

	/*
	 * LEDS
	 */
	mxc_register_device(&ge_imx53_devboard_led_device, 
		&ge_imx53_devboard_led_data);

	/*
	 * GRAPHICS
	 */
	mxc_register_device(&mxc_ldb_device, &ge_imx53_modul_ldb_data);
	mxc_register_device(&mxcvpu_device, &ge_imx53_modul_vpu_data);
	mxc_register_device(&gpu_device, &gpu_data);

	/*
	 * Switches
	 */
	mxc_register_device(&ge_imx53_devboard_switch_device,
		&ge_imx53_devboard_switch_data);

	/*
	 * wakeup on touch pendown
	 */
	gpio_request(GPIO_TOUCH, "touch");
	gpio_direction_input(GPIO_TOUCH);
	set_irq_type(gpio_to_irq(GPIO_TOUCH),IRQ_TYPE_EDGE_RISING);
	enable_irq_wake(gpio_to_irq(GPIO_TOUCH));
	gpio_free(GPIO_TOUCH);
}

/*
 * board init
 */
void __init ge_imx53_devboard_init(void)
{
	pr_info("Ginzinger imx53_devboard (0x%X) Version 0x%X\n",GE_IMX_BOARD_TYPE(),GE_IMX_BOARD_VERSION());

	/* peripheral init */
	ge_imx53_devboard_periph_init();
}

