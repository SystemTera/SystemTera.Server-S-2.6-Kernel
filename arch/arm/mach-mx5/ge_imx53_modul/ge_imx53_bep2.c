/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 * @Descr: ge_imx-board specific code for ge_imx53_bep2
 * @TODO: test edt-ft5x06 touch wakeup
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
 *  	2012KW33 - manfred.schlaegl:
 *  		* force search for alt-boot images(update), 
 *  			if no sd-card was inserted on boot
 *	2012KW23 - melchior.franz:
 *		* adaptation to new edt-ft5x06 touchscreen driver version
 *	2012KW18 - manfred.schlaegl:
 *		* support for BE-P2a (bep2_hardware_version==1)
 *			* inverted mmc-card-detect (CAUTION: workaround in mx_sdhci.c)
 *		* determine early bep2_hardware_version-value
 *	2012KW12 - manfred.schlaegl:
 *		* cleanup todo's
 *		* corrected names (module->bep2)
 *		* ipu interface 24bit but only 18 lines used:
 *			termination on unused ipu-display-lines added (emc)
 *	2012KW11 - manfred.schlaegl:
 *		* disabled hsync and vsync for urt-display (emc)
 *	2012KW10 - manfred.schlaegl:
 *		* documented correct setting for pixclk on displays
 *		* use display-line driver-strength and slew-rates 
 *			defined in ge_imx53_iomux.h
 *		* support for external display-spread-clock implemented and tested
 *			board-version bits 12-8 determines if spread-clock is used
 *			(0 .. use internal clk, else use external spread clock)
 *	2012KW07 - manfred.schlaegl: 
 *		* support for ocular ftc10005_58 spi touchscreen (irlbacher) 
 *			implemented and tested
 *			board-version bits 12-16 determines touchscreen type
 *		 	(0 .. ar1020(res), 1 .. ft5x06(cap), 2 .. ftc10005_58, else .. no touch)
 *		* support for spi1 implemented and tested
 *		* board-version bits 12-16 determines touchscreen type
 *		 	(0 .. ar1020(res), 1 .. ft5x06(cap), else .. no touch)
 *		* capacitive touchscreen ft5x06(GLYN) implemented and tested
 *		* GLYN Family Display implemented and tested
 *		* beeper: use pwm-beeper instead of pwm-leds driver
 *		* vin_on tested
 *		* lcd_type, dip-switch and hw-version over ge_switch 
 *			implemented and tested
 *	2012KW04 - manfred.schlaegl: 
 *		* use ge_imx53_gpio_keys for gpio-keys
 *		* can and uart1 shutdown / enable implemented and tested
 *		* uart1 implemented and tested
 *		* spread-clock input implemented
 *		* node-number switch implemented and tested
 *		* hardware-version switch implemented 
 *		* stat_led implemented and tested
 *		* graphics (inkl lcd_enable and lcd_type implemented and tested
 *		* pwm1/2 (backlight and summer) implemented and tested
 *		* can1 implemented and tested
 *		* ar1020 implemented and tested
 *		* ds1337 implemented and tested
 *		* i2c2 implemented and tested
 *		* mmc2 implemented and tested
 *		* usb host/device implemented and tested
 *		* do graphics init only, if this board-type is active
 *		* begin implementation based on ge_imx53_devboard
 *  @TODO:
 *	* wake on touch doesn't work with capacitive driver ft5x06
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
#include <linux/gpio_keys.h>
#include <linux/ge_switch.h>
#include <linux/input/edt-ft5x06.h>
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
#include <mach/clock.h>
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

#include "ge_imx53_gpio_keys.h"
#include "ge_imx53_iomux.h"
#include "ge_imx53_bep2.h"

#ifdef CONFIG_ALTERNATIVE_ROOTFS
/*
 * force search for alternative boot images
 */
extern unsigned int alt_boot_request_force;
#endif /* CONFIG_ALTERNATIVE_ROOTFS */


/* usb host - usbh1 - oc/pwr */
#define GPIO_USBH1_OC		(2*32 + 30)	/* GPIO_3_30 - EIM_D30 */
#define GPIO_USBH1_PWR		(2*32 + 31)	/* GPIO_3_31 - EIM_D31 */

/* usb otg - otg - oc/pwr */
#define GPIO_USBOTG_OC		(2*32 + 21)	/* GPIO_3_21 - EIM_D21 */
#define GPIO_USBOTG_PWR		(2*32 + 22)	/* GPIO_3_22 - EIM_D22 */

/* sd2 cs */
#define GPIO_SD2_CD		(0*32 + 4)	/* GPIO_1_4 - GPIO_4 */

/* stat_led */
#define GPIO_STAT_LED		(2*32 + 12)	/* GPIO_3_12 - EIM_DA12 */

/* lcd type */
#define GPIO_LCD_TYPE		(2*32 + 26)	/* GPIO_3_26 - EIM_D26 */
/* lcd enable */
#define GPIO_LCD_EN		(2*32 + 27)	/* GPIO_3_27 - EIM_D27 */

/* touch interrupt */
#define GPIO_TOUCH		(2*32 + 11)	/* GPIO_3_11 - EIM_DA11 */

/* hw-version switch */
#define GPIO_SWITCH_HW1		(0*32 + 5)	/* GPIO_1_5 - GPIO_5 */
#define GPIO_SWITCH_HW2		(0*32 + 7)	/* GPIO_1_7 - GPIO_7 */
#define GPIO_SWITCH_HW4		(0*32 + 8)	/* GPIO_1_8 - GPIO_8 */

/* node number switch */
#define GPIO_SWITCH_NM1		(5*32 + 2)	/* GPIO_6_2 - CSI0_DAT16 */
#define GPIO_SWITCH_NM2		(5*32 + 3)	/* GPIO_6_3 - CSI0_DAT17 */
#define GPIO_SWITCH_NM4		(5*32 + 4)	/* GPIO_6_4 - CSI0_DAT18 */

/* shutdown signals */
#define GPIO_UART1_NSHDN	(6*32 + 2)	/* GPIO_7_2 - PATA_INTRQ */
#define GPIO_CAN_SHDN		(6*32 + 3)	/* GPIO_7_3 - PATA_DIOR */

/* vin_on */
#define GPIO_VIN_ON		(2*32 + 17)	/* GPIO_3_17 - EIM_D17 */

/* edt-display */
#define GPIO_ETV_HARDRST	(2*32 + 18)	/* GPIO_3_18 - EIM_D18 */
#define GPIO_EVM_RST		(2*32 + 20)	/* GPIO_3_20 - EIM_D20 */
#define GPIO_EVM_WAKE		(2*32 + 29)	/* GPIO_3_29 - EIM_D29 */

/*
 * pad config
 */
static iomux_v3_cfg_t ge_imx53_bep2_pads[] = {

	/*
	 * usb host - usbh1
	 */
	/* <AltMode Name="ALT6" BallNumber="W4" BallName="EIM_D30" PowerGroup="WEIM_SEC" Signal="USBOH3_USBH1_OC" IsExcluded="false" /> */
	MX53_PAD_EIM_D30__GPIO3_30,			/* gpio: GPIO_USBH1_OC */

	/* <AltMode Name="ALT6" BallNumber="W5" BallName="EIM_D31" PowerGroup="WEIM_SEC" Signal="USBOH3_USBH1_PWR" IsExcluded="false" /> */
	MX53_PAD_EIM_D31__GPIO3_31,			/* gpio: GPIO_USBH1_PWR */

	/*
	 * usb otg - 
	 */
	/* <AltMode Name="ALT6" BallNumber="V3" BallName="EIM_D21" PowerGroup="WEIM_SEC" Signal="USBOH3_USBOTG_OC" IsExcluded="false" /> */
	MX53_PAD_EIM_D21__GPIO3_21,			/* gpio: GPIO_USBOTG_OC */
	/* <AltMode Name="ALT6" BallNumber="W2" BallName="EIM_D22" PowerGroup="WEIM_SEC" Signal="USBOH3_USBOTG_PWR" IsExcluded="false" /> */
	MX53_PAD_EIM_D22__GPIO3_22,			/* gpio: GPIO_USBOTG_PWR */

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
	(_MX53_PAD_GPIO_4__GPIO1_4|MUX_PAD_CTRL(MX53_47K_PULLUP_PAD_CTRL)),	/* gpio: GPIO_SD2_CD - 47 K pullup */

	/*
	 * i2c2
	 */
	/* <AltMode Name="ALT4" BallNumber="F6" BallName="KEY_COL3" PowerGroup="KEYPAD" Signal="I2C2_SCL" Comment="Audio" IsExcluded="false" /> */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	/* <AltMode Name="ALT4" BallNumber="D4" BallName="KEY_ROW3" PowerGroup="KEYPAD" Signal="I2C2_SDA" Comment="Audio" IsExcluded="false" /> */
	MX53_PAD_KEY_ROW3__I2C2_SDA,

	/*
	 * can1
	 */
	/* <AltMode Name="ALT2" BallNumber="D5" BallName="KEY_ROW2" PowerGroup="KEYPAD" Signal="CAN1_RXCAN" IsExcluded="false" /> */
	MX53_PAD_KEY_ROW2__CAN1_RXCAN,
	/* <AltMode Name="ALT2" BallNumber="C4" BallName="KEY_COL2" PowerGroup="KEYPAD" Signal="CAN1_TXCAN" IsExcluded="false" /> */
	MX53_PAD_KEY_COL2__CAN1_TXCAN,

	/*
	 * GPIO
	 */
	/* <AltMode Name="ALT1" BallNumber="AC6" BallName="EIM_DA11" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[11]" IsExcluded="false" />
	 * used as touch-pen-down interrupt for resistive touch internal pulldown because of level-interrupt
	 */
	(_MX53_PAD_EIM_DA11__GPIO3_11|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),
	/* <AltMode Name="ALT1" BallNumber="V10" BallName="EIM_DA12" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[12]" IsExcluded="false" /> 
	 * used as stat-led
	 */
	MX53_PAD_EIM_DA12__GPIO3_12,
	/* <AltMode Name="ALT1" BallNumber="U5" BallName="EIM_D17" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[17]" IsExcluded="false" /> 
	 * used as VIN_ON
	 */
	(_MX53_PAD_EIM_D17__GPIO3_17|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),
	/* <AltMode Name="ALT1" BallNumber="V5" BallName="EIM_D26" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[26]" Comment="LCD_TYPE" IsExcluded="false" /> 
	 * used as LCD_TYPE
	 */
	MX53_PAD_EIM_D26__GPIO3_26,
	/* <AltMode Name="ALT1" BallNumber="V4" BallName="EIM_D27" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[27]" Comment="LCD_EN" IsExcluded="false" /> 
	 * used as LCD_EN
	 */
	MX53_PAD_EIM_D27__GPIO3_27,
	/* HW 1,2,4 */
	(_MX53_PAD_GPIO_5__GPIO1_5|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),
	(_MX53_PAD_GPIO_7__GPIO1_7|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),
	(_MX53_PAD_GPIO_8__GPIO1_8|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),
	/* NM 1,2,4 */
	(_MX53_PAD_CSI0_DAT16__GPIO6_2|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),
	(_MX53_PAD_CSI0_DAT17__GPIO6_3|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),
	(_MX53_PAD_CSI0_DAT18__GPIO6_4|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),
	/* #IMX53_UART1_SHDN */
	(_MX53_PAD_PATA_INTRQ__GPIO7_2|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),
	/* IMX53_CAN_SHDN */
	(_MX53_PAD_PATA_DIOR__GPIO7_3|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),
	/* EDT-Display -> ETV_HARDRST */
	(_MX53_PAD_EIM_D18__GPIO3_18|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),
	/* EDT-Display -> EVM_RESET */
	(_MX53_PAD_EIM_D20__GPIO3_20|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),
	/* EDT-Display -> EVM_WAKE */
	(_MX53_PAD_EIM_D29__GPIO3_29|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),

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
	/* <AltMode Name="ALT0" BallNumber="H2" BallName="DISP0_DAT2" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[2]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="F4" BallName="DISP0_DAT20" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[20]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="C1" BallName="DISP0_DAT21" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[21]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="E3" BallName="DISP0_DAT22" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[22]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	/* <AltMode Name="ALT0" BallNumber="C3" BallName="DISP0_DAT23" PowerGroup="IPU_LCD__2" Signal="IPU_DISP0_DAT[23]" IsExcluded="false" /> */
	(_MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
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

	/* <AltMode Name="ALT0" BallNumber="G6" BallName="DISP0_DAT8" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[8]" IsExcluded="false" /> */
	//(_MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	(_MX53_PAD_DISP0_DAT8__GPIO4_29|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* unused -> disabled for emc */
	/* <AltMode Name="ALT0" BallNumber="E2" BallName="DISP0_DAT9" PowerGroup="IPU_LCD__1" Signal="IPU_DISP0_DAT[9]" IsExcluded="false" /> */
	//(_MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9|MUX_PAD_CTRL(MX53_DISPLAY_DATA_PAD_CTRL)),
	(_MX53_PAD_DISP0_DAT9__GPIO4_30|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* unused -> disabled for emc */

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
};

/*
 * hardware-version of be-p2-board
 */
static unsigned char ge_imx53_bep2_hardware_version;

/*
 * sd2 read write-protect
 */
static int ge_imx53_bep2_sdhc_write_protect(struct device *dev)
{
	int ret = 0;
	if (to_platform_device(dev)->id == 0) {
		/* mmc0: not supported */
		ret = 1;
	} else {
		/* mmc1: no support for wp on micro-sd */
		ret = 0;
	}

	return ret;
}

/*
 * sd2 read card-detect
 */
unsigned int ge_imx53_bep2_sdhc_get_card_det_status_sd2(void)
{
	int ret = 0;
	ret = gpio_get_value(GPIO_SD2_CD);
	if(ge_imx53_bep2_hardware_version>=1) {
		/* cd is inverted for BE-P2a */
		ret=!ret;
	} else {
		/* cd is not inverted BE-P2 */
		ret=ret;
	}

	return ret;
}

static unsigned int ge_imx53_bep2_sdhc_get_card_det_status(struct device *dev)
{
	if (to_platform_device(dev)->id == 0) {
		/* sd1 not supported */
		return 1;
	} else {
		/* sd2 */
		return ge_imx53_bep2_sdhc_get_card_det_status_sd2();
	}
}

/*
 * sd2 driver data
 */
static struct mxc_mmc_platform_data ge_imx53_bep2_mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = ge_imx53_bep2_sdhc_get_card_det_status,
	.wp_status = ge_imx53_bep2_sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

/*
 * usbh1 power
 */
static void ge_imx53_bep2_usbh1_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(GPIO_USBH1_PWR, 1);
	else
		gpio_set_value(GPIO_USBH1_PWR, 0);
}

/*
 * usbotg power
 */
static void ge_imx53_bep2_usbotg_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(GPIO_USBOTG_PWR, 1);
	else
		gpio_set_value(GPIO_USBOTG_PWR, 0);
}

/*
 * i2c2
 */
static struct imxi2c_platform_data ge_imx53_bep2_i2c2_data = {
	.bitrate = 400000,
};

static struct i2c_board_info ge_imx53_bep2_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("ds1307", 0x68),
	},
	/* touchscreen drivers are added in ge_imx53_bep2_init_touchscreen */
};

/*
 * can1
 */
/* based on emtrion-settings */
static struct flexcan_platform_data ge_imx53_bep2_can1_data = {
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
 * PWM-Beeper
 */
static struct platform_device ge_imx53_bep2_beeper_device = {
	.name = "pwm-beeper",
};

/*
 * LED
 */
static struct gpio_led ge_imx53_bep2_leds[] = {
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
	},
	{
		.name = "uart1_en",
		.gpio = GPIO_UART1_NSHDN,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = NULL,
	},
	{
		.name = "can_en",
		.gpio = GPIO_CAN_SHDN,
		.active_low = 1,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = NULL,
	},
	{
		.name = "lcd_etv_hardreset",
		.gpio = GPIO_ETV_HARDRST,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
//		.default_trigger = NULL,
		.default_trigger = "backlight",
	},
	{
		.name = "evm_wake",
		.gpio = GPIO_EVM_WAKE,
		.active_low = 0,
		/* do not put touch-controller to sleep while system-suspend (wake on touch) -> doesn't work */
		.retain_state_suspended = 1,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = NULL,
	},
};

static struct gpio_led_platform_data ge_imx53_bep2_led_data = {
	.num_leds	= ARRAY_SIZE(ge_imx53_bep2_leds),
	.leds		= ge_imx53_bep2_leds,
};

static struct platform_device ge_imx53_bep2_led_device = {
	.name = "leds-gpio",
};

/*
 * GRAPHICS
 * TODO
 */

/*
 * BACKLIGHT: PWM 1
 * no frequency control from userspace
 */
static struct platform_pwm_backlight_data ge_imx53_bep2_pwm1_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 1000,
	/* .dft_brightness is set in ge_imx53_bep2_init_fb */
	.pwm_period_ns =  400000,	/* 2.5 KHz */
//	.pwm_period_ns =  1000000,	/* 1 KHz */
//	.pwm_period_ns =  100000,	/* 10 KHz */
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data ge_imx53_bep2_vpu_data = {
//	.iram_enable = true,	/* TODO */
//	.iram_size = 0x14000,	/* TODO */
	.reset = mx5_vpu_reset,
};

/* for lvds */
static struct ldb_platform_data ge_imx53_bep2_ldb_data = {
	.lvds_bg_reg = "VAUDIO",
	.ext_ref = 1,
};

#define GE_IMX53_BEP2_PIXCLK	37887		/* ps -> 26MHz -> 60Hz */
//#define GE_IMX53_BEP2_PIXCLK	45464		/* ps -> 21MHz -> 50Hz */
static struct fb_videomode ge_imx53_bep2_video_modes[] = {
	{
		.name="URT_UMSH_8089MD_3T compatible",
//		.refresh=60,			/* Hz -> NOT USED */
		.xres=640,
		.yres=480,
		.pixclock=GE_IMX53_BEP2_PIXCLK,
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
		.name="EDT_ET057007DHU compatible",
//		.refresh=60,
		.xres=640,
		.yres=480,
		.pixclock=GE_IMX53_BEP2_PIXCLK,
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

static struct mxc_fb_platform_data ge_imx53_bep2_mxcfb_data[] = {
	{
		.mode_str = "URT_UMSH_8089MD_3T compatible",
		/* .interface_pix_fmt is set in ge_imx53_bep2_init_fb */
		.default_bpp = 32,
		.mode = ge_imx53_bep2_video_modes,
		.num_modes = ARRAY_SIZE(ge_imx53_bep2_video_modes),
	},
	{
		.mode_str = "EDT_ET057007DHU compatible",
		/* .interface_pix_fmt is set in ge_imx53_bep2_init_fb */
		.default_bpp = 32,
		.mode = ge_imx53_bep2_video_modes,
		.num_modes = ARRAY_SIZE(ge_imx53_bep2_video_modes),
	},
};

extern struct resource ge_imx53_modul_mxcfb_resources[];
static int __init ge_imx53_bep2_init_fb(void)
{
	int lcd_type;

	/* 
	 * abort if not this board-type
	 */
	if(GE_IMX_BOARD_TYPE()!=GE_IMX_BOARD_TYPE_GE_IMX53_BEP2)
		return 0;

	/* get type */
	gpio_request(GPIO_LCD_TYPE, "lcd_type");
	gpio_direction_input(GPIO_LCD_TYPE);
	lcd_type=gpio_get_value(GPIO_LCD_TYPE) ? 1 : 0;
	pr_info("%s - lcd_type=%i - \"%s\"\n",__FUNCTION__,lcd_type,ge_imx53_bep2_mxcfb_data[lcd_type].mode_str);
	gpio_free(GPIO_LCD_TYPE);
	ge_imx53_bep2_mxcfb_data[lcd_type].interface_pix_fmt=IPU_PIX_FMT_RGB24;

	/* use di0 */
	mxc_fb_devices[0].num_resources = 1;//ARRAY_SIZE(mxcfb_resources);
	mxc_fb_devices[0].resource = ge_imx53_modul_mxcfb_resources;
	mxc_register_device(&mxc_fb_devices[0], &ge_imx53_bep2_mxcfb_data[lcd_type]);

	/* register backlight-pwm */
	if(lcd_type) {
		/* EDT-Display */
		/* inverted brighness control */
		ge_imx53_bep2_pwm1_backlight_data.dft_brightness=0;
	} else {
		/* URT-Display */
		/* non-inverted brighness control */
		ge_imx53_bep2_pwm1_backlight_data.dft_brightness=
			ge_imx53_bep2_pwm1_backlight_data.max_brightness;

		/* disable hsync and vsync */
		mxc_iomux_v3_setup_pad(MX53_PAD_DI0_PIN2__GPIO4_18);
		mxc_iomux_v3_setup_pad(MX53_PAD_DI0_PIN3__GPIO4_19);
	}
	mxc_register_device(&mxc_pwm1_backlight_device,
		&ge_imx53_bep2_pwm1_backlight_data);

	/* set initial enable and free after (see leds) */
	gpio_request(GPIO_LCD_EN, "lcd_en");
	gpio_direction_output(GPIO_LCD_EN, 1);
	gpio_free(GPIO_LCD_EN);

	return 0;
}
device_initcall(ge_imx53_bep2_init_fb);

/*
 * external display spread clock
 */
static unsigned long ge_imx53_bep2_get_di0_ext_clock_rate(struct clk *clk)
{
	return 24000000;
}
static struct clk ge_imx53_bep2_di0_ext_clk = {
	.get_rate = ge_imx53_bep2_get_di0_ext_clock_rate,
};
static void __init ge_imx53_bep2_init_display(void)
{
	struct clk *ipu_di0_clk;

	/* determine, if spread-clock should be used */
	if((GE_IMX_BOARD_VERSION()&0x0f00)>>8) {
		/* spread-clock enabled for  */
		pr_info("%s - external spread-clock enabled\n",__FUNCTION__);

		/* mux setting for spread-clock input */
		mxc_iomux_v3_setup_pad(MX53_PAD_EIM_DA14__CCM_DI0_EXT_CLK);

		/* set clock-parent */
		ipu_di0_clk = clk_get(NULL, "ipu_di0_clk");
		clk_set_parent(ipu_di0_clk,&ge_imx53_bep2_di0_ext_clk);
		clk_put(ipu_di0_clk);
	} else {
		/* spread-clock is gpio-input */
		mxc_iomux_v3_setup_pad(MX53_PAD_EIM_DA14__GPIO3_14);
	}
}


/*
 * TOUCHSCREEN
 */

/* ar1020 i2c resistive touchscreen driver */
static struct i2c_board_info ge_imx53_bep2_ar1020_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("ar1020", 0x4d),	/* use emtrion-based driver */
		.irq = gpio_to_irq(GPIO_TOUCH),
	},
};

/* edt-ft5x06-i2c capacitive touchscreen driver */
static struct edt_ft5x06_platform_data ge_imx53_bep2_ft5x06_data = {
	.reset_pin=GPIO_EVM_RST,
	.irq_pin = GPIO_TOUCH,
};
static struct i2c_board_info ge_imx53_bep2_ft5x06_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("edt-ft5x06", 0x38),
		.platform_data=&ge_imx53_bep2_ft5x06_data,
	},
};

/* ftc10005_58 capacitive spi touchscreen driver (irlbacher) */
/* pad config */
static iomux_v3_cfg_t ge_imx53_bep2_spi_pads[] = {
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
	_MX53_PAD_CSI0_DAT7__GPIO5_25|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL), /* gpio */
};

#define GPIO_CSPI1_CS0		(4*32 + 25)	/* GPIO_5_25 - CSI0_DAT7 */

/* workaround for ecspi chipselect pin may not keep correct level when idle */
static void ge_imx53_bep2_spi_chipselect_active(int cspi_mode, int status,
					     int chipselect)
{
	/* ignore unsupported spi-interface */
	if(cspi_mode!=1)
		return;

	if(chipselect==1) {
		gpio_set_value(GPIO_CSPI1_CS0, 0);
	} else {
		gpio_set_value(GPIO_CSPI1_CS0, 1);
	}
}

static void ge_imx53_bep2_spi_chipselect_inactive(int cspi_mode, int status,
					       int chipselect)
{
	/* ignore unsupported spi-interface */
	if(cspi_mode!=1)
		return;

	if(chipselect==1) {
		gpio_set_value(GPIO_CSPI1_CS0, 1);
	} else {
		gpio_set_value(GPIO_CSPI1_CS0, 0);
	}
}

static struct mxc_spi_master ge_imx53_bep2_mxcspi1_data = {
	.maxchipselect = 1,
	.spi_version = 23,
	.chipselect_active = ge_imx53_bep2_spi_chipselect_active,
	.chipselect_inactive = ge_imx53_bep2_spi_chipselect_inactive,
};

static struct spi_board_info ge_imx53_bep2_mxcspi1_board_info[] = {
	{
		.modalias = "ftc10005_58",
//		.max_speed_hz = 4000000,	/* 4MHz (MAX!) */
		.max_speed_hz = 2000000,	/* 4MHz */
//		.max_speed_hz = 1500000,	/* 1.5MHz (broken!!) */
		.mode = SPI_MODE_1,
		.bus_num = 1,
		.chip_select = 0,
		.irq = gpio_to_irq(GPIO_TOUCH),
	},
};

/* init */
static int __init ge_imx53_bep2_init_touchscreen(void)
{
	int touch_type;

	/* determine which touch-drivers should be loaded by board-version */
	touch_type=(GE_IMX_BOARD_VERSION()&0xf000)>>12;
	switch(touch_type) {
		case 0:
			/* default touchscreen -> ar1020 (resistive) */
			pr_info("%s - touch_type=%i - ar1020 (resistive)\n",__FUNCTION__,touch_type);
			i2c_register_board_info(1,ge_imx53_bep2_ar1020_i2c2_board_info,1);
		break;
		case 1:
			pr_info("%s - touch_type=%i - ft5x06 (capacitive)\n",__FUNCTION__,touch_type);
			i2c_register_board_info(1,ge_imx53_bep2_ft5x06_i2c2_board_info,1);
			/* wakeup */
			gpio_request(GPIO_EVM_WAKE, "evm_wake");
			gpio_direction_output(GPIO_EVM_WAKE, 1);
			gpio_free(GPIO_EVM_WAKE);
		break;
		case 2:
			pr_info("%s - touch_type=%i - ftc10005_58 (capacitive - spi)\n",__FUNCTION__,touch_type);

			/* mux settings spi */
			mxc_iomux_v3_setup_multiple_pads(ge_imx53_bep2_spi_pads,
						ARRAY_SIZE(ge_imx53_bep2_spi_pads));

			/* request and disassert cs0 */
			gpio_request(GPIO_CSPI1_CS0, "cspi1_cs0");
			gpio_direction_output(GPIO_CSPI1_CS0,1);

			/* register spi */
			mxc_register_device(&mxcspi1_device, &ge_imx53_bep2_mxcspi1_data);

			/* register ftc10005_58 */
			spi_register_board_info(ge_imx53_bep2_mxcspi1_board_info,1);
		break;
		default:
			pr_info("%s - touch_type=%i - unknown - no touchscreen\n",__FUNCTION__,touch_type);
	}

	/*
	 * wakeup on touch pendown
	 * TODO: problem doesn't work with ft5x06
	 */
	gpio_request(GPIO_TOUCH, "touch");
	gpio_direction_input(GPIO_TOUCH);
//	set_irq_type(gpio_to_irq(GPIO_TOUCH),IRQ_TYPE_EDGE_RISING);
	set_irq_type(gpio_to_irq(GPIO_TOUCH),IRQ_TYPE_EDGE_BOTH);
	enable_irq_wake(gpio_to_irq(GPIO_TOUCH));
	gpio_free(GPIO_TOUCH);

	return 0;
}

/*
 * Keys
 */
static struct gpio_keys_button ge_imx53_bep2_keys[] = {
	/*
	 * VIN_ON is key
	 */
	{
		.desc="vin_on",
		.gpio=GPIO_VIN_ON,
		.code=KEY_POWER, //KEY_BATTERY, KEY_POWER2
		.type=EV_KEY,
		.active_low=0,
		.wakeup=1,
		.debounce_interval=0,
		.can_disable=0,
	},
};


/*
 * Switches
 */

/* lcd_type */
static struct ge_gpio_pin ge_imx53_bep2_lcd_type_switch_map[] = {
	{ 
		.gpio=GPIO_LCD_TYPE,
		.mask=1<<0,
		.active_low=0
	},
};

/* hw-version */
static struct ge_gpio_pin ge_imx53_bep2_hardware_version_switch_map[] = {
	{ 
		.gpio=GPIO_SWITCH_HW1,
		.mask=1<<0,
		.active_low=1
	},
	{ 
		.gpio=GPIO_SWITCH_HW2,
		.mask=1<<1,
		.active_low=1
	},
	{ 
		.gpio=GPIO_SWITCH_HW4,
		.mask=1<<2,
		.active_low=1
	},
};

/* node number */
static struct ge_gpio_pin ge_imx53_bep2_node_number_switch_map[] = {
	{ 
		.gpio=GPIO_SWITCH_NM1,
		.mask=1<<0,
		.active_low=1
	},
	{ 
		.gpio=GPIO_SWITCH_NM2,
		.mask=1<<1,
		.active_low=1
	},
	{ 
		.gpio=GPIO_SWITCH_NM4,
		.mask=1<<2,
		.active_low=1
	},
};

static struct ge_gpio_switch ge_imx53_bep2_switches[] = {
	{
		.name = "lcd_type",
		.pins = ge_imx53_bep2_lcd_type_switch_map,
		.num_pins = ARRAY_SIZE(ge_imx53_bep2_lcd_type_switch_map),
	},
	{
		.name = "bep2_hardware_version",
		.pins = ge_imx53_bep2_hardware_version_switch_map,
		.num_pins = ARRAY_SIZE(ge_imx53_bep2_hardware_version_switch_map),
	},
	{
		.name = "bep2_hex_switch",
		.pins = ge_imx53_bep2_node_number_switch_map,
		.num_pins = ARRAY_SIZE(ge_imx53_bep2_node_number_switch_map),
	},
};

static struct ge_gpio_switch_platform_data ge_imx53_bep2_switch_data = {
	.num_switches   = ARRAY_SIZE(ge_imx53_bep2_switches),
	.switches       = ge_imx53_bep2_switches,
};

static struct platform_device ge_imx53_bep2_switch_device = {
       .name = "switches-gpio",
};



/*
 * peripheral init
 */
static void __init ge_imx53_bep2_periph_init(void)
{
	int i;

	/* 
	 * mux settings 
	 */
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_bep2_pads,
				ARRAY_SIZE(ge_imx53_bep2_pads));

	/*
	 * get bep2_hardware_version early for usage in local file
	 */
	ge_imx53_bep2_hardware_version=0;
	for(i=0;i<ARRAY_SIZE(ge_imx53_bep2_hardware_version_switch_map);i++) {
		int tmp;

		gpio_request(ge_imx53_bep2_hardware_version_switch_map[i].gpio, "");
		gpio_direction_input(ge_imx53_bep2_hardware_version_switch_map[i].gpio);

		tmp=gpio_get_value(ge_imx53_bep2_hardware_version_switch_map[i].gpio) ? 1 : 0;
		tmp=ge_imx53_bep2_hardware_version_switch_map[i].active_low ? !tmp : tmp;
		ge_imx53_bep2_hardware_version|=(tmp*ge_imx53_bep2_hardware_version_switch_map[i].mask);

		gpio_free(ge_imx53_bep2_hardware_version_switch_map[i].gpio);
	}
	pr_info("bep2_hardware_version=0x%X\n",ge_imx53_bep2_hardware_version);

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
	mx5_set_host1_vbus_func(ge_imx53_bep2_usbh1_driver_vbus);
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
	mx5_set_otghost_vbus_func(ge_imx53_bep2_usbotg_driver_vbus);
	mx5_usb_dr_init();

	/*
	 * SD2
	 */
	/* SD2 CD GPIO */
	gpio_request(GPIO_SD2_CD, "sdhc2-cd");
	gpio_direction_input(GPIO_SD2_CD);
	/* SD2 CD IRQ */
	mxcsdhc2_device.resource[2].start = gpio_to_irq(GPIO_SD2_CD);
	mxcsdhc2_device.resource[2].end = gpio_to_irq(GPIO_SD2_CD);
	/* SD2 register */
	mxc_register_device(&mxcsdhc2_device, &ge_imx53_bep2_mmc2_data);
#ifdef CONFIG_ALTERNATIVE_ROOTFS
	/* force alt_boot_request, if no card is detected */
	if(ge_imx53_bep2_sdhc_get_card_det_status_sd2()) {
		alt_boot_request_force=1;
	}
#endif /* CONFIG_ALTERNATIVE_ROOTFS */

	/*
	 * I2C2
	 */
	mxc_register_device(&mxci2c_devices[1], &ge_imx53_bep2_i2c2_data);
	i2c_register_board_info(1, ge_imx53_bep2_i2c2_board_info,
				ARRAY_SIZE(ge_imx53_bep2_i2c2_board_info));

	/*
	 * CAN1
	 */
	mxc_register_device(&mxc_flexcan0_device, &ge_imx53_bep2_can1_data);

	/*
	 * PWM
	 */
	mxc_register_device(&mxc_pwm1_device, NULL);
	mxc_register_device(&mxc_pwm2_device, NULL);

	/*
	 * PWM-Beeper: PWM2
	 */
	mxc_register_device(&ge_imx53_bep2_beeper_device, (void *)1UL);

	/*
	 * LEDS
	 */
	mxc_register_device(&ge_imx53_bep2_led_device, 
		&ge_imx53_bep2_led_data);

	/*
	 * GRAPHICS
	 */
	mxc_register_device(&mxc_ldb_device, &ge_imx53_bep2_ldb_data);
	mxc_register_device(&mxcvpu_device, &ge_imx53_bep2_vpu_data);
	mxc_register_device(&gpu_device, &gpu_data);
	ge_imx53_bep2_init_display();

	/*
	 * keys
	 */
	ge_imx53_gpio_keys_add(
		ge_imx53_bep2_keys,
		ARRAY_SIZE(ge_imx53_bep2_keys)
	);

	/*
	 * Switches
	 */
	mxc_register_device(&ge_imx53_bep2_switch_device,
		&ge_imx53_bep2_switch_data);

	/*
	 * Touchscreen
	 */
	ge_imx53_bep2_init_touchscreen();

	/* deassert reset for edt-family displays */
	gpio_request(GPIO_ETV_HARDRST, "etv_hardreset");
	gpio_direction_output(GPIO_ETV_HARDRST, 1);
	gpio_free(GPIO_ETV_HARDRST);

	/*
	 * wakeup on vin_on state change
	 */
	gpio_request(GPIO_VIN_ON, "vin_on");
	gpio_direction_input(GPIO_VIN_ON);
	set_irq_type(gpio_to_irq(GPIO_VIN_ON),IRQ_TYPE_EDGE_BOTH);
	enable_irq_wake(gpio_to_irq(GPIO_VIN_ON));
	gpio_free(GPIO_VIN_ON);
}

/*
 * board init
 */
void __init ge_imx53_bep2_init(void)
{
	pr_info("Ginzinger imx53_bep2 (0x%X) Version 0x%X\n",GE_IMX_BOARD_TYPE(),GE_IMX_BOARD_VERSION());

	/* peripheral init */
	ge_imx53_bep2_periph_init();
}

