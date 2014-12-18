/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 *           Melchior FRANZ, Ginzinger electronic systems GmbH
 * @Descr: ge_imx-board specific code for ge_imx53_sigma2
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
 *	2013KW36 - melchior.franz:
 *		* begin implementation based on ge_imx53_mmi
 *
 *  ge_imx53_board configuration:
 *	mandatory base number: 0x00070000
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
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/i2c.h>
#include <linux/ge_leds-max6956.h>
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
#include <linux/ge_variables.h>
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
#include "ge_imx53_sigma2.h"
#include "ge_imx53_gpio_keys.h"

#define GPIO(bank, number) (((bank) - 1) * 32 + (number))

#define GPIO_TOUCH_INT		GPIO(1, 5)	/* GPIO1_GPIO5 */
#define GPIO_SD1_CD		GPIO(3, 13)	/* EIM_DA13 */
#define GPIO_LCD_EN		GPIO(3, 27)	/* EIM_D27 */
#define GPIO_USBH1_OC		GPIO(3, 30)	/* EIM_D30 */
#define GPIO_USBH1_PWR		GPIO(3, 31)	/* EIM_D31; USBH1_PEN */

#define GPIO_SWITCH_NOT_AUS	GPIO(1, 8)	/* GPIO1_GPIO8 */
#define GPIO_SWITCH_EKA		GPIO(3, 11)	/* EIM_DA11 */
#define GPIO_SWITCH_EKB		GPIO(3, 12)	/* EIM_DA12 */
#define GPIO_SWITCH_EKC		GPIO(3, 14)	/* EIM_DA14 */
#define GPIO_SWITCH_POW		GPIO(3, 15)	/* EIM_DA15 */
#define GPIO_SWITCH_STONE	GPIO(3, 16)	/* EIM_DA16 */
#define GPIO_SWITCH_DRESS	GPIO(3, 17)	/* EIM_DA17 */
#define GPIO_SWITCH_WAP		GPIO(3, 18)	/* EIM_DA18 */
#define GPIO_SWITCH_BAND	GPIO(3, 20)	/* EIM_DA20 */
#define GPIO_SWITCH_SEG		GPIO(3, 29)	/* EIM_DA29 */


/*
 * pad config
 */
static iomux_v3_cfg_t ge_imx53_sigma2_pads[] = {
	/*
	 * usb host - usbh1
	 */
	/* <AltMode Name="ALT6" BallNumber="W4" BallName="EIM_D30" PowerGroup="WEIM_SEC" Signal="USBOH3_USBH1_OC" IsExcluded="false" /> */
	MX53_PAD_EIM_D30__GPIO3_30,			/* gpio: GPIO_USBH1_OC */
	/* <AltMode Name="ALT6" BallNumber="W5" BallName="EIM_D31" PowerGroup="WEIM_SEC" Signal="USBOH3_USBH1_PWR" IsExcluded="false" /> */
	MX53_PAD_EIM_D31__GPIO3_31,			/* gpio: GPIO_USBH1_PWR */

	/*
	 * i2c2 (Touch)
	 */
	/* <AltMode Name="ALT4" BallNumber="F6" BallName="KEY_COL3" PowerGroup="KEYPAD" Signal="I2C2_SCL" Comment="Touch/RTC" IsExcluded="false" /> */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	/* <AltMode Name="ALT4" BallNumber="D4" BallName="KEY_ROW3" PowerGroup="KEYPAD" Signal="I2C2_SDA" Comment="Touch/RTC" IsExcluded="false" /> */
	MX53_PAD_KEY_ROW3__I2C2_SDA,

	/*
	 * i2c3 (LED Controller)
	 */
	/* <AltMode Name="ALT2" BallNumber="A6" BallName="GPIO_3" PowerGroup="GPIO" Signal="I2C3_SCL" Comment="LED-Controller" IsExcluded="false" /> */
	MX53_PAD_GPIO_3__I2C3_SCL,
	/* <AltMode Name="ALT2" BallNumber="B6" BallName="GPIO_6" PowerGroup="GPIO" Signal="I2C3_SDA" Comment="LED-Controller" IsExcluded="false" /> */
	MX53_PAD_GPIO_6__I2C3_SDA,

	/*
	 * can1
	 */
	/* <AltMode Name="ALT2" BallNumber="D5" BallName="KEY_ROW2" PowerGroup="KEYPAD" Signal="CAN1_RXCAN" IsExcluded="false" /> */
	MX53_PAD_KEY_ROW2__CAN1_RXCAN,
	/* <AltMode Name="ALT2" BallNumber="C4" BallName="KEY_COL2" PowerGroup="KEYPAD" Signal="CAN1_TXCAN" IsExcluded="false" /> */
	MX53_PAD_KEY_COL2__CAN1_TXCAN,

	/* sd1 - mmc-card */
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
	/* <AltMode Name="ALT1" BallNumber="AC7" BallName="EIM_DA13" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[13]" IsExcluded="false" /> */
	MX53_PAD_EIM_DA13__GPIO3_13,			/* gpio: GPIO_SD1_CD */

	/*
	 * GPIO
	 */
	/* <AltMode Name="ALT1" BallNumber="B5" BallName="GPIO_8" PowerGroup="GPIO" Signal="GPIO1_GPIO[8]" IsExcluded="false" /> */
	MX53_PAD_GPIO_8__GPIO1_8,			/* gpio: gpio: GPIO_SWITCH_NOT_AUS */
	/* <AltMode Name="ALT1" BallNumber="AC6" BallName="EIM_DA11" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[11]" IsExcluded="false" /> */
	MX53_PAD_EIM_DA11__GPIO3_11,			/* gpio: GPIO_SWITCH_EKA */
	/* <AltMode Name="ALT1" BallNumber="V10" BallName="EIM_DA12" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[12]" IsExcluded="false" /> */
	MX53_PAD_EIM_DA12__GPIO3_12,			/* gpio: GPIO_SWITCH_EKB */
	/* <AltMode Name="ALT1" BallNumber="Y10" BallName="EIM_DA14" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[14]" IsExcluded="false" /> */
	MX53_PAD_EIM_DA14__GPIO3_14,			/* gpio: GPIO_SWITCH_EKC */
	/* <AltMode Name="ALT1" BallNumber="AA9" BallName="EIM_DA15" PowerGroup="WEIM_MAIN__1" Signal="GPIO3_GPIO[15]" IsExcluded="false" /> */
	MX53_PAD_EIM_DA15__GPIO3_15,			/* gpio: GPIO_SWITCH_POW */
	/* <AltMode Name="ALT1" BallNumber="U6" BallName="EIM_D16" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[16]" IsExcluded="false" /> */
	MX53_PAD_EIM_D16__GPIO3_16,			/* gpio: GPIO_SWITCH_STONE */
	/* <AltMode Name="ALT1" BallNumber="U5" BallName="EIM_D17" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[17]" IsExcluded="false" /> */
	MX53_PAD_EIM_D17__GPIO3_17,			/* gpio: GPIO_SWITCH_DRESS */
	/* <AltMode Name="ALT1" BallNumber="V1" BallName="EIM_D18" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[18]" IsExcluded="false" /> */
	MX53_PAD_EIM_D18__GPIO3_18,			/* gpio: GPIO_SWITCH_WAP */
	/* <AltMode Name="ALT1" BallNumber="W1" BallName="EIM_D20" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[20]" IsExcluded="false" /> */
	MX53_PAD_EIM_D20__GPIO3_20,			/* gpio: GPIO_SWITCH_BAND */
	/* <AltMode Name="ALT1" BallNumber="AA2" BallName="EIM_D29" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[29]" IsExcluded="false" /> */
	MX53_PAD_EIM_D29__GPIO3_29,			/* gpio: GPIO_SWITCH_SEG */

	/* <AltMode Name="ALT1" BallNumber="A5" BallName="GPIO_5" PowerGroup="GPIO" Signal="GPIO1_GPIO[5]" IsExcluded="false" /> */
	(_MX53_PAD_GPIO_5__GPIO1_5|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* gpio: GPIO_TOUCH_INT */
	/* <AltMode Name="ALT1" BallNumber="V4" BallName="EIM_D27" PowerGroup="WEIM_SEC" Signal="GPIO3_GPIO[27]" Comment="LCD_EN" IsExcluded="false" /> */
	MX53_PAD_EIM_D27__GPIO3_27,						/* gpio: GPIO_LCD_EN */

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
};

/*
 * sd1 read write-protect
 */
static int ge_imx53_sigma2_sdhc_write_protect(struct device *dev)
{
	return 0;
}

/*
 * sd1 read card-detect
 */
static unsigned int ge_imx53_sigma2_sdhc_get_card_det_status(struct device *dev)
{
	if (to_platform_device(dev)->id == 0)
		return !gpio_get_value(GPIO_SD1_CD);

	return 0;
}

/*
 * sd1 driver data
 */
static struct mxc_mmc_platform_data ge_imx53_sigma2_mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
			| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = ge_imx53_sigma2_sdhc_get_card_det_status,
	.wp_status = ge_imx53_sigma2_sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

/*
 * usbh1 power
 */
static void ge_imx53_sigma2_usbh1_driver_vbus(bool on)
{
	gpio_set_value(GPIO_USBH1_PWR, on);
}

/*
 * i2c2
 */
static struct imxi2c_platform_data ge_imx53_sigma2_i2c2_data = {
	.bitrate = 400000,
};

static struct i2c_board_info ge_imx53_sigma2_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("ar1020", 0x4d), /* emtrion-based touch driver */
		.irq = gpio_to_irq(GPIO_TOUCH_INT),
	},
	{
		I2C_BOARD_INFO("ds1307", 0x68), /* RTC */
	},
};

/*
 * i2c3
 */
static struct max6956_platform_data max_platform_data_1 = {
	.leds = {
		{
			.id = 4,
			.name = "dress_red_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 5,
			.name = "dress_blue_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 6,
			.name = "wap_blue_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 7,
			.name = "wap_red_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 8,
			.name = "eka_blue_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 9,
			.name = "ekb_blue_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 10,
			.name = "ekb_red_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 11,
			.name = "ekb_blue_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 12,
			.name = "eka_green_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 13,
			.name = "ekb_green_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 14,
			.name = "ekb_red_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 15,
			.name = "ekb_green_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 16,
			.name = "ekc_blue_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 17,
			.name = "ekc_green_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 18,
			.name = "ekc_red_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 19,
			.name = "ekc_red_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 20,
			.name = "ekc_blue_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 21,
			.name = "ekc_green_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 22,
			.name = "eka_red_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 23,
			.name = "eka_red_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 24,
			.name = "eka_green_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 25,
			.name = "eka_blue_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 26,
			.name = "wap_green_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 27,
			.name = "wap_blue_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 28,
			.name = "wap_red_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 29,
			.name = "wap_green_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 30,
			.name = "dress_green_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
	},
};

static struct max6956_platform_data max_platform_data_2 = {
	.leds = {
		{
			.id = 4,
			.name = "power_blue_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 5,
			.name = "power_green_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 6,
			.name = "power_red_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 7,
			.name = "power_green_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 8,
			.name = "belt_green_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 9,
			.name = "belt_red_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 10,
			.name = "belt_blue_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 11,
			.name = "seg_green_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 12,
			.name = "belt_blue_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 13,
			.name = "belt_green_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 14,
			.name = "belt_red_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 15,
			.name = "seg_blue_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 16,
			.name = "seg_red_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 17,
			.name = "seg_green_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 18,
			.name = "seg_blue_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 19,
			.name = "seg_red_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 20,
			.name = "dress_red_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 21,
			.name = "dress_green_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 22,
			.name = "dress_blue_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 23,
			.name = "stone_green_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 24,
			.name = "stone_blue_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 25,
			.name = "stone_red_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 26,
			.name = "stone_red_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 27,
			.name = "stone_green_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 28,
			.name = "stone_blue_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 29,
			.name = "power_blue_2",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
		{
			.id = 30,
			.name = "power_red_1",
			.type = MAX6956_TYPE_LED,
			.state = MAX6956_OFF,
		},
	},
};

static struct imxi2c_platform_data ge_imx53_sigma2_i2c3_data = {
	.bitrate = 400000,
};

static struct i2c_board_info ge_imx53_sigma2_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("max6956", 0x40),
		.platform_data = &max_platform_data_1,
	},
	{
		I2C_BOARD_INFO("max6956", 0x45),
		.platform_data = &max_platform_data_2,
	},
};

/*
 * can1
 */
/* based on emtrion-settings */
static struct flexcan_platform_data ge_imx53_sigma2_can1_data = {
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
static struct platform_pwm_backlight_data ge_imx53_sigma2_pwm1_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 1000,
	.dft_brightness = 0,
//	.pwm_period_ns =  400000,	/* 2.5 KHz */
//	.pwm_period_ns =  100000,	/* 10 KHz */
	.pwm_period_ns =  1000000,	/* 1 KHz */
};

/*
 * LED
 */
static struct gpio_led ge_imx53_sigma2_leds[] = {
	{
		.name = "lcd_en",
		.gpio = GPIO_LCD_EN,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = "backlight",
	},
};

static struct gpio_led_platform_data ge_imx53_sigma2_led_data = {
	.num_leds = ARRAY_SIZE(ge_imx53_sigma2_leds),
	.leds = ge_imx53_sigma2_leds,
};

static struct platform_device ge_imx53_sigma2_led_device = {
	.name = "leds-gpio",
};

/*
 * Keys
 */
#define DEBOUNCE_INTERVAL 50 // ms
static struct gpio_keys_button ge_imx53_sigma2_keys[] = {
	{
		.desc = "emergency_stop",
		.gpio = GPIO_SWITCH_NOT_AUS,
		.code = BTN_0,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "easy_key_a",
		.gpio = GPIO_SWITCH_EKA,
		.code = BTN_1,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "easy_key_b",
		.gpio = GPIO_SWITCH_EKB,
		.code = BTN_2,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "easy_key_c",
		.gpio = GPIO_SWITCH_EKC,
		.code = BTN_3,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "power",
		.gpio = GPIO_SWITCH_POW,
		.code = BTN_4,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "stone",
		.gpio = GPIO_SWITCH_STONE,
		.code = BTN_5,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "dress",
		.gpio = GPIO_SWITCH_DRESS,
		.code = BTN_6,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "wap",
		.gpio = GPIO_SWITCH_WAP,
		.code = BTN_7,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "belt",
		.gpio = GPIO_SWITCH_BAND,
		.code = BTN_8,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "seg",
		.gpio = GPIO_SWITCH_SEG,
		.code = BTN_9,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = DEBOUNCE_INTERVAL,
		.can_disable = 0,
	},
	{
		.desc = "usb_overcurrent",
		.gpio = GPIO_USBH1_OC,
		.code = BTN_A,
		.type = EV_KEY,
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 0,
		.can_disable = 0,
	},
};

static unsigned int ge_imx53_sigma2_read_key_state(struct gpio_keys_button *btn, int size)
{
	int i, val;
	unsigned int ret = 0;
	const int bits_per_int = 32;
	if (size > bits_per_int) {
		printk(KERN_WARNING "%s: only %d gpio keys supported\n", __func__, bits_per_int);
		size = bits_per_int;
	}
	for (i = 0; i < size; i++, btn++) {
		ret = gpio_request(btn->gpio, __func__);
		if (ret) {
			printk(KERN_WARNING "%s: requesting gpio %d failed (%d)\n",
					__func__, btn->gpio, ret);
			continue;
		}
		gpio_direction_input(btn->gpio);
		val = gpio_get_value(btn->gpio);
		if (!!val ^ btn->active_low)
			ret |= 1 << i;
		gpio_free(btn->gpio);
	}
	return ret;
}

/*
 * Graphics
 */
extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data ge_imx53_sigma2_vpu_data = {
//	.iram_enable = true,	/* TODO */
//	.iram_size = 0x14000,	/* TODO */
	.reset = mx5_vpu_reset,
};

/* for lvds */
static struct ldb_platform_data ge_imx53_sigma2_ldb_data = {
	.lvds_bg_reg = "VAUDIO",
	.ext_ref = 1,
};

static struct fb_videomode ge_imx53_sigma2_video_mode = {
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
};

static struct mxc_fb_platform_data ge_imx53_sigma2_mxcfb_data = {
	.mode_str = "ET0700_800x480",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.default_bpp = 32,
	.mode = &ge_imx53_sigma2_video_mode,
	.num_modes = 1,
};

extern struct resource ge_imx53_modul_mxcfb_resources[];
static int __init ge_imx53_sigma2_init_fb(void)
{
	if (GE_IMX_BOARD_TYPE() != GE_IMX_BOARD_TYPE_GE_IMX53_SIGMA2)
		return 0; // check necessary because of device_initcall()

	pr_info("%s - \"%s\"\n", __func__, ge_imx53_sigma2_mxcfb_data.mode_str);
	/* use di0 */
	mxc_fb_devices[0].num_resources = 1;//ARRAY_SIZE(mxcfb_resources);
	mxc_fb_devices[0].resource = ge_imx53_modul_mxcfb_resources;
	mxc_register_device(&mxc_fb_devices[0], &ge_imx53_sigma2_mxcfb_data);

	mxc_register_device(&mxc_pwm1_backlight_device,
			&ge_imx53_sigma2_pwm1_backlight_data);

	/* set initial enable and free after (see leds) */
	gpio_request(GPIO_LCD_EN, "lcd_en");
	gpio_direction_output(GPIO_LCD_EN, 1);
	gpio_free(GPIO_LCD_EN);
	return 0;
}
device_initcall(ge_imx53_sigma2_init_fb);

static int __init ge_imx53_sigma2_init_touchscreen(void)
{
	gpio_request(GPIO_TOUCH_INT, "touch");
	gpio_direction_input(GPIO_TOUCH_INT);
	set_irq_type(gpio_to_irq(GPIO_TOUCH_INT), IRQ_TYPE_EDGE_RISING);
	enable_irq_wake(gpio_to_irq(GPIO_TOUCH_INT));
	gpio_free(GPIO_TOUCH_INT);
	return 0;
}

/*
 * Variables export
 */
static unsigned int ge_imx53_initial_key_state_get(void)
{
	static unsigned int initial_key_state;
	static bool initialized = false;
	if (!initialized) {
		initial_key_state = ge_imx53_sigma2_read_key_state(ge_imx53_sigma2_keys,
				ARRAY_SIZE(ge_imx53_sigma2_keys));
		initialized = true;
	}
	return initial_key_state;
}

static struct ge_variable ge_imx53_variables_map[] = {
	{
		.name = "initial_key_state",
		.get_value = ge_imx53_initial_key_state_get,
	},
};

static struct ge_variables_platform_data ge_imx53_sigma2_variables_data = {
	.num_variables = ARRAY_SIZE(ge_imx53_variables_map),
	.variables = ge_imx53_variables_map,
};

static struct platform_device ge_imx53_sigma2_variables_device = {
	.name = "ge_variables",
	.id = 1,
};

/*
 * peripheral init
 */
static void __init ge_imx53_sigma2_periph_init(void)
{
	/*
	 * mux settings
	 */
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_sigma2_pads,
			ARRAY_SIZE(ge_imx53_sigma2_pads));

	/*
	 * USBH1
	 */
	/* USBH1_PWR */
	gpio_request(GPIO_USBH1_PWR, "usbh1-pwr");
	gpio_direction_output(GPIO_USBH1_PWR, 0);
	/* USBH1 register */
	mx5_set_host1_vbus_func(ge_imx53_sigma2_usbh1_driver_vbus);
	mx5_usbh1_init();

	/*
	 * SD1 - mmc-card
	 */
	/* SD1 CD GPIO */
	gpio_request(GPIO_SD1_CD, "sdhc1-cd");
	gpio_direction_input(GPIO_SD1_CD);
	/* SD1 CD IRQ */
	mxcsdhc1_device.resource[2].start = gpio_to_irq(GPIO_SD1_CD);
	mxcsdhc1_device.resource[2].end = gpio_to_irq(GPIO_SD1_CD);
	/* SD1 register */
	mxc_register_device(&mxcsdhc1_device, &ge_imx53_sigma2_mmc1_data);

	/*
	 * I2C2
	 */
	mxc_register_device(&mxci2c_devices[1], &ge_imx53_sigma2_i2c2_data);
	i2c_register_board_info(1, ge_imx53_sigma2_i2c2_board_info,
			ARRAY_SIZE(ge_imx53_sigma2_i2c2_board_info));

	/*
	 * I2C3
	 */
	mxc_register_device(&mxci2c_devices[2], &ge_imx53_sigma2_i2c3_data);
	i2c_register_board_info(2, ge_imx53_sigma2_i2c3_board_info,
			ARRAY_SIZE(ge_imx53_sigma2_i2c3_board_info));

	/*
	 * CAN1
	 */
	mxc_register_device(&mxc_flexcan0_device, &ge_imx53_sigma2_can1_data);

	/*
	 * PWM
	 */
	mxc_register_device(&mxc_pwm1_device, NULL);

	/*
	 * Variables export
	 */
	mxc_register_device(&ge_imx53_sigma2_variables_device, &ge_imx53_sigma2_variables_data);

	/*
	 * LEDS
	 */
	mxc_register_device(&ge_imx53_sigma2_led_device,
			&ge_imx53_sigma2_led_data);

	/*
	 * Keys
	 */
	ge_imx53_gpio_keys_add(ge_imx53_sigma2_keys,
			ARRAY_SIZE(ge_imx53_sigma2_keys));

	/*
	 * Graphics
	 */
	mxc_register_device(&mxc_ldb_device, &ge_imx53_sigma2_ldb_data);
	mxc_register_device(&mxcvpu_device, &ge_imx53_sigma2_vpu_data);
	mxc_register_device(&gpu_device, &gpu_data);

	ge_imx53_sigma2_init_touchscreen();
}

/*
 * board init
 */
void __init ge_imx53_sigma2_init(void)
{
	pr_info("Ginzinger imx53_sigma2 (0x%X) Version 0x%X\n", GE_IMX_BOARD_TYPE(), GE_IMX_BOARD_VERSION());
	ge_imx53_sigma2_periph_init();
}
