/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 * @Descr: ge_imx-board specific code for ge_imx53_tera_com
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
 *  	2013KW51 - manfred.schlaegl:
 *  		* set rs485 receiver buffer interrupt threshold 
 *  			from 16/32 (default) to 1/32 
 *  			(reduced overflow on 1MBaud)
 *  	2013KW50 - manfred.schlaegl:
 *  		* added rs485 sender enable delay (8us) for 1MBaud/s
 *  	2012KW47 - manfred.schlaegl:
 *  		* half-duplex-mode for uart3 (rs485)
 *  	2012KW35 - manfred.schlaegl:
 *  		* gpio-watchdog implemented and tested
 *  		* uart5(mbus) tested
 *  		* uart4(mbus) tested
 *  		* uart3(rs485) tested
 *  		* spi-adc implemented and tested
 *  		* spi1 implemented and tested
 *  		* extio tested
 *  		* added missing led 5mA_en_adin7
 *  	2012KW34 - manfred.schlaegl:
 *  		* uart4(mbus) implemented
 *  		* uart5(knx) implemented
 *  		* uart3(rs485) implemented and tested
 *  		* rs485-shutdown implemented and tested
 *  		* uart1(rs232) implemented and tested
 *  		* adc-switch outputs implemented and tested
 *  		* digital in (counter) implemented and tested
 *  		* watchdog implemented and tested
 *  			use led-interface and toggle
 *  		* leds implemented and tested
 *  		* rtc implemented and tested
 *  		* i2c2 implemented and tested
 *  		* sd2 implemented and tested
 *  		* usb1 and usb2 implemented and tested
 *  		* begin implementation based on ge_imx53_tera_com
 *  
 *  @TODO:
 *  	* verify
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
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/fec.h>
#include <linux/gpio_keys.h>
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
#include <linux/ge_counter.h>
#include <linux/gpio_wdt.h>

#include "../crm_regs.h"
#include "../devices.h"
#include "../usb.h"
#include "../serial.h"

#include "ge_imx53_gpio_keys.h"
#include "ge_imx53_iomux.h"
#include "ge_imx53_tera_com.h"

#include <linux/leds.h>
#include <linux/timer.h>
#include <linux/fs.h>

/* usb host - usbh1 - oc/pwr */
#define GPIO_USBH1_OC		(2*32 + 30)	/* GPIO_3_30 - EIM_D30 */
#define GPIO_USBH1_PWR		(2*32 + 31)	/* GPIO_3_31 - EIM_D31 */

/* usb otg - otg - oc/pwr */
#define GPIO_USBOTG_OC		(2*32 + 21)	/* GPIO_3_21 - EIM_D21 */
#define GPIO_USBOTG_PWR		(2*32 + 22)	/* GPIO_3_22 - EIM_D22 */

/* sd2 cd */
#define GPIO_SD2_CD		(0*32 + 4)	/* GPIO_1_4 - GPIO_4 */

/* spi1 chipselect */
#define GPIO_CSPI1_CS0		(4*32 + 25)	/* GPIO_5_25 - CSI0_DAT7 */

/* rtc-goldcap load */
#define GPIO_VBAT_LOAD		(0*32 + 5)	/* GPIO_1_5 - GPIO_5 */

/* rs485 shutdown */
#define GPIO_RS485_SHDN		(6*32 + 8)	/* GPIO_7_8 - PATA_DA_2 */

/* rs485 rw */
#define GPIO_RS485RW		(6*32 + 7)	/* GPIO_7_7 - PATA_DA_1 */

/* leds */
	#define GPIO_LED1_ROT		(0*32 + 7)	/* GPIO_1_7 - GPIO_7 */
	#define GPIO_LED1_GRUEN		(0*32 + 8)	/* GPIO_1_8 - GPIO_8 */
	#define GPIO_LED2_ROT		(4*32 + 20)	/* GPIO_5_20 - CSI0_DATA_EN */
	#define GPIO_LED2_GRUEN		(4*32 + 19)	/* GPIO_5_19 - CSI0_MCLK */
	#define GPIO_LED3_ROT		(4*32 + 18)	/* GPIO_5_18 - CSI0_PIXCLK */
	#define GPIO_LED3_GRUEN		(4*32 + 21)	/* GPIO_5_21 - GPIO_VSYNC */

	/* adc load enables (leds) */
	#define GPIO_5MA_EN_ADIN0	(2*32 + 11)	/* GPIO_3_11 - EIM_DA11 */
	#define GPIO_5MA_EN_ADIN1	(2*32 + 12)	/* GPIO_3_12 - EIM_DA12 */
	#define GPIO_5MA_EN_ADIN2	(2*32 + 14)	/* GPIO_3_14 - EIM_DA14 */
	#define GPIO_5MA_EN_ADIN3	(2*32 + 15)	/* GPIO_3_15 - EIM_DA15 */
	#define GPIO_5MA_EN_ADIN4	(2*32 + 16)	/* GPIO_3_16 - EIM_D16 */
	#define GPIO_5MA_EN_ADIN5	(2*32 + 17)	/* GPIO_3_17 - EIM_D17 */
	#define GPIO_5MA_EN_ADIN6	(2*32 + 18)	/* GPIO_3_18 - EIM_D18 */
	#define GPIO_5MA_EN_ADIN7	(2*32 + 20)	/* GPIO_3_20 - EIM_D20 */

	/* extio */
	#define GPIO_EXTIO0		(2*32 + 13)	/* GPIO_3_13 - EIM_DA13 */
	#define GPIO_EXTIO1		(0*32 + 20)	/* GPIO_1_20 - SD1_CLK */
	#define GPIO_EXTIO2		(0*32 + 18)	/* GPIO_1_18 - SD1_CMD */
	#define GPIO_EXTIO3		(0*32 + 16)	/* GPIO_1_16 - SD1_DATA0 */
	#define GPIO_EXTIO4		(0*32 + 17)	/* GPIO_1_17 - SD1_DATA1 */
	#define GPIO_EXTIO5		(0*32 + 19)	/* GPIO_1_19 - SD1_DATA2 */
	#define GPIO_EXTIO6		(0*32 + 21)	/* GPIO_1_21 - SD1_DATA3 */

	/* watchdog trigger */
	#define GPIO_WDI		(2*32 + 29)	/* GPIO_3_29 - EIM_D29 */

	/* digital inputs (counter) */
	#define GPIO_DI0		(5*32 + 2)	/* GPIO_6_2 - CSI0_DAT16 */
	#define GPIO_DI1		(5*32 + 3)	/* GPIO_6_3 - CSI0_DAT17 */
	#define GPIO_DI2		(5*32 + 4)	/* GPIO_6_4 - CSI0_DAT18 */
	#define GPIO_DI3		(5*32 + 5)	/* GPIO_6_5 - CSI0_DAT19 */

	/*
	 * pad config
	 */
	static iomux_v3_cfg_t ge_imx53_tera_com_pads[] = {

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
		 * GPIO
		 */
		/* GPIO_VBAT_LOAD */
		(_MX53_PAD_GPIO_5__GPIO1_5|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),		/* GPIO_VBAT_LOAD */

		/* leds */
		(_MX53_PAD_GPIO_7__GPIO1_7|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),		/* GPIO_LED1_ROT */
		(_MX53_PAD_GPIO_8__GPIO1_8|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),		/* GPIO_LED1_GRUEN */
		(_MX53_PAD_CSI0_DATA_EN__GPIO5_20|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_LED2_ROT */
		(_MX53_PAD_CSI0_MCLK__GPIO5_19|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_LED2_GRUEN */
		(_MX53_PAD_CSI0_PIXCLK__GPIO5_18|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_LED3_ROT */
		(_MX53_PAD_CSI0_VSYNC__GPIO5_21|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_LED3_GRUEN */

		/* adc load enables (leds) */
		(_MX53_PAD_EIM_DA11__GPIO3_11|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_5MA_EN_ADIN0 */
		(_MX53_PAD_EIM_DA12__GPIO3_12|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_5MA_EN_ADIN1 */
		(_MX53_PAD_EIM_DA14__GPIO3_14|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_5MA_EN_ADIN2 */
		(_MX53_PAD_EIM_DA15__GPIO3_15|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_5MA_EN_ADIN3 */
		(_MX53_PAD_EIM_D16__GPIO3_16|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_5MA_EN_ADIN4 */
		(_MX53_PAD_EIM_D17__GPIO3_17|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_5MA_EN_ADIN5 */
		(_MX53_PAD_EIM_D18__GPIO3_18|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_5MA_EN_ADIN6 */	
		(_MX53_PAD_EIM_D20__GPIO3_20|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_5MA_EN_ADIN7 */

		/* extio */
		(_MX53_PAD_EIM_DA13__GPIO3_13|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_EXTIO0 */
		(_MX53_PAD_SD1_CLK__GPIO1_20|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_EXTIO1 */
		(_MX53_PAD_SD1_CMD__GPIO1_18|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_EXTIO2 */
		(_MX53_PAD_SD1_DATA0__GPIO1_16|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_EXTIO3 */
		(_MX53_PAD_SD1_DATA1__GPIO1_17|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_EXTIO4 */
		(_MX53_PAD_SD1_DATA2__GPIO1_19|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_EXTIO5 */
		(_MX53_PAD_SD1_DATA3__GPIO1_21|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_EXTIO6 */

		/* watchdog trigger */ 
		(_MX53_PAD_EIM_D29__GPIO3_29|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_WDI */

		/* digital inputs (counter) */
		(_MX53_PAD_CSI0_DAT16__GPIO6_2|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_DI0 */
		(_MX53_PAD_CSI0_DAT17__GPIO6_3|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_DI1 */
		(_MX53_PAD_CSI0_DAT18__GPIO6_4|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_DI2 */
		(_MX53_PAD_CSI0_DAT19__GPIO6_5|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_DI3 */


		/*
		 * UART1
		 */
		/* <AltMode Name="ALT3" BallNumber="K2" BallName="PATA_RESET_B" PowerGroup="PATA__0" Signal="UART1_CTS" IsExcluded="false" /> */
		MX53_PAD_PATA_RESET_B__UART1_DTE_CTS,
		/* <AltMode Name="ALT3" BallNumber="K1" BallName="PATA_IORDY" PowerGroup="PATA__0" Signal="UART1_RTS" IsExcluded="false" /> */
		MX53_PAD_PATA_IORDY__UART1_DTE_RTS,
		/* <AltMode Name="ALT3" BallNumber="J2" BallName="PATA_DMACK" PowerGroup="PATA__0" Signal="UART1_RXD_MUX" Comment="bootfähig bei diesem Mapping? nein" IsExcluded="false" /> */
		MX53_PAD_PATA_DMACK__UART1_DTE_TXD_MUX,	/* pad is named RXD, but in DTE-Mode function is TXD */
		/* <AltMode Name="ALT3" BallNumber="J3" BallName="PATA_DIOW" PowerGroup="PATA__0" Signal="UART1_TXD_MUX" IsExcluded="false" /> */
		MX53_PAD_PATA_DIOW__UART1_DTE_RXD_MUX,	/* pad is named TXD, but in DTE-Mode function is RXD */

		/*
		 * UART3 (rs485)
		 */
		(_MX53_PAD_PATA_DA_2__GPIO7_8|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),		/* GPIO_RS485_SHDN */
		(_MX53_PAD_PATA_DA_1__GPIO7_7|MUX_PAD_CTRL(MX53_100K_PULLDOWN_PAD_CTRL)),	/* GPIO_RS485RW */
		/* <AltMode Name="ALT4" BallNumber="L5" BallName="PATA_CS_0" PowerGroup="PATA__0" Signal="UART3_TXD_MUX" IsExcluded="false" /> */
		MX53_PAD_PATA_CS_0__UART3_DCE_TXD_MUX,
		/* <AltMode Name="ALT4" BallNumber="L2" BallName="PATA_CS_1" PowerGroup="PATA__0" Signal="UART3_RXD_MUX" IsExcluded="false" /> */
		MX53_PAD_PATA_CS_1__UART3_DCE_RXD_MUX,

		/*
		 * UART4 (mbus)
		 */
		/* <AltMode Name="ALT2" BallNumber="T3" BallName="CSI0_DAT12" PowerGroup="IPU_CSI" Signal="UART4_TXD_MUX" Comment="Doppelbelegt mit Kamera (CSIO)" IsExcluded="false" /> */
		MX53_PAD_CSI0_DAT12__UART4_DCE_TXD_MUX,
		/* <AltMode Name="ALT2" BallNumber="T6" BallName="CSI0_DAT13" PowerGroup="IPU_CSI" Signal="UART4_RXD_MUX" Comment="Doppelbelegt mit Kamera (CSIO)" IsExcluded="false" /> */
		MX53_PAD_CSI0_DAT13__UART4_DCE_RXD_MUX,

		/*
		 * UART5 (knx)
		 */
		/* <AltMode Name="ALT2" BallNumber="U1" BallName="CSI0_DAT14" PowerGroup="IPU_CSI" Signal="UART5_TXD_MUX" Comment="Doppelbelegt mit Kamera (CSIO)" IsExcluded="false" /> */
		MX53_PAD_CSI0_DAT14__UART5_DCE_TXD_MUX,
		/* <AltMode Name="ALT2" BallNumber="U2" BallName="CSI0_DAT15" PowerGroup="IPU_CSI" Signal="UART5_RXD_MUX" Comment="Doppelbelegt mit Kamera (CSIO)" IsExcluded="false" /> */
		MX53_PAD_CSI0_DAT15__UART5_DCE_RXD_MUX,

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

	/*
	 * sd2 read write-protect
	 */
	static int ge_imx53_tera_com_sdhc_write_protect(struct device *dev)
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
	static unsigned int ge_imx53_tera_com_sdhc_get_card_det_status(struct device *dev)
	{
		if (to_platform_device(dev)->id == 0) {
			/* sd1 not supported */
			return 1;
		} else {
			/* sd2 */
			return !gpio_get_value(GPIO_SD2_CD);
		}
	}

	/*
	 * sd2 driver data
	 */
	static struct mxc_mmc_platform_data ge_imx53_tera_com_mmc2_data = {
		.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
			| MMC_VDD_31_32,
		.caps = MMC_CAP_4_BIT_DATA,
		.min_clk = 400000,
		.max_clk = 50000000,
		.card_inserted_state = 0,
		.status = ge_imx53_tera_com_sdhc_get_card_det_status,
		.wp_status = ge_imx53_tera_com_sdhc_write_protect,
		.clock_mmc = "esdhc_clk",
	};

	/*
	 * usbh1 power
	 */
	static void ge_imx53_tera_com_usbh1_driver_vbus(bool on)
	{
		if (on)
			gpio_set_value(GPIO_USBH1_PWR, 1);
		else
			gpio_set_value(GPIO_USBH1_PWR, 0);
	}

	/*
	 * usbotg power
	 */
	static void ge_imx53_tera_com_usbotg_driver_vbus(bool on)
	{
		if (on)
			gpio_set_value(GPIO_USBOTG_PWR, 1);
		else
			gpio_set_value(GPIO_USBOTG_PWR, 0);
	}

	/*
	 * i2c2
	 */
	static struct imxi2c_platform_data ge_imx53_tera_com_i2c2_data = {
		.bitrate = 400000,
	};

	static struct i2c_board_info ge_imx53_tera_com_i2c2_board_info[] = {
		{
			I2C_BOARD_INFO("ds1307", 0x68),
		},
	};

	/*
	 * LED
	 */
	static struct gpio_led ge_imx53_tera_com_leds[] = {
	{
		.name = "vbat_load",
		.gpio = GPIO_VBAT_LOAD,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "led1_rot",
		.gpio = GPIO_LED1_ROT,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = NULL, 
	},{
		.name = "led1_gruen",
		.gpio = GPIO_LED1_GRUEN,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "led2_rot",
		.gpio = GPIO_LED2_ROT,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = "timer",
	},{
		.name = "led2_gruen",
		.gpio = GPIO_LED2_GRUEN,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "led3_rot",
		.gpio = GPIO_LED3_ROT,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = "timer",
	},{
		.name = "led3_gruen",
		.gpio = GPIO_LED3_GRUEN,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "5mA_en_adin0",
		.gpio = GPIO_5MA_EN_ADIN0,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "5mA_en_adin1",
		.gpio = GPIO_5MA_EN_ADIN1,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "5mA_en_adin2",
		.gpio = GPIO_5MA_EN_ADIN2,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "5mA_en_adin3",
		.gpio = GPIO_5MA_EN_ADIN3,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "5mA_en_adin4",
		.gpio = GPIO_5MA_EN_ADIN4,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "5mA_en_adin5",
		.gpio = GPIO_5MA_EN_ADIN5,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "5mA_en_adin6",
		.gpio = GPIO_5MA_EN_ADIN6,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "5mA_en_adin7",
		.gpio = GPIO_5MA_EN_ADIN7,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "extio0",
		.gpio = GPIO_EXTIO0,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "extio1",
		.gpio = GPIO_EXTIO1,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "extio2",
		.gpio = GPIO_EXTIO2,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "extio3",
		.gpio = GPIO_EXTIO3,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "extio4",
		.gpio = GPIO_EXTIO4,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "extio5",
		.gpio = GPIO_EXTIO5,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "extio6",
		.gpio = GPIO_EXTIO6,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.default_trigger = NULL,
	},{
		.name = "rs485_shdn",
		.gpio = GPIO_RS485_SHDN,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.default_trigger = NULL,
	}
};

static struct gpio_led_platform_data ge_imx53_tera_com_led_data = {
	.num_leds	= ARRAY_SIZE(ge_imx53_tera_com_leds),
	.leds		= ge_imx53_tera_com_leds,
};

static struct platform_device ge_imx53_tera_com_led_device = {
	.name = "leds-gpio",
};

/*
 * DI / COUNTER
 */
static struct ge_counter ge_imx53_tera_com_counter_map[] = {
	{
                .name = "di0",
                .gpio = GPIO_DI0,
                .irq_type = IRQ_TYPE_EDGE_RISING,
                .interval = 0, // [ms]
		.reset_on_read = false
        },
	{
                .name = "di1",
                .gpio = GPIO_DI1,
                .irq_type = IRQ_TYPE_EDGE_RISING,
                .interval = 0, // [ms]
		.reset_on_read = false
        },
	{
                .name = "di2",
                .gpio = GPIO_DI2,
                .irq_type = IRQ_TYPE_EDGE_RISING,
                .interval = 0, // [ms]
		.reset_on_read = false
        },
	{
                .name = "di3",
                .gpio = GPIO_DI3,
                .irq_type = IRQ_TYPE_EDGE_RISING,
                .interval = 0, // [ms]
		.reset_on_read = false
        },
};
static struct ge_counter_platform_data ge_imx53_tera_com_counter_data = {
        .num_counter = ARRAY_SIZE(ge_imx53_tera_com_counter_map),
        .counter = ge_imx53_tera_com_counter_map,
};
static struct platform_device ge_imx53_tera_com_counter_device = {
        .name = "ge_counter",
};


/*
 * ECSPI1
 */

/* workaround for ecspi chipselect pin may not keep correct level when idle */
static void ge_imx53_tera_com_spi_chipselect_active(int cspi_mode, int status,
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

static void ge_imx53_tera_com_spi_chipselect_inactive(int cspi_mode, int status,
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

static struct mxc_spi_master ge_imx53_tera_com_mxcspi1_data = {
	.maxchipselect = 1,
	.spi_version = 23,
	.chipselect_active = ge_imx53_tera_com_spi_chipselect_active,
	.chipselect_inactive = ge_imx53_tera_com_spi_chipselect_inactive,
};

static struct spi_board_info ge_imx53_tera_com_mxcspi1_board_info[] = {
	{
	       .modalias = "mcp3208",
	       .platform_data = NULL,
	       .max_speed_hz = 1000000,
	       .bus_num = 1,
	       .chip_select = 0,
	}
};


/*
 * UART3 - RS485 direction
 */
static void ge_imx53_tera_com_uart3_sender_enable_func(bool ena)
{
	if(ena) {
		gpio_set_value(GPIO_RS485RW, 1);
		udelay(8);
	} else {
		gpio_set_value(GPIO_RS485RW, 0);
	}
}


/*
 * GPIO_WDT (external watchdog)
 */
static struct gpio_wdt_platform_data ge_imx53_tera_com_gpio_wdt_data = {
	.feed_gpio = GPIO_WDI,
};
static struct platform_device ge_imx53_tera_com_gpio_wdt_device = {
        .name = "gpio_wdt",
};


/*
 * peripheral init
 */
static void __init ge_imx53_tera_com_periph_init(void)
{
	/* 
	 * mux settings 
	 */
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_tera_com_pads,
				ARRAY_SIZE(ge_imx53_tera_com_pads));

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
	mx5_set_host1_vbus_func(ge_imx53_tera_com_usbh1_driver_vbus);
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
	mx5_set_otghost_vbus_func(ge_imx53_tera_com_usbotg_driver_vbus);
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
	mxc_register_device(&mxcsdhc2_device, &ge_imx53_tera_com_mmc2_data);

	/*
	 * I2C2
	 */
	mxc_register_device(&mxci2c_devices[1], &ge_imx53_tera_com_i2c2_data);
	i2c_register_board_info(1, ge_imx53_tera_com_i2c2_board_info,
			ARRAY_SIZE(ge_imx53_tera_com_i2c2_board_info));

	/*
	 * ECSPI1
	 */
	/* request and disassert cs0 */
	gpio_request(GPIO_CSPI1_CS0, "cspi1_cs0");
	gpio_direction_output(GPIO_CSPI1_CS0,1);

	/* register spi and devices */
	mxc_register_device(&mxcspi1_device, &ge_imx53_tera_com_mxcspi1_data);
	spi_register_board_info(ge_imx53_tera_com_mxcspi1_board_info,
			ARRAY_SIZE(ge_imx53_tera_com_mxcspi1_board_info));

	/*
	 * LEDS
	 */
	mxc_register_device(&ge_imx53_tera_com_led_device, 
		&ge_imx53_tera_com_led_data);
	
	/*
	 * COUNTER
	 */
	mxc_register_device(&ge_imx53_tera_com_counter_device, 
		&ge_imx53_tera_com_counter_data);

	/*
	 * UART3 - RS485 direction
	 * enable half-duplex-mode
	 * interrupt after first received char (32 chars buffer)
	 */
	gpio_request(GPIO_RS485RW, "rs485rw");
	gpio_direction_output(GPIO_RS485RW, 0);
	mx5_set_mxc_uart_sender_enable_func(
			2, 
			ge_imx53_tera_com_uart3_sender_enable_func,
			true);
	mx5_set_mxc_uart_receive_trigger_level(2, 1);

	/*
	 * GPIO_WDT (external watchdog)
	 */
	mxc_register_device(&ge_imx53_tera_com_gpio_wdt_device, 
		&ge_imx53_tera_com_gpio_wdt_data);
}


/*
 * board init
 */
void __init ge_imx53_tera_com_init(void)
{
	pr_info("Ginzinger imx53_tera_com (0x%X) Version 0x%X\n",GE_IMX_BOARD_TYPE(),GE_IMX_BOARD_VERSION());

	/* peripheral init */
	ge_imx53_tera_com_periph_init();
}

