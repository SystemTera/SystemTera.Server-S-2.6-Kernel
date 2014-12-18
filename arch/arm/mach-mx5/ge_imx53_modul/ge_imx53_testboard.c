/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 * @Descr: ge_imx-board specific code for ge_imx53_testboard
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
 *	2012KW04 - manfred.schlaegl:
 *		* include ge_imx53 iomux-definitions
 *		* format for tab-size 8
 *	2011KW47 - manfred.schlaegl:
 *		* usb host/device ok
 *	2011KW44 - manfred.schlaegl: 
 *		* rename modul and boards
 *	2011KW43 - manfred.schlaegl: 
 *		* begin implementation
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
#include <linux/powerkey.h>
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

#include "../crm_regs.h"
#include "../devices.h"
#include "../usb.h"

#include"ge_imx53_iomux.h"
#include "ge_imx53_testboard.h"

/* usb host - usbh1 - oc/pwr */
#define GPIO_USBH1_OC		(2*32 + 30)	/* GPIO_3_30 - EIM_D30 (TODO: gpio or function) */
#define GPIO_USBH1_PWR		(2*32 + 31)	/* GPIO_3_31 - EIM_D31 (TODO: gpio or function) */

/* usb otg - otg - oc/pwr */
#define GPIO_USBOTG_OC		(2*32 + 21)	/* GPIO_3_21 - EIM_D21 (TODO: gpio or function) */
#define GPIO_USBOTG_PWR		(2*32 + 22)	/* GPIO_3_22 - EIM_D22 (TODO: gpio or function) */

/*
 * pad config
 */
static iomux_v3_cfg_t ge_imx53_testboard_pads[] = {

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
//	MX53_PAD_EIM_D22__USBOH3_USBOTG_PWR,	/* function */
	MX53_PAD_EIM_D22__GPIO3_22,			/* gpio: GPIO_USBOTG_PWR */
};


/*
 * usbh1 power
 */
static void ge_imx53_testboard_usbh1_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(GPIO_USBH1_PWR, 1);
	else
		gpio_set_value(GPIO_USBH1_PWR, 0);
}

/*
 * usbotg power
 */
static void ge_imx53_testboard_usbotg_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(GPIO_USBOTG_PWR, 1);
	else
		gpio_set_value(GPIO_USBOTG_PWR, 0);
}


/*
 * peripheral init
 */
static void __init ge_imx53_testboard_periph_init(void)
{
	/* 
	 * mux settings 
	 */
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_testboard_pads,
				ARRAY_SIZE(ge_imx53_testboard_pads));

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
	mx5_set_host1_vbus_func(ge_imx53_testboard_usbh1_driver_vbus);
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
	mx5_set_otghost_vbus_func(ge_imx53_testboard_usbotg_driver_vbus);
	mx5_usb_dr_init();
}


/*
 * board init
 */
void __init ge_imx53_testboard_init(void)
{
	pr_info("Ginzinger imx53_testboard (0x%X) Version %i\n",GE_IMX_BOARD_TYPE(),GE_IMX_BOARD_VERSION());

	/* peripheral init */
	ge_imx53_testboard_periph_init();
}

