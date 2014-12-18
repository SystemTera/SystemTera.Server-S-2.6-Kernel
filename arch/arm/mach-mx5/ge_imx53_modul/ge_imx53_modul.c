/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 * @Descr: ge_imx53_modul specific code
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
 *  	2013KW64 - manfred.schlaegl
 *  		* external pullups on LTC_IRQ and LTC_PBSTAT are missing -> 
 *  			added internal 100k pullups
 *  	2012KW34 - manfred.schlaegl
 *  		* added support for tera_com
 *	2012KW19 - manfred.schlaegl
 *		* internal temp-sensor (imx-ahci-hwmon) tested but failed
 *		* added support for bt
 *	2012KW11 - manfred.schlaegl:
 *		* support for GPIO6_GPIO16_PWREXT_EN
 *			see: https://development.ginzinger.com/armlinux/wiki/ManfredsLogs12KW11-2
 *	2012KW07 - manfred.schlaegl: 
 *		* ge_imx53_module and ge_imx53_board over ge_variables 
 *			implemented and tested
 *	2012KW04 - manfred.schlaegl:
 *		* added support for bep2 and mmi boards
 *		* include ge_imx53 iomux-definitions
 *		* format for tab-size 8
 *	2012KW03 - manfred.schlaegl:
 *		* fill cpuinfo-data (/proc/cpuinfo)
 *			* set module revision as revision
 *			* misuse serial_high = module-version/revision
 *			* misuse serial_low = board-version/revision
 *	2011KW50 - manfred.schlaegl:
 *		* platform-data may be kept valid while running system (mmc-issues)
 *			removed structure attribute __initdata
 *	2011KW48 - manfred.schlaegl:
 *		* structure attributes (__initdata)
 *		* cleanup
 *		* general support for graphics (ipu)
 *	2011KW46 - manfred.schlaegl:
 *		* moved ltc-gpios to ge_imx53_pmic_ltc3589.c
 *		* cleanup
 *		* determine nand-bus-width from boot-cfg-pin BT_CFG2_5
 *		* wakeup from standby/suspend on pbstat (push-button)
 *		* corrected gpio-comments
 *	2011KW45 - manfred.schlaegl:
 *		* removed predefined nand-partition-table
 *		* disabled mux and other settings, that are already done in u-boot
 *		* correct clock-settings
 *		* nfc init and mux ok
 *	2011KW44 - manfred.schlaegl: 
 *		* rename modul and boards
 *	2011KW43 - manfred.schlaegl: 
 *		* begin implementation
 *		* copy from mx53_evk.c
 *			* Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * TODO:
 *	* internal temp-sensor (imx-ahci-hwmon)
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
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/ge_variables.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/board-ge_imx53_modul.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/system.h>
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

#include "ge_imx53_iomux.h"
#include "ge_imx53_modul_pmic_ltc3589.h"
#include "ge_imx53_testboard.h"
#include "ge_imx53_devboard.h"
#include "ge_imx53_bep2.h"
#include "ge_imx53_mmi.h"
#include "ge_imx53_bt.h"
#include "ge_imx53_tera_com.h"
#include "ge_imx53_sigma2.h"

#define MACHINE_NAME	"Ginzinger imx53_modul"

/*
 * get module version/revision
 * not used yet
 */
unsigned int ge_imx53_module=0;
static int __init ge_imx53_module_setup(char *p)
{
	ge_imx53_module = simple_strtoul(p, NULL, 16);
	/* cpuinfo: set module revision as revision */
	system_rev=ge_imx53_module&0xffff;
	/* cpuinfo: misuse serial: serial_high = module-version/revision */
	system_serial_high=ge_imx53_module;
	return 0;
}
early_param("ge_imx53_module", ge_imx53_module_setup);
/* get function for ge_variable */
static unsigned int ge_imx53_module_get(void)
{
	return ge_imx53_module;
}

/*
 * get board version/revistion
 */
unsigned int ge_imx53_board=0;
static int __init ge_imx53_board_setup(char *p)
{
	ge_imx53_board = simple_strtoul(p, NULL, 16);
	/* cpuinfo: misuse serial: serial_low = board-version/revision */
	system_serial_low=ge_imx53_board;
	return 0;
}
early_param("ge_imx53_board", ge_imx53_board_setup);
/* get function for ge_variable */
static unsigned int ge_imx53_board_get(void)
{
	return ge_imx53_board;
}

/* 
 * Console 
 */
static iomux_v3_cfg_t ge_imx53_modul_console_pads[] = {
/*
 * already initialized with correct settings in u-boot-loader
 * if settings should be changed, this must be done in u-boot-code
 * do not enable following code
 */
#if 0

	/* UART2 TXD 
     * IOMUX: <AltMode Name="ALT3" BallNumber="J1" BallName="PATA_DMARQ" PowerGroup="PATA__0" Signal="UART2_TXD_MUX" IsExcluded="false" />
     */
	MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,

	/* UART2 RXD
     * IOMUX: <AltMode Name="ALT3" BallNumber="K4" BallName="PATA_BUFFER_EN" PowerGroup="PATA__0" Signal="UART2_RXD_MUX" Comment="bootfähig (lt. Hrn Matt)" IsExcluded="false" />
     */
	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
#endif
};

/* 
 * I2C1 - PMIC 
 */
static iomux_v3_cfg_t ge_imx53_modul_i2c1_pads[] = {
/*
 * already initialized with correct settings in u-boot-loader
 * if settings should be changed, this must be done in u-boot-code
 * do not enable following code
 */
#if 0

	/* 
	 * I2C1_SDA
	 * IOMUX: <AltMode Name="ALT5" BallNumber="AA1" BallName="EIM_D28" PowerGroup="WEIM_SEC" Signal="I2C1_SDA" IsExcluded="false" />
	 */
	MX53_PAD_EIM_D28__I2C1_SDA,

	/*
	 * I2C1_SCL
	 * IOMUX: <AltMode Name="ALT5" BallNumber="R4" BallName="CSI0_DAT9" PowerGroup="IPU_CSI" Signal="I2C1_SCL" IsExcluded="false" />
	 */
	MX53_PAD_CSI0_DAT9__I2C1_SCL,
#endif
};

/* 
 * PMIC - LTC3589
 */
static iomux_v3_cfg_t ge_imx53_modul_pmic_pads[] = {
	/*
	 * PMIC_INT - LTC_IRQ
	 * <AltMode Name="ALT1" BallNumber="C6" BallName="GPIO_16" PowerGroup="GPIO" Signal="GPIO7_GPIO[11]" Comment="/LTC_IRQ" IsExcluded="false" />
	 */
	(_MX53_PAD_GPIO_16__GPIO7_11|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL)),
    
	/*
	 * PMIC_RDY - LTC_PGOOD
	 * IOMUX: <AltMode Name="ALT3" BallNumber="A3" BallName="GPIO_17" PowerGroup="GPIO" Signal="GPC_PMIC_RDY" IsExcluded="false" />
	 */
	//MX53_PAD_GPIO_17__GPC_PMIC_RDY		/* function (TODO: check) */
	MX53_PAD_GPIO_17__GPIO7_12,			/* gpio (like ard) */

	/*
	 * PMIC_PBSTAT - LTC_PBSTAT
	 * <AltMode Name="ALT1" BallNumber="D7" BallName="GPIO_18" PowerGroup="GPIO" Signal="GPIO7_GPIO[13]" Comment="/LTC_PBSTAT" IsExcluded="false" />
	 */
	(_MX53_PAD_GPIO_18__GPIO7_13|MUX_PAD_CTRL(MX53_100K_PULLUP_PAD_CTRL))
};

/*
 * FEC Ethernet
 */
#define GE_IMX53_MODUL_FEC_PHY_RESET		(6*32 + 6)	/* GPIO_7_6 */
static iomux_v3_cfg_t ge_imx53_modul_eth_pads[] = {
/*
 * already initialized with correct settings in u-boot-loader
 * if settings should be changed, this must be done in u-boot-code
 * do not enable following code
 */
#if 0

	/*
	 * FEC_MDIO
	 * IOMUX: <AltMode Name="ALT0" BallNumber="D12" BallName="FEC_MDIO" PowerGroup="FEC" Signal="FEC_MDIO" IsExcluded="false" />
	 * like emtrion-module
	 */
	MX53_PAD_FEC_MDIO__FEC_MDIO,

	/*
	 * FEC_MDC
	 * IOMUX: <AltMode Name="ALT0" BallNumber="E10" BallName="FEC_MDC" PowerGroup="FEC" Signal="FEC_MDC" IsExcluded="false" />
	 * like emtrion-module
	 */
	MX53_PAD_FEC_MDC__FEC_MDC,


	/* 
	 * FEC RXD1
	 * IOMUX: <AltMode Name="ALT0" BallNumber="E11" BallName="FEC_RXD1" PowerGroup="FEC" Signal="FEC_RDATA[1]" IsExcluded="false" />
	 * like emtrion-module
	 */
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,

	/* 
	 * FEC RXD0
	 * IOMUX: <AltMode Name="ALT0" BallNumber="C11" BallName="FEC_RXD0" PowerGroup="FEC" Signal="FEC_RDATA[0]" IsExcluded="false" />
	 * like emtrion-module
	 */
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,


	/* 
	 * FEC TXD1
	 * IOMUX: <AltMode Name="ALT0" BallNumber="D10" BallName="FEC_TXD1" PowerGroup="FEC" Signal="FEC_TDATA[1]" IsExcluded="false" />
	 * like emtrion-module
	 */
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,


	/*
	 * FEC TXD0
	 * IOMUX: <AltMode Name="ALT0" BallNumber="F10" BallName="FEC_TXD0" PowerGroup="FEC" Signal="FEC_TDATA[0]" IsExcluded="false" />
	 * like emtrion-module
	 */
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,

	/*
	 * FEC TX_EN 
	 * IOMUX: <AltMode Name="ALT0" BallNumber="C10" BallName="FEC_TX_EN" PowerGroup="FEC" Signal="FEC_TX_EN" IsExcluded="false" />
	 * like emtrion-module
	 */
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,

	/*
	 * FEC TX_CLK
	 * IOMUX: <AltMode Name="ALT0" BallNumber="E12" BallName="FEC_REF_CLK" PowerGroup="FEC" Signal="FEC_TX_CLK" Comment="FEC_REF_CLK bei Quickstartschaltplan" IsExcluded="false" />
	 * like emtrion-module
	 */
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,

	/*
	 * FEC RX_ER
	 * IOMUX: <AltMode Name="ALT0" BallNumber="F12" BallName="FEC_RX_ER" PowerGroup="FEC" Signal="FEC_RX_ER" IsExcluded="false" />
	 * like emtrion-module
	 */
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,

	/*
	 * FEC CRS
	 * IOMUX: <AltMode Name="ALT0" BallNumber="D11" BallName="FEC_CRS_DV" PowerGroup="FEC" Signal="FEC_RX_DV" Comment="FEC_CRS_DV bei Quickstartschaltplan" IsExcluded="false" />
	 * like emtrion-module
	 */
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,

	/*
	 * FEC_nRST
	 * PYH Reset GPIO (gpio7-6)    
     * IOMUX: <AltMode Name="ALT1" BallNumber="K6" BallName="PATA_DA_0" PowerGroup="PATA__0" Signal="GPIO7_GPIO[6]" Comment="Ethernet FEC_nRST (nur GPIO), gleich zu Quickstartboard" IsExcluded="false" />
	 */
	MX53_PAD_PATA_DA_0__GPIO7_6,
#endif
};



/*
 * NFC NAND-Flash
 */
static iomux_v3_cfg_t ge_imx53_modul_nand_pads[] = {
/*
 * already initialized with correct settings in u-boot-loader
 * if settings should be changed, this must be done in u-boot-code
 * do not enable following code
 */
#if 0

	/*
	 * NANDF_WP_B
	 * IOMUX: <AltMode Name="ALT0" BallNumber="AC9" BallName="NANDF_WP_B" PowerGroup="NANDF" Signal="EMI_NANDF_WP_B" IsExcluded="false" />
	 */
	MX53_PAD_NANDF_WP_B__EMI_NANDF_WP_B,

	/*
	 * NANDF_CS0
	 * IOMUX: <AltMode Name="ALT0" BallNumber="W12" BallName="NANDF_CS0" PowerGroup="NANDF" Signal="EMI_NANDF_CS[0]" IsExcluded="false" />
	 */
	MX53_PAD_NANDF_CS0__EMI_NANDF_CS_0,

	/*
	 * NANDF_RB0
	 * IOMUX: <AltMode Name="ALT0" BallNumber="U11" BallName="NANDF_RB0" PowerGroup="NANDF" Signal="EMI_NANDF_RB[0]" IsExcluded="false" />
	 */
	MX53_PAD_NANDF_RB0__EMI_NANDF_RB_0,

	/*
	 * NANDF_CLE
	 * IOMUX: <AltMode Name="ALT0" BallNumber="AA10" BallName="NANDF_CLE" PowerGroup="NANDF" Signal="EMI_NANDF_CLE" IsExcluded="false" />
	 */
	MX53_PAD_NANDF_CLE__EMI_NANDF_CLE,

	/* 
	 * NANDF_ALE
	 * IOMUX: <AltMode Name="ALT0" BallNumber="Y11" BallName="NANDF_ALE" PowerGroup="NANDF" Signal="EMI_NANDF_ALE" IsExcluded="false" />
	 */
	MX53_PAD_NANDF_ALE__EMI_NANDF_ALE,

	/* 
	 * NANDF_RE_B
	 * IOMUX: <AltMode Name="ALT0" BallNumber="AC8" BallName="NANDF_RE_B" PowerGroup="WEIM_MAIN__1" Signal="EMI_NANDF_RE_B" IsExcluded="false" />
	 */
	MX53_PAD_NANDF_RE_B__EMI_NANDF_RE_B,

	/*
	 * NANDF_WE_B
	 * IOMUX: <AltMode Name="ALT0" BallNumber="AB8" BallName="NANDF_WE_B" PowerGroup="WEIM_MAIN__1" Signal="EMI_NANDF_WE_B" IsExcluded="false" />
	 */
	MX53_PAD_NANDF_WE_B__EMI_NANDF_WE_B,

	/*
	 * NANDF_D0 - NANDFD15
	 * IOMUX: 
	 *	* <AltMode Name="ALT3" BallNumber="L1" BallName="PATA_DATA0" PowerGroup="PATA__1" Signal="EMI_NANDF_D[0]" IsExcluded="false" />
	 *  * <AltMode Name="ALT3" BallNumber="M1" BallName="PATA_DATA1" PowerGroup="PATA__1" Signal="EMI_NANDF_D[1]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="L6" BallName="PATA_DATA2" PowerGroup="PATA__1" Signal="EMI_NANDF_D[2]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="M2" BallName="PATA_DATA3" PowerGroup="PATA__1" Signal="EMI_NANDF_D[3]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="M3" BallName="PATA_DATA4" PowerGroup="PATA__1" Signal="EMI_NANDF_D[4]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="M4" BallName="PATA_DATA5" PowerGroup="PATA__1" Signal="EMI_NANDF_D[5]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="N1" BallName="PATA_DATA6" PowerGroup="PATA__1" Signal="EMI_NANDF_D[6]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="M5" BallName="PATA_DATA7" PowerGroup="PATA__1" Signal="EMI_NANDF_D[7]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="N2" BallName="PATA_DATA8" PowerGroup="PATA__1" Signal="EMI_NANDF_D[8]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="N3" BallName="PATA_DATA9" PowerGroup="PATA__1" Signal="EMI_NANDF_D[9]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="M6" BallName="PATA_DATA11" PowerGroup="PATA__1" Signal="EMI_NANDF_D[11]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="N5" BallName="PATA_DATA12" PowerGroup="PATA__1" Signal="EMI_NANDF_D[12]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="N6" BallName="PATA_DATA13" PowerGroup="PATA__1" Signal="EMI_NANDF_D[13]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="P6" BallName="PATA_DATA14" PowerGroup="PATA__1" Signal="EMI_NANDF_D[14]" IsExcluded="false" />
	 *	* <AltMode Name="ALT3" BallNumber="P5" BallName="PATA_DATA15" PowerGroup="PATA__1" Signal="EMI_NANDF_D[15]" IsExcluded="false" />
	 */
	MX53_PAD_PATA_DATA0__EMI_NANDF_D_0,
	MX53_PAD_PATA_DATA1__EMI_NANDF_D_1,
	MX53_PAD_PATA_DATA2__EMI_NANDF_D_2,
	MX53_PAD_PATA_DATA3__EMI_NANDF_D_3,
	MX53_PAD_PATA_DATA4__EMI_NANDF_D_4,
	MX53_PAD_PATA_DATA5__EMI_NANDF_D_5,
	MX53_PAD_PATA_DATA6__EMI_NANDF_D_6,
	MX53_PAD_PATA_DATA7__EMI_NANDF_D_7,
	MX53_PAD_PATA_DATA8__EMI_NANDF_D_8,
	MX53_PAD_PATA_DATA9__EMI_NANDF_D_9,
	MX53_PAD_PATA_DATA10__EMI_NANDF_D_10,
	MX53_PAD_PATA_DATA11__EMI_NANDF_D_11,
	MX53_PAD_PATA_DATA12__EMI_NANDF_D_12,
	MX53_PAD_PATA_DATA13__EMI_NANDF_D_13,
	MX53_PAD_PATA_DATA14__EMI_NANDF_D_14,
	MX53_PAD_PATA_DATA15__EMI_NANDF_D_15,
#endif
};

/* 
 * PWREXT_EN - GPIO6_GPIO16_PWREXT_EN 
 */
#define GE_IMX53_MODUL_PWREXT_EN		(5*32 + 16)	/* GPIO_6_16 */
static iomux_v3_cfg_t ge_imx53_modul_pwrext_en[] = {
/*
 * already initialized with correct settings in u-boot-loader
 * if settings should be changed, this must be done in u-boot-code
 * do not enable following code
 */
#if 0
	/* https://development.ginzinger.com/armlinux/wiki/ManfredsLogs12KW11-2 - imx53_modul A02 - Vorabschaltplan (Software RST-OUT) */
	MX53_PAD_NANDF_CS3__GPIO6_16,
#endif
};


static struct fec_platform_data ge_imx53_modul_fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};


static struct imxi2c_platform_data ge_imx53_modul_mxci2c1_data = {
	.bitrate = 400000,
};

static struct mxc_dvfs_platform_data ge_imx53_modul_dvfs_core_data = {
	.reg_id = "SW1",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
};

static struct mxc_bus_freq_platform_data ge_imx53_modul_bus_freq_data = {
	.gp_reg_id = "SW1",
	.lp_reg_id = "SW2",
};

static void ge_imx53_modul_iim_enable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;

	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg |= 0x10;
	writel(reg, ccm_base + 0x64);
}

static void ge_imx53_modul_iim_disable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;

	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg &= ~0x10;
	writel(reg, ccm_base + 0x64);
}

static struct mxc_iim_data ge_imx53_modul_iim_data = {
	.bank_start = MXC_IIM_MX53_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX53_BANK_END_ADDR,
	.enable_fuse = ge_imx53_modul_iim_enable_fuse,
	.disable_fuse = ge_imx53_modul_iim_disable_fuse,
};

static int ge_imx53_modul_nand_init(void)
{
/*
 * are already initialized with correct settings in u-boot-loader
 * if settings should be changed, this must be done in u-boot-code
 * do not enable following code
 */
#if 0
	u32 i, reg;
	void __iomem *base;

	/*
	 * set M4IF_GENP_WEIM_MM bit
	 * (already set by boot-config-pins)
	 *
	 * 49.2.3 General Purpose Register (M4IF_GPR)
	 * MM=1 means that NFC_D is muxed with EIM DATA on NANDF_D (on PATA_DATA[15:0] pins). If MM=0,
	 * NFC_D is muxed with EIM A/D on EIM_DA[15:1]. In this case the corresponding CSxGCR2[12] must also
	 * be set to notify EIM that its address is muxed with NF data. This MM bit is also selected by boot according
	 * to BOOT_CFG1[6] fuse.
	 */
	#define M4IF_GENP_WEIM_MM_MASK			(1<<0)
	base = ioremap(MX53_BASE_ADDR(M4IF_BASE_ADDR), SZ_4K);
	reg = __raw_readl(M4IF_BASE_ADDR + 0xc);
	reg |= M4IF_GENP_WEIM_MM_MASK;
	__raw_writel(reg, M4IF_BASE_ADDR + 0xc);
	iounmap(base);

	/*
	 * reset WEIM_GCR2_MUX16_BYP_GRANT bits, if NFC PATA_DATA is used
	 *
	 * 25.4.2 Chip Select n General Configuration Register 2 (EIM_CSnGCR2)
	 * Addresses: 
	 * 	* EIM_CS0GCR2 is 63FD_A000h base + 2004h offset = 63FD_C004h
	 *	* EIM_CS1GCR2 is 63FD_A000h base + 201Ch offset = 63FD_C01Ch
	 *	* EIM_CS2GCR2 is 63FD_A000h base + 2034h offset = 63FD_C034h
	 *	* EIM_CS3GCR2 is 63FD_A000h base + 204Ch offset = 63FD_C04Ch
	 *	* EIM_CS4GCR2 is 63FD_A000h base + 2064h offset = 63FD_C064h
	 *	* EIM_CS5GCR2 is 63FD_A000h base + 207Ch offset = 63FD_C07Ch
	 * 
	 * 12 .. MUX16_BYP_GRANT
	 * Muxed 16 bypass grant. This bit when asserted causes EIM to bypass the grant/ack. arbitration with NFC
	 * (only for 16 bit muxed mode accesses).
	 * NOTE: The reset value for EIM_CS0GCR2[MUX16_BYP_GRANT] = EIM_BOOT[12]. For
	 * EIM_CS1GCR2 - EIM_CS5GCR2, MUX16_BYP_GRANT reset value is 1.
	 *
	 * 0   EIM waits for grant before driving a 16 bit muxed mode access to the memory.
	 * 1   EIM ignores the grant signal and immediately drives a 16 bit muxed mode access to the memory.
	 *
	 */
	#define WEIM_GCR2_MUX16_BYP_GRANT_MASK	(1<<12)
	base = ioremap(MX53_BASE_ADDR(WEIM_BASE_ADDR), SZ_4K);
	for (i = 0x4; i < 0x94; i += 0x18) {
		reg = __raw_readl(WEIM_BASE_ADDR + i);
		reg &= ~WEIM_GCR2_MUX16_BYP_GRANT_MASK;
		__raw_writel(reg, WEIM_BASE_ADDR + i);
	}
	iounmap(base);
#endif
	return 0;
}

static struct flash_platform_data ge_imx53_modul_nand_data = {
#ifdef CONFIG_MTD_PARTITIONS
	.name = "physmap-flash.0",
	.parts = 0,
	.nr_parts = 0,
#endif
	/* nand-bus-width is determined in ge_imx53_modul_nand_register below */
	.init = ge_imx53_modul_nand_init,
};

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config ge_imx53_modul_ipu_data = {
	.rev = 3,
	.reset = mx5_ipu_reset,
};

struct resource ge_imx53_modul_mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init ge_imx53_modul_fixup(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_1G;
	int left_mem = 0;
	int gpu_mem = SZ_128M;
	int fb_mem = SZ_32M;
	char *str;

	mxc_set_cpu_type(MXC_CPU_MX53);

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			break;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				left_mem = memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_nommu");
			if (str != NULL)
				gpu_data.enable_mmu = 0;

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}

			break;
		}
	}

	if (gpu_data.enable_mmu)
		gpu_mem = 0;

	if (left_mem == 0 || left_mem > total_mem)
		left_mem = total_mem - gpu_mem - fb_mem;

	if (mem_tag) {
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		if (!gpu_data.enable_mmu) {
			gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
			gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
		}

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
			ge_imx53_modul_mxcfb_resources[0].start =
				gpu_data.enable_mmu ?
				mem_tag->u.mem.start + left_mem :
				gpu_device.resource[5].end + 1;
			ge_imx53_modul_mxcfb_resources[0].end =
				ge_imx53_modul_mxcfb_resources[0].start + fb_mem - 1;
		} else {
			ge_imx53_modul_mxcfb_resources[0].start = 0;
			ge_imx53_modul_mxcfb_resources[0].end = 0;
		}
#endif
	}
}

static void __init ge_imx53_modul_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_modul_console_pads,
				ARRAY_SIZE(ge_imx53_modul_console_pads));
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_modul_i2c1_pads,
				ARRAY_SIZE(ge_imx53_modul_i2c1_pads));
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_modul_pmic_pads,
				ARRAY_SIZE(ge_imx53_modul_pmic_pads));
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_modul_eth_pads,
				ARRAY_SIZE(ge_imx53_modul_eth_pads));
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_modul_nand_pads,
				ARRAY_SIZE(ge_imx53_modul_nand_pads));
	mxc_iomux_v3_setup_multiple_pads(ge_imx53_modul_pwrext_en,
				ARRAY_SIZE(ge_imx53_modul_pwrext_en));

	/* reset FEC PHY */
	gpio_request(GE_IMX53_MODUL_FEC_PHY_RESET, "fec-phy-reset");
	gpio_direction_output(GE_IMX53_MODUL_FEC_PHY_RESET, 0);
	msleep(1);
	gpio_set_value(GE_IMX53_MODUL_FEC_PHY_RESET, 1);

	/* 
	 * PWREXT_EN - GPIO6_GPIO16_PWREXT_EN 
	 */
	gpio_request(GE_IMX53_MODUL_PWREXT_EN, "pwrext_en");
	gpio_direction_output(GE_IMX53_MODUL_PWREXT_EN, 1);
	gpio_free(GE_IMX53_MODUL_PWREXT_EN);
}

static void __init ge_imx53_modul_nand_register(void)
{
	void __iomem *src_base;
	unsigned int sbmr;

	/* automatic determination of nand-bus-width from boot-config-pin BT_CFG2_5 */

	/* read SBMR-Register (Boot-Config) */
	src_base = ioremap(MX53_BASE_ADDR(SRC_BASE_ADDR), PAGE_SIZE);
	sbmr = __raw_readl(src_base + 0x4);
	iounmap(src_base);

	/* set nand-bus-width from BT_CFG2_5 (NAND Interface Bus Width: 0 .. 8bit, 1 .. 16bit) */
	ge_imx53_modul_nand_data.width=(((sbmr>>8)&(0xFF))&(1<<5)) ? 2 : 1;

	/* register */
	mxc_register_device(&mxc_nandv2_mtd_device, &ge_imx53_modul_nand_data);
}



/*
 * Variable Export (ge_variable)
 */
static struct ge_variable ge_imx53_modul_variable_map[] = {
	{
		.name = "ge_imx53_module",
		.get_value = ge_imx53_module_get,
	},
	{
		.name = "ge_imx53_board",
		.get_value = ge_imx53_board_get,
	},
};

static struct ge_variables_platform_data ge_imx53_modul_variables_data = {
	.num_variables = ARRAY_SIZE(ge_imx53_modul_variable_map),
	.variables = ge_imx53_modul_variable_map,
};

static struct platform_device ge_imx53_modul_ge_variables_device = {
	.name = "ge_variables",
};


/*
 * Board specific initialization.
 */
static void __init ge_imx53_modul_board_init(void)
{
	mxc_cpu_common_init();

	pr_info("%s Rev%i\n",MACHINE_NAME,ge_imx53_module);

	ge_imx53_modul_io_init();

	ge_imx53_modul_pmic_ltc3589_pmic_init();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxci2c_devices[0], &ge_imx53_modul_mxci2c1_data);
	mxc_register_device(&mxc_rtc_device, NULL);
	mxc_register_device(&mxcscc_device, NULL);

	/* internal temp-sensor (imx-ahci-hwmon) tested but failed */
#if 0
	mxc_register_device(&ahci_fsl_device, &sata_data);
	mxc_register_device(&imx_ahci_device_hwmon, NULL);
#endif

	/*
	mxc_register_device(&mx53_lpmode_device, NULL);
	mxc_register_device(&sdram_autogating_device, NULL);
	*/
	mxc_register_device(&mxc_dvfs_core_device, &ge_imx53_modul_dvfs_core_data);
	mxc_register_device(&busfreq_device, &ge_imx53_modul_bus_freq_data);
	/*
	mxc_register_device(&mxc_dvfs_per_device, &dvfs_per_data);
	*/
	mxc_register_device(&mxc_iim_device, &ge_imx53_modul_iim_data);
	mxc_register_device(&mxc_fec_device, &ge_imx53_modul_fec_data);

	/*
	 * IPU has to be initialized generally (kernel-fault happens if not)
	 */
	ge_imx53_modul_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	ge_imx53_modul_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	ge_imx53_modul_ipu_data.csi_clk[0] = clk_get(NULL, "ssi_ext1_clk");
	mxc_register_device(&mxc_ipu_device, &ge_imx53_modul_ipu_data);

	ge_imx53_modul_nand_register();

	/*
	 * Variables Export (ge_variable)
	 */
	mxc_register_device(&ge_imx53_modul_ge_variables_device, 
		&ge_imx53_modul_variables_data);

	/*
	 * start board initialization
	 */
	switch(GE_IMX_BOARD_TYPE()) {
#ifdef CONFIG_MACH_GE_IMX53_TESTBOARD
		case GE_IMX_BOARD_TYPE_GE_IMX53_TESTBOARD:
			ge_imx53_testboard_init();
		break;
#endif
#ifdef CONFIG_MACH_GE_IMX53_DEVBOARD
		case GE_IMX_BOARD_TYPE_GE_IMX53_DEVBOARD:
			ge_imx53_devboard_init();
		break;
#endif
#ifdef CONFIG_MACH_GE_IMX53_BEP2
		case GE_IMX_BOARD_TYPE_GE_IMX53_BEP2:
			ge_imx53_bep2_init();
		break;
#endif
#ifdef CONFIG_MACH_GE_IMX53_MMI
		case GE_IMX_BOARD_TYPE_GE_IMX53_MMI:
			ge_imx53_mmi_init();
		break;
#endif
#ifdef CONFIG_MACH_GE_IMX53_BT
		case GE_IMX_BOARD_TYPE_GE_IMX53_BT:
			ge_imx53_bt_init();
		break;
#endif
#ifdef CONFIG_MACH_GE_IMX53_TERA_COM
		case GE_IMX_BOARD_TYPE_GE_IMX53_TERA_COM:
			ge_imx53_tera_com_init();
		break;
#endif
#ifdef CONFIG_MACH_GE_IMX53_SIGMA2
		case GE_IMX_BOARD_TYPE_GE_IMX53_SIGMA2:
			ge_imx53_sigma2_init();
		break;
#endif
		case GE_IMX_BOARD_TYPE_UNKOWN:
		default:
			pr_info("Unknown Board-Type (0x%X)-> Starting in module-only mode\n", GE_IMX_BOARD_TYPE());
	}
}

static void __init ge_imx53_modul_timer_init(void)
{
	struct clk *uart_clk;

	/* 
	 * Low-Clock: 32KHz
	 * High-Clock: 24MHz
	 * CKIH1: 0Hz
	 * CKIH2: 0Hz
	 */
	mx53_clocks_init(32768, 24000000, 0, 0);

	/* init early console */
	uart_clk = clk_get_sys("mxcintuart.1", NULL);
	early_console_setup(MX53_BASE_ADDR(UART2_BASE_ADDR), uart_clk);
}

static struct sys_timer ge_imx53_modul_timer = {
	.init	= ge_imx53_modul_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_EVK data structure.
 */
MACHINE_START(GE_IMX53_MODUL, MACHINE_NAME)
	/* Maintainer: Ginzinger electronic systems */
	.fixup = ge_imx53_modul_fixup,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = ge_imx53_modul_board_init,
	.timer = &ge_imx53_modul_timer,
MACHINE_END

