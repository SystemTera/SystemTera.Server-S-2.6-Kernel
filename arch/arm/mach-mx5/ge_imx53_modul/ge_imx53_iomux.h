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
 *	2012KW10 - manfred.schlaegl:
 *		* defined display-line driver-strength and slew-rates 
 *	2012KW04 - manfred.schlaegl:
 *		* begin implementation
 ***********************************************************************/



#define MX53_100K_PULLDOWN_PAD_CTRL		(PAD_CTL_PUS_100K_DOWN|PAD_CTL_PUE|PAD_CTL_PKE)		/* PKE and PUE has to be set for pulldown */
#define MX53_100K_PULLUP_PAD_CTRL		(PAD_CTL_PUS_100K_UP|PAD_CTL_PUE|PAD_CTL_PKE)		/* PKE and PUE has to be set for pullup */
#define MX53_47K_PULLUP_PAD_CTRL		(PAD_CTL_PUS_47K_UP|PAD_CTL_PUE|PAD_CTL_PKE)		/* PKE and PUE has to be set for pullup */


/*
 * correct mux-settings for UART1 in DTE-Mode
 */
#define MX53_PAD_PATA_DIOW__UART1_DTE_RXD_MUX		IOMUX_PAD(0x5F0, 0x270, 3, 0x878, 2, MX53_UART_PAD_CTRL)	/* pad is named RXD, but in DTE-Mode function is TXD */
#define MX53_PAD_PATA_DMACK__UART1_DTE_TXD_MUX		IOMUX_PAD(0x5F4, 0x274, 3, 0x0, 0, MX53_UART_PAD_CTRL)		/* pad is named TXD, but in DTE-Mode function is RXD */
#define MX53_PAD_PATA_RESET_B__UART1_DTE_CTS		IOMUX_PAD(0x608, 0x288, 3, 0x874, 2, MX53_UART_PAD_CTRL)
#define MX53_PAD_PATA_IORDY__UART1_DTE_RTS		IOMUX_PAD(0x60C, 0x28C, 3, 0x0, 0, MX53_UART_PAD_CTRL)
#define MX53_PAD_EIM_D23__UART1_DTE_DCD			IOMUX_PAD(0x47C, 0x134, 3, 0x0, 0, MX53_UART_PAD_CTRL)
#define MX53_PAD_EIM_D25__UART1_DTE_DSR			IOMUX_PAD(0x488, 0x140, 7, 0x0, 0, MX53_UART_PAD_CTRL)
#define MX53_PAD_EIM_EB3__UART1_DTE_RI			IOMUX_PAD(0x480, 0x138, 3, 0x0, 0, MX53_UART_PAD_CTRL)
#define MX53_PAD_EIM_D24__UART1_DTE_DTR			IOMUX_PAD(0x484, 0x13C, 7, 0x0, 0, MX53_UART_PAD_CTRL)


/*
 * correct mux-settings for UART2 in DCE-Mode
 */
#define MX53_PAD_PATA_BUFFER_EN__UART2_DCE_RXD_MUX	IOMUX_PAD(0x5FC, 0x27C, 3, 0x880, 3, MX53_UART_PAD_CTRL)
#define MX53_PAD_PATA_DMARQ__UART2_DCE_TXD_MUX    	IOMUX_PAD(0x5F8, 0x278, 3, 0x0, 0, MX53_UART_PAD_CTRL)
#define MX53_PAD_PATA_INTRQ__UART2_DCE_RTS		IOMUX_PAD(0x600, 0x280, 3, 0x0, 0, MX53_UART_PAD_CTRL)		/* pad is named CTS, but in DCE-Mode function is RTS */
#define MX53_PAD_PATA_DIOR__UART2_DCE_CTS		IOMUX_PAD(0x604, 0x284, 3, 0x87C, 3, MX53_UART_PAD_CTRL)	/* pad is named RTS, but in DCE-Mode function is CTS */

/*
 * correct mux-settings for UART3 in DCE-Mode
 */
#define MX53_PAD_PATA_CS_1__UART3_DCE_RXD_MUX		IOMUX_PAD(0x620, 0x2A0, 4, 0x888, 3, MX53_UART_PAD_CTRL)
#define MX53_PAD_PATA_CS_0__UART3_DCE_TXD_MUX		IOMUX_PAD(0x61C, 0x29C, 4, 0x0, 0, MX53_UART_PAD_CTRL)
#define MX53_PAD_PATA_DA_1__UART3_DCE_RTS		IOMUX_PAD(0x614, 0x294, 4, 0x0, 0, MX53_UART_PAD_CTRL)		/* pad is named CTS, but in DCE-Mode function is RTS */
#define MX53_PAD_PATA_DA_2__UART3_DCE_CTS		IOMUX_PAD(0x618, 0x298, 4, 0x884, 5, MX53_UART_PAD_CTRL)	/* pad is named RTS, but in DCE-Mode function is CTS */

/*
 * correct mux-settings for UART3 in DCE-Mode
 */
#define MX53_PAD_CSI0_DAT13__UART4_DCE_RXD_MUX		IOMUX_PAD(0x420, 0xF4, 2, 0x890, 3, MX53_UART_PAD_CTRL)
#define MX53_PAD_CSI0_DAT12__UART4_DCE_TXD_MUX		IOMUX_PAD(0x41C, 0xF0, 2, 0x0, 0, MX53_UART_PAD_CTRL)

/*
 * correct mux-settings for UART4 in DCE-Mode
 */
#define MX53_PAD_CSI0_DAT15__UART5_DCE_RXD_MUX		IOMUX_PAD(0x428, 0xFC, 2, 0x898, 3, MX53_UART_PAD_CTRL)
#define MX53_PAD_CSI0_DAT14__UART5_DCE_TXD_MUX		IOMUX_PAD(0x424, 0xF8, 2, 0x0, 0, MX53_UART_PAD_CTRL)

/*
 * driver-settings for parallel display
 */
//#define MX53_DISPLAY_CTRL_PAD_CTRL 		(PAD_CTL_DSE_HIGH|PAD_CTL_SRE_FAST) // default freescale
//#define MX53_DISPLAY_DATA_PAD_CTRL		(PAD_CTL_DSE_HIGH|PAD_CTL_SRE_SLOW) // default freescale
#define MX53_DISPLAY_CTRL_PAD_CTRL		(PAD_CTL_DSE_LOW|PAD_CTL_SRE_SLOW) // ginzinger
#define MX53_DISPLAY_DATA_PAD_CTRL		(PAD_CTL_DSE_LOW|PAD_CTL_SRE_SLOW) // ginzinger

