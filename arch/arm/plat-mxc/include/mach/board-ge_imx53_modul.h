/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 * @Descr: common definions for ge_imx53_modul-based boards
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
 *  	2012KW34 - manfred.schlaegl
 *  		* added support for tera_com
 *	2012KW19 - manfred.schlaegl
 *		* added support for bt
 *	2012KW04 - manfred.schlaegl:
 *		* added support for bep2 and mmi boards
 *		* format for tab-size 8
 *	2011KW44 - manfred.schlaegl: 
 *		* rename modul and boards
 *	2011KW43 - manfred.schlaegl: 
 *		* begin implementation
 ***********************************************************************/

#ifndef __ASM_ARCH_MXC_BOARD_GE_IMX53_MODUL_H__
#define __ASM_ARCH_MXC_BOARD_GE_IMX53_MODUL_H__

/*
 * ginzinger board-types
 */
#define GE_IMX_BOARD_TYPE_UNKOWN			0
#define GE_IMX_BOARD_TYPE_GE_IMX53_TESTBOARD		1
#define GE_IMX_BOARD_TYPE_GE_IMX53_DEVBOARD		2
#define GE_IMX_BOARD_TYPE_GE_IMX53_BEP2			3
#define GE_IMX_BOARD_TYPE_GE_IMX53_MMI			4
#define GE_IMX_BOARD_TYPE_GE_IMX53_BT			5
#define GE_IMX_BOARD_TYPE_GE_IMX53_TERA_COM		6
#define GE_IMX_BOARD_TYPE_GE_IMX53_SIGMA2		7


#ifndef __ASSEMBLY__

/* macros, to get board-type and version */
extern unsigned int ge_imx53_board;
#define GE_IMX_BOARD_TYPE()		((ge_imx53_board>>16)&0xffff)
#define GE_IMX_BOARD_VERSION()		((ge_imx53_board>>0)&0xffff)

#endif /*  __ASSEMBLY__ */

#endif /* __ASM_ARCH_MXC_BOARD_GE_IMX53_MODUL_H__ */
