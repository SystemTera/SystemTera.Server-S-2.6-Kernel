/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 * @Descr: gpio-key handling for ge_imx53_modul-based boards
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
 *		* implemented and tested
 *  @TODO:
 ***********************************************************************/

#include <linux/gpio_keys.h>

/*
 * register number of gpio-key buttons
 * parameters:
 *	keys .. list of gpio-keys to register
 *	nr .. number of elements in list
 * return: 0 .. ok, else -errno
 */
int ge_imx53_gpio_keys_add(struct gpio_keys_button *keys, int nr);


