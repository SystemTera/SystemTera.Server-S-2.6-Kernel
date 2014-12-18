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

#include <linux/slab.h>
#include <linux/platform_device.h>
#include <mach/common.h>
#include <mach/gpio.h>
#include "ge_imx53_gpio_keys.h"

/* alloc-size of key-table in key-data */
static int ge_imx53_gpio_keys_size=0;

/* key-data */
static struct gpio_keys_platform_data ge_imx53_gpio_keys_data = {
	.buttons=0,
	.nbuttons=0,
	.rep=0,
};

/*
 * register number of gpio-key buttons
 * parameters:
 *	keys .. list of gpio-keys to register
 *	nr .. number of elements in list
 * return: 0 .. ok, else -errno
 */
int __init ge_imx53_gpio_keys_add(struct gpio_keys_button *keys, int nr)
{
	int new_size;
	struct gpio_keys_button *new_list;

	/* calculate new array len */
	new_size=ge_imx53_gpio_keys_size;
	while(ge_imx53_gpio_keys_data.nbuttons+nr>new_size)
		new_size+=5;

	/* if not enough space -> realloc */
	if(new_size!=ge_imx53_gpio_keys_size) {
		new_list=kmalloc(new_size*sizeof(struct gpio_keys_button),GFP_KERNEL);
		if(!new_list)
			return -ENOMEM;
		memcpy(
			new_list,
			ge_imx53_gpio_keys_data.buttons,
			ge_imx53_gpio_keys_data.nbuttons*sizeof(struct gpio_keys_button)
		);
		if(ge_imx53_gpio_keys_data.buttons)
			kfree(ge_imx53_gpio_keys_data.buttons);
		ge_imx53_gpio_keys_data.buttons=new_list;
		ge_imx53_gpio_keys_size=new_size;
	}

	/* add new keys to list */
	memcpy(
		&ge_imx53_gpio_keys_data.buttons[ge_imx53_gpio_keys_data.nbuttons],
		keys,
		nr*sizeof(struct gpio_keys_button)
	);
	ge_imx53_gpio_keys_data.nbuttons+=nr;

	/* ok */
	return 0;
}


/* driver */
static struct platform_device ge_imx53_gpio_keys_device = {
	.name = "gpio-keys",
};

/* 
 * register all keys on system
 * DIRTY: do late to keep touchscreens on event0
 */
static int __init ge_imx53_gpio_keys_register(void)
{
	pr_info("%s\n",__FUNCTION__);

	mxc_register_device(&ge_imx53_gpio_keys_device, 
		&ge_imx53_gpio_keys_data);

	return 0;
}
late_initcall(ge_imx53_gpio_keys_register);

