/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 * @Descr: ge_imx53_modul pmic(ltc3589) specific code
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
 *		* split initialization in init(early) and start(late)
 *		* use ge_imx53_gpio_keys for gpio-keys
 *	2011KW50 - manfred.schlaegl:
 *		* platform-data may be kept valid while running system (mmc-issues)
 *			removed structure attribute __initdata
 *	2011KW48 - manfred.schlaegl:
 *		* structure attributes (__initdata)
 *		* cleanup naming of functions and structures
 *		* pbstat is a key
 *	2011KW46 - manfred.schlaegl: 
 *		* begin: powerkey
 *		* moved ltc-gpios from ge_imx53_module.c
 *		* corrected min/max voltages
 *	2011KW45 - manfred.schlaegl: 
 *		* begin implementation
 *		* mx53_ard_pmic_ltc3589.c  --  i.MX53 ARD Driver for Linear LTC3589
 *			* Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 ***********************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/regulator/ltc3589.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/ltc3589/core.h>
#include <linux/powerkey.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <mach/iomux-mx53.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/common.h>

#include "ge_imx53_modul_pmic_ltc3589.h"
#include "ge_imx53_gpio_keys.h"

/*
 * pmic gpios
 */
#define GPIO_PMIC_INT			(6*32 + 11)	/* GPIO_7_11 - GPIO16 */
#define GPIO_PMIC_RDY			(6*32 + 12)	/* GPIO_7_12 - GPIO17 (TODO: gpio or function) */
#define GPIO_PMIC_PBSTAT		(6*32 + 13)	/* GPIO_7_13 - GPIO18 */

/* CPU */
static struct regulator_consumer_supply ge_imx53_modul_pmic_ltc3589_sw1_consumers[] = {
	{
		.supply = "cpu_vcc",
	}
};

struct ltc3589;

static struct regulator_init_data ge_imx53_modul_pmic_ltc3589_sw1_init = {
	.constraints = {
		.name = "SW1",
		.min_uV = 589000,
		.max_uV = 1219000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = 0,
		.always_on = 1,
		.boot_on = 1,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
			.uV = 950000,
			.mode = REGULATOR_MODE_NORMAL,
			.enabled = 1,
		},
	},
	.num_consumer_supplies = ARRAY_SIZE(ge_imx53_modul_pmic_ltc3589_sw1_consumers),
	.consumer_supplies = ge_imx53_modul_pmic_ltc3589_sw1_consumers,
};

static struct regulator_init_data ge_imx53_modul_pmic_ltc3589_sw2_init = {
	.constraints = {
		.name = "SW2",
		.min_uV = 692000,
		.max_uV = 1432000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
			.uV = 950000,
			.mode = REGULATOR_MODE_NORMAL,
			.enabled = 1,
		},
	},
};

static struct regulator_init_data ge_imx53_modul_pmic_ltc3589_sw3_init = {
	.constraints = {
		.name = "SW3",
		.min_uV = 1359000,
		.max_uV = 2813000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	},
};

static struct regulator_init_data ge_imx53_modul_pmic_ltc3589_sw4_init = {
	.constraints = {
		.name = "SW4",
		.apply_uV = 1,
		.boot_on = 1,
	}
};

static struct regulator_init_data ge_imx53_modul_pmic_ltc3589_ldo1_init = {
	.constraints = {
		.name = "LDO1_STBY",
		.apply_uV = 1,
		.boot_on = 1,
	},
};

static struct regulator_init_data ge_imx53_modul_pmic_ltc3589_ldo2_init = {
	.constraints = {
		.name = "LDO2",
		.min_uV = 697000,
		.max_uV = 1442000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,
		.boot_on = 1,
	},
};

static struct regulator_init_data ge_imx53_modul_pmic_ltc3589_ldo3_init = {
	.constraints = {
		.name = "LDO3",
		.apply_uV = 1,
		.boot_on = 1,
	},
};

static struct regulator_init_data ge_imx53_modul_pmic_ltc3589_ldo4_init = {
	.constraints = {
		.name = "LDO4",
		.min_uV = 1800000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,
		.boot_on = 1,
	},
};

static void ge_imx53_modul_pmic_ltc3589_nop_release(struct device *dev)
{
	/* Nothing */
}

static struct platform_device ge_imx53_modul_pmic_ltc3589_regulator_device[] = {
	{
	.name = "ltc3589-dev",
	.id = 0,
	.dev = {
		.release = ge_imx53_modul_pmic_ltc3589_nop_release,
		},
			},
};

static int ge_imx53_modul_pmic_ltc3589_init(struct ltc3589 *ltc3589)
{
	int i;

	printk(KERN_INFO "Initializing regulators for Ginzinger imx53_modul\n");
	for (i = 0; i < ARRAY_SIZE(ge_imx53_modul_pmic_ltc3589_regulator_device); i++) {
		if (platform_device_register(&ge_imx53_modul_pmic_ltc3589_regulator_device[i]) < 0)
			dev_err(&ge_imx53_modul_pmic_ltc3589_regulator_device[i].dev,
				"Unable to register LTC3589 device\n");
	}

	ltc3589_register_regulator(ltc3589, LTC3589_SW1, &ge_imx53_modul_pmic_ltc3589_sw1_init);
	ltc3589_register_regulator(ltc3589, LTC3589_SW2, &ge_imx53_modul_pmic_ltc3589_sw2_init);
	ltc3589_register_regulator(ltc3589, LTC3589_SW3, &ge_imx53_modul_pmic_ltc3589_sw3_init);
	ltc3589_register_regulator(ltc3589, LTC3589_SW4, &ge_imx53_modul_pmic_ltc3589_sw4_init);
	ltc3589_register_regulator(ltc3589, LTC3589_LDO1, &ge_imx53_modul_pmic_ltc3589_ldo1_init);
	ltc3589_register_regulator(ltc3589, LTC3589_LDO2, &ge_imx53_modul_pmic_ltc3589_ldo2_init);
	ltc3589_register_regulator(ltc3589, LTC3589_LDO3, &ge_imx53_modul_pmic_ltc3589_ldo3_init);
	ltc3589_register_regulator(ltc3589, LTC3589_LDO4, &ge_imx53_modul_pmic_ltc3589_ldo4_init);

	return 0;
}

static struct ltc3589_platform_data ge_imx53_modul_pmic_ltc3589_plat = {
	.init = ge_imx53_modul_pmic_ltc3589_init,
};

static struct i2c_board_info ge_imx53_modul_pmic_ltc3589_i2c_device = {
	I2C_BOARD_INFO("ltc3589", 0x34),
	.irq = gpio_to_irq(GPIO_PMIC_INT),
	.platform_data = &ge_imx53_modul_pmic_ltc3589_plat,
};

static __init int ge_imx53_modul_pmic_ltc3589_init_i2c(void)
{
	/* register ltc on I2C1 */
	return i2c_register_board_info(0, &ge_imx53_modul_pmic_ltc3589_i2c_device, 1);
}

subsys_initcall(ge_imx53_modul_pmic_ltc3589_init_i2c);

/*
 * KEYS
 */
static struct gpio_keys_button ge_imx53_modul_pmic_ltc3589_keys[] = {
	/*
	 * PBSTAT is key
	 */
	{
		.desc="pbstat",
		.gpio=GPIO_PMIC_PBSTAT,
		.code=KEY_SUSPEND,
		.type=EV_KEY,
		.active_low=0,
		.wakeup=1,
		.debounce_interval=0,
		.can_disable=0,
	},
};

/*
 * do initialization/registration of devices (early)
 */
__init int ge_imx53_modul_pmic_ltc3589_pmic_init(void)
{
	int ret = 0;

	/* request pmic */
	gpio_request(GPIO_PMIC_INT, "pmic-int");
	gpio_direction_input(GPIO_PMIC_INT);	/* PMIC_INT */
	gpio_request(GPIO_PMIC_RDY, "pmic-rdy");
	gpio_direction_input(GPIO_PMIC_RDY);	/* PMIC_RDY (TODO: gpio or function) */

	/*
	 * pbstat - key
	 */
	ret=ge_imx53_gpio_keys_add(
		ge_imx53_modul_pmic_ltc3589_keys,
		ARRAY_SIZE(ge_imx53_modul_pmic_ltc3589_keys)
	);

	return ret;
}

/*
 * enable regulators (late
 */
static __init int ge_imx53_modul_pmic_ltc3589_pmic_start(void)
{
	int i = 0;
	struct regulator *regulator;
	char *ltc3589_global_regulator[] = {
		"SW1",
		"SW2",
		"SW3",
		"SW4",
		"LDO1_STBY",
		"LDO2",
		"LDO3",
		"LDO4",
	};

	while ((i < ARRAY_SIZE(ltc3589_global_regulator)) &&
		!IS_ERR_VALUE(
			(unsigned long)(regulator =
					regulator_get(NULL,
						ltc3589_global_regulator
						[i])))) {
		regulator_enable(regulator);
		i++;
	}

	return 0;
}
late_initcall(ge_imx53_modul_pmic_ltc3589_pmic_start);

