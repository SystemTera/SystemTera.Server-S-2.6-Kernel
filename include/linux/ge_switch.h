/*
 * Definitions for switch driver
 *
 * Copyright (C) 2012 Melchior FRANZ <melchior.franz@ginzinger.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 */

#ifndef _GE_SWITCH_H
#define _GE_SWITCH_H

struct device;

struct ge_switch_classdev {
	const char *name;
	struct device *dev;
	unsigned int value;
	unsigned int initial_value;
	unsigned int (*switch_value_get)(struct ge_switch_classdev *cdev);
};

extern int ge_switch_classdev_register(struct device *parent,
		struct ge_switch_classdev *cdev);
extern void ge_switch_classdev_unregister(struct ge_switch_classdev *cdev);

struct ge_gpio_pin {
	unsigned int gpio;
	unsigned int mask;
	unsigned char active_low:1;
};

struct ge_gpio_switch {
	const char *name;
	struct ge_gpio_pin *pins;
	unsigned int num_pins;
};

struct ge_gpio_switch_platform_data {
	int num_switches;
	struct ge_gpio_switch *switches;
};

#endif /* _GE_SWITCH_H */
