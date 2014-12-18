/*
 * Definitions for gpio-based watchdog (gpio_wdt)
 *
 * Copyright (C) 2012 Manfred Schlaegl <manfred.schlaegl@ginzinger.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * 	no warrianty
 */

#ifndef _GPIO_WDT_H
#define _GPIO_WDT_H

struct gpio_wdt_platform_data {
	unsigned feed_gpio;                /* GPIO number for gpio-feed */
};

#endif /* _GPIO_WDT_H */
