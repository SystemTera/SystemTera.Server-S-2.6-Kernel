#ifndef _EDT_FT5X06_H
#define _EDT_FT5X06_H

/*
 * Copyright (c) 2011 Simon Budig, <simon.budig@kernelconcepts.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

struct edt_ft5x06_platform_data {
	int irq_pin;
	int reset_pin;
};

#endif /* _EDT_FT5X06_H */
