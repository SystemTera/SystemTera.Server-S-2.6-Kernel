/*
 * max6956.h - platform data structure for max6956 led controller
 *
 * Copyright (C) 2008 Riku Voipio <riku.voipio@movial.fi>
 * Copyright (C) 2013 Melchior FRANZ <melchior.franz@ginzinger.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * Datasheet: http://datasheets.maximintegrated.com/en/ds/MAX6956.pdf
 *
 */

#ifndef __LINUX_MAX6956_H
#define __LINUX_MAX6956_H

#include <linux/leds.h>
#include <linux/workqueue.h>

enum max6956_state {
	MAX6956_OFF  = 0x0,
	MAX6956_ON   = 0x10,
};

enum max6956_type {
	MAX6956_TYPE_NONE,
	MAX6956_TYPE_LED,
};

struct max6956_led {
	u8 id;				// port number (4..32)
	struct i2c_client *client;
	char *name;
	struct led_classdev ldev;
	struct work_struct work;
	enum max6956_type type;
	enum max6956_state state;
};

struct max6956_platform_data {
	struct max6956_led leds[28];
};

#endif /* __LINUX_MAX6956_H */

