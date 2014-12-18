/*
 * Definitions for variables driver
 *
 * Copyright (C) 2012 Melchior FRANZ <melchior.franz@ginzinger.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 */

#ifndef _GE_VARIABLES_H
#define _GE_VARIABLES_H

struct ge_variable {
	const char *name;
	unsigned int value;               /* static value */
	unsigned int (*get_value)(void);  /* value getter: overrides static value */
};

struct ge_variables_platform_data {
	struct ge_variable *variables;
	unsigned int num_variables;
};

#endif /* _GE_VARIABLES_H */
