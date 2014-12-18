/*
 * Switch class driver
 *
 * Copyright (C) 2012 Melchior FRANZ <melchior.franz@ginzinger.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * derived from LED class driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ge_switch.h>


static struct class *switch_class;

static ssize_t value_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ge_switch_classdev *cdev = dev_get_drvdata(dev);
	if (cdev->switch_value_get)
		cdev->value = cdev->switch_value_get(cdev);
	return snprintf(buf, PAGE_SIZE, "%u\n", cdev->value);
}

static ssize_t init_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ge_switch_classdev *d = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u\n", d->initial_value);
}

static struct device_attribute switch_class_attrs[] = {
	__ATTR(value, 0644, value_show, NULL),
	__ATTR(initial_value, 0444, init_show, NULL),
	__ATTR_NULL,
};

int ge_switch_classdev_register(struct device *parent,
		struct ge_switch_classdev *cdev)
{
	cdev->dev = device_create(switch_class, parent, 0, cdev,
			"%s", cdev->name);
	if (IS_ERR(cdev->dev))
		return PTR_ERR(cdev->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(ge_switch_classdev_register);

void ge_switch_classdev_unregister(struct ge_switch_classdev *cdev)
{
	device_unregister(cdev->dev);
}
EXPORT_SYMBOL_GPL(ge_switch_classdev_unregister);

static int __init ge_switch_class_init(void)
{
	switch_class = class_create(THIS_MODULE, "switches");
	if (IS_ERR(switch_class))
		return PTR_ERR(switch_class);

	switch_class->dev_attrs = switch_class_attrs;
	return 0;
}

static void __exit ge_switch_class_exit(void)
{
	class_destroy(switch_class);
}

subsys_initcall(ge_switch_class_init);
module_exit(ge_switch_class_exit);

MODULE_DESCRIPTION("Ginzinger Switch Class Driver");
MODULE_AUTHOR("Melchior FRANZ <melchior.franz@ginzinger.com>");
MODULE_LICENSE("GPL");
