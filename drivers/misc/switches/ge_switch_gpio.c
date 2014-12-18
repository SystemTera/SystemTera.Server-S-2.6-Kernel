/*
 * GPIO switch driver
 *
 * Copyright (C) 2012 Melchior FRANZ <melchior.franz@ginzinger.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * derived from GPIO LEDs driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/ge_switch.h>
#include <asm/gpio.h>


struct gpio_switch_data {
	struct ge_switch_classdev cdev;
	struct ge_gpio_pin        *pins;
	unsigned int              num_pins;
};

unsigned int gpio_switch_value_get(struct ge_switch_classdev *sw_cdev)
{
	struct gpio_switch_data *switch_data = container_of(sw_cdev,
			struct gpio_switch_data, cdev);
	unsigned int value = 0;
	int i;

	for (i = 0; i < switch_data->num_pins; i++) {
		struct ge_gpio_pin *pin = &switch_data->pins[i];
		if (!!gpio_get_value(pin->gpio) ^ pin->active_low)
			value |= pin->mask;
	}

	return value;
}

static int __devinit create_gpio_switch(const struct ge_gpio_switch *template,
		struct gpio_switch_data *switch_data, struct device *parent)
{
	int i;
	unsigned int gpio;
	for (i = 0; i < template->num_pins; i++) {
		gpio = template->pins[i].gpio;
		if (!gpio_is_valid(gpio))
			goto err;
		if (gpio_request(gpio, template->name) < 0)
			goto err;
		if (gpio_direction_input(gpio) < 0) {
			gpio_free(gpio);
			goto err;
		}
	}

	switch_data->cdev.name = template->name;
	switch_data->cdev.switch_value_get = gpio_switch_value_get;
	switch_data->num_pins = template->num_pins;
	switch_data->pins = template->pins;
	switch_data->cdev.initial_value = gpio_switch_value_get(&switch_data->cdev);
	switch_data->cdev.value = switch_data->cdev.initial_value;
	return ge_switch_classdev_register(parent, &switch_data->cdev);

err:
	dev_err(parent, "Cannot setup switch gpio %d (%s)\n",
			gpio, template->name);
	for (--i; i >= 0; i--)
		gpio_free(template->pins[i].gpio);

	switch_data->num_pins = 0;
	return 0;
}

static void delete_gpio_switch(struct gpio_switch_data *sw)
{
	int i;
	if (!sw->num_pins)
		return;

	for (i = 0; i < sw->num_pins; i++) {
		unsigned int gpio = sw->pins[i].gpio;
		if (!gpio_is_valid(gpio))
			return;
	}

	ge_switch_classdev_unregister(&sw->cdev);
	for (i = 0; i < sw->num_pins; i++)
		gpio_free(sw->pins[i].gpio);
}

static int __devinit ge_gpio_switch_probe(struct platform_device *pdev)
{
	struct ge_gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	int i, ret = 0;

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct gpio_switch_data)
			* pdata->num_switches, GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	for (i = 0; i < pdata->num_switches; i++) {
		ret = create_gpio_switch(&pdata->switches[i], &switch_data[i],
				&pdev->dev);
		if (ret < 0)
			goto err;
	}

	platform_set_drvdata(pdev, switch_data);
	return 0;

err:
	for (--i; i >= 0; i--)
		delete_gpio_switch(&switch_data[i]);

	kfree(switch_data);
	return ret;
}

static int __devexit ge_gpio_switch_remove(struct platform_device *pdev)
{
	int i;
	struct ge_gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	switch_data = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_switches; i++)
		delete_gpio_switch(&switch_data[i]);

	kfree(switch_data);
	return 0;
}

static struct platform_driver gpio_switch_driver = {
	.driver = {
		.name  = "switches-gpio",
		.owner = THIS_MODULE,
	},
	.probe         = ge_gpio_switch_probe,
	.remove        = __devexit_p(ge_gpio_switch_remove),
};

static int __init ge_gpio_switch_init(void)
{
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit ge_gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

module_init(ge_gpio_switch_init);
module_exit(ge_gpio_switch_exit);

MODULE_DESCRIPTION("Ginzinger GPIO Switch Driver");
MODULE_AUTHOR("Melchior FRANZ <melchior.franz@ginzinger.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:switches-gpio");
