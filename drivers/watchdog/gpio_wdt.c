/*
 * linux/drivers/char/watchdog/gpio_wdt.c
 *
 * Copyright (C) 2012 Manfred Schlaegl <manfred.schlaegl@ginzinger.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * 	no warrianty
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/gpio_wdt.h>

#define DRIVER_NAME "gpio_wdt"


/* private data */
static struct {
	unsigned long inuse;
	int state;
	unsigned feed_gpio;
} gpio_wdt_priv;

static void gpio_wdt_feed(void)
{
	/* toggle */
	gpio_wdt_priv.state=!gpio_wdt_priv.state;
	gpio_set_value(gpio_wdt_priv.feed_gpio,gpio_wdt_priv.state);
}


/* Filesystem functions */

static int gpio_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &gpio_wdt_priv.inuse))
		return -EBUSY;
	return nonseekable_open(inode, file);
}


static int gpio_wdt_release(struct inode *inode, struct file *file)
{
	clear_bit(0, &gpio_wdt_priv.inuse);
	return 0;
}

static long gpio_wdt_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = (int __user *)argp;
	static const struct watchdog_info ident = {
		.identity = DRIVER_NAME,
		.options = 0,
		.firmware_version = 0,
	};

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		if (copy_to_user(argp, &ident, sizeof(ident)))
			return -EFAULT;
		break;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		/* no support */
		put_user(0, p);
		break;
	case WDIOC_KEEPALIVE:
		gpio_wdt_feed();
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}


static ssize_t gpio_wdt_write(struct file *file, const char *buf,
						size_t count, loff_t *ppos)
{
	if (!count)
		return -EIO;
	gpio_wdt_feed();
	return count;
}

static const struct file_operations gpio_wdt_fops = {
	.owner 		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= gpio_wdt_ioctl,
	.open 		= gpio_wdt_open,
	.write 		= gpio_wdt_write,
	.release 	= gpio_wdt_release,
};


static struct miscdevice gpio_wdt_misc = {
	.minor 	= WATCHDOG_MINOR,
	.name 	= "watchdog",
	.fops 	= &gpio_wdt_fops,
};


static int __devinit gpio_wdt_probe(struct platform_device *pdev)
{
	int ret;
	struct gpio_wdt_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev,
			"no platform data supplied\n");
		return -ENODEV;
	}

	gpio_wdt_priv.feed_gpio=pdata->feed_gpio;
	gpio_wdt_priv.state=0;
	
	ret = gpio_request(gpio_wdt_priv.feed_gpio,DRIVER_NAME);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to request update GPIO: %d\n",
			ret);
		return ret;
	}

	ret = gpio_direction_output(gpio_wdt_priv.feed_gpio, 0);
	if (ret != 0) {
		dev_err(&pdev->dev,
			"gpio_direction_output returned: %d\n",
			ret);
		goto __ret_free_gpio;
	}

	clear_bit(0, &gpio_wdt_priv.inuse);
	
	ret = misc_register(&gpio_wdt_misc);
	if (ret < 0) {
		dev_err(&pdev->dev, 
			"Failed to register miscdev: %d\n", 
			ret);
		goto __ret_free_gpio;
	}
	
	dev_info(&pdev->dev, "Watchdog initialized and running\n");

	return 0;

__ret_free_gpio:
	gpio_free(gpio_wdt_priv.feed_gpio);
	return ret;
}

static int __devexit gpio_wdt_remove(struct platform_device *pdev)
{
	clear_bit(0, &gpio_wdt_priv.inuse);
	misc_deregister(&gpio_wdt_misc);
	gpio_free(gpio_wdt_priv.feed_gpio);
	return 0;
}

static struct platform_driver gpio_wdt = {
	.probe = gpio_wdt_probe,
	.remove = __devexit_p(gpio_wdt_remove),
	.driver.name = DRIVER_NAME,
	.driver.owner = THIS_MODULE,
};

static int __init gpio_wdt_init(void)
{
	return platform_driver_register(&gpio_wdt);
}

static void __exit gpio_wdt_exit(void)
{
	platform_driver_unregister(&gpio_wdt);
}

module_init(gpio_wdt_init);
module_exit(gpio_wdt_exit);

MODULE_AUTHOR("Manfred Schlaegl");
MODULE_DESCRIPTION("Driver for gpio-based watchdog");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
