/*
 *	BOOT LEDS
 *	AUTHOR: Patrik Pfaffenbauer
 *	DATE: 04-04-2013
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>	
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <asm/uaccess.h>

#include <linux/leds.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <mach/gpio.h>
#include <linux/slab.h>
#include "leds.h"

#define GPIO_LED1_ROT           (0*32 + 7)      /* GPIO_1_7 - GPIO_7 */
#define GPIO_LED1_GRUEN         (0*32 + 8)      /* GPIO_1_8 - GPIO_8 */
#define GPIO_LED2_ROT           (4*32 + 20)     /* GPIO_5_20 - CSI0_DATA_EN */
#define GPIO_LED2_GRUEN         (4*32 + 19)     /* GPIO_5_19 - CSI0_MCLK */
#define GPIO_LED3_ROT           (4*32 + 18)     /* GPIO_5_18 - CSI0_PIXC*/
#define GPIO_LED3_GRUEN         (4*32 + 21)     /* GPIO_5_21 - GPIO_VSYNC */

#define DEV_NAME "lc1"

static dev_t first;
static struct cdev c_dev;
static struct class *cl;
static char state;

static void init_boot_leds(void);
static void init_boot_led(int i);

struct timer_data {
        struct timer_list timer;
		int brightness;
        int led_info;
};

enum led_color {
	LED_COLOR_OFF = 0,
	LED_GREEN,
	LED_RED,
	LED_ORANGE
};

enum led_operation {
	LED_BLINK = 0,
	LED_ON
};

struct gpio_leds {
	enum led_color color;
	enum led_operation operation;
	unsigned gpio_red;
	unsigned gpio_green;
	struct timer_data* timer;
};

static struct gpio_leds leds[] = {
	{
		.color = LED_ORANGE,
		.operation = LED_BLINK,
		.gpio_red = GPIO_LED1_ROT,
		.gpio_green = GPIO_LED1_GRUEN,
		.timer = 0
	},	{
		.color = LED_ORANGE,
		.operation = LED_ON,
		.gpio_red = GPIO_LED2_ROT,
		.gpio_green = GPIO_LED2_GRUEN,
		.timer = 0
	},	{
		.color = LED_ORANGE,
		.operation = LED_ON,
		.gpio_red = GPIO_LED3_ROT,
		.gpio_green = GPIO_LED3_GRUEN,
		.timer = 0
	}
};

static int device_open(struct inode* inode, struct file *file) {
        return 0;
};

static int device_release(struct inode *inode, struct file *file) {
        return 0;
};

static ssize_t device_read(struct file *filep, char* buffer, size_t length, loff_t* offset) {
	return 0;
};

static ssize_t device_write(struct file *filep, const char *buffer, size_t length, loff_t *offset) {
    
	int ledNr = -1;
	enum led_operation operation;
	enum led_color color;
		
	if(length >= 6)
	{
		char newState[6];
		copy_from_user(&newState, buffer, 6);
		
		ledNr = (newState[0] - 48);
		operation = (enum led_operation)(newState[2] - 48);
		color = (enum led_color)(newState[4] - 48);
		
		if((ledNr >= 0 && ledNr <= 2) && (operation >= 0 && operation <= 1) && (color >= 0 && color <= 3))
		{
			leds[ledNr].operation = operation;
			leds[ledNr].color = color;

			init_boot_led(ledNr);
		}
		
	}
	return length;
};

static struct file_operations fops = {
        .read = device_read,
        .write = device_write,
        .open = device_open,
        .release = device_release
};

static void timer_func(unsigned long data) 
{
        struct timer_data *timer = (struct timer_data*)data;
		
		if(!timer)
			return;

		switch(leds[timer->led_info].color)
		{
			case LED_ORANGE:
				 gpio_set_value(leds[timer->led_info].gpio_red, timer->brightness);
				 gpio_set_value(leds[timer->led_info].gpio_green, timer->brightness);
			break;
			case LED_RED:
				 gpio_set_value(leds[timer->led_info].gpio_red, timer->brightness);
				 gpio_set_value(leds[timer->led_info].gpio_green, 0);
			break;
			case LED_GREEN:
				gpio_set_value(leds[timer->led_info].gpio_green, timer->brightness);
				 gpio_set_value(leds[timer->led_info].gpio_red, 0);
			break;
			case LED_COLOR_OFF:
				 gpio_set_value(leds[timer->led_info].gpio_red, 0);
				 gpio_set_value(leds[timer->led_info].gpio_green, 0);
			break;
		}

        timer->brightness = !timer->brightness;

		if(leds[timer->led_info].operation == LED_BLINK)
		{
			mod_timer(&timer->timer, jiffies + 10);
		}
		else 
		{
			leds[timer->led_info].timer = 0;
			del_timer_sync(&timer->timer);
			kfree(timer);
		}
};

void init_boot_leds(void) {
	int i;
	state = 1;

	for(i = 0; i < ARRAY_SIZE(leds); i++) {
		init_boot_led(i);
	}
};

void init_boot_led(int i) {

	struct timer_data *timer;
	
	if(leds[i].timer)
	{
		del_timer_sync(&leds[i].timer->timer);
		kfree(leds[i].timer);
		leds[i].timer = 0;
	}
	
	if(leds[i].color == LED_COLOR_OFF) {
		gpio_set_value(leds[i].gpio_red, 0);
		gpio_set_value(leds[i].gpio_green, 0);
	}
	else {
		timer = kzalloc(sizeof(struct timer_data), GFP_KERNEL);

		if(!timer)
			return;
		timer->led_info=i;
		timer->brightness = 1;
		leds[i].timer = timer;
		timer->timer.function = timer_func;
		timer->timer.data = (unsigned long)timer;
		init_timer(&timer->timer);
		add_timer(&timer->timer);
		
		timer_func((unsigned long)timer);
	}
};

static int init_device(void) {
	if(alloc_chrdev_region(&first, 0, 1, DEV_NAME) < 0){
		return -1;
	}

	if((cl = class_create(THIS_MODULE, DEV_NAME)) == NULL) {
		unregister_chrdev_region(first, 1);
		return -1;
	}

	if(device_create(cl, NULL, first, NULL, DEV_NAME) == NULL) {
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return -1;
	}

	cdev_init(&c_dev, &fops);
	if(cdev_add(&c_dev, first, 1) == -1) {
		device_destroy(cl, first);
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return -1;
	}

	return 0;
}


static int __init start_module(void) {
	state = -1;

	if(init_device() != 0)
	{
		printk(KERN_INFO "Could not load led-control module -> char device could not be created");
		return -1;
	}
	init_boot_leds();
	printk(KERN_INFO "Loading led-control module (c) BeKa-Software 2012 - 2013");
	printk(KERN_INFO "Original written by Patrik Pfaffenbauer");
	return 0;
};

static void __exit close_module(void) {
	cdev_del(&c_dev);
	device_destroy(cl, first);
	class_destroy(cl);
	unregister_chrdev_region(first, 1);

};
	

module_init(start_module);
module_exit(close_module);

MODULE_AUTHOR("Patrik Pfaffenbauer");
MODULE_DESCRIPTION("Inits the systemtera.server boot leds");
MODULE_LICENSE("GPL");
