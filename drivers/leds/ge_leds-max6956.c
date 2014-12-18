/*
 * max6956.c - 28-bit LED driver (derived from leds-pca9532.c by Riku Voipio)
 *
 * Copyright (C) 2013 Melchior FRANZ <melchior.franz@ginzinger.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * Datasheet: http://datasheets.maximintegrated.com/en/ds/MAX6956.pdf
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/ge_leds-max6956.h>

#define NUM_LEDS 28

#define CONF_REG(id) (0x08 + (id) / 4)
#define CONF_SHIFT(id) (((id) % 4) * 2)

#define ldev_to_led(c) container_of(c, struct max6956_led, ldev)


struct max6956_data {
	struct i2c_client *client;
	struct max6956_led leds[NUM_LEDS];
	struct mutex update_lock;
	struct input_dev *idev;
	struct work_struct work;
};

static int max6956_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int max6956_remove(struct i2c_client *client);

static const struct i2c_device_id max6956_id[] = {
	{ "max6956", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, max6956_id);

static struct i2c_driver max6956_driver = {
	.driver = {
		.name = "max6956",
	},
	.probe = max6956_probe,
	.remove = max6956_remove,
	.id_table = max6956_id,
};

static void max6956_setled(struct max6956_led *led)
{
	struct i2c_client *client = led->client;
	struct max6956_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->update_lock);

	if (led->state > 0) {
		int reg = 0x10 + led->id / 2;
		u32 val = i2c_smbus_read_byte_data(client, reg);
		int brightness = led->state - 1;
		if (val >= 0) {
			if (led->id & 1)
				val = (val & 0x0f) | (brightness << 4);
			else
				val = (val & 0xf0) | brightness;
			i2c_smbus_write_byte_data(client, reg, val);
		}
	}

	i2c_smbus_write_byte_data(client, 0x20 + led->id, !!led->state);
	mutex_unlock(&data->update_lock);
}

static void max6956_set_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct max6956_led *led = ldev_to_led(led_cdev);
	led->state = value;
	schedule_work(&led->work);
}

static void max6956_led_work(struct work_struct *work)
{
	struct max6956_led *led;
	led = container_of(work, struct max6956_led, work);
	max6956_setled(led);
}

static int max6956_configure(struct i2c_client *client,
		struct max6956_data *data, struct max6956_platform_data *pdata)
{
	int i, err;
	mutex_lock(&data->update_lock);
	err = i2c_smbus_write_byte_data(client, 0x04, 0x41);	// normal op, individual current (brightness)
	mutex_unlock(&data->update_lock);

	if (err) {
		dev_err(&client->dev, "configuration failed\n");
		return err;
	}

	for (i = 0; i < NUM_LEDS; i++) {
		u32 val;
		struct max6956_led *led = &data->leds[i];
		struct max6956_led *pled = &pdata->leds[i];
		int id = pled->id;
		if (!id)		// not initialized
			continue;
		if (id < 4 || id > 32) {
			dev_err(&client->dev, "ignoring illegal LED id %d (range 4..32)\n", id);
			continue;
		}

		led->client = client;
		led->id = id;
		led->type = pled->type;

		switch (led->type) {
		case MAX6956_TYPE_NONE:
			break;
		case MAX6956_TYPE_LED:
			led->state = pled->state;
			led->name = pled->name;
			led->ldev.name = led->name;
			led->ldev.max_brightness = 16;
			led->ldev.brightness = LED_OFF;
			led->ldev.brightness_set = max6956_set_brightness;

			mutex_lock(&data->update_lock);
			val = i2c_smbus_read_byte_data(client, CONF_REG(led->id));
			if (val >= 0) {
				val &= ~(0x3 << CONF_SHIFT(led->id));
				i2c_smbus_write_byte_data(client, CONF_REG(led->id), val & 0xff);
			}
			mutex_unlock(&data->update_lock);

			INIT_WORK(&led->work, max6956_led_work);
			err = led_classdev_register(&client->dev, &led->ldev);
			if (err < 0) {
				dev_err(&client->dev,
					"couldn't register LED %s\n",
					led->name);
				goto exit;
			}
			max6956_setled(led);
			break;
		}
	}
	return 0;

exit:
	if (i > 0) {
		for (i = i - 1; i >= 0; i--) {
			switch (data->leds[i].type) {
			case MAX6956_TYPE_NONE:
				break;
			case MAX6956_TYPE_LED:
				led_classdev_unregister(&data->leds[i].ldev);
				cancel_work_sync(&data->leds[i].work);
				break;
			}
		}
	}

	return err;
}

static int max6956_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct max6956_data *data = i2c_get_clientdata(client);
	struct max6956_platform_data *max6956_pdata = client->dev.platform_data;
	int err;

	if (!max6956_pdata)
		return -EIO;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev_info(&client->dev, "setting platform data\n");
	i2c_set_clientdata(client, data);
	data->client = client;
	mutex_init(&data->update_lock);

	err = max6956_configure(client, data, max6956_pdata);
	if (err)
		kfree(data);

	return err;
}

static int max6956_remove(struct i2c_client *client)
{
	struct max6956_data *data = i2c_get_clientdata(client);
	int i;
	for (i = 0; i < NUM_LEDS; i++)
		switch (data->leds[i].type) {
		case MAX6956_TYPE_NONE:
			break;
		case MAX6956_TYPE_LED:
			led_classdev_unregister(&data->leds[i].ldev);
			cancel_work_sync(&data->leds[i].work);
			break;
		}

	kfree(data);
	return 0;
}

static int __init max6956_init(void)
{
	return i2c_add_driver(&max6956_driver);
}

static void __exit max6956_exit(void)
{
	i2c_del_driver(&max6956_driver);
}

MODULE_AUTHOR("Melchior FRANZ <melchior.franz@ginzinger.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAX6956 28-bit LED Driver");

module_init(max6956_init);
module_exit(max6956_exit);
