/*
 * mcp3208.c - MCP3204/3208 +2.7V, Low-Power, Multichannel, Serial 12-bit ADCs
 *
 * Heavily based on drivers/hwmon/max1111.c
 *
 * Copyright (C) 2004-2005 Richard Purdie
 *
 * Copyright (C) 2008 Marvell International Ltd.
 *	Eric Miao <eric.miao at marvell.com>
 *
 * Copyright (C) 2011 Paul Fertser <fercerpav at gmail.com>
 *
 * Copyright (C) 2012 Manfred Schlaegl <manfred.schlaegl@ginzinger.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
/* used for sysfs-device attributes */
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#define MCP3208_TX_BUF_SIZE	1
#define MCP3208_RX_BUF_SIZE	3

#define MCP3208_CTRL_START	(1<<4)
#define MCP3208_CTRL_SINGLE	(1<<3)

struct mcp3208_data {
	struct spi_device	*spi;
	struct spi_message	msg;
	struct spi_transfer	xfer;
	uint8_t tx_buf[MCP3208_TX_BUF_SIZE];
	uint8_t rx_buf[MCP3208_RX_BUF_SIZE];
	struct mutex		drvdata_lock;
	/* protect msg, xfer and buffers from multiple access */
};

static int mcp3208_read(struct device *dev, int channel)
{
	struct mcp3208_data *data = dev_get_drvdata(dev);
	uint8_t v1, v2;
	int err;
	
	/* writing to drvdata struct is not thread safe, wait on mutex */
	mutex_lock(&data->drvdata_lock);

	data->tx_buf[0] = MCP3208_CTRL_START | channel;

	err = spi_sync(data->spi, &data->msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		mutex_unlock(&data->drvdata_lock);
		return err;
	}

	v1 = data->rx_buf[1];
	v2 = data->rx_buf[2];

	mutex_unlock(&data->drvdata_lock);

	if (v1 & (1<<6))
		return -EINVAL;

	return (v1 & 0x3f) << 6 | (v2 >> 2);
}

/*
 * NOTE: SPI devices do not have a default 'name' attribute, which is
 * likely to be used by hwmon applications to distinguish between
 * different devices, explicitly add a name attribute here.
 */
static ssize_t show_name(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "mcp3208\n");
}

static ssize_t show_adc(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int channel = to_sensor_dev_attr(attr)->index;
	int ret;

	ret = mcp3208_read(dev, channel);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", ret);
}

#define MCP3208_ADC_ATTR(_id)		\
	SENSOR_DEVICE_ATTR(adc##_id##_in, S_IRUGO, show_adc,	\
			   NULL, _id | MCP3208_CTRL_SINGLE)

#define MCP3208_DIFF_ADC_ATTR(_id)	\
	SENSOR_DEVICE_ATTR(adc_diff##_id##_in, S_IRUGO,	\
			   show_adc, NULL, _id)

static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);
static MCP3208_ADC_ATTR(0);
static MCP3208_ADC_ATTR(1);
static MCP3208_ADC_ATTR(2);
static MCP3208_ADC_ATTR(3);
static MCP3208_ADC_ATTR(4);
static MCP3208_ADC_ATTR(5);
static MCP3208_ADC_ATTR(6);
static MCP3208_ADC_ATTR(7);

static MCP3208_DIFF_ADC_ATTR(0);
static MCP3208_DIFF_ADC_ATTR(1);
static MCP3208_DIFF_ADC_ATTR(2);
static MCP3208_DIFF_ADC_ATTR(3);
static MCP3208_DIFF_ADC_ATTR(4);
static MCP3208_DIFF_ADC_ATTR(5);
static MCP3208_DIFF_ADC_ATTR(6);
static MCP3208_DIFF_ADC_ATTR(7);

static struct attribute *mcp3208_attributes[] = {
	&dev_attr_name.attr,
	&sensor_dev_attr_adc0_in.dev_attr.attr,
	&sensor_dev_attr_adc1_in.dev_attr.attr,
	&sensor_dev_attr_adc2_in.dev_attr.attr,
	&sensor_dev_attr_adc3_in.dev_attr.attr,
	&sensor_dev_attr_adc4_in.dev_attr.attr,
	&sensor_dev_attr_adc5_in.dev_attr.attr,
	&sensor_dev_attr_adc6_in.dev_attr.attr,
	&sensor_dev_attr_adc7_in.dev_attr.attr,
	&sensor_dev_attr_adc_diff0_in.dev_attr.attr,
	&sensor_dev_attr_adc_diff1_in.dev_attr.attr,
	&sensor_dev_attr_adc_diff2_in.dev_attr.attr,
	&sensor_dev_attr_adc_diff3_in.dev_attr.attr,
	&sensor_dev_attr_adc_diff4_in.dev_attr.attr,
	&sensor_dev_attr_adc_diff5_in.dev_attr.attr,
	&sensor_dev_attr_adc_diff6_in.dev_attr.attr,
	&sensor_dev_attr_adc_diff7_in.dev_attr.attr,
	NULL,
};

static const struct attribute_group mcp3208_attr_group = {
	.attrs	= mcp3208_attributes,
};

static int __devinit setup_transfer(struct mcp3208_data *data)
{
	struct spi_message *m;
	struct spi_transfer *x;

	m = &data->msg;
	x = &data->xfer;

	spi_message_init(m);

	x->tx_buf = data->tx_buf;
	x->rx_buf = data->rx_buf;
	x->len = (MCP3208_TX_BUF_SIZE > MCP3208_RX_BUF_SIZE) ? (MCP3208_TX_BUF_SIZE) : (MCP3208_RX_BUF_SIZE);
	spi_message_add_tail(x, m);

	return 0;
}

static int __devinit mcp3208_probe(struct spi_device *spi)
{
	struct mcp3208_data *data;
	int err;
	
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	err = spi_setup(spi);
	if (err < 0)
		return err;

	data = kzalloc(sizeof(struct mcp3208_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&spi->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	err = setup_transfer(data);
	if (err)
		goto err_free_data;

	mutex_init(&data->drvdata_lock);

	data->spi = spi;
	spi_set_drvdata(spi, data);

	err = sysfs_create_group(&spi->dev.kobj, &mcp3208_attr_group);
	if (err) {
		dev_err(&spi->dev, "failed to create attribute group\n");
		goto err_free_mutex;
	}

	return 0;

err_free_mutex:
	mutex_destroy(&data->drvdata_lock);
err_free_data:
	kfree(data);
	return err;
}

static int __devexit mcp3208_remove(struct spi_device *spi)
{
	struct mcp3208_data *data = spi_get_drvdata(spi);

	sysfs_remove_group(&spi->dev.kobj, &mcp3208_attr_group);
	mutex_destroy(&data->drvdata_lock);
	kfree(data);
	return 0;
}

static struct spi_driver mcp3208_driver = {
	.driver		= {
		.name	= "mcp3208",
		.owner	= THIS_MODULE,
	},
	.probe		= mcp3208_probe,
	.remove		= __devexit_p(mcp3208_remove),
};

static int __init mcp3208_init(void)
{
	return spi_register_driver(&mcp3208_driver);
}
module_init(mcp3208_init);

static void __exit mcp3208_exit(void)
{
	spi_unregister_driver(&mcp3208_driver);
}
module_exit(mcp3208_exit);

MODULE_AUTHOR("Paul Fertser <fercerpav at gmail.com>");
MODULE_DESCRIPTION("MCP3208 ADC Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:mcp3208");
