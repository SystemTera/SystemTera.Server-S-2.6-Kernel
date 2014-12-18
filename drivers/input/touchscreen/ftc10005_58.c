/***********************************************************************
 * drivers/input/touchscreen/ftc10005_58.c
 *
 * Using code from:
 * - drivers/input/touchscreen/edt-ft5x06.c
 * 	Copyright (C) 2011 Simon Budig, <simon.budig@kernelconcepts.de>
 * - drivers/input/touchscreen/ad7879.c
 * 	Copyright (C) 2008-2009 Michael Hennerich, Analog Devices Inc.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 ***********************************************************************/
/***********************************************************************
 *  @History:
 *	2012KW07 - manfred.schlaegl: 
 *		* implemented and tested
 *		* merge driver-code
 *		* begin implementation
 *  @TODO:
 *	* cleanup
 *	* validate
 *	* optimize
 ***********************************************************************/

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>
#include <linux/smp_lock.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/gpio.h>

struct ftc10005_58_spi_ts_data
{
	struct spi_device *spi;
	struct input_dev *input;
	struct mutex mutex;

	int irq;
	int irq_pin;

	/* config */
	int sens1;
	int sens2;
	int charge;
	int pendown_val;
	int penup_threshold;
	int min_x;
	int max_x;
	int min_y;
	int max_y;
	int min_z;
	int max_z;
};

/******************************************************************************
 * BEGIN: HARDWARE DRIVER CONFIGURATION AND FUNCTIONS
 * merged from https://development.ginzinger.com/armlinux/browser/software/arm7/arm7_pnet/trunk/pnet/pnet_ts_cap.c?rev=5792
 ******************************************************************************/

#define FTC10005_58_READ		0xa0
#define FTC10005_58_WRITE		0x80
#define FTC10005_58_FILL		0xfb
#define FTC10005_58_INC			0xfc

//commands
#define FTC10005_58_STATUS		0x02
#define FTC10005_58_POWER		0x03
#define FTC10005_58_CONTROL		0x04
#define FTC10005_58_CALIBRATE		0x07
#define FTC10005_58_Z_IDLE		0x0a
#define FTC10005_58_X_POS_LOW		0x14
#define FTC10005_58_Y_POS_LOW		0x15
#define FTC10005_58_XY_POS_HIGH		0x16
#define FTC10005_58_Z_LEVEL		0x17

//bitcodings
#define FTC10005_58_STATUS_POST		0x08
#define FTC10005_58_STATUS_DR		0x04

#define FTC10005_58_CONTROL_YINV	0x80
#define FTC10005_58_CONTROL_XINV	0x40
#define FTC10005_58_CONTROL_AR		0x02
#define FTC10005_58_CONTROL_EN		0x01

#define FTC10005_58_POWER_RESET		0x01

#define FTC10005_58_CHARGE_1000NS	0x08
#define FTC10005_58_CHARGE_500NS	0x00

//config
/* controller timeout */
#define FTC10005_58_INIT_TIMEOUT		msecs_to_jiffies(200)
/* message delay */
#define FTC10005_58_MESSAGE_DELAY		50	/* us (TODO: optimize) */
/* controller */
#define	FTC10005_58_SENS1			1			//0-3
#define FTC10005_58_SENS2			1			//0-3
#define FTC10005_58_CHARGE			FTC10005_58_CHARGE_1000NS	//500NS, 1000NS
/* alg */
#define FTC10005_58_PENDOWN_VAL			15
#define FTC10005_58_PENUP_THRESHOLD		5
/* coords */
#warning "TODO: manfred.schlaegl@ginzinger.com: use correct settings for ocular-touch"
#if 0
/* settings from userguide */
#define FTC10005_58_MIN_X			64
#define FTC10005_58_MAX_X			1983
#define FTC10005_58_MIN_Y			64
#define FTC10005_58_MAX_Y			1471
#else
/* settings from andreas schrattenecker */
#define FTC10005_58_MIN_X			0
#define FTC10005_58_MAX_X			2047
#define FTC10005_58_MIN_Y			0
#define FTC10005_58_MAX_Y			1535
#endif
#define FTC10005_58_MIN_Z			0
#define FTC10005_58_MAX_Z			63


static int ftc10005_58_read(struct spi_device *spi, unsigned char command)
{
	int ret;
	unsigned char out[5];
	unsigned char in[5];
	struct spi_transfer xfer;
	struct spi_message msg;

	/* reset */
	memset(&xfer,0,sizeof(struct spi_transfer));
	memset(&msg,0,sizeof(struct spi_message));

	/* buffer to write */
	out[0]=FTC10005_58_READ|command;
	out[1]=FTC10005_58_FILL;	//filler byte
	out[2]=FTC10005_58_FILL;	//filler byte
	out[3]=FTC10005_58_FILL;	//filler byte und status lesen

	/* transfer */
	xfer.tx_buf=&out;
	xfer.rx_buf=&in;
	xfer.len=4;
	xfer.delay_usecs=FTC10005_58_MESSAGE_DELAY;
	/* message */
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	/* rw */
	udelay(FTC10005_58_MESSAGE_DELAY);
	ret = spi_sync(spi, &msg);
	if(ret<0)
		return ret;

	/* check for filler bytes in reply */
	if(
		(in[1]!=FTC10005_58_FILL)	||
		(in[2]!=FTC10005_58_FILL)
	) {
		return -EPROTO;
	}

	/* ok */
	return (unsigned int)in[3];
}

static int ftc10005_58_write(struct spi_device *spi, unsigned char command, unsigned char value)
{
	int ret;
	unsigned char out[2];
	unsigned char in[2];
	struct spi_transfer xfer;
	struct spi_message msg;

	/* reset */
	memset(&xfer,0,sizeof(struct spi_transfer));
	memset(&msg,0,sizeof(struct spi_message));

	/* buffer to write */
	out[0]=FTC10005_58_WRITE|command;
	out[1]=value;

	/* transfer */
	xfer.tx_buf=&out;
	xfer.rx_buf=&in;
	xfer.len=2;
	xfer.delay_usecs=FTC10005_58_MESSAGE_DELAY;
	/* message */
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	/* rw */
	udelay(FTC10005_58_MESSAGE_DELAY);
	ret=spi_sync(spi, &msg);
	if(ret<0)
		return ret;

	/* check reply */
	if(
		(in[1]!=FTC10005_58_FILL)
	) {
		return -EPROTO;
	}

	/* ok */
	return 0;
}	

static int ftc10005_58_status_read(struct spi_device *spi)
{
	return ftc10005_58_read(spi,FTC10005_58_STATUS);
}

static int ftc10005_58_status_clear(struct spi_device *spi)
{
	//clear status
	return ftc10005_58_write(spi,FTC10005_58_STATUS,0);
}

/*
 * do mult read for xyz
 */
static int ftc10005_58_xyz_pos_read(struct spi_device *spi, unsigned short xyz[3])
{
	int ret;
	unsigned char out[7];
	unsigned char in[7];
	struct spi_transfer xfer;
	struct spi_message msg;
  	unsigned char x_low,y_low,xy_high,z;

	/* reset */
	memset(&xfer,0,sizeof(struct spi_transfer));
	memset(&msg,0,sizeof(struct spi_message));

	/* buffer to write */
	out[0]=FTC10005_58_READ|FTC10005_58_X_POS_LOW;
	out[1]=FTC10005_58_INC;		//inc byte
	out[2]=FTC10005_58_INC;		//inc byte
	out[3]=FTC10005_58_INC;		//inc byte, read x_low
	out[4]=FTC10005_58_INC;		//inc byte, read y_low
	out[5]=FTC10005_58_INC;		//inc byte, read xy_high
	out[6]=FTC10005_58_FILL;	//filler byte, read z

	/* transfer */
	xfer.tx_buf=&out;
	xfer.rx_buf=&in;
	xfer.len=7;
	xfer.delay_usecs=FTC10005_58_MESSAGE_DELAY;
	/* message */
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	/* rw */
	udelay(FTC10005_58_MESSAGE_DELAY);
	ret = spi_sync(spi, &msg);
	if(ret<0)
		return ret;
	x_low=in[3];
	y_low=in[4];
	xy_high=in[5];
	z=in[6];

	/* convert data */
	xyz[0]=((xy_high&0x0f)<<8)|x_low;
	xyz[1]=((xy_high&0xf0)<<4)|y_low;
	xyz[2]=(z&0x3f);

	return 0;
}


static int ftc10005_58_control_write(struct spi_device *spi, unsigned char control)
{
	return ftc10005_58_write(spi,FTC10005_58_CONTROL,control);
}


/*
 * return: <0 .. error, 0 .. ok, 1 .. timeout 
 */
static int ftc10005_58_control_wait_status_post(struct spi_device *spi)
{
	int ret;
	int timeout;
	unsigned char reg;

	timeout=jiffies+FTC10005_58_INIT_TIMEOUT;
	while(jiffies<timeout) {
		ret=ftc10005_58_status_read(spi);
		/* error */
		if(ret<0)
			return ret;
		reg=ret;
		/* ok */
		if(reg&FTC10005_58_STATUS_POST)
			return 0;
	}

	/* timeout */
	return 1;
}

/*
 * return: <0 .. error, 0 .. ok, 1 .. timeout 
 */
static int ftc10005_58_control_wait(struct spi_device *spi)
{
	int ret;
	int timeout;
	unsigned char reg;

	timeout=jiffies+FTC10005_58_INIT_TIMEOUT;
	while(jiffies<timeout) {
		ret=ftc10005_58_read(spi,0x1e);

		/* error */
		if(ret<0)
			return ret;
		reg=ret;
		/* ok */
		if(!reg)
			return 0;
	}

	/* timeout */
	return 1;
}

/*
 * return: <0 .. error, 0 .. ok, 1 .. timeout 
 */
static int ftc10005_58_config(struct spi_device *spi, unsigned char sens1, unsigned char sens2, unsigned char charge)
{
	int ret;
	unsigned char reg;

	//disable finger tracking
	ret=ftc10005_58_write(spi,FTC10005_58_POWER_RESET,0x08);
	//wait 10ms
	mdelay(10);

	//read modify write extended register 0x187 sensitivity1
	//(attention: wrong address 0x178 in user manual)
	//upper address
	ret=ftc10005_58_write(spi,0x1c,0x01);
	if(ret<0)
		return ret;
	//lower address
	ret=ftc10005_58_write(spi,0x1d,0x87);
	if(ret<0)
		return ret;

	//read command
	ret=ftc10005_58_write(spi,0x1e,0x01);
	if(ret<0)
		return ret;

	// wait
	ret=ftc10005_58_control_wait(spi);
	if(ret)
		return ret;

	// read/modify/write
	ret=ftc10005_58_read(spi,0x1b);
	if(ret<0)
		return ret;
	reg=ret;
	reg=(reg&0x3f)|(sens1<<6);
	ret=ftc10005_58_write(spi,0x1b,reg);
	if(ret<0)
		return ret;

	//write command
	ret=ftc10005_58_write(spi,0x1e,0x02);
	if(ret<0)
		return ret;

	// wait
	ret=ftc10005_58_control_wait(spi);
	if(ret)
		return ret;

	//read modify write extended register 0x189 sensitivity2
	//necessary regarding irlbacher (register not in user manual)
	//upper address
	ret=ftc10005_58_write(spi,0x1c,0x01);
	if(ret<0)
		return ret;
	//lower address
	ret=ftc10005_58_write(spi,0x1d,0x89);	
	if(ret<0)
		return ret;

	//read command
	ret=ftc10005_58_write(spi,0x1e,0x01);	
	if(ret<0)
		return ret;

	// wait
	ret=ftc10005_58_control_wait(spi);
	if(ret)
		return ret;

	// read/modify/write
	ret=ftc10005_58_read(spi,0x1b);
	if(ret<0)
		return ret;
	reg=ret;
	reg=(reg&0x3f)|(sens2<<6);
	ret=ftc10005_58_write(spi,0x1b,reg);
	if(ret<0)
		return ret;

	ret=ftc10005_58_write(spi,0x1e,0x02);
	if(ret<0)
		return ret;

	// wait
	ret=ftc10005_58_control_wait(spi);
	if(ret)
		return ret;


	//read modify write extended register 0x00EE charge time
	//upper address
	ret=ftc10005_58_write(spi,0x1c,0x00);
	if(ret<0)
		return ret;
	//lower address
	ret=ftc10005_58_write(spi,0x1d,0xee);	
	if(ret<0)
		return ret;

	//read command
	ret=ftc10005_58_write(spi,0x1e,0x01);	
	if(ret<0)
		return ret;

	// wait
	ret=ftc10005_58_control_wait(spi);
	if(ret<0)
		return ret;

	// read/modify/write
	ret=ftc10005_58_read(spi,0x1b);
	if(ret<0)
		return ret;
	reg=ret;
	reg=(reg&0xf0)|charge;
	ret=ftc10005_58_write(spi,0x1b,reg);
	if(ret<0)
		return ret;

	ret=ftc10005_58_write(spi,0x1e,0x02);
	if(ret<0)
		return ret;

	// wait
	ret=ftc10005_58_control_wait(spi);
	if(ret)
		return ret;
	
	//clear status
	ret=ftc10005_58_status_clear(spi);
	if(ret<0)
		return ret;

	//enable finger tracking
	ret=ftc10005_58_write(spi,FTC10005_58_POWER_RESET,0x00);
	if(ret<0)
		return ret;

	//calibrate
	//read modify write necessary regarding ocular
	ret=ftc10005_58_read(spi,FTC10005_58_CALIBRATE);
	if(ret<0)
		return ret;
	reg=ret;
	ret=ftc10005_58_write(spi,FTC10005_58_CALIBRATE,reg|0x01);	
	if(ret<0)
		return ret;

	//wait 100ms
	msleep(100);

	// wait till POST (power on self test)
	ret=ftc10005_58_control_wait_status_post(spi);
	if(ret)
		return ret;

	//clear status
	ret=ftc10005_58_status_clear(spi);
	if(ret<0)
		return ret;

	/* ok */
	return 0;
}

/*
 * return: <0 .. error, 0 .. ok, 1 .. timeout 
 */
static int ftc10005_58_init(struct ftc10005_58_spi_ts_data *tsdata)
{
	int ret;
	struct spi_device *spi=tsdata->spi;

	//100ms wartezeit für kalibrierung
//	msleep(100);

	/* SPI INIT - SPI */
	//Reset Controller - otherwise POST would not be set on software reset
	ret=ftc10005_58_write(spi,FTC10005_58_POWER,FTC10005_58_POWER_RESET);
	if(ret<0)
		return -1;

	//neccessary wait, otherwise strange behaviour
	msleep(100);
	// wait till POST (power on self test)
	ret=ftc10005_58_control_wait_status_post(spi);
	if(ret)
		return ret;
	//enable crytal touch, absolut mode
	ret=ftc10005_58_control_write(spi,FTC10005_58_CONTROL_EN|FTC10005_58_CONTROL_AR);
	if(ret<0)
		return ret;

	//clear status
	ret=ftc10005_58_status_clear(spi);
	if(ret<0)
		return ret;
	
	//sens=0 -> gain=4 (max sensitivity) -> bereits erkennung ohne berührung
	//sens=1 -> gain=3 (sensitivity) -> rechts oben zu wenig sensibel. sonst bereits berührung wenn sehr nahe
	//sens=2 -> gain=2 (sensitivity) -> rechts oben öfters keine erkennung
	//sens=3 -> gain=1 (min sensitivity) default after reset -> rechts oben sehr oft keine erkennung
	if(ftc10005_58_config(
	 	spi,
		tsdata->sens1,
		tsdata->sens2,
		tsdata->charge
	)) {
		return -1;
	}
	
	//enable crytal touch, absolut mode
	ret=ftc10005_58_control_write(spi,FTC10005_58_CONTROL_EN|FTC10005_58_CONTROL_AR);
	if(ret<0)
		return ret;

	//lesser Z-idle packets (TODO optimize and test)
	ret=ftc10005_58_write(spi,FTC10005_58_Z_IDLE,5);
	if(ret<0)
		return ret;

	//clear status
	ret=ftc10005_58_status_clear(spi);
	if(ret<0)
		return ret;

	/* ok */
	return 0;
}

/******************************************************************************
 * END: HARDWARE DRIVER CONFIGURATION AND FUNCTIONS
 ******************************************************************************/

static irqreturn_t ftc10005_58_ts_isr (int irq, void *dev_id)
{
	struct ftc10005_58_spi_ts_data *tsdata = dev_id;
	int ret;
	unsigned short xyz[3];
	unsigned char status;
	int pendown_val,penup_val;

	mutex_lock (&tsdata->mutex);

	/* read status register */
	ret=ftc10005_58_status_read(tsdata->spi);
	if(ret<0) {
		dev_err (&tsdata->spi->dev, "Unable to read status from spi touchscreen!\n");
		mutex_unlock (&tsdata->mutex);
		goto out;
	}
	status=ret;

	/* check, if touch-controller is present and initialized */
	if(status&FTC10005_58_STATUS_POST) {
		/* touch-controller offline, or reset -> abort */
		dev_err (&tsdata->spi->dev, "Touchcontroller failure: Reset/Offline!\n");
		mutex_unlock (&tsdata->mutex);
		goto out;
	}

	if(status&FTC10005_58_STATUS_DR) {
		/* read pos */
		ret=ftc10005_58_xyz_pos_read(tsdata->spi,xyz);
		if(ret<0) {
			dev_err (&tsdata->spi->dev, "Unable to coords from spi touchscreen!\n");
			goto out;
		}

		/* clear status */
		ret=ftc10005_58_status_clear(tsdata->spi);
		if(ret<0) {
			dev_err (&tsdata->spi->dev, "Unable to clear status of spi touchscreen!\n");
			goto out;
		}

		mutex_unlock (&tsdata->mutex);

		pendown_val=tsdata->pendown_val;	
		penup_val=tsdata->pendown_val-tsdata->penup_threshold;	
		if (penup_val<0) 
			penup_val=0;
		
		if (xyz[2]>pendown_val) {
			// DOWN
			input_report_key (tsdata->input, BTN_TOUCH,    1);
			input_report_abs (tsdata->input, ABS_X, xyz[0]);
			input_report_abs (tsdata->input, ABS_Y, xyz[1]);
			input_report_abs (tsdata->input, ABS_PRESSURE, xyz[2]);
		}
		else if (xyz[2]<=penup_val) {
			//UP
			input_report_key (tsdata->input, BTN_TOUCH,    0);
			input_report_abs (tsdata->input, ABS_PRESSURE, 0);
		}
	} else {
		mutex_unlock (&tsdata->mutex);

		//UP
		input_report_key (tsdata->input, BTN_TOUCH,    0);
		input_report_abs (tsdata->input, ABS_PRESSURE, 0);
	}

	input_sync (tsdata->input);

out:
	return IRQ_HANDLED;
}

static ssize_t ftc10005_58_spi_setting_show (struct device *dev,
                                            struct device_attribute *attr,
                                            char *buf)
{
	struct ftc10005_58_spi_ts_data *tsdata = dev_get_drvdata (dev);
	int ret = 0;
	int val;

	mutex_lock (&tsdata->mutex);

	if(!strcmp(attr->attr.name,"pendown_val")) {
		val=tsdata->pendown_val;
	} else 	if(!strcmp(attr->attr.name,"penup_threshold")) {
		val=tsdata->penup_threshold;
	} else 	if(!strcmp(attr->attr.name,"sens1")) {
		val=tsdata->sens1;
	} else 	if(!strcmp(attr->attr.name,"sens2")) {
		val=tsdata->sens2;
	} else 	if(!strcmp(attr->attr.name,"charge")) {
		val=tsdata->charge;
	} else {
		dev_err (&tsdata->spi->dev, "unknown attribute for ftc10005_58_spi_setting_show: %s\n", attr->attr.name);
		ret=-ENODEV;
		goto out;
	}

	/* ok */
	ret=sprintf (buf, "%d\n", val);

out:
	mutex_unlock (&tsdata->mutex);
	return ret;
}

static ssize_t ftc10005_58_spi_setting_store (struct device *dev,
                                             struct device_attribute *attr,
                                             const char *buf, size_t count)
{
	struct ftc10005_58_spi_ts_data *tsdata = dev_get_drvdata (dev);
	int ret = 0;
	unsigned int val;

	if (sscanf(buf, "%u", &val) != 1) {
		dev_err (dev, "Invalid value for attribute %s\n", attr->attr.name);
		return -EINVAL;
	}

	mutex_lock (&tsdata->mutex);
	disable_irq (tsdata->irq);

	/* TODO: check value bounds */
	ret=0;
	if(!strcmp(attr->attr.name,"pendown_val")) {
		tsdata->pendown_val=val;
	} else 	if(!strcmp(attr->attr.name,"penup_threshold")) {
		tsdata->penup_threshold=val;
	} else 	if(!strcmp(attr->attr.name,"sens1")) {
		tsdata->sens1=val;
		ret = ftc10005_58_init(tsdata);
	} else 	if(!strcmp(attr->attr.name,"sens2")) {
		tsdata->sens2=val;
		ret = ftc10005_58_init(tsdata);
	} else 	if(!strcmp(attr->attr.name,"charge")) {
		tsdata->charge=val;
		ret = ftc10005_58_init(tsdata);
	} else {
		dev_err (&tsdata->spi->dev, "unknown attribute for ftc10005_58_spi_setting_show: %s\n", attr->attr.name);
		ret = -EINVAL;
		goto out;
	}
	if(ret) {
		dev_err (&tsdata->spi->dev, "failed to re-init touchscreen-controller\n");
		ret=-ENODEV;
		goto out;
	}

	/* ok */
	ret=count;

out:
	enable_irq (tsdata->irq);
	mutex_unlock (&tsdata->mutex);
	return ret;
}

static DEVICE_ATTR(pendown_val,		0664, ftc10005_58_spi_setting_show, ftc10005_58_spi_setting_store);
static DEVICE_ATTR(penup_threshold,	0664, ftc10005_58_spi_setting_show, ftc10005_58_spi_setting_store);
static DEVICE_ATTR(sens1,		0664, ftc10005_58_spi_setting_show, ftc10005_58_spi_setting_store);
static DEVICE_ATTR(sens2,		0664, ftc10005_58_spi_setting_show, ftc10005_58_spi_setting_store);
static DEVICE_ATTR(charge,		0664, ftc10005_58_spi_setting_show, ftc10005_58_spi_setting_store);

static struct attribute *ftc10005_58_spi_attrs[] = {
	&dev_attr_pendown_val.attr,
	&dev_attr_penup_threshold.attr,
	&dev_attr_sens1.attr,
	&dev_attr_sens2.attr,
	&dev_attr_charge.attr,
	NULL
};

static const struct attribute_group ftc10005_58_spi_attr_group = {
	.attrs = ftc10005_58_spi_attrs,
};

static int ftc10005_58_spi_ts_probe (struct spi_device *spi)
{
	struct ftc10005_58_spi_ts_data *tsdata;
	struct input_dev *input;
	int ret;
	printk(KERN_ERR "%s.%s.%i: xxxxxxxxxxxxxxxx\n",__FILE__,__FUNCTION__,__LINE__);

	dev_info (&spi->dev, "probing for Ocular FTC100005_58 SPI\n");

	if (!spi->irq) {
		dev_dbg (&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	tsdata = kzalloc (sizeof (*tsdata), GFP_KERNEL);
	if (!tsdata) {
		dev_err (&spi->dev, "failed to allocate driver data!\n");
		dev_set_drvdata (&spi->dev, NULL);
		return -ENOMEM;
	}

	dev_set_drvdata (&spi->dev, tsdata);
	tsdata->spi = spi;

	mutex_init (&tsdata->mutex);

	/* request IRQ pin */
	tsdata->irq = spi->irq;
	tsdata->irq_pin = irq_to_gpio (tsdata->irq);

	ret = gpio_request (tsdata->irq_pin, NULL);
	if (ret < 0) {
		dev_err (&spi->dev,
		         "Failed to request GPIO %d for IRQ %d, ret %d\n",
		         tsdata->irq_pin, tsdata->irq, ret);
		ret = -ENOMEM;
		goto err_free_tsdata;
	}
	gpio_direction_input (tsdata->irq_pin);

	mutex_lock (&tsdata->mutex);

	/* init settings */
	tsdata->sens1=FTC10005_58_SENS1;
	tsdata->sens2=FTC10005_58_SENS2;
	tsdata->charge=FTC10005_58_CHARGE;
	tsdata->pendown_val=FTC10005_58_PENDOWN_VAL;
	tsdata->penup_threshold=FTC10005_58_PENUP_THRESHOLD;
	tsdata->min_x=FTC10005_58_MIN_X;
	tsdata->min_y=FTC10005_58_MIN_Y;
	tsdata->min_z=FTC10005_58_MIN_Z;
	tsdata->max_x=FTC10005_58_MAX_X;
	tsdata->max_y=FTC10005_58_MAX_Y;
	tsdata->max_z=FTC10005_58_MAX_Z;

	/* init hardware */
	ret=ftc10005_58_init(tsdata);
	if(ret) {
		dev_err (&spi->dev,
		         "Failed to probe, ret %d\n",
		         ret);
		mutex_unlock (&tsdata->mutex);
		ret = -ENODEV;
		goto err_free_irq_pin;
	}

	mutex_unlock (&tsdata->mutex);

	/* init input */
	input = input_allocate_device ();
	if (!input) {
		dev_err (&spi->dev, "failed to allocate input device!\n");
		ret = -ENOMEM;
		goto err_free_irq_pin;
	}

	set_bit (EV_SYN, input->evbit);
	set_bit (EV_KEY, input->evbit);
	set_bit (EV_ABS, input->evbit);
	set_bit (BTN_TOUCH, input->keybit);
	input_set_abs_params (input, ABS_X, tsdata->min_x, tsdata->max_x, 0, 0);
	input_set_abs_params (input, ABS_Y, tsdata->min_y, tsdata->max_y, 0, 0);
	input_set_abs_params (input, ABS_PRESSURE, tsdata->min_z, tsdata->max_z, 0, 0);
	input->name = kstrdup ("ftc10005_58", GFP_NOIO);
//	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&bus->dev));
//	input->phys = ts->phys;
	input->dev.parent = &spi->dev;

	input_set_drvdata (input, tsdata);

	tsdata->input = input;

	if ((ret = input_register_device (input)))
		goto err_free_input_device;

	/* init irq */
	if (request_threaded_irq (tsdata->irq, NULL, ftc10005_58_ts_isr,
	                          IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
	                          input->name, tsdata)) {
		dev_err (&spi->dev, "Unable to request touchscreen IRQ.\n");
		input = NULL;
		ret = -ENOMEM;
		goto err_unregister_device;
	}

	/* init sysfs */
	ret = sysfs_create_group (&spi->dev.kobj, &ftc10005_58_spi_attr_group);
	if (ret)
		goto err_free_irq;

	/* start */
	device_init_wakeup (&spi->dev, 1);

	dev_info (&tsdata->spi->dev,
	          "Ocular FTC10005_58 initialized: IRQ pin %d.\n",
	          tsdata->irq_pin);
	printk(KERN_ERR "%s.%s.%i: xxxxxxxxxxxxxxxx\n",__FILE__,__FUNCTION__,__LINE__);
	return 0;

err_free_irq:
	free_irq (spi->irq, tsdata);
err_unregister_device:
	input_unregister_device (input);
err_free_input_device:
	kfree (input->name);
	input_free_device (input);
err_free_irq_pin:
	gpio_free (tsdata->irq_pin);
err_free_tsdata:
	kfree (tsdata);
	return ret;
}

static int __devexit ftc10005_58_spi_ts_remove (struct spi_device *spi)
{
	struct ftc10005_58_spi_ts_data *tsdata = dev_get_drvdata (&spi->dev);

	sysfs_remove_group (&spi->dev.kobj, &ftc10005_58_spi_attr_group);

	free_irq (spi->irq, tsdata);
	input_unregister_device (tsdata->input);
	kfree (tsdata->input->name);
	input_free_device (tsdata->input);
	gpio_free (tsdata->irq_pin);
	kfree (tsdata);

	dev_set_drvdata (&spi->dev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int ftc10005_58_spi_ts_suspend (struct spi_device *spi, pm_message_t mesg)
{
	struct ftc10005_58_spi_ts_data *tsdata = dev_get_drvdata (&spi->dev);

	if (device_may_wakeup (&spi->dev))
		enable_irq_wake (tsdata->irq);

	return 0;
}

static int ftc10005_58_spi_ts_resume (struct spi_device *spi)
{
	struct ftc10005_58_spi_ts_data *tsdata = dev_get_drvdata (&spi->dev);

	if (device_may_wakeup (&spi->dev))
		disable_irq_wake (tsdata->irq);

	return 0;
}
#else
#define ftc10005_58_spi_ts_suspend NULL
#define ftc10005_58_spi_ts_resume  NULL
#endif


static struct spi_driver ftc10005_58_spi_ts_driver = {
	.driver = {
		.owner 	= THIS_MODULE,
		.name 	= "ftc10005_58",
		.bus	= &spi_bus_type,
	},
	.probe    = ftc10005_58_spi_ts_probe,
	.remove   = __devexit_p(ftc10005_58_spi_ts_remove),
	.suspend  = ftc10005_58_spi_ts_suspend,
	.resume   = ftc10005_58_spi_ts_resume,
};

static int __init ftc10005_58_spi_ts_init (void)
{
	return spi_register_driver(&ftc10005_58_spi_ts_driver);
}
module_init (ftc10005_58_spi_ts_init);

static void __exit ftc10005_58_spi_ts_exit (void)
{
	spi_unregister_driver(&ftc10005_58_spi_ts_driver);
}
module_exit (ftc10005_58_spi_ts_exit);

MODULE_AUTHOR ("Manfred Schlaegl <manfred.schlaegl@ginzinger.com>");
MODULE_DESCRIPTION ("Ocular FTC10005-58 capacitive SPI Touchscreen Driver");
MODULE_LICENSE (GPL);

