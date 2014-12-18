/*
 * ar1020.c - Microchip AR1020 touchscreen driver
 *
 * Copyright (c) 2010 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:			Markus Pietrek
 *
 * References:
 *	 [1] Microchip DS41393A
 *
 **/

/***********************************************************************
 * @History:
 * 	2012KW32 - manfred.schlaegl:
 *		* cleanup sysfs-attribute handling: missing remove
 *		* added base-support for multiple sysfs-files
 *		* added sysfs-parameter to skip a number of samples (slowdown sampling-rate) 
 *		* make not working calibration-stuff configureable (CONFIG_TOUCHSCREEN_AR1020_CALIBRATE)
 *		* disabled calibration stuff (CONFIG_TOUCHSCREEN_AR1020_CALIBRATE)
 *		* cleanup style
 *	2011KW50 - melchior.franz:
 *		* remove warning; verification/review done:
 *		https://development.ginzinger.com/armlinux/wiki/MelchiorfLogs11Kw49-2
 *	2011KW50 - melchior.franz:
 *		* fixed stepping bug
 *		* fixed ABS_X/Y
 *	2011KW48 - manfred.schlaegl:
 *		* copied from emtrion-kernel
 ***********************************************************************/

#define DEBUG

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/wait.h>

//#define CONFIG_TOUCHSCREEN_AR1020_CALIBRATE
//#define CONFIG_TOUCHSCREEN_AR1020_AUTO_CALIBRATE

/* Timeout */
#define AR1020_TIMEOUT_DISABLE_TOUCH	70  /* ms */
#define AR1020_TIMEOUT_CMD		200 /* ms */

/* Command and response data [1] 7.14 */
#define AR1020_CMD			0x55
#define AR1020_TOUCH			0x80

/* Command response data[2] */
#define AR1020_CMD_SUCCESS		0x00

/* Commands data[3] */
#define AR1020_CMD_ENABLE_TOUCH		0x12
#define AR1020_CMD_DISABLE_TOUCH	0x13
#define AR1020_CMD_READ_REG		0x20
#define AR1020_CMD_WRITE_REG		0x21
#define AR1020_CMD_INVALID		0xFF

/* Register addresses of the undocumented calibration registers. 
 * These addresses are "physical", the chip address offset has to 
 * be added. 
 */
#define AR1020_REG_CALIB_CTL		0x8D
#define AR1020_REG_CALIB_VAL		0x99
#define AR1020_REG_TOUCH_MODE		0x0C

#define AR1020_BIT_CALIB_CTL_START	(1 << 2)


/* to be compatible with most other drivers who report a pressure >= 1 and
 * require "module pthres pmin=1" in their /etc/ts.conf. Technically we only
 * have 0 or 1	
 */
#define AR_PRESSURE_MAX			255

#define AR1020_DATA_SIZE		7
#define AR1020_BUF_SIZE			10

struct ar1020_cmd {
	u8 cmd;
	u8 data[AR1020_DATA_SIZE];
	u8 data_size;
};

struct ar1020_buf {
	u8 cmd_sent;
	u8 buf[AR1020_BUF_SIZE];
	struct i2c_msg msg;
};

struct ar1020_priv {
	struct i2c_client *client;
	struct input_dev	*input;
	struct mutex		 receive_mutex;
	struct workqueue_struct *workq;
	struct work_struct work;

	wait_queue_head_t waitq;

#ifdef CONFIG_TOUCHSCREEN_AR1020_CALIBRATE
	int reg_start_addr;

	struct delayed_work cali_work;
	struct mutex		 mutex_usr;
	struct mutex		 send_mutex;

	// True if we have a working calibration.
	unsigned int initialized;
	unsigned int last_calib_valid;
	unsigned int last_calib_value;
	
	struct ar1020_buf send;
#endif

	int sample_skip_cnt;
	int last_pen_down;
	int last_x;
	int last_y;

#define AR1020_STATE_NEUTRAL	0x00
#define AR1020_STATE_TOUCH	0x01
#define AR1020_STATE_CMD	0x02
#define AR1020_STATE_CMD_DONE	0x03
	int state;

	/* for receiving touch events from touch controller */
	struct ar1020_buf touch;
	struct ar1020_cmd receive;

	/* settings */
	unsigned int sample_skip_nr;
};

#ifdef CONFIG_TOUCHSCREEN_AR1020_CALIBRATE

static int ar1020_get_calibration(struct ar1020_priv *priv, int *status, int *val);
static int ar1020_calibrate(struct ar1020_priv *priv);
static int ar1020_send(struct ar1020_priv *priv, struct ar1020_cmd *req, struct ar1020_cmd *resp);

#ifdef CONFIG_TOUCHSCREEN_AR1020_AUTO_CALIBRATE
static void ar1020_initialize(struct work_struct *work)
{
	struct delayed_work *dcali_work = container_of(work, struct delayed_work, work);
	struct ar1020_priv *priv = container_of(dcali_work, struct ar1020_priv, cali_work);
	int status, val, res;

	mutex_lock(&priv->mutex_usr);

	if(
		((res = ar1020_get_calibration(priv, &status, &val)) >= 0) &&
		status
	) {
		if(!priv->last_calib_valid) {
			priv->last_calib_value = val;
			priv->last_calib_valid = 1;
			ar1020_calibrate(priv);
		} else {
			if(
				(priv->last_calib_value < 0x20) &&
				(val < 0x20) &&
				(abs(priv->last_calib_value - val) < 3)
			 ) {
				pr_debug("ar1020: Current calibration value: %d, Last value: %d\n",
					val, priv->last_calib_value);
				priv->initialized = 1;
			} else {
				pr_debug("ar1020: Current calibration value: %d, Last value: %d\n",
					val, priv->last_calib_value);
				priv->last_calib_valid = 0;
				priv->last_calib_value = val;
			}
		}
	}

	if(!priv->initialized)
		schedule_delayed_work(&priv->cali_work, msecs_to_jiffies(20));

	mutex_unlock(&priv->mutex_usr);
}
#endif /* CONFIG_TOUCHSCREEN_AR1020_AUTO_CALIBRATE */
#endif /* CONFIG_TOUCHSCREEN_AR1020_CALIBRATE */


static void ar1020_handle_touch(struct ar1020_priv *priv)
{
	int pen_down;

	/* [1] 7.11 */
	pen_down = priv->touch.buf[0] & 0x01;


	/*
	 * manfred.schlaegl@ginzinger.com
	 * sampling slowdown
	 */
	/* do action on change, else skip number of samples */
	if(pen_down!=priv->last_pen_down) {
		priv->last_pen_down=pen_down;
		priv->sample_skip_cnt=0;
	} else if(priv->sample_skip_cnt) {
		priv->sample_skip_cnt--;
		return;
	}
	priv->sample_skip_cnt=priv->sample_skip_nr;	/* set skip */
	
	/* original */
	priv->last_x = (priv->touch.buf[1] & 0x7F) | ((priv->touch.buf[2] & 0x1F) << 7);
	priv->last_y = (priv->touch.buf[3] & 0x7F) | ((priv->touch.buf[4] & 0x1F) << 7);

	input_report_key(priv->input, BTN_TOUCH, !!pen_down);
	input_report_abs(priv->input, ABS_X, priv->last_x);
	input_report_abs(priv->input, ABS_Y, priv->last_y);
	input_report_abs(priv->input, ABS_PRESSURE, pen_down ? AR_PRESSURE_MAX : 0);
	input_sync(priv->input);

#ifdef CONFIG_TOUCHSCREEN_AR1020_AUTO_CALIBRATE
	if(!priv->initialized && pen_down) {
		cancel_delayed_work(&priv->cali_work);
		schedule_delayed_work(&priv->cali_work, msecs_to_jiffies(250));
	}
#endif
		
}

static void ar1020_handle_cmd(struct ar1020_priv *priv)
{
	/* check data status and response */
	if (priv->touch.buf[2] != AR1020_CMD_SUCCESS || 
	priv->touch.buf[3] != priv->touch.cmd_sent) {
		pr_debug("Expected command response, received something else.\n");
		goto err;
	}

	/* check received data length */
	switch(priv->touch.cmd_sent) {
		case AR1020_CMD_ENABLE_TOUCH:
		case AR1020_CMD_DISABLE_TOUCH:
		case AR1020_CMD_WRITE_REG:
			if (priv->touch.buf[1] != 0x02) {
				pr_debug("Command failed.\n");
				goto err;
			}
		break;
		case AR1020_CMD_READ_REG:
			if (priv->touch.buf[1] != 0x03) {
				pr_debug("Command failed.\n");
				goto err;
			}
		break;
		default:
			dev_dbg(&priv->input->dev, "unknown command.\n");
			goto err;
	}

	/* fill response structure */
	memset(&priv->receive, 0 , sizeof(priv->receive));
	priv->receive.cmd = priv->touch.buf[3];
	priv->receive.data_size = priv->touch.buf[1] - 2;
	memcpy(priv->receive.data, &priv->touch.buf[4], priv->receive.data_size);

	goto handled;

err:
	priv->receive.cmd = AR1020_CMD_INVALID;
	dev_dbg(&priv->input->dev, "Invalid command response.\n");
handled:
	priv->touch.cmd_sent = 0;
	wake_up(&priv->waitq);
}

static void ar1020_receive(struct work_struct *work)
{
	struct ar1020_priv *priv = container_of(work, struct ar1020_priv, work);
	//pr_debug("ar1020: receive\n");
	int err=0;

	mutex_lock(&priv->receive_mutex);
	//disable_irq(priv->client->irq);

	priv->receive.cmd = AR1020_CMD_INVALID;
	// We always read 5 bytes, because residual bytes
	// crash the controller.
	priv->touch.msg.len = 5;
	// Read access.
	priv->touch.msg.flags = I2C_M_RD;

	if ((err = i2c_transfer(priv->client->adapter, &priv->touch.msg, 1)) != 1) {
		/* was a command being sent when the user did touch the display? */
		printk("ar1020: errno: %d\n", err);;
		dev_dbg(&priv->client->dev, "I/O error (receive)\n");
		goto out;
	}

	if (priv->state == AR1020_STATE_TOUCH && priv->touch.buf[0] & AR1020_TOUCH) {
		/* handle touch */
		ar1020_handle_touch(priv);
	} else if (priv->state != AR1020_STATE_TOUCH && priv->touch.buf[0] == AR1020_CMD) {
		/* handle cmd */
		ar1020_handle_cmd(priv);
	} else {
		dev_dbg(&priv->input->dev, "state: %d, buf[0]: %d\n", priv->state, priv->touch.buf[0]);
		dev_dbg(&priv->input->dev, "invalid response.\n");
	}

out:
	mutex_unlock(&priv->receive_mutex);
	enable_irq(priv->client->irq);
	return;
}


#ifdef CONFIG_TOUCHSCREEN_AR1020_CALIBRATE
static int ar1020_send(struct ar1020_priv *priv, 
		struct ar1020_cmd *req, struct ar1020_cmd *resp)
{
	int res = 0;

	mutex_lock(&priv->send_mutex);
	
	
	priv->state = AR1020_STATE_CMD;
	priv->send.cmd_sent = req->cmd;
	priv->touch.cmd_sent = req->cmd;

	/* send i2c data */
	priv->send.buf[0] = 0x00;		/* dummy */
	priv->send.buf[1] = AR1020_CMD;	/* command header */
	priv->send.buf[2] = req->data_size + 1; /* data + cmd size */
	priv->send.buf[3] = req->cmd;
	memcpy(&priv->send.buf[4], req->data, req->data_size);

	priv->send.msg.len	 = req->data_size + 4;
	priv->send.msg.flags = 0;	/* send i2c message */

	if (i2c_transfer(priv->client->adapter, &priv->send.msg, 1) != 1) {
		dev_dbg(&priv->input->dev, "sending failed.\n");
		res = -ENODEV;
		goto out;
	}

	/* wait for data or timeout */
	if (!wait_event_timeout(priv->waitq, priv->receive.cmd != AR1020_CMD_INVALID, msecs_to_jiffies(AR1020_TIMEOUT_CMD))) {
		dev_dbg(&priv->input->dev, "receiving failed.\n");
		res = -ETIMEDOUT;
		goto out;
	}

	mutex_lock(&priv->receive_mutex);

	if(
		(priv->state != AR1020_STATE_CMD) || 
		(priv->receive.cmd == AR1020_CMD_INVALID)
	) {
		dev_dbg(&priv->input->dev, "invalid command response.\n");
		res = -EINVAL;
		goto out;
	}

	/* copy response data and invalidate receive command buffer */
	memcpy(resp, &priv->receive, sizeof(*resp));
	priv->receive.cmd = AR1020_CMD_INVALID;

	mutex_unlock(&priv->receive_mutex);

out:
	priv->state = AR1020_STATE_TOUCH;
	priv->send.msg.len	 = sizeof(priv->send.buf);

	priv->touch.cmd_sent = 0;
	priv->touch.msg.len	 = sizeof(priv->touch.buf);

	mutex_unlock(&priv->send_mutex);

	return res;
}

static int ar1020_enable_touch(struct ar1020_priv *priv, int enable)
{
	struct ar1020_cmd req, resp;
	int res;

	if (enable)
		req.cmd = AR1020_CMD_ENABLE_TOUCH;
	else
		req.cmd = AR1020_CMD_DISABLE_TOUCH;

	req.data_size = 0;

	res = ar1020_send(priv, &req, &resp);

	if (!enable) {
		/* wait 50ms */
		msleep(AR1020_TIMEOUT_DISABLE_TOUCH);
	}

	return res;
}

static int ar1020_calibrate(struct ar1020_priv *priv)
{

	struct ar1020_cmd req, resp;
	int res;
	static char configured = 0;
	
	/* disable touch */
	res = ar1020_enable_touch(priv, 0);
	if (res < 0)
		goto err;

	/* send calibrate command */
	req.cmd = AR1020_CMD_READ_REG;
	req.data[0] = 0x00;		/* high byte */
	req.data[1] = priv->reg_start_addr + AR1020_REG_CALIB_CTL; /* low byte */
	req.data[2] = 0x01;		/* regs to read */
	req.data_size = 3;
	res = ar1020_send(priv, &req, &resp);
	if (res < 0)
		goto err;

	req.cmd = AR1020_CMD_WRITE_REG;
	req.data[0] = 0x00;		/* high byte */
	req.data[1] = priv->reg_start_addr + AR1020_REG_CALIB_CTL; /* low byte */
	req.data[2] = 0x01;		/* regs to write */
	req.data[3] = resp.data[0] & ~AR1020_BIT_CALIB_CTL_START;		/* clear start calibration bit */
	req.data_size = 4;
	res = ar1020_send(priv, &req, &resp);
	if (res < 0)
		goto err;

	if (!configured) {
		req.cmd = AR1020_CMD_WRITE_REG;
		req.data[0] = 0x00; /* register address high byte */
		req.data[1] = priv->reg_start_addr + AR1020_REG_TOUCH_MODE;
		req.data[2] = 0x01; /* write a single register */
		req.data[3] = 0x51; /* see 7.12.10 */
		req.data_size = 4;
		
		res = ar1020_send(priv, &req, &resp);
		if (res < 0)
			goto err;

		configured = 1;
	}

	/* enabled touch */
	res = ar1020_enable_touch(priv, 1);
	if (res < 0)
		goto err;

	return res;

err:
	ar1020_enable_touch(priv, 1);
	return res;
}

static int ar1020_get_calibration(struct ar1020_priv *priv, int *status, int *val)
{
	struct ar1020_cmd req, resp;
	int res;

	*val = -1;

	flush_workqueue(priv->workq);

	/* disable touch */
	res = ar1020_enable_touch(priv, 0);
	if (res < 0)
		goto err;

	/* get calibration status and value */
	req.cmd = AR1020_CMD_READ_REG;
	req.data[0] = 0x00;		/* high byte */
	req.data[1] = priv->reg_start_addr + AR1020_REG_CALIB_CTL; /* low byte */
	req.data[2] = 0x01;		/* regs to read */
	req.data_size = 3;
	res = ar1020_send(priv, &req, &resp);
	if (res < 0)
		goto err;

	*status = (resp.data[0] & AR1020_BIT_CALIB_CTL_START) >> 2;

	if (*status) {
		req.cmd = AR1020_CMD_READ_REG;
		req.data[0] = 0x00;		/* high byte */
		req.data[1] = priv->reg_start_addr + AR1020_REG_CALIB_VAL; /* low byte */
		req.data[2] = 0x01;		/* regs to read */
		req.data_size = 3;
		res = ar1020_send(priv, &req, &resp);
		if (res < 0)
			goto err;

		*val= resp.data[0];
	}

	/* enabled touch */
	res = ar1020_enable_touch(priv, 1);
	if (res < 0)
		goto err;

	return res;

err:
	ar1020_enable_touch(priv, 1);
	return res;
}

static ssize_t ar1020_attr_calibration_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = container_of(dev, struct input_dev, dev);
	struct ar1020_priv *priv = (struct ar1020_priv *)input_get_drvdata(input);
	int res, val = -1, status = 0;

	mutex_lock(&priv->mutex_usr);
	res = ar1020_get_calibration(priv, &status, &val);
	mutex_unlock(&priv->mutex_usr);
	if (res < 0)
		return snprintf(buf, PAGE_SIZE, "%s: %d\n", "failed", val);

	return snprintf(buf, PAGE_SIZE, "%s: %d\n", status ? "calibrated" : "calibrating", val);
}

static ssize_t ar1020_attr_calibration_set(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct input_dev *input = container_of(dev, struct input_dev, dev);
	struct ar1020_priv *priv = (struct ar1020_priv *)input_get_drvdata(input);

	mutex_lock(&priv->mutex_usr);
	ar1020_calibrate(priv);
	mutex_unlock(&priv->mutex_usr);

	return count;
}
#endif /* CONFIG_TOUCHSCREEN_AR1020_CALIBRATE */


static ssize_t ar1020_attr_sample_skip_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = container_of(dev, struct input_dev, dev);
	struct ar1020_priv *priv = (struct ar1020_priv *)input_get_drvdata(input);
	return snprintf(buf, PAGE_SIZE, "%u\n", priv->sample_skip_nr);
}

static ssize_t ar1020_attr_sample_skip_set(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	int tmp;
	struct input_dev *input = container_of(dev, struct input_dev, dev);
	struct ar1020_priv *priv = (struct ar1020_priv *)input_get_drvdata(input);
	sscanf(buf, "%u", &tmp);
	priv->sample_skip_nr=tmp;
	return count;
}


static struct device_attribute ar1020_attr_sysfs[] = {
#ifdef CONFIG_TOUCHSCREEN_AR1020_CALIBRATE
	__ATTR(calibration, 0644, ar1020_attr_calibration_show, ar1020_attr_calibration_set),
#endif /* CONFIG_TOUCHSCREEN_AR1020_CALIBRATE */
	__ATTR(sample_skip, 0644, ar1020_attr_sample_skip_show, ar1020_attr_sample_skip_set),
};

static irqreturn_t ar1020_isr(int irq, void *dev_id)
{
	struct ar1020_priv *priv = dev_id;

	disable_irq_nosync(priv->client->irq);

	queue_work(priv->workq, &priv->work);

	return IRQ_HANDLED;
}

static int ar1020_open(struct input_dev *dev)
{
	return 0;
}

static void ar1020_close(struct input_dev *dev)
{
}

static int ar1020_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ar1020_priv *priv;
	int i,rc;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		rc = -ENOMEM;
		goto error;
	}
	i2c_set_clientdata(client, priv);
	priv->client = client;

	priv->state = AR1020_STATE_TOUCH;

#ifdef CONFIG_TOUCHSCREEN_AR1020_CALIBRATE
	mutex_init(&priv->mutex_usr);
	mutex_init(&priv->send_mutex);
#endif /* CONFIG_TOUCHSCREEN_AR1020_CALIBRATE */
	mutex_init(&priv->receive_mutex);

	init_waitqueue_head(&priv->waitq);

	/* we are waiting with mutexes in the workqueue -> don't influence other drivers */
	priv->workq = create_singlethread_workqueue(priv->client->name);
	if (!priv->workq) {
		dev_err(&client->dev, "can't create workqueue thread\n");
		rc = -ENOMEM;
		goto error_workq;
	}

	INIT_WORK(&priv->work, ar1020_receive);

#ifdef CONFIG_TOUCHSCREEN_AR1020_AUTO_CALIBRATE
	INIT_DELAYED_WORK(&priv->cali_work, ar1020_initialize);
#endif /* CONFIG_TOUCHSCREEN_AR1020_CALIBRATE */


#ifdef CONFIG_TOUCHSCREEN_AR1020_CALIBRATE
	/* TODO only valid for ar1020 */
	priv->reg_start_addr = 0x20;
	priv->initialized = 0;
	priv->last_calib_value = -1;
	priv->last_calib_valid = 0;

	/* setup constant initialization of i2c transfers	*/
	priv->send.msg.len	 = sizeof(priv->send.buf);
	priv->send.msg.buf	 = priv->send.buf;
	priv->send.msg.addr	 = priv->client->addr;
	priv->send.msg.flags 	 = 0;
#endif /* CONFIG_TOUCHSCREEN_AR1020_CALIBRATE */

	priv->sample_skip_cnt=0;
	priv->sample_skip_nr=0;

	priv->receive.cmd = AR1020_CMD_INVALID;

	/* setup constant initialization of i2c transfers	*/
	priv->touch.msg.len	 = sizeof(priv->touch.buf);
	priv->touch.msg.buf	 = priv->touch.buf;
	priv->touch.msg.addr	= priv->client->addr;
	priv->touch.msg.flags = I2C_M_RD;

	priv->input = input_allocate_device();
	if (!priv->input) {
		dev_err(&client->dev, "failed to allocate input device\n");
		rc = -ENOMEM;
		goto error_input;
	}

	priv->input->name = priv->client->name;
	priv->input->id.bustype = BUS_I2C;
	priv->input->dev.parent = &priv->client->dev;

	priv->input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	priv->input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	priv->input->open = ar1020_open;
	priv->input->close = ar1020_close;

	input_set_abs_params(priv->input, ABS_X, 0, (1<<12) - 1, 0, 0);
	input_set_abs_params(priv->input, ABS_Y, 0, (1<<12) - 1, 0, 0);
	input_set_abs_params(priv->input, ABS_PRESSURE, 0, AR_PRESSURE_MAX, 0, 0);

	input_set_drvdata(priv->input, priv);

	rc = input_register_device(priv->input);
	if(rc)
		goto error_input_reg;


	rc = request_irq(priv->client->irq, ar1020_isr, IRQF_TRIGGER_HIGH,
			priv->client->name, priv);
	if(rc) {
		dev_err(&priv->client->dev, "unable to request touchscreen IRQ\n");
		goto error_irq;
	}

	for(i=0;i<ARRAY_SIZE(ar1020_attr_sysfs);i++) {
			rc = device_create_file(&priv->input->dev, &ar1020_attr_sysfs[i]);
		if (rc < 0) {
			dev_err(&priv->client->dev, "unable to create sysfs-attr\n");
			while(i--)
		 		device_remove_file(&priv->input->dev, &ar1020_attr_sysfs[i]);
			goto error_sysfs;
		}
	}


#ifdef CONFIG_TOUCHSCREEN_AR1020_AUTO_CALIBRATE
	schedule_delayed_work(&priv->cali_work, msecs_to_jiffies(0));
#endif
	return 0;
error_sysfs:
	free_irq(priv->client->irq, priv);

error_irq:
	input_unregister_device(priv->input);
	
error_input_reg:
	input_free_device(priv->input);
	
error_input:
	destroy_workqueue(priv->workq);
	
error_workq:
	i2c_set_clientdata(client, NULL);
	kfree(priv);
	
error:
	return rc;
}

static int ar1020_remove(struct i2c_client *client)
{
	int i;
	struct ar1020_priv *priv = i2c_get_clientdata(client);

	for(i=0;i<ARRAY_SIZE(ar1020_attr_sysfs);i++)
		device_remove_file(&priv->input->dev, &ar1020_attr_sysfs[i]);

	free_irq(priv->client->irq, priv);
	flush_workqueue(priv->workq);

	input_unregister_device(priv->input);
	input_free_device(priv->input);
	destroy_workqueue(priv->workq);
	i2c_set_clientdata(client, NULL);
	kfree(priv);

	return 0;
}

static const struct i2c_device_id ar1020_id[] = {
	{ "ar1020", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ar1020_id);

static struct i2c_driver ar1020_driver = {
	.driver = {
		.name = "ar1020",
	},
	.probe		= ar1020_probe,
	.remove		= ar1020_remove,
	.id_table	= ar1020_id,
};

static __init int ar1020_init(void)
{
	return i2c_add_driver(&ar1020_driver);
}
module_init(ar1020_init);

static __exit void ar1020_exit(void)
{
	i2c_del_driver(&ar1020_driver);
}
module_exit(ar1020_exit);

MODULE_DESCRIPTION("Microchip AR1020 touch driver");
MODULE_AUTHOR("Markus Pietrek");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ar1020");
