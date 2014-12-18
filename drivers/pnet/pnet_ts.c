/***********************************************************************
 *
 * @Authors: Manfred Schlägl, Ginzinger electronic systems GmbH (C) 2007
 * @Descr: Driver to use a remote Touchscreen-Device over PNET
 * @TODO:
 *
 * Based on:
 * 	a9m9pnet0_ts.c 
 *		Copyright (C) 2005 by Sistemas Embebidos S.A.
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
 ***********************************************************************/
/***********************************************************************
 *  @History:
 *	2011KW11 - manfred.schlaegl@gmx.at
 *		support for capacitive touchscreen (basic)
 *	2010-KW22 manfred.schlaegl@gmx.at
 *		* port to linux-2.6.32
 *
 *	28.09.2007:	
 *		* core-changed connection state handling (hotplug)
 *		* extended comments
 *  06.10.2007
 *      * enable/disable with module-parameter
 *	31.01.2008
 *		* timeout for waiting on connect in open
 *	03.02.2008
 *		* update of settings in tasklet
 *			* enabling is done in tasklet -> (autoenable = hotplug)
 *  09.02.2008
 *		* update of settings in event-workqueue
 *		* save & granular locking
 *  15.01.2009
 *      * sometimes the xserver closes the device with signal so we 
 *		  have to use wait_event instead of wait_event_interruptible
 *  29.09.2009
 *      * correction: undefining PNET_TASKLET_WORK causing some compile-errors with 2.6.20
 *		* correction: wrong locks used with PNET_TASKLET_WORK defined
 *      * correction: locking errors
 *		* changed default: work in workqueue instead of tasklet
 *
 ***********************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
//#include <linux/errno.h>
//#include <linux/slab.h>
//#include <linux/err.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/input.h>
#include "pnet_core.h"

/***********************************************************************
 *  Driver Data
 ***********************************************************************/

#define PNET_TS_DRIVER_NAME 		"pnet_ts"
#define PNET_TS_DRIVER_AUTHOR 		"Manfred Schlaegl jun. <manfred.schlaegl@gmx.at>"
#define PNET_TS_DRIVER_DESC 		"PNET Touchscreen driver"
#define PNET_TS_ID					0x10
#define PNET_TS_VERSION				18

/* if defined -> do work in tasklet 
 * if not defined, work is done in event-workqueue
 */
//#define PNET_TASKLET_WORK

/* uncomment to enable debugging at build-time */
//#define DEBUG

struct pnet_ts;

/***********************************************************************
 *  Debug-Messages
 ***********************************************************************/

/* flag for debug-messages */
#ifdef DEBUG
static int debug=1;
#else
static int debug=0;
#endif

#define _KERN_DEBUG_ KERN_INFO /* for debug */

#define MY_NAME	PNET_TS_DRIVER_NAME

/* kernel Output */
#define err(format, arg...) printk(KERN_ERR "%s: %s: " format "\n" , MY_NAME , __FUNCTION__ , ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: %s: " format "\n" , MY_NAME , __FUNCTION__ , ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: %s: " format "\n" , MY_NAME , __FUNCTION__ , ## arg)

/* debug messages */
#define dbg_raw(fmt, arg...) 						\
	do {								\
		if (debug)						\
			printk (_KERN_DEBUG_ fmt "\n", ## arg);		\
	} while (0)
/* debug messages with module- and function-information */
#define dbg(fmt, arg...) 						\
	dbg_raw("%s: %s:\t\t" fmt, MY_NAME , __FUNCTION__ , ## arg)	



/***********************************************************************
 *  Touchscreen-Settings Handling
 ***********************************************************************/

#define PNET_TS_SETTING_ENABLE					(1<<0)
#define PNET_TS_SETTING_AUTOCALIBRATE			(1<<1)
#define PNET_TS_SETTING_USE_PENDOWN_LINE		(1<<2)

#define PNET_TS_SETTING_FLAG_ENABLE				(1<<0)
#define PNET_TS_SETTING_FLAG_AUTOCALIBRATE		(1<<1)
#define PNET_TS_SETTING_FLAG_SAMPLE_DELAY		(1<<2)
#define PNET_TS_SETTING_FLAG_SAMPLE_RATE		(1<<3)
#define PNET_TS_SETTING_FLAG_USE_PENDOWN_LINE	(1<<4)
#define PNET_TS_SETTING_FLAG_PENDOWN_SAMPLES	(1<<5)
#define PNET_TS_SETTING_FLAG_PENDOWN_VAL		(1<<6)
#define PNET_TS_SETTING_FLAG_MAX_DELTA			(1<<7)
#define PNET_TS_SETTING_FLAG_MED_SAMPLES		(1<<8)
#define PNET_TS_SETTING_FLAG_MIN_X				(1<<9)
#define PNET_TS_SETTING_FLAG_MAX_X				(1<<10)
#define PNET_TS_SETTING_FLAG_MIN_Y				(1<<11)
#define PNET_TS_SETTING_FLAG_MAX_Y				(1<<12)
#define PNET_TS_SETTING_FLAG_TYPE				(1<<13)


/* 4wire -> deprecated */
#define PNET_TS_TYPE_4WIRE			0
/* 5wire -> deprecated */
#define PNET_TS_TYPE_5WIRE			1

/* internal adc (used on cmcx) */
#define PNET_TS_TYPE_INTADC			2
/* external adc */
#define PNET_TS_TYPE_EXTADC			3
/* external delta-sigma-adc */
#define PNET_TS_TYPE_EXTADC_DSIG	4
/* capacitive touch */
#define PNET_TS_TYPE_CAP			5

struct pnet_ts_settings {
	/* lock for parameters */
	spinlock_t				lock;
	/* wait queue for processes waiting for settings applied */
	wait_queue_head_t		wq;
	/* return val of handle settings (0 .. running; 1 .. ok; <0 error) */
	unsigned int 			ret;
	/* settings to set */
	unsigned short 			set;
	/* settings to get */
	unsigned short 			get;


	/* settings data */
	/* ts flags */
	unsigned char flags;	
	/* type of touchscreen */
	int type;
	/* delay between x&y samples */
	int sample_delay;
	/* sample-rate */
	int sample_rate;
	/* samples to determine pen-down */
	int pendown_samples;
	/* pen is down if x-value is smaller than this */
	int pendown_val;
	/* samples to average */
	int max_delta;
	/* averaged samples to med-sample */
	int med_samples;

	/* calibration data */
	int min_x;
	int min_y;
	int max_x;
	int max_y;
};

/* init remote-device (done in handle settings if not initialized) */
static int pnet_ts_init_remote_device(struct pnet_ts *pnet_ts);

/* handle pnet_ts setting update & remote init (state-machine) */
static int pnet_ts_handle_settings(struct pnet_ts *pnet_ts);

/* trigger settings update */
int pnet_ts_update_settings(struct pnet_ts *pnet_ts);

/***********************************************************************
 *  Protocol-Definitions
 ***********************************************************************/

/* timeout for rcv */
#define PNET_TS_COMM_TIMEOUT			(2*HZ)

/* protocol */
#define PNET_TS_CMD_FLAG_REQUEST		(1<<31)		/* request */
#define PNET_TS_CMD_FLAG_REPLY			(1<<30)		/* reply */

/* commands */
#define PNET_TS_CMD_PENDOWN					1
#define PNET_TS_CMD_PENUP					2

/* set commands */
#define PNET_TS_CMD_SET_ENABLE				11
#define PNET_TS_CMD_SET_AUTOCALIBRATION		12
#define PNET_TS_CMD_SET_X_MIN				13
#define PNET_TS_CMD_SET_X_MAX				14
#define PNET_TS_CMD_SET_Y_MIN				15
#define PNET_TS_CMD_SET_Y_MAX				16
#define PNET_TS_CMD_SET_USE_PENDOWN_LINE	17
#define PNET_TS_CMD_SET_PENDOWN_VAL			18
#define PNET_TS_CMD_SET_PENDOWN_SAMPLES		19
#define PNET_TS_CMD_SET_MAX_DELTA			20
#define PNET_TS_CMD_SET_MED_SAMPLES			21
#define PNET_TS_CMD_SET_TYPE				22
#define PNET_TS_CMD_SET_SAMPLE_DELAY		23
#define PNET_TS_CMD_SET_SAMPLE_RATE			24

/* get commands */
#define PNET_TS_CMD_GET_ENABLE				110		/* answer: ena, type 				*/
#define PNET_TS_CMD_GET_AUTOCALIBRATION		111		/* answer: ena 						*/
#define PNET_TS_CMD_GET_X_BOUNDS			112		/* answer: min_x, max_x 			*/
#define PNET_TS_CMD_GET_Y_BOUNDS			113		/* answer: min_y, max_y 			*/
#define PNET_TS_CMD_GET_USE_PENDOWN_LINE	114		/* answer: ena 						*/
#define PNET_TS_CMD_GET_PENDOWN_SAMP		115		/* answer: val, samples 			*/
#define PNET_TS_CMD_GET_SAMPLING_VALS		116		/* answer: max_delta, med_samples	*/
#define PNET_TS_CMD_GET_INFO				117		/* answer: id, version 				*/
#define PNET_TS_CMD_GET_SAMPLE_DELAY		118		/* answer: sample_delay				*/


/* packet */
struct pnet_ts_packet
{
	unsigned int	cmd;	/* command */	/* 4 bytes */
	unsigned int 	arg1;			/* 8 bytes */
	unsigned int 	arg2;			/* 12 bytes */
	/* padding */
	unsigned int 	pad;			/* 16 bytes */
}/* __attribute__((packed))*/;

/* send pnet data packet */
static int pnet_ts_send_cmd(struct pnet_ts *pnet_ts, unsigned int cmd, unsigned int arg1, unsigned int arg2);

/* send pnet request */
inline int pnet_ts_send_request(struct pnet_ts *pnet_ts, unsigned int cmd, unsigned int arg1, unsigned int arg2);

/* send pnet reply */
inline int pnet_ts_send_reply(struct pnet_ts *pnet_ts, unsigned int cmd, unsigned int arg1, unsigned int arg2);

/* do command state machine (send request, wait for reply(ret=0), get reply, check) */
static int pnet_ts_do_command(struct pnet_ts *pnet_ts, unsigned int cmd, unsigned int *arg1, unsigned int *arg2);

/* function that handles received packets */
static void pnet_ts_handle_packet(struct pnet_ts *pnet_ts);


/***********************************************************************
 *  Touch-Screen-Driver data + core
 ***********************************************************************/

#define PNET_TS_BIT_LOCKS_INITIALIZED	(0)
#define PNET_TS_BIT_INITIALIZED			(1)
#define PNET_TS_BIT_CONNECTED			(2)
#define PNET_TS_BIT_REMOTE_INITIALIZED	(3)
#define PNET_TS_BIT_WAITING_FOR_REPLY	(4)

/* stucture for a touchscreen over pnet */
struct pnet_ts {
	unsigned long			flags;				/* init flag & mutex init flag*/

	spinlock_t				lock;				/* rw-lock for cmd-send */
	struct semaphore		mutex;				/* mutex for structure */
	int 					opencount;			/* open counter */

	struct input_dev		*dev;				/* input device */

	struct pnet_core_port	port;				/* pnet port */

	struct pnet_ts_packet	rcv_packet;			/* last received packet */
	unsigned char			rcvflag;

#ifdef PNET_TASKLET_WORK
	struct tasklet_struct	work;				/* worker task */
#else
	struct work_struct		work;				/* worker task */
#endif

	struct pnet_ts_settings	settings;		/* driver settings */

	/* statistics */
	unsigned int rx_requests, tx_replies, tx_requests, rx_replies;
};

/* callback functions */
static void pnet_ts_port_received_data(struct pnet_core_port *port);
static void pnet_ts_port_sent_data(struct pnet_core_port *port);
static void pnet_ts_port_changed_state(struct pnet_core_port *port, int flags);


/* pnet port */
static struct pnet_ts pnet_ts_gbl = {
	.flags=0,
	.opencount=0,
	.port = {
		.owner = THIS_MODULE,
		.adr=0,
		.pri=2,
		.port=10,
		.rx_size=32,
		.tx_size=32,
		.tx_order_size=32,
		.sent_data=pnet_ts_port_sent_data,
		.received_data=pnet_ts_port_received_data,
		.changed_state=pnet_ts_port_changed_state,
		.private_data=(void*)&pnet_ts_gbl,
		.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT,
	},

	.rx_requests=0,
	.tx_replies=0,
	.tx_requests=0,
	.rx_replies=0,
};

/* init input-device (done on state-change to connected) */
static int pnet_ts_init_input(struct pnet_ts *pnet_ts);

/* deinit input (done on state change to disconnected or module-exit) */
static void pnet_ts_deinit_input(struct pnet_ts *pnet_ts);

/***********************************************************************
 *  Input-Layer functions
 ***********************************************************************/

/* function called on open */
static int pnet_ts_open(struct input_dev *dev);

/* function called on close */
static void pnet_ts_close(struct input_dev *dev);

/* report events to input-layer */
static void pnet_ts_report(struct pnet_ts *pnet_ts, int pendown, int x, int y);

/***********************************************************************
 *  Core-Port Callbacks
 ***********************************************************************/

/* data sent callback */
static void pnet_ts_port_sent_data(struct pnet_core_port *port)
{
//	struct pnet_ts *pnet_ts = (struct pnet_ts*) port->private_data;
	dbg("pnet_core_port sent callback");
}


/* data received callback */
static void pnet_ts_port_received_data(struct pnet_core_port *port)
{
	struct pnet_ts *pnet_ts = (struct pnet_ts*) port->private_data;
	dbg("pnet_core_port received callback");
#ifdef PNET_TASKLET_WORK
	tasklet_schedule(&pnet_ts->work);
#else
	if(schedule_work(&pnet_ts->work)==0)
		dbg("already scheduled");
#endif
}

/* state changed callback */
static void pnet_ts_port_changed_state(struct pnet_core_port *port, int state)
{
	struct pnet_ts *pnet_ts = (struct pnet_ts*) port->private_data;
	dbg("pnet_core_port changed-state callback (state=%d)",state);

	/* update state */
	switch (state) {
		case PNET_CORE_PORT_CONNECTED:
			set_bit(PNET_TS_BIT_CONNECTED,&pnet_ts->flags);
#ifdef PNET_TASKLET_WORK
			tasklet_schedule(&pnet_ts->work);
#else
			if(schedule_work(&pnet_ts->work)==0)
				dbg("already scheduled");
#endif
			break;
		case PNET_CORE_PORT_DISCONNECTED:
#ifdef PNET_TASKLET_WORK
			spin_lock_bh(&pnet_ts->settings.lock);
#else
			spin_lock(&pnet_ts->settings.lock);
#endif
			/* reset flags */
			pnet_ts->flags&=~(
				(1<<PNET_TS_BIT_REMOTE_INITIALIZED)|
				(1<<PNET_TS_BIT_CONNECTED)|
				(1<<PNET_TS_BIT_WAITING_FOR_REPLY)
			);
			/* reset stats */
			pnet_ts->tx_requests=0;
			pnet_ts->rx_replies=0;
			pnet_ts->rx_requests=0;
			pnet_ts->tx_replies=0;

#ifdef PNET_TASKLET_WORK
			spin_unlock_bh(&pnet_ts->settings.lock);
#else
			spin_unlock(&pnet_ts->settings.lock);
#endif
			break;
	}
}

/***********************************************************************
 *  Protocol-Functions
 ***********************************************************************/

/* send pnet data packet */
static int pnet_ts_send_cmd(struct pnet_ts *pnet_ts, unsigned int cmd, unsigned int arg1, unsigned int arg2)
{
	int ret;
	struct pnet_ts_packet packet;

	dbg("send cmd %u %i %i",cmd,arg1,arg2);

	/* convert */
	packet.cmd=htonl(cmd);
	packet.arg1=htonl(arg1);
	packet.arg2=htonl(arg2);

	/* only send if connected */
	if(!(pnet_ts->flags&(1<<PNET_TS_BIT_CONNECTED))) {
		return 0;
	}

	if(pnet_core_port_write_free(&pnet_ts->port)<sizeof(struct pnet_ts_packet)) {
		ret=0;
	} else {
		if(pnet_core_port_write(&pnet_ts->port,(unsigned char*)&packet,sizeof(struct pnet_ts_packet))!=sizeof(struct pnet_ts_packet)) {
			ret=0;
		} else {
			ret=1;
		}
	}

	return ret;
}

/* send pnet request */
inline int pnet_ts_send_request(struct pnet_ts *pnet_ts, unsigned int cmd, unsigned int arg1, unsigned int arg2)
{
	int ret;
	dbg("send request %u %i %i",cmd,arg1,arg2);
	ret=pnet_ts_send_cmd(pnet_ts, (cmd|PNET_TS_CMD_FLAG_REQUEST), arg1, arg2);
	if(ret)
		pnet_ts->tx_requests++;
	return ret;
}

/* send pnet reply */
inline int pnet_ts_send_reply(struct pnet_ts *pnet_ts, unsigned int cmd, unsigned int arg1, unsigned int arg2)
{
	int ret;
	dbg("send reply %u %i %i",cmd,arg1,arg2);
	ret=pnet_ts_send_cmd(pnet_ts, (cmd|PNET_TS_CMD_FLAG_REPLY), arg1, arg2);
	if(ret)
		pnet_ts->tx_replies++;
	return ret;
}

/* do a command (send request, receive reply, check) 
 * return: <0 .. Error; 0 .. sent-waiting for receive; 1 .. received
 */
static int pnet_ts_do_command(struct pnet_ts *pnet_ts, unsigned int cmd, unsigned int *arg1, unsigned int *arg2)
{
	int ret;

	dbg("do command %u %i %i (%s) rcv:%i",cmd,*arg1,*arg2,
		(pnet_ts->flags&(1<<PNET_TS_BIT_WAITING_FOR_REPLY)) ? "receive":"send",pnet_ts->rcvflag
	);

	/* send */
	if(!(pnet_ts->flags&(1<<PNET_TS_BIT_WAITING_FOR_REPLY))) {
		/* do command */
		if(!pnet_ts_send_request(pnet_ts,cmd,(*arg1),(*arg2))) {
			ret=-ENODEV;
			goto pnet_ts_do_command_ret;
		}

		pnet_ts->flags|=(1<<PNET_TS_BIT_WAITING_FOR_REPLY);

	} /* receive */ else if(pnet_ts->rcvflag) {
		/* answer is in structure */
		if(pnet_ts->rcv_packet.cmd!=cmd) {
			dbg("sent %u got %u\n",cmd, pnet_ts->rcv_packet.cmd);
			ret=-EPROTO;
			goto pnet_ts_do_command_ret;
		}

		(*arg1)=pnet_ts->rcv_packet.arg1;
		(*arg2)=pnet_ts->rcv_packet.arg2;

		pnet_ts->rcvflag=0;

		/* done -> new state */
		pnet_ts->flags&=~(1<<PNET_TS_BIT_WAITING_FOR_REPLY);
	}

	ret=((pnet_ts->flags&(1<<PNET_TS_BIT_WAITING_FOR_REPLY)) ? 0 : 1);

pnet_ts_do_command_ret:
	return ret;
}


/* function that handles received packets */
static void pnet_ts_handle_packet(struct pnet_ts *pnet_ts) 
{
	unsigned int arg1,arg2,cmd;

	/* convert */
	cmd=pnet_ts->rcv_packet.cmd;
	arg1=pnet_ts->rcv_packet.arg1;
	arg2=pnet_ts->rcv_packet.arg2;

	dbg("handle cmd=%u arg1=%i arg2=%i",cmd,arg1,arg2);

	/* request was received */
	if(cmd&PNET_TS_CMD_FLAG_REQUEST) {
		cmd&=~PNET_TS_CMD_FLAG_REQUEST;

		pnet_ts->rx_requests++;
		dbg("got request for %u arg1=%i arg2=%i",cmd,arg1,arg2);
		/* handle request */
		switch(cmd) {
			case PNET_TS_CMD_PENDOWN:
			case PNET_TS_CMD_PENUP:
				/* read is possible */
				dbg("pass pen: pen%s on x:%i y:%i",(cmd==PNET_TS_CMD_PENDOWN) ? "down" : "up", arg1,arg2);

				pnet_ts_report(pnet_ts,(cmd==PNET_TS_CMD_PENDOWN),arg1,arg2);
			break;
			default:
				err("Received invalid request: %u (arg1: %u; arg2: %u)",cmd,arg1,arg2);
				/* ignore invalid requests -> send reply */
		}
		/* send reply */
		if(!pnet_ts_send_reply(pnet_ts,cmd,arg1,arg2)) {
			/* never happens */
			err("Heavy Error");
		}
		pnet_ts->rcvflag=0;
	} else {
		/* reply received */
		if(cmd&PNET_TS_CMD_FLAG_REPLY) {
			pnet_ts->rcv_packet.cmd&=~PNET_TS_CMD_FLAG_REPLY;
			pnet_ts->rx_replies++;
			dbg("got reply for %u arg1=%i arg2=%i",pnet_ts->rcv_packet.cmd,arg1,arg2);
			/* handled automatically in work_proc */
		} else {
		 	err("Received invalid command: %u (arg1: %u; arg2: %u)",cmd,arg1,arg2);
			pnet_ts->rcvflag=0;
		}
	}
}

/***********************************************************************
 *  Touchscreen-Settings Handling
 ***********************************************************************/

/* init remote-device
 * return: <0 .. error, 0 .. initializing, 1 .. initialized
 */
static int pnet_ts_init_remote_device(struct pnet_ts *pnet_ts)
{
	unsigned int arg1,arg2;
	int ret;

	dbg("init");

	/* not connected -> not initialized */
	if(!(pnet_ts->flags & (1<<PNET_TS_BIT_CONNECTED))) {
		dbg("not connected");
		return -ENODEV;
	}

	if(pnet_ts->flags & (1<<PNET_TS_BIT_REMOTE_INITIALIZED)) {
		dbg("remote already initialized");
		return 1;
	}

	dbg("get info");
	arg1=0;
	arg2=0;

	ret=pnet_ts_do_command(pnet_ts,PNET_TS_CMD_GET_INFO,&arg1,&arg2);
	if(ret<0) {
		err("Failed to get remote-info");
		return -ENODEV;
	} else if(!ret) {
		return 0;
	}

	if(arg1!=PNET_TS_ID) {
		err("id differs (local: 0x%x != remote: 0x%x)",PNET_TS_ID,arg1);
		return -ENODEV;
	}

	if(arg2!=PNET_TS_VERSION) {
		err("version differs (local: %u != remote: %u)",PNET_TS_VERSION,arg2);
		return -ENODEV;
	}

	/* set only valid settings & settings already done */
	pnet_ts->settings.set|=~pnet_ts->settings.get;
	dbg("get: %X",pnet_ts->settings.get);
	dbg("set: %X",pnet_ts->settings.set);

	set_bit(PNET_TS_BIT_REMOTE_INITIALIZED,&pnet_ts->flags);

	return 1;
}

/* handle pnet_ts setting update & remote init 
 * return: <0 .. error; 0 .. running; 1 .. ok 
 */
static int pnet_ts_handle_settings(struct pnet_ts *pnet_ts)
{
	unsigned int cmd,arg1,arg2;
	int ret;

	dbg("settings_tl");

	/* init peer, not connected || failed -> stop */
	ret=pnet_ts_init_remote_device(pnet_ts);
	if(ret<0) {
		/* stop with error */
		goto pnet_ts_get_settings_ret;
	} else if(!ret) {
		goto pnet_ts_get_settings_ret;
	}

	/* nothing to do ? -> stop */
	if(!(pnet_ts->settings.set||pnet_ts->settings.get)) {
		dbg("nothing to do -> done\n");
		ret=1;
		goto pnet_ts_get_settings_ret;
	}

	/* set sample delay */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_SAMPLE_DELAY) {
		cmd=PNET_TS_CMD_SET_SAMPLE_DELAY;
		arg1=pnet_ts->settings.sample_delay;

		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		dbg("set sample_delay: %i",arg1);
		pnet_ts->settings.sample_delay=arg1;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_SAMPLE_DELAY;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_SAMPLE_DELAY;
	}

	/* set sample rate */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_SAMPLE_RATE) {
		cmd=PNET_TS_CMD_SET_SAMPLE_RATE;
		arg1=pnet_ts->settings.sample_rate;

		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		dbg("set sample_rate: %i",arg1);
		pnet_ts->settings.sample_rate=arg1;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_SAMPLE_RATE;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_SAMPLE_RATE;
	}

	/* get sample delay and/or sample rate */
	if(
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_SAMPLE_DELAY)||
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_SAMPLE_RATE)
	) {
		
		cmd=PNET_TS_CMD_GET_SAMPLE_DELAY;
		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_SAMPLE_DELAY) {
			pnet_ts->settings.sample_delay=arg1;
			dbg("get sample_rate: %i",arg1);

			pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_SAMPLE_DELAY;
		}

		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_SAMPLE_RATE) {
			pnet_ts->settings.sample_rate=arg2;
			dbg("get sample_delay: %i",arg2);

			pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_SAMPLE_RATE;
		}
	}

	/* set autocalibration */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_AUTOCALIBRATE) {
		/* set value */
		cmd=PNET_TS_CMD_SET_AUTOCALIBRATION;
		arg1=(pnet_ts->settings.flags & PNET_TS_SETTING_AUTOCALIBRATE) ? 1 : 0;

		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		dbg("set autocalibrate: %i",arg1);
		if(arg1)
			pnet_ts->settings.flags|=PNET_TS_SETTING_AUTOCALIBRATE;
		else
			pnet_ts->settings.flags&=~PNET_TS_SETTING_AUTOCALIBRATE;


		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_AUTOCALIBRATE;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_AUTOCALIBRATE;
	}

	/* get autocalibration */
	if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_AUTOCALIBRATE) {
		/* get value */
		cmd=PNET_TS_CMD_GET_AUTOCALIBRATION;

		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}
		dbg("get autocalibrate: %i",arg1);

		if(arg1)
			pnet_ts->settings.flags|=PNET_TS_SETTING_AUTOCALIBRATE;
		else
			pnet_ts->settings.flags&=~PNET_TS_SETTING_AUTOCALIBRATE;

		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_AUTOCALIBRATE;
	}


	/* set use-pendown-line */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_USE_PENDOWN_LINE) {
		cmd=PNET_TS_CMD_SET_USE_PENDOWN_LINE;
		arg1=(pnet_ts->settings.flags&PNET_TS_SETTING_USE_PENDOWN_LINE) ? 1 : 0;
		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		dbg("set use-pendown-line: %i",arg1);
		if(arg1)
			pnet_ts->settings.flags|=PNET_TS_SETTING_USE_PENDOWN_LINE;
		else
			pnet_ts->settings.flags&=~PNET_TS_SETTING_USE_PENDOWN_LINE;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_USE_PENDOWN_LINE;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_USE_PENDOWN_LINE;
	} 

	/* get use-pendown-line */
	if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_USE_PENDOWN_LINE) {
		cmd=PNET_TS_CMD_GET_USE_PENDOWN_LINE;
		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		dbg("get use-pendown-line: %i",arg1);
		if(arg1)
			pnet_ts->settings.flags|=PNET_TS_SETTING_USE_PENDOWN_LINE;
		else
			pnet_ts->settings.flags&=~PNET_TS_SETTING_USE_PENDOWN_LINE;

		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_USE_PENDOWN_LINE;
	}


	/* set min_x */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_MIN_X) {
		cmd=PNET_TS_CMD_SET_X_MIN;
		arg1=pnet_ts->settings.min_x;

		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}
		dbg("set min_x: %i",arg1);
		pnet_ts->settings.min_x=arg1;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_MIN_X;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MIN_X;
	}

	/* set max_x */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_MAX_X) {
		cmd=PNET_TS_CMD_SET_X_MAX;
		arg1=pnet_ts->settings.max_x;

		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}
		dbg("set max_x %i",arg1);
		pnet_ts->settings.max_x=arg1;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_MAX_X;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MAX_X;
	}

	/* get min_x and/or max_x */
	if(
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MIN_X) ||
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MAX_X)
	) {
		cmd=PNET_TS_CMD_GET_X_BOUNDS;
		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MIN_X) {
			dbg("get min_x %i",arg1);
			pnet_ts->settings.min_x=arg1;

			pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MIN_X;
		}

		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MAX_X) {
			dbg("get max_x: %i",arg2);
			pnet_ts->settings.max_x=arg2;

			pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MAX_X;
		}
	}


	/* set min_y */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_MIN_Y) {
		cmd=PNET_TS_CMD_SET_Y_MIN;
		arg1=pnet_ts->settings.min_y;

		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}
		dbg("set min_y: %i",arg1);
		pnet_ts->settings.min_y=arg1;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_MIN_Y;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MIN_Y;
	}

	/* set max_y */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_MAX_Y) {
		cmd=PNET_TS_CMD_SET_Y_MAX;
		arg1=pnet_ts->settings.max_y;

		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}
		dbg("set max_y: %i",arg1);
		pnet_ts->settings.max_y=arg1;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_MAX_Y;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MAX_Y;
	}

	/* get min_y and/or max_x */
	if(
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MIN_Y)||
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MAX_Y)
	) {
		cmd=PNET_TS_CMD_GET_Y_BOUNDS;
		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MIN_Y) {
			dbg("get min_y: %i",arg1);
			pnet_ts->settings.min_y=arg1;

			pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MIN_Y;
		}

		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MAX_Y) {
			dbg("get max_y: %i",arg2);
			pnet_ts->settings.max_y=arg2;

			pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MAX_Y;
		}
	}


	/* set pendown-samples */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_PENDOWN_SAMPLES) {
		cmd=PNET_TS_CMD_SET_PENDOWN_SAMPLES;
		arg1=pnet_ts->settings.pendown_samples;

		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}
		dbg("set pendown-samples: %i",arg1);
		pnet_ts->settings.pendown_samples=arg1;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_PENDOWN_SAMPLES;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_PENDOWN_SAMPLES;
	}

	/* get pendown-val */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_PENDOWN_VAL) {
		cmd=PNET_TS_CMD_SET_PENDOWN_VAL;
		arg1=pnet_ts->settings.pendown_val;

		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		dbg("set pendown-val: %i",arg1);
		pnet_ts->settings.pendown_val=arg1;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_PENDOWN_VAL;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_PENDOWN_VAL;
	}

	/* set pendown-samples and/or pendown-val */
	if(
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_PENDOWN_SAMPLES)||
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_PENDOWN_VAL)
	) {
		cmd=PNET_TS_CMD_GET_PENDOWN_SAMP;
		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_PENDOWN_VAL) {
			pnet_ts->settings.pendown_val=arg1;
			dbg("get pendown-val: %i",arg1);

			pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_PENDOWN_VAL;
		}


		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_PENDOWN_SAMPLES) {
			pnet_ts->settings.pendown_samples=arg2;
			dbg("get pendown-samples: %i",arg2);

			pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_PENDOWN_SAMPLES;
		}
	}

	/* set max-delta */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_MAX_DELTA) {
		cmd=PNET_TS_CMD_SET_MAX_DELTA;
		arg1=pnet_ts->settings.max_delta;

		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}
		dbg("set max_delta: %i",arg1);
		pnet_ts->settings.max_delta=arg1;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_MAX_DELTA;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MAX_DELTA;
	}

	/* set med-samples */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_MED_SAMPLES) {
		cmd=PNET_TS_CMD_SET_MED_SAMPLES;
		arg1=pnet_ts->settings.med_samples;
		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}
		dbg("set med_samples: %i",arg1);
		pnet_ts->settings.med_samples=arg1;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_MED_SAMPLES;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MED_SAMPLES;
	}

	/* get max-delta and/or med_samples */
	if(
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MAX_DELTA)||
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MED_SAMPLES)
	) {
		cmd=PNET_TS_CMD_GET_SAMPLING_VALS;
		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MAX_DELTA) {
			dbg("get max_delta: %i",arg1);
			pnet_ts->settings.max_delta=arg1;

			pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MAX_DELTA;
		}


		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_MED_SAMPLES) {
			dbg("get med_samples: %i",arg2);
			pnet_ts->settings.med_samples=arg2;

			pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_MED_SAMPLES;
		}
	}

	/* set type */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_TYPE) {
		/* set value */
		cmd=PNET_TS_CMD_SET_TYPE;
		arg1=pnet_ts->settings.type;
	
		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}
		dbg("set type: %i",arg1);
		pnet_ts->settings.type=arg1;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_TYPE;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_TYPE;
	} 
	/* set enabled value */
	if(pnet_ts->settings.set&PNET_TS_SETTING_FLAG_ENABLE) {
		cmd=PNET_TS_CMD_SET_ENABLE;
		arg1=(pnet_ts->settings.flags&PNET_TS_SETTING_ENABLE) ? 1 : 0;
		ret=pnet_ts_do_command(pnet_ts,PNET_TS_CMD_SET_ENABLE,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}
		dbg("set ena: %i",arg1);
		if(arg1)
			pnet_ts->settings.flags|=PNET_TS_SETTING_ENABLE;
		else
			pnet_ts->settings.flags&=~PNET_TS_SETTING_ENABLE;

		pnet_ts->settings.set&=~PNET_TS_SETTING_FLAG_ENABLE;
		pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_ENABLE;
	}

	/* get enabled and/or type */
	if(
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_ENABLE)||
		(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_TYPE)
	) {
		/* get enable-value & type of touch */
		cmd=PNET_TS_CMD_GET_ENABLE;
		ret=pnet_ts_do_command(pnet_ts,cmd,&arg1,&arg2);
		if(ret<0) {
			/* stop with error */
			goto pnet_ts_get_settings_err;
		} else if(!ret) {
			goto pnet_ts_get_settings_ret;
		}

		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_ENABLE) {
			dbg("get ena: %i",arg1);
			if(arg1)
				pnet_ts->settings.flags|=PNET_TS_SETTING_ENABLE;
			else
				pnet_ts->settings.flags&=~PNET_TS_SETTING_ENABLE;

			pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_ENABLE;
		}

		if(pnet_ts->settings.get&PNET_TS_SETTING_FLAG_TYPE) {
			dbg("get type: %i",arg2);
			pnet_ts->settings.type=arg2;
			/* update always */
			//pnet_ts->settings.get&=~PNET_TS_SETTING_FLAG_TYPE;
		}
	}

	/* all done */
	pnet_ts->settings.get=0;
	pnet_ts->settings.set=0;

	ret=1; /* ok */
	dbg("done");
	goto pnet_ts_get_settings_ret;


	/* error */
pnet_ts_get_settings_err:
	err("Failed Command %i",cmd);
	/* set device as not initialized */
	clear_bit(PNET_TS_BIT_REMOTE_INITIALIZED,&pnet_ts->flags);

pnet_ts_get_settings_ret:
	return ret;
}



/***********************************************************************
 *  Worker-Task
 ***********************************************************************/

/* task pass data to input-layer */
#ifdef PNET_TASKLET_WORK
static void pnet_ts_work_proc(unsigned long data) 
#else
static void pnet_ts_work_proc(struct work_struct *work)
#endif
{
	struct pnet_ts *pnet_ts = &pnet_ts_gbl;
	unsigned int ret;
	struct pnet_ts_packet packet;

	dbg("");

	/* if data */
	if(test_bit(PNET_TS_BIT_CONNECTED,&pnet_ts->flags)&&pnet_core_port_read_len(&pnet_ts->port)>=sizeof(struct pnet_ts_packet)) {
		/* read packet */
		pnet_core_port_read(&pnet_ts->port,(void*)&packet,sizeof(struct pnet_ts_packet));

		/* convert */
		pnet_ts->rcv_packet.cmd=ntohl(packet.cmd);
		pnet_ts->rcv_packet.arg1=ntohl(packet.arg1);
		pnet_ts->rcv_packet.arg2=ntohl(packet.arg2);
		dbg("got cmd %u %i %i",pnet_ts->rcv_packet.cmd,pnet_ts->rcv_packet.arg1,pnet_ts->rcv_packet.arg2);

#ifdef PNET_TASKLET_WORK
		spin_lock_bh(&pnet_ts->settings.lock);
#else
		spin_lock(&pnet_ts->settings.lock);
#endif

		/* pass packet */
		pnet_ts->rcvflag=1;
		pnet_ts_handle_packet(pnet_ts);

#ifdef PNET_TASKLET_WORK
		spin_unlock_bh(&pnet_ts->settings.lock);
#else
		spin_unlock(&pnet_ts->settings.lock);
#endif

		/* reschedule work-object until no more data */
#ifdef PNET_TASKLET_WORK
		tasklet_schedule(&pnet_ts->work);
#else
		if(schedule_work(&pnet_ts->work)==0)
			dbg("already scheduled");
#endif
	}

#ifdef PNET_TASKLET_WORK
	spin_lock_bh(&pnet_ts->settings.lock);
#else
	spin_lock(&pnet_ts->settings.lock);
#endif

	/* handle settings, when externally triggered */
	ret=pnet_ts_handle_settings(pnet_ts);

#ifdef PNET_TASKLET_WORK
	spin_unlock_bh(&pnet_ts->settings.lock);
#else
	spin_unlock(&pnet_ts->settings.lock);
#endif
	/* wakeup waiting if done */
	dbg("ret=%i",ret);
	if(ret) {
		dbg("pnet_ts->settings.ret=%i",ret);
		/* save ret */
		pnet_ts->settings.ret=ret;
		/* wakeup */
		dbg("wakeup");
		wake_up(&pnet_ts->settings.wq);
	}

}


/***********************************************************************
 *  Update remote Touchscreen-Settings
 ***********************************************************************/

/* get remote pnet_ts settings 
 * return: 1..ok; <0 error
 */
int pnet_ts_update_settings(struct pnet_ts *pnet_ts)
{
	int ret;

	dbg();

	/* reset ret */
	pnet_ts->settings.ret=0;

	/* trigger work */
#ifdef PNET_TASKLET_WORK
	tasklet_schedule(&pnet_ts->work);
#else
	if(schedule_work(&pnet_ts->work)==0)
		dbg("already scheduled");
#endif

	/* save wait until done */
	do {
		ret=wait_event_timeout(pnet_ts->settings.wq,pnet_ts->settings.ret,PNET_TS_COMM_TIMEOUT);
		if(ret==0) {
			err("timeout");
			return -EBUSY;
		}
		dbg("pnet_ts->settings.ret=%i",pnet_ts->settings.ret);
	} while(pnet_ts->settings.ret==0);
//	ret=pnet_ts->settings.ret;
	ret=0;

	return ret;
}


/***********************************************************************
 *  Input-Layer Communications
 ***********************************************************************/


/* function called on open */
static int pnet_ts_open(struct input_dev *dev) 
{
	struct pnet_ts *pnet_ts=(struct pnet_ts*) dev->dev.platform_data;
	int ret;

	dbg("called");
	down(&pnet_ts->mutex);

#ifdef PNET_TASKLET_WORK
	spin_lock_bh(&pnet_ts->settings.lock);
#else
	spin_lock(&pnet_ts->settings.lock);
#endif

	pnet_ts->settings.flags|=PNET_TS_SETTING_ENABLE;
	pnet_ts->settings.set|=PNET_TS_SETTING_FLAG_ENABLE;

#ifdef PNET_TASKLET_WORK
	spin_unlock_bh(&pnet_ts->settings.lock);
#else
	spin_unlock(&pnet_ts->settings.lock);
#endif

	ret=pnet_ts_update_settings(pnet_ts);
	if(ret<0) {
		up(&pnet_ts->mutex);
		return ret;
	}

	/* inc open-count */
	pnet_ts->opencount++;

	dbg("done");
	up(&pnet_ts->mutex);
	return 0;
}

/* function called on close */
static void pnet_ts_close(struct input_dev *dev) 
{
	struct pnet_ts *pnet_ts=(struct pnet_ts*) dev->dev.platform_data;
	int ret;

	dbg("called");
	down(&pnet_ts->mutex);

	/* dec open-count */
	if(pnet_ts->opencount)
		pnet_ts->opencount--;

	/* last close */
	if(!pnet_ts->opencount) {
		/* disable */
#ifdef PNET_TASKLET_WORK
		spin_lock_bh(&pnet_ts->settings.lock);
#else
		spin_lock(&pnet_ts->settings.lock);
#endif

		pnet_ts->settings.flags&=~PNET_TS_SETTING_ENABLE;
		pnet_ts->settings.set|=PNET_TS_SETTING_FLAG_ENABLE;
#ifdef PNET_TASKLET_WORK
		spin_unlock_bh(&pnet_ts->settings.lock);
#else
		spin_unlock(&pnet_ts->settings.lock);
#endif

		ret=pnet_ts_update_settings(pnet_ts);
		if(ret<0) {
			err("error update settings %i",ret);
		}
	}
	dbg("done");
	up(&pnet_ts->mutex);
}

/* report events to input-layer */
static void pnet_ts_report(struct pnet_ts *pnet_ts, int pendown, int x, int y)
{
	/* only if opened */
	if(pnet_ts->opencount) {
		/* coordinates */
		input_report_abs( pnet_ts->dev, ABS_X, x );
		input_report_abs( pnet_ts->dev, ABS_Y, y );

		/* pen status */
		input_report_key( pnet_ts->dev, BTN_TOUCH, pendown );
		input_report_abs( pnet_ts->dev, ABS_PRESSURE, pendown );

		/* sync */
		input_sync( pnet_ts->dev );
	}
}



#ifdef CONFIG_PROC_FS

/***********************************************************************
 *  procfs statistics
 ***********************************************************************/

static int pnet_ts_info_show(struct seq_file *m, void *v)
{
	down(&pnet_ts_gbl.mutex);

#ifdef PNET_TASKLET_WORK
	spin_lock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_lock(&pnet_ts_gbl.settings.lock);
#endif

	seq_printf(m,"Driver Id:                %x\n",PNET_TS_ID);
	seq_printf(m,"Driver Version:           %i\n",PNET_TS_VERSION);

	if(pnet_ts_gbl.flags&(1<<PNET_TS_BIT_CONNECTED)) {
		seq_printf(m,"PNET:                     %s\n","Connected");
		seq_printf(m,"Remote:                   %s\n",
			(pnet_ts_gbl.flags&(1<<PNET_TS_BIT_INITIALIZED)) ? "Initialized" : "Not Initialized (Error)"
		);
	} else {
		seq_printf(m,"PNET:                     %s\n","Not connected");
		seq_printf(m,"Remote:                   %s\n","PNET not connected");
	}

	seq_printf(m,"Flags:\n");
	seq_printf(m," + Type:                  ");
	switch(pnet_ts_gbl.settings.type) {
		/* 4wire -> deprecated */
		case PNET_TS_TYPE_4WIRE:
			seq_printf(m,"resistive 4wire - deprecated");
		break;
		/* 5wire -> deprecated */
		case PNET_TS_TYPE_5WIRE:
			seq_printf(m,"resistive 5wire - deprecated");
		break;
		/* resistive 4wire with internal adc */
		case PNET_TS_TYPE_INTADC:
			seq_printf(m,"resistive 4wire - int adc");
		break;
		/* resistive 4wire with external adc */
		case PNET_TS_TYPE_EXTADC:
			seq_printf(m,"resistive 4wire - ext adc");
		break;
		/* resistive 4wire with external delta-sigma-adc */
		case PNET_TS_TYPE_EXTADC_DSIG:
			seq_printf(m,"resistive 4wire - ext dsig-adc");
		break;
		/* capacitive touch with external controller */
		case PNET_TS_TYPE_CAP:
			seq_printf(m,"capacitive - ext controller");
		break;
		default:
			seq_printf(m,"unknown");
		break;
	}
	seq_printf(m,"\n");

	seq_printf(m," + Sampling:              %s\n",(pnet_ts_gbl.settings.flags&PNET_TS_SETTING_ENABLE ? "enabled" : "disabled"));
	seq_printf(m," + Autocalibration:       %s (deprecated)\n",(pnet_ts_gbl.settings.flags&PNET_TS_SETTING_AUTOCALIBRATE ? "enabled" : "disabled"));

	seq_printf(m," + Pen-Down-Line:         ");
	if(pnet_ts_gbl.settings.flags&PNET_TS_SETTING_USE_PENDOWN_LINE) {
		seq_printf(m,"enabled\n");
		seq_printf(m,"   + Pen down value:      %i (not used)\n",pnet_ts_gbl.settings.pendown_val);
		seq_printf(m,"   + Pen down samples:    %i (not used)\n",pnet_ts_gbl.settings.pendown_samples);
	} else {
		seq_printf(m,"disabled\n");
		seq_printf(m,"   + Pen down value:      %i\n",pnet_ts_gbl.settings.pendown_val);
		seq_printf(m,"   + Pen down samples:    %i\n",pnet_ts_gbl.settings.pendown_samples);
	}

	seq_printf(m,"Bounds (deprecated):\n");
	seq_printf(m," + min_x:                 %i\n",pnet_ts_gbl.settings.min_x);
	seq_printf(m," + max_x:                 %i\n",pnet_ts_gbl.settings.max_x);
	seq_printf(m," + min_y:                 %i\n",pnet_ts_gbl.settings.min_y);
	seq_printf(m," + max_y:                 %i\n",pnet_ts_gbl.settings.max_y);

	seq_printf(m,"Sampling:\n");
	seq_printf(m," + Sample-rate:           %i Hz\n",(pnet_ts_gbl.settings.sample_rate));
	seq_printf(m," + Sample-delay:          %i us\n",(pnet_ts_gbl.settings.sample_delay));
	seq_printf(m," + max_delta:             %i\n",pnet_ts_gbl.settings.max_delta);
	seq_printf(m," + med_samples:           %i\n",pnet_ts_gbl.settings.med_samples);

#ifdef PNET_TASKLET_WORK
	spin_unlock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_unlock(&pnet_ts_gbl.settings.lock);
#endif

	seq_printf(m,"Statistic:\n");

#ifdef PNET_TASKLET_WORK
	spin_lock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_lock(&pnet_ts_gbl.settings.lock);
#endif
	seq_printf(m," + sent requests:         %i\n",pnet_ts_gbl.tx_requests);
	seq_printf(m," + received replies:      %i\n",pnet_ts_gbl.rx_replies);
	seq_printf(m," + received requests:     %i\n",pnet_ts_gbl.rx_requests);
	seq_printf(m," + sent replies:          %i\n",pnet_ts_gbl.tx_replies);
#ifdef PNET_TASKLET_WORK
	spin_unlock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_unlock(&pnet_ts_gbl.settings.lock);
#endif	
	seq_printf(m," + opencount:             %i\n",pnet_ts_gbl.opencount);
	
	up(&pnet_ts_gbl.mutex);
	return 0;
}

static int pnet_ts_info_open( struct inode *inode, struct file *file)
{
	return single_open(file, pnet_ts_info_show, NULL);
}

static struct file_operations pnet_ts_info_fops = {
	.owner = THIS_MODULE,
	.open = pnet_ts_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static struct proc_dir_entry *pnet_ts_proc_dir,*pnet_ts_proc_info;
static int pnet_ts_proc_create(void)
{
	pnet_ts_proc_dir=proc_mkdir("driver/"PNET_TS_DRIVER_NAME, NULL);
	pnet_ts_proc_info=create_proc_entry("info",S_IRUGO,pnet_ts_proc_dir);
	if (pnet_ts_proc_info) {
		pnet_ts_proc_info->proc_fops = &pnet_ts_info_fops;
	}
	return 0;
}
static void pnet_ts_proc_remove(void)
{
	if (pnet_ts_proc_info) remove_proc_entry("info", pnet_ts_proc_dir);
	if (pnet_ts_proc_dir) remove_proc_entry("driver/"PNET_TS_DRIVER_NAME, NULL);
}
#endif	


/***********************************************************************
 *  init/deinit functions
 ***********************************************************************/

/* alloc input device */
struct input_dev *pnet_alloc_input_device(void)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)
	return (struct input_dev *)input_allocate_device();
#else
/* KERNEL_VERSION(2,6,12) */
	struct input_dev *dev;
	dev=(struct input_dev *)kmalloc(sizeof(struct input_dev),GFP_KERNEL);
	if(dev)
		memset(dev,0,sizeof(struct input_dev));
	return dev;
#endif

	err("wrong kernel-version\n");
	/* other versions unkown */
	return (struct input_dev *)0;
}

/* free input device */
void pnet_free_input_device(struct input_dev *dev)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)
	input_free_device(dev);
#else
/* KERNEL_VERSION(2,6,12) */
	kfree(dev);
#endif

	/* other versions unkown */
}



/* init input-device (done on state-change to connected)
 * WARNING: never call while mutex down. evbug calls open while input_register_input (deadlock)
 */
static int pnet_ts_init_input(struct pnet_ts *pnet_ts)
{
	int ret;

	/* alloc input-dev struct */
	pnet_ts->dev=pnet_alloc_input_device();
	if(!pnet_ts->dev) {
		err("Unable to allocate device-struct");
		return -ENOMEM;
	}
	
	/* init input-driver */
#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,12) && \
	LINUX_VERSION_CODE == KERNEL_VERSION(2,6,15)
	init_input_dev(pnet_ts->dev);
#endif

	pnet_ts->dev->dev.platform_data = pnet_ts;
	pnet_ts->dev->open = pnet_ts_open;
	pnet_ts->dev->close = pnet_ts_close;
	pnet_ts->dev->name = PNET_TS_DRIVER_NAME;
/*
	pnet_ts.dev->id.bustype = 0;
	pnet_ts.dev->id.vendor = 0;
	pnet_ts.dev->id.product = 0;
	pnet_ts.dev->id.version = 0;
	pnet_ts.dev->cdev.dev = NULL;
*/
	/* enable events */
	__set_bit(EV_ABS, pnet_ts->dev->evbit);
	__set_bit(EV_SYN, pnet_ts->dev->evbit);
	__set_bit(EV_KEY, pnet_ts->dev->evbit);
	__set_bit(ABS_X, pnet_ts->dev->absbit);
	__set_bit(ABS_Y, pnet_ts->dev->absbit);
	__set_bit(ABS_PRESSURE, pnet_ts->dev->absbit);
	__set_bit(BTN_TOUCH, pnet_ts->dev->keybit);

//	pnet_ts->dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
//	pnet_ts->dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	/* set attributes */
	input_set_abs_params( pnet_ts->dev, ABS_X, 0, 9999, 0, 0 );
	input_set_abs_params( pnet_ts->dev, ABS_Y, 0, 9999, 0, 0 );
	input_set_abs_params( pnet_ts->dev, ABS_PRESSURE, 0, 1, 0, 0 );

	/* register input_driver */
	ret=input_register_device(pnet_ts->dev);
	if(ret) {
		err("Unable to register input-device");
		pnet_free_input_device(pnet_ts->dev);
		return ret;
	}
	
	return 0;
}

/* deinit input (done on state change to disconnected or module-exit) 
 * WARNING: never call while mutex down. evbug calls close while input_register_input (deadlock)
 */
static void pnet_ts_deinit_input(struct pnet_ts *pnet_ts)
{
	/* unregister driver */
	input_unregister_device(pnet_ts->dev);
	pnet_free_input_device(pnet_ts->dev);
}

/* init mutex if not initialized */
static void pnet_ts_init_MUTEX(struct pnet_ts *pnet_ts)
{
	if(!(test_bit(PNET_TS_BIT_LOCKS_INITIALIZED,&pnet_ts->flags))) {
		/* init spinlock */
		dbg("spinlock-init");
		spin_lock_init(&pnet_ts->lock);
		spin_lock_init(&pnet_ts->settings.lock);
		/* init mutex */
		init_MUTEX(&pnet_ts->mutex);

		set_bit(PNET_TS_BIT_LOCKS_INITIALIZED,&pnet_ts->flags);
	}
}

/* deinit mutex if initialized */
static void pnet_ts_deinit_MUTEX(struct pnet_ts *pnet_ts)
{
#ifdef PNET_TASKLET_WORK
	spin_lock_bh(&pnet_ts->settings.lock);
#else
	spin_lock(&pnet_ts->settings.lock);
#endif
	if(pnet_ts->flags&(1<<PNET_TS_BIT_LOCKS_INITIALIZED)) {
		pnet_ts->flags&=~(1<<PNET_TS_BIT_LOCKS_INITIALIZED);
	}
#ifdef PNET_TASKLET_WORK
	spin_unlock_bh(&pnet_ts->settings.lock);
#else
	spin_unlock(&pnet_ts->settings.lock);
#endif
}


/* init pnet_ts - driver */	
static int __init pnet_ts_init( void )
{
	int ret;

	dbg("Registering driver");

	/* init lock if necessary */
	pnet_ts_init_MUTEX(&pnet_ts_gbl);

	/* register input device (do not call while mutex-down) */
	ret=pnet_ts_init_input(&pnet_ts_gbl);
	if(ret) {
		err("Unable to register input");
		/* disable receive */
		if(pnet_core_port_unregister(&pnet_ts_gbl.port))
			err("Unable to unregister port");

		goto pnet_ts_init_err_init_input;
	}

	dbg("down");
	if (down_interruptible(&pnet_ts_gbl.mutex)) {
		ret=-ERESTARTSYS;
		goto pnet_ts_init_err_down;
	}

	/* init work tl */
#ifdef PNET_TASKLET_WORK
	tasklet_init(&pnet_ts_gbl.work,pnet_ts_work_proc,(unsigned long)&pnet_ts_gbl);
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
	INIT_WORK(&pnet_ts_gbl.work,pnet_ts_work_proc);
#endif
#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,15)
	INIT_WORK(&pnet_ts_gbl.work,pnet_ts_work_proc,NULL);
#endif
#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,12)
	INIT_WORK(&pnet_ts_gbl.work,pnet_ts_work_proc,NULL);
#endif
#endif

	/* init settings wq */
	init_waitqueue_head(&pnet_ts_gbl.settings.wq);

	/* enable receive */
	ret=pnet_core_port_register(&pnet_ts_gbl.port);
	if(ret) {
		err("Unable to register port");
		goto pnet_ts_init_err_pnet_port;
	}

#ifdef CONFIG_PROC_FS
	pnet_ts_proc_create();
#endif

	/* get all settings on init */
	pnet_ts_gbl.settings.get=0xffff;

	/* set initialized */
	set_bit(PNET_TS_BIT_INITIALIZED,&pnet_ts_gbl.flags);

	/* update */
	pnet_ts_update_settings(&pnet_ts_gbl);

	dbg("up");
	up(&pnet_ts_gbl.mutex);


	dbg("loaded\n");
	ret=0;
	goto pnet_ts_init_ret;

pnet_ts_init_err_pnet_port:
#ifdef PNET_TASKLET_WORK
	tasklet_kill(&pnet_ts_gbl.work);
#else
	flush_scheduled_work();
#endif
	up(&pnet_ts_gbl.mutex);
pnet_ts_init_err_down:
	pnet_ts_deinit_input(&pnet_ts_gbl);
pnet_ts_init_err_init_input:
	pnet_ts_deinit_MUTEX(&pnet_ts_gbl);
pnet_ts_init_ret:
	return ret;
}

static void __exit pnet_ts_exit( void )
{
	dbg("Unregistering driver");

	/* down (if signal -> exit unregister happens also (but dirty)) */
	if(down_interruptible(&pnet_ts_gbl.mutex)) {
		err("Signal while exit");
	}

	/* close (disable) peer */
	pnet_ts_gbl.opencount=0;
#ifdef PNET_TASKLET_WORK
	spin_lock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_lock(&pnet_ts_gbl.settings.lock);
#endif

	pnet_ts_gbl.settings.flags&=~PNET_TS_SETTING_ENABLE;
	pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_ENABLE;
#ifdef PNET_TASKLET_WORK
	spin_unlock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_unlock(&pnet_ts_gbl.settings.lock);
#endif

	pnet_ts_update_settings(&pnet_ts_gbl);

	clear_bit(PNET_TS_BIT_INITIALIZED,&pnet_ts_gbl.flags);

	#ifdef CONFIG_PROC_FS
	pnet_ts_proc_remove();
	#endif

	/* flush work */
#ifdef PNET_TASKLET_WORK
	tasklet_kill(&pnet_ts_gbl.work);
#else
	flush_scheduled_work();
#endif

	/* deinit input device (do not call while mutex-down) */
	pnet_ts_deinit_input(&pnet_ts_gbl);

	/* disable receive */
	if(pnet_core_port_unregister(&pnet_ts_gbl.port))
		err("Unable to unregister port");

	dbg("unloaded");

	up(&pnet_ts_gbl.mutex);

	pnet_ts_deinit_MUTEX(&pnet_ts_gbl);
}


module_init(pnet_ts_init);
module_exit(pnet_ts_exit);



/***********************************************************************
 *  module-parameter relatet functions (for settings)
 ***********************************************************************/

/* set value of module parameter */
static int param_set_pnet_ts_param(const char *pbuf, struct kernel_param *kp)
{
	int offset=0;
	int ret=-EINVAL;

	/* init lock if necessary */
	pnet_ts_init_MUTEX(&pnet_ts_gbl);

	down(&pnet_ts_gbl.mutex);

	dbg("param set %s", &kp->name[offset]);

	/* remove prefix "pnet_ts." */
	if(!strncmp(PNET_TS_DRIVER_NAME,&kp->name[offset],strlen(PNET_TS_DRIVER_NAME))) {
		offset=strlen(PNET_TS_DRIVER_NAME)+1;
	}

	dbg("param set %s", &kp->name[offset]);

#ifdef PNET_TASKLET_WORK
	spin_lock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_lock(&pnet_ts_gbl.settings.lock);
#endif


	if(!strcmp(&kp->name[offset],"enable")) {
		dbg("%s",pbuf);

		if(pbuf[0]=='0') {
			*(unsigned int*)kp->arg&=~PNET_TS_SETTING_ENABLE;
			dbg("disable");
		} else {
			*(unsigned int*)kp->arg|=PNET_TS_SETTING_ENABLE;
			dbg("enable");
		}
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_ENABLE;

		dbg("%x",*(int*)kp->arg);
		ret=0;
		dbg("enable");
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"type")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		ret=param_set_int(pbuf,kp);
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_TYPE;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"autocalibrate")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		dbg("%s",pbuf);
		if(pbuf[0]=='0') {
			*(unsigned int*)kp->arg&=~PNET_TS_SETTING_AUTOCALIBRATE;
			dbg("disable");
		} else {
			*(unsigned int*)kp->arg|=PNET_TS_SETTING_AUTOCALIBRATE;
			dbg("enable");
		}
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_AUTOCALIBRATE;
		dbg("%x",*(int*)kp->arg);
		ret=0;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"sample_delay")) {
		ret=param_set_int(pbuf,kp);
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_SAMPLE_DELAY;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"sample_rate")) {
		ret=param_set_int(pbuf,kp);
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_SAMPLE_RATE;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"use_pendown_line")) {
		if(pbuf[0]=='0') 
			*(unsigned int*)kp->arg&=~PNET_TS_SETTING_USE_PENDOWN_LINE;
		else 
			*(unsigned int*)kp->arg|=PNET_TS_SETTING_USE_PENDOWN_LINE;
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_USE_PENDOWN_LINE;
		ret=0;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"pendown_samples")) {
		ret=param_set_int(pbuf,kp);
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_PENDOWN_SAMPLES;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"pendown_val")) {
		ret=param_set_int(pbuf,kp);
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_PENDOWN_VAL;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"max_delta")) {
		ret=param_set_int(pbuf,kp);
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_MAX_DELTA;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"med_samples")) {
		ret=param_set_int(pbuf,kp);
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_MED_SAMPLES;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"min_x")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		ret=param_set_int(pbuf,kp);
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_MIN_X;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"max_x")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		ret=param_set_int(pbuf,kp);
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_MAX_X;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"min_y")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		ret=param_set_int(pbuf,kp);
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_MIN_Y;
		goto param_set_pnet_ts_param_update;
	}

	if(!strcmp(&kp->name[offset],"max_y")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		ret=param_set_int(pbuf,kp);
		pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_MAX_Y;
		goto param_set_pnet_ts_param_update;
	}

#ifdef PNET_TASKLET_WORK
	spin_unlock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_unlock(&pnet_ts_gbl.settings.lock);
#endif

	goto param_set_pnet_ts_param_return;
	
param_set_pnet_ts_param_update:
#ifdef PNET_TASKLET_WORK
	spin_unlock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_unlock(&pnet_ts_gbl.settings.lock);
#endif

	if(test_bit(PNET_TS_BIT_INITIALIZED,&pnet_ts_gbl.flags)) {
		dbg("update");
		ret=pnet_ts_update_settings(&pnet_ts_gbl);
		if(ret>0)
			ret=0;		
	}

param_set_pnet_ts_param_return:
	up(&pnet_ts_gbl.mutex);
	return ret;
}


/* get value of module parameter */
static int param_get_pnet_ts_param(char *pbuf, struct kernel_param *kp)
{
	int offset=0;
	int ret=-EINVAL;

	/* init lock if necessary */
	pnet_ts_init_MUTEX(&pnet_ts_gbl);

	down(&pnet_ts_gbl.mutex);

	dbg("param get %s", &kp->name[offset]);

	/* remove prefix "pnet_ts." */
	if(!strncmp(PNET_TS_DRIVER_NAME,&kp->name[offset],strlen(PNET_TS_DRIVER_NAME))) {
		offset=strlen(PNET_TS_DRIVER_NAME)+1;
	}

	dbg("param get %s", &kp->name[offset]);

#ifdef PNET_TASKLET_WORK
	spin_lock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_lock(&pnet_ts_gbl.settings.lock);
#endif


	if(!strcmp(&kp->name[offset],"enable")) {
		ret=(((*(unsigned int*)kp->arg)&PNET_TS_SETTING_ENABLE) ? 1 : 0);
		//ret=sprintf((char*)pbuf,(*(unsigned int*)kp->arg&PNET_TS_SETTING_ENABLE ? "enabled" : "disabled"));
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"type")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		ret=param_get_int(pbuf,kp);
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"sample_delay")) {
		ret=param_get_int(pbuf,kp);
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"sample_rate")) {
		ret=param_get_int(pbuf,kp);
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"autocalibrate")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		ret=(((*(unsigned int*)kp->arg)&PNET_TS_SETTING_AUTOCALIBRATE) ? 1 : 0);
//		ret=sprintf((char*)pbuf,(*(unsigned int*)kp->arg&PNET_TS_SETTING_AUTOCALIBRATE ? "enabled" : "disabled"));
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"use_pendown_line")) {
		ret=(((*(unsigned int*)kp->arg)&PNET_TS_SETTING_USE_PENDOWN_LINE) ? 1 : 0);
//		ret=sprintf((char*)pbuf,(*(unsigned int*)kp->arg&PNET_TS_SETTING_USE_PENDOWN_LINE ? "enabled" : "disabled"));
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"pendown_samples")) {
		ret=param_get_int(pbuf,kp);
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"pendown_val")) {
		ret=param_get_int(pbuf,kp);
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"max_delta")) {
		ret=param_get_int(pbuf,kp);
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"med_samples")) {
		ret=param_get_int(pbuf,kp);
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"min_x")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		ret=param_get_int(pbuf,kp);
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"max_x")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		ret=param_get_int(pbuf,kp);
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"min_y")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		ret=param_get_int(pbuf,kp);
		goto param_get_pnet_ts_param_ret;
	}

	if(!strcmp(&kp->name[offset],"max_y")) {
		warn("parameter \"%s\" is deprecated",&kp->name[offset]);
		ret=param_get_int(pbuf,kp);
		goto param_get_pnet_ts_param_ret;
	}

	/* return */
param_get_pnet_ts_param_ret:
#ifdef PNET_TASKLET_WORK
	spin_unlock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_unlock(&pnet_ts_gbl.settings.lock);
#endif

	up(&pnet_ts_gbl.mutex);
	return ret;
}

/* dummy */
#define param_check_pnet_ts_param(name,p)

module_param_named(enable,pnet_ts_gbl.settings.flags,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(enable, "Enable/Disable touchscreen");

module_param_named(type,pnet_ts_gbl.settings.type,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(type, "Type of touchscreen (0 .. 4wire; 1 .. 5wire) (deprecated)");

module_param_named(autocalibrate,pnet_ts_gbl.settings.flags,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(autocalibrate, "Turn on autocalibration (deprecated)");

module_param_named(sample_delay,pnet_ts_gbl.settings.sample_delay,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(sample_delay, "Delay between single samples (us)");

module_param_named(sample_rate,pnet_ts_gbl.settings.sample_rate,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(sample_rate, "Sample Rate (Hz)");

module_param_named(use_pendown_line,pnet_ts_gbl.settings.flags,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(use_pendown_line, "Use the pen downline (faster but needs gpio)");

module_param_named(pendown_samples,pnet_ts_gbl.settings.pendown_samples,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pendown_samples, "Samples to determine pen_down (only if pendownline usage is disabled)");

module_param_named(pendown_val,pnet_ts_gbl.settings.pendown_val,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pendown_val, "Samples lower than this value have pen-down status");

module_param_named(max_delta,pnet_ts_gbl.settings.max_delta,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_delta, "Samples to take to calculate average value");

module_param_named(med_samples,pnet_ts_gbl.settings.med_samples,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(med_samples, "Average values to take to calculate med value");

module_param_named(min_x,pnet_ts_gbl.settings.min_x,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(min_x, "min_x ADC-Value (deprecated)");

module_param_named(max_x,pnet_ts_gbl.settings.max_x,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_x, "max_x ADC-Value (deprecated)");

module_param_named(min_y,pnet_ts_gbl.settings.min_y,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(min_y, "min_y ADC-Value (deprecated)");

module_param_named(max_y,pnet_ts_gbl.settings.max_y,pnet_ts_param,S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_y, "max_y ADC-Value (deprecated)");

/* debug-parameter */
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");



#if 0
/* SYSFS-Parameters example */


static ssize_t pnet_ts_show_enable(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
#if 0
	struct lis302dl_info *lis = dev_get_drvdata(dev);

	/* Display the device view of the threshold setting */
	return sprintf(buf, "%d\n", __threshold_to_mg(lis,
			__mg_to_threshold(lis, lis->threshold)));
#endif
	/* Display the device view of the threshold setting */
	dbg("buf: %s",buf);
	return sprintf(buf, "%d\n", (pnet_ts_gbl.settings.flags&PNET_TS_SETTING_ENABLE) ? 1 : 0);
}

static ssize_t pnet_ts_set_enable(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
#if 0
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	unsigned int val;

	if (sscanf(buf, "%u\n", &val) != 1)
		return -EINVAL;
	/* 8g is the maximum if FS is 1 */
	if (val > 8000)
		return -ERANGE;

	/* Set the threshold and write it out if the device is used */
	lis->threshold = val;

	if (lis->flags & LIS302DL_F_INPUT_OPEN) {
		unsigned long flags;

		local_irq_save(flags);
		__enable_data_collection(lis);
		local_irq_restore(flags);
	}

	return count;
#endif
#ifdef PNET_TASKLET_WORK
	spin_lock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_lock(&pnet_ts_gbl.settings.lock);
#endif

	if (!strcmp(buf, "enable\n")) {
		pnet_ts_gbl.settings.flags|=PNET_TS_SETTING_ENABLE;
		dbg("disable");
	} else {
		pnet_ts_gbl.settings.flags|=PNET_TS_SETTING_ENABLE;
		dbg("enable");
	}
	pnet_ts_gbl.settings.set|=PNET_TS_SETTING_FLAG_ENABLE;

#ifdef PNET_TASKLET_WORK
	spin_unlock_bh(&pnet_ts_gbl.settings.lock);
#else
	spin_unlock(&pnet_ts_gbl.settings.lock);
#endif

	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, pnet_ts_show_enable, pnet_ts_set_enable);

static struct attribute *lis302dl_sysfs_entries[] = {
	&dev_attr_sample_rate.attr,
	&dev_attr_full_scale.attr,
	&dev_attr_threshold.attr,
	&dev_attr_duration.attr,
	&dev_attr_dump.attr,
	&dev_attr_wakeup_threshold.attr,
	&dev_attr_wakeup_duration.attr,
	&dev_attr_overruns.attr,
	NULL
};

static struct attribute_group lis302dl_attr_group = {
	.name	= NULL,
	.attrs	= lis302dl_sysfs_entries,
};


	rc = sysfs_create_group(&lis->dev->kobj, &lis302dl_attr_group);
	if (rc) {
		dev_err(lis->dev, "error creating sysfs group\n");
		goto bail_free_lis;
	}


	sysfs_remove_group(&lis->dev->kobj, &lis302dl_attr_group);

#endif

MODULE_AUTHOR(PNET_TS_DRIVER_AUTHOR);
MODULE_DESCRIPTION(PNET_TS_DRIVER_DESC);
MODULE_LICENSE("GPL");
