/***********************************************************************
 *
 * @Authors: Manfred Schlägl, Ginzinger electronic systems GmbH
 * @Descr: pnet_can driver for pnet
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
 *	2010-KW22 manfred.schlaegl@gmx.at
 *		* port to linux-2.6.32
 *	2010-KW11 manfred.schlaegl@gmx.at
 *		* error-correction: wait for connection on open
 *	2010-KW10 manfred.schlaegl@gmx.at
 *		* new api-functions (pnet_can_mbox_set_enabled/pnet_can_mbox_set_disabled)
 *	2010-KW06 manfred.schlaegl@gmx.at
 *		* interruptible waits replaced by timeout waits
 *		* better api-comments
 *	2010-KW04 manfred.schlaegl@gmx.at
 *		* fifo-waits with timeout and not interruptible
 *			* simpler receive in userspace
 *			* clean userspace close with pending signal
 *		* down not interruptible in close
 *			* clean userspace close with pending signal
 *	2010-KW03 manfred.schlaegl@gmx.at
 *		* pnet_can_update-call from proc
 *		* seperate semaphores for fifo_read, fifo_write and registers, 
 *			instead of semaphores for read and write
 *	2009-KW51 manfred.schlaegl@gmx.at
 *		* seperate read/write semaphores
 *		* comments for api-functions
 *	2009-KW51 manfred.schlaegl@gmx.at
 *		* first working version
 *	2009-KW36 manfred.schlaegl@gmx.at
 *		* development start (based on pnet_rdev)
 ***********************************************************************/
/***********************************************************************
 *  @TODO:
 *		* open with O_NONBLOCK
 ***********************************************************************/

#include <linux/fs.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include "pnet_rdev.h"
#include "pnet_can.h"

/*
 **************************************************************************************************
 * Begin: Basic Definitions
 **************************************************************************************************
 */

#define DRV_MAJOR  										246
#define DRV_NAME										"pnet_can"
#define PNET_CAN_DEV_N									1

/* enable debug output */
#ifdef CONFIG_PNET_CAN_DEBUG
#define DEBUG
#endif
//#define DEBUG

/*
 **************************************************************************************************
 * End: Basic Definitions
 **************************************************************************************************
 */


/*
 **************************************************************************************************
 * Begin: Driver data
 **************************************************************************************************
 */

#define PNET_CAN_DEBUG_STATE_COUNTERS

/* state of pnet_can receiver */
#define PNET_CAN_RCV_STATE_RCV_HEADER	0
#define PNET_CAN_RCV_STATE_RCV_LOAD		1

/* state of pnet_can sender */
#define PNET_CAN_SND_STATE_SND_HEADER	0
#define PNET_CAN_SND_STATE_SND_LOAD		1

typedef struct pnet_can_private
{
	/* driver data */
	/* associated linux device */
	struct device *device;
	int read_count;				/* read-open counter */
	int write_count;			/* write-open counter */
	int disconnected;			/* if set device got disconnected after open */
	int connection;				/* connection-state */
	int enabled;				/* can controller enabled */
	unsigned int mbox_enabled;	/* mbox-enable bits */

	/* wait queues */
	wait_queue_head_t open_wait;
	wait_queue_head_t snd_wait;
	wait_queue_head_t rcv_wait;

	/* receiver-state */
	int rcv_state;

	/* sender-state */
	int snd_state;

	/* remote driver data */
	struct pnet_can_info remote_info;
	/* can configuration */
	struct pnet_can_config config;
	/* mailbox configuration */
	struct pnet_can_mbox_config mbox_config[PNET_CAN_MAILBOX_MAX];

	/* data spinlock */
	spinlock_t data_lock;

	/* associated pnet_rdev */
	pnet_rdev_private_t *pnet_rdev;
	struct semaphore reg_sem;		/* semaphore for registers(lin-ioctl) functions */
	struct semaphore fifo_rsem;		/* semaphore for fifo(lin-write) read-functions */
	struct semaphore fifo_wsem;		/* semaphore for fifo(lin-write) write-functions */

#ifdef PNET_CAN_DEBUG_STATE_COUNTERS
#endif
} pnet_can_private_t;

static struct pnet_can_private pnet_can_private[PNET_CAN_DEV_N];

/*
 **************************************************************************************************
 * End: Driver data
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Misc
 **************************************************************************************************
 */
#ifndef MIN
#define MIN(a,b)	((a)>(b) ? (b) : (a))
#endif
 
#ifndef MAX
#define MAX(a,b)	((a)>(b) ? (a) : (b))
#endif

//#define LOGS
#ifdef LOGS
#define logs(str)		logs_str(str)
#define	LOGS_MEM_SIZE 		4096
static unsigned char 	logs_mem[LOGS_MEM_SIZE];
static int		logs_i;
static void logs_init(void)
{
	logs_i=0;
	memset(logs_mem,0,LOGS_MEM_SIZE);
}
static void logs_str(unsigned char *s)
{
	while(*s) {
		logs_mem[logs_i++]=*s++;
		if (logs_i>=LOGS_MEM_SIZE) logs_i=0;
	}
}
#else
#define logs(str)		do {} while (0)
#define logs_init()		do {} while (0)
#define logs_print()		do {} while (0)
#endif

/*
 **************************************************************************************************
 * End: Misc
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Debug
 **************************************************************************************************
 */
#ifdef DEBUG
#define dbg(format, arg...) printk(KERN_INFO "%s: %s: " format "\n" , __FILE__, __FUNCTION__, ## arg)
//#define dbg(format, arg...) printk(KERN_DEBUG "%s: " format "\n" , __FILE__ , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif

#define err(format, arg...) printk(KERN_ERR "%s: %s: " format "\n" , __FILE__, __FUNCTION__, ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: %s: " format "\n" , __FILE__, __FUNCTION__, ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: %s: " format "\n", __FILE__, __FUNCTION__, ## arg)

/*
 **************************************************************************************************
 * End: Debug
 **************************************************************************************************
 */



/************************************************************************
 * BEGIN: pnet_rdev interface
 ************************************************************************/

void pnet_can_rdev_data_callback(pnet_rdev_private_t *pnet_rdev, int evcode)
{
	/* get pnet_can */
	struct pnet_can_private *pnet_can=(struct pnet_can_private*)pnet_rdev->priv;

	if(evcode==PNET_RDEV_CALLBACK_EV_CONNECTED) {
		/* pnet_rdev connected */
		dbg("PNET_RDEV_CALLBACK_EV_CONNECTED");
	} else if(evcode==PNET_RDEV_CALLBACK_EV_DISCONNECTED) {
		/* pnet_rdev disconnected */
		dbg("PNET_RDEV_CALLBACK_EV_DISCONNECTED");
	} else if(evcode==PNET_RDEV_CALLBACK_EV_RECEIVED) {
		/* data received */
		dbg("PNET_RDEV_CALLBACK_EV_RECEIVED -> wakeup rcv");
		wake_up(&pnet_can->rcv_wait);
	} else if(evcode==PNET_RDEV_CALLBACK_EV_TRANSMITTED) {
		/* data sent */
		dbg("PNET_RDEV_CALLBACK_EV_TRANSMITTED -> wakeup snd");
		wake_up(&pnet_can->snd_wait);
	}
}

void pnet_can_rdev_register_callback(pnet_rdev_private_t *pnet_rdev, int evcode)
{
	/* get pnet_can */
	struct pnet_can_private *pnet_can=(struct pnet_can_private*)pnet_rdev->priv;

	if(evcode==PNET_RDEV_CALLBACK_EV_CONNECTED) {
		dbg("PNET_RDEV_CALLBACK_EV_CONNECTED");

		/* pnet_rdev connected */
		pnet_can->connection=1;

		/* reset data and states */
		pnet_can->rcv_state=PNET_CAN_RCV_STATE_RCV_HEADER;

		/* wakeup all */
		wake_up(&pnet_can->rcv_wait);
		wake_up(&pnet_can->snd_wait);
		wake_up(&pnet_can->open_wait);
	} else if(evcode==PNET_RDEV_CALLBACK_EV_DISCONNECTED) {
		dbg("PNET_RDEV_CALLBACK_EV_DISCONNECTED");

		/* reset data */
		memset(&pnet_can->config,0,sizeof(struct pnet_can_config));
		memset(&pnet_can->remote_info,0,sizeof(struct pnet_can_info));
		memset(pnet_can->mbox_config,0,sizeof(struct pnet_can_mbox_config)*PNET_CAN_MAILBOX_MAX);
		pnet_can->enabled=0;
		pnet_can->mbox_enabled=0;

		/* pnet_rdev disconnected */
		pnet_can->connection=0;

		/* set disconnect-flag */
		pnet_can->disconnected=1;

		/* wakeup all */
		wake_up(&pnet_can->rcv_wait);
		wake_up(&pnet_can->snd_wait);
		wake_up(&pnet_can->open_wait);
	} else if(evcode==PNET_RDEV_CALLBACK_EV_RECEIVED) {
		/* registers received (remote commit) */
		dbg("PNET_RDEV_CALLBACK_EV_RECEIVED -> wakeup rcv");
		wake_up(&pnet_can->rcv_wait);
	} else if(evcode==PNET_RDEV_CALLBACK_EV_TRANSMITTED) {
		/* registers sent (local commit done) */
		dbg("PNET_RDEV_CALLBACK_EV_TRANSMITTED -> wakeup snd");
		wake_up(&pnet_can->snd_wait);
	}
}

/*
 **************************************************************************************************
 * End: pnet_rdev interface
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: pnet_can driver
 **************************************************************************************************
 */

/***** Low-Level functions: rx/tx-fifo *****/

/*
 * read a fixed amount of data from pnet_rdev
 * parameters:
 *	pnet_can .. pnet_can instance
 *	buf .. buffer to read in
 *	count .. number of bytes to read
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
static inline int pnet_can_fifo_rcv(struct pnet_can_private *pnet_can, char *buf, int count, int wait)
{
	int ret;

	dbg("buflen: %i",count);

	/* check for received data */
	while(pnet_rdev_read_len(pnet_can->pnet_rdev)<count) {
		if(wait) {
			dbg("sleep");
			if (!wait_event_timeout(pnet_can->rcv_wait,
					(pnet_rdev_read_len(pnet_can->pnet_rdev)>=count) || pnet_can->disconnected,
					wait)
			) {
				return -EAGAIN;
			}
			dbg("wakeup");
			if(pnet_can->disconnected)
				return -EPIPE;
		} else {
			return -EAGAIN;
		}
	}

	if(pnet_can->disconnected)
		return -EPIPE;

	/* read data */
	ret=pnet_rdev_read(pnet_can->pnet_rdev,buf,count);
	if(ret<0)
		return ret;
	if(ret!=count) {
		/* error */
		return -EPIPE;
	}

	return 0;
}


/*
 * write a fixed amount of data to pnet_rdev
 * parameters:
 *	pnet_can .. pnet_can instance
 *	buf .. buffer to write from
 *	count .. number of bytes to write
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
static inline int pnet_can_fifo_snd(struct pnet_can_private *pnet_can, char *buf, int count, int wait)
{
	int ret;

	dbg("buflen: %i",count);

	/* check for enough space */
	while(pnet_rdev_write_free(pnet_can->pnet_rdev)<count) {
		if(wait) {
			dbg("sleep");
			if (!wait_event_timeout(pnet_can->snd_wait,
					(pnet_rdev_write_free(pnet_can->pnet_rdev)>=count) || pnet_can->disconnected,
					wait)
			) {
				return -ETIME;
			}
			dbg("wakeup");
			if(pnet_can->disconnected)
				return -EPIPE;
		} else {
			return -EAGAIN;
		}
	}

	/* write data */
	ret=pnet_rdev_write(pnet_can->pnet_rdev,buf,count);
	if(ret<0)
		return ret;
	if(ret!=count) {
		/* error */
		return -EPIPE;
	}

	return 0;
}

/***** Low-Level functions: async-commands (ioctl) *****/

#define PNET_CAN_RDEV_REGISTER_CMD 				0
#define PNET_CAN_RDEV_REGISTER_CMDRET			1
#define PNET_CAN_RDEV_REGISTER_ARG(__idx__)		(2+(__idx__))

#define PNET_CAN_CMD_INFO				0
#define PNET_CAN_CMD_ENABLE				1
#define PNET_CAN_CMD_CONFIG				2
#define PNET_CAN_CMD_MBOX_CONFIG		3
#define PNET_CAN_CMD_MBOX_ENABLE		4
#define PNET_CAN_CMD_MBOX_REPLY_UPDATE	5
#define PNET_CAN_CMD_MBOX_SEND_REQUEST	6
#define PNET_CAN_CMD_MBOX_SEND_DATA		7
#define PNET_CAN_CMD_MBOX_SEND_MSG		8

/*
 * do command/ioctl over pnet_rdev
 * parameters:
 *	pnet_can .. pnet_can instance
 *	cmd .. command-code
 *	arg .. argument list
 *  argnr .. number of arguments in list
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); >=0 .. ok(number of return arguments)
 */
static inline int pnet_can_cmd(struct pnet_can_private *pnet_can, int cmd, int arg[], int argc, int wait)
{
	int rcmd,rargc;
	int i,ret;
#ifdef DEBUG
	char str[512];
#endif

#ifdef DEBUG
	sprintf(str, "command sent: cmd: 0x%X ",cmd);
	for(i=0;i<argc;i++)
		sprintf(str, "%s arg%i: 0x%X ",str,i,arg[i]);
	dbg("%s",str);
#endif

	/* set cmd & argc */
	i=cmd<<16|argc;

	dbg("reset");

	/* reset registers */
	ret=pnet_rdev_register_reset(pnet_can->pnet_rdev, wait);
	if(ret<0)
		return ret;

	dbg("write");

	/* set args */
	ret=pnet_rdev_register_write(pnet_can->pnet_rdev, PNET_CAN_RDEV_REGISTER_CMD, (unsigned int)i, wait);
	if(ret<0)
		goto __ret_reset;	/* reset registers */
	for(i=0;i<argc;i++) {
		ret=pnet_rdev_register_write(pnet_can->pnet_rdev, PNET_CAN_RDEV_REGISTER_ARG(i), (unsigned int)arg[i], wait);
		if(ret<0)
			goto __ret_reset;	/* reset registers */
	}

	dbg("read");

	/* dummy read -> reset register changed flag */
	pnet_rdev_register_read(pnet_can->pnet_rdev, PNET_CAN_RDEV_REGISTER_CMDRET, (unsigned int*)&i);

	dbg("commit");

	/* do command */
	ret=pnet_rdev_register_commit(pnet_can->pnet_rdev, wait);
	if(ret<0)
		goto __ret_reset;	/* reset registers */

	dbg("wait remote commit");

	/* wait for return */
	do {
		dbg("wait (%i)",wait);

		/* wait for remote commit */
		ret=pnet_rdev_register_wait_remote_commit(pnet_can->pnet_rdev, wait);
		if(ret<0)
			return ret;

		dbg("wakeup");

		/* dummy read -> reset register changed flag */
		ret=pnet_rdev_register_read(pnet_can->pnet_rdev, PNET_CAN_RDEV_REGISTER_CMDRET, (unsigned int*)&i);
		if(ret<0)
			return ret;
		if(ret==1) {
			/* return received -> stop loop */
			dbg("ret");
			break;
		}
		dbg("no ret");
	} while(1);

	/* return was received */
	
	/* get cmd & argc */
	rargc=i&0xffff;
	rcmd=(i>>16)&0xffff;

	/* check return */
	if(rcmd!=cmd) {
		err("wong return-cmd for command (sent -> cmd: 0x%X, argc: 0x%X, received -> cmd: 0x%X, argc: 0x%X)",cmd,argc,rcmd,rargc);
		return -EPIPE;
	}

	dbg("read");

	/* copy return arguments */
	for(i=0;i<rargc;i++)
		pnet_rdev_register_read(pnet_can->pnet_rdev, PNET_CAN_RDEV_REGISTER_ARG(i), (unsigned int*)&arg[i]);

#ifdef DEBUG
	sprintf(str, "command return: cmd: 0x%X ",rcmd);
	for(i=0;i<rargc;i++)
		sprintf(str, "%s arg%i: 0x%X ",str,i,arg[i]);
	dbg("%s",str);
#endif

	/* ok */
	return rargc;

__ret_reset:
	/* reset registers */
	ret=pnet_rdev_register_reset(pnet_can->pnet_rdev, wait);
	return ret;
}








/***** High-level functions: async-commands (ioctl) *****/

/*
 * update info-struct of can
 * parameters:
 *	pnet_can .. pnet_can instance
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_update_info(struct pnet_can_private *pnet_can, int wait)
{
	int ret;
	int arg[2];

	dbg("");

	/* do command */
	ret=pnet_can_cmd(pnet_can, PNET_CAN_CMD_INFO, arg, 0, wait);
	if(ret<0)
		return ret;

	/* update info */
	pnet_can->remote_info.version_lo=(arg[0]>>16&0xffff);
	pnet_can->remote_info.version_hi=arg[0]&0xffff;
	pnet_can->remote_info.mbox_n=(arg[1]>>24)&0xff;
	pnet_can->remote_info.mask_n=(arg[1]>>16)&0xff;
	
	/* normal return */
	return 0;
}
EXPORT_SYMBOL(pnet_can_update_info);

/*
 * enable/disable can controller
 * parametesr:
 *	pnet_can .. pnet_can instance
 *	ena .. enable/disable can (0 .. disable; 1 .. enable)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_enable(struct pnet_can_private *pnet_can, int ena, int wait)
{
	int ret;
	int arg[1];

	dbg("enable: %i",ena);

	/* arg0 = ena */
	arg[0]=ena;

	/* do command */
	ret=pnet_can_cmd(pnet_can, PNET_CAN_CMD_ENABLE, arg, 1, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_can_enable);

/*
 * configure can controller
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_cfg .. can-configuration
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok; 1 .. invalid settings
 */
int pnet_can_config(struct pnet_can_private *pnet_can, struct pnet_can_config can_cfg, int wait)
{
	int i,ret;
	int arg[2+32];

	dbg("bitrate: %i, sample_point: %i, edge_resync_mode: %i, sync_jump_width: %i, byte_order: %i, masks:todo",
		can_cfg.bitrate,
		can_cfg.sample_point,
		can_cfg.edge_resync_mode,
		can_cfg.sync_jump_width,
		can_cfg.byte_order
	);

	/* arg0 = bitrate */
	arg[0]=can_cfg.bitrate;

	/* arg1 = sample_point, erm, sjw, byteorder */
	arg[1]=can_cfg.byte_order;
	arg[1]|=(can_cfg.sync_jump_width<<8);
	arg[1]|=(can_cfg.edge_resync_mode<<16);
	arg[1]|=(can_cfg.sample_point<<24);

	/* arg2 - arg[2+CAN1_MASK_MAX] = masks[CAN1_MASK_MAX] */
	for(i=0;i<pnet_can->remote_info.mask_n;i++)
		arg[2+i]=can_cfg.mask[i];

	/* do command */
	ret=pnet_can_cmd(pnet_can, PNET_CAN_CMD_CONFIG, arg, 2+pnet_can->remote_info.mask_n, wait);
	if(ret<0)
		return ret;

	/* check remote error */
	if(arg[0])
		return arg[0];

	/* update data */
	memcpy(&pnet_can->config,&can_cfg,sizeof(struct pnet_can_config));

	/* ok */
	return 0;
}
EXPORT_SYMBOL(pnet_can_config);


/*
 * configures a specified mailbox (mailbox is left disabled)
 * parameter:
 *	pnet_can .. pnet_can instance
 *	can_mbox_cfg .. can-mailbox-configuration
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_config(struct pnet_can_private *pnet_can, struct pnet_can_mbox_config can_mbox_cfg, int wait)
{
	int ret;
	int arg[2];

	dbg("mbox: %i, message_id: 0x%X, type: 0x%X, data_len: %i",can_mbox_cfg.mbox,can_mbox_cfg.message_id,can_mbox_cfg.type,can_mbox_cfg.data_len);

	/* arg0 = message_id */
	arg[0]=can_mbox_cfg.message_id;

	/* arg1 = mbox, type, data_len */
	arg[1]=0;
	arg[1]|=(can_mbox_cfg.data_len<<8);
	arg[1]|=(can_mbox_cfg.type<<16);
	arg[1]|=(can_mbox_cfg.mbox<<24);

	/* do command */
	ret=pnet_can_cmd(pnet_can, PNET_CAN_CMD_MBOX_CONFIG, arg, 2, wait);
	if(ret<0)
		return ret;

	/* check remote error */
	if(arg[0])
		return arg[0];

	/* update data */
	memcpy(&pnet_can->mbox_config[can_mbox_cfg.mbox],&can_mbox_cfg,sizeof(struct pnet_can_mbox_config));

	/* ok */
	return 0;
}
EXPORT_SYMBOL(pnet_can_mbox_config);


/*
 * set mailboxes enabled or disabled
 * parameters:
 *	pnet_can .. pnet_can instance
 *	ena .. bitfield of mboxes (1..enable/0..disable)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_enable(struct pnet_can_private *pnet_can, unsigned int ena, int wait)
{
	int ret;
	int arg[1];

	dbg("ena: 0x%X",ena);

	/* arg0 = ena */
	arg[0]=ena;

	/* do command */
	ret=pnet_can_cmd(pnet_can, PNET_CAN_CMD_MBOX_ENABLE, arg, 1, wait);
	if(ret<0)
		return ret;

	/* check remote error */
	if(arg[0])
		return arg[0];

	/* update data */
	pnet_can->mbox_enabled=ena;

	/* ok */
	return 0;
}
EXPORT_SYMBOL(pnet_can_mbox_enable);

/*
 * set mailboxes enabled
 * parameters:
 *	pnet_can .. pnet_can instance
 *	ena .. bitfield of mboxes (1..enable/0..do nothing)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_set_enabled(struct pnet_can_private *pnet_can, unsigned int ena, int wait)
{
	int ret;
	int arg[1];

	dbg("ena: 0x%X",ena);

	/* arg0 = ena */
	ena=pnet_can->mbox_enabled|ena;
	arg[0]=ena;

	/* do command */
	ret=pnet_can_cmd(pnet_can, PNET_CAN_CMD_MBOX_ENABLE, arg, 1, wait);
	if(ret<0)
		return ret;

	/* check remote error */
	if(arg[0])
		return arg[0];

	/* update data */
	pnet_can->mbox_enabled=ena;

	/* ok */
	return 0;
}
EXPORT_SYMBOL(pnet_can_mbox_set_enabled);


/*
 * set mailboxes disabled
 * parameters:
 *	pnet_can .. pnet_can instance
 *	ena .. bitfield of mboxes (1..disable/0..do nothing)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_set_disabled(struct pnet_can_private *pnet_can, unsigned int dis, int wait)
{
	int ena;
	int ret;
	int arg[1];

	dbg("dis: 0x%X",dis);

	/* arg0 = ena */
	ena=pnet_can->mbox_enabled&~dis;
	arg[0]=dis;

	/* do command */
	ret=pnet_can_cmd(pnet_can, PNET_CAN_CMD_MBOX_ENABLE, arg, 1, wait);
	if(ret<0)
		return ret;

	/* check remote error */
	if(arg[0])
		return arg[0];

	/* update data */
	pnet_can->mbox_enabled=ena;

	/* ok */
	return 0;
}
EXPORT_SYMBOL(pnet_can_mbox_set_disabled);


/*
 * update data in reply-mailbox - unqueued
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. mailbox-data
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok; 1.. busy
 */
int pnet_can_mbox_update_snd_async(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait)
{
	int ret;
	int arg[3];

	dbg("data: 0x%llX",*((unsigned long long*)can_msg.data));

	/* arg0 = mbox */
	arg[0]=can_msg.mbox;

	/* arg1,arg2 = data */
	arg[1]=(can_msg.data[4]<<24) | (can_msg.data[5]<<16) | (can_msg.data[6]<<8) | (can_msg.data[7]<<0);
	arg[2]=(can_msg.data[0]<<24) | (can_msg.data[1]<<16) | (can_msg.data[2]<<8) | (can_msg.data[3]<<0);

	/* do command */
	ret=pnet_can_cmd(pnet_can, PNET_CAN_CMD_MBOX_REPLY_UPDATE, arg, 3, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_can_mbox_update_snd_async);


/* request */

/*
 * do request on mailbox - unqueued
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok; 1.. busy
 */
int pnet_can_mbox_request_snd_async(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait)
{
	int ret;
	int arg[1];

	dbg("mbox: %i",can_msg.mbox);

	/* arg0 = mbox */
	arg[0]=can_msg.mbox;

	/* do command */
	ret=pnet_can_cmd(pnet_can, PNET_CAN_CMD_MBOX_SEND_REQUEST, arg, 1, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_can_mbox_request_snd_async);


/* send */

/*
 * send data on mailbox with configured message_id - unqueued
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok; 1.. busy
 */
int pnet_can_mbox_data_snd_async(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait)
{
	int ret;
	int arg[3];

	dbg("mbox: %i, data: 0x%llX, data_len: %i",can_msg.mbox,*((unsigned long long*)can_msg.data),can_msg.data_len);

	/* arg0 = mbox, data_len */
	arg[0]=(can_msg.mbox<<24)|(can_msg.data_len<<16);

	/* arg1,arg2 = data */
	arg[1]=(can_msg.data[4]<<24) | (can_msg.data[5]<<16) | (can_msg.data[6]<<8) | (can_msg.data[7]<<0);
	arg[2]=(can_msg.data[0]<<24) | (can_msg.data[1]<<16) | (can_msg.data[2]<<8) | (can_msg.data[3]<<0);

	/* do command */
	ret=pnet_can_cmd(pnet_can, PNET_CAN_CMD_MBOX_SEND_DATA, arg, 3, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_can_mbox_data_snd_async);


/*
 * send can message on mailbox - unqueued
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.message_id .. message_id
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok; 1.. busy
 */
int pnet_can_mbox_msg_snd_async(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait)
{
	int ret;
	int arg[4];

	dbg("mbox: %i, message_id: 0x%X, data: 0x%llX, data_len: %i",can_msg.mbox,can_msg.message_id,*((unsigned long long*)can_msg.data),can_msg.data_len);

	/* arg0 = message_id */
	arg[0]=can_msg.message_id;

	/* arg1 = mbox, data_len */
	arg[1]=(can_msg.mbox<<24)|(can_msg.data_len<<16);

	/* arg1,arg2 = data */
	arg[2]=(can_msg.data[4]<<24) | (can_msg.data[5]<<16) | (can_msg.data[6]<<8) | (can_msg.data[7]<<0);
	arg[3]=(can_msg.data[0]<<24) | (can_msg.data[1]<<16) | (can_msg.data[2]<<8) | (can_msg.data[3]<<0);

	/* do command */
	ret=pnet_can_cmd(pnet_can, PNET_CAN_CMD_MBOX_SEND_MSG, arg, 4, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_can_mbox_msg_snd_async);


/***** High-level functions: sync-commands (fifo) *****/

/*
 * send can package - queued
 * parameters:
 *	pnet_can .. pnet_can instance
 * 	package .. can-message to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_package_snd_sync(struct pnet_can_private *pnet_can, struct pnet_can_package package, int wait)
{
	int ret, datalen;
	char *databuf;

	if(pnet_can->snd_state==PNET_CAN_SND_STATE_SND_HEADER) {
		dbg("mbox: %i, type: %i, datalen: %i, message_id: %X, data: 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X",
			package.head.mbox_nr,
			(package.head.hdr>>4),
			(package.head.hdr&0x0f),
			package.load.message_id,
			package.load.md[0],
			package.load.md[1],
			package.load.md[2],
			package.load.md[3],
			package.load.md[4],
			package.load.md[5],
			package.load.md[6],
			package.load.md[7]
		);

		/* convert */
		package.load.message_id=htonl(package.load.message_id);

		/* send */
		ret=pnet_can_fifo_snd(pnet_can,(char*)&package.head,sizeof(struct pnet_can_package_header),wait);
		if(ret)
			return ret;

		/* header sent -> send load */
		pnet_can->snd_state=PNET_CAN_SND_STATE_SND_LOAD;
	}

	if(pnet_can->snd_state==PNET_CAN_SND_STATE_SND_LOAD) {
		if( (package.head.hdr>>4) == PNET_CAN_MSGTYPE_MSG) {
			/* send message_id and data */
			databuf=(char*)&package.load;
			datalen=(package.head.hdr&0x0f)+sizeof(unsigned int);
		} else {
			/* send data */
			databuf=(char*)&package.load.md;
			datalen=(package.head.hdr&0x0f);
		}
		
		/* send */
		ret=pnet_can_fifo_snd(pnet_can,databuf,datalen,wait);
		if(ret)
			return ret;

		/* load sent -> send next header */
		pnet_can->snd_state=PNET_CAN_SND_STATE_SND_HEADER;
	}

	return 0;
}
EXPORT_SYMBOL(pnet_can_package_snd_sync);

/*
 * receive can package - queued
 * parameters:
 *	pnet_can .. pnet_can intance
 * 	package .. can-message received
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_package_rcv_sync(struct pnet_can_private *pnet_can, struct pnet_can_package *package, int wait)
{
	int ret, datalen;
	char *databuf;

	if(pnet_can->rcv_state==PNET_CAN_RCV_STATE_RCV_HEADER) {
		/* no header received -> try to read header */

		/* receive */
		ret=pnet_can_fifo_rcv(pnet_can,(char*)&package->head,sizeof(struct pnet_can_package_header),wait);
		if(ret)
			return ret;

		/* header received -> receive load */
		pnet_can->rcv_state=PNET_CAN_RCV_STATE_RCV_LOAD;
	}

	if(pnet_can->rcv_state==PNET_CAN_RCV_STATE_RCV_LOAD) {
		/* header received -> try to read load */
		if( (package->head.hdr>>4) == PNET_CAN_MSGTYPE_MSG) {
			/* receive message_id and data */
			databuf=(char*)&package->load;
			datalen=(package->head.hdr&0x0f)+sizeof(unsigned int);
		} else {
			/* receive data */
			databuf=(char*)&package->load.md;
			datalen=(package->head.hdr&0x0f);
		}

		/* receive */
		ret=pnet_can_fifo_rcv(pnet_can,databuf,datalen,wait);
		if(ret)
			return ret;

		dbg("mbox: %i, type: %i, datalen: %i, message_id: %X, data: 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X",
			package->head.mbox_nr,
			(package->head.hdr>>4),
			(package->head.hdr&0x0f),
			package->load.message_id,
			package->load.md[0],
			package->load.md[1],
			package->load.md[2],
			package->load.md[3],
			package->load.md[4],
			package->load.md[5],
			package->load.md[6],
			package->load.md[7]
		);

		/* convert */
		package->load.message_id=ntohl(package->load.message_id);

		/* data received -> read next header */
		pnet_can->rcv_state=PNET_CAN_RCV_STATE_RCV_HEADER;
	}

	/* done */
	return 0;
}
EXPORT_SYMBOL(pnet_can_mbox_package_rcv_sync);

/*
 * send can message on mailbox - queued
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.message_id .. message_id
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_msg_snd_sync(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait)
{
	struct pnet_can_package can_package;

	dbg("mbox: %i, datalen: %i, message_id: %X, data: 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X",
		can_msg.mbox,
		can_msg.data_len,
		can_msg.message_id,
		can_msg.data[0],
		can_msg.data[1],
		can_msg.data[2],
		can_msg.data[3],
		can_msg.data[4],
		can_msg.data[5],
		can_msg.data[6],
		can_msg.data[7]
	);

	/* create can_package */
	can_package.head.mbox_nr=can_msg.mbox;
	can_package.head.hdr=( PNET_CAN_MSGTYPE_MSG<<4 ) | (can_msg.data_len);
	can_package.load.message_id=can_msg.message_id;
	can_data_cp(can_package.load.md,can_msg.data);

	/* send */
	return pnet_can_package_snd_sync(pnet_can,can_package,wait);
}
EXPORT_SYMBOL(pnet_can_mbox_msg_snd_sync);

/*
 * receive can message - queued
 *	pnet_can .. pnet_can instance
 *	can_msg->mbox .. mailbox-number
 *	can_msg->message_id .. message_id
 *	can_msg->data .. data to send
 *	can_msg->data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_msg_rcv_sync(struct pnet_can_private *pnet_can, struct pnet_can_msg *can_msg, int wait)
{
	int ret;
	struct pnet_can_package can_package;

	/* receive package */
	ret=pnet_can_mbox_package_rcv_sync(pnet_can,&can_package,wait);
	if(ret)
		return ret;

	/* create can_msg */
	can_msg->mbox=can_package.head.mbox_nr;
	can_msg->message_id=can_package.load.message_id;
	can_msg->data_len=(can_package.head.hdr&0xf);
	can_data_cp(can_msg->data,can_package.load.md);

	dbg("mbox: %i, datalen: %i, message_id: %X, data: 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X",
		can_msg->mbox,
		can_msg->data_len,
		can_msg->message_id,
		can_msg->data[0],
		can_msg->data[1],
		can_msg->data[2],
		can_msg->data[3],
		can_msg->data[4],
		can_msg->data[5],
		can_msg->data[6],
		can_msg->data[7]
	);

	return 0;
}
EXPORT_SYMBOL(pnet_can_mbox_msg_rcv_sync);

/*
 * send data on mailbox with configured message_id - queued
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_data_snd_sync(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait)
{
	struct pnet_can_package can_package;

	dbg("mbox: %i, datalen: %i, data: 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X",
		can_msg.mbox,
		can_msg.data_len,
		can_msg.data[0],
		can_msg.data[1],
		can_msg.data[2],
		can_msg.data[3],
		can_msg.data[4],
		can_msg.data[5],
		can_msg.data[6],
		can_msg.data[7]
	);

	/* create can_package */
	can_package.head.mbox_nr=can_msg.mbox;
	can_package.head.hdr=(PNET_CAN_MSGTYPE_DATA<<4)|(can_msg.data_len&0xf);
	can_data_cp(can_package.load.md,can_msg.data);

	/* send */
	return pnet_can_package_snd_sync(pnet_can,can_package,wait);
}
EXPORT_SYMBOL(pnet_can_mbox_data_snd_sync);

/*
 * update data in reply-mailbox - queued
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_update_snd_sync(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait)
{
	struct pnet_can_package can_package;

	dbg("mbox: %i, data: 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X",
		can_msg.mbox,
		can_msg.data[0],
		can_msg.data[1],
		can_msg.data[2],
		can_msg.data[3],
		can_msg.data[4],
		can_msg.data[5],
		can_msg.data[6],
		can_msg.data[7]
	);

	/* create can_package */
	can_package.head.mbox_nr=can_msg.mbox;
	can_package.head.hdr=(PNET_CAN_MSGTYPE_UPDATE<<4)|(can_msg.data_len&0xf);
	can_data_cp(can_package.load.md,can_msg.data);

	/* send */
	return pnet_can_package_snd_sync(pnet_can,can_package,wait);
}
EXPORT_SYMBOL(pnet_can_mbox_update_snd_sync);

/*
 * do request on mailbox - queued
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_request_snd_sync(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait)
{
	struct pnet_can_package can_package;

	dbg("mbox: %i",can_msg.mbox);

	/* create can_package */
	can_package.head.mbox_nr=can_msg.mbox;
	can_package.head.hdr=(PNET_CAN_MSGTYPE_REQUEST<<4);

	/* send */
	return pnet_can_package_snd_sync(pnet_can,can_package,wait);
}
EXPORT_SYMBOL(pnet_can_mbox_request_snd_sync);

/***** High-level functions: driver - metafunctions *****/

/*
 * reset can controller
 * parameters:
 * 	pnet_can .. pnet_can instance
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_reset(struct pnet_can_private *pnet_can, int wait)
{
	int ret;
	char ch;

	dbg("");

	/* get info */
	ret=pnet_can_update_info(pnet_can, wait);
	if((ret<0)&&(ret!=-EAGAIN)) {
		err("pnet_can_update_info");
		return ret;
	}

	/* disable mailboxes */
	ret=pnet_can_mbox_enable(pnet_can,0,wait);
	if((ret<0)&&(ret!=-EAGAIN)) {
		err("pnet_can_mbox_enable");
		return ret;
	}

	/* disable can */
	ret=pnet_can_enable(pnet_can,0,wait);
	if((ret<0)&&(ret!=-EAGAIN)) {
		err("pnet_can_enable");
		return ret;
	}

	/* flush fifo */
	while((ret=pnet_rdev_read(pnet_can->pnet_rdev,&ch,1))>0);
	if((ret<0)&&(ret!=-EAGAIN)) {
		err("pnet_rdev_read");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(pnet_can_reset);

/*
 **************************************************************************************************
 * End: pnet_can driver
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: API-Helper - Functions
 **************************************************************************************************
 */

/*
 * low-level api-can-open
 * parameters:
 *	minor .. can minor (same as chrdev-minor)
 *	pnet_can .. pointer will to pnet_can instance-pointer (will be set on success)
 *	accmod .. like file ACCMODE (O_RDWR, O_RDONLY, O_WRONLY)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can__open(int minor, struct pnet_can_private **pnet_can, int accmode, int wait)
{
	unsigned long iflags;
	int ret=0;

	if (minor>=PNET_CAN_DEV_N)
		return -EINVAL;

	dbg("minor: %i",minor);

	spin_lock_irqsave(&pnet_can_private[minor].data_lock,iflags);

	if (accmode==O_RDWR) {
		if (pnet_can_private[minor].write_count>0 || pnet_can_private[minor].read_count>0) {
			ret=-EBUSY;
			goto busy_error;
		}
		pnet_can_private[minor].write_count++;
		pnet_can_private[minor].read_count++;
	}
	else if (accmode==O_WRONLY) {
		if (pnet_can_private[minor].write_count>0) {
			ret=-EBUSY;
			goto busy_error;
		}
		pnet_can_private[minor].write_count++;
	}
	else if (accmode==O_RDONLY) {
		if (pnet_can_private[minor].read_count>0) {
			ret=-EBUSY;
			goto busy_error;
		}
		pnet_can_private[minor].read_count++;
	}

	spin_unlock_irqrestore(&pnet_can_private[minor].data_lock,iflags);

	/* TODO: open with O_NONBLOCK */
	if(!wait)
		return -EPERM;

	/* reset disconnected before wait */
	pnet_can_private[minor].disconnected=0;
	/* wait for connected */
	if (!pnet_can_private[minor].connection) {
		if (!wait) {
			ret=-EPIPE;
			goto open_error;
		}

		if (wait_event_timeout(pnet_can_private[minor].open_wait,(pnet_can_private[minor].connection),wait)<0) {
			ret=-ERESTARTSYS;
			goto open_error;
		}
	}

	/* open pnet_rdev for can */
	ret=pnet_rdev_open(pnet_can_private[minor].pnet_rdev);
	if(ret)
		goto open_error;

	/* reset pnet_can */
	ret=pnet_can_reset(&pnet_can_private[minor],wait);
	if(ret)
		goto can_open_error;

	/* ok */
	(*pnet_can)=&pnet_can_private[minor];
	ret=0;
	goto ret;

can_open_error:
	pnet_rdev_close(pnet_can_private[minor].pnet_rdev);
open_error:
	spin_lock_irqsave(&pnet_can_private[minor].data_lock,iflags);
	if (accmode==O_RDWR || accmode==O_WRONLY) 
		if (pnet_can_private[minor].write_count>0) pnet_can_private[minor].write_count--;
	if (accmode==O_RDWR || accmode==O_RDONLY) 
		if (pnet_can_private[minor].read_count>0) pnet_can_private[minor].read_count--;
busy_error:
	spin_unlock_irqrestore(&pnet_can_private[minor].data_lock,iflags);
ret:
	return ret;
}

/*
 * low-level api-can-close
 * parameters:
 *	minor .. can minor (same as chrdev-minor)
 *	accmod .. like file ACCMODE (O_RDWR, O_RDONLY, O_WRONLY)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can__close(int minor, int accmode, int wait)
{
	unsigned long iflags;

	if (minor>=PNET_CAN_DEV_N)
		return -EINVAL;

	dbg("minor: %i",minor);

	/* check if open */
	if((!pnet_can_private[minor].write_count)&&(!pnet_can_private[minor].read_count))
		return 0;

	spin_lock_irqsave(&pnet_can_private[minor].data_lock,iflags);

	if (accmode==O_RDWR || accmode==O_WRONLY) 
		if (pnet_can_private[minor].write_count>0) pnet_can_private[minor].write_count--;
	if (accmode==O_RDWR || accmode==O_RDONLY) 
		if (pnet_can_private[minor].read_count>0) pnet_can_private[minor].read_count--;

	spin_unlock_irqrestore(&pnet_can_private[minor].data_lock,iflags);

	/* reset pnet_can */
	pnet_can_reset(&pnet_can_private[minor],wait);

	/* close pnet_rdev */
	pnet_rdev_close(pnet_can_private[minor].pnet_rdev);


	/* ok */
	return 0;
}
/*
 **************************************************************************************************
 * End: API-Helper - Functions
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Kernel API driver only - Functions
 **************************************************************************************************
 */

/*
 * open can controller
 * parameters:
 * 	minor .. pnet_can-minor
 * 	pnet_can .. pnet_can instance (set by open)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_open(int minor, struct pnet_can_private **pnet_can, int wait)
{
	dbg("");

	/* raw open */
	return pnet_can__open(minor, pnet_can, O_RDWR, wait);
}
EXPORT_SYMBOL(pnet_can_open);

/*
 * close can controller
 * parameters:
 * 	minor .. pnet_can-minor
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_close(int minor, int wait)
{
	dbg("");

	/* raw close */
	return pnet_can__close(minor, O_RDWR, wait);
}
EXPORT_SYMBOL(pnet_can_close);

/*
 **************************************************************************************************
 * End: Kernel API driver only - Functions
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Linux-Driver-Functions
 **************************************************************************************************
 */
static int pnet_can_lin_open(struct inode *inode, struct file *file)
{
	int minor,wait;
	int ret=0;
	struct pnet_can_private *pnet_can=NULL;

	minor=iminor(inode);
	if (minor>=PNET_CAN_DEV_N)
		return -EINVAL;

	dbg("minor: %i",minor);

	if (down_interruptible(&pnet_can_private[minor].reg_sem)) {
		ret=-ERESTARTSYS;
		goto __ret;
	}
	if (down_interruptible(&pnet_can_private[minor].fifo_rsem)) {
		ret=-ERESTARTSYS;
		goto __ret_up_reg_sem;
	}
	if (down_interruptible(&pnet_can_private[minor].fifo_wsem)) {
		ret=-ERESTARTSYS;
		goto __ret_up_fifo_rsem;
	}

	/* get wait-status */
	wait=(file->f_flags & O_NONBLOCK) ? 0 : PNET_RDEV_LIN_DEFAULT_TIMEOUT;

	/* open */
	ret=pnet_can__open(
		minor, 
		&pnet_can, 
		(file->f_flags&O_ACCMODE), 
		wait
	);

	up(&pnet_can_private[minor].fifo_wsem);
__ret_up_fifo_rsem:
	up(&pnet_can_private[minor].fifo_rsem);
__ret_up_reg_sem:
	up(&pnet_can_private[minor].reg_sem);
__ret:
	return ret;
}

static int pnet_can_lin_close(struct inode *inode, struct file *file)
{
	int minor,wait;
	int ret=0;

	minor=iminor(inode);
	if (minor>=PNET_CAN_DEV_N)
		return -EINVAL;

	dbg("minor: %i",minor);

	down(&pnet_can_private[minor].reg_sem);
	down(&pnet_can_private[minor].fifo_rsem);
	down(&pnet_can_private[minor].fifo_wsem);

	/* get wait-status */
	wait=(file->f_flags & O_NONBLOCK) ? 0 : PNET_RDEV_LIN_DEFAULT_TIMEOUT;

	/* close */
	ret=pnet_can__close(
		minor, 
		(file->f_flags&O_ACCMODE), 
		wait
	);

	up(&pnet_can_private[minor].fifo_wsem);
	up(&pnet_can_private[minor].fifo_rsem);
	up(&pnet_can_private[minor].reg_sem);

	return ret;
}

static int pnet_can_lin_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct pnet_can_private *pnet_can;
	int minor,wait;
	int ret=0;
	int ena,dis;
	struct pnet_can_msg can_msg;
	struct pnet_can_config can_cfg;
	struct pnet_can_mbox_config can_mbox_cfg;

	/* get pnet_rdev struct */
	minor = iminor(file->f_dentry->d_inode);
	if (minor>=PNET_CAN_DEV_N)
		return -EINVAL;

	pnet_can =&pnet_can_private[minor];
	dbg("%i, %i",minor,cmd);

	/* get wait-status */
	wait=(file->f_flags & O_NONBLOCK) ? 0 : PNET_CAN_LIN_DEFAULT_TIMEOUT;

	/* error EPIPE when disconnected */
	if (pnet_can->disconnected)
		return -EPIPE;

	/* parse command */
	switch(cmd)
	{
		case PNET_CAN_IOCTL_CMD_INFO:
			/* check read-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_RDONLY)
				return -EINVAL;

			dbg("PNET_CAN_IOCTL_CMD_INFO: do");

			/* down register semaphore */
			if (down_interruptible(&pnet_can->reg_sem))
				return -ERESTARTSYS;

			/* get remote-info */
			if(pnet_can->connection)
				pnet_can_update_info(pnet_can,wait);

			/* up */
			up(&pnet_can->reg_sem);

			/* put structure */
			ret=copy_to_user (argp, &pnet_can->remote_info, sizeof(struct pnet_can_info));
			if(ret)
				return ret;

			dbg("PNET_CAN_IOCTL_CMD_INFO: version_lo: 0x%X, version_hi: 0x%X, mbox_n: %i, mask_n: %i", 
				pnet_can->remote_info.version_lo,
				pnet_can->remote_info.version_hi,
				pnet_can->remote_info.mbox_n,
				pnet_can->remote_info.mask_n
			);

		break;

		case PNET_CAN_IOCTL_CMD_ENABLE:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameter */
			if (get_user(ena, (int __user *)arg))
				return -EFAULT;

			dbg("PNET_CAN_IOCTL_CMD_ENABLE: ena: %i",ena);

			/* down register semaphore */
			if (down_interruptible(&pnet_can->reg_sem))
				return -ERESTARTSYS;

			/* set enable/disable */
			ret=pnet_can_enable(pnet_can,ena,wait);

			/* up */
			up(&pnet_can->reg_sem);

			if(ret)
				return ret;

			/* save enabled */
			pnet_can->enabled=ena;
		break;

		case PNET_CAN_IOCTL_CMD_CONFIG:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameters */
			ret=copy_from_user (&can_cfg, argp, sizeof(struct pnet_can_config));
			if(ret)
				return ret;

			dbg("PNET_CAN_IOCTL_CMD_CONFIG: bitrate: %i, sample_point: %i, edge_resync_mode: %i, sync_jump_width: %i, byte_order: %i",
				can_cfg.bitrate,
				can_cfg.sample_point,
				can_cfg.edge_resync_mode,
				can_cfg.sync_jump_width,
				can_cfg.byte_order
			);

			/* down register semaphore */
			if (down_interruptible(&pnet_can->reg_sem))
				return -ERESTARTSYS;

			/* configure can */
			ret=pnet_can_config(pnet_can, can_cfg, wait);

			/* up */
			up(&pnet_can->reg_sem);

			if(ret)
				return ret;
		break;


		case PNET_CAN_IOCTL_CMD_MBOX_CONFIG:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameters */
			ret=copy_from_user (&can_mbox_cfg, argp, sizeof(struct pnet_can_mbox_config));
			if(ret)
				return ret;

			dbg("PNET_CAN_IOCTL_CMD_MBOX_CONFIG: mbox: %i, type: 0x%X, message_id: 0x%X, data_len: %i",
				can_mbox_cfg.mbox,
				can_mbox_cfg.type,
				can_mbox_cfg.message_id,
				can_mbox_cfg.data_len
			);

			/* down register semaphore */
			if (down_interruptible(&pnet_can->reg_sem))
				return -ERESTARTSYS;

			/* configure mailbox */
			ret=pnet_can_mbox_config(pnet_can, can_mbox_cfg, wait);

			/* up */
			up(&pnet_can->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_CAN_IOCTL_CMD_MBOX_ENABLE:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameter */
			if (get_user(ena, (int __user *)arg))
				return -EFAULT;

			dbg("PNET_CAN_IOCTL_CMD_MBOX_ENABLE: ena: 0x%X",ena);

			/* down register semaphore */
			if (down_interruptible(&pnet_can->reg_sem))
				return -ERESTARTSYS;

			/* set enable/disable */
			ret=pnet_can_mbox_enable(pnet_can,ena,wait);

			/* up */
			up(&pnet_can->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_CAN_IOCTL_CMD_MBOX_SET_ENABLED:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameter */
			if (get_user(ena, (int __user *)arg))
				return -EFAULT;

			dbg("PNET_CAN_IOCTL_CMD_MBOX_SET_ENABLED: ena: 0x%X",ena);

			/* down register semaphore */
			if (down_interruptible(&pnet_can->reg_sem))
				return -ERESTARTSYS;

			/* set enable/disable */
			ret=pnet_can_mbox_set_enabled(pnet_can,ena,wait);

			/* up */
			up(&pnet_can->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_CAN_IOCTL_CMD_MBOX_SET_DISABLED:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameter */
			if (get_user(dis, (int __user *)arg))
				return -EFAULT;

			dbg("PNET_CAN_IOCTL_CMD_MBOX_SET_DISABLED: dis: 0x%X",dis);

			/* down register semaphore */
			if (down_interruptible(&pnet_can->reg_sem))
				return -ERESTARTSYS;

			/* set enable/disable */
			ret=pnet_can_mbox_set_enabled(pnet_can,dis,wait);

			/* up */
			up(&pnet_can->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_CAN_IOCTL_MSG_SND_ASYNC:
		case PNET_CAN_IOCTL_MSG_SND_SYNC:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get message */
			ret=copy_from_user (&can_msg, argp, sizeof(struct pnet_can_msg));
			if(ret)
				return ret;

			if(cmd==PNET_CAN_IOCTL_MSG_SND_SYNC) {
				dbg("%i PNET_CAN_IOCTL_MSG_SND_SYNC: mbox: %i, message_id: %X, data_len: %i, data: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X",
					minor,
					can_msg.mbox,
					can_msg.message_id,
					can_msg.data_len,
					can_msg.data[0],
					can_msg.data[1],
					can_msg.data[2],
					can_msg.data[3],
					can_msg.data[4],
					can_msg.data[5],
					can_msg.data[6],
					can_msg.data[7]
				);

				/* down fifo write-semaphore */
				if (down_interruptible(&pnet_can->fifo_wsem))
					return -ERESTARTSYS;

				/* send can-msg */
				ret=pnet_can_mbox_msg_snd_sync(pnet_can,can_msg,wait);

				/* up */
				up(&pnet_can->fifo_wsem);
			} else {
				dbg("%i PNET_CAN_IOCTL_MSG_SND_ASYNC: mbox: %i, message_id: %X, data_len: %i, data: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X",
					minor,
					can_msg.mbox,
					can_msg.message_id,
					can_msg.data_len,
					can_msg.data[0],
					can_msg.data[1],
					can_msg.data[2],
					can_msg.data[3],
					can_msg.data[4],
					can_msg.data[5],
					can_msg.data[6],
					can_msg.data[7]
				);

				/* down register semaphore */
				if (down_interruptible(&pnet_can->reg_sem))
					return -ERESTARTSYS;

				/* send can-msg */
				ret=pnet_can_mbox_msg_snd_async(pnet_can,can_msg,wait);

				/* up */
				up(&pnet_can->reg_sem);
			}

			if(ret)
				return ret;
		break;

		case PNET_CAN_IOCTL_MSG_RCV_SYNC:
			/* check read-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_RDONLY)
				return -EINVAL;

			dbg("%i PNET_CAN_IOCTL_MSG_RCV_SYNC: do",minor);

			/* down register read-semaphore */
			if (down_interruptible(&pnet_can->fifo_rsem))
				return -ERESTARTSYS;

			/* receive can-msg */
			ret=pnet_can_mbox_msg_rcv_sync(pnet_can,&can_msg,wait);

			/* up */
			up(&pnet_can->fifo_rsem);

			if(ret)
				return ret;

			dbg("%i PNET_CAN_IOCTL_MSG_RCV_SYNC: mbox: %i, message_id: %X, data_len: %i, data: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X",
				minor,
				can_msg.mbox,
				can_msg.message_id,
				can_msg.data_len,
				can_msg.data[0],
				can_msg.data[1],
				can_msg.data[2],
				can_msg.data[3],
				can_msg.data[4],
				can_msg.data[5],
				can_msg.data[6],
				can_msg.data[7]
			);
				
			/* put structure */
			ret=copy_to_user (argp, &can_msg, sizeof(struct pnet_can_msg));
			if(ret)
				return ret;
		break;

		case PNET_CAN_IOCTL_DATA_SND_ASYNC:
		case PNET_CAN_IOCTL_DATA_SND_SYNC:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get message */
			ret=copy_from_user (&can_msg, argp, sizeof(struct pnet_can_msg));
			if(ret)
				return ret;

			if(cmd==PNET_CAN_IOCTL_DATA_SND_SYNC) {
				dbg("%i PNET_CAN_IOCTL_DATA_SND_SYNC: mbox: %i, data_len: %i, data: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X",
					minor,
					can_msg.mbox,
					can_msg.data_len,
					can_msg.data[0],
					can_msg.data[1],
					can_msg.data[2],
					can_msg.data[3],
					can_msg.data[4],
					can_msg.data[5],
					can_msg.data[6],
					can_msg.data[7]
				);

				/* down fifo write-semaphore */
				if (down_interruptible(&pnet_can->fifo_wsem))
					return -ERESTARTSYS;

				/* send data on mailbox */
				ret=pnet_can_mbox_data_snd_sync(pnet_can,can_msg,wait);

				/* up */
				up(&pnet_can->fifo_wsem);
			} else {
				dbg("%i PNET_CAN_IOCTL_DATA_SND_ASYNC: mbox: %i, data_len: %i, data: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X",
					minor,
					can_msg.mbox,
					can_msg.data_len,
					can_msg.data[0],
					can_msg.data[1],
					can_msg.data[2],
					can_msg.data[3],
					can_msg.data[4],
					can_msg.data[5],
					can_msg.data[6],
					can_msg.data[7]
				);

				/* down register semaphore */
				if (down_interruptible(&pnet_can->reg_sem))
					return -ERESTARTSYS;

				/* send data on mailbox */
				ret=pnet_can_mbox_data_snd_async(pnet_can,can_msg,wait);

				/* up */
				up(&pnet_can->reg_sem);
			}

			if(ret)
				return ret;
		break;

		case PNET_CAN_IOCTL_UPDATE_SND_ASYNC:
		case PNET_CAN_IOCTL_UPDATE_SND_SYNC:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get message */
			ret=copy_from_user (&can_msg, argp, sizeof(struct pnet_can_msg));
			if(ret)
				return ret;

			if(cmd==PNET_CAN_IOCTL_UPDATE_SND_SYNC) {
				dbg("%i PNET_CAN_IOCTL_UPDATE_SND_SYNC: mbox: %i, data: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X",
					minor,
					can_msg.mbox,
					can_msg.data[0],
					can_msg.data[1],
					can_msg.data[2],
					can_msg.data[3],
					can_msg.data[4],
					can_msg.data[5],
					can_msg.data[6],
					can_msg.data[7]
				);

				/* down fifo write-semaphore */
				if (down_interruptible(&pnet_can->fifo_wsem))
					return -ERESTARTSYS;

				/* send data-update for mailbox */
				ret=pnet_can_mbox_update_snd_sync(pnet_can,can_msg,wait);

				/* up */
				up(&pnet_can->fifo_wsem);
			} else {
				dbg("%i PNET_CAN_IOCTL_UPDATE_SND_ASYNC: mbox: %i, data: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X",
					minor,
					can_msg.mbox,
					can_msg.data[0],
					can_msg.data[1],
					can_msg.data[2],
					can_msg.data[3],
					can_msg.data[4],
					can_msg.data[5],
					can_msg.data[6],
					can_msg.data[7]
				);

				/* down register semaphore */
				if (down_interruptible(&pnet_can->reg_sem))
					return -ERESTARTSYS;

				/* send data-update for mailbox */
				ret=pnet_can_mbox_update_snd_async(pnet_can,can_msg,wait);

				/* up */
				up(&pnet_can->reg_sem);
			}

			if(ret)
				return ret;
		break;

		case PNET_CAN_IOCTL_REQUEST_SND_ASYNC:
		case PNET_CAN_IOCTL_REQUEST_SND_SYNC:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get message */
			ret=copy_from_user (&can_msg, argp, sizeof(struct pnet_can_msg));
			if(ret)
				return ret;

			if(cmd==PNET_CAN_IOCTL_REQUEST_SND_SYNC) {
				dbg("%i PNET_CAN_IOCTL_REQUEST_SND_SYNC: mbox: %i",
					minor,
					can_msg.mbox
				);

				/* down fifo write-semaphore */
				if (down_interruptible(&pnet_can->fifo_wsem))
					return -ERESTARTSYS;

				/* send can-request on mailbox */
				ret=pnet_can_mbox_request_snd_sync(pnet_can,can_msg,wait);

				/* up */
				up(&pnet_can->fifo_wsem);
			} else {
				dbg("%i PNET_CAN_IOCTL_REQUEST_SND_ASYNC: mbox: %i",
					minor,
					can_msg.mbox
				);

				/* down register semaphore */
				if (down_interruptible(&pnet_can->reg_sem))
					return -ERESTARTSYS;

				/* send can-request on mailbox */
				ret=pnet_can_mbox_request_snd_async(pnet_can,can_msg,wait);

				/* up */
				up(&pnet_can->reg_sem);
			}

			if(ret)
				return ret;
		break;

		default:
			dbg("invalid command");
			return -ENOTTY;
	}

	return ret;
}

static struct file_operations pnet_can_fops = {
	.owner = THIS_MODULE,
	.ioctl = pnet_can_lin_ioctl,
	.open = pnet_can_lin_open,
	.release = pnet_can_lin_close,
};

static struct class *pnet_can_class;

/*
 **************************************************************************************************
 * End: Linux-Driver-Functions
 **************************************************************************************************
 */



/***********************************************************************
 *  Begin: procfs statistics
 ***********************************************************************/
#ifdef CONFIG_PROC_FS

static int pnet_can_info_show(struct seq_file *m, void *v)
{
	int i,j;
	char *type_str;

	seq_printf(m,"%s\n",PNET_CAN_VERSION_STR);

	for (i=0;i<PNET_CAN_DEV_N;i++) {
		seq_printf(m,"%s%i:\n",DRV_NAME,i);

		down(&pnet_can_private[i].reg_sem);

		/* reset disconnected if first open */
		if( pnet_can_private[i].connection && ( !(pnet_can_private[i].read_count|pnet_can_private[i].write_count) ) ) {
			/* open pnet_rdev for can */
			if(pnet_rdev_open(pnet_can_private[i].pnet_rdev)) {
				up(&pnet_can_private[i].reg_sem);
				continue;
			}
			pnet_can_private[i].disconnected=0;
		}

		seq_printf(m," - Open:\n");
		seq_printf(m,"   - read:                       %i\n",pnet_can_private[i].read_count);
		seq_printf(m,"   - write:                      %i\n",pnet_can_private[i].write_count);
		seq_printf(m," - States:\n");
		seq_printf(m,"   - disconnected:               %i\n",pnet_can_private[i].disconnected);
		seq_printf(m,"   - connection:                 %i\n",pnet_can_private[i].connection);
		seq_printf(m,"   - enabled:                    %i\n",pnet_can_private[i].enabled);

		/* stop output for this port here, if not connected */
		if(!pnet_can_private[i].connection) {
			pnet_rdev_close(pnet_can_private[i].pnet_rdev);
			up(&pnet_can_private[i].reg_sem);
			continue;
		}

		/* update data */
		if(pnet_can_update_info(&pnet_can_private[i],PNET_CAN_LIN_DEFAULT_TIMEOUT)) {
			err("error on update");
			pnet_rdev_close(pnet_can_private[i].pnet_rdev);
			up(&pnet_can_private[i].reg_sem);
			return -1;
		}
		pnet_rdev_close(pnet_can_private[i].pnet_rdev);

		seq_printf(m," - Remote-Info:\n");
		seq_printf(m,"   - remote_lo_version:          0x%.4X\n",pnet_can_private[i].remote_info.version_lo);
		seq_printf(m,"   - remote_hi_version:          0x%.4X\n",pnet_can_private[i].remote_info.version_hi);
		seq_printf(m,"   - number of mboxes:           %i\n",pnet_can_private[i].remote_info.mbox_n);
		seq_printf(m,"   - number of masks:            %i\n",pnet_can_private[i].remote_info.mask_n);
		seq_printf(m," - Configuration:\n");
		seq_printf(m,"   - bitrate:                    %i\n",pnet_can_private[i].config.bitrate);
		seq_printf(m,"   - sample-point:               %i%%\n",pnet_can_private[i].config.sample_point);
		seq_printf(m,"   - edge_resync:                %i\n",pnet_can_private[i].config.edge_resync_mode);
		seq_printf(m,"   - sync_jump_width:            %i\n",pnet_can_private[i].config.sync_jump_width);
		seq_printf(m,"   - byte_order:                 %i\n",pnet_can_private[i].config.byte_order);
		seq_printf(m,"   - masks:\n");
		for(j=0;j<pnet_can_private[i].remote_info.mask_n;j++)
			seq_printf(m,"     - mask[%i]:                 0x%.8X\n",j,pnet_can_private[i].config.mask[j]);
		seq_printf(m,"   - mailboxes enabled:          0x%.5X\n",pnet_can_private[i].mbox_enabled);
		seq_printf(m,"   - mailboxes:\n");
		for(j=0;j<pnet_can_private[i].remote_info.mbox_n;j++) {
			switch(pnet_can_private[i].mbox_config[j].type) {
				case CAN_MBOX_TYPE_TX:
					type_str="CAN_MBOX_TYPE_TX";
				break;
				case CAN_MBOX_TYPE_RX:
					type_str="CAN_MBOX_TYPE_RX";
				break;
				case CAN_MBOX_TYPE_REQUEST:
					type_str="CAN_MBOX_TYPE_REQUEST";
				break;
				case CAN_MBOX_TYPE_REPLY:
					type_str="CAN_MBOX_TYPE_REPLY";
				break;
				default:
					type_str="unknown";
				break;
			}

			seq_printf(m,"     - mbox%i\t%s\t0x%.8X\t%i\n",j,type_str,pnet_can_private[i].mbox_config[j].message_id,pnet_can_private[i].mbox_config[j].data_len);			
		}

#if 0
		seq_printf(m,"   - register_commit_ack_state:  %i\n",pnet_can_private[i].register_commit_ack_state);
		seq_printf(m,"   - flowctrl_state:             %i\n",pnet_can_private[i].flowctrl_state);
		seq_printf(m,"   - remote_commit_received:     %i\n",pnet_can_private[i].remote_commit_received);
		seq_printf(m,"   - remote_space:               %i\n",pnet_can_private[i].remote_space);
		seq_printf(m,"   - bytes_received:             %i\n",pnet_can_private[i].bytes_received);

#ifdef PNET_CAN_DEBUG_STATE_COUNTERS
		seq_printf(m," - Counters:\n");		
		seq_printf(m,"   - sent commits:               %i\n",pnet_can_private[i].sent_commits);
		seq_printf(m,"   - received commit acks:       %i\n",pnet_can_private[i].received_commit_acks);
		seq_printf(m,"   - received commits:           %i\n",pnet_can_private[i].received_commits);
		seq_printf(m,"   - sent commit acks:           %i\n",pnet_can_private[i].sent_commit_acks);
		seq_printf(m,"   - sent_flowctrl:              %i\n",pnet_can_private[i].sent_flowctrl);
		seq_printf(m,"   - received_flowctrl:          %i\n",pnet_can_private[i].received_flowctrl);
#endif
#endif

		pnet_rdev_close(pnet_can_private[i].pnet_rdev);
		up(&pnet_can_private[i].reg_sem);
	}

	return 0;
}

static int pnet_can_info_open( struct inode *inode, struct file *file)
{
	return single_open(file, pnet_can_info_show, NULL);
}

static struct file_operations pnet_can_info_fops = {
	.owner = THIS_MODULE,
	.open = pnet_can_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static struct proc_dir_entry *pnet_can_proc_dir,*pnet_can_proc_info;
static int pnet_can_proc_create(void)
{
	pnet_can_proc_dir=proc_mkdir("driver/"DRV_NAME,NULL);
	pnet_can_proc_info=create_proc_entry("info",S_IRUGO,pnet_can_proc_dir);
	if (pnet_can_proc_info) {
		pnet_can_proc_info->proc_fops = &pnet_can_info_fops;
	}
	return 0;
}
static void pnet_can_proc_remove(void)
{
	if (pnet_can_proc_info) remove_proc_entry("info", pnet_can_proc_dir);
	if (pnet_can_proc_dir) remove_proc_entry("driver/"DRV_NAME,NULL);
}
#endif	

/***********************************************************************
 *  End: procfs statistics
 ***********************************************************************/

/*
 **************************************************************************************************
 * Begin: Module-Stuff
 **************************************************************************************************
 */

static int __init pnet_can_init(void)
{
	int i;
	int ret=0;

	dbg("");
	
	memset(&pnet_can_private, 0, sizeof(struct pnet_can_private)*PNET_CAN_DEV_N);
	logs_init();

	/* init */
	for (i=0;i<PNET_CAN_DEV_N;i++) {
		/* init */
		init_MUTEX(&pnet_can_private[i].reg_sem);
		init_MUTEX(&pnet_can_private[i].fifo_rsem);
		init_MUTEX(&pnet_can_private[i].fifo_wsem);
		init_waitqueue_head(&pnet_can_private[i].open_wait); 
		init_waitqueue_head(&pnet_can_private[i].snd_wait);
		init_waitqueue_head(&pnet_can_private[i].rcv_wait); 
		spin_lock_init(&pnet_can_private[i].data_lock);

		/* request pnet_rdev */
		if(pnet_rdev_request(PNET_CAN_RDEV_MINOR, &pnet_can_private[i].pnet_rdev, &pnet_can_private[i], &pnet_can_rdev_register_callback, &pnet_can_rdev_data_callback))
			goto pnet_can_init_err_pnet_rdev;
	}


	/* register chrdev */
	ret=register_chrdev(DRV_MAJOR,DRV_NAME,&pnet_can_fops);
	if (ret) {
		err("register chrdev failed");
		goto pnet_can_init_err_cdev;
	}

	/* register device class */
	pnet_can_class=class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(pnet_can_class)) {
		err("class_create failed");
		ret = PTR_ERR(pnet_can_class);
		goto pnet_can_init_err_class;
	}

	/* register devices */
	for (i=0;i<PNET_CAN_DEV_N;i++) {
		pnet_can_private[i].device=device_create(pnet_can_class,NULL,MKDEV(DRV_MAJOR,i),NULL,"%s%d",DRV_NAME,i);
		if (IS_ERR(pnet_can_private[i].device)) {
			err("device_create failed");
			ret=PTR_ERR(pnet_can_private[i].device);
			/* unregister devices */
			i--;
			while(i--)
				device_destroy(pnet_can_class,MKDEV(DRV_MAJOR,i));
			goto pnet_can_init_err_class_device;
		}	
	}		

#ifdef CONFIG_PROC_FS
	pnet_can_proc_create();
#endif

	return ret;

pnet_can_init_err_class_device:
	class_destroy(pnet_can_class);
pnet_can_init_err_class:
	unregister_chrdev(DRV_MAJOR,DRV_NAME);
pnet_can_init_err_cdev:
pnet_can_init_err_pnet_rdev:
	return ret;
}

static void __exit pnet_can_exit(void)
{
	int i;

	dbg("");

#ifdef CONFIG_PROC_FS
	pnet_can_proc_remove();
#endif
	for (i=0;i<PNET_CAN_DEV_N;i++)
		device_destroy(pnet_can_class,MKDEV(DRV_MAJOR,i));
	class_destroy(pnet_can_class);
	unregister_chrdev(DRV_MAJOR,DRV_NAME);
}

module_init( pnet_can_init );
module_exit( pnet_can_exit );

MODULE_AUTHOR("Manfred Schlaegl");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PNET can driver over pnet_rdev");

/*
 **************************************************************************************************
 * End: Module-Stuff
 **************************************************************************************************
 */
