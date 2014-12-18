/***********************************************************************
 *
 * @Authors: Manfred Schlägl, Ginzinger electronic systems GmbH
 * @Descr: ge_can driver for pnet
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
 *	2010-KW13 manfred.schlaegl@gmx.at
 *		* increased broadcast buffer-size (20 -> 100 messages)
 *	2010-KW12 manfred.schlaegl@gmx.at
 *		* change in design
 *		* support for
 *			* rpc send
 *			* broadcast receive
 *			* sequence send
 *	2010-KW11 manfred.schlaegl@gmx.at
 *		* port to linux / error-corrections
 *	2010KW07 - manfred.schlaegl
 *		* detailed concept (buildable but untested)
 *	2010KW06 - manfred.schlaegl
 *		* begin/concept
 ***********************************************************************/
/***********************************************************************
 *  @TODO:
 *		* validate concept
 *		* module<->module communication on single node
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
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include "pnet_can.h"
#include "pnet_ge_can.h"

/*
 **************************************************************************************************
 * Begin: Basic Definitions
 **************************************************************************************************
 */

#define DRV_MAJOR  										248
#define DRV_NAME										"pnet_ge_can"
#define PNET_GE_CAN_DEV_N								1

#define PNET_GE_CAN_TIMEOUT								(2*HZ)
#define PNET_GE_CAN_RX_TIMEOUT							(3*HZ)

/* enable debug output */
#ifdef CONFIG_PNET_GE_CAN_DEBUG
#define DEBUG
#endif
//#define DEBUG



/*
 * message_id:
 *
 * desc | reserved
 * bits | 28 27
 *
 * desc | FLAG: broadcast (0 .. sequence; 1 .. broadcast)
 * bits | 26
 *
 * desc | FLAG: SEQUENCE-START (0 .. sequence running; 1 .. sequence start)
 * bits | 25
 *
 * desc | node_id
 * bits | 24 23 22 21 20 19
 *
 * desc | modul_id
 * bits | 18 17 16 15 14 13
 *
 * desc | module_instance_id
 * bits | 12 11 10
 *
 * desc | parameter_id
 * bits | 09 08 07 06 05 04 03 02 01 00
 *
 * (note: maskbit=1 .. don't care; maskbit=0 .. care)
 *
 * rpc-receiver:
 *	* desc:					standard-frame
 *	* mask:					1 1 1 1 1 1 1 1 1 1 1 (AMI=0)
 *	* rx-mbox-message_id:	X X X X X X X X X X X (IDE=0)
 * broadcast receiver
 *	* desc:					broadcast=1; sequence_start=0; node_id, module_id, module_instance_id, parameter_id = sender-data
 * 	* mask: 				0 0 0 | 0 | 1 | 1 1 1 1 1 1 | 1 1 1 1 1 1 | 1 1 1 | 1 1 1 1 1 1 1 1 1 1 (AMI=0)
 *	* rx-mbox-message_id:	0 0 0 | 1 | X | X X X X X X | X X X X X X | X X X | X X X X X X X X X X (IDE=1)
 * seqence receiver
 *	* desc:					broadcast=0; sequence_start=X; node_id, module_id, module_instance_id, parameter_id = sender-data
 * 	* mask: 				0 0 0 | 0 | 1 | 0 0 0 0 0 0 | 1 1 1 1 1 1 | 1 1 1 | 1 1 1 1 1 1 1 1 1 1 (AMI=0)
 *	* rx-mbox-message_id:	0 0 0 | 0 | X | node-id     | X X X X X X | X X X | X X X X X X X X X X (IDE=1)
 */
#define PNET_GE_CAN_RES_MASK 					0x18000000
#define PNET_GE_CAN_RES_SHIFT 					27
#define PNET_GE_CAN_FLAG_BC_MASK 				0x04000000
#define PNET_GE_CAN_FLAG_BC_SHIFT				26
#define PNET_GE_CAN_FLAG_SS_MASK 				0x02000000
#define PNET_GE_CAN_FLAG_SS_SHIFT				25
#define PNET_GE_CAN_NODE_ID_MASK 				0x01F80000
#define PNET_GE_CAN_NODE_ID_SHIFT 				19
#define PNET_GE_CAN_MODULE_ID_MASK 				0x0007E000
#define PNET_GE_CAN_MODULE_ID_SHIFT 			13
#define PNET_GE_CAN_MODULE_INSTANCE_ID_MASK 	0x00001C00
#define PNET_GE_CAN_MODULE_INSTANCE_ID_SHIFT 	10
#define PNET_GE_CAN_PARAMETER_ID_MASK 			0x000003FF
#define PNET_GE_CAN_PARAMETER_ID_SHIFT 			0

/*
 **************************************************************************************************
 * End: Basic Definitions
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

/*
 **************************************************************************************************
 * BEGIN: Configuration
 **************************************************************************************************
 */

/* id of this node */
#define PNET_GE_CAN_NODE_ID			0x1

/*
 **************************************************************************************************
 * END: Configuration
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * BEGIN: Driver-data
 **************************************************************************************************
 */

/*
 * message receive buffer
 */
#define PNET_GE_CAN_MESSAGE_BUFFER_SIZE 100
struct pnet_ge_can_msg_buf {
	struct pnet_can_msg buf[PNET_GE_CAN_MESSAGE_BUFFER_SIZE];
	unsigned char len;
	unsigned char iptr;
	unsigned char optr;
	wait_queue_head_t wq;
	spinlock_t lock;
};

/*
 * init message-buffer
 * parameters:
 *	msg_buf .. msg_buf-object
 * return: <0 .. error; 0 .. ok
 */
static int __init pnet_ge_can_msg_buf_init(struct pnet_ge_can_msg_buf *msg_buf)
{
	/* init struct */
	memset(msg_buf,0,sizeof(struct pnet_ge_can_msg_buf));
	init_waitqueue_head(&msg_buf->wq); 
	spin_lock_init(&msg_buf->lock);

	return 0;
}


/*
 * rpc_snd 
 */
#define PNET_GE_CAN_RPC_SND_FLAG_REPLY_RECEIVED	(1<<15)
#define PNET_GE_CAN_RPC_SND_RX_MBOX					0
#define PNET_GE_CAN_RPC_SND_TX_MBOX					1
#define PNET_GE_CAN_RPC_SND_DEFAULT_TIMEOUT			1000	/* ms */

struct pnet_ge_can_rpc_snd {
	unsigned short reply_msg_id;	/* reply-message_id and reply-received flag */
	can_data reply_data;			/* reply-message-data */
	wait_queue_head_t wq;			/* waitqueue for pnet_ge_can_rpc_snd rx */
	struct semaphore sem;			/* rpc_snd-semaphore */

	/* counters */
	unsigned int cnt_tx_err;
	unsigned int cnt_rx_timeout;
	unsigned int cnt_rx_err;
	unsigned int cnt_rpc_ok;
};

/*
 * init rpc_snd
 * parameters:
 *	rpc_snd .. rpc_snd-object
 * return: <0 .. error; 0 .. ok
 */
static int __init pnet_ge_can_rpc_snd_init(struct pnet_ge_can_rpc_snd *rpc_snd)
{
	/* init struct */
	rpc_snd->reply_msg_id=0;
	init_MUTEX(&rpc_snd->sem);
	init_waitqueue_head(&rpc_snd->wq);

	/* reset counters */
	rpc_snd->cnt_tx_err=0;
	rpc_snd->cnt_rx_timeout=0;
	rpc_snd->cnt_rx_err=0;
	rpc_snd->cnt_rpc_ok=0;

	return 0;
}


/*
 * broadcast_rcv
 */
#define PNET_GE_CAN_BROADCAST_RCV_RX_MBOX			3
#define PNET_GE_CAN_BROADCAST_SND_TX_MBOX			4
#define PNET_GE_CAN_BROADCAST_RCV_DEFAULT_TIMEOUT	1000	/* ms */

struct pnet_ge_can_broadcast_rcv {
	struct pnet_ge_can_msg_buf msg_buf;	/* broadcast_rcv rx message buffer */
	struct semaphore sem;				/* broadcast_rcv-semaphore */

	/* counters */
	unsigned int cnt_rx_timeout;
	unsigned int cnt_rx_err;
	unsigned int cnt_broadcast_ok;
};

/*
 * init broadcast_rcv
 * parameters:
 *	rpc_snd .. broadcast_rcv-object
 * return: <0 .. error; 0 .. ok
 */
static int __init pnet_ge_can_broadcast_rcv_init(struct pnet_ge_can_broadcast_rcv *broadcast_rcv)
{
	/* init struct */
	init_MUTEX(&broadcast_rcv->sem);
	pnet_ge_can_msg_buf_init(&broadcast_rcv->msg_buf);

	/* reset counters */
	broadcast_rcv->cnt_rx_err=0;
	broadcast_rcv->cnt_rx_timeout=0;
	broadcast_rcv->cnt_broadcast_ok=0;

	return 0;
}


/*
 * sequence_snd
 */

#define PNET_GE_CAN_SEQUENCE_SND_RX_MBOX			6
#define PNET_GE_CAN_SEQUENCE_SND_TX_MBOX			7
#define PNET_GE_CAN_SEQUENCE_SND_DEFAULT_TIMEOUT	1000	/* ms */

struct pnet_ge_can_sequence_snd {
	wait_queue_head_t wq;					/* waitqueue for pnet_ge_can_sequence_snd rx */
	struct semaphore sem;					/* sequence_snd-semaphore */
	unsigned char reply_received;			/* reply_received-flag */
	struct pnet_can_msg reply_msg;		/* received reply-message */

	/* counters */
	unsigned int cnt_tx_err;
	unsigned int cnt_rx_timeout;
	unsigned int cnt_rx_err;
	unsigned int cnt_sequence_ok;
};

/*
 * init sequence_snd
 * parameters:
 *	rpc_snd .. sequence_snd-object
 * return: <0 .. error; 0 .. ok
 */
static int __init pnet_ge_can_sequence_snd_init(struct pnet_ge_can_sequence_snd *sequence_snd)
{
	/* init struct */
	init_MUTEX(&sequence_snd->sem);
	init_waitqueue_head(&sequence_snd->wq); 
	sequence_snd->reply_received=0;

	/* reset counters */
	sequence_snd->cnt_tx_err=0;
	sequence_snd->cnt_rx_err=0;
	sequence_snd->cnt_rx_timeout=0;
	sequence_snd->cnt_sequence_ok=0;

	return 0;
}




/*
 * pnet_ge_can
 */

typedef struct pnet_ge_can_private {

	/* GENERAL */
	struct pnet_can_private *pnet_can;

	/* driver-stat */
	unsigned int open_count;		/* open-counter */
	struct device *device;

	/* locking */
	struct semaphore sem;					/* driver semaphore */

	/* CAN RECEIVE */

	/* can configuration */
	struct pnet_can_config can_cfg;
	struct pnet_can_mbox_config can_mbox_cfg[PNET_CAN_MAILBOX_MAX];
	unsigned int can_mbox_ena;

	struct task_struct *msg_rcv_thread;		/* receiver thread */

	unsigned int cnt_pnet_can_err;
	unsigned int cnt_can_rcv_err;
	unsigned int cnt_can_rcv_unknown;
	unsigned int cnt_can_rcv_ok;

	/* RPC_SND */
	struct pnet_ge_can_rpc_snd rpc_snd;

	/* BROADCAST_RCV */
	struct pnet_ge_can_broadcast_rcv broadcast_rcv;

	/* SEQENCE_SND */
	struct pnet_ge_can_sequence_snd sequence_snd;
} pnet_ge_can_private_t;

static struct pnet_ge_can_private pnet_ge_can_private[PNET_GE_CAN_DEV_N];

/*
 **************************************************************************************************
 * END: Driver-data
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * BEGIN: Message-Buffer
 **************************************************************************************************
 */

/*
 * add ge_can-message to receiver buffer
 * overwrite existing elements, if full
 * parameters:
 *	msg_buf .. message-buffer
 *	msg .. can-message
 * return: 0 .. ok; 1 .. overwritten
 */
int pnet_ge_can_msg_buf_add(struct pnet_ge_can_msg_buf *msg_buf, struct pnet_can_msg msg)
{
	int ret;
	unsigned long iflags;

	dbg("");

	spin_lock_irqsave(&msg_buf->lock,iflags);

	/* fill until full; then overwrite */
	if(msg_buf->len<PNET_GE_CAN_MESSAGE_BUFFER_SIZE) {
		/* fill */
		msg_buf->len++;
		ret=0;
	} else {
		/* overwrite */
		ret=1;
	}

	msg_buf->buf[msg_buf->iptr].message_id=msg.message_id;
	can_data_cp(msg_buf->buf[msg_buf->iptr].data,msg.data);
	msg_buf->buf[msg_buf->iptr].data_len=msg.data_len;

	msg_buf->iptr++;
	if(msg_buf->iptr==PNET_GE_CAN_MESSAGE_BUFFER_SIZE)
		msg_buf->iptr=0;

	spin_unlock_irqrestore(&msg_buf->lock,iflags);

	/* wakeup buffer_get */
	wake_up(&msg_buf->wq);

	/* ok */
	return ret;
}

/*
 * get ge_can message from receiver buffer
 * parameters:
 *	msg_buf .. message-buffer
 *	msg .. can-message
 *	timeout .. timeout-value (jiffies)
 * return: 0 .. nothing read; 1 .. message read
 */
int pnet_ge_can_msg_buf_get(struct pnet_ge_can_msg_buf *msg_buf, struct pnet_can_msg *msg, int timeout)
{
	int ret;
	unsigned long iflags;

	dbg("");

	/* wait for data, or timeout */
	dbg("wait");
	if (!wait_event_timeout(msg_buf->wq,
		msg_buf->len,
		timeout)
	)
		return 0;
	dbg("msg received");

	spin_lock_irqsave(&msg_buf->lock,iflags);

	if(!msg_buf->len) {
		ret=0;
	} else {
		msg->message_id=msg_buf->buf[msg_buf->optr].message_id;
		can_data_cp(msg->data,msg_buf->buf[msg_buf->optr].data);
		msg->data_len=msg_buf->buf[msg_buf->optr].data_len;

		msg_buf->optr++;
		if(msg_buf->optr==PNET_GE_CAN_MESSAGE_BUFFER_SIZE)
			msg_buf->optr=0;
		msg_buf->len--;
		ret=1;
	}

	spin_unlock_irqrestore(&msg_buf->lock,iflags);

	/* ok */
	return ret;
}

/*
 **************************************************************************************************
 * END: Message-Buffer
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * BEGIN: can receiver thread
 **************************************************************************************************
 */

int pnet_ge_can_msg_rcv_thread(void *data)
{
	int i,ret;
	int minor=(int)data;
	struct pnet_ge_can_private *ge_can=&pnet_ge_can_private[minor];
	struct pnet_can_msg can_msg;

	dbg("start: minor=%i ge_can=0x%X",minor,(unsigned int)ge_can);

	down(&ge_can->sem);

	while(!kthread_should_stop()) {
		dbg("can-open");

		/* open */
		ret=pnet_can_open(minor,&ge_can->pnet_can,PNET_GE_CAN_TIMEOUT);
		if(ret<0) {
			ge_can->cnt_pnet_can_err++;
			err("error open pnet_can%i",minor);
			schedule_timeout(PNET_GE_CAN_TIMEOUT);
			continue;
		}

		/* config can */
		pnet_can_config(ge_can->pnet_can,ge_can->can_cfg,PNET_GE_CAN_TIMEOUT);

		/* configure mailboxes */
		for(i=0;i<PNET_CAN_MAILBOX_MAX;i++) {
			if(ge_can->can_mbox_cfg[i].mbox!=PNET_CAN_MAILBOX_MAX) {
				/* valid -> config */
				pnet_can_mbox_config(ge_can->pnet_can,ge_can->can_mbox_cfg[i],PNET_GE_CAN_TIMEOUT);
			}
		}

		/* enable mbox */
		pnet_can_mbox_enable(ge_can->pnet_can,ge_can->can_mbox_ena,PNET_GE_CAN_TIMEOUT);

		/* enable can */
		if(pnet_can_enable(ge_can->pnet_can,1,PNET_GE_CAN_TIMEOUT)<0) {
			ge_can->cnt_pnet_can_err++;
			err("pnet_can-error -> retry");
			pnet_can_close(minor,PNET_GE_CAN_TIMEOUT);
			/* retry */
			schedule_timeout(PNET_GE_CAN_TIMEOUT);
			continue;
		}

		up(&ge_can->sem);

		/* do receive */
		while(!kthread_should_stop()) {
			ret=pnet_can_mbox_msg_rcv_sync(ge_can->pnet_can, &can_msg, PNET_GE_CAN_RX_TIMEOUT);
			if(ret==-EAGAIN) {
				/* try again */
				continue;
			}
			else if(ret<0) {
				ge_can->cnt_can_rcv_err++;
				break;
			}

			dbg("can-msg received(mbox=%i, data_len=%i, message_id=0x%X)", can_msg.mbox, can_msg.data_len, can_msg.message_id);

			/* received */
			ge_can->cnt_can_rcv_ok++;

			if(can_msg.mbox==PNET_GE_CAN_RPC_SND_RX_MBOX) {
				/* rpc-reply received */
				dbg("rpc-reply received");
				if(ge_can->rpc_snd.reply_msg_id&PNET_GE_CAN_RPC_SND_FLAG_REPLY_RECEIVED) {
					/* other reply buffered -> ignore */
					dbg("rpc reply waiting -> ignore");
				} else {
					/* new reply -> wakeup */
					dbg("rpc reply ok -> set data & wakup");
					can_data_cp(ge_can->rpc_snd.reply_data,can_msg.data);
					ge_can->rpc_snd.reply_msg_id=PNET_GE_CAN_RPC_SND_FLAG_REPLY_RECEIVED|can_msg.message_id;
					wake_up(&ge_can->rpc_snd.wq);
				}

			} else if(can_msg.mbox==PNET_GE_CAN_BROADCAST_RCV_RX_MBOX) {
				dbg("broadcast received -> add message to ringbuffer and wakeup");
				pnet_ge_can_msg_buf_add(&ge_can->broadcast_rcv.msg_buf, can_msg);

			} else if(can_msg.mbox==PNET_GE_CAN_SEQUENCE_SND_RX_MBOX) {
				/* sequence reply received */
				if(ge_can->sequence_snd.reply_received) {
					/* other reply buffered -> ignore */
					dbg("sequence reply waiting -> ignore");
				} else {
					/* new reply -> wakeup */
					dbg("sequence reply ok -> set data & wakup");
					ge_can->sequence_snd.reply_msg.message_id=can_msg.message_id;
					ge_can->sequence_snd.reply_msg.data_len=can_msg.data_len;
					can_data_cp(ge_can->sequence_snd.reply_msg.data,can_msg.data);
					ge_can->sequence_snd.reply_received=1;
					wake_up(&ge_can->sequence_snd.wq);
				}

			} else {
				dbg("unknown message received");
				ge_can->cnt_can_rcv_unknown++;
			}
		}

		down(&ge_can->sem);

		dbg("can-close");

		/* close */
		pnet_can_close(minor,PNET_GE_CAN_TIMEOUT);
	}

	/* close */
	pnet_can_close(minor,PNET_GE_CAN_TIMEOUT);

	up(&ge_can->sem);

	dbg("stop: minor=%i ge_can=0x%X",minor,(unsigned int)ge_can);

	return 0;
}

/* **************************************************************************************************
 * END: can receiver thread
 **************************************************************************************************
 */


/*
 **************************************************************************************************
 * BEGIN: KERNEL-API
 **************************************************************************************************
 */

/*
 * send rpc
 * parameters:
 *	ge_can .. ge-can structure
 *	ge_can_rpc .. rpc to send
 * return: <0 .. error, 0 .. ok
 */
int pnet_ge_can_rpc_snd(struct pnet_ge_can_private *ge_can, struct pnet_ge_can_rpc_data *ge_can_rpc)
{
	int ret=0;
	int timeout;
	struct pnet_can_msg can_msg;

	dbg("msg_id=0x%X",ge_can_rpc->message_id);

	if(!ge_can->pnet_can)
		return -EBUSY;

	/* set timeout */
	if(!ge_can_rpc->timeout) {
		ge_can_rpc->timeout=PNET_GE_CAN_RPC_SND_DEFAULT_TIMEOUT;
		dbg("set default-timeout: %ims",ge_can_rpc->timeout);
	} else {
		dbg("user-timeout: %ims",ge_can_rpc->timeout);
	}
	timeout=msecs_to_jiffies(ge_can_rpc->timeout);

	down(&ge_can->rpc_snd.sem);

	/* reset */
	ge_can->rpc_snd.reply_msg_id=0;

	/* send rpc_snd */
	can_msg.message_id=ge_can_rpc->message_id&=~PNET_CAN_MBOX_CONFIG_IDE;
	can_msg.data_len=8;
	can_data_cp(can_msg.data,ge_can_rpc->data);
	can_msg.mbox=PNET_GE_CAN_RPC_SND_TX_MBOX;

	dbg("send rpc_snd");
	ret=pnet_can_mbox_msg_snd_sync(ge_can->pnet_can, can_msg, PNET_GE_CAN_TIMEOUT);
	if(ret) {
		ge_can->rpc_snd.cnt_tx_err++;
		goto __ret;
	}

	dbg("wait for reply");
	if (!wait_event_timeout(ge_can->rpc_snd.wq,
		(ge_can->rpc_snd.reply_msg_id&PNET_GE_CAN_RPC_SND_FLAG_REPLY_RECEIVED),
		timeout)
	) {
		dbg("timeout");
		ge_can->rpc_snd.cnt_rx_timeout++;
		ret=-ETIME;
		goto __ret;
	}
	/* remove flag */
	ge_can->rpc_snd.reply_msg_id&=~PNET_GE_CAN_RPC_SND_FLAG_REPLY_RECEIVED;
	dbg("reply received");

	/* check message_id */
	if(ge_can->rpc_snd.reply_msg_id!=ge_can_rpc->message_id) {
		ge_can->rpc_snd.cnt_rx_err++;
		ret=-EPROTO;
		goto __ret;
	}

	/* update cnt */
	ge_can->rpc_snd.cnt_rpc_ok++;

	/* copy data */
	can_data_cp(ge_can_rpc->data,ge_can->rpc_snd.reply_data);

__ret:
	/* reset */
	ge_can->rpc_snd.reply_msg_id=0;

	up(&ge_can->rpc_snd.sem);

	return ret;
}
EXPORT_SYMBOL(pnet_ge_can_rpc_snd);

/*
 * receive broadcast
 * parameters:
 *	ge_can .. ge-can structure
 *	ge_can_bc .. pointer to save broadcast-data
 * return: <0 .. error, 0 .. ok
 */
int pnet_ge_can_broadcast_rcv(struct pnet_ge_can_private *ge_can, struct pnet_ge_can_sequence *ge_can_bc)
{
	int ret=0;
	int timeout;
	struct pnet_can_msg can_msg;

	/* check len > 0 */
	if(ge_can_bc->hdr.len<1)
		return -EINVAL;

	/* set timeout */
	if(!ge_can_bc->hdr.timeout) {
		ge_can_bc->hdr.timeout=PNET_GE_CAN_BROADCAST_RCV_DEFAULT_TIMEOUT;
		dbg("set default-timeout: %ims",ge_can_bc->hdr.timeout);
	} else {
		dbg("user-timeout: %ims",ge_can_bc->hdr.timeout);
	}
	timeout=msecs_to_jiffies(ge_can_bc->hdr.timeout);

	down(&ge_can->broadcast_rcv.sem);

	dbg("wait for broadcast");

	if(pnet_ge_can_msg_buf_get(&ge_can->broadcast_rcv.msg_buf, &can_msg, timeout)) {
		if(can_msg.data_len<1) {
			/* invalid data-len */
			err("protocol-error: wrong data-len (rx=%i < 1)",
				can_msg.data_len
			);
			ge_can->broadcast_rcv.cnt_rx_err++;
			ret=-EPROTO;
		} else if(can_msg.data[0]) {
			/* invalid seq-data */
			err("protocol-error: wrong seq-data (rx=%i != 0)",
				can_msg.data[0]
			);
			ge_can->broadcast_rcv.cnt_rx_err++;
			ret=-EPROTO;
		} else {
			/* received -> parse message */
			dbg("broadcast received");
			ge_can_bc->hdr.node_id=(can_msg.message_id&PNET_GE_CAN_NODE_ID_MASK)>>PNET_GE_CAN_NODE_ID_SHIFT;
			ge_can_bc->hdr.module_id=(can_msg.message_id&PNET_GE_CAN_MODULE_ID_MASK)>>PNET_GE_CAN_MODULE_ID_SHIFT;
			ge_can_bc->hdr.module_instance_id=(can_msg.message_id&PNET_GE_CAN_MODULE_INSTANCE_ID_MASK)>>PNET_GE_CAN_MODULE_INSTANCE_ID_SHIFT;
			ge_can_bc->hdr.len=1;
			ge_can_bc->data[0].parameter_id=(can_msg.message_id&PNET_GE_CAN_PARAMETER_ID_MASK)>>PNET_GE_CAN_PARAMETER_ID_SHIFT;
			ge_can_bc->data[0].dlen=can_msg.data_len-1;
			ge_can_bc->data[0].data[0]=can_msg.data[1];
			ge_can_bc->data[0].data[1]=can_msg.data[2];
			ge_can_bc->data[0].data[2]=can_msg.data[3];
			ge_can_bc->data[0].data[3]=can_msg.data[4];
			ge_can_bc->data[0].data[4]=can_msg.data[5];
			ge_can_bc->data[0].data[5]=can_msg.data[6];
			ge_can_bc->data[0].data[6]=can_msg.data[7];

			/* ok */
			ge_can->broadcast_rcv.cnt_broadcast_ok++;
			ret=0;
		}
	} else {
		/* timeout */
		dbg("timeout");
		ge_can->broadcast_rcv.cnt_rx_timeout++;
		ret=-ETIME;
	}

	up(&ge_can->broadcast_rcv.sem);

	return ret;
}
EXPORT_SYMBOL(pnet_ge_can_broadcast_rcv);

/*
 * send sequence
 * parameters:
 *	ge_can .. ge-can structure
 *	ge_can_seq .. sequence to send
 * return: <0 .. error; 0 .. ok
 */
int pnet_ge_can_sequence_snd(struct pnet_ge_can_private *ge_can, struct pnet_ge_can_sequence *ge_can_sequence)
{
	int ret=0;
	int timeout;
	unsigned short seq_msg_cnt;
	struct pnet_can_msg can_msg;

	if(!ge_can->pnet_can)
		return -EBUSY;

	/* check len > 0 */
	if(ge_can_sequence->hdr.len==0)
		return -EINVAL;

	/* set timeout */
	if(!ge_can_sequence->hdr.timeout) {
		ge_can_sequence->hdr.timeout=PNET_GE_CAN_SEQUENCE_SND_DEFAULT_TIMEOUT;
		dbg("set default-timeout: %ims",ge_can_sequence->hdr.timeout);
	} else {
		dbg("user-timeout: %ims",ge_can_sequence->hdr.timeout);
	}
	timeout=msecs_to_jiffies(ge_can_sequence->hdr.timeout);

	down(&ge_can->sequence_snd.sem);

	dbg("send sequence");

	for(seq_msg_cnt=0;seq_msg_cnt<ge_can_sequence->hdr.len;seq_msg_cnt++) {
		/* create packet */

		/* create message_id */
		can_msg.message_id=
			PNET_CAN_MBOX_CONFIG_IDE 																										|
			((ge_can_sequence->hdr.node_id<<PNET_GE_CAN_NODE_ID_SHIFT)&PNET_GE_CAN_NODE_ID_MASK)									|
			((ge_can_sequence->hdr.module_id<<PNET_GE_CAN_MODULE_ID_SHIFT)&PNET_GE_CAN_MODULE_ID_MASK)								|
			((ge_can_sequence->hdr.module_instance_id<<PNET_GE_CAN_MODULE_INSTANCE_ID_SHIFT)&PNET_GE_CAN_MODULE_INSTANCE_ID_MASK)	|
			((ge_can_sequence->data[seq_msg_cnt].parameter_id<<PNET_GE_CAN_PARAMETER_ID_SHIFT)&PNET_GE_CAN_PARAMETER_ID_MASK);

		/* check & set datalen */
		if(ge_can_sequence->data[seq_msg_cnt].dlen>7) {
			err("invalid data-len: %i",ge_can_sequence->data[seq_msg_cnt].dlen);
			ret=-EINVAL;
			goto __ret;
		}
		can_msg.data_len=ge_can_sequence->data[seq_msg_cnt].dlen+1;

		/* set seq_nr */
		can_msg.data[0]=ge_can_sequence->hdr.len-seq_msg_cnt-1;

		/* set data */
		can_msg.data[1]=ge_can_sequence->data[seq_msg_cnt].data[0];
		can_msg.data[2]=ge_can_sequence->data[seq_msg_cnt].data[1];
		can_msg.data[3]=ge_can_sequence->data[seq_msg_cnt].data[2];
		can_msg.data[4]=ge_can_sequence->data[seq_msg_cnt].data[3];
		can_msg.data[5]=ge_can_sequence->data[seq_msg_cnt].data[4];
		can_msg.data[6]=ge_can_sequence->data[seq_msg_cnt].data[5];
		can_msg.data[7]=ge_can_sequence->data[seq_msg_cnt].data[6];

		if(seq_msg_cnt==0) {
			dbg("configure receiver");
			/* first msessage in sequence -> send start-flag */
			can_msg.message_id|=(1<<PNET_GE_CAN_FLAG_SS_SHIFT);

			/* configure receiver for answer */
			ge_can->can_mbox_cfg[PNET_GE_CAN_SEQUENCE_SND_RX_MBOX].message_id=PNET_CAN_MBOX_CONFIG_AME|can_msg.message_id;
			ret=pnet_can_mbox_config(ge_can->pnet_can,ge_can->can_mbox_cfg[PNET_GE_CAN_SEQUENCE_SND_RX_MBOX],PNET_GE_CAN_TIMEOUT);
			if(ret<0) {
				ge_can->cnt_pnet_can_err++;
				err("pnet_can-error");
				goto __ret;
			}

			/* enable receiver for answer */
			ge_can->can_mbox_ena|=(1<<PNET_GE_CAN_SEQUENCE_SND_RX_MBOX);
			ret=pnet_can_mbox_enable(ge_can->pnet_can,ge_can->can_mbox_ena,PNET_GE_CAN_TIMEOUT);
			if(ret<0) {
				ge_can->cnt_pnet_can_err++;
				err("pnet_can-error");
				goto __ret;
			}
		}

		/* reset receiver */
		ge_can->sequence_snd.reply_received=0;

		/* send sequence part */
		dbg("send sequence-part");
		can_msg.mbox=PNET_GE_CAN_SEQUENCE_SND_TX_MBOX;
		ret=pnet_can_mbox_msg_snd_sync(ge_can->pnet_can, can_msg, PNET_GE_CAN_TIMEOUT);
		if(ret) {
			ge_can->sequence_snd.cnt_tx_err++;
			goto __ret;
		}

		/* wait for answer */
		dbg("wait for reply");
		if (!wait_event_timeout(
			ge_can->sequence_snd.wq,
			ge_can->sequence_snd.reply_received,
			timeout)
		) {
			dbg("timeout");
			ret=-ETIME;
			goto __ret;
		}
		dbg("reply received");

		/* check message_id */
		if(
			(can_msg.message_id&(PNET_GE_CAN_NODE_ID_MASK|PNET_GE_CAN_MODULE_ID_MASK|PNET_GE_CAN_MODULE_INSTANCE_ID_MASK|PNET_GE_CAN_PARAMETER_ID_MASK)) !=
			(ge_can->sequence_snd.reply_msg.message_id&(PNET_GE_CAN_NODE_ID_MASK|PNET_GE_CAN_MODULE_ID_MASK|PNET_GE_CAN_MODULE_INSTANCE_ID_MASK|PNET_GE_CAN_PARAMETER_ID_MASK))
		) {
			err("protocol-error: wrong message-id (tx=0x%X != rx=0x%X)",
				can_msg.message_id&(PNET_GE_CAN_NODE_ID_MASK|PNET_GE_CAN_MODULE_ID_MASK|PNET_GE_CAN_MODULE_INSTANCE_ID_MASK|PNET_GE_CAN_PARAMETER_ID_MASK),
				ge_can->sequence_snd.reply_msg.message_id&(PNET_GE_CAN_NODE_ID_MASK|PNET_GE_CAN_MODULE_ID_MASK|PNET_GE_CAN_MODULE_INSTANCE_ID_MASK|PNET_GE_CAN_PARAMETER_ID_MASK)
			);
			ge_can->sequence_snd.cnt_rx_err++;
			ret=-EPROTO;
			goto __ret;
		}

		/* check dlen */
		if(can_msg.data_len!=ge_can->sequence_snd.reply_msg.data_len) {
			err("protocol-error: wrong data-len (tx=%i != rx=%i)",
				can_msg.data_len,
				ge_can->sequence_snd.reply_msg.data_len
			);
			/* invalid data-len */
			ge_can->sequence_snd.cnt_rx_err++;
			ret=-EPROTO;
			goto __ret;
		}

		/* check sequence-nr */
		if(can_msg.data[0]!=ge_can->sequence_snd.reply_msg.data[0]) {
			err("protocol-error: wrong sequence_nr (tx=%i != rx=%i)",
				(ge_can_sequence->hdr.len-seq_msg_cnt-1),
				ge_can->sequence_snd.reply_msg.data[0]
			);
			/* invalid data-len */
			ge_can->sequence_snd.cnt_rx_err++;
			ret=-EPROTO;
			goto __ret;
		}

		/* received ok */

		/* set data */
		ge_can_sequence->data[seq_msg_cnt].data[0]=ge_can->sequence_snd.reply_msg.data[1];
		ge_can_sequence->data[seq_msg_cnt].data[1]=ge_can->sequence_snd.reply_msg.data[2];
		ge_can_sequence->data[seq_msg_cnt].data[2]=ge_can->sequence_snd.reply_msg.data[3];
		ge_can_sequence->data[seq_msg_cnt].data[3]=ge_can->sequence_snd.reply_msg.data[4];
		ge_can_sequence->data[seq_msg_cnt].data[4]=ge_can->sequence_snd.reply_msg.data[5];
		ge_can_sequence->data[seq_msg_cnt].data[5]=ge_can->sequence_snd.reply_msg.data[6];
		ge_can_sequence->data[seq_msg_cnt].data[6]=ge_can->sequence_snd.reply_msg.data[7];
	}

__ret:

	/* inc ok-counter if no error */
	if(!ret)
		ge_can->sequence_snd.cnt_sequence_ok++;

	/* disable mailbox */
	ge_can->can_mbox_ena&=~(1<<PNET_GE_CAN_SEQUENCE_SND_RX_MBOX);
	if(pnet_can_mbox_enable(ge_can->pnet_can,ge_can->can_mbox_ena,PNET_GE_CAN_TIMEOUT)) {
		ge_can->cnt_pnet_can_err++;
		err("pnet_can-error");
	}

	/* reset receiver */
	ge_can->sequence_snd.reply_received=0;

	up(&ge_can->sequence_snd.sem);

	return ret;
}
EXPORT_SYMBOL(pnet_ge_can_sequence_snd);

/*
 * open ge-can device
 * parameters:
 * 	minor .. pnet_ge_can-minor
 *	ge_can .. returned structure
 * return: 0 .. ok; <0 .. error
 */
int pnet_ge_can_open(int minor, struct pnet_ge_can_private **_ge_can)
{
	int ret=0;
	struct pnet_ge_can_private *ge_can=NULL;

	/* check */
	if (minor>=PNET_GE_CAN_DEV_N)
		return -EINVAL;
	ge_can=&pnet_ge_can_private[minor];

	dbg("minor=%i ge_can=0x%X",minor,(unsigned int)ge_can);

	/* down */
	if (down_interruptible(&ge_can->sem))
		return -ERESTARTSYS;

	/* first open */
	if(!ge_can->open_count) {
		/* get struct */
		pnet_can_open(minor,&ge_can->pnet_can,PNET_GE_CAN_TIMEOUT);
		pnet_can_close(minor,PNET_GE_CAN_TIMEOUT);

		/* create and start can-message-receiver */
		ge_can->msg_rcv_thread=kthread_run(pnet_ge_can_msg_rcv_thread,(void *)minor,DRV_NAME"%i-mr",minor);
		if(!ge_can->msg_rcv_thread) {
			err("error creating rx_thread "DRV_NAME"%i",minor);
			ret=-ENOMEM;
			goto __ret_msg_rcv_thread_err;
		}
	}
	ge_can->open_count++;

	/* ok */
	if(_ge_can)
		(*_ge_can)=ge_can;
	ret=0;
	goto __ret_up;

__ret_msg_rcv_thread_err:
__ret_up:	
	up(&ge_can->sem);
	return ret;
}
EXPORT_SYMBOL(pnet_ge_can_open);

/*
 * close ge-can device
 * parameters:
 * 	minor .. pnet_ge_can-minor
 * return: 0 .. ok; <0 .. error
 */
int pnet_ge_can_close(int minor)
{
	int ret=0;
	struct pnet_ge_can_private *ge_can=NULL;

	if (minor>=PNET_GE_CAN_DEV_N)
		return -EINVAL;
	ge_can=&pnet_ge_can_private[minor];

	dbg("minor=%i ge_can=0x%X",minor,(unsigned int)ge_can);

	if(ge_can->open_count==1) {
		/* stop can-message receiver */
		dbg("stop msg_rcv_thread");
		kthread_stop(ge_can->msg_rcv_thread);
	}
	dbg("dec cnt_rcv_thread");

	down(&ge_can->sem);
	ge_can->open_count--;
	up(&ge_can->sem);

	return ret;
}
EXPORT_SYMBOL(pnet_ge_can_close);

/*
 **************************************************************************************************
 * END: KERNEL-API
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Linux-Driver-Functions
 **************************************************************************************************
 */
static int pnet_ge_can_lin_open(struct inode *inode, struct file *file)
{
	dbg("");

	/* check */
	if(file->f_flags & O_NONBLOCK) {
		err("O_NONBLOCK not supported");
		return -EINVAL;
	}

	/* do */
	return pnet_ge_can_open(iminor(inode), NULL);
}

static int pnet_ge_can_lin_close(struct inode *inode, struct file *file)
{
	dbg("");

	/* do */
	return pnet_ge_can_close(iminor(inode));
}

static int pnet_ge_can_lin_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int minor,ret;
	struct pnet_ge_can_rpc_data rpc_snd;
	struct pnet_ge_can_sequence ge_can_seq_user, ge_can_seq_kernel;
	struct pnet_ge_can_private *ge_can=NULL;

	minor=iminor(inode);
	if (minor>=PNET_GE_CAN_DEV_N)
		return -EINVAL;
	ge_can=&pnet_ge_can_private[minor];

	dbg("minor=%i ge_can=0x%X",minor,(unsigned int)ge_can);

	/* parse command */
	switch(cmd)
	{
		case PNET_GE_CAN_IOCTL_CMD_RPC_SND:
			/* check read-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_RDONLY)
				return -EINVAL;

			/* get parameters */
			ret=copy_from_user (&rpc_snd, argp, sizeof(struct pnet_ge_can_rpc_data));
			if(ret)
				return ret;

			dbg("PNET_GE_CAN_IOCTL_CMD_RPC_SND_SEND(msg_id=0x%X, data[0]=0x%X, data[1]=0x%X, data[2]=0x%X, data[3]=0x%X, data[4]=0x%X, data[5]=0x%X, data[6]=0x%X, data[7]=0x%X): do",
				rpc_snd.message_id,
				rpc_snd.data[0],rpc_snd.data[1],rpc_snd.data[2],rpc_snd.data[3],
				rpc_snd.data[4],rpc_snd.data[5],rpc_snd.data[6],rpc_snd.data[7]
			);

			ret=pnet_ge_can_rpc_snd(ge_can, &rpc_snd);

			dbg("PNET_GE_CAN_IOCTL_CMD_RPC_SND_SEND(msg_id=0x%X): done(ret=%i, data[0]=0x%X, data[1]=0x%X, data[2]=0x%X, data[3]=0x%X, data[4]=0x%X, data[5]=0x%X, data[6]=0x%X, data[7]=0x%X)",
				rpc_snd.message_id,
				ret,
				rpc_snd.data[0],rpc_snd.data[1],rpc_snd.data[2],rpc_snd.data[3],
				rpc_snd.data[4],rpc_snd.data[5],rpc_snd.data[6],rpc_snd.data[7]
			);

			if(ret)
				return ret;

			/* put parameters */
			ret=copy_to_user (argp, &rpc_snd, sizeof(struct pnet_ge_can_rpc_data));
			if(ret)
				return ret;
		break;

		case PNET_GE_CAN_IOCTL_CMD_SEQUENCE_SND:
			/* check read-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_RDONLY)
				return -EINVAL;

			/* get user base struct */
			ret=copy_from_user (&ge_can_seq_user, argp, sizeof(struct pnet_ge_can_sequence));
			if(ret)
				return ret;

			/* alloc kernel-seq-buffer */
			ge_can_seq_kernel.data=kmalloc(ge_can_seq_user.hdr.len*sizeof(struct pnet_ge_can_data), GFP_KERNEL);
			if(!ge_can_seq_kernel.data)
				return -ENOMEM;

			/* get seq_buffer */
			ret=copy_from_user (ge_can_seq_kernel.data, ge_can_seq_user.data, ge_can_seq_user.hdr.len*sizeof(struct pnet_ge_can_data));
			if(ret) {
				kfree(ge_can_seq_kernel.data);
				return ret;
			}

			/* build kernel structure */
			memcpy(&ge_can_seq_kernel.hdr,&ge_can_seq_user.hdr,sizeof(struct pnet_ge_can_hdr));

			dbg("PNET_GE_CAN_IOCTL_CMD_SEQUENCE_SND(node_id=0x%X, module_id=%i, module_instance_id=%i, seqlen=%i): do",
				ge_can_seq_kernel.hdr.node_id,
				ge_can_seq_kernel.hdr.module_id,
				ge_can_seq_kernel.hdr.module_instance_id,
				ge_can_seq_kernel.hdr.len
			);

			ret=pnet_ge_can_sequence_snd(ge_can, &ge_can_seq_kernel);

			dbg("PNET_GE_CAN_IOCTL_CMD_SEQUENCE_SND(node_id=0x%X, module_id=%i, module_instance_id=%i, seqlen=%i): done(ret=%i)",
				ge_can_seq_kernel.hdr.node_id,
				ge_can_seq_kernel.hdr.module_id,
				ge_can_seq_kernel.hdr.module_instance_id,
				ge_can_seq_kernel.hdr.len,
				ret
			);
			if(ret) {
				/* free kernel-seq-buffer */
				kfree(ge_can_seq_kernel.data);
				return ret;
			}

			/* build user structure */
			memcpy(&ge_can_seq_user.hdr,&ge_can_seq_kernel.hdr,sizeof(struct pnet_ge_can_hdr));

			/* put seq_buffer */
			ret=copy_to_user (ge_can_seq_user.data, ge_can_seq_kernel.data, ge_can_seq_kernel.hdr.len*sizeof(struct pnet_ge_can_data));

			/* free kernel-seq-buffer */
			kfree(ge_can_seq_kernel.data);

			if(ret)
				return ret;

			/* put user base struct */
			ret=copy_to_user (argp, &ge_can_seq_user, sizeof(struct pnet_ge_can_sequence));
			if(ret)
				return ret;
		break;

		case PNET_GE_CAN_IOCTL_CMD_BROADCAST_RCV:
			/* check read-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_RDONLY)
				return -EINVAL;

			/* get user base struct */
			ret=copy_from_user (&ge_can_seq_user, argp, sizeof(struct pnet_ge_can_sequence));
			if(ret)
				return ret;

			/* alloc kernel-seq-buffer */
			ge_can_seq_kernel.data=kmalloc(ge_can_seq_user.hdr.len*sizeof(struct pnet_ge_can_data), GFP_KERNEL);
			if(!ge_can_seq_kernel.data)
				return -ENOMEM;

			/* get seq_buffer */
			ret=copy_from_user (ge_can_seq_kernel.data, ge_can_seq_user.data, ge_can_seq_user.hdr.len*sizeof(struct pnet_ge_can_data));
			if(ret) {
				kfree(ge_can_seq_kernel.data);
				return ret;
			}

			/* build kernel structure */
			memcpy(&ge_can_seq_kernel.hdr,&ge_can_seq_user.hdr,sizeof(struct pnet_ge_can_hdr));

			dbg("PNET_GE_CAN_IOCTL_CMD_BROADCAST_RCV(node_id=0x%X, module_id=%i, module_instance_id=%i, seqlen=%i): do",
				ge_can_seq_kernel.hdr.node_id,
				ge_can_seq_kernel.hdr.module_id,
				ge_can_seq_kernel.hdr.module_instance_id,
				ge_can_seq_kernel.hdr.len
			);

			ret=pnet_ge_can_broadcast_rcv(ge_can, &ge_can_seq_kernel);

			dbg("PNET_GE_CAN_IOCTL_CMD_BROADCAST_RCV(node_id=0x%X, module_id=%i, module_instance_id=%i, seqlen=%i): done(ret=%i, parameter_id=%i, dlen=%i, data[0]=0x%X, data[1]=0x%X, data[2]=0x%X, data[3]=0x%X, data[4]=0x%X, data[5]=0x%X, data[6]=0x%X)",
				ge_can_seq_kernel.hdr.node_id,
				ge_can_seq_kernel.hdr.module_id,
				ge_can_seq_kernel.hdr.module_instance_id,
				ge_can_seq_kernel.hdr.len,
				ret, 
				ge_can_seq_kernel.data[0].parameter_id,
				ge_can_seq_kernel.data[0].dlen,
				ge_can_seq_kernel.data[0].data[0],ge_can_seq_kernel.data[0].data[1],ge_can_seq_kernel.data[0].data[2],
				ge_can_seq_kernel.data[0].data[3],ge_can_seq_kernel.data[0].data[4],ge_can_seq_kernel.data[0].data[5],
				ge_can_seq_kernel.data[0].data[6]
			);
			if(ret) {
				/* free kernel-seq-buffer */
				kfree(ge_can_seq_kernel.data);
				return ret;
			}

			/* build user structure */
			memcpy(&ge_can_seq_user.hdr,&ge_can_seq_kernel.hdr,sizeof(struct pnet_ge_can_hdr));

			/* put seq_buffer */
			ret=copy_to_user (ge_can_seq_user.data, ge_can_seq_kernel.data, ge_can_seq_kernel.hdr.len*sizeof(struct pnet_ge_can_data));

			/* free kernel-seq-buffer */
			kfree(ge_can_seq_kernel.data);

			if(ret)
				return ret;

			/* put user base struct */
			ret=copy_to_user (argp, &ge_can_seq_user, sizeof(struct pnet_ge_can_sequence));
			if(ret)
				return ret;
		break;

		default:
			err("invalid cmd %i",cmd);
			return -EINVAL;
		break;
	}

	return ret;
}

static struct file_operations pnet_ge_can_fops = {
	.owner = THIS_MODULE,
	.ioctl = pnet_ge_can_lin_ioctl,
	.open = pnet_ge_can_lin_open,
	.release = pnet_ge_can_lin_close,
};

static struct class *pnet_ge_can_class;

/*
 **************************************************************************************************
 * End: Linux-Driver-Functions
 **************************************************************************************************
 */


/***********************************************************************
 *  Begin: procfs statistics
 ***********************************************************************/
#ifdef CONFIG_PROC_FS

static int pnet_ge_can_info_show(struct seq_file *m, void *v)
{
	int i;

	dbg("");

	seq_printf(m,"%s\n",PNET_GE_CAN_VERSION_STR);

	for (i=0;i<PNET_GE_CAN_DEV_N;i++) {
		seq_printf(m,"%s%i:\n",DRV_NAME,i);

		down(&pnet_ge_can_private[i].sem);

		seq_printf(m," - Open:\n");
		seq_printf(m,"   - open:                       %i\n",pnet_ge_can_private[i].open_count);
		seq_printf(m," - Receiver:\n");
		seq_printf(m,"   - cnt_pnet_can_err:           %i\n",pnet_ge_can_private[i].cnt_pnet_can_err);
		seq_printf(m,"   - cnt_can_rcv_err:            %i\n",pnet_ge_can_private[i].cnt_can_rcv_err);
		seq_printf(m,"   - cnt_can_rcv_unknown:        %i\n",pnet_ge_can_private[i].cnt_can_rcv_unknown);
		seq_printf(m,"   - cnt_can_rcv_ok:             %i\n",pnet_ge_can_private[i].cnt_can_rcv_ok);
		seq_printf(m," - RPC sender:\n");
		seq_printf(m,"   - tx_err:                     %i\n",pnet_ge_can_private[i].rpc_snd.cnt_tx_err);
		seq_printf(m,"   - rx_timeout:                 %i\n",pnet_ge_can_private[i].rpc_snd.cnt_rx_timeout);
		seq_printf(m,"   - rx_err:                     %i\n",pnet_ge_can_private[i].rpc_snd.cnt_rx_err);
		seq_printf(m,"   - rpc_ok:                     %i\n",pnet_ge_can_private[i].rpc_snd.cnt_rpc_ok);
		seq_printf(m," - Broadcast receiver:\n");
		seq_printf(m,"   - rx_timeout:                 %i\n",pnet_ge_can_private[i].broadcast_rcv.cnt_rx_timeout);
		seq_printf(m,"   - rx_err:                     %i\n",pnet_ge_can_private[i].broadcast_rcv.cnt_rx_err);
		seq_printf(m,"   - broadcast_ok:               %i\n",pnet_ge_can_private[i].broadcast_rcv.cnt_broadcast_ok);
		seq_printf(m," - Sequence sender:\n");
		seq_printf(m,"   - tx_err:                     %i\n",pnet_ge_can_private[i].sequence_snd.cnt_tx_err);
		seq_printf(m,"   - rx_timeout:                 %i\n",pnet_ge_can_private[i].sequence_snd.cnt_rx_timeout);
		seq_printf(m,"   - rx_err:                     %i\n",pnet_ge_can_private[i].sequence_snd.cnt_rx_err);
		seq_printf(m,"   - sequence_ok:                %i\n",pnet_ge_can_private[i].sequence_snd.cnt_sequence_ok);

		up(&pnet_ge_can_private[i].sem);
	}

	return 0;
}

static int pnet_ge_can_info_open( struct inode *inode, struct file *file)
{
	return single_open(file, pnet_ge_can_info_show, NULL);
}

static struct file_operations pnet_ge_can_info_fops = {
	.owner = THIS_MODULE,
	.open = pnet_ge_can_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static struct proc_dir_entry *pnet_ge_can_proc_dir,*pnet_ge_can_proc_info;
static int pnet_ge_can_proc_create(void)
{
	pnet_ge_can_proc_dir=proc_mkdir("driver/"DRV_NAME,NULL);
	pnet_ge_can_proc_info=create_proc_entry("info",S_IRUGO,pnet_ge_can_proc_dir);
	if (pnet_ge_can_proc_info) {
		pnet_ge_can_proc_info->proc_fops = &pnet_ge_can_info_fops;
	}
	return 0;
}
static void pnet_ge_can_proc_remove(void)
{
	if (pnet_ge_can_proc_info) remove_proc_entry("info", pnet_ge_can_proc_dir);
	if (pnet_ge_can_proc_dir) remove_proc_entry("driver/"DRV_NAME,NULL);
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

static int __init pnet_ge_can_init(void)
{
	int i,j,ret;

	dbg("");

	/* register chrdev */
	ret=register_chrdev(DRV_MAJOR,DRV_NAME,&pnet_ge_can_fops);
	if (ret) {
		err("register chrdev failed");
		goto pnet_ge_can_init_err_cdev;
	}

	/* register device class */
	pnet_ge_can_class=class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(pnet_ge_can_class)) {
		err("class_create failed");
		ret = PTR_ERR(pnet_ge_can_class);
		goto pnet_ge_can_init_err_class;
	}

	/* register device */
	for (i=0;i<PNET_GE_CAN_DEV_N;i++) {

		/* init counters */
		pnet_ge_can_private[i].cnt_pnet_can_err=0;
		pnet_ge_can_private[i].cnt_can_rcv_err=0;
		pnet_ge_can_private[i].cnt_can_rcv_unknown=0;
		pnet_ge_can_private[i].cnt_can_rcv_ok=0;

		/* init rpc_snd */
		pnet_ge_can_rpc_snd_init(&pnet_ge_can_private[i].rpc_snd);

		/* init broadcast_rcv */
		pnet_ge_can_broadcast_rcv_init(&pnet_ge_can_private[i].broadcast_rcv);

		/* init sequence_snd */
		pnet_ge_can_sequence_snd_init(&pnet_ge_can_private[i].sequence_snd);

		/* init */
		init_MUTEX(&pnet_ge_can_private[i].sem);

		pnet_ge_can_private[i].device=device_create(pnet_ge_can_class,NULL,MKDEV(DRV_MAJOR,i),NULL,"%s%d",DRV_NAME,i);
		if (IS_ERR(pnet_ge_can_private[i].device)) {
			err("device_create failed");
			ret=PTR_ERR(pnet_ge_can_private[i].device);
			/* unregister devices */
			i--;
			while(i--)
				device_destroy(pnet_ge_can_class,MKDEV(DRV_MAJOR,i));
			goto pnet_ge_can_init_err_class_device;
		}

		/* config */
		pnet_ge_can_private[i].can_cfg.bitrate=125000;
		pnet_ge_can_private[i].can_cfg.sample_point=87;
		pnet_ge_can_private[i].can_cfg.edge_resync_mode=0;
		pnet_ge_can_private[i].can_cfg.sync_jump_width=1;
		pnet_ge_can_private[i].can_cfg.byte_order=0;

		pnet_ge_can_private[i].can_cfg.mask[0]=0xffffffff&~(PNET_CAN_MBOX_CONFIG_IDE); 										/* rpc_snd rx */
		pnet_ge_can_private[i].can_cfg.mask[1]=~(PNET_GE_CAN_RES_MASK|PNET_GE_CAN_FLAG_BC_MASK|PNET_GE_CAN_FLAG_SS_MASK);	/* mask for broadcast_rcv rx */
		pnet_ge_can_private[i].can_cfg.mask[2]=~(PNET_GE_CAN_RES_MASK|PNET_GE_CAN_FLAG_BC_MASK|PNET_GE_CAN_NODE_ID_MASK);	/* mask for sequence_snd rx */

		/* reset configuration */
		pnet_ge_can_private[i].can_mbox_ena=0;
		for(j=0;j<PNET_CAN_MAILBOX_MAX;j++) {
			/* disable */
			pnet_ge_can_private[i].can_mbox_cfg[j].mbox=PNET_CAN_MAILBOX_MAX;
		}
		
		/* basic configuration */

		/* rpc_snd */
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_RPC_SND_RX_MBOX].mbox=PNET_GE_CAN_RPC_SND_RX_MBOX;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_RPC_SND_RX_MBOX].type=CAN_MBOX_TYPE_RX;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_RPC_SND_RX_MBOX].message_id=PNET_CAN_MBOX_CONFIG_AME;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_RPC_SND_RX_MBOX].data_len=8;

		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_RPC_SND_TX_MBOX].mbox=PNET_GE_CAN_RPC_SND_TX_MBOX;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_RPC_SND_TX_MBOX].type=CAN_MBOX_TYPE_TX;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_RPC_SND_TX_MBOX].message_id=0;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_RPC_SND_TX_MBOX].data_len=8;

		pnet_ge_can_private[i].can_mbox_ena|=(1<<PNET_GE_CAN_RPC_SND_RX_MBOX)|(1<<PNET_GE_CAN_RPC_SND_TX_MBOX);


		/* broadcast_rcv */
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_BROADCAST_RCV_RX_MBOX].mbox=PNET_GE_CAN_BROADCAST_RCV_RX_MBOX;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_BROADCAST_RCV_RX_MBOX].type=CAN_MBOX_TYPE_RX;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_BROADCAST_RCV_RX_MBOX].message_id=PNET_CAN_MBOX_CONFIG_AME|PNET_CAN_MBOX_CONFIG_IDE|(1<<PNET_GE_CAN_FLAG_BC_SHIFT);
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_BROADCAST_RCV_RX_MBOX].data_len=8;

		pnet_ge_can_private[i].can_mbox_ena|=(1<<PNET_GE_CAN_BROADCAST_RCV_RX_MBOX);


		/* sequence_snd (configured while sequence send - function) */
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_SEQUENCE_SND_TX_MBOX].mbox=PNET_GE_CAN_SEQUENCE_SND_TX_MBOX;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_SEQUENCE_SND_TX_MBOX].type=CAN_MBOX_TYPE_TX;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_SEQUENCE_SND_TX_MBOX].message_id=PNET_CAN_MBOX_CONFIG_IDE;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_SEQUENCE_SND_TX_MBOX].data_len=2;

		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_SEQUENCE_SND_RX_MBOX].mbox=PNET_GE_CAN_SEQUENCE_SND_RX_MBOX;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_SEQUENCE_SND_RX_MBOX].type=CAN_MBOX_TYPE_RX;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_SEQUENCE_SND_RX_MBOX].message_id=PNET_CAN_MBOX_CONFIG_AME|PNET_CAN_MBOX_CONFIG_IDE;
		pnet_ge_can_private[i].can_mbox_cfg[PNET_GE_CAN_SEQUENCE_SND_RX_MBOX].data_len=2;

		pnet_ge_can_private[i].can_mbox_ena|=(1<<PNET_GE_CAN_SEQUENCE_SND_TX_MBOX);
	}		

#ifdef CONFIG_PROC_FS
	pnet_ge_can_proc_create();
#endif

	return ret;

pnet_ge_can_init_err_class_device:
	class_destroy(pnet_ge_can_class);
pnet_ge_can_init_err_class:
	unregister_chrdev(DRV_MAJOR,DRV_NAME);
pnet_ge_can_init_err_cdev:
	return ret;
}

static void __exit pnet_ge_can_exit(void)
{
	int i;

	dbg("");

#ifdef CONFIG_PROC_FS
	pnet_ge_can_proc_remove();
#endif

	/* unregister device */
	for (i=0;i<PNET_GE_CAN_DEV_N;i++)
		device_destroy(pnet_ge_can_class,MKDEV(DRV_MAJOR,i));
	class_destroy(pnet_ge_can_class);
	unregister_chrdev(DRV_MAJOR,DRV_NAME);
}

module_init( pnet_ge_can_init );
module_exit( pnet_ge_can_exit );

MODULE_AUTHOR("Manfred Schlaegl");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ge_can driver over pnet_can");

/*
 **************************************************************************************************
 * End: Module-Stuff
 **************************************************************************************************
 */
