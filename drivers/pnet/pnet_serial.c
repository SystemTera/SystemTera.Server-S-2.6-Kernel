/***********************************************************************
 *
 * @Authors: Manfred Schlägl, Ginzinger electronic systems GmbH (C) 2007
 * @Descr: Driver to use a remote Serial-Port over PNET
 * @TODO:
 *    o test
 *
 * Based on:
 *	usb-serial.c
 * 		Copyright (C) 1999 - 2004 Greg Kroah-Hartman (greg@kroah.com)
 * 		Copyright (C) 2000 Peter Berger (pberger@brimson.com)
 * 		Copyright (C) 2000 Al Borchers (borchers@steinerpoint.com)
 * 	tiny_tty
 *		Copyright (C) 2002-2004 Greg Kroah-Hartman (greg@kroah.com)
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
 *	2011KW11 - manfred.schlaegl
 *		support for kap_bed-board (A9M9_BOARD_KAP_BED)
 *	2010-KW22 manfred.schlaegl@gmx.at
 *		* port to linux-2.6.32
 *	11.11.2009: manfred.schlaegl
 *		* Support for bhz_b-board
 ***********************************************************************/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <asm/delay.h>
#include "pnet_serial.h"

/*
++++++++++++++++++++++++++++++++++
DRIVER INFORMATION
++++++++++++++++++++++++++++++++++
*/
#define PNET_SERIAL_DRIVER_NAME 	"pnet_serial"
#define PNET_SERIAL_DRIVER_VERSION 	"v0.7"
#define PNET_SERIAL_DRIVER_AUTHOR 	"Manfred Schlaegl, Ginzinger electronic systems GmbH (C) 2007"
#define PNET_SERIAL_DRIVER_DESC 	"PNET Serial Driver"

#define PNET_SERIAL_TTY_MAJOR		242				/* experimental range */
#define PNET_SERIAL_TTY_MINOR		0				/* 0 to 0+PNET_SERIAL_TTY_MINORS */


/* Number of Ports */
#if 	defined(CONFIG_A9M9_BOARD_ETABEB) 	|| \
		defined(CONFIG_A9M9_BOARD_ETA_BEP)

#define PNET_SERIAL_TTY_MINORS		1				/* nr of ports */

#elif 	defined(CONFIG_A9M9_BOARD_VALI) 		|| \
		defined(CONFIG_A9M9_BOARD_GEA9A7DEV) 	|| \
		defined(CONFIG_A9M9_BOARD_CMCX)			|| \
		defined(CONFIG_A9M9_BOARD_BHZ_B)		|| 		/* bhz_b: hw not avaliable, but support possible */		\
		defined(CONFIG_A9M9_BOARD_LWE_CU)		||		/* lwe_cu: hw not avaliable, but support possible */	\
		defined(CONFIG_A9M9_BOARD_KAP_BED)		 		/* kap_bed: hw not avaliable, but support possible */	\

#define PNET_SERIAL_TTY_MINORS		2				/* nr of ports */

#else
#warning "Unkown board-configuration"
#define PNET_SERIAL_TTY_MINORS		2				/* nr of ports */
#endif

#define PNET_SERIAL_PNET_PORT_START	5				/* minor 0 -> 5,6; minor 1 -> 7,8; ... */

#define PNET_SERIAL_TTY_NAME		PNET_SERIAL_DRIVER_NAME		/* driver name */
#define PNET_SERIAL_TTY_NAME_DEV	"ttyPNET"			/* name of dev-node */


/* table for serial-lines over pnet */
static struct pnet_serial *pnet_serial_table[PNET_SERIAL_TTY_MINORS];


/*
++++++++++++++++++++++++++++++++++
MESSAGES & DEBUG
++++++++++++++++++++++++++++++++++
*/

/* flag for debug-messages */
static int debug=0;

#define MY_NAME	PNET_SERIAL_DRIVER_NAME
#define _KERN_DEBUG_ KERN_INFO /* for debug */

/* kernel Output */
#define err(format, arg...) printk(KERN_ERR "%s: %s:\t\t" format "\n" , MY_NAME , __FUNCTION__ , ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: %s:\t\t" format "\n" , MY_NAME , __FUNCTION__, ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: %s:\t\t" format "\n" , MY_NAME , __FUNCTION__, ## arg)

/* debug messages */
#define dbg_raw(fmt, arg...) 						\
	do {								\
		if (debug)						\
			printk (_KERN_DEBUG_ fmt, ## arg);		\
	} while (0)
/* debug messages with module- and function-information */
#define dbg(fmt, arg...) 						\
	dbg_raw("%s: %s:\t\t" fmt, MY_NAME , __FUNCTION__ , ## arg)	
/* debug messages with line-information */
#define dbg_line(fmt, arg...) dbg("Port-Line %i " fmt, tty->index, ## arg)


/*
++++++++++++++++++++++++++++++++++
MISC 
++++++++++++++++++++++++++++++++++
*/

#if 0
#ifndef schedule_timeout_interruptible
signed long schedule_timeout_interruptible(signed long timeout)
{
	set_current_state(TASK_INTERRUPTIBLE);
	return schedule_timeout(timeout);
}
#endif
#endif

/* get PNET_serial struct */ 
#define _GET_PNET_SERIAL_ struct pnet_serial *pnet_serial=pnet_serial_table[tty->index];


/* check, if pnet_serial is open (return error if possible)*/
#define _CHECK_OPEN_COUNT_(retval) 			\
		if(pnet_serial->open==0) {			\
			dbg_line("not openend\n");		\
			return retval;					\
		}

#define _CHECK_INT_CONTEXT_(retval)						\
		if(in_interrupt())	{							\
			err("not allowed in interrupt context");	\
			return retval;								\
		}	

/* driver post-initialisation (done once per driver-load in open) */
static void pnet_serial_try_post_init(struct pnet_serial *pnet_serial);

/*
++++++++++++++++++++++++++++++++++
FUNCTIONS (callbacks)
++++++++++++++++++++++++++++++++++
*/

/* data was sent to fifo peer */
static void pnet_serial_dport_sent(struct pnet_core_port *port)
{
	struct pnet_serial *pnet_serial=(struct pnet_serial *)port->private_data;
//	pnet_serial=pnet_serial_table[(port->port-PNET_SERIAL_PNET_PORT_START)/2];

	dbg("dport sent callback %i\n",(port->port-PNET_SERIAL_PNET_PORT_START)/2);

	/* wakeup waiting write-tasks */
	if(pnet_serial->tty) 
		tty_wakeup(pnet_serial->tty);

	/* wakeup processes waiting for hangup */
	wake_up(&pnet_serial->sent_wq);
}

/* data was received from fifo peer */
static void pnet_serial_dport_received(struct pnet_core_port *port)
{
	struct pnet_serial *pnet_serial=(struct pnet_serial *)port->private_data;
//	pnet_serial=pnet_serial_table[(port->port-PNET_SERIAL_PNET_PORT_START)/2];

	dbg("dport received callback %i\n",(port->port-PNET_SERIAL_PNET_PORT_START)/2);

	tasklet_schedule(&pnet_serial->dread_tl);
}

/* ctrl message was sent to fifo peer */
static void pnet_serial_cport_sent(struct pnet_core_port *port)
{
	struct pnet_serial *pnet_serial=(struct pnet_serial *)port->private_data;
//	pnet_serial=pnet_serial_table[(port->port-PNET_SERIAL_PNET_PORT_START-1)/2];

	dbg("cport sent callback %i\n",(port->port-PNET_SERIAL_PNET_PORT_START-1)/2);

	/* wakeup processes waiting for ctrl */
	wake_up(&pnet_serial->csent_wq);

	/* wakeup processes waiting for hangup */
	wake_up(&pnet_serial->sent_wq);
}

/* ctrl message was received from fifo peer */
static void pnet_serial_cport_received(struct pnet_core_port *port)
{
	struct pnet_serial *pnet_serial=(struct pnet_serial *)port->private_data;
//	pnet_serial=pnet_serial_table[(port->port-PNET_SERIAL_PNET_PORT_START-1)/2];

	dbg("cport received callback %i\n",(port->port-PNET_SERIAL_PNET_PORT_START-1)/2);

	/* start cread tasklet */
	tasklet_schedule(&pnet_serial->cread_tl);
}


/* connection state of port changed */
static void pnet_serial_changed_state(struct pnet_core_port *port, int state)
{
	struct pnet_serial *pnet_serial=(struct pnet_serial *)port->private_data;
	int i;

	dbg("port connected callback state=%i\n",state);
	/* reset flag */
	pnet_serial->flags|=PNET_SERIAL_FLAG_DISCONNECTED;

	/* reset stats */
	pnet_serial->tx_count=0;
	pnet_serial->rx_count=0;
	/* reset termios */
#if 1	/* CAUTION -> TODO */
	if(pnet_serial->tty_driver&&pnet_serial->tty_driver->termios) {
		for(i=0;pnet_serial->tty_driver->termios[i]&&(i<PNET_SERIAL_TTY_MINORS);i++) {
			pnet_serial->tty_driver->termios[i]->c_iflag=tty_std_termios.c_iflag;
			pnet_serial->tty_driver->termios[i]->c_oflag=tty_std_termios.c_oflag;
			pnet_serial->tty_driver->termios[i]->c_cflag=tty_std_termios.c_cflag;
			pnet_serial->tty_driver->termios[i]->c_lflag=tty_std_termios.c_lflag;
			pnet_serial->tty_driver->termios[i]->c_line=tty_std_termios.c_line;
			memcpy(pnet_serial->tty_driver->termios[i]->c_cc,tty_std_termios.c_cc,NCCS*sizeof(cc_t));

			pnet_serial->tty_driver->termios[i]->c_cflag 
							= B9600 | CS8 | CREAD | HUPCL | CLOCAL;
		}
#endif
	}

	/* wakup waiting */
	wake_up(&pnet_serial->conn_state_changed_wq);
}


/* check connection state */
static int pnet_serial_is_connected(struct pnet_serial *pnet_serial)
{
	int connected;
	spin_lock_bh(&pnet_serial->lock);
	connected=(
			(pnet_serial->pnet_channel.data_port.state&PNET_CORE_PORT_CONNECTED)&&
			(pnet_serial->pnet_channel.ctrl_port.state&PNET_CORE_PORT_CONNECTED)
			);
	spin_unlock_bh(&pnet_serial->lock);
	return connected;
}

/* data read work object proc */
static void pnet_serial_dread_tl_proc(unsigned long arg) 
{
	unsigned int bytes, i;
	unsigned char buf[256];
	struct pnet_serial *pnet_serial = (struct pnet_serial*) arg;

	dbg("\n");

	/* throttled mode */
	spin_lock_bh(&pnet_serial->lock);
	i=pnet_serial->flags&PNET_SERIAL_FLAG_RX_THROTTLED;
	spin_unlock_bh(&pnet_serial->lock);
	if(i) {
		dbg("throttled > exit\n");
		return;
	}

	/* get bytes to push */
	bytes=pnet_core_port_read_len(&pnet_serial->pnet_channel.data_port);

	/* no data -> return */
	if(!bytes) {
		dbg("nothing to read -> exit\n");
		return;
	}

	/* while data to push */
	while(bytes) {
		dbg("insert %i\n",bytes);
		/* max buffer */
		if(bytes>256)
			bytes=256;

		dbg("read\n");
		bytes=pnet_core_port_read(&pnet_serial->pnet_channel.data_port,buf,bytes);

		/* if opened -> pass data to tty-layer */
		for(i=0;i<bytes;i+=2) {

			spin_lock_bh(&pnet_serial->lock);

			if(pnet_serial->tty) {
				/* check for space in ldisc */
				if (tty_buffer_request_room(pnet_serial->tty,2) < 2) {
					/* break out & wait for throttling/unthrottling to happen */
					dbg("receive room low\n");
					spin_unlock_bh(&pnet_serial->lock);
					break;
				}

#if 0
				/* need buffer push ? */
				if (pnet_serial->tty->flip.count >= TTY_FLIPBUF_SIZE) {
					dbg("tty_flip_buffer_push\n");
					tty_flip_buffer_push(pnet_serial->tty);
				}
#endif

				/* insert char & err */
				dbg("tty_insert_flip_char(0x%X,%c,%i)\n",(unsigned int)pnet_serial->tty,buf[i],buf[i+1]);
				tty_insert_flip_char(pnet_serial->tty, buf[i], buf[i+1]);

#if 1
				if(buf[i+1])
					err("RX-Error: %i",buf[i+1]);
#endif

			} else {
				dbg("waste data\n");
				/* waste data */
			}

			spin_unlock_bh(&pnet_serial->lock);
		}
	

		pnet_serial->rx_count+=bytes;

		bytes=pnet_core_port_read_len(&pnet_serial->pnet_channel.data_port);
	}

	spin_lock_bh(&pnet_serial->lock);
    if( !pnet_serial->tty->low_latency ) {
		spin_unlock_bh(&pnet_serial->lock);
		tty_flip_buffer_push( pnet_serial->tty );
	} else
		spin_unlock_bh(&pnet_serial->lock);

	
#if 0
	if(pnet_serial->tty) {
		tty_flip_buffer_push(pnet_serial->tty);
	}
#endif

}


/*
++++++++++++++++++++++++++++++++++
FUNCTIONS (ctrl-communication)
++++++++++++++++++++++++++++++++++
*/

/* ctrl read work object proc */
static void pnet_serial_cread_tl_proc(unsigned long arg) 
{
	int ret;
	struct pnet_serial *pnet_serial = (struct pnet_serial*) arg;
	struct pnet_serial_ctrl_packet packet;


	spin_lock_bh(&pnet_serial->lock);
	/* while enough data */
	while(pnet_core_port_read_len(&pnet_serial->pnet_channel.ctrl_port)>=sizeof(struct pnet_serial_ctrl_packet)) {

		/* receive */
		ret=pnet_core_port_read(&pnet_serial->pnet_channel.ctrl_port,(void*)&packet,sizeof(struct pnet_serial_ctrl_packet));
		spin_unlock_bh(&pnet_serial->lock);

		if(ret!=sizeof(struct pnet_serial_ctrl_packet)) {
			err("error reading cport data");
			return;
		}
		/* convert & save */
		packet.cmd=ntohl(packet.cmd);
		packet.arg=ntohl(packet.arg);

		/* check protocol */
		if(packet.cmd&PNET_SERIAL_CMD_FLAG_REPLY) {
			/* received reply */
			/* check if last packet is unread */
			if(unlikely(pnet_serial->flags&PNET_SERIAL_FLAG_UNREAD_CREPLY)) {
				err("cport overflow (never happens because only one reply per command is sent)");
				return;
			}

			/* save reply */
			spin_lock_bh(&pnet_serial->lock);
			pnet_serial->creply.cmd=packet.cmd;
			pnet_serial->creply.arg=packet.arg;
			pnet_serial->flags|=PNET_SERIAL_FLAG_UNREAD_CREPLY;	/* new packet */
			spin_unlock_bh(&pnet_serial->lock);

			/* wakeup processes waiting for reply */
			wake_up(&pnet_serial->crcv_wq);

		} else {
			if(packet.cmd&PNET_SERIAL_CMD_FLAG_REQUEST) {
				/* received request */
				packet.cmd&=~PNET_SERIAL_CMD_FLAG_REQUEST;
				err("reveived invalid command (cmd=%i,arg=%i)\n",packet.cmd,packet.arg);
			} else {
				err("reveived invalid message type (cmd=%i,arg=%i)\n",packet.cmd,packet.arg);
			} /* else */
		} /* else */
		spin_lock_bh(&pnet_serial->lock);
	} /* while */
	spin_unlock_bh(&pnet_serial->lock);
}



/* send ctrl packet */
static int pnet_serial_send_packet(struct pnet_serial *pnet_serial, unsigned int cmd, unsigned int arg)
{
	int ret=0;
	struct pnet_serial_ctrl_packet packet;

	/* convert packet */
	cmd|=PNET_SERIAL_CMD_FLAG_REQUEST;
	packet.cmd=htonl(cmd);
	packet.arg=htonl(arg);

	spin_lock_bh(&pnet_serial->lock);

	/* called in interrupt (no wait) */
	if(in_interrupt()) {
		/* not enough space -> return */
		if(pnet_core_port_write_free(&pnet_serial->pnet_channel.ctrl_port)<sizeof(struct pnet_serial_ctrl_packet)) {
			err("not enough space to send command\n");
			spin_unlock_bh(&pnet_serial->lock);
			return -EIO;
		}
	} else {
		do {
			spin_unlock_bh(&pnet_serial->lock);
			/* not enough space -> wait unlocked */
			if(!wait_event_timeout(pnet_serial->csent_wq, 
					(pnet_core_port_write_free(&pnet_serial->pnet_channel.ctrl_port)>=sizeof(struct pnet_serial_ctrl_packet)),
					HZ
			)) {
				err("timeout waiting for cmd-reply");
				return -ERESTARTSYS;
			}
			spin_lock_bh(&pnet_serial->lock);
		} while(pnet_core_port_write_free(&pnet_serial->pnet_channel.ctrl_port)<sizeof(struct pnet_serial_ctrl_packet));
	}
	
	/* send */	
	ret=pnet_core_port_write(&pnet_serial->pnet_channel.ctrl_port, (void*)&packet, sizeof(struct pnet_serial_ctrl_packet));
	if(ret!=sizeof(struct pnet_serial_ctrl_packet)) {
		err("%i instead of %i written\n",ret,sizeof(struct pnet_serial_ctrl_packet));
		ret=-EIO;
	} else {
		ret=0;
	}

	spin_unlock_bh(&pnet_serial->lock);
	/* ok */
	return ret;
}

/* rcv ctrl packet */
static int pnet_serial_rcv_packet(struct pnet_serial *pnet_serial, unsigned int *cmd, unsigned int *arg)
{
	_CHECK_INT_CONTEXT_(-EPERM);

	/* wait for received reply */
	if(!wait_event_timeout( (pnet_serial->crcv_wq), (pnet_serial->flags&PNET_SERIAL_FLAG_UNREAD_CREPLY), HZ)) {
		err("timeout waiting for cmd-reply");
		return -ERESTARTSYS;
	}

	/* get reply */
	spin_lock_bh(&pnet_serial->lock);
	(*cmd)=pnet_serial->creply.cmd;
	(*arg)=pnet_serial->creply.arg;
	pnet_serial->flags&=~PNET_SERIAL_FLAG_UNREAD_CREPLY;	/* packet read */
	spin_unlock_bh(&pnet_serial->lock);

	/* check protocol */
	if((*cmd)&PNET_SERIAL_CMD_FLAG_REPLY) {
		/* received reply */
		(*cmd)&=~PNET_SERIAL_CMD_FLAG_REPLY;
		/* ok */
		return 0;
	} else {
		/* protocol error */
		return -EPROTO;
	}
}

/* send packet wait for ack */
static int pnet_serial_do_cmd(struct pnet_serial *pnet_serial, unsigned int cmd, unsigned int *arg)
{
	int ret,rcmd;

	_CHECK_INT_CONTEXT_(-EPERM);

	/* send request */
	ret=pnet_serial_send_packet(pnet_serial,cmd,*arg);
	if(ret) {
		err("send packet error: %i",ret);
		return ret;
	}

	/* receive reply */
	ret=pnet_serial_rcv_packet(pnet_serial,&rcmd,arg);
	if(ret) {
		err("rcv packet error: %i",ret);
		return ret;
	}

	/* check */
	if(rcmd!=cmd) {
		err("wrong answer to packet\n");
		return -EPROTO;
	}

	return 0;
}

/* update line control_state */
static int pnet_serial_update_control_state(struct pnet_serial *pnet_serial) 
{
	int ret;
	union pnet_serial_ctrl_lines line_status;
	unsigned int control_state=0;

	ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_GET_LINES,&line_status.val);
	if(ret) {
		err("error: %i",ret);
		return -1;
	}

	spin_lock_bh(&pnet_serial->lock);

	control_state=pnet_serial->control_state;

	/* check for cmd-error */
	if (line_status.bits.error) {
		spin_unlock_bh(&pnet_serial->lock);
		return -2;
	}

	/* set bits of control_state */
	if (line_status.bits.DTR) 
		control_state |= TIOCM_DTR;
	else
		control_state &= ~TIOCM_DTR;

	if (line_status.bits.RTS)
		control_state |= TIOCM_RTS;
	else
		control_state &= ~TIOCM_RTS;

	if (line_status.bits.CTS)
		control_state |= TIOCM_CTS;
	else
		control_state &= ~TIOCM_CTS;

	if (line_status.bits.DSR)
		control_state |= TIOCM_DSR;
	else
		control_state &= ~TIOCM_DSR;

	if (line_status.bits.CD)
		control_state |= TIOCM_CD;
	else
		control_state &= ~TIOCM_CD;

	if (line_status.bits.RI)
		control_state |= TIOCM_RI;
	else
		control_state &= ~TIOCM_RI;

	if (line_status.bits.LL)
		control_state |= TIOCM_LOOP;
	else
		control_state &= ~TIOCM_LOOP;

	/* save control_state */
	pnet_serial->control_state=control_state;

	spin_unlock_bh(&pnet_serial->lock);

	return control_state;
}


static int pnet_serial_disable_flowctrl(struct pnet_serial *pnet_serial) 
{
	unsigned int arg,ret;
	union pnet_serial_flow_ctrl flowctrl;

	/* disable flow control */
	flowctrl.val=0;

	/* start/stop (xon/xoff) chars */
	flowctrl.bits.vstart=pnet_serial->tty->termios->c_cc[VSTART];
	flowctrl.bits.vstop=pnet_serial->tty->termios->c_cc[VSTOP];

	/* output flow ctrl */
	flowctrl.bits.oxon = 0;

	/* input flow ctrl */
	flowctrl.bits.ixon = 0;

	/* input/output flow ctrl */
	flowctrl.bits.crtscts = 0;

	/* use modem lines (dtr/cd) */
	flowctrl.bits.clocal = 1;

	/* -> set flowctrl */
	arg=flowctrl.val;
	ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_FLOWCTRL,&arg);
	if(ret) {
		err("disable flowctrl error: %i",ret);
	}
	return ret;
}

/*
++++++++++++++++++++++++++++++++++
FUNCTIONS (driver tty interface)
++++++++++++++++++++++++++++++++++
*/

/* serial open */
static int pnet_serial_open (struct tty_struct *tty, struct file *file) 
{
	unsigned int arg;
	int ret=0;
	_GET_PNET_SERIAL_
	_CHECK_INT_CONTEXT_(-EPERM);

	dbg_raw("\n");
	
	if(down_interruptible(&pnet_serial->mutex)) {
		warn("mutex interrupted");
		return -ERESTARTSYS;
	}

	/* do post initialiation if neccessary */
	pnet_serial_try_post_init(pnet_serial);

	dbg_line("rx_throttled: %i\n",(pnet_serial->flags&PNET_SERIAL_FLAG_RX_THROTTLED)>0);
	
	/* wait until connected */
	if (!pnet_serial_is_connected(pnet_serial)) {
		if (file->f_flags & O_NONBLOCK) {
			ret=-EPIPE;
			goto pnet_serial_open_ret;
		}
		if (wait_event_interruptible(pnet_serial->conn_state_changed_wq,pnet_serial_is_connected(pnet_serial))) {
			ret=-ERESTARTSYS;
			goto pnet_serial_open_ret;
		}
	}

	/* first open */
	if(pnet_serial->open==0) {
		/* first time opened */
		spin_lock_bh(&pnet_serial->lock);
		/* unthrottle all on open & set connected*/
		pnet_serial->flags&=~(PNET_SERIAL_FLAG_TX_THROTTLED|PNET_SERIAL_FLAG_RX_THROTTLED|PNET_SERIAL_FLAG_DISCONNECTED);

		/* link structures */
		pnet_serial->tty=tty;

//		tty->low_latency=1;				/* TODO */
		tty->driver_data=pnet_serial;
		pnet_serial->flags&=~PNET_SERIAL_FLAG_UNREAD_CREPLY;
		spin_unlock_bh(&pnet_serial->lock);

		/* reset remote port */
		dbg("reset\n");

		/* disable port */
		arg=0;
		ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_ENABLE,&arg);
		if(ret) {
			err("failed to disable port err: %i",ret);
			ret=-ENODEV;
			goto pnet_serial_open_ret;
		}
		if(arg!=0) {
			err("remote failed to disable port");
			ret=-EPIPE;
			goto pnet_serial_open_ret;
		}

		/* wait until all data handled on remote side or signal */
		while(1) {
			dbg("get tx-count\n");
			pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_GET_TXCOUNT,&arg);
			if(pnet_serial->tx_count==arg) {
				/* all data sent -> done */
				break;
			}
			dbg("%i!=%i -> wait\n",pnet_serial->tx_count,arg);
			if(schedule_timeout_interruptible(HZ)) {
				printk(KERN_INFO "pnet_serial signal");
				err("signal");
				ret=-EPIPE;
				goto pnet_serial_open_ret;
			}
		}
		dbg("%i==%i -> wait\n",pnet_serial->tx_count,arg);

		/* enable port */
		arg=1;
		ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_ENABLE,&arg);
		if(ret) {
			err("failed to enable port err: %i",ret);
			ret=-EIO;
			goto pnet_serial_open_ret;
		}
		if(arg!=1) {
			err("remote failed to enable port");
			ret=-EPIPE;
			goto pnet_serial_open_ret;
		}

		tasklet_schedule(&pnet_serial->dread_tl);
	}

	/* update counter */
	pnet_serial->open++;

pnet_serial_open_ret:
	up(&pnet_serial->mutex);

	dbg_line("done\n");

	return ret;
}

/* close */
static void pnet_serial_close(struct tty_struct *tty, struct file * file) 
{
	unsigned int arg,ret;
	unsigned int c_cflag = tty->termios->c_cflag;
	_GET_PNET_SERIAL_

	dbg_line("\n");
	_CHECK_INT_CONTEXT_();
	_CHECK_OPEN_COUNT_();

	down(&pnet_serial->mutex);

	dbg_line("in mutex \n");

	/* last close */
	/* disable port */
	if(pnet_serial->open==1) {
		/* wait until all data written on remote side or signal */
		/* if it was disconnected while open, both remote and local tx_count has been reset to 0 */
		while(1) {
			pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_GET_TXCOUNT,&arg);
			if(pnet_serial->tx_count==arg) {
				/* all data sent -> done */
				break;
			}
			up(&pnet_serial->mutex);
			/* wait unlocked */
			if(schedule_timeout_interruptible(HZ)) {
				down(&pnet_serial->mutex);
				printk(KERN_INFO "pnet_serial signal");
				/* signal -> abort */
				break;
			}
			down(&pnet_serial->mutex);
		}

		/* disable */
		arg=0;
		ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_ENABLE,&arg);
		if(ret) {
			err("failed to disable port err: %i",ret);
			up(&pnet_serial->mutex);
			return;
		}
		if(arg!=0) {
			err("remote failed to disable port");
			up(&pnet_serial->mutex);
			return;
		}


		/* stop tx */
		spin_lock_bh(&pnet_serial->lock);
		pnet_serial->flags|=PNET_SERIAL_FLAG_TX_THROTTLED;
		spin_unlock_bh(&pnet_serial->lock);

		/* HUPCL handling */
		if (c_cflag & HUPCL){
#if 0
			/* disable flowctrl */
			if(pnet_serial_disable_flowctrl(pnet_serial)) {
				err("error disabling flowctrl");
				up(&pnet_serial->mutex);
				return;
			}
#endif

			/* clear rts and dtr */
			arg=0;
			ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_RTS,&arg);
			if(ret) {
				err("error clear rts: %i",ret);
				up(&pnet_serial->mutex);
				return;
			}

			arg=0;	
			ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_DTR,&arg);
			if(ret) {
				err("error clear dtr: %i",ret);
				up(&pnet_serial->mutex);
				return;
			}

			/* update line-state */
			if(pnet_serial_update_control_state(pnet_serial)<0) {
				err("error getting control state: %i",ret);
				up(&pnet_serial->mutex);
				return;
			}
		} /* Note change no line if hupcl is off */

		spin_lock_bh(&pnet_serial->lock);
		pnet_serial->tty=NULL;		
		spin_unlock_bh(&pnet_serial->lock);
	}

	/* update counter */
	pnet_serial->open--;

	up(&pnet_serial->mutex);

	dbg_line("\n");
	dbg_raw("\n");
}


/* from tty-layer -> driver */
static int pnet_serial_write(struct tty_struct * tty, const unsigned char *buf, int count) 
{
	int bytes;
	_GET_PNET_SERIAL_

	dbg_line("%i %i %s\n",tty->index,count,in_interrupt() ? "interrupt call" : "user call");

	_CHECK_OPEN_COUNT_(-EINVAL);

	spin_lock_bh(&pnet_serial->lock);

	/* error if disconnected */
	if (pnet_serial->flags&PNET_SERIAL_FLAG_DISCONNECTED) {
		bytes=-EPIPE;
		goto pnet_serial_write_ret;
	}

	/* transmit throttled */
	if(pnet_serial->flags&PNET_SERIAL_FLAG_TX_THROTTLED) {
		dbg_line("throttled\n");
		bytes=0;
		goto pnet_serial_write_ret;
	}

	bytes=pnet_core_port_write_free(&pnet_serial->pnet_channel.data_port);

	if(bytes>count)
		bytes=pnet_core_port_write(&pnet_serial->pnet_channel.data_port,(void *)buf,count);
	else
		bytes=pnet_core_port_write(&pnet_serial->pnet_channel.data_port,(void *)buf,bytes);

	pnet_serial->tx_count+=bytes;

pnet_serial_write_ret:
	spin_unlock_bh(&pnet_serial->lock);

	dbg_line("%i/%i\n",bytes,count);

	return bytes;
}

/* returns room in fifo */
static int pnet_serial_write_room(struct tty_struct *tty) 
{
	int ret;
	_GET_PNET_SERIAL_
	dbg_line("\n");
	_CHECK_OPEN_COUNT_(-EINVAL);

	spin_lock_bh(&pnet_serial->lock);

	/* error if disconnected */
	if (pnet_serial->flags&PNET_SERIAL_FLAG_DISCONNECTED) {
		ret=-EPIPE;
		goto pnet_serial_write_room_ret;
	}

	/* transmit throttled */
	if (pnet_serial->flags&PNET_SERIAL_FLAG_TX_THROTTLED) {
		dbg_line("throttled ret=0\n");
		ret=0;
		goto pnet_serial_write_room_ret;
	}

	ret=pnet_core_port_write_free(&pnet_serial->pnet_channel.data_port);

pnet_serial_write_room_ret:
	spin_unlock_bh(&pnet_serial->lock);

	dbg_line("ret=%i\n",ret);

	return ret;
}


/* returns number chars in out-fifo */
static int pnet_serial_chars_in_buffer(struct tty_struct *tty) 
{
#if 1
	int ret;
	_GET_PNET_SERIAL_
	dbg_line("\n");
	_CHECK_OPEN_COUNT_(-EINVAL);

	spin_lock_bh(&pnet_serial->lock);
	ret = pnet_core_port_write_len(&pnet_serial->pnet_channel.data_port);
	spin_unlock_bh(&pnet_serial->lock);
	return ret;
#else
	return 0;
#endif
}

/* disable passing data from fifo to tty-layer */
static void pnet_serial_throttle (struct tty_struct * tty) 
{
	_GET_PNET_SERIAL_
	dbg_line("\n");
	spin_lock_bh(&pnet_serial->lock);
	pnet_serial->flags|=PNET_SERIAL_FLAG_RX_THROTTLED;
	spin_unlock_bh(&pnet_serial->lock);
}

/* enable passing data from fifo to tty-layer */
static void pnet_serial_unthrottle (struct tty_struct * tty) 
{
	_GET_PNET_SERIAL_
	dbg_line("\n");

	spin_lock_bh(&pnet_serial->lock);
	pnet_serial->flags&=~PNET_SERIAL_FLAG_RX_THROTTLED;

	/* error if disconnected */
	if(!(pnet_serial->flags&PNET_SERIAL_FLAG_DISCONNECTED)) {
		/* restart read */
		tasklet_schedule(&pnet_serial->dread_tl);
	}

	spin_unlock_bh(&pnet_serial->lock);
}


/* parse termios and set device */
static void pnet_serial_set_termios (struct tty_struct *tty, struct ktermios *old_termios)
{
	unsigned int arg;
	int ret;
	/* copy flags */
	unsigned int iflag;
	unsigned int cflag;
	unsigned int old_iflag = 0;
	unsigned int old_cflag = 0;
	/* settings */
	int control_state;
	union pnet_serial_flow_ctrl flowctrl;
	
	_GET_PNET_SERIAL_
	dbg_line("\n");

	_CHECK_OPEN_COUNT_();
	_CHECK_INT_CONTEXT_();

	if(down_interruptible(&pnet_serial->mutex)) {
		warn("mutex interrupted");
		return;
	}

	iflag = tty->termios->c_iflag;
	cflag = tty->termios->c_cflag;

	if (old_termios) {
		old_iflag = old_termios->c_iflag;
		old_cflag = old_termios->c_cflag;
	}

	/* get a local copy of the current port settings */
	control_state=pnet_serial_update_control_state(pnet_serial);
	if(control_state<0) {
		err("get ctrlstate error: %i",control_state);
		goto pnet_serial_set_termios_return;
	}

	/* PNET Set the baud rate if changed */
	if( (!old_termios) || ((cflag&CBAUD) != (old_cflag&CBAUD)) ) { 	
		if ((cflag & CBAUD) == B0 ) {
			dbg("set B0\n");

			/* disable flowctrl */			
			if(pnet_serial_disable_flowctrl(pnet_serial)) {
				err("error disable flowctrl");
				goto pnet_serial_set_termios_return;
			}

			/* clear rts and dtr */
			arg=0;
			ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_RTS,&arg);
			if(ret) {
				err("error clear rts: %i",ret);
				goto pnet_serial_set_termios_return;
			}

			arg=0;	
			ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_DTR,&arg);
			if(ret) {
				err("error clear dtr: %i",ret);
				goto pnet_serial_set_termios_return;
			}

			/* update control state */
			if(pnet_serial_update_control_state(pnet_serial)<0) {
				err("error getting control state: %i",ret);
				goto pnet_serial_set_termios_return;
			}

			/* disable port */
			arg=0;
			ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_ENABLE,&arg);
			if(ret) {
				err("failed to disable port err: %i",ret);
				goto pnet_serial_set_termios_return;
			}
			if(arg!=0) {
				err("remote failed to disable port");
				goto pnet_serial_set_termios_return;
			}

		} else {
			dbg("not B0\n");

			/* see baudrate-table in pnet_serial.h */
			switch(cflag & CBAUD) {
				case B50:		arg=0; break;
				case B75:		arg=1; break;
				case B110:		arg=2; break;
				case B134:		arg=3; break;
				case B150:		arg=4; break;
				case B200:		arg=5; break;
				case B300:		arg=6; break;
				case B600:		arg=7; break;
				case B1200:		arg=8; break;
				case B1800:		arg=9; break;
				case B2400:		arg=10; break;
				case B4800:		arg=11; break;
				case B9600:		arg=12; break;
				case B19200:	arg=13; break;
				case B38400:	arg=14; break;
				case B57600:	arg=15; break;
				case B115200:	arg=16; break;
//				case B230400:	arg=17; break;
//				case B460800:	arg=18; break;
				default: 
					err("unsupportet baud-rate -> default 9600");
					arg=12;
			}

			/* -> set baudrate  */
			ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_BAUDRATE,&arg);
			if(ret) {
				err("set baud error: %i",ret);
				goto pnet_serial_set_termios_return;
			}
		} /* else */
	}

	/* set the parity if changed */
	if( (!old_termios) || ( (cflag&(PARENB|PARODD)) != (old_cflag&(PARENB|PARODD)) ) ) {
		if (cflag & PARENB) {
			arg=(1<<0);
			arg|=(cflag & PARODD) ? (0<<1) : (1<<1);
		} else {
			arg=(0<<0);
		}
		
		ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_PARITY,&arg);
		if(ret) {
			err("set par error: %i",ret);
			goto pnet_serial_set_termios_return;
		}
	}

	/* set the number of data bits if changed */
	if( (!old_termios) || ( (cflag&CSIZE) != (old_cflag&CSIZE) ) ) {
		switch (cflag & CSIZE) {
			case CS5: arg=5; break;
			case CS6: arg=6; break;
			case CS7: arg=7; break;
			case CS8: arg=8; break;
			default: 
				err("CSIZE was not CS5-CS8, using default of 8");
				arg=8;
				break;
		}

		ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_DATAB,&arg);
		if(ret) {
			err("set datab error: %i",ret);
			goto pnet_serial_set_termios_return;
		}
	}

	/* set the number of stop bits if changed */
	if( (!old_termios) || ( (cflag&CSTOPB) != (old_cflag&CSTOPB) ) ) {
		/* -> set stop bits request */	
		arg=(cflag & CSTOPB) ? 1 : 0;
		ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_STOPB,&arg);
		if(ret) {
			err("set stopb error: %i",ret);
			goto pnet_serial_set_termios_return;
		}
	}

	/* Set flow control */
	dbg("set flowctrl: ixoff=%i, ixon=%i crtscts=%i clocal=%i\n",
		iflag&IXOFF,iflag&IXON,cflag&CRTSCTS,cflag&CLOCAL);

	flowctrl.val=0;

	/* start/stop (xon/xoff) chars */
	flowctrl.bits.vstart=tty->termios->c_cc[VSTART];
	flowctrl.bits.vstop=tty->termios->c_cc[VSTOP];

	/* output flow ctrl */
	flowctrl.bits.oxon = (iflag&IXOFF) ? 1 : 0;

	/* input flow ctrl */
	flowctrl.bits.ixon = (iflag&IXON) ? 1 : 0;

	/* input/output flow ctrl */
	flowctrl.bits.crtscts = (cflag&CRTSCTS) ? 1 : 0;

	/* use modem lines (dtr/cd) */
	flowctrl.bits.clocal = (cflag&CLOCAL) ? 1 : 0;

	/* -> set flowctrl */
	arg=flowctrl.val;
	ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_FLOWCTRL,&arg);
	if(ret) {
		err("set flowctrl error: %i",ret);
		goto pnet_serial_set_termios_return;
	}

	/* set receive enable/disable */
	if( (!old_termios) || ( (cflag&CREAD) != (old_cflag&CREAD) ) ) {
		/* -> set receive enabled */
		arg=cflag&CREAD;
		ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_RECEIVE,&arg);
		if(ret) {
			err("set rcv error: %i",ret);
			goto pnet_serial_set_termios_return;
		}
	}

	/* save off the modified port settings */
	pnet_serial->control_state = control_state;

pnet_serial_set_termios_return:
	up(&pnet_serial->mutex);
}


/* information in proc-fs */
static int pnet_serial_proc_show(struct seq_file *m, void *v)
{
	struct pnet_serial *pnet_serial;
	int i;
	unsigned int arg=0;

	dbg("\n");
    
	seq_printf(m, "%s(%s) %s\nMajor=%i, Minor=%i-%i\n",
		PNET_SERIAL_DRIVER_NAME, PNET_SERIAL_DRIVER_DESC, PNET_SERIAL_DRIVER_VERSION,
		PNET_SERIAL_TTY_MAJOR, PNET_SERIAL_TTY_MINOR, PNET_SERIAL_TTY_MINOR+PNET_SERIAL_TTY_MINORS-1);

	for(i=0;i<PNET_SERIAL_TTY_MINORS;i++) {
		pnet_serial=pnet_serial_table[i];

		if(down_interruptible(&pnet_serial->mutex)) {
			warn("mutex interrupted");
			return -ERESTARTSYS;
		}

		/* do post initialiation if neccessary */
		pnet_serial_try_post_init(pnet_serial);

		if(pnet_serial_is_connected(pnet_serial))
			if(pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_GET_TXCOUNT,&arg))
				arg=0;

		seq_printf(m, "%i (Minor %i):\n",i, PNET_SERIAL_TTY_MINOR+i);
		seq_printf(m, "    opencount=%i tx=%i(%i) rx=%i\n",
				pnet_serial->open,pnet_serial->tx_count,arg,pnet_serial->rx_count);
		seq_printf(m, "    connected: %s, tx_throttled: %i, rx_throttled %i\n",
				pnet_serial_is_connected(pnet_serial) ? "yes" : "no", 
				(pnet_serial->flags&PNET_SERIAL_FLAG_TX_THROTTLED)>0,
				(pnet_serial->flags&PNET_SERIAL_FLAG_RX_THROTTLED)>0
			);


		up(&pnet_serial->mutex);
	}

	return 0;
}


/* enable/disable break status of rs232
 * is called by user-ioctl (extra functionality -> for special applications)
*/
static int pnet_serial_break_ctl(struct tty_struct *tty, int break_state) 
{
	int ret,val;

	_GET_PNET_SERIAL_

	dbg_line("break %s\n", break_state==-1 ? "on" : "off");

	_CHECK_OPEN_COUNT_(-EINVAL);
	_CHECK_INT_CONTEXT_(-EPERM);
	/* error if disconnected */
	if (pnet_serial->flags&PNET_SERIAL_FLAG_DISCONNECTED) {
		return 0; /* todo */
	}

	if(down_interruptible(&pnet_serial->mutex)) {
		warn("mutex interrupted");
		return -ERESTARTSYS;
	}

	if(break_state==-1)
		val=1;
	else
		val=0;

	ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_BREAK_CTL,&val);
	if(ret)
		err("error: %i",ret);

	up(&pnet_serial->mutex);

	return 0;
}


/* get state of control-lines 
is called by user-ioctl (extra functionality -> for special applications)
*/
static int pnet_serial_tiocmget (struct tty_struct *tty, struct file *file) 
{
	int control_state;

	_GET_PNET_SERIAL_

	dbg_line("\n");

	_CHECK_OPEN_COUNT_(-EINVAL);
	_CHECK_INT_CONTEXT_(-EPERM);
	/* error if disconnected */
	if (pnet_serial->flags&PNET_SERIAL_FLAG_DISCONNECTED) {
		return -EPIPE;
	}

	if(down_interruptible(&pnet_serial->mutex)) {
		warn("mutex interrupted");
		return -ERESTARTSYS;
	}

	control_state=pnet_serial_update_control_state(pnet_serial);
	if(control_state<0)
		control_state=-EIO;

	up(&pnet_serial->mutex);

	return control_state;
}


/* set state of control-lines
is called by ioctl (extra functionality -> for special applications)
*/
static int pnet_serial_tiocmset (struct tty_struct *tty, struct file *file,
			    unsigned int set, unsigned int clear) 
{
	int control_state;
	int ret;
	int rts = 0;
	int dtr = 0;

	_GET_PNET_SERIAL_

	dbg_line("\n");

	_CHECK_OPEN_COUNT_(-EINVAL);
	_CHECK_INT_CONTEXT_(-EPERM);
	/* error if disconnected */
	if (pnet_serial->flags&PNET_SERIAL_FLAG_DISCONNECTED) {
		return -EPIPE;
	}

	if(down_interruptible(&pnet_serial->mutex)) {
		warn("mutex interrupted");
		return -ERESTARTSYS;
	}

	control_state = pnet_serial_update_control_state(pnet_serial);
	if(control_state<0) {
		up(&pnet_serial->mutex);
		return -EIO;
	}
	
	if (set & TIOCM_RTS) {
		control_state |= TIOCM_RTS;
		rts = 1;
	}
	if (set & TIOCM_DTR) {
		control_state |= TIOCM_DTR;
		dtr = 1;
	}
	if (clear & TIOCM_RTS) {
		control_state &= ~TIOCM_RTS;
		rts = 0;
	}
	if (clear & TIOCM_DTR) {
		control_state &= ~TIOCM_DTR;
		dtr = 0;
	}

	if(pnet_serial->control_state != control_state) {

		pnet_serial->control_state = control_state;

		/* set rts and dtr request */
		ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_RTS,&rts);
		if(ret) {
			err("error set rts: %i",ret);
			up(&pnet_serial->mutex);
			return ret;
		}

		ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_DTR,&dtr);
		if(ret) {
			err("error set dtr: %i",ret);
			up(&pnet_serial->mutex);
			return ret;
		}
	}

	up(&pnet_serial->mutex);

	return 0;
}

#if 0
static int pnet_serial_ioctl(struct tty_struct *tty, struct file * file, unsigned int cmd, unsigned long arg)
{
	/* TODO */
	return(-ENOIOCTLCMD);
}
#endif


/*
++++++++++++++++++++++++++++++++++
FUNCTIONS (module/driver-reg)
++++++++++++++++++++++++++++++++++
*/

static int pnet_serial_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, pnet_serial_proc_show, NULL);
}

static const struct file_operations pnet_serial_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= pnet_serial_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct tty_operations pnet_serial_ops = {
	.open =				pnet_serial_open,
	.close =			pnet_serial_close,
	.write =			pnet_serial_write,
	.write_room =		pnet_serial_write_room,
/*	.ioctl =			pnet_serial_ioctl,			*/
	.set_termios =		pnet_serial_set_termios,
	.throttle =			pnet_serial_throttle,
	.unthrottle =		pnet_serial_unthrottle,
	.break_ctl =		pnet_serial_break_ctl,
	.chars_in_buffer =	pnet_serial_chars_in_buffer,
	.tiocmget =			pnet_serial_tiocmget,
	.tiocmset =			pnet_serial_tiocmset,
	.proc_fops = 		&pnet_serial_proc_fops,
};


/* driver post-initialisation (done once per driver-load in open) */
static void pnet_serial_try_post_init(struct pnet_serial *pnet_serial)
{
	int arg;

	/* already done -> do nothing */
	if(pnet_serial->flags&PNET_SERIAL_FLAG_POST_INIT)
		return;

	/* sync tx-counter */
	if(pnet_serial_is_connected(pnet_serial))
		if(pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_GET_TXCOUNT,&arg)==0)
			pnet_serial->tx_count=arg;

	/* set flag */
	pnet_serial->flags|=PNET_SERIAL_FLAG_POST_INIT;
}

/* register pnet ports */
static struct pnet_serial * __init pnet_serial_register_port(int line) 
{
	struct pnet_serial *pnet_serial;

	/* alloc struct */
	if(!(pnet_serial=kmalloc(sizeof(struct pnet_serial),GFP_KERNEL)))
		return 0;

	/* clear structure */
	memset(pnet_serial,0,sizeof(struct pnet_serial));

	/* init mutex */
	init_MUTEX(&pnet_serial->mutex);

	if(down_interruptible(&pnet_serial->mutex)) {
		warn("mutex interrupted");
		goto pnet_serial_register_port_err0;
	}


	pnet_serial->flags&=~PNET_SERIAL_FLAG_UNREAD_CREPLY;

	/* start tx/rx !throttled */
	pnet_serial->flags&=~(PNET_SERIAL_FLAG_TX_THROTTLED|PNET_SERIAL_FLAG_RX_THROTTLED);


	pnet_serial->pnet_channel.data_port.owner = THIS_MODULE;
	pnet_serial->pnet_channel.data_port.adr=0;
	pnet_serial->pnet_channel.data_port.pri=3;
	pnet_serial->pnet_channel.data_port.port=PNET_SERIAL_PNET_PORT_START+0+(line*2);
	pnet_serial->pnet_channel.data_port.rx_size=2048;
	pnet_serial->pnet_channel.data_port.tx_size=2048;
	pnet_serial->pnet_channel.data_port.tx_order_size=4096;
	pnet_serial->pnet_channel.data_port.sent_data=pnet_serial_dport_sent;
	pnet_serial->pnet_channel.data_port.received_data=pnet_serial_dport_received;
	pnet_serial->pnet_channel.data_port.changed_state=pnet_serial_changed_state;
	pnet_serial->pnet_channel.data_port.private_data=(void*)pnet_serial;
	pnet_serial->pnet_channel.data_port.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT;


	pnet_serial->pnet_channel.ctrl_port.owner = THIS_MODULE;
	pnet_serial->pnet_channel.ctrl_port.adr=0;
	pnet_serial->pnet_channel.ctrl_port.pri=2;
	pnet_serial->pnet_channel.ctrl_port.port=PNET_SERIAL_PNET_PORT_START+1+(line*2);
	pnet_serial->pnet_channel.ctrl_port.rx_size=64;
	pnet_serial->pnet_channel.ctrl_port.tx_size=64;
	pnet_serial->pnet_channel.ctrl_port.tx_order_size=512;
	pnet_serial->pnet_channel.ctrl_port.sent_data=pnet_serial_cport_sent;
	pnet_serial->pnet_channel.ctrl_port.received_data=pnet_serial_cport_received;
	pnet_serial->pnet_channel.ctrl_port.changed_state=pnet_serial_changed_state;
	pnet_serial->pnet_channel.ctrl_port.private_data=(void*)pnet_serial;
	pnet_serial->pnet_channel.ctrl_port.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT;

	/* init data-read tl */
	tasklet_init(&pnet_serial->dread_tl,pnet_serial_dread_tl_proc,(unsigned long)pnet_serial);

	/* init ctrl-read tl */
	tasklet_init(&pnet_serial->cread_tl,pnet_serial_cread_tl_proc,(unsigned long)pnet_serial);
	
	/* init data-sent wq */
	init_waitqueue_head(&pnet_serial->sent_wq);
	/* init ctrl-send/received wq */
	init_waitqueue_head(&pnet_serial->csent_wq); 
	init_waitqueue_head(&pnet_serial->crcv_wq);
	/* init connection state changed wq */
	init_waitqueue_head(&pnet_serial->conn_state_changed_wq);    

	/* init spinlock */
	spin_lock_init(&pnet_serial->lock);

	/* init fifo */
	dbg("connect line %i to pnet-frawework\n",line);

	if(pnet_core_port_register(&pnet_serial->pnet_channel.data_port)) {
		err("Unable to register data-port");
		goto pnet_serial_register_port_err1;
	}

	if(pnet_core_port_register(&pnet_serial->pnet_channel.ctrl_port)) {
		err("Unable to register ctrl-port");
		goto pnet_serial_register_port_err2;
	}

	/* ok */
	up(&pnet_serial->mutex);
	return pnet_serial;

pnet_serial_register_port_err2:
	pnet_core_port_unregister(&pnet_serial->pnet_channel.data_port);
pnet_serial_register_port_err1:
	up(&pnet_serial->mutex);
pnet_serial_register_port_err0:
	kfree(pnet_serial);
	up(&pnet_serial->mutex);
	return 0;
}

/* unregister pnet ports */
static int pnet_serial_unregister_port(struct pnet_serial *pnet_serial) 
{
	int ret,arg;

	if(!pnet_serial) 
		return -EINVAL;

	/* lock -> if signal unregister also (but dirty) */
	if(down_interruptible(&pnet_serial->mutex)) {
		err("Signal while unregister");
	}

	/* disable remote */
	arg=0;
	ret=pnet_serial_do_cmd(pnet_serial,PNET_SERIAL_CMD_SET_ENABLE,&arg);
	if(ret)
		err("failed to disable port err: %i",ret);
	if(arg!=0) {
		err("remote failed to disable port");
	}

	tasklet_kill(&pnet_serial->dread_tl);
	tasklet_kill(&pnet_serial->cread_tl);

	/* deinit */
	dbg("disconnect from pnet-frawework\n");

	/* disable receive */
	ret=pnet_core_port_unregister(&pnet_serial->pnet_channel.data_port);
	if(ret) 
		err("Unable to unregister data-port");

	ret=pnet_core_port_unregister(&pnet_serial->pnet_channel.ctrl_port);
	if(ret) 
		err("Unable to unregister ctrl-port");
	

	/* unlock before free */
	up(&pnet_serial->mutex);

	/* free */
	kfree(pnet_serial);

	return ret;
}


/* tty-driver structure */
struct tty_driver *pnet_serial_tty_driver;

/* module_init */
static int __init pnet_serial_init(void) 
{
	int i;
	int result = 0;

	dbg("\n");
	_CHECK_INT_CONTEXT_(-EPERM);
	
	/* allocate the tty-driver structure */
	pnet_serial_tty_driver = alloc_tty_driver(PNET_SERIAL_TTY_MINORS);
	if (!pnet_serial_tty_driver)
		return -ENOMEM;

	/* set tty-driver */
	pnet_serial_tty_driver->owner		= THIS_MODULE;
	pnet_serial_tty_driver->driver_name	= PNET_SERIAL_DRIVER_NAME;
	pnet_serial_tty_driver->name		= PNET_SERIAL_TTY_NAME_DEV;
	pnet_serial_tty_driver->major		= PNET_SERIAL_TTY_MAJOR;
	pnet_serial_tty_driver->minor_start	= PNET_SERIAL_TTY_MINOR;
	pnet_serial_tty_driver->type		= TTY_DRIVER_TYPE_SERIAL;
	pnet_serial_tty_driver->subtype		= SERIAL_TYPE_NORMAL;
	pnet_serial_tty_driver->flags		= TTY_DRIVER_REAL_RAW;
	pnet_serial_tty_driver->init_termios= tty_std_termios;
	pnet_serial_tty_driver->init_termios.c_cflag 
						= B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	
//	tty_register_ldisc; /* ??? */
	
	/* set operations on tty-driver */
	tty_set_operations(pnet_serial_tty_driver, &pnet_serial_ops);
	
	/* register the driver */
	result = tty_register_driver(pnet_serial_tty_driver);
	if (result) {
		err("tty_register_driver failed");
		goto exit_generic;
	}

	/* register ports */
	for(i=0;i<PNET_SERIAL_TTY_MINORS; i++) {
		/* register port */
		dbg("init port %i\n",i);
		pnet_serial_table[i]=pnet_serial_register_port(i);

		if(!pnet_serial_table[i]) {
			err("Failed to register port %i",i);
			/* unregister already registered ports */
			while(i--)
				pnet_serial_unregister_port(pnet_serial_table[i]);
			/* exit */
			result=-ENODEV;
			goto exit_unregister;
		}
		pnet_serial_table[i]->tty_driver=pnet_serial_tty_driver;
	}

	/* exit with no error */
	return 0;


exit_unregister:
	/* on error */
	tty_unregister_driver(pnet_serial_tty_driver);
exit_generic:
	/* on error */
	err("returning with error %d", result);
	put_tty_driver(pnet_serial_tty_driver);
	return result;
}


/* module_exit */
static void __exit pnet_serial_exit(void) 
{
	int i;

	dbg("\n");
	_CHECK_INT_CONTEXT_();

	/* unregister ports */
	for(i=0;i<PNET_SERIAL_TTY_MINORS;i++) {
		dbg("Free port %i\n",i);
		if(pnet_serial_unregister_port(pnet_serial_table[i]))
			err("Failed to unregister port %i",i);
	}

	/* unregister driver */
	tty_unregister_driver(pnet_serial_tty_driver);
	/* free structure */
	put_tty_driver(pnet_serial_tty_driver);
}



/*
++++++++++++++++++++++++++++++++++
MODULE
++++++++++++++++++++++++++++++++++
*/

module_init(pnet_serial_init);
module_exit(pnet_serial_exit);

/* debug-parameter */
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");

/* Module information */
MODULE_AUTHOR( PNET_SERIAL_DRIVER_AUTHOR );
MODULE_DESCRIPTION( PNET_SERIAL_DRIVER_DESC );
MODULE_LICENSE("GPL");

