/***********************************************************************
 *
 * @Authors: Manfred Schlägl, Ginzinger electronic systems GmbH
 * @Descr: gpio driver for pnet
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
 *	2010-KW06 manfred.schlaegl@gmx.at
 *		* history reordered
 *		* registers: interruptible waits replaced by timeout waits
 *		* better api-comments
 *	2009-KW52 manfred.schlaegl@gmx.at
 *		* no wakup while lock
 *		* wrong spin-lock usage corrected
 *	2009-KW36 manfred.schlaegl@gmx.at
 *		* direct-api as concurrent of linux-driver
 *		* callbacks for direct-api
 *  2009-KW24 manfred.schlaegl@gmx.at
 *      * flowcontrol
 *  2009-KW18 manfred.schlaegl@gmx.at
 *      * commit and commit_ack registers
 *      * commit_ack
 *	2009-KW17 manfred.schlaegl@gmx.at
 *		* development start (based on pnet_rdev)
 ***********************************************************************/
/***********************************************************************
 *  @TODO:
 *	  * Error corrections
 *	  * Remove semaphore in ioctl
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
#include "pnet_core.h"
#include "pnet_rdev.h"

/*
 **************************************************************************************************
 * Begin: Basic Definitions
 **************************************************************************************************
 */

#define DRV_MAJOR  										245
#define DRV_NAME										"pnet_rdev"

/* enable debug output */
#ifdef CONFIG_PNET_RDEV_DEBUG
#define DEBUG
#endif
//#define DEBUG
/* enable datafifo debug */
//#define DEBUG_DATAFIFO


#define FLOW_CONTROL

#define PNET_RDEV_DEV_N									4

/* start-port number */
#define PNET_RDEV_PORT_START							11	

/* number special registers */
#define PNET_RDEV_SPECIAL_REGISTERS						10

/* commit register */
#define PNET_RDEV_SPECIAL_REGNR_COMMIT					5
/* commit_ack register */
#define PNET_RDEV_SPECIAL_REGNR_COMMIT_ACK				6
/* flowctrl register */
#define PNET_RDEV_SPECIAL_REGNR_FLOWCTRL				7

/* register-number for datafifo */
#define PNET_RDEV_SPECIAL_REGNR_DATAFIFO_COUNT			(sizeof(pnet_rdev_register_t))	/* 4 for unsigned int */

/* commit states */
#define PNET_RDEV_COMMIT_STATE_IDLE						0
#define PNET_RDEV_COMMIT_STATE_COMMIT					1
#define PNET_RDEV_COMMIT_STATE_WAIT_COMMIT_ACK			2
/* commit_ack states */
#define PNET_RDEV_COMMIT_ACK_STATE_IDLE					0
#define PNET_RDEV_COMMIT_ACK_STATE_COMMIT_ACK			1
/* send flowctrl states */
#define PNET_RDEV_FLOWCTRL_STATE_IDLE					0
#define PNET_RDEV_FLOWCTRL_STATE_SEND_MAX_SPACE			1
#define PNET_RDEV_FLOWCTRL_STATE_SEND_RECEIVED_BYTES	2

/*
 **************************************************************************************************
 * End: Basic Definitions
 **************************************************************************************************
 */


/*
 **************************************************************************************************
 * Begin: IOCTL-Definitions
 **************************************************************************************************
 */


#define PNET_RDEV_REGISTER_IOCTL_RESET					1
#define PNET_RDEV_REGISTER_IOCTL_READ					2
#define PNET_RDEV_REGISTER_IOCTL_IS_CHANGED				3
#define PNET_RDEV_REGISTER_IOCTL_WAIT_REMOTE_COMMIT		5
#define PNET_RDEV_REGISTER_IOCTL_WRITE					6
#define PNET_RDEV_REGISTER_IOCTL_COMMIT					7


struct pnet_rdev_register_ioctl_data {
	unsigned char regnr;
	unsigned char changed;
	pnet_rdev_register_t value;
};


/*
 **************************************************************************************************
 * End: IOCTL-Definitions
 **************************************************************************************************
 */


/*
 **************************************************************************************************
 * Begin: Driver data
 **************************************************************************************************
 */

/* pnet register packet for communication */
#define PNET_RDEV_REGISTER_PACKET__PACKED_SIZE		(sizeof(unsigned int)+sizeof(unsigned char))

inline static void pnet_rdev_register_packet_pack(char *pnet_rdev_register_packet, unsigned char regnr, unsigned int value)
{
	pnet_rdev_register_packet[0]=regnr;
	pnet_rdev_register_packet[1]=(value>>24)&0xff;
	pnet_rdev_register_packet[2]=(value>>16)&0xff;
	pnet_rdev_register_packet[3]=(value>>8)&0xff;
	pnet_rdev_register_packet[4]=(value>>0)&0xff;
}

inline static void pnet_rdev_register_packet_unpack(char *pnet_rdev_register_packet, unsigned char *regnr, unsigned int *value)
{
	(*regnr)=pnet_rdev_register_packet[0];
	(*value)=
		(pnet_rdev_register_packet[1]<<24) |
		(pnet_rdev_register_packet[2]<<16) |
		(pnet_rdev_register_packet[3]<<8)  |
		(pnet_rdev_register_packet[4]<<0);
}

static pnet_rdev_private_t pnet_rdev_private[PNET_RDEV_DEV_N];

static void pnet_rdev_port_sent_data(struct pnet_core_port *port);
static void pnet_rdev_port_received_data(struct pnet_core_port *port);
static void pnet_rdev_port_changed_state(struct pnet_core_port *port, int state);

static struct pnet_core_port pnet_rdev_port[PNET_RDEV_DEV_N] = 
{
/* minor0 : system-rdev (todo) */
{
	.owner = THIS_MODULE,
	.adr=0,
	.pri=3,
	.port=PNET_RDEV_PORT_START,
	.rx_size=1024,
	.tx_size=1024,
	.tx_order_size=256,
	.sent_data=pnet_rdev_port_sent_data,
	.received_data=pnet_rdev_port_received_data,
	.changed_state=pnet_rdev_port_changed_state,
	.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT,
},
/* minor1 : pnet_io */
{
	.owner = THIS_MODULE,
	.adr=0,
	.pri=1,
	.port=PNET_RDEV_PORT_START+1,
	.rx_size=1024,
	.tx_size=1024,
	.tx_order_size=256,
	.sent_data=pnet_rdev_port_sent_data,
	.received_data=pnet_rdev_port_received_data,
	.changed_state=pnet_rdev_port_changed_state,
	.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT,
},
/* minor2 : pnet_can */
{
	.owner = THIS_MODULE,
	.adr=0,
	.pri=2,
	.port=PNET_RDEV_PORT_START+2,
	.rx_size=1024,
	.tx_size=1024,
	.tx_order_size=256,
	.sent_data=pnet_rdev_port_sent_data,
	.received_data=pnet_rdev_port_received_data,
	.changed_state=pnet_rdev_port_changed_state,
	.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT,
},
/* minor3: custom-extension */
{
	.owner = THIS_MODULE,
	.adr=0,
	.pri=0,
	.port=PNET_RDEV_PORT_START+3,
	.rx_size=1024,
	.tx_size=1024,
	.tx_order_size=256,
	.sent_data=pnet_rdev_port_sent_data,
	.received_data=pnet_rdev_port_received_data,
	.changed_state=pnet_rdev_port_changed_state,
	.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT,
}};

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
 * End: Driver data
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Datafifo-stuff
 **************************************************************************************************
 */

/* begin: register - datafifo functions */

/* 
 * reset the whole fifo 
 */
inline static void pnet_rdev_datafifo_reset(struct pnet_rdev_datafifo *fifo)
{
	spin_lock_bh(&fifo->slock);
	fifo->iptr=0;
	fifo->optr=0;
	fifo->size=0;
	spin_unlock_bh(&fifo->slock);
}

/*
 * get fifo-len
 * returns: number of elements in fifo
 */
inline static int pnet_rdev_datafifo_len(struct pnet_rdev_datafifo *fifo)
{
	int ret;
	spin_lock_bh(&fifo->slock);
	ret=fifo->size;
	spin_unlock_bh(&fifo->slock);
	return ret;
}

/*
 * get fifo-free
 * returns: free space in fifo
 */
inline static int pnet_rdev_datafifo_free(struct pnet_rdev_datafifo *fifo)
{
	return PNET_RDEV_DATAFIFO_SIZE-pnet_rdev_datafifo_len(fifo);
}

/*
 * put one element in fifo
 * returns: -ENOMEM .. no space; 0 .. ok
 */
static int pnet_rdev_datafifo_put(struct pnet_rdev_datafifo *fifo, char value)
{
	spin_lock_bh(&fifo->slock);

	/* check */
	if(fifo->size>=PNET_RDEV_DATAFIFO_SIZE) {
		spin_unlock_bh(&fifo->slock);
		return -ENOMEM;
	}

	/* add */
	fifo->data[fifo->iptr++]=value;
	if(fifo->iptr>=PNET_RDEV_DATAFIFO_SIZE)
		fifo->iptr=0;
	fifo->size++;

	spin_unlock_bh(&fifo->slock);
	return 0;
}

/*
 * get one element from fifo 
 * returns: -ENODATA .. no data in fifo; 0 .. ok
 */
static int pnet_rdev_datafifo_get(struct pnet_rdev_datafifo *fifo, char *value)
{
	spin_lock_bh(&fifo->slock);

	/* check */
	if(fifo->size==0) {
		spin_unlock_bh(&fifo->slock);
		return -ENODATA;
	}

	/* remove */
	*value=fifo->data[fifo->optr++];
	if(fifo->optr>=PNET_RDEV_DATAFIFO_SIZE)
		fifo->optr-=PNET_RDEV_DATAFIFO_SIZE;
	fifo->size--;

	spin_unlock_bh(&fifo->slock);
	return 0;
}

/* 
 * write elements to fifo
 * returns: number of written bytes 
 */
static int pnet_rdev_datafifo_write(struct pnet_rdev_datafifo *fifo, char *data, unsigned int len)
{
	int write_size;
	int elem_end,elem_begin;

	spin_lock_bh(&fifo->slock);

#ifdef DEBUG_DATAFIFO
	int i;
	char buf[255];
	for(i=0;i<len;i++)
		buf[i]=data[i];
	buf[i]=0;
	printk("sys read:    %s\n",buf);
#endif

	/* calc size to write */
	write_size=MIN(PNET_RDEV_DATAFIFO_SIZE-fifo->size,len);

	if(PNET_RDEV_DATAFIFO_SIZE-fifo->iptr >= write_size) {
		/* copy in one cycle */
		memcpy(&fifo->data[fifo->iptr],&data[0],write_size);
	} else {
		/* copy needs two cycles */
		dbg("write 2 cycles");
		elem_end=PNET_RDEV_DATAFIFO_SIZE-fifo->iptr;
		elem_begin=write_size-elem_end;
		memcpy(&fifo->data[fifo->iptr],&data[0],elem_end);
		memcpy(&fifo->data[0],&data[elem_end],elem_begin);
	}

	fifo->iptr+=write_size;
	if(fifo->iptr>=PNET_RDEV_DATAFIFO_SIZE)
		fifo->iptr-=PNET_RDEV_DATAFIFO_SIZE;
	fifo->size+=write_size;

	spin_unlock_bh(&fifo->slock);

	return write_size;
}

/* 
 * write elements from userspace to fifo
 * returns: number of written bytes 
 */
static int pnet_rdev_datafifo_write__user(struct pnet_rdev_datafifo *fifo, const char __user *data, unsigned int len)
{
	int write_size;
	int elem_end,elem_begin;

	spin_lock_bh(&fifo->slock);

#ifdef DEBUG_DATAFIFO
	int i;
	char buf[255];
	for(i=0;i<len;i++)
		buf[i]=data[i];
	buf[i]=0;
	printk("user write:  %s\n",buf);
#endif

	/* calc size to write */
	write_size=MIN(PNET_RDEV_DATAFIFO_SIZE-fifo->size,len);

	if(PNET_RDEV_DATAFIFO_SIZE-fifo->iptr >= write_size) {
		/* copy in one cycle */
		if(copy_from_user(&fifo->data[fifo->iptr],&data[0],write_size)) {
			err("copy_from_user failed");
			write_size=-EFAULT;
			goto __ret;
		}
	} else {
		/* copy needs two cycles */
		dbg("write 2 cycles");
		elem_end=PNET_RDEV_DATAFIFO_SIZE-fifo->iptr;
		elem_begin=write_size-elem_end;
		if(copy_from_user(&fifo->data[fifo->iptr],&data[0],elem_end)) {
			err("copy_from_user failed");
			write_size=-EFAULT;
			goto __ret;
		}			
		if(copy_from_user(&fifo->data[0],&data[elem_end],elem_begin)) {
			err("copy_from_user failed");
			write_size=-EFAULT;
			goto __ret;
		}
	}

	fifo->iptr+=write_size;
	if(fifo->iptr>=PNET_RDEV_DATAFIFO_SIZE)
		fifo->iptr-=PNET_RDEV_DATAFIFO_SIZE;
	fifo->size+=write_size;

__ret:
	spin_unlock_bh(&fifo->slock);

	return write_size;
}


/* 
 * read elements from fifo
 * returns: number of read bytes 
 */
static int pnet_rdev_datafifo_read(struct pnet_rdev_datafifo *fifo, char *data, unsigned int len)
{
	int read_size;
	int elem_end,elem_begin;

	spin_lock_bh(&fifo->slock);

	/* calc size to read */
	read_size=MIN(fifo->size,len);

	if(PNET_RDEV_DATAFIFO_SIZE-fifo->optr >= read_size) {
		/* copy in one cycle */
		memcpy(&data[0],&fifo->data[fifo->optr],read_size);
	} else {
		/* copy needs two cycles */
		dbg("read 2 cycles");
		elem_end=PNET_RDEV_DATAFIFO_SIZE-fifo->optr;
		elem_begin=read_size-elem_end;
		memcpy(&data[0],&fifo->data[fifo->optr],elem_end);
		memcpy(&data[elem_end],&fifo->data[0],elem_begin);
	}

	fifo->optr+=read_size;
	if(fifo->optr>=PNET_RDEV_DATAFIFO_SIZE)
		fifo->optr-=PNET_RDEV_DATAFIFO_SIZE;
	fifo->size-=read_size;

#ifdef DEBUG_DATAFIFO
	int i;
	char buf[255];
	for(i=0;i<read_size;i++)
		buf[i]=data[i];
	buf[i]=0;
	printk("sys read:    %s\n",buf);
#endif


	spin_unlock_bh(&fifo->slock);

	return read_size;
}

/* 
 * read elements from fifo to userspace
 * returns: number of read bytes 
 */
static int pnet_rdev_datafifo_read__user(struct pnet_rdev_datafifo *fifo, char __user *data, unsigned int len)
{
	int read_size;
	int elem_end,elem_begin;

	spin_lock_bh(&fifo->slock);

	/* calc size to read */
	read_size=MIN(fifo->size,len);

	if(PNET_RDEV_DATAFIFO_SIZE-fifo->optr >= read_size) {
		/* copy in one cycle */
		if(copy_to_user(&data[0],&fifo->data[fifo->optr],read_size)) {
			err("copy_to_user failed");
			read_size=-EFAULT;
			goto __ret;
		}
	} else {
		/* copy needs two cycles */
		dbg("read 2 cycles");
		elem_end=PNET_RDEV_DATAFIFO_SIZE-fifo->optr;
		elem_begin=read_size-elem_end;
		if(copy_to_user(&data[0],&fifo->data[fifo->optr],elem_end)) {
			err("copy_to_user failed");
			read_size=-EFAULT;
			goto __ret;
		}			
		if(copy_to_user(&data[elem_end],&fifo->data[0],elem_begin)) {
			err("copy_to_user failed");
			read_size=-EFAULT;
			goto __ret;
		}			
	}

	fifo->optr+=read_size;
	if(fifo->optr>=PNET_RDEV_DATAFIFO_SIZE)
		fifo->optr-=PNET_RDEV_DATAFIFO_SIZE;
	fifo->size-=read_size;

#ifdef DEBUG_DATAFIFO
	int i;
	char buf[255];
	for(i=0;i<read_size;i++)
		buf[i]=data[i];
	buf[i]=0;
	printk("user read:   %s\n",buf);
#endif


__ret:
	spin_unlock_bh(&fifo->slock);

	return read_size;
}

/*
 **************************************************************************************************
 * End: Datafifo-stuff
 **************************************************************************************************
 */


/*
 **************************************************************************************************
 * Begin: Register-stuff
 **************************************************************************************************
 */

/* begin: register - changed-flag functions */

inline static void pnet_rdev_register_reset_changed(struct pnet_rdev_registerset *registerset)
{
	int i;
	for(i=0;i<((PNET_RDEV_REGISTER_MAX+31)/32);i++)
		registerset->changed[i]=0;
}

inline static void pnet_rdev_register_set_changed(struct pnet_rdev_registerset *registerset, int regnr)
{
	int regnr_hi, regnr_lo;
	regnr_hi=regnr/32;
	regnr_lo=regnr-(regnr_hi*32);

	registerset->changed[regnr_hi]|=(1<<regnr_lo);
}

inline static void pnet_rdev_register_clear_changed(struct pnet_rdev_registerset *registerset, int regnr)
{
	int regnr_hi, regnr_lo;
	regnr_hi=regnr/32;
	regnr_lo=regnr-(regnr_hi*32);

	registerset->changed[regnr_hi]&=~(1<<regnr_lo);
}

inline static int pnet_rdev_register_any_changed(struct pnet_rdev_registerset *registerset)
{
	int i,ret=0;
	for(i=0;i<((PNET_RDEV_REGISTER_MAX+31)/32);i++)
		ret|=registerset->changed[i];
	return ret;
}

inline static int pnet_rdev_register_get_changed(struct pnet_rdev_registerset *registerset, int regnr)
{
	int regnr_hi, regnr_lo;
	regnr_hi=regnr/32;
	regnr_lo=regnr-(regnr_hi*32);
	return ( registerset->changed[regnr_hi]&(1<<regnr_lo) ) ? 1 : 0;
}

inline static int pnet_rdev_register_test_clear_changed(struct pnet_rdev_registerset *registerset, int regnr)
{
	int regnr_hi, regnr_lo, ret;
	regnr_hi=regnr/32;
	regnr_lo=regnr-(regnr_hi*32);
	ret = ( registerset->changed[regnr_hi]&(1<<regnr_lo) ) ? 1 : 0;
	registerset->changed[regnr_hi]&=~(1<<regnr_lo);
	return ret;
}

/*
 * receive registers (reader task)
 */
int pnet_rdev_register_receive(pnet_rdev_private_t *pnet_rdev)
{
	unsigned int tmp;
	int ret,req_reg_cb,req_dat_cb;
	unsigned char regnr;
	unsigned int regnr_lo, regnr_hi, value;
	char rcvbuffer[PNET_RDEV_REGISTER_PACKET__PACKED_SIZE];
	ret=0;

	dbg("adr=%d,port=%d", pnet_rdev->port->adr, pnet_rdev->port->port);

	/* reset callback requests */
	req_reg_cb=0;
	req_dat_cb=0;

	/* if connected & data */
	while(
/*		(!pnet_rdev->disconnected) && */
		pnet_core_port_read_len(pnet_rdev->port)>=PNET_RDEV_REGISTER_PACKET__PACKED_SIZE
	) {
		/* read packet */
		if(pnet_core_port_read(pnet_rdev->port,(void*)rcvbuffer,PNET_RDEV_REGISTER_PACKET__PACKED_SIZE)<PNET_RDEV_REGISTER_PACKET__PACKED_SIZE) {
			err("adr=%d,port=%d: error reading from port (disconnect)", pnet_rdev->port->adr, pnet_rdev->port->port);
			ret=-EPIPE;
			goto __ret;
		}

		/* unpack */
		pnet_rdev_register_packet_unpack(rcvbuffer,&regnr,&value);
		dbg("adr=%d,port=%d: received: regnr=%i value=%i",pnet_rdev->port->adr, pnet_rdev->port->port,regnr,value);

		/* is internal(special) register ? -> handle */
		if(regnr<PNET_RDEV_SPECIAL_REGISTERS) {

			/* data received? -> add to datafifo */
			if(regnr<PNET_RDEV_SPECIAL_REGNR_DATAFIFO_COUNT) {
				/* register nr is load
				 * special_regnr==0 -> 1 byte load
				 * special_regnr==1 -> 2 byte load
				 * special_regnr==2 -> 3 byte load
				 * special_regnr==3 -> 4 byte load
				 * ...
				*/
				dbg("adr=%d,port=%d: received data: %i bytes: 0x%X",pnet_rdev->port->adr, pnet_rdev->port->port,regnr+1,value);
				pnet_rdev_datafifo_write(&pnet_rdev->datafifo_rx, &rcvbuffer[1], regnr+1);

				/* request call of data-callback received */
				req_dat_cb=PNET_RDEV_CALLBACK_EV_RECEIVED;

				goto __continue;
			}

			/* commit received -> do commit */
			if(regnr==PNET_RDEV_SPECIAL_REGNR_COMMIT) {
#ifdef PNET_RDEV_DEBUG_STATE_COUNTERS
				pnet_rdev->received_commits++;
#endif

				dbg("adr=%d,port=%d: received commit",pnet_rdev->port->adr, pnet_rdev->port->port);
				spin_lock_bh(&pnet_rdev->slock);

				/* update register */
				for(regnr_hi=0;regnr_hi<((PNET_RDEV_REGISTER_MAX+31)/32);regnr_hi++) {
					tmp=pnet_rdev->regset_remote.changed[regnr_hi];
					for(regnr_lo=0; (regnr_lo<32) && tmp;regnr_lo++) {
						if(tmp&1) {
							/* register changed -> update */
							regnr=(regnr_hi*32)+regnr_lo;
							pnet_rdev->regset.reg[regnr]=pnet_rdev->regset_remote.reg[regnr];
							pnet_rdev_register_set_changed(&pnet_rdev->regset,regnr);
							dbg("adr=%d,port=%d: received commit for %i = 0x%X",pnet_rdev->port->adr, pnet_rdev->port->port,regnr,pnet_rdev->regset.reg[regnr]);
						}
						tmp>>=1;
					}
				}


				/* reset changed flags */
				pnet_rdev_register_reset_changed(&pnet_rdev->regset_remote);

				/* set flag */
				pnet_rdev->remote_commit_received=1;

				/* trigger send of commit_ack */
				pnet_rdev->register_commit_ack_state=PNET_RDEV_COMMIT_ACK_STATE_COMMIT_ACK;

				spin_unlock_bh(&pnet_rdev->slock);

				/* trigger sender (not neccessary?) */
				tasklet_schedule(&pnet_rdev->worker_tasklet);

				/* request call of register callback received */
				req_reg_cb=PNET_RDEV_CALLBACK_EV_RECEIVED;

				goto __continue;
			}

			/* commit_ack received */
			if(regnr==PNET_RDEV_SPECIAL_REGNR_COMMIT_ACK) {
#ifdef PNET_RDEV_DEBUG_STATE_COUNTERS
				pnet_rdev->received_commit_acks++;
#endif

				dbg("adr=%d,port=%d: received commit_ack",pnet_rdev->port->adr, pnet_rdev->port->port);
				spin_lock_bh(&pnet_rdev->slock);

				/* check protocol consistency */
				if(pnet_rdev->register_commit_state!=PNET_RDEV_COMMIT_STATE_WAIT_COMMIT_ACK) {
					err("adr=%d,port=%d: commit_ack received, but no commit was sent",pnet_rdev->port->adr, pnet_rdev->port->port);				
				} else {
					/* commit-state = idle (done) */
					pnet_rdev->register_commit_state=PNET_RDEV_COMMIT_STATE_IDLE;
				}

				spin_unlock_bh(&pnet_rdev->slock);

				/* request call of register callback transmitted */
				req_reg_cb=PNET_RDEV_CALLBACK_EV_TRANSMITTED;

				goto __continue;
			}

			/* remote received update */
			if(regnr==PNET_RDEV_SPECIAL_REGNR_FLOWCTRL) {
#ifdef PNET_RDEV_DEBUG_STATE_COUNTERS
				pnet_rdev->received_flowctrl++;
#endif

				spin_lock_bh(&pnet_rdev->slock);
				pnet_rdev->remote_space+=value;
				dbg("adr=%d,port=%d: received remote_received=%i, remote_space=%i",pnet_rdev->port->adr, pnet_rdev->port->port,value,pnet_rdev->remote_space);
				spin_unlock_bh(&pnet_rdev->slock);

				/* trigger sender (not neccessary?) */
				tasklet_schedule(&pnet_rdev->worker_tasklet);

				goto __continue;
			}

		} else {

			/* convert register-nr */
			regnr-=PNET_RDEV_SPECIAL_REGISTERS;


			/* check register number -> ignore */
			if(regnr>PNET_RDEV_REGISTER_MAX)
				goto __continue;

			dbg("adr=%d,port=%d: received: regnr=%i, value=0x%X",pnet_rdev->port->adr, pnet_rdev->port->port,regnr,value);

			/* update remote registerset */
			pnet_rdev->regset_remote.reg[regnr]=value;
			pnet_rdev_register_set_changed(&pnet_rdev->regset_remote,regnr);
		}


__continue:
		/* call register-callback, if requested
		 * called in loop to get more accuracy
		 */
		if(req_reg_cb) {
			/* call register-callback */
			pnet_rdev->register_callback_calls++;
			if(pnet_rdev->register_callback)
				pnet_rdev->register_callback(pnet_rdev,req_reg_cb);
			req_reg_cb=0;
		}
	} /* while */

	/* check for disconnect */
/*
	if(pnet_rdev->disconnected)
		ret=-EPIPE;
*/
__ret:
	/* call data-callback, if requested
	 * called after loop, to get more throughput
	*/
	if(req_dat_cb) {
		/* call data-callback */
		pnet_rdev->data_callback_calls++;
		if(pnet_rdev->data_callback)
			pnet_rdev->data_callback(pnet_rdev,req_dat_cb);
		req_dat_cb=0;
	}

	/* wakeup waiting */
	wake_up(&pnet_rdev->rcv_wait);

	/* return */
	return ret;
}

/*
 * send registers (sender task)
 */
int pnet_rdev_register_send(pnet_rdev_private_t *pnet_rdev)
{
	unsigned int tmp;
	int ret,req_dat_cb;
	unsigned char regnr;
	int regnr_lo, regnr_hi;
	char sendbuf[PNET_RDEV_REGISTER_PACKET__PACKED_SIZE*PNET_RDEV_REGISTER_MAX];
	int pcnt;
	ret=0;

	dbg("adr=%d,port=%d", pnet_rdev->port->adr, pnet_rdev->port->port);

	/* reset callback requests */
	req_dat_cb=0;

	/* if connected & free space */
	while(
/*		(!pnet_rdev->disconnected) && */
		pnet_core_port_write_free(pnet_rdev->port)>=(PNET_RDEV_REGISTER_PACKET__PACKED_SIZE*PNET_RDEV_REGISTER_MAX)
	) {
		/* reset pcnt */
		pcnt=0;

		spin_lock_bh(&pnet_rdev->slock);

		/* commit? -> send registers */
		if(pnet_rdev->register_commit_state==PNET_RDEV_COMMIT_STATE_COMMIT) {
			dbg("adr=%d,port=%d commit", pnet_rdev->port->adr, pnet_rdev->port->port);
			/* build register packets */
			for(regnr_hi=0;regnr_hi<((PNET_RDEV_REGISTER_MAX+31)/32);regnr_hi++) {
				tmp=pnet_rdev->regset_local.changed[regnr_hi];
				for(regnr_lo=0; (regnr_lo<32) && tmp;regnr_lo++) {
					if(tmp&1) {
						/* commit register */
						/* registernr to commit */
						regnr=(regnr_hi*32)+regnr_lo;
						/* build packet (convert regnr) */
						pnet_rdev_register_packet_pack(&sendbuf[pcnt], regnr+PNET_RDEV_SPECIAL_REGISTERS, pnet_rdev->regset_local.reg[regnr]);
						pcnt+=PNET_RDEV_REGISTER_PACKET__PACKED_SIZE;

						/* update register */
						pnet_rdev->regset.reg[regnr]=pnet_rdev->regset_local.reg[regnr];
						pnet_rdev_register_set_changed(&pnet_rdev->regset,regnr);

						dbg("adr=%d,port=%d: send commit for %i = 0x%X",pnet_rdev->port->adr, pnet_rdev->port->port,regnr,pnet_rdev->regset_local.reg[regnr]);
					}
					tmp>>=1;
				}
			}
#ifdef PNET_RDEV_DEBUG_STATE_COUNTERS
			pnet_rdev->sent_commits++;
#endif
			/* send commit */
			pnet_rdev_register_packet_pack(&sendbuf[pcnt], PNET_RDEV_SPECIAL_REGNR_COMMIT, 0);
			pcnt+=PNET_RDEV_REGISTER_PACKET__PACKED_SIZE;

			/* reset changed flags */
			pnet_rdev_register_reset_changed(&pnet_rdev->regset_local);

			/* commit-state = waiting for commit_ack */
			pnet_rdev->register_commit_state=PNET_RDEV_COMMIT_STATE_WAIT_COMMIT_ACK;
		}

		spin_unlock_bh(&pnet_rdev->slock);
		spin_lock_bh(&pnet_rdev->slock);

		/* commit_ack? -> send commit ack */
		if(pnet_rdev->register_commit_ack_state==PNET_RDEV_COMMIT_ACK_STATE_COMMIT_ACK) {
#ifdef PNET_RDEV_DEBUG_STATE_COUNTERS
			pnet_rdev->sent_commit_acks++;
#endif
			dbg("adr=%d,port=%d commit_ack", pnet_rdev->port->adr, pnet_rdev->port->port);

			/* send commit_ack */
			pnet_rdev_register_packet_pack(&sendbuf[pcnt], PNET_RDEV_SPECIAL_REGNR_COMMIT_ACK, 0);
			pcnt+=PNET_RDEV_REGISTER_PACKET__PACKED_SIZE;

			pnet_rdev->register_commit_ack_state=PNET_RDEV_COMMIT_ACK_STATE_IDLE;
		}

		spin_unlock_bh(&pnet_rdev->slock);
		spin_lock_bh(&pnet_rdev->slock);

		/* flowctrl */
		if(pnet_rdev->flowctrl_state!=PNET_RDEV_FLOWCTRL_STATE_IDLE) {
#ifdef PNET_RDEV_DEBUG_STATE_COUNTERS
			pnet_rdev->sent_flowctrl++;
#endif
			if(pnet_rdev->flowctrl_state==PNET_RDEV_FLOWCTRL_STATE_SEND_MAX_SPACE) {
				dbg("adr=%d,port=%d flowctrl_state -> send max space", pnet_rdev->port->adr, pnet_rdev->port->port);
				/* send max free_space */
				pnet_rdev_register_packet_pack(&sendbuf[pcnt],
					PNET_RDEV_SPECIAL_REGNR_FLOWCTRL,
					pnet_rdev_datafifo_free(&pnet_rdev->datafifo_rx)
				);
			/* means: } else if(pnet_rdev->flowctrl_state==PNET_RDEV_FLOWCTRL_STATE_SEND_RECEIVED_BYTES) { */
			} else {
				dbg("adr=%d,port=%d flowctrl_state -> send received bytes", pnet_rdev->port->adr, pnet_rdev->port->port);
				/* read and reset */
				tmp=pnet_rdev->bytes_received;
				pnet_rdev->bytes_received=0;
				/* send received bytes */
				pnet_rdev_register_packet_pack(&sendbuf[pcnt],
					PNET_RDEV_SPECIAL_REGNR_FLOWCTRL,
					tmp
				);
			}
			pcnt+=PNET_RDEV_REGISTER_PACKET__PACKED_SIZE;
			pnet_rdev->flowctrl_state=PNET_RDEV_FLOWCTRL_STATE_IDLE;
		}

		spin_unlock_bh(&pnet_rdev->slock);

		/* data to send? -> send only one packet per cycle */
		tmp=pnet_rdev_datafifo_len(&pnet_rdev->datafifo_tx);
#ifdef FLOW_CONTROL
		
		if(tmp)
			dbg("adr=%d,port=%d: send data: remote_space=%i",pnet_rdev->port->adr, pnet_rdev->port->port,pnet_rdev->remote_space);

		/* max bytes allowed to send */
		tmp=MIN(tmp,pnet_rdev->remote_space);
#endif
		if(tmp) {
			/* max bytes able to send */
			tmp=MIN(tmp,PNET_RDEV_SPECIAL_REGNR_DATAFIFO_COUNT);
			/* calc new remote space */
			pnet_rdev->remote_space-=tmp;

			sendbuf[pcnt]=(tmp-1);
			tmp=pnet_rdev_datafifo_read(&pnet_rdev->datafifo_tx, &sendbuf[pcnt+1], tmp);
			/* set register nr 
			 * special_regnr==0 -> 1 byte load
			 * special_regnr==1 -> 2 byte load
			 * special_regnr==2 -> 3 byte load
			 * special_regnr==3 -> 4 byte load
			 * ...
			*/
			pcnt+=PNET_RDEV_REGISTER_PACKET__PACKED_SIZE;
			dbg("adr=%d,port=%d: send data: %i bytes",pnet_rdev->port->adr, pnet_rdev->port->port,tmp-1);

			/* request call of data-callback transmitted */
			req_dat_cb=PNET_RDEV_CALLBACK_EV_TRANSMITTED;
		}

		/* send */
		if(pcnt) {
			/* send */		
			dbg("adr=%d,port=%d: send: %i bytes",pnet_rdev->port->adr, pnet_rdev->port->port,pcnt);
			if(pnet_core_port_write(pnet_rdev->port,(void*)sendbuf,pcnt)<pcnt) {
				err("adr=%d,port=%d: error writing to port (disconnect)", pnet_rdev->port->adr, pnet_rdev->port->port);
				ret=-EPIPE;
				goto __ret;
			}
		} else {
			dbg("adr=%d,port=%d: nothing to send",pnet_rdev->port->adr, pnet_rdev->port->port);
			/* nothing to send */
			break;
		}
	}

	/* check for disconnect */
/*
	if(pnet_rdev->disconnected)
		ret=-EPIPE;
*/
__ret:
	/* call data-callback, if requested
	 * called after loop, to get more throughput
	*/
	if(req_dat_cb) {
		/* call data-callback */
		pnet_rdev->data_callback_calls++;
		if(pnet_rdev->data_callback)
			pnet_rdev->data_callback(pnet_rdev,req_dat_cb);
		req_dat_cb=0;
	}

	/* wakeup waiting */
	wake_up(&pnet_rdev->snd_wait);

	/* return */
	return ret;
}

/*
 **************************************************************************************************
 * End: Register-stuff
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: PNET Callbacks
 **************************************************************************************************
 */

static void pnet_rdev_port_sent_data(struct pnet_core_port *port)
{
	dbg("adr=%d,port=%d", port->adr, port->port);
	tasklet_schedule(&((pnet_rdev_private_t*)port->private_data)->worker_tasklet);
}

static void pnet_rdev_port_received_data(struct pnet_core_port *port)
{
	dbg("adr=%d,port=%d", port->adr, port->port);
	tasklet_schedule(&((pnet_rdev_private_t*)port->private_data)->worker_tasklet);
}

static void pnet_rdev_port_changed_state(struct pnet_core_port *port, int state)
{
	pnet_rdev_private_t *pnet_rdev = ((pnet_rdev_private_t*)port->private_data);

	dbg("adr=%d,port=%d,state=%d", port->adr, port->port, state);

	switch (state) {
		case PNET_CORE_PORT_CONNECTED:
			spin_lock_bh(&pnet_rdev->slock);
			/* set default registers */
			memset(&pnet_rdev->regset_remote,0,sizeof(struct pnet_rdev_registerset));
			memset(&pnet_rdev->regset_local,0,sizeof(struct pnet_rdev_registerset));
			memset(&pnet_rdev->regset,0,sizeof(struct pnet_rdev_registerset));
			/* reset rx/tx fifos */
			pnet_rdev_datafifo_reset(&pnet_rdev->datafifo_tx);
			pnet_rdev_datafifo_reset(&pnet_rdev->datafifo_rx);
			/* reset commit state */
			pnet_rdev->register_commit_state=PNET_RDEV_COMMIT_STATE_IDLE;
			/* reset commit_ack state */
			pnet_rdev->register_commit_ack_state=PNET_RDEV_COMMIT_ACK_STATE_IDLE;
			/* reset remote commit received */
			pnet_rdev->remote_commit_received=0;
			/* reset remote_space */
			pnet_rdev->remote_space=0;
			/* reset bytes received */
			pnet_rdev->bytes_received=0;
			/* trigger send of flowctrl - max-free-space */
			pnet_rdev->flowctrl_state=PNET_RDEV_FLOWCTRL_STATE_SEND_MAX_SPACE;

#ifdef PNET_RDEV_DEBUG_STATE_COUNTERS
			pnet_rdev->sent_commits=0;
			pnet_rdev->received_commits=0;
			pnet_rdev->sent_commit_acks=0;
			pnet_rdev->received_commit_acks=0;
			pnet_rdev->sent_flowctrl=0;
			pnet_rdev->received_flowctrl=0;
#endif
			spin_unlock_bh(&pnet_rdev->slock);
			/* call callbacks with state disconnected */
			state=PNET_RDEV_CALLBACK_EV_CONNECTED;

			tasklet_schedule(&((pnet_rdev_private_t*)port->private_data)->worker_tasklet);

		break;

		case PNET_CORE_PORT_DISCONNECTED:
			spin_lock_bh(&pnet_rdev->slock);
			/* device needs to get opened again */
			pnet_rdev->disconnected=1;
			spin_unlock_bh(&pnet_rdev->slock);
			/* wakeup all */
			wake_up(&pnet_rdev->rcv_wait);
			wake_up(&pnet_rdev->snd_wait);
			/* call callbacks with state connected */
			state=PNET_RDEV_CALLBACK_EV_DISCONNECTED;
		break;
	}

	/* call register-callback */
	pnet_rdev->register_callback_calls++;
	if(pnet_rdev->register_callback)
		pnet_rdev->register_callback(pnet_rdev,state);

	/* call data-callback */
	pnet_rdev->data_callback_calls++;
	if(pnet_rdev->data_callback)
		pnet_rdev->data_callback(pnet_rdev,state);

	wake_up(&pnet_rdev->open_wait);
}
/*
 **************************************************************************************************
 * End: PNET Callbacks
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Tasklet-Handlers
 **************************************************************************************************
 */

static void pnet_rdev_worker_proc(unsigned long data)
{
	pnet_rdev_private_t *pnet_rdev = ((pnet_rdev_private_t*)data);

	dbg("adr=%d,port=%d",pnet_rdev->port->adr, pnet_rdev->port->port);

	/* receive */
	pnet_rdev_register_receive(pnet_rdev);

	/* send */
	pnet_rdev_register_send(pnet_rdev);
}

/*
 **************************************************************************************************
 * End: Tasklet-Handlers
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Driver-Functions
 **************************************************************************************************
 */

/*
 * get struct by minor and register handlers
 * parameters:
 *	minor .. pnet_rdev-instance to requets
 *	pnet_rdev .. pointer to save requested pnet_rdev instance
 *	priv .. pointer to private data
 *	register_callback .. callback called on register events
 *	data_callback .. callback called on data events
 * return: <0 .. error; 0 .. ok
 */
int pnet_rdev_request(int minor, pnet_rdev_private_t **pnet_rdev_ret, void *priv, void (*register_callback)(pnet_rdev_private_t *, int), void (*data_callback)(pnet_rdev_private_t *, int))
{
	pnet_rdev_private_t *pnet_rdev=&pnet_rdev_private[minor];

	if (minor>=PNET_RDEV_DEV_N)
		return -EINVAL;

	/* set callbacks */
	pnet_rdev->register_callback=register_callback;
	pnet_rdev->data_callback=data_callback;

	/* set private-data */
	pnet_rdev->priv=priv;

	/* do callbacks */
	if(pnet_rdev_private[minor].port->state==PNET_CORE_PORT_CONNECTED) {
		/* call register-callback */
		pnet_rdev->register_callback_calls++;
		if(pnet_rdev->register_callback)
			pnet_rdev->register_callback(pnet_rdev,PNET_RDEV_CALLBACK_EV_CONNECTED);

		/* call data-callback */
		pnet_rdev->data_callback_calls++;
		if(pnet_rdev->data_callback)
			pnet_rdev->data_callback(pnet_rdev,PNET_RDEV_CALLBACK_EV_CONNECTED);

		/* call register-callback */
		pnet_rdev->register_callback_calls++;
		if(pnet_rdev->register_callback)
			pnet_rdev->register_callback(pnet_rdev,PNET_RDEV_CALLBACK_EV_RECEIVED);

		/* call data-callback */
		pnet_rdev->data_callback_calls++;
		if(pnet_rdev->data_callback)
			pnet_rdev->data_callback(pnet_rdev,PNET_RDEV_CALLBACK_EV_RECEIVED);

		/* call register-callback */
		pnet_rdev->register_callback_calls++;
		if(pnet_rdev->register_callback)
			pnet_rdev->register_callback(pnet_rdev,PNET_RDEV_CALLBACK_EV_TRANSMITTED);

		/* call data-callback */
		pnet_rdev->data_callback_calls++;
		if(pnet_rdev->data_callback)
			pnet_rdev->data_callback(pnet_rdev,PNET_RDEV_CALLBACK_EV_TRANSMITTED);
	} else {
		/* call register-callback */
		pnet_rdev->register_callback_calls++;
		if(pnet_rdev->register_callback)
			pnet_rdev->register_callback(pnet_rdev,PNET_RDEV_CALLBACK_EV_DISCONNECTED);

		/* call data-callback */
		pnet_rdev->data_callback_calls++;
		if(pnet_rdev->data_callback)
			pnet_rdev->data_callback(pnet_rdev,PNET_RDEV_CALLBACK_EV_DISCONNECTED);
	}

	/* return structure */
	(*pnet_rdev_ret)=pnet_rdev;

	return 0;
}
EXPORT_SYMBOL(pnet_rdev_request);

/*
 * open pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 * return: <0 .. error; 0 .. ok
 */
int pnet_rdev_open(pnet_rdev_private_t *pnet_rdev)
{
	int ret=0;

	ret=0;
	spin_lock_bh(&pnet_rdev->slock);

	/* check if already open */
	if (pnet_rdev->write_count||pnet_rdev->read_count) {
		ret=-EBUSY;
		goto __ret;
	}

	if (pnet_rdev->port->state!=PNET_CORE_PORT_CONNECTED) {
		ret=-EPIPE;
		goto __ret;
	}

	/* set open */
	pnet_rdev->write_count++;
	pnet_rdev->read_count++;

	/* reset disconnected */
	pnet_rdev->disconnected=0;

__ret:
	spin_unlock_bh(&pnet_rdev->slock);
	return ret;
}
EXPORT_SYMBOL(pnet_rdev_open);

/*
 * close pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 * return: <0 .. error; 0 .. ok
 */
int pnet_rdev_close(pnet_rdev_private_t *pnet_rdev)
{
	/* close */
	spin_lock_bh(&pnet_rdev->slock);
	pnet_rdev->write_count=0;
	pnet_rdev->read_count=0;
	spin_unlock_bh(&pnet_rdev->slock);
	return 0;
}
EXPORT_SYMBOL(pnet_rdev_close);

/*
 * reset temp registers
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	wait .. 0 .. no wait; >0 .. wait max jiffies
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_rdev_register_reset(pnet_rdev_private_t *pnet_rdev, int wait)
{
	int ret=0;

	spin_lock_bh(&pnet_rdev->slock);

	/* check for disconnect */
	if(pnet_rdev->disconnected) {
		ret=-EPIPE;
		goto __ret;
	}

	/* already running commit? -> wait */
	while(pnet_rdev->register_commit_state!=PNET_RDEV_COMMIT_STATE_IDLE) {
		if(wait) {
			spin_unlock_bh(&pnet_rdev->slock);
			if (!wait_event_timeout(pnet_rdev->rcv_wait, 
				pnet_rdev->disconnected || 
				(pnet_rdev->register_commit_state==PNET_RDEV_COMMIT_STATE_IDLE),
				wait
			) ) {
				return -ETIME;
			}
			spin_lock_bh(&pnet_rdev->slock);
		} else {
			ret=-EAGAIN;
			goto __ret;
		}
		/* check for disconnect */
		if(pnet_rdev->disconnected) {
			ret=-EPIPE;
			goto __ret;
		}
	}

	dbg("adr=%d,port=%d",pnet_rdev->port->adr, pnet_rdev->port->port);

	/* reset with current registerset */
	memcpy(&pnet_rdev->regset_local.reg,&pnet_rdev->regset.reg,PNET_RDEV_REGISTER_MAX*sizeof(pnet_rdev_register_t));

__ret:
	spin_unlock_bh(&pnet_rdev->slock);
	return ret;
}
EXPORT_SYMBOL(pnet_rdev_register_reset);

/*
 * read register
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	regnr .. register-number to read
 *	val .. pointer to save value
 * return: <0 .. error; 0 .. not changed, 1 .. changed
 */
int pnet_rdev_register_read(pnet_rdev_private_t *pnet_rdev, unsigned int regnr, pnet_rdev_register_t *val)
{
	int ret=0;

	if(regnr>=PNET_RDEV_REGISTER_MAX)
		return -EINVAL;

	spin_lock_bh(&pnet_rdev->slock);

	/* check for disconnect */
	if(pnet_rdev->disconnected) {
		ret=-EPIPE;
		goto __ret;
	}

	/* read from current registerset */
	(*val)=pnet_rdev->regset.reg[regnr];
	dbg("adr=%d,port=%d: %i=%X",pnet_rdev->port->adr, pnet_rdev->port->port,regnr,*val);

	/* get and reset changed flag */
	ret=pnet_rdev_register_test_clear_changed(&pnet_rdev->regset,regnr);

__ret:
	spin_unlock_bh(&pnet_rdev->slock);
	return ret;
}
EXPORT_SYMBOL(pnet_rdev_register_read);

/*
 * get changed status of register
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	regnr .. register-number
 * return: <0 .. error; 0 .. not changed, 1 .. changed
 */
int pnet_rdev_register_is_changed(pnet_rdev_private_t *pnet_rdev, unsigned int regnr)
{
	int ret;

	if(regnr>=PNET_RDEV_REGISTER_MAX)
		return -EINVAL;

	spin_lock_bh(&pnet_rdev->slock);

	/* check for disconnect */
	if(pnet_rdev->disconnected) {
		ret=-EPIPE;
		goto __ret;
	}

	/* get changed flag */
	ret=pnet_rdev_register_get_changed(&pnet_rdev->regset,regnr);

__ret:
	spin_unlock_bh(&pnet_rdev->slock);
	return ret;
}
EXPORT_SYMBOL(pnet_rdev_register_is_changed);

/*
 * wait until a remote commit
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	wait .. 0 .. no wait; >0 .. wait max jiffies
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 1 .. changed
 */
int pnet_rdev_register_wait_remote_commit(pnet_rdev_private_t *pnet_rdev, int wait)
{
	int ret=0;

	spin_lock_bh(&pnet_rdev->slock);

	/* check for disconnect */
	if(pnet_rdev->disconnected) {
		ret=-EPIPE;
		goto __ret;
	}

	/* wait until changed */
	while( !pnet_rdev->remote_commit_received ) {
		if(wait) {
			spin_unlock_bh(&pnet_rdev->slock);
			if (!wait_event_timeout(pnet_rdev->rcv_wait, 
				pnet_rdev->disconnected || 
				pnet_rdev->remote_commit_received,
				wait
			) ) {
				return -ETIME;
			}
			spin_lock_bh(&pnet_rdev->slock);
		} else {
			ret=-EAGAIN;
			goto __ret;
		}
		/* check for disconnect */
		if(pnet_rdev->disconnected) {
			ret=-EPIPE;
			goto __ret;
		}
	}

	/* reset flag */
	pnet_rdev->remote_commit_received=0;

__ret:
	spin_unlock_bh(&pnet_rdev->slock);
	return ret;
}
EXPORT_SYMBOL(pnet_rdev_register_wait_remote_commit);

/*
 * write register
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	regnr .. regsiter-number to write
 *	val .. value to write to register
 *	wait .. 0 .. no wait; >0 .. wait max jiffies
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_rdev_register_write(pnet_rdev_private_t *pnet_rdev, unsigned int regnr, pnet_rdev_register_t val, int wait)
{
	int ret=0;

	if(regnr>=PNET_RDEV_REGISTER_MAX)
		return -EINVAL;

	spin_lock_bh(&pnet_rdev->slock);

	/* check for disconnect */
	if(pnet_rdev->disconnected) {
		ret=-EPIPE;
		goto __ret;
	}

	/* already running commit? -> wait */
	while(pnet_rdev->register_commit_state!=PNET_RDEV_COMMIT_STATE_IDLE) {
		if(wait) {
			spin_unlock_bh(&pnet_rdev->slock);
			if (!wait_event_timeout(pnet_rdev->rcv_wait, 
				pnet_rdev->disconnected || 
				(pnet_rdev->register_commit_state==PNET_RDEV_COMMIT_STATE_IDLE),
				wait 
			) ) {
				return -ETIME;
			}
			spin_lock_bh(&pnet_rdev->slock);
		} else {
			ret=-EAGAIN;
			goto __ret;
		}
		/* check for disconnect */
		if(pnet_rdev->disconnected) {
			ret=-EPIPE;
			goto __ret;
		}
	}

	/* read from current registerset */
	pnet_rdev->regset_local.reg[regnr]=val;
	pnet_rdev_register_set_changed(&pnet_rdev->regset_local,regnr);
	dbg("adr=%d,port=%d: %i=%X",pnet_rdev->port->adr, pnet_rdev->port->port,regnr,val);

__ret:
	spin_unlock_bh(&pnet_rdev->slock);
	return ret;
}	
EXPORT_SYMBOL(pnet_rdev_register_write);

/*
 * commit registers
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	wait .. 0 .. no wait; >0 .. wait max jiffies
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_rdev_register_commit(pnet_rdev_private_t *pnet_rdev, int wait)
{
	int ret=0;

	spin_lock_bh(&pnet_rdev->slock);

	/* check for disconnect */
	if(pnet_rdev->disconnected) {
		ret=-EPIPE;
		goto __ret;
	}

	/* already running commit? -> wait */
	while(pnet_rdev->register_commit_state!=PNET_RDEV_COMMIT_STATE_IDLE) {
		if(wait) {
			spin_unlock_bh(&pnet_rdev->slock);
			if (!wait_event_timeout(pnet_rdev->rcv_wait, 
				pnet_rdev->disconnected || 
				(pnet_rdev->register_commit_state==PNET_RDEV_COMMIT_STATE_IDLE),
				wait
			) ) {
				return -ETIME;
			}
			spin_lock_bh(&pnet_rdev->slock);
		} else {
			ret=-EAGAIN;
			goto __ret;
		}
		/* check for disconnect */
		if(pnet_rdev->disconnected) {
			ret=-EPIPE;
			goto __ret;
		}
	}

	dbg("adr=%d,port=%d",pnet_rdev->port->adr, pnet_rdev->port->port);

	/* trigger commit */
	pnet_rdev->register_commit_state=PNET_RDEV_COMMIT_STATE_COMMIT;
	spin_unlock_bh(&pnet_rdev->slock);

	tasklet_schedule(&pnet_rdev->worker_tasklet);

	/* wait until commit done */
	spin_lock_bh(&pnet_rdev->slock);
	if(wait) {
		while(pnet_rdev->register_commit_state!=PNET_RDEV_COMMIT_STATE_IDLE) {
			spin_unlock_bh(&pnet_rdev->slock);
			if (!wait_event_timeout(pnet_rdev->rcv_wait, 
				pnet_rdev->disconnected || 
				(pnet_rdev->register_commit_state==PNET_RDEV_COMMIT_STATE_IDLE),
				wait
			) ) {
				return -ETIME;
			}
			spin_lock_bh(&pnet_rdev->slock);
			/* check for disconnect */
			if(pnet_rdev->disconnected) {
				ret=-EPIPE;
				goto __ret;
			}
		}
	} else {
		/* check for disconnect */
		if(pnet_rdev->disconnected) {
				ret=-EPIPE;
				goto __ret;
		}
	}

__ret:
	spin_unlock_bh(&pnet_rdev->slock);
	return ret;
}
EXPORT_SYMBOL(pnet_rdev_register_commit);

/*
 * get rx-fifo-len of pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 * return: <0 .. error; >0 .. read len
 */
int pnet_rdev_read_len (pnet_rdev_private_t *pnet_rdev)
{
	/* return len of fifo */
	return pnet_rdev_datafifo_len(&pnet_rdev->datafifo_rx);
}	
EXPORT_SYMBOL(pnet_rdev_read_len);

/*
 * read data of pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	buf .. buffer to read data in
 *	count .. number of bytes to read
 * return: <0 .. error; >0 .. read len
 */
int pnet_rdev_read (pnet_rdev_private_t *pnet_rdev, char *buf, int count)
{
	/* EOF when disconnected */
	if (pnet_rdev->disconnected) {
		return 0; /* EOF */
	}

	/* wait for data or disconnected */
	if(!pnet_rdev_datafifo_len(&pnet_rdev->datafifo_rx)) {
		return -EAGAIN;
	}

	/* read fifo */
	count=pnet_rdev_datafifo_read(&pnet_rdev->datafifo_rx,buf,count);

	/* update remote-space */
	spin_lock_bh(&pnet_rdev->slock);
	pnet_rdev->bytes_received+=count;
	pnet_rdev->flowctrl_state=PNET_RDEV_FLOWCTRL_STATE_SEND_RECEIVED_BYTES;
	spin_unlock_bh(&pnet_rdev->slock);
	tasklet_schedule(&pnet_rdev->worker_tasklet);

	return count;
}
EXPORT_SYMBOL(pnet_rdev_read);

/*
 * get tx-fifo-space of pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 * return: <0 .. error; >0 .. write size
 */
int pnet_rdev_write_free (pnet_rdev_private_t *pnet_rdev)
{
	/* return free space */
	return pnet_rdev_datafifo_free(&pnet_rdev->datafifo_tx);
}
EXPORT_SYMBOL(pnet_rdev_write_free);

/*
 * write data to pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	buf .. buffer to write
 *	count .. number of bytes to write
 * return: <0 .. error; >0 .. write size
 */
int pnet_rdev_write (pnet_rdev_private_t *pnet_rdev, const char *buf, unsigned int count)
{
	/* error EPIPE when disconnected */
	if (pnet_rdev->disconnected) {
		return -EPIPE;
	}

	/* wait for free space or disconnected */
	if(!pnet_rdev_datafifo_free(&pnet_rdev->datafifo_tx))
		return -EAGAIN;

	/* write fifo */
	count=pnet_rdev_datafifo_write(&pnet_rdev->datafifo_tx,(char*)buf,count);

	/* trigger writer */
	tasklet_schedule(&pnet_rdev->worker_tasklet);

	return count;
}
EXPORT_SYMBOL(pnet_rdev_write);

/*
 **************************************************************************************************
 * End: Driver-Functions
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Linux-Driver-Functions
 **************************************************************************************************
 */
static int pnet_rdev_lin_open(struct inode *inode, struct file *file)
{
	int minor;
	int ret=0;

	minor=iminor(inode);
	if (minor>=PNET_RDEV_DEV_N)
		return -EINVAL;

	dbg("adr=%d,port=%d", pnet_rdev_private[minor].port->adr, pnet_rdev_private[minor].port->port);

	if (down_interruptible(&pnet_rdev_private[minor].lock))
		return -ERESTARTSYS;

	if ((file->f_flags&O_ACCMODE)==O_RDWR) {
		if (pnet_rdev_private[minor].write_count>0 || pnet_rdev_private[minor].read_count>0) {
			up(&pnet_rdev_private[minor].lock);
			return -EBUSY;
		}
		pnet_rdev_private[minor].write_count++;
		pnet_rdev_private[minor].read_count++;
	}
	else if ((file->f_flags&O_ACCMODE)==O_WRONLY) {
		if (pnet_rdev_private[minor].write_count>0) {
			up(&pnet_rdev_private[minor].lock);
			return -EBUSY;
		}
		pnet_rdev_private[minor].write_count++;
	}
	else if ((file->f_flags&O_ACCMODE)==O_RDONLY) {
		if (pnet_rdev_private[minor].read_count>0) {
			up(&pnet_rdev_private[minor].lock);
			return -EBUSY;
		}
		pnet_rdev_private[minor].read_count++;
	}

	/* reset disconnected before wait */
	pnet_rdev_private[minor].disconnected=0;
	/* wait for connected */
	if (pnet_rdev_port[minor].state!=PNET_CORE_PORT_CONNECTED) {
		if (file->f_flags & O_NONBLOCK) {
			ret=-EPIPE;
			goto open_error;
		}

		if (wait_event_interruptible(pnet_rdev_private[minor].open_wait,(pnet_rdev_port[minor].state==PNET_CORE_PORT_CONNECTED))) {
			ret=-ERESTARTSYS;
			goto open_error;
		}
	}

	up(&pnet_rdev_private[minor].lock);
	return 0;

open_error:
	if ((file->f_flags&O_ACCMODE)==O_RDWR || (file->f_flags&O_ACCMODE)==O_WRONLY) 
		if (pnet_rdev_private[minor].write_count>0) pnet_rdev_private[minor].write_count--;
	if ((file->f_flags&O_ACCMODE)==O_RDWR || (file->f_flags&O_ACCMODE)==O_RDONLY) 
		if (pnet_rdev_private[minor].read_count>0) pnet_rdev_private[minor].read_count--;
	up(&pnet_rdev_private[minor].lock);
	return ret;
}

static int pnet_rdev_lin_close(struct inode *inode, struct file *file)
{
	int minor;

	minor=iminor(inode);
	if (minor>=PNET_RDEV_DEV_N)
		return -EINVAL;

	dbg("adr=%d,port=%d", pnet_rdev_private[minor].port->adr, pnet_rdev_private[minor].port->port);

	down(&pnet_rdev_private[minor].lock);
	if ((file->f_flags&O_ACCMODE)==O_RDWR || (file->f_flags&O_ACCMODE)==O_WRONLY) 
		if (pnet_rdev_private[minor].write_count>0) pnet_rdev_private[minor].write_count--;
	if ((file->f_flags&O_ACCMODE)==O_RDWR || (file->f_flags&O_ACCMODE)==O_RDONLY) 
		if (pnet_rdev_private[minor].read_count>0) pnet_rdev_private[minor].read_count--;
	up(&pnet_rdev_private[minor].lock);
	return 0;
}

static ssize_t pnet_rdev_lin_read (struct file *file, char __user *buf, size_t count,
                         loff_t *offset)
{
	int minor;

	minor = iminor(file->f_dentry->d_inode);
	if (minor>=PNET_RDEV_DEV_N)
		return -EINVAL;

	dbg("adr=%d,port=%d", pnet_rdev_private[minor].port->adr, pnet_rdev_private[minor].port->port);

	if (down_interruptible(&pnet_rdev_private[minor].lock))
		return -ERESTARTSYS;

	/* EOF when disconnected */
	if (pnet_rdev_private[minor].disconnected) {
		up(&pnet_rdev_private[minor].lock);
		return 0; /* EOF */
	}
	/* wait for data or disconneted */
	while(!(pnet_rdev_datafifo_len(&pnet_rdev_private[minor].datafifo_rx) || pnet_rdev_private[minor].disconnected)) {
		up(&pnet_rdev_private[minor].lock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(pnet_rdev_private[minor].rcv_wait,
		   (pnet_rdev_datafifo_len(&pnet_rdev_private[minor].datafifo_rx) || pnet_rdev_private[minor].disconnected)))
			return -ERESTARTSYS;
		if (down_interruptible(&pnet_rdev_private[minor].lock))
			return -ERESTARTSYS;
	}
	/* data is now available or port got disconneted 
	 * check disconneted again
	 * EOF when disconnected
	 */
	if (pnet_rdev_private[minor].disconnected) {
		up(&pnet_rdev_private[minor].lock);
		return 0; /* EOF */
	}

	/* read fifo */
	count=pnet_rdev_datafifo_read__user(&pnet_rdev_private[minor].datafifo_rx,buf,count);

	/* update remote-space */
	spin_lock_bh(&pnet_rdev_private[minor].slock);
	pnet_rdev_private[minor].bytes_received+=count;
	pnet_rdev_private[minor].flowctrl_state=PNET_RDEV_FLOWCTRL_STATE_SEND_RECEIVED_BYTES;
	spin_unlock_bh(&pnet_rdev_private[minor].slock);

	up(&pnet_rdev_private[minor].lock);

	tasklet_schedule(&pnet_rdev_private[minor].worker_tasklet);

	return count;
}


static ssize_t pnet_rdev_lin_write (struct file *file, const char __user *buf, size_t count,
                          loff_t *offset)
{
	int minor;

	minor = iminor(file->f_dentry->d_inode);
	if (minor>=PNET_RDEV_DEV_N)
		return -EINVAL;

	dbg("adr=%d,port=%d", pnet_rdev_private[minor].port->adr, pnet_rdev_private[minor].port->port);

	if (down_interruptible(&pnet_rdev_private[minor].lock))
		return -ERESTARTSYS;

	/* error EPIPE when disconnected */
	if (pnet_rdev_private[minor].disconnected) {
		up(&pnet_rdev_private[minor].lock);
		return -EPIPE;
	}

	/* wait for free space or disconnected */
	while(!(pnet_rdev_datafifo_free(&pnet_rdev_private[minor].datafifo_tx) || pnet_rdev_private[minor].disconnected)) {
		up(&pnet_rdev_private[minor].lock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(pnet_rdev_private[minor].snd_wait,
		   (pnet_rdev_datafifo_free(&pnet_rdev_private[minor].datafifo_tx) || pnet_rdev_private[minor].disconnected)))
			return -ERESTARTSYS;
		if (down_interruptible(&pnet_rdev_private[minor].lock))
			return -ERESTARTSYS;
	}

	/* free space is now available or port got disconnected
	 * check disconneted again
	 * error EPIPE when disconnected
	 */
	if (pnet_rdev_private[minor].disconnected) {
		up(&pnet_rdev_private[minor].lock);
		return -EPIPE;
	}

	/* write fifo */
	count=pnet_rdev_datafifo_write__user(&pnet_rdev_private[minor].datafifo_tx,buf,count);

	up(&pnet_rdev_private[minor].lock);

	/* trigger writer */
	tasklet_schedule(&pnet_rdev_private[minor].worker_tasklet);

	return count;
}

static unsigned int pnet_rdev_lin_poll (struct file *file, poll_table *wait)
{
	int minor;
	unsigned int mask=0;

	minor = iminor(file->f_dentry->d_inode);
	if (minor>=PNET_RDEV_DEV_N)
		return -EINVAL;

	dbg("adr=%d,port=%d", pnet_rdev_private[minor].port->adr, pnet_rdev_private[minor].port->port);

	if (down_interruptible(&pnet_rdev_private[minor].lock))
		return -ERESTARTSYS;
	poll_wait(file,&pnet_rdev_private[minor].rcv_wait,wait);
	poll_wait(file,&pnet_rdev_private[minor].snd_wait,wait);
 	
	if (pnet_rdev_datafifo_free(&pnet_rdev_private[minor].datafifo_tx)||pnet_rdev_private[minor].disconnected)
		mask |= POLLOUT | POLLWRNORM;
	if (pnet_rdev_datafifo_len(&pnet_rdev_private[minor].datafifo_rx)||pnet_rdev_private[minor].disconnected)
		mask |= POLLIN | POLLRDNORM;
	up(&pnet_rdev_private[minor].lock);
	return mask;
}

static int pnet_rdev_lin_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	pnet_rdev_private_t *pnet_rdev;
	int minor,wait;
	int ret=0;
	struct pnet_rdev_register_ioctl_data ioctl_data;

	/* get pnet_rdev struct */
	minor = iminor(file->f_dentry->d_inode);
	if (minor>=PNET_RDEV_DEV_N)
		return -EINVAL;

	pnet_rdev =&pnet_rdev_private[minor];
	dbg("adr=%d,port=%d", pnet_rdev->port->adr, pnet_rdev->port->port);

	/* get wait-status */
	wait=(file->f_flags & O_NONBLOCK) ? 0 : PNET_RDEV_LIN_DEFAULT_TIMEOUT;

	/* down */
	if (down_interruptible(&pnet_rdev->lock))
		return -ERESTARTSYS;

	/* error EPIPE when disconnected */
	if (pnet_rdev->disconnected) {
		ret=-EPIPE;
		goto __ret_up;
	}

	/* parse command */
	switch(cmd)
	{
		case PNET_RDEV_REGISTER_IOCTL_RESET:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY) {
				ret=-EINVAL;
				goto __ret_up;
			}

			dbg("adr=%d,port=%d PNET_RDEV_REGISTER_IOCTL_RESET", pnet_rdev->port->adr, pnet_rdev->port->port);
			ret=pnet_rdev_register_reset(pnet_rdev,wait);
		break;

		case PNET_RDEV_REGISTER_IOCTL_READ:
			/* check read-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_RDONLY) {
				ret=-EINVAL;
				goto __ret_up;
			}
				
			/* get structure */
			ret=copy_from_user (&ioctl_data, argp, sizeof(struct pnet_rdev_register_ioctl_data));
			if(ret)
				goto __ret_up;

			/* read register */
			ret=pnet_rdev_register_read(pnet_rdev,ioctl_data.regnr,&ioctl_data.value);			
			if(ret<0)
				goto __ret_up;
			ioctl_data.changed=ret;

			dbg("adr=%d,port=%d PNET_RDEV_REGISTER_IOCTL_READ %i=%X changed=%i", pnet_rdev->port->adr, pnet_rdev->port->port,ioctl_data.regnr,ioctl_data.value,ioctl_data.changed);

			/* put structure */
			ret=copy_to_user (argp, &ioctl_data, sizeof(struct pnet_rdev_register_ioctl_data));
		break;

		case PNET_RDEV_REGISTER_IOCTL_IS_CHANGED:
			/* check read-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_RDONLY) {
				ret=-EINVAL;
				goto __ret_up;
			}
				
			/* get structure */
			ret=copy_from_user (&ioctl_data, argp, sizeof(struct pnet_rdev_register_ioctl_data));
			if(ret)
				goto __ret_up;

			/* read register */
			ret=pnet_rdev_register_is_changed(pnet_rdev,ioctl_data.regnr);			
			if(ret<0)
				goto __ret_up;
			ioctl_data.changed=ret;

			dbg("adr=%d,port=%d PNET_RDEV_REGISTER_IOCTL_IS_CHANGED %i changed=%i", pnet_rdev->port->adr, pnet_rdev->port->port, ioctl_data.regnr, ioctl_data.changed);

			/* put structure */
			ret=copy_to_user (argp, &ioctl_data, sizeof(struct pnet_rdev_register_ioctl_data));
		break;

		case PNET_RDEV_REGISTER_IOCTL_WAIT_REMOTE_COMMIT:
			/* check read-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_RDONLY) {
				ret=-EINVAL;
				goto __ret_up;
			}

			dbg("adr=%d,port=%d PNET_RDEV_REGISTER_IOCTL_WAIT_REMOTE_COMMIT", pnet_rdev->port->adr, pnet_rdev->port->port);	

			ret=pnet_rdev_register_wait_remote_commit(pnet_rdev,wait);
		break;

		case PNET_RDEV_REGISTER_IOCTL_WRITE:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY) {
				ret=-EINVAL;
				goto __ret_up;
			}

			/* get structure */
			ret=copy_from_user (&ioctl_data, argp, sizeof(struct pnet_rdev_register_ioctl_data));
			if(ret)
				goto __ret_up;

			dbg("adr=%d,port=%d PNET_RDEV_REGISTER_IOCTL_WRITE %i=%X", pnet_rdev->port->adr, pnet_rdev->port->port,ioctl_data.regnr,ioctl_data.value);

			/* write register */
			ret=pnet_rdev_register_write(pnet_rdev,ioctl_data.regnr,ioctl_data.value,wait);			
		break;

		/* set remote task periode */
		case PNET_RDEV_REGISTER_IOCTL_COMMIT:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY) {
				ret=-EINVAL;
				goto __ret_up;
			}

			/* commit */
			dbg("adr=%d,port=%d PNET_RDEV_REGISTER_IOCTL_COMMIT", pnet_rdev->port->adr, pnet_rdev->port->port);
			ret=pnet_rdev_register_commit(pnet_rdev,wait);
		break;

		default:
			ret=-ENOTTY;
	}

__ret_up:
	up(&pnet_rdev_private[minor].lock);
	return ret;
}

static struct file_operations pnet_rdev_fops = {
	.owner = THIS_MODULE,
	.ioctl = pnet_rdev_lin_ioctl,
	.open = pnet_rdev_lin_open,
	.release = pnet_rdev_lin_close,
	.read = pnet_rdev_lin_read,
	.write = pnet_rdev_lin_write,
	.poll = pnet_rdev_lin_poll,
};

static struct class *pnet_rdev_class;

/*
 **************************************************************************************************
 * End: Linux-Driver-Functions
 **************************************************************************************************
 */




#ifdef CONFIG_PROC_FS

/***********************************************************************
 *  Begin: procfs statistics
 ***********************************************************************/

static int pnet_rdev_info_show(struct seq_file *m, void *v)
{
	int i;

	seq_printf(m,"%s\n",DRV_NAME);

	for (i=0;i<PNET_RDEV_DEV_N;i++) {
		seq_printf(m,"%s%i:\n",DRV_NAME,i);
		down(&pnet_rdev_private[i].lock);

		seq_printf(m," - Port:\n");
		seq_printf(m,"   - DevAdr:                     %i\n",pnet_rdev_port[i].adr);
		seq_printf(m,"   - PortNr:                     %i\n",pnet_rdev_port[i].port);
		seq_printf(m,"   - Pri:                        %i\n",pnet_rdev_port[i].pri);

		spin_lock_bh(&pnet_rdev_private[i].slock);

		seq_printf(m," - Open:\n");
		seq_printf(m,"   - read:                       %i\n",pnet_rdev_private[i].read_count);
		seq_printf(m,"   - write:                      %i\n",pnet_rdev_private[i].write_count);
		seq_printf(m," - States:\n");
		seq_printf(m,"   - disconnected:               %i\n",pnet_rdev_private[i].disconnected);
		seq_printf(m,"   - register_commit_state:      %i\n",pnet_rdev_private[i].register_commit_state);
		seq_printf(m,"   - register_commit_ack_state:  %i\n",pnet_rdev_private[i].register_commit_ack_state);
		seq_printf(m,"   - flowctrl_state:             %i\n",pnet_rdev_private[i].flowctrl_state);
		seq_printf(m,"   - remote_commit_received:     %i\n",pnet_rdev_private[i].remote_commit_received);
		seq_printf(m,"   - remote_space:               %i\n",pnet_rdev_private[i].remote_space);
		seq_printf(m,"   - bytes_received:             %i\n",pnet_rdev_private[i].bytes_received);
		seq_printf(m,"   - datafifo_tx size:           %i/%i\n",pnet_rdev_private[i].datafifo_tx.size,PNET_RDEV_DATAFIFO_SIZE);
		seq_printf(m,"   - datafifo_rx size:           %i/%i\n",pnet_rdev_private[i].datafifo_rx.size,PNET_RDEV_DATAFIFO_SIZE);
		seq_printf(m," - Callback-Counters:\n");		
		seq_printf(m,"   - register_callback_calls:    %i\n",pnet_rdev_private[i].register_callback_calls);
		seq_printf(m,"   - data_callback_calls:        %i\n",pnet_rdev_private[i].data_callback_calls);

#ifdef PNET_RDEV_DEBUG_STATE_COUNTERS
		seq_printf(m," - Counters:\n");		
		seq_printf(m,"   - sent commits:               %i\n",pnet_rdev_private[i].sent_commits);
		seq_printf(m,"   - received commit acks:       %i\n",pnet_rdev_private[i].received_commit_acks);
		seq_printf(m,"   - received commits:           %i\n",pnet_rdev_private[i].received_commits);
		seq_printf(m,"   - sent commit acks:           %i\n",pnet_rdev_private[i].sent_commit_acks);
		seq_printf(m,"   - sent_flowctrl:              %i\n",pnet_rdev_private[i].sent_flowctrl);
		seq_printf(m,"   - received_flowctrl:          %i\n",pnet_rdev_private[i].received_flowctrl);
#endif

		spin_unlock_bh(&pnet_rdev_private[i].slock);

		up(&pnet_rdev_private[i].lock);
	}

	return 0;
}

static int pnet_rdev_info_open( struct inode *inode, struct file *file)
{
	return single_open(file, pnet_rdev_info_show, NULL);
}

static struct file_operations pnet_rdev_info_fops = {
	.owner = THIS_MODULE,
	.open = pnet_rdev_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static struct proc_dir_entry *pnet_rdev_proc_dir,*pnet_rdev_proc_info;
static int pnet_rdev_proc_create(void)
{
	pnet_rdev_proc_dir=proc_mkdir("driver/"DRV_NAME,NULL);
	pnet_rdev_proc_info=create_proc_entry("info",S_IRUGO,pnet_rdev_proc_dir);
	if (pnet_rdev_proc_info) {
		pnet_rdev_proc_info->proc_fops = &pnet_rdev_info_fops;
	}
	return 0;
}
static void pnet_rdev_proc_remove(void)
{
	if (pnet_rdev_proc_info) remove_proc_entry("info", pnet_rdev_proc_dir);
	if (pnet_rdev_proc_dir) remove_proc_entry("driver/"DRV_NAME,NULL);
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

static int __init pnet_rdev_init(void)
{
	int i;
	int ret=0;

	dbg("");
	
	memset(&pnet_rdev_private, 0, sizeof(pnet_rdev_private_t)*PNET_RDEV_DEV_N);
	logs_init();

	/* init */
	for (i=0;i<PNET_RDEV_DEV_N;i++) {
		/* link structures */
		pnet_rdev_private[i].port=&pnet_rdev_port[i];
		pnet_rdev_port[i].private_data=&pnet_rdev_private[i];

		/* reset callbacks */
		pnet_rdev_private[i].register_callback=0;
		pnet_rdev_private[i].register_callback_calls=0;
		pnet_rdev_private[i].data_callback=0;
		pnet_rdev_private[i].data_callback_calls=0;

		/* init */
		init_MUTEX(&pnet_rdev_private[i].lock);
		init_waitqueue_head(&pnet_rdev_private[i].open_wait); 
		init_waitqueue_head(&pnet_rdev_private[i].snd_wait);
		init_waitqueue_head(&pnet_rdev_private[i].rcv_wait); 
		spin_lock_init(&pnet_rdev_private[i].slock);
		tasklet_init(&pnet_rdev_private[i].worker_tasklet,pnet_rdev_worker_proc,(unsigned long)&pnet_rdev_private[i]);
		spin_lock_init(&pnet_rdev_private[i].datafifo_rx.slock);
		spin_lock_init(&pnet_rdev_private[i].datafifo_tx.slock);

		/* register port */
		ret=pnet_core_port_register(&pnet_rdev_port[i]);
		if (ret) {
			err("register port %d failed",pnet_rdev_port[i].port); 
			/* unregister registered ports */
			i--;
			while(i--)
				pnet_core_port_unregister(&pnet_rdev_port[i]);
			goto pnet_rdev_init_err_ports;
		}
	}

	/* register chrdev */
	ret=register_chrdev(DRV_MAJOR,DRV_NAME,&pnet_rdev_fops);
	if (ret) {
		err("register chrdev failed");
		goto pnet_rdev_init_err_cdev;
	}

	/* register device class */
	pnet_rdev_class=class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(pnet_rdev_class)) {
		err("class_create failed");
		ret = PTR_ERR(pnet_rdev_class);
		goto pnet_rdev_init_err_class;
	}

	/* register devices */
	for (i=0;i<PNET_RDEV_DEV_N;i++) {
		pnet_rdev_private[i].device=device_create(pnet_rdev_class,NULL,MKDEV(DRV_MAJOR,i),NULL,"%s%d",DRV_NAME,i);
		if (IS_ERR(pnet_rdev_private[i].device)) {
			err("device_create failed");
			ret=PTR_ERR(pnet_rdev_private[i].device);
			/* unregister devices */
			i--;
			while(i--)
				device_destroy(pnet_rdev_class,MKDEV(DRV_MAJOR,i));
			goto pnet_rdev_init_err_class_device;
		}	
	}		

#ifdef CONFIG_PROC_FS
	pnet_rdev_proc_create();
#endif

	return ret;

pnet_rdev_init_err_class_device:
	class_destroy(pnet_rdev_class);
pnet_rdev_init_err_class:
	unregister_chrdev(DRV_MAJOR,DRV_NAME);
pnet_rdev_init_err_cdev:
	for (i=0;i<PNET_RDEV_DEV_N;i++) {
		pnet_core_port_unregister(&pnet_rdev_port[i]);
		tasklet_kill(&pnet_rdev_private[i].worker_tasklet);
	}
pnet_rdev_init_err_ports:
	return ret;
}

static void __exit pnet_rdev_exit(void)
{
	int i;

	dbg("");

#ifdef CONFIG_PROC_FS
	pnet_rdev_proc_remove();
#endif
	for (i=0;i<PNET_RDEV_DEV_N;i++)
		device_destroy(pnet_rdev_class,MKDEV(DRV_MAJOR,i));
	class_destroy(pnet_rdev_class);
	unregister_chrdev(DRV_MAJOR,DRV_NAME);
	for (i=0;i<PNET_RDEV_DEV_N;i++) {
		pnet_core_port_unregister(&pnet_rdev_port[i]);
		tasklet_kill(&pnet_rdev_private[i].worker_tasklet);
	}
}

module_init( pnet_rdev_init );
module_exit( pnet_rdev_exit );

MODULE_AUTHOR("Manfred Schlaegl");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PNET gpio driver");

/*
 **************************************************************************************************
 * End: Module-Stuff
 **************************************************************************************************
 */
