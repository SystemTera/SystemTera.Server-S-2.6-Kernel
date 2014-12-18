/***********************************************************************
 *
 * @Authors: Andreas Schrattenecker, Ginzinger electronic systems GmbH
 * @Descr: PNET core
 * @TODO:
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
 *	20.07.09 - manfred schlaegl: lock-error in pnet_core_tasklet repaired
 ***********************************************************************/
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "pnet_core.h"

#define DRV_NAME		"pnet_core"

//#define DEBUG


#ifndef MIN
#define MIN(a,b)	((a)>(b) ? (b) : (a))
#endif
 
#ifndef MAX
#define MAX(a,b)	((a)>(b) ? (a) : (b))
#endif

#define LOGS
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
	while(*s && logs_i<LOGS_MEM_SIZE-1)
		logs_mem[logs_i++]=*s++;
}
#else
#define logs(str)		do {} while (0)
#define logs_init()		do {} while (0)
#define logs_print()		do {} while (0)
#endif


#ifdef DEBUG
#define dbg(format, arg...) printk(format "\n" , ## arg)
//#define dbg(format, arg...) printk(KERN_DEBUG "%s: " format "\n" , __FILE__ , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif

#define err(format, arg...) printk(KERN_ERR "%s: " format "\n" , __FILE__ , ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: " format "\n" , __FILE__ , ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: " format "\n" , __FILE__ , ## arg)


static int pnet_core_fifo_init(struct pnet_core_fifo *fifo, unsigned int size);
static void pnet_core_fifo_dealloc(struct pnet_core_fifo *fifo);
static void pnet_core_fifo_clear(struct pnet_core_fifo *fifo, unsigned int flags);
static unsigned int pnet_core_fifo_write(struct pnet_core_fifo *fifo,
			 unsigned char *buffer, unsigned int len, unsigned int flags);
static void pnet_core_fifo_write_clear(struct pnet_core_fifo *fifo, unsigned int flags);
static unsigned int pnet_core_fifo_read(struct pnet_core_fifo *fifo,
			 unsigned char *buffer, unsigned int len, unsigned int flags);
static void pnet_core_fifo_read_clear(struct pnet_core_fifo *fifo, unsigned int flags);
static unsigned int pnet_core_fifo_remove(struct pnet_core_fifo *fifo, unsigned int len, unsigned int flags);
static unsigned int pnet_core_fifo_len(struct pnet_core_fifo *fifo, unsigned int flags);
static unsigned int pnet_core_fifo_free(struct pnet_core_fifo *fifo, unsigned int flags);
static unsigned char *pnet_core_fifo_tail_ptr(struct pnet_core_fifo *fifo, unsigned int flags);
static unsigned char *pnet_core_fifo_prehead_ptr(struct pnet_core_fifo *fifo, unsigned int pre, unsigned int flags);

/* pnet_core mutex for all register and unregister funktions
   (possible optimization to use device mutex in pnet_core_port_register and unregister)
*/
DECLARE_MUTEX(pnet_core_mutex);

/* device list */
static LIST_HEAD(pnet_core_dev_list);

/* pnet_core_tasklet - tasklet for receive and transmit packets
 * @data: pointer to device
 */
static void pnet_core_tasklet(unsigned long data)
{
	unsigned long iflags;
	int pri_nr,port_nr;
	struct pnet_core_dev *dev = (struct pnet_core_dev*)data;
	struct pnet_core_pri *pri;
	struct pnet_core_port *port=NULL, *port_received=NULL, *port_received_last=NULL;
	struct pnet_core_packet packet;
	struct list_head *p,*n;
	int ret,receive,transmit,sent_data;
	unsigned int port_state_old;

	unsigned int order_head;
	struct pnet_core_order *order=NULL;

	BUG_ON(dev==NULL);
	logs("tsk;");
	dbg("pnet_core_tasklet called: adr=%d", dev->adr);

	/* state change
	 * TODO: handle very fast state changes
	 */
	spin_lock_irqsave(&dev->lock, iflags);
	if (dev->state_new!=dev->state) {
		dev->state=dev->state_new;
		spin_unlock_irqrestore(&dev->lock, iflags);
		dbg("pnet_core_tasklet state change");
		/* inform all registered ports a state change */
		for (port_nr=0;port_nr<dev->port_n;port_nr++) {
			port=dev->port_table[port_nr];
			if (port) {
				port_state_old=port->state;
				switch (dev->state) {
					case PNET_CORE_DEVICE_DISCONNECTED:
						port->state=PNET_CORE_PORT_DISCONNECTED;
						/* clear port fifos requested via options */
						if (port && port->options&PNET_CORE_PORT_RXCLEAR_ON_DISCONNECT) {
							/* use write_clear to avoid locking in pnet_core_port_read */
							pnet_core_fifo_write_clear(&port->rx_fifo,PNET_CORE_FIFO_LOCK);
						}
						if (port && port->options&PNET_CORE_PORT_TXCLEAR_ON_DISCONNECT) {
							spin_lock_irqsave(&dev->lock, iflags);
							/* use read_clear to avoid locking in pnet_core_port_write */
							pnet_core_fifo_read_clear(&port->tx_fifo,0);
							pnet_core_fifo_read_clear(&port->tx_order,0);
							spin_unlock_irqrestore(&dev->lock, iflags);
						}
						break;
					case PNET_CORE_DEVICE_CONNECTED:
						port->state=PNET_CORE_PORT_CONNECTED;
						break;
				}
				if (port->state!=port_state_old && port->changed_state) 
					port->changed_state(port,port->state);
			}		
		}
		spin_lock_irqsave(&dev->lock, iflags);
	}
	spin_unlock_irqrestore(&dev->lock, iflags);

	/* if disconnected dont transfer packets */
	if (dev->state==PNET_CORE_DEVICE_DISCONNECTED)
		return;

	/* receive */
	dbg("pnet_core_tasklet receive");
	receive=1;
	do {
		ret=dev->receive_packet(dev,&packet);
		switch(ret) {
			case 0:
				dbg("pnet_core_tasklet packet received");
				/* free space check already done in driver via pnet_core_packet_receive_ready */

				port_received_last=port_received;
				port_received=dev->port_table[packet.port];

				BUG_ON(port_received==NULL);
				pnet_core_fifo_write(&port_received->rx_fifo,packet.data,packet.len,0);
				port_received->rx_pending=0;
				/* rx stats */
				port_received->stat_rx_packets++;
				port_received->stat_rx_bytes+=packet.len;

				break;
			case -EBUSY:
				/* rx_pending already got set in pnet_core_packet_receive_ready */
			case -ENODATA:
			case -ENOPROTOOPT:
			default:
				receive=0;
				break;
		}
		/* call callback on port change in packets */
		if (port_received_last!=NULL && port_received_last!=port_received) {
			/* execute callback */
			if (port_received_last->received_data != NULL) {
				port_received_last->received_data(port_received_last);
				port_received_last->stat_rx_callbacks++;
			}
			/* dont call last again on end of receive */
			port_received_last=NULL;
		}
		/* call callback on end of receive */
		if (port_received!=NULL && !receive) {
			/* execute callback */
			if (port_received->received_data != NULL) {
				port_received->received_data(port_received);
				port_received->stat_rx_callbacks++;
			}
		}
	}while(receive);

	/* transmit */
	dbg("pnet_core_tasklet transmit");
	pri_nr=0;
	while (pri_nr<dev->pri_n) {
		dbg("pri:%d",pri_nr);
		pri=&dev->pri[pri_nr];
		BUG_ON(&dev->pri[pri_nr]==NULL);
		order_head=pri->order_head;
		if (order_head==pri->order_tail) {
			pri_nr++;
			continue;
		}
		/* search port with order_nr==order_tail */
		dbg("search port");
		order=NULL;
		list_for_each_safe(p,n,&pri->port_list) {
			port=list_entry(p, struct pnet_core_port, list);
			BUG_ON(port==NULL);
			dbg("search: port:%d",port->port);
			if (pnet_core_fifo_len(&port->tx_order,PNET_CORE_FIFO_LOCK)) {
				order=(struct pnet_core_order*) pnet_core_fifo_tail_ptr(&port->tx_order,0);
				BUG_ON(order==NULL);
				dbg("search: port:%d, order_nr=%d, order_tail=%d",port->port,order->order_nr,pri->order_tail);
				if (order->order_nr==pri->order_tail) {
					dbg("found: port:%d",port->port);
					break;
				}
			}
		}
		if (order==NULL) {
			/* order entry not found -> port got unregisterd with unsent data
			 * proceed with next order_nr
			 */
			pri->order_tail++;
			continue;
		}
		
		dbg("transmit port:%d order_nr:%d",port->port,order->order_nr);
		/* split into packets and try to send */
		BUG_ON(port==NULL);
		BUG_ON(order==NULL);
		transmit=1;
		packet.port=port->port;
		packet.pri=pri_nr;
		sent_data=0;
		do
		{
			packet.len=min(order->size,dev->data_n);
			pnet_core_fifo_read(&port->tx_fifo,packet.data,packet.len,PNET_CORE_FIFO_NOREMOVE);
			if (dev->send_packet(dev,&packet)==0) {
				/* sucessfull sent */
				sent_data=1;
				/* remove data from tx_fifo */
				pnet_core_fifo_remove(&port->tx_fifo,packet.len,0);
				/* lock to prevent append data to order in core_port_write */				
				spin_lock_irqsave(&port->tx_order.lock, iflags);
				order->size-=packet.len;
				if (order->size==0) {
					/* order->size completely sent, remove order from order fifo */
					pnet_core_fifo_remove(&port->tx_order,sizeof(struct pnet_core_order),0);
					pri->order_tail++;
					transmit=0;
				}
				spin_unlock_irqrestore(&port->tx_order.lock, iflags);
				/* stats */
				port->stat_tx_packets++;
				port->stat_tx_bytes+=packet.len;
			} else {
				/* unable to send->proceed with next pri */
				pri_nr++;
				transmit=0;
			}
		}while(transmit);		
		/* execute callback */
		if (sent_data && port->sent_data != NULL) {
			port->sent_data(port);
			port->stat_tx_callbacks++;
		}	

	}
}

/*
 * pnet_core_device_find - find registered pnet_core device with address==adr
 * @adr: address of device
 * returns pointer to device if found, NULL if not found
 */
struct pnet_core_dev *pnet_core_device_find(unsigned int adr)
{
	struct pnet_core_dev	*dev;
	struct list_head *p,*n;
	list_for_each_safe(p,n,&pnet_core_dev_list) {
		dev=list_entry(p, struct pnet_core_dev, list);
		if (dev->adr==adr) return dev;
	}
	return NULL;
}

/* pnet_core_device_register - register pnet_core device
 * @dev: pointer to device to register
 *
 * not allowed in interrupt context
 * 
 * returns 0 on success
 */
int pnet_core_device_register(struct pnet_core_dev *dev)
{
	unsigned int pri;
	int ret;

	dbg("pnet_core_device_register called");

	down(&pnet_core_mutex);
	if (pnet_core_device_find(dev->adr)!=NULL)
	{
		warn("device adr=%d is already registered",dev->adr);
		ret=-EBUSY;
		goto crd_err_mutex;
	}

	/* init pri struct */
	dev->pri=kmalloc(sizeof(struct pnet_core_pri)*dev->pri_n, GFP_KERNEL);
	if (dev->pri == NULL) {
		ret=-ENOMEM;
		goto crd_err_mutex;
	}

	for (pri=0;pri<dev->pri_n;pri++) {
		dev->pri[pri].pri=pri;
		dev->pri[pri].dev=dev;
		INIT_LIST_HEAD(&dev->pri[pri].port_list);
		dev->pri[pri].order_head=0;
		dev->pri[pri].order_tail=0;
	}
		
	/* init port table */
	dev->port_table=kmalloc(sizeof(struct pnet_core_port*)*dev->port_n, GFP_KERNEL);
	if (dev->port_table == NULL)  {
		ret=-ENOMEM;
		goto crd_err;
	}
	memset(dev->port_table, 0, sizeof(struct pnet_core_port*)*dev->port_n);

	spin_lock_init(&dev->lock);
	tasklet_init(&dev->tl_descr,pnet_core_tasklet,(unsigned long)dev);
	/* add device to device list */
	INIT_LIST_HEAD(&dev->list);
	list_add_tail(&dev->list,&pnet_core_dev_list);

	up(&pnet_core_mutex);
	return 0;

crd_err:
	kfree(dev->pri);
crd_err_mutex:
	up(&pnet_core_mutex);
	return ret;
}
EXPORT_SYMBOL(pnet_core_device_register);

/* pnet_core_device_unregister - unregister pnet_core device
 * @dev: pointer to device to unregister
 *
 * not allowed in interrupt context
 * 
 * returns 0 on success
 */
int pnet_core_device_unregister(struct pnet_core_dev *dev)
{
	unsigned int port;
	dbg("pnet_core_device_unregister called");

	down(&pnet_core_mutex);
	if (pnet_core_device_find(dev->adr)==NULL)
	{
		warn("device adr=%d is not registered",dev->adr);
		up(&pnet_core_mutex);
		return -ENXIO;
	}
	tasklet_kill(&dev->tl_descr);
	/* free pri struct */
	kfree(dev->pri);
	/* free port table */
	for (port=0;port<dev->port_n;port++) {
		if (dev->port_table[port]) {
			err("port %d is registered",port);
			up(&pnet_core_mutex);
			pnet_core_port_unregister(dev->port_table[port]);
			down(&pnet_core_mutex);
		}
	}
	kfree(dev->port_table);
	
	/* remove device from device list */
	list_del(&dev->list);
	
	up(&pnet_core_mutex);
	return 0;
}
EXPORT_SYMBOL(pnet_core_device_unregister);


/* pnet_core_device_transfer - initiate transfer for device
 * @dev: pointer to device
 * 
 * this function is called in device driver when new packets were received
 * and/or transmitted
 * 
 * returns 0 on success
 */
int pnet_core_device_transfer(struct pnet_core_dev *dev)
{
	tasklet_hi_schedule(&dev->tl_descr);
	return 0;
}
EXPORT_SYMBOL(pnet_core_device_transfer);



/* pnet_core_device_state - pnet_core device state changed
 * @dev: pointer to device
 * 
 * this function is called in device driver when the (connection) state of the 
 * device changed
 * 
 * returns 0 on success
 */
int pnet_core_device_state(struct pnet_core_dev *dev, unsigned int state)
{
	dev->state_new=state;
	tasklet_hi_schedule(&dev->tl_descr);
	return 0;
}
EXPORT_SYMBOL(pnet_core_device_state);

/* pnet_core_packet_receive_ready - check if pnet_core is ready to receive packet
 * @dev: pointer to device
 * @packet: pointer to packet
 * 
 * this function is called in device driver to check that pnet_core is
 * able to receive the packet received by device driver. 
 * 
 * returns 0 if pnet_core will take packet
 * returns -EBUSY if port is not ready
 * returns -ENOPROTOOPT if port is not registered on device
 */
int pnet_core_packet_receive_ready(struct pnet_core_dev *dev, struct pnet_core_packet *packet)
{
	dbg("pnet_core_packet_receive_ready called");

	if (dev->port_table[packet->port]) {
		if (pnet_core_port_read_free(dev->port_table[packet->port])<packet->len) {
			if (dev->port_table[packet->port])
				dev->port_table[packet->port]->rx_pending=1;
			return -EBUSY;
		}
	} 
	else {
		#if (PNET_CORE_IGNORE_INVALID_PORT_PACKETS==1)
			return -ENOPROTOOPT;
		#else
			return -EBUSY;
		#endif
		
	}
	return 0;
}
EXPORT_SYMBOL(pnet_core_packet_receive_ready);

/* pnet_core_port_register - register port
 * @port: pointer to port to register
 *
 * not allowed in interrupt context
 * 
 * returns 0 on success
 */
int pnet_core_port_register(struct pnet_core_port *port)
{
	unsigned long iflags;
	struct pnet_core_dev *dev;
	int ret;
	dbg("pnet_core_port_register called");
	
	down(&pnet_core_mutex);
	dev=pnet_core_device_find(port->adr);
	if (dev==NULL) 
	{
		warn("device adr=%d is not registered",port->adr);
		ret=-ENXIO;
		goto cpr_err_mutex;
	}
	if (port->pri>=dev->pri_n || port->port>=dev->port_n) {
		ret=-EINVAL;
		goto cpr_err_mutex;
	}
	if (dev->port_table[port->port]) {
		warn("port %d on device adr=%d is already registered",port->port,port->adr);
		ret=-EBUSY;
		goto cpr_err_mutex;
	}
	/* increment ref counter for device to prevent module unloading */
	if (!try_module_get(dev->owner)) {
		err("get module for device adr=%d failed",port->adr);
		ret=-EBUSY;
		goto cpr_err_mutex;
	}
	/* rx_fifo init */
	port->rx_size=roundup_pow_of_two(port->rx_size);
	if (pnet_core_fifo_init(&port->rx_fifo,port->rx_size)) {
		ret=-ENOMEM;
		goto cpr_err_rxfifo;
	}

	/* tx_fifo init */
	port->tx_size=roundup_pow_of_two(port->tx_size);
	if (pnet_core_fifo_init(&port->tx_fifo,port->tx_size)) {
		ret=-ENOMEM;
		goto cpr_err_txfifo;
	}

	/* tx_order init */
	port->tx_order_size=MAX(roundup_pow_of_two(port->tx_order_size),PNET_CORE_TX_ORDER_SIZE_MIN);
	if (pnet_core_fifo_init(&port->tx_order, port->tx_order_size)) {
		ret=-ENOMEM;
		goto cpr_err_txorder;
	}

	port->rx_pending=0;
	port->dev=dev;
	INIT_LIST_HEAD(&port->list);

	spin_lock_irqsave(&dev->lock, iflags);
	dev->port_table[port->port]=port;
	list_add_tail(&port->list,&dev->pri[port->pri].port_list);

	/* init port state and call callback */
	switch (dev->state) {
		case PNET_CORE_DEVICE_DISCONNECTED:
			port->state=PNET_CORE_PORT_DISCONNECTED;
			break;
		case PNET_CORE_DEVICE_CONNECTED:
			port->state=PNET_CORE_PORT_CONNECTED;
			break;
	}
	if (port->changed_state) 
		port->changed_state(port,port->state);

	spin_unlock_irqrestore(&dev->lock, iflags);



	up(&pnet_core_mutex);
	return 0;

cpr_err_txorder:
	pnet_core_fifo_dealloc(&port->tx_fifo);
cpr_err_txfifo:
	pnet_core_fifo_dealloc(&port->rx_fifo);
cpr_err_rxfifo:
	module_put(dev->owner);
cpr_err_mutex:
	up(&pnet_core_mutex);
	return ret;
}
EXPORT_SYMBOL(pnet_core_port_register);

/* pnet_core_port_unregister - unregister port
 * @port: pointer to port to unregister
 *
 * not allowed in interrupt context
 * 
 * returns 0 on success
 */
int pnet_core_port_unregister(struct pnet_core_port *port)
{
	unsigned long iflags;
	struct pnet_core_dev *dev;
	int ret;
	dbg("pnet_core_port_unregister called");

	down(&pnet_core_mutex);
	dev=pnet_core_device_find(port->adr);
	if (dev==NULL) 
	{
		warn("device adr=%d is not registered",port->adr);
		ret=-ENXIO;
		goto cpu_err_mutex;
	}
	if (port->port>=dev->port_n) {
		ret=-EINVAL;
		goto cpu_err_mutex;
	}	
	if (dev->port_table[port->port]==NULL)
	{
		warn("port %d on device adr=%d is not registered",port->port,port->adr);
		ret=-ENOPROTOOPT;
		goto cpu_err_mutex;
	}

	spin_lock_irqsave(&dev->lock, iflags);
	dev->port_table[port->port]=NULL;		
	list_del(&port->list);
	spin_unlock_irqrestore(&dev->lock, iflags);

	pnet_core_fifo_dealloc(&port->rx_fifo);
	pnet_core_fifo_dealloc(&port->tx_fifo);
	pnet_core_fifo_dealloc(&port->tx_order);

	module_put(dev->owner);

	up(&pnet_core_mutex);
	return 0;

cpu_err_mutex:
	up(&pnet_core_mutex);
	return ret;

}
EXPORT_SYMBOL(pnet_core_port_unregister);

/* pnet_core_port_read - read data from port
 * @port: pointer to port to read
 * @buffer: pointer to buffer for data
 * @count: max bytes to read
 *
 * only one reader is allowed  
 * 
 * returns how many bytes were read
 */
unsigned int pnet_core_port_read(struct pnet_core_port *port, unsigned char *buffer, unsigned int count)
{
	int rx;

	dbg("pnet_core_port_read called: port=%d",port->port);

	/* no locking needed, only one reader */
	rx=pnet_core_fifo_read(&port->rx_fifo,buffer,count,0);

	/* to avoid locking, handle possible disconnect while reading from port */
	if (port->state==PNET_CORE_PORT_DISCONNECTED) {
		if (port->options&PNET_CORE_PORT_RXCLEAR_ON_DISCONNECT) {
			dbg("pnet_core_port_read: disconnect while read");
			pnet_core_fifo_read_clear(&port->rx_fifo,PNET_CORE_FIFO_LOCK);
			return 0;
		}
	}
	/* no need to lock rx_pendig. TODO: check again */
	if (rx && port->rx_pending)
		pnet_core_device_transfer(port->dev);
	return rx;
}
EXPORT_SYMBOL(pnet_core_port_read);

/* pnet_core_port_write - write data to port
 * @port: pointer to port to write
 * @buffer: pointer to buffer for data
 * @count: max bytes to write
 *
 * only one writer is allowed  
 * 
 * returns how many bytes were written
 */
unsigned int pnet_core_port_write(struct pnet_core_port *port, unsigned char *buffer, unsigned int count)
{
	unsigned long iflags;
	struct pnet_core_order	order_new;
	struct pnet_core_order	*order;
	int tx;

	dbg("pnet_core_port_write called: port=%d",port->port);

	/* check there is free space in order fifo */
	if (pnet_core_fifo_free(&port->tx_order,PNET_CORE_FIFO_LOCK)<sizeof(struct pnet_core_order))
		return 0;

	/* no locking needed, only one writer */
	tx=pnet_core_fifo_write(&port->tx_fifo,buffer,count,0);
	if (tx) {
		/* add entry in order fifo
		 * needs locking, otherwise it would be difficult to detect in tasklet
		 * when order_nr could not be found whether port was unloaded with not sent data 
		 * or write to order fifo was not completed.
		 */
		spin_lock_irqsave(&port->tx_order.lock, iflags);
		/* optimization dont create new order if possible */
		if (pnet_core_fifo_len(&port->tx_order,0) >=sizeof(struct pnet_core_order)) {
			/* order entry exists */
			order=(struct pnet_core_order*) pnet_core_fifo_prehead_ptr(&port->tx_order,sizeof(struct pnet_core_order),0);
			if (order->order_nr==port->dev->pri[port->pri].order_head-1) {
				/* no new order exists for other ports (order_nr is latest order_nr)
				 * append new data to order
				 */
				order->size+=tx;
				goto write_done;
			}	
		}			
		/* create new order */
		order_new.order_nr=port->dev->pri[port->pri].order_head++;
		order_new.size=tx;
		pnet_core_fifo_write(&port->tx_order,(unsigned char*)&order_new,sizeof(struct pnet_core_order),0);

write_done:
		/* to avoid locking, handle possible disconnect while writing to port */
		if (port->state==PNET_CORE_PORT_DISCONNECTED) {
			dbg("pnet_core_port_write: disconnect while write");
			if (port->options&PNET_CORE_PORT_TXCLEAR_ON_DISCONNECT) {
				pnet_core_fifo_write_clear(&port->tx_fifo,0);
				pnet_core_fifo_write_clear(&port->tx_order,0);
				spin_unlock_irqrestore(&port->tx_order.lock, iflags);
				return 0;
			}
		}
		spin_unlock_irqrestore(&port->tx_order.lock, iflags);
		pnet_core_device_transfer(port->dev);

	}
	return tx;
}
EXPORT_SYMBOL(pnet_core_port_write);

/* pnet_core_port_read_len - receive buffer len of port
 * @port: pointer to port
 * 
 * returns how many bytes are stored in receive buffer
 */
unsigned int pnet_core_port_read_len(struct pnet_core_port *port)
{
	return pnet_core_fifo_len(&port->rx_fifo,PNET_CORE_FIFO_LOCK);
}
EXPORT_SYMBOL(pnet_core_port_read_len);

/* pnet_core_port_write_len - transmit buffer len of port
 * @port: pointer to port
 * 
 * returns how many bytes are stored in transmit buffer
 */
unsigned int pnet_core_port_write_len(struct pnet_core_port *port)
{
	return pnet_core_fifo_len(&port->tx_fifo,PNET_CORE_FIFO_LOCK);
}
EXPORT_SYMBOL(pnet_core_port_write_len);

/* pnet_core_port_read_free - receive buffer free of port
 * @port: pointer to port
 * 
 * returns how many bytes are free in receive buffer
 */
unsigned int pnet_core_port_read_free(struct pnet_core_port *port)
{
	return pnet_core_fifo_free(&port->rx_fifo,PNET_CORE_FIFO_LOCK);
}
EXPORT_SYMBOL(pnet_core_port_read_free);

/* pnet_core_port_write_free - transmit buffer free of port
 * @port: pointer to port
 * 
 * returns how many bytes are free in transmit buffer
 */
unsigned int pnet_core_port_write_free(struct pnet_core_port *port)
{
	/* check free space in order fifo */
	if (pnet_core_fifo_free(&port->tx_order,PNET_CORE_FIFO_LOCK)<sizeof(struct pnet_core_order))
		return 0;
	return pnet_core_fifo_free(&port->tx_fifo,PNET_CORE_FIFO_LOCK);
}
EXPORT_SYMBOL(pnet_core_port_write_free);


#ifdef CONFIG_PROC_FS

static void pnet_core_dev_show(struct seq_file *m,struct pnet_core_dev *dev)
{

	seq_printf(m,"owner: %s, ",module_name(dev->owner));
	seq_printf(m,"adr: %d, ",dev->adr);
	seq_printf(m,"pri_n: %d, ",dev->pri_n);
	seq_printf(m,"port_n: %d, ",dev->port_n);
	seq_printf(m,"data_n: %d\n",dev->data_n);
}

static void pnet_core_pri_show(struct seq_file *m,struct pnet_core_pri *pri)
{
	struct list_head *p,*n;
	struct pnet_core_port *port;
	
	seq_printf(m,"pri: %d, ",pri->pri);
	seq_printf(m,"order_head: %d, ",pri->order_head);
	seq_printf(m,"order_tail: %d, ",pri->order_tail);
	seq_printf(m,"ports: ");
	list_for_each_safe(p,n,&pri->port_list) {
		port=list_entry(p, struct pnet_core_port, list);
		seq_printf(m,"%d ",port->port);
	}
	seq_printf(m,"\n");
}

static void pnet_core_fifo_show(struct seq_file *m,struct pnet_core_fifo *fifo)
{
	#ifdef DEBUG
	int i;
	#endif

	seq_printf(m,"len: %d, ",pnet_core_fifo_len(fifo,PNET_CORE_FIFO_LOCK));
	seq_printf(m,"free: %d\n",pnet_core_fifo_free(fifo,PNET_CORE_FIFO_LOCK));

	#ifdef DEBUG
	seq_printf(m," head: %d, tail: %d",fifo->head,fifo->tail);
	for (i=0;i<fifo->size;i++) {
		if (i%32==0) seq_printf(m,"\n ");
		seq_printf(m,"%2.2x ",fifo->buffer[i]);
	}
	seq_printf(m,"\n");
	#endif
}

static void pnet_core_port_show(struct seq_file *m,struct pnet_core_port *port)
{
	
	seq_printf(m,"port: %d, ",port->port);
	seq_printf(m,"owner: %s, ",module_name(port->owner));
	seq_printf(m,"adr: %d, ",port->adr);
	seq_printf(m,"pri: %d, ",port->pri);
	seq_printf(m,"rx_size: %d, ",port->rx_size);
	seq_printf(m,"tx_size: %d, ",port->tx_size);
	seq_printf(m,"tx_order_size: %d, ",port->tx_order_size);
	seq_printf(m,"rx_pending: %d",port->rx_pending);
	seq_printf(m,"\n");
	seq_printf(m," rx_fifo: ");
	pnet_core_fifo_show(m,&port->rx_fifo);
	seq_printf(m," tx_fifo: ");
	pnet_core_fifo_show(m,&port->tx_fifo);
	seq_printf(m," tx_order: ");
	pnet_core_fifo_show(m,&port->tx_order);
	seq_printf(m," stat_rx_bytes: %d, ",port->stat_rx_bytes);
	seq_printf(m," stat_rx_packets: %d, ",port->stat_rx_packets);
	seq_printf(m," stat_rx_callbacks: %d\n",port->stat_rx_callbacks);
	seq_printf(m," stat_tx_bytes: %d, ",port->stat_tx_bytes);
	seq_printf(m," stat_tx_packets: %d, ",port->stat_tx_packets);
	seq_printf(m," stat_tx_callbacks: %d\n",port->stat_tx_callbacks);

}

static int pnet_core_info_show(struct seq_file *m, void *v)
{
	unsigned int pri,port;
	struct pnet_core_dev *dev;
	struct list_head *p,*n;

	down(&pnet_core_mutex);
	list_for_each_safe(p,n,&pnet_core_dev_list) {
		dev=list_entry(p, struct pnet_core_dev, list);

		/* device */
		seq_printf(m,"DEVICE:\n");
		pnet_core_dev_show(m,dev);

		/* priorities */
		seq_printf(m,"PRIORITIES:\n");
		for(pri=0;pri<dev->pri_n;pri++) {
			pnet_core_pri_show(m,&dev->pri[pri]);
		}
		/* ports */
		seq_printf(m,"PORTS:\n");
		for (port=0;port<dev->port_n;port++) {
			if (dev->port_table[port]) {
				pnet_core_port_show(m,dev->port_table[port]);
			}
		}
	}
	up(&pnet_core_mutex);
	return 0;
}
static int pnet_core_info_open( struct inode *inode, struct file *file)
{
	return single_open(file, pnet_core_info_show, NULL);
}

static struct file_operations pnet_core_info_fops = {
	.owner = THIS_MODULE,
	.open = pnet_core_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static struct proc_dir_entry *pnet_core_proc_dir,*pnet_core_proc_info;
static int pnet_core_proc_create(void)
{
	pnet_core_proc_dir=proc_mkdir("driver/"DRV_NAME,NULL);
	pnet_core_proc_info=create_proc_entry("info",S_IRUGO,pnet_core_proc_dir);
	if (pnet_core_proc_info) {
		pnet_core_proc_info->proc_fops = &pnet_core_info_fops;
	}
	return 0;
}
static void pnet_core_proc_remove(void)
{
	if (pnet_core_proc_info) remove_proc_entry("info", pnet_core_proc_dir);
	if (pnet_core_proc_dir) remove_proc_entry("driver/"DRV_NAME,NULL);
}
#endif	


static int __init pnet_core_init(void)
{
	dbg("pnet_core_init called");
	logs_init();

	#ifdef CONFIG_PROC_FS
		pnet_core_proc_create();
	#endif

	return 0;
}

static void __exit pnet_core_exit(void)
{
	dbg("pnet_core_exit called");

	#ifdef CONFIG_PROC_FS
		pnet_core_proc_remove();
	#endif
}

//pnet_core_fifo derived from linux/kernel/kfifo.c linux/include/kfifo.h 
static int pnet_core_fifo_init(struct pnet_core_fifo *fifo, unsigned int size)
{
	/*
	 * round up to the next power of 2, since our 'let the indices
	 * wrap' tachnique works only in this case.
	 */
	if (size & (size - 1)) {
		BUG_ON(size > 0x80000000);
		size = roundup_pow_of_two(size);
	}

	fifo->buffer = kmalloc(size, GFP_KERNEL);
	if (fifo->buffer == NULL) return -ENOMEM;

	fifo->size = size;
	pnet_core_fifo_clear(fifo,0);
	spin_lock_init(&fifo->lock);

	return 0;
}

static void pnet_core_fifo_dealloc(struct pnet_core_fifo *fifo)
{
	kfree(fifo->buffer);
}

static void pnet_core_fifo_clear(struct pnet_core_fifo *fifo, unsigned int flags)
{
	unsigned long iflags;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_lock_irqsave(&fifo->lock, iflags);

	fifo->head = fifo->tail = 0;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_unlock_irqrestore(&fifo->lock, iflags);
}

static unsigned int pnet_core_fifo_write(struct pnet_core_fifo *fifo,
			 unsigned char *buffer, unsigned int len, unsigned int flags)
{
	unsigned int l;
	unsigned int len_m;
	unsigned long iflags;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_lock_irqsave(&fifo->lock, iflags);

	len_m = min(len, fifo->size - fifo->head + fifo->tail);
	
	if (flags&PNET_CORE_FIFO_COMPLETE && len_m!=len) {
		if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
			spin_unlock_irqrestore(&fifo->lock, iflags);
		return 0;
	}
	/* first put the data starting from fifo->head to buffer end */
	l = min(len_m, fifo->size - (fifo->head & (fifo->size - 1)));
	memcpy(fifo->buffer + (fifo->head & (fifo->size - 1)), buffer, l);

	/* then put the rest (if any) at the beginning of the buffer */
	memcpy(fifo->buffer, buffer + l, len_m - l);
	fifo->head += len_m;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_unlock_irqrestore(&fifo->lock, iflags);
	return len_m;
}

static void pnet_core_fifo_write_clear(struct pnet_core_fifo *fifo, unsigned int flags)
{
	unsigned long iflags;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_lock_irqsave(&fifo->lock, iflags);

	fifo->head = fifo->tail;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_unlock_irqrestore(&fifo->lock, iflags);
}


static unsigned int pnet_core_fifo_read(struct pnet_core_fifo *fifo,
			 unsigned char *buffer, unsigned int len, unsigned int flags)
{
	unsigned int l;
	unsigned long iflags;
	unsigned int len_m;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_lock_irqsave(&fifo->lock, iflags);

	len_m = min(len, fifo->head - fifo->tail);
	if (flags&PNET_CORE_FIFO_COMPLETE && len_m!=len) {
		if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
			spin_unlock_irqrestore(&fifo->lock, iflags);
		return 0;
	}
	/* first get the data from fifo->tail until the end of the buffer */
	l = min(len_m, fifo->size - (fifo->tail & (fifo->size - 1)));
	memcpy(buffer, fifo->buffer + (fifo->tail & (fifo->size - 1)), l);
	/* then get the rest (if any) from the beginning of the buffer */
	memcpy(buffer + l, fifo->buffer, len_m - l);
	if (!(flags&PNET_CORE_FIFO_NOREMOVE))
		fifo->tail += len_m;
	/*
	 * optimization: if the FIFO is empty, set the indices to 0
	 * so we don't wrap the next time

	 * the optimization is not safe without locking
         * (without locking, fifo->tail must not be altered)

	if (fifo->head == fifo->tail)
		fifo->head = fifo->tail = 0;
	 */
	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
			spin_unlock_irqrestore(&fifo->lock, iflags);

	return len_m;
}

static void pnet_core_fifo_read_clear(struct pnet_core_fifo *fifo, unsigned int flags)
{
	unsigned long iflags;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_lock_irqsave(&fifo->lock, iflags);

	fifo->tail = fifo->head = 0;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_unlock_irqrestore(&fifo->lock, iflags);
}

static unsigned int pnet_core_fifo_remove(struct pnet_core_fifo *fifo, unsigned int len, unsigned int flags)
{
	unsigned long iflags;
	unsigned int len_m;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_lock_irqsave(&fifo->lock, iflags);

	len_m = min(len, fifo->head - fifo->tail);
	if (flags&PNET_CORE_FIFO_COMPLETE && len_m!=len) {
		if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
			spin_unlock_irqrestore(&fifo->lock, iflags);
		return 0;
	}
	if (!(flags&PNET_CORE_FIFO_NOREMOVE))
		fifo->tail += len_m;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
			spin_unlock_irqrestore(&fifo->lock, iflags);

	return len_m;
}

static unsigned int pnet_core_fifo_len(struct pnet_core_fifo *fifo, unsigned int flags)
{
	unsigned long iflags;
	unsigned int ret;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_lock_irqsave(&fifo->lock, iflags);

	ret = fifo->head - fifo->tail;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_unlock_irqrestore(&fifo->lock, iflags);

	return ret;
}

static unsigned int pnet_core_fifo_free(struct pnet_core_fifo *fifo, unsigned int flags)
{
	unsigned long iflags;
	unsigned int ret;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_lock_irqsave(&fifo->lock, iflags);

	ret = fifo->size - fifo->head + fifo->tail;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_unlock_irqrestore(&fifo->lock, iflags);

	return ret;
}
static unsigned char *pnet_core_fifo_tail_ptr(struct pnet_core_fifo *fifo, unsigned int flags)
{
	unsigned long iflags;
	unsigned char *ret;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_lock_irqsave(&fifo->lock, iflags);

	ret = fifo->buffer + (fifo->tail & (fifo->size - 1));

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_unlock_irqrestore(&fifo->lock, iflags);

	return ret;

}
static unsigned char *pnet_core_fifo_prehead_ptr(struct pnet_core_fifo *fifo, unsigned int pre, unsigned int flags)
{
	unsigned long iflags;
	unsigned char *ret;

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_lock_irqsave(&fifo->lock, iflags);

	ret = fifo->buffer + ((fifo->head-pre) & (fifo->size - 1));

	if (flags&PNET_CORE_FIFO_LOCK || PNET_CORE_FIFO_FORCELOCK) 
		spin_unlock_irqrestore(&fifo->lock, iflags);

	return ret;

}
module_init( pnet_core_init );
module_exit( pnet_core_exit );

MODULE_AUTHOR("Andreas Schrattenecker");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PNET core");
