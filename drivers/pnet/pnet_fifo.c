/***********************************************************************
 *
 * @Authors: Andreas Schrattenecker, Ginzinger electronic systems GmbH
 * @Descr: fifo driver for pnet
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
 *	2009-17 manfred.schlaegl@gmx.at
 *		* class-device for automated device-node generation
 ***********************************************************************/
/***********************************************************************
 *  @TODO:
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


#define DRV_MAJOR  			241
#define DRV_NAME			"pnet_fifo"

//#define DEBUG


#define PNET_FIFO_DEV_N				5
#define PNET_FIFO_PORT_START		0	


#define PNET_FIFO_READ_BUFFER_LEN	1024
#define PNET_FIFO_WRITE_BUFFER_LEN	1024

typedef struct
{
	int read_count;
	int write_count;
	struct semaphore lock;	
	wait_queue_head_t read_wait;
	wait_queue_head_t write_wait;
	wait_queue_head_t open_wait;
	unsigned char read_buffer[PNET_FIFO_READ_BUFFER_LEN];
	unsigned char write_buffer[PNET_FIFO_WRITE_BUFFER_LEN];
	int disconnected;	/* if set device got disconnected after open */
	struct device *device;	/* linux device */
}pnet_fifo_private_t;

static pnet_fifo_private_t pnet_fifo_private[PNET_FIFO_DEV_N];

static void pnet_fifo_port_sent_data(struct pnet_core_port *port);
static void pnet_fifo_port_received_data(struct pnet_core_port *port);
static void pnet_fifo_port_changed_state(struct pnet_core_port *port, int state);

static struct pnet_core_port pnet_fifo_port[PNET_FIFO_DEV_N] = 
{{
	.owner = THIS_MODULE,
	.adr=0,
	.pri=0,
	.port=PNET_FIFO_PORT_START,
	.rx_size=1024,
	.tx_size=1024,
	.tx_order_size=256,
	.sent_data=pnet_fifo_port_sent_data,
	.received_data=pnet_fifo_port_received_data,
	.changed_state=pnet_fifo_port_changed_state,
	.private_data=&pnet_fifo_private[0],
	.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT,
},
{
	.owner = THIS_MODULE,
	.adr=0,
	.pri=1,
	.port=PNET_FIFO_PORT_START+1,
	.rx_size=1024,
	.tx_size=1024,
	.tx_order_size=256,
	.sent_data=pnet_fifo_port_sent_data,
	.received_data=pnet_fifo_port_received_data,
	.changed_state=pnet_fifo_port_changed_state,
	.private_data=&pnet_fifo_private[1],
	.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT,
},
{
	.owner = THIS_MODULE,
	.adr=0,
	.pri=2,
	.port=PNET_FIFO_PORT_START+2,
	.rx_size=1024,
	.tx_size=1024,
	.tx_order_size=256,
	.sent_data=pnet_fifo_port_sent_data,
	.received_data=pnet_fifo_port_received_data,
	.changed_state=pnet_fifo_port_changed_state,
	.private_data=&pnet_fifo_private[2],
	.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT,
},
{
	.owner = THIS_MODULE,
	.adr=0,
	.pri=3,
	.port=PNET_FIFO_PORT_START+3,
	.rx_size=1024,
	.tx_size=1024,
	.tx_order_size=256,
	.sent_data=pnet_fifo_port_sent_data,
	.received_data=pnet_fifo_port_received_data,
	.changed_state=pnet_fifo_port_changed_state,
	.private_data=&pnet_fifo_private[3],
	.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT,
},
/* pnet-update-port */
{
	.owner = THIS_MODULE,
	.adr=0,
	.pri=3,
	.port=PNET_FIFO_PORT_START+4,
	.rx_size=4096,
	.tx_size=4096,
	.tx_order_size=256,
	.sent_data=pnet_fifo_port_sent_data,
	.received_data=pnet_fifo_port_received_data,
	.changed_state=pnet_fifo_port_changed_state,
	.private_data=&pnet_fifo_private[4],
	.options=PNET_CORE_PORT_CLEAR_ON_DISCONNECT,
}};

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
#define dbg(format, arg...) printk(format "\n" , ## arg)
//#define dbg(format, arg...) printk(KERN_DEBUG "%s: " format "\n" , __FILE__ , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif

#define err(format, arg...) printk(KERN_ERR "%s: " format "\n" , __FILE__ , ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: " format "\n" , __FILE__ , ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: " format "\n" , __FILE__ , ## arg)


static void pnet_fifo_port_sent_data(struct pnet_core_port *port)
{
	dbg("pnet_fifo_port_sent_data called: adr=%d,port=%d", port->adr, port->port);
	wake_up_interruptible(&((pnet_fifo_private_t*)port->private_data)->write_wait);
}
static void pnet_fifo_port_received_data(struct pnet_core_port *port)
{
	dbg("pnet_fifo_port_received_data called: adr=%d,port=%d", port->adr, port->port);
	wake_up_interruptible(&((pnet_fifo_private_t*)port->private_data)->read_wait);
}
static void pnet_fifo_port_changed_state(struct pnet_core_port *port, int state)
{
	dbg("pnet_fifo_port_changed_state called: adr=%d,port=%d,state=%d", port->adr, port->port, state);
	switch (state) {
		case PNET_CORE_PORT_CONNECTED:
			wake_up_interruptible(&((pnet_fifo_private_t*)port->private_data)->open_wait);
			break;
		case PNET_CORE_PORT_DISCONNECTED:
			((pnet_fifo_private_t*)port->private_data)->disconnected=1; /* device needs to get opened again */
			wake_up_interruptible(&((pnet_fifo_private_t*)port->private_data)->read_wait);
			wake_up_interruptible(&((pnet_fifo_private_t*)port->private_data)->write_wait);
			break;
	}
}

static ssize_t pnet_fifo_read (struct file *file, char __user *buf, size_t count,
                         loff_t *offset)
{
	int minor;

	dbg("pnet_fifo_read called");

	minor = iminor(file->f_dentry->d_inode);
	if (minor>=PNET_FIFO_DEV_N)
		return -EINVAL;

	if (down_interruptible(&pnet_fifo_private[minor].lock))
		return -ERESTARTSYS;

	/* EOF when disconnected */
	if (pnet_fifo_private[minor].disconnected) {
		up(&pnet_fifo_private[minor].lock);
		return 0; /* EOF */
	}
	/* wait for data or disconneted */
	while(!(pnet_core_port_read_len(&pnet_fifo_port[minor]) || pnet_fifo_private[minor].disconnected)) {
		up(&pnet_fifo_private[minor].lock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(pnet_fifo_private[minor].read_wait,
		   (pnet_core_port_read_len(&pnet_fifo_port[minor]) || pnet_fifo_private[minor].disconnected)))
			return -ERESTARTSYS;
		if (down_interruptible(&pnet_fifo_private[minor].lock))
			return -ERESTARTSYS;
	}
	/* data is now available or port got disconneted 
	 * check disconneted again
	 * EOF when disconnected
	 */
	if (pnet_fifo_private[minor].disconnected) {
		up(&pnet_fifo_private[minor].lock);
		return 0; /* EOF */
	}
	/* TODO: optimize reading all data from port */
	/* TODO: consider to provide core function and write directly to user*/
	count=min(count,(size_t)PNET_FIFO_READ_BUFFER_LEN);
	count=pnet_core_port_read(&pnet_fifo_port[minor],pnet_fifo_private[minor].read_buffer,count);
	if (!count) {
		/* got disconnected and buffers got cleared while read */
		up(&pnet_fifo_private[minor].lock);
		return 0; /* EOF */
	}
	dbg("%d Bytes read",count);
	if (copy_to_user(buf,pnet_fifo_private[minor].read_buffer,count)) {
		up(&pnet_fifo_private[minor].lock);
		err("copy_to_user failed");
		return -EFAULT;
	}
	up(&pnet_fifo_private[minor].lock);
	return count;
}


static ssize_t pnet_fifo_write (struct file *file, const char __user *buf, size_t count,
                          loff_t *offset)
{
	int minor;

	dbg("pnet_fifo_write called");

	minor = iminor(file->f_dentry->d_inode);
	if (minor>=PNET_FIFO_DEV_N)
		return -EINVAL;

	if (down_interruptible(&pnet_fifo_private[minor].lock))
		return -ERESTARTSYS;

	/* error EPIPE when disconnected */
	if (pnet_fifo_private[minor].disconnected) {
		up(&pnet_fifo_private[minor].lock);
		return -EPIPE;
	}

	/* wait for free space or disconnected */
	while(!(pnet_core_port_write_free(&pnet_fifo_port[minor]) || pnet_fifo_private[minor].disconnected)) {
		up(&pnet_fifo_private[minor].lock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(pnet_fifo_private[minor].write_wait,
		   (pnet_core_port_write_free(&pnet_fifo_port[minor]) || pnet_fifo_private[minor].disconnected)))
			return -ERESTARTSYS;
		if (down_interruptible(&pnet_fifo_private[minor].lock))
			return -ERESTARTSYS;
	}
	/* free space is now available or port got disconnected
	 * check disconneted again
	 * error EPIPE when disconnected
	 */
	if (pnet_fifo_private[minor].disconnected) {
		up(&pnet_fifo_private[minor].lock);
		return -EPIPE;
	}
	/* TODO: optimize writing all data from port */
	/* TODO: consider to provide core function and read directly from user */
	count=min(count,(size_t)PNET_FIFO_WRITE_BUFFER_LEN);
	if (copy_from_user(pnet_fifo_private[minor].write_buffer,buf,count)) {
		up(&pnet_fifo_private[minor].lock);
		err("copy_from_user failed");
		return -EFAULT;
	}
	count=pnet_core_port_write(&pnet_fifo_port[minor],pnet_fifo_private[minor].write_buffer,count);
	if (!count) {
		/* got disconnected and buffers got cleared while write */
		up(&pnet_fifo_private[minor].lock);
		return -EPIPE;
	}
	dbg("%d Bytes written",count);
	up(&pnet_fifo_private[minor].lock);
	return count;

}

static unsigned int pnet_fifo_poll (struct file *file, poll_table *wait)
{
	int minor;
	unsigned int mask=0;

	dbg("pnet_fifo_poll called");

	minor = iminor(file->f_dentry->d_inode);
	if (minor>=PNET_FIFO_DEV_N)
		return -EINVAL;

	if (down_interruptible(&pnet_fifo_private[minor].lock))
		return -ERESTARTSYS;
	poll_wait(file,&pnet_fifo_private[minor].read_wait,wait);
	poll_wait(file,&pnet_fifo_private[minor].write_wait,wait);
 	
	if (pnet_core_port_write_free(&pnet_fifo_port[minor])||pnet_fifo_private[minor].disconnected)
		mask |= POLLOUT | POLLWRNORM;
	if (pnet_core_port_read_len(&pnet_fifo_port[minor])||pnet_fifo_private[minor].disconnected)
		mask |= POLLIN | POLLRDNORM;
	up(&pnet_fifo_private[minor].lock);
	return mask;
}

static int pnet_fifo_open(struct inode *inode, struct file *file)
{
	int minor;
	int ret;

	dbg("pnet_fifo_open called");
	minor=iminor(inode);
	if (minor>=PNET_FIFO_DEV_N)
		return -EINVAL;

	if (down_interruptible(&pnet_fifo_private[minor].lock))
		return -ERESTARTSYS;
	if ((file->f_flags&O_ACCMODE)==O_RDWR) {
		if (pnet_fifo_private[minor].write_count>0 || pnet_fifo_private[minor].read_count>0) {
			up(&pnet_fifo_private[minor].lock);
			return -EBUSY;
		}
		pnet_fifo_private[minor].write_count++;
		pnet_fifo_private[minor].read_count++;
	}
	else if ((file->f_flags&O_ACCMODE)==O_WRONLY) {
		if (pnet_fifo_private[minor].write_count>0) {
			up(&pnet_fifo_private[minor].lock);
			return -EBUSY;
		}
		pnet_fifo_private[minor].write_count++;
	}
	else if ((file->f_flags&O_ACCMODE)==O_RDONLY) {
		if (pnet_fifo_private[minor].read_count>0) {
			up(&pnet_fifo_private[minor].lock);
			return -EBUSY;
		}
		pnet_fifo_private[minor].read_count++;
	}
	/* reset disconnected before wait */
	pnet_fifo_private[minor].disconnected=0;
	/* wait for connected */
	if (pnet_fifo_port[minor].state!=PNET_CORE_PORT_CONNECTED) {
		if (file->f_flags & O_NONBLOCK) {
			ret=-EPIPE;
			goto open_error;
		}
		if (wait_event_interruptible(pnet_fifo_private[minor].open_wait,(pnet_fifo_port[minor].state==PNET_CORE_PORT_CONNECTED))) {
			ret=-ERESTARTSYS;
			goto open_error;
		}
	}
	up(&pnet_fifo_private[minor].lock);
	return 0;

open_error:
	if ((file->f_flags&O_ACCMODE)==O_RDWR || (file->f_flags&O_ACCMODE)==O_WRONLY) 
		if (pnet_fifo_private[minor].write_count>0) pnet_fifo_private[minor].write_count--;
	if ((file->f_flags&O_ACCMODE)==O_RDWR || (file->f_flags&O_ACCMODE)==O_RDONLY) 
		if (pnet_fifo_private[minor].read_count>0) pnet_fifo_private[minor].read_count--;
	up(&pnet_fifo_private[minor].lock);
	return ret;
}

static int pnet_fifo_close(struct inode *inode, struct file *file)
{
	int minor;
	dbg("pnet_fifo_close called");
	minor=iminor(inode);
	if (minor>=PNET_FIFO_DEV_N)
		return -EINVAL;
	if (down_interruptible(&pnet_fifo_private[minor].lock))
		return -ERESTARTSYS;
	if ((file->f_flags&O_ACCMODE)==O_RDWR || (file->f_flags&O_ACCMODE)==O_WRONLY) 
		if (pnet_fifo_private[minor].write_count>0) pnet_fifo_private[minor].write_count--;
	if ((file->f_flags&O_ACCMODE)==O_RDWR || (file->f_flags&O_ACCMODE)==O_RDONLY) 
		if (pnet_fifo_private[minor].read_count>0) pnet_fifo_private[minor].read_count--;
	up(&pnet_fifo_private[minor].lock);
	return 0;
}

static struct file_operations pnet_fifo_fops = {
	.owner = THIS_MODULE,
	.read = pnet_fifo_read,
	.write = pnet_fifo_write,
	.poll = pnet_fifo_poll,
	.open = pnet_fifo_open,
	.release = pnet_fifo_close,
};

static struct class *pnet_fifo_class;


static int __init pnet_fifo_init(void)
{
	int i;
	int ret=0;
	dbg("pnet_fifo_init called");
	
	memset(&pnet_fifo_private, 0, sizeof(pnet_fifo_private_t)*PNET_FIFO_DEV_N);
	logs_init();

	for (i=0;i<PNET_FIFO_DEV_N;i++) {
		init_MUTEX(&pnet_fifo_private[i].lock);
		init_waitqueue_head(&pnet_fifo_private[i].read_wait); 
		init_waitqueue_head(&pnet_fifo_private[i].write_wait); 
		init_waitqueue_head(&pnet_fifo_private[i].open_wait); 
		ret=pnet_core_port_register(&pnet_fifo_port[i]);
		if (ret) {
			err("register port %d failed",pnet_fifo_port[i].port); 
			/* unregister registered ports */
			i--;
			while(i--)
				pnet_core_port_unregister(&pnet_fifo_port[i]);
			goto pnet_fifo_init_err_ports;
		}
	}

	/* register chrdev */
	ret=register_chrdev(DRV_MAJOR,DRV_NAME,&pnet_fifo_fops);
	if (ret) {
		err("register chrdev failed");
		goto pnet_fifo_init_err_cdev;
	}

	/* register device class */
	pnet_fifo_class=class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(pnet_fifo_class)) {
		err("class_create failed");
		ret = PTR_ERR(pnet_fifo_class);
		goto pnet_fifo_init_err_class;
	}

	/* register devices */
	for (i=0;i<PNET_FIFO_DEV_N;i++) {
		pnet_fifo_private[i].device=device_create(pnet_fifo_class,NULL,MKDEV(DRV_MAJOR,i),NULL,"%s%d",DRV_NAME,i);
		if (IS_ERR(pnet_fifo_private[i].device)) {
			err("device_create failed");
			ret=PTR_ERR(pnet_fifo_private[i].device);
			/* unregister devices */
			i--;
			while(i--)
				device_destroy(pnet_fifo_class,MKDEV(DRV_MAJOR,i));
			goto pnet_fifo_init_err_class_device;
		}	
	}		

	#ifdef CONFIG_PROC_FS
	#endif

	return ret;

pnet_fifo_init_err_class_device:
	class_destroy(pnet_fifo_class);
pnet_fifo_init_err_class:
	unregister_chrdev(DRV_MAJOR,DRV_NAME);
pnet_fifo_init_err_cdev:
	for (i=0;i<PNET_FIFO_DEV_N;i++) 
		pnet_core_port_unregister(&pnet_fifo_port[i]);
pnet_fifo_init_err_ports:
	return ret;
}

static void __exit pnet_fifo_exit(void)
{
	int i;
	dbg("pnet_fifo_exit called");
	#ifdef CONFIG_PROC_FS
	#endif
	for (i=0;i<PNET_FIFO_DEV_N;i++)
		device_destroy(pnet_fifo_class,MKDEV(DRV_MAJOR,i));
	class_destroy(pnet_fifo_class);
	unregister_chrdev(DRV_MAJOR,DRV_NAME);
	for (i=0;i<PNET_FIFO_DEV_N;i++)
		pnet_core_port_unregister(&pnet_fifo_port[i]);
}

module_init( pnet_fifo_init );
module_exit( pnet_fifo_exit );

MODULE_AUTHOR("Andreas Schrattenecker");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PNET fifo driver");
