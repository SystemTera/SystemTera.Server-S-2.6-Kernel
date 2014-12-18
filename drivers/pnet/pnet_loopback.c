/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 *           Andreas Schrattenecker, Ginzinger electronic systems GmbH
 * @Descr: Loopback Packet Driver
 * @TODO:
 *    o ...
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
 ***********************************************************************/
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/seq_file.h>
#include "pnet_core.h"
 

#define DRV_NAME		"pnet_loopback"


#define PNET_LOOPBACK_PACKET_PRI_BITS	2
#define PNET_LOOPBACK_PACKET_PORT_BITS	4
#define PNET_LOOPBACK_PACKET_PRI_N	(1<<PNET_LOOPBACK_PACKET_PRI_BITS)
#define PNET_LOOPBACK_PACKET_PORT_N	(1<<PNET_LOOPBACK_PACKET_PORT_BITS)
#define	PNET_LOOPBACK_PACKET_DATA_LEN	10


#ifndef MIN
#define MIN(a,b)	((a)>(b) ? (b) : (a))
#endif
 
#ifndef MAX
#define MAX(a,b)	((a)>(b) ? (a) : (b))
#endif


//#define DEBUG

#ifdef DEBUG
#define dbg(format, arg...) printk(format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif

#define err(format, arg...) printk(KERN_ERR "%s: " format "\n" , __FILE__ , ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: " format "\n" , __FILE__ , ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: " format "\n" , __FILE__ , ## arg)

static int pnet_loopback_send_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet);
static int pnet_loopback_receive_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet);

static struct pnet_core_dev pnet_loopback_pnet_core_dev = {
	.owner = THIS_MODULE,
	.adr = 1,
	.pri_n = PNET_LOOPBACK_PACKET_PRI_N,
	.port_n = PNET_LOOPBACK_PACKET_PORT_N,
	.data_n = PNET_LOOPBACK_PACKET_DATA_LEN,
	.send_packet = pnet_loopback_send_pnet_core_packet,
	.receive_packet = pnet_loopback_receive_pnet_core_packet,
};

#define PNET_LOOPBACK_BUF_SIZE 10
static int pnet_loopback_buf_iptr;
static int pnet_loopback_buf_optr;
static int pnet_loopback_buf_elements;
static struct pnet_core_packet pnet_loopback_buf[20];
static spinlock_t pnet_loopback_buf_lock=SPIN_LOCK_UNLOCKED;

static int pnet_loopback_send_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet)
{
	unsigned long iflags;

	spin_lock_irqsave(&pnet_loopback_buf_lock,iflags);
	if(pnet_loopback_buf_elements==PNET_LOOPBACK_BUF_SIZE) {
		spin_unlock_irqrestore(&pnet_loopback_buf_lock,iflags);
		return -1;
	}
	memcpy(&pnet_loopback_buf[pnet_loopback_buf_iptr],packet,sizeof(struct pnet_core_packet));
	pnet_loopback_buf_iptr++;
	pnet_loopback_buf_elements++;
	if(pnet_loopback_buf_iptr>=PNET_LOOPBACK_BUF_SIZE) pnet_loopback_buf_iptr=0;
	spin_unlock_irqrestore(&pnet_loopback_buf_lock,iflags);

	pnet_core_device_transfer(dev);

	return 0;	
}

static int pnet_loopback_receive_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet)
{
	unsigned long iflags;
	int ret;

	spin_lock_irqsave(&pnet_loopback_buf_lock,iflags);
	if(pnet_loopback_buf_elements==0) {
		spin_unlock_irqrestore(&pnet_loopback_buf_lock,iflags);
		return -ENODATA;
	}
	memcpy(packet,&pnet_loopback_buf[pnet_loopback_buf_optr],sizeof(struct pnet_core_packet));
	spin_unlock_irqrestore(&pnet_loopback_buf_lock,iflags);

	ret=pnet_core_packet_receive_ready(dev, packet);
	switch (ret) {
		case 0:
		case -ENOPROTOOPT:
			spin_lock_irqsave(&pnet_loopback_buf_lock,iflags);
			pnet_loopback_buf_optr++;
			pnet_loopback_buf_elements--;
			if(pnet_loopback_buf_optr>=PNET_LOOPBACK_BUF_SIZE) pnet_loopback_buf_optr=0;
			spin_unlock_irqrestore(&pnet_loopback_buf_lock,iflags);
		default:
			return ret;
	}

	return -ENODATA;
}



static int __init pnet_loopback_init(void)
{
	info("pnet_loopback (Linux) starting");

	dbg("pnet_loopback_init called");

	if (pnet_core_device_register(&pnet_loopback_pnet_core_dev)!=0)
	{
		err("Error registering pnet_loopback-device");
		return -EIO;
	}

	return 0;
}

static void __exit pnet_loopback_exit(void)
{
	dbg("pnet_loopback_exit called");

	pnet_core_device_unregister(&pnet_loopback_pnet_core_dev);
}

module_init( pnet_loopback_init );
module_exit( pnet_loopback_exit );

MODULE_AUTHOR("Manfred Schlaegl");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Loopback");
