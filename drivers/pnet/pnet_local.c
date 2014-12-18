/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 *           Andreas Schrattenecker, Ginzinger electronic systems GmbH
 * @Descr: Local Packet Driver
 * @TODO:
 *    o make functional 
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
 

#define DRV_NAME		"pnet_local"


#define PNET_LOCAL_PACKET_DEV_START	2
#define PNET_LOCAL_PACKET_PRI_BITS	2
#define PNET_LOCAL_PACKET_PORT_BITS	4
#define PNET_LOCAL_PACKET_PRI_N		(1<<PNET_LOCAL_PACKET_PRI_BITS)
#define PNET_LOCAL_PACKET_PORT_N	(1<<PNET_LOCAL_PACKET_PORT_BITS)
#define	PNET_LOCAL_PACKET_DATA_LEN	10


#ifndef MIN
#define MIN(a,b)	((a)>(b) ? (b) : (a))
#endif
 
#ifndef MAX
#define MAX(a,b)	((a)>(b) ? (a) : (b))
#endif


//#define DEBUG

#ifdef DEBUG
#define dbg(format, arg...) printk(KERN_INFO "%s: %s: " format "\n" , __FILE__, __FUNCTION__, ## arg)
//#define dbg(format, arg...) printk(KERN_DEBUG "%s: " format "\n" , __FILE__ , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif

#define err(format, arg...) printk(KERN_ERR "%s: %s: " format "\n" , __FILE__, __FUNCTION__, ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: %s: " format "\n" , __FILE__, __FUNCTION__, ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: %s: " format "\n", __FILE__, __FUNCTION__, ## arg)

static int pnet_local_send_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet);
static int pnet_local_receive_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet);

static struct pnet_core_dev pnet_local_pnet_core_dev[2] = {
	{
		.owner = THIS_MODULE,
		.adr = PNET_LOCAL_PACKET_DEV_START+0,
		.pri_n = PNET_LOCAL_PACKET_PRI_N,
		.port_n = PNET_LOCAL_PACKET_PORT_N,
		.data_n = PNET_LOCAL_PACKET_DATA_LEN,
		.send_packet = pnet_local_send_pnet_core_packet,
		.receive_packet = pnet_local_receive_pnet_core_packet
	},
	{
		.owner = THIS_MODULE,
		.adr = PNET_LOCAL_PACKET_DEV_START+1,
		.pri_n = PNET_LOCAL_PACKET_PRI_N,
		.port_n = PNET_LOCAL_PACKET_PORT_N,
		.data_n = PNET_LOCAL_PACKET_DATA_LEN,
		.send_packet = pnet_local_send_pnet_core_packet,
		.receive_packet = pnet_local_receive_pnet_core_packet
	}
};


#define PNET_LOCAL_BUF_SIZE 20
static int pnet_local_buf_iptr[2];
static int pnet_local_buf_optr[2];
static int pnet_local_buf_elements[2];
static struct pnet_core_packet pnet_local_buf[PNET_LOCAL_BUF_SIZE][2];
static spinlock_t pnet_local_buf_lock[2];

static int pnet_local_send_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet)
{
	int source_dev=(dev->adr-PNET_LOCAL_PACKET_DEV_START);
	int dest_dev=1-source_dev;

	spin_lock_bh(&pnet_local_buf_lock[dest_dev]);

	if(pnet_local_buf_elements[dest_dev]==PNET_LOCAL_BUF_SIZE) {
		err("buffer full");
		spin_unlock_bh(&pnet_local_buf_lock[dest_dev]);
		return -1;
	}
	memcpy(&pnet_local_buf[pnet_local_buf_iptr[dest_dev]][dest_dev],packet,sizeof(struct pnet_core_packet));
	pnet_local_buf_iptr[dest_dev]++;
	pnet_local_buf_elements[dest_dev]++;
	if(pnet_local_buf_iptr[dest_dev]>=PNET_LOCAL_BUF_SIZE)
		 pnet_local_buf_iptr[dest_dev]=0;

	spin_unlock_bh(&pnet_local_buf_lock[dest_dev]);

	pnet_core_device_transfer(&pnet_local_pnet_core_dev[dest_dev]);
	pnet_core_device_transfer(&pnet_local_pnet_core_dev[source_dev]);

	return 0;	
}

static int pnet_local_receive_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet)
{
	int ret;
	int source_dev=(dev->adr-PNET_LOCAL_PACKET_DEV_START);
	int dest_dev=1-source_dev;

	spin_lock_bh(&pnet_local_buf_lock[source_dev]);
	if(pnet_local_buf_elements[source_dev]==0) {
		err("buffer empty");
		spin_unlock_bh(&pnet_local_buf_lock[source_dev]);
		return -ENODATA;
	}
	memcpy(packet,&pnet_local_buf[pnet_local_buf_optr[source_dev]][source_dev],sizeof(struct pnet_core_packet));
	spin_unlock_bh(&pnet_local_buf_lock[source_dev]);

	ret=pnet_core_packet_receive_ready(dev, packet);
	switch (ret) {
		case 0:
		case -ENOPROTOOPT:
			spin_lock_bh(&pnet_local_buf_lock[source_dev]);
			pnet_local_buf_optr[source_dev]++;
			pnet_local_buf_elements[source_dev]--;
			if(pnet_local_buf_optr[source_dev]>=PNET_LOCAL_BUF_SIZE) 
				pnet_local_buf_optr[source_dev]=0;
			spin_unlock_bh(&pnet_local_buf_lock[source_dev]);
		default:
			return ret;
	}

	pnet_core_device_transfer(&pnet_local_pnet_core_dev[source_dev]);

	return -ENODATA;
}



static int __init pnet_local_init(void)
{
	info("pnet_local (Linux) starting");

	dbg("pnet_local_init called");

	spin_lock_init(&pnet_local_buf_lock[0]);
	spin_lock_init(&pnet_local_buf_lock[1]);


	if (pnet_core_device_register(&pnet_local_pnet_core_dev[0])!=0)
	{
		err("Error registering pnet_local-device 0");
		return -EIO;
	}

	if (pnet_core_device_register(&pnet_local_pnet_core_dev[1])!=0)
	{
		err("Error registering pnet_local-device 1");
		pnet_core_device_unregister(&pnet_local_pnet_core_dev[0]);
		return -EIO;
	}

	pnet_core_device_state(&pnet_local_pnet_core_dev[0], PNET_CORE_DEVICE_CONNECTED);
	pnet_core_device_state(&pnet_local_pnet_core_dev[1], PNET_CORE_DEVICE_CONNECTED);
	return 0;
}

static void __exit pnet_local_exit(void)
{
	dbg("pnet_local_exit called");

	pnet_core_device_unregister(&pnet_local_pnet_core_dev[0]);
	pnet_core_device_unregister(&pnet_local_pnet_core_dev[1]);
}

module_init( pnet_local_init );
module_exit( pnet_local_exit );

MODULE_AUTHOR("Manfred Schlaegl");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Loopback");
