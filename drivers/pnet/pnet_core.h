#ifndef _PNET_CORE_H
#define _PNET_CORE_H

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <asm/atomic.h>


#define PNET_CORE_PACKET_DATA_MAX	16
#define PNET_CORE_TX_ORDER_SIZE_MIN	sizeof(struct pnet_core_order)

#define PNET_CORE_IGNORE_INVALID_PORT_PACKETS	0

struct pnet_core_packet
{
	unsigned int pri;
	unsigned int port;
	unsigned int len;
	unsigned char data[PNET_CORE_PACKET_DATA_MAX];
};


/* device states */
#define PNET_CORE_DEVICE_DISCONNECTED	0
#define PNET_CORE_DEVICE_CONNECTED	1

struct pnet_core_dev
{
	struct module *owner;
	unsigned int adr;	
	unsigned int pri_n;
	unsigned int port_n;
	unsigned int data_n;
	int (*send_packet)(struct pnet_core_dev *dev,struct pnet_core_packet *packet);
	int (*receive_packet)(struct pnet_core_dev *dev,struct pnet_core_packet *packet);
	/* core private */
	unsigned int state;
	unsigned int state_new;
	struct list_head list;
	struct tasklet_struct tl_descr;
	struct pnet_core_pri *pri;
	struct pnet_core_port **port_table;
	spinlock_t lock;
};


struct pnet_core_order
{
	unsigned int order_nr;
	unsigned int size;	
};

#define PNET_CORE_FIFO_LOCK		1
#define PNET_CORE_FIFO_COMPLETE		2
#define PNET_CORE_FIFO_NOREMOVE		4

#define PNET_CORE_FIFO_FORCELOCK	0

struct pnet_core_fifo {
	unsigned int size;	
	unsigned int head;	
	unsigned int tail;	
	unsigned char *buffer;	
	spinlock_t lock;	
};

struct pnet_core_pri
{
	unsigned int pri;
	struct pnet_core_dev *dev;
	struct list_head port_list;
	unsigned int order_head;
	unsigned int order_tail;
};


/* port states */
#define PNET_CORE_PORT_DISCONNECTED		0
#define PNET_CORE_PORT_CONNECTED		1

/* port options */
#define PNET_CORE_PORT_NOCLEAR_ON_DISCONNECT	0
#define PNET_CORE_PORT_RXCLEAR_ON_DISCONNECT	1
#define PNET_CORE_PORT_TXCLEAR_ON_DISCONNECT	2
#define PNET_CORE_PORT_CLEAR_ON_DISCONNECT	(PNET_CORE_PORT_RXCLEAR_ON_DISCONNECT|PNET_CORE_PORT_TXCLEAR_ON_DISCONNECT)

struct pnet_core_port
{
	struct module *owner;
	unsigned int adr;
	unsigned int pri;
	unsigned int port;
	unsigned int rx_size;
	unsigned int tx_size;
	unsigned int tx_order_size;
	void (*sent_data)(struct pnet_core_port *port);
	void (*received_data)(struct pnet_core_port *port);
	void (*changed_state)(struct pnet_core_port *port, int state);
	void *private_data;
	unsigned int options;
	/* core private */
	unsigned int state;
	struct list_head list;
	struct pnet_core_dev *dev;
	struct pnet_core_fifo rx_fifo;
	struct pnet_core_fifo tx_fifo;
	struct pnet_core_fifo tx_order;
	int rx_pending;
	/* stats */
	unsigned int stat_rx_bytes;
	unsigned int stat_rx_packets;
	unsigned int stat_rx_callbacks;
	unsigned int stat_tx_bytes;
	unsigned int stat_tx_packets;
	unsigned int stat_tx_callbacks;
	

};

int pnet_core_device_register(struct pnet_core_dev *dev);
int pnet_core_device_unregister(struct pnet_core_dev *dev);
int pnet_core_device_transfer(struct pnet_core_dev *dev);
int pnet_core_device_state(struct pnet_core_dev *dev, unsigned int state);
int pnet_core_packet_receive_ready(struct pnet_core_dev *dev, struct pnet_core_packet *packet);
int pnet_core_port_register(struct pnet_core_port *port);
int pnet_core_port_unregister(struct pnet_core_port *port);
unsigned int pnet_core_port_read(struct pnet_core_port *port, unsigned char *buffer, unsigned int count);
unsigned int pnet_core_port_read_len(struct pnet_core_port *port);
unsigned int pnet_core_port_read_free(struct pnet_core_port *port);
unsigned int pnet_core_port_write(struct pnet_core_port *port, unsigned char *buffer, unsigned int count);
unsigned int pnet_core_port_write_len(struct pnet_core_port *port);
unsigned int pnet_core_port_write_free(struct pnet_core_port *port);

#endif
