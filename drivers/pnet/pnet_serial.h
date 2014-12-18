#ifndef _PNET_SERIAL_H
#define _PNET_SERIAL_H

#include "pnet_core.h"
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/workqueue.h>

/*
++++++++++++++++++++++++++++++++++
STRUCTURES
++++++++++++++++++++++++++++++++++
*/
/* pnet-channel structure */

struct pnet_serial_channel {
	struct pnet_core_port	data_port;			
	struct pnet_core_port	ctrl_port;				
};

/* PNET CTRL-PACKET (PNET up to pnet-datapacket size?) */
struct pnet_serial_ctrl_packet {
	unsigned int cmd;			// Auszuführendes Kommando
	unsigned int arg;
} __attribute__((packed));

#define PNET_SERIAL_FLAG_POST_INIT		(1<<0)	/* post init (done once per driver-load in open) */
#define PNET_SERIAL_FLAG_DISCONNECTED	(1<<1)	/* device was disconnected */
#define PNET_SERIAL_FLAG_RX_THROTTLED	(1<<2)	/* rx throttled mode (no data fifo -> tty is sent) */
#define PNET_SERIAL_FLAG_TX_THROTTLED	(1<<3)	/* tx throttled mode (no data tty -> fifo is sent) */
#define PNET_SERIAL_FLAG_UNREAD_CREPLY	(1<<4)	/* creply is unread */

/* stucture for a serial-line over pnet */
struct pnet_serial {
	unsigned char				flags;					/* driver flags */
	struct semaphore			mutex;					/* device mutex */	
	spinlock_t					lock;					/* device spinlock */

	struct tty_struct			*tty;					/* associated tty struct (on open) */
	struct tty_driver			*tty_driver;			/* associated tty-driver struct (on register) */
	int 						open;					/* open counter */
	struct pnet_serial_channel	pnet_channel;			/* channel for pnet-connection */

	unsigned int				tx_count;				/* for statistics */
	unsigned int				rx_count;				/* for statistics */

	wait_queue_head_t			sent_wq;				/* any sent-wq */ 
	struct tasklet_struct		dread_tl;				/* data read work tasklet */
	struct tasklet_struct		cread_tl;				/* ctrl read work tasklet */

	wait_queue_head_t			csent_wq;				/* ctrl sent-wq */
	wait_queue_head_t			crcv_wq;				/* ctrl receive-wq */

	wait_queue_head_t			conn_state_changed_wq;	/* connection state changed wq */    

	unsigned int				control_state;			/* for tiocmset/get (dtr/rts)*/

	struct pnet_serial_ctrl_packet	creply;				/* last reply on ctrl port received */
};



/*
++++++++++++++++++++++++++++++++++
CTRL-COMMUNICATION
++++++++++++++++++++++++++++++++++
*/

#define PNET_SERIAL_CMD_FLAG_REQUEST	(1<<31)
#define PNET_SERIAL_CMD_FLAG_REPLY		(1<<30)

/* Commands (request-data is parameter, reply-data is retval) */
/* Set-Commands */
#define PNET_SERIAL_CMD_SET_BAUDRATE 	1		/* set baudrate (coded as table-index (see table below)) */
#define PNET_SERIAL_CMD_SET_DATAB		2		/* set data-bits (5-8) */
#define PNET_SERIAL_CMD_SET_PARITY		3		/* set parity (0..none, 1..even, 2..odd) */
#define PNET_SERIAL_CMD_SET_STOPB		4		/* set stob-bits (1-2) */
#define PNET_SERIAL_CMD_SET_BREAK_CTL	5		/* set break_ctl (bool) */
#define PNET_SERIAL_CMD_SET_RTS			6		/* set RTS (bool) */
#define PNET_SERIAL_CMD_SET_DTR			7		/* set DTR (bool) */
#define PNET_SERIAL_CMD_SET_RECEIVE		8		/* enable receiver (bool) */
#define PNET_SERIAL_CMD_SET_FLOWCTRL	9		/* set flowcontrol (see flow-control-bits below) */
#define PNET_SERIAL_CMD_SEND_XCHAR		10		/* send high priority char */
#define PNET_SERIAL_CMD_SET_ENABLE		11		/* send high priority char */
#if 0 
#define PNET_SERIAL_CMD_SET_LOOPBACK	10		/* set Loopback (bool) -> Not implementet yet */ 
#endif

/* Get-Commands */
#define PNET_SERIAL_CMD_GET_LINES		100		/* read status of (DTR, RTS), CTS, DSR, CD, RI, LL */
#define PNET_SERIAL_CMD_GET_TXCOUNT		101		/* get # of sent bytes */


/* LINE_BITS */
/* important */
union pnet_serial_ctrl_lines {
	unsigned int val;
	struct {
		unsigned DTR : 1;
		unsigned RTS : 1;
		unsigned CTS : 1;
		unsigned DSR : 1;
		unsigned CD : 1;
		unsigned RI : 1;
		unsigned LL : 1;
		unsigned error : 1; 	/* bit is set on error */
	} bits;
};


/* Flow-control bits */
union pnet_serial_flow_ctrl {
	unsigned int val;
	struct {
		unsigned oxon : 1; 	/* output xon/xoff (bit)*/
		unsigned ixon : 1; 	/* input xon/xoff (bit)*/
		unsigned crtscts : 1; 	/* rts cts handshake */
		unsigned clocal: 1; /* use dtr/cd */
		unsigned fill: 12;	/* pad */
		char vstart;		/* start character (xon) */
		char vstop;			/* stop character (xoff) */
	} bits;
};

/* baud-rate-table (based on serial_ns9750.c) for coded baud-rate */
#if 0
static unsigned long pnet_serial_baud_rates[] = {50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800,
	9600, 19200, 38400, 57600, 115200, 230400, 460800};
#endif

#endif
