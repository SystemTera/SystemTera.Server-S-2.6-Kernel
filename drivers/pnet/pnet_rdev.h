#ifndef _PNET_RDEV_H_
#define _PNET_RDEV_H_

#include <linux/module.h>
#include <linux/interrupt.h>

/* enable state countes */
#define PNET_RDEV_DEBUG_STATE_COUNTERS

/* sizeof tx and rx datafifo */
#define PNET_RDEV_DATAFIFO_SIZE					1024

/* maximum registers per port */
#define PNET_RDEV_REGISTER_MAX					128

/* default timeout for register-operations on linux-driver-interface (userspace) */
#define PNET_RDEV_LIN_DEFAULT_TIMEOUT			(2*HZ)


/* type of register */
typedef unsigned int pnet_rdev_register_t;


/* pnet register data-fifo for rx/tx data (regnr=0) */
struct pnet_rdev_datafifo
{
	unsigned char data[PNET_RDEV_DATAFIFO_SIZE];
	int iptr,optr,size;
	spinlock_t slock;
};

/* registerset definition */
struct pnet_rdev_registerset
{
	unsigned int changed[(PNET_RDEV_REGISTER_MAX+31)/32];
	pnet_rdev_register_t reg[PNET_RDEV_REGISTER_MAX];
	unsigned int version;
};

typedef struct pnet_rdev_private
{
	/* driver flags */
	int write_count;	/* write open count */
	int read_count;		/* read open count */
	int disconnected;	/* if set device got disconnected after open */
	struct device *device;

	/* lock */
	struct semaphore lock;

	/* register states */
	/* received register set */
	struct pnet_rdev_registerset	regset;
	/* locally changed register set */
	struct pnet_rdev_registerset	regset_local;
	/* remotly changed register set */
	struct pnet_rdev_registerset	regset_remote;
	/* lock */
	spinlock_t slock;

	/* states */
	/* commit state */
	unsigned char register_commit_state;
	/* commit_ack state */
	unsigned char register_commit_ack_state;
	/* send space state */
	unsigned char flowctrl_state;
	/* remote commit received */
	unsigned char remote_commit_received;

	/* flowctrl-data */
	/* remote receive space */
	unsigned short remote_space;
	/* bytes read counter */
	unsigned short bytes_received;

	/* data fifos */
	struct pnet_rdev_datafifo datafifo_tx;
	struct pnet_rdev_datafifo datafifo_rx;

	/* associated pnet-port */
	struct pnet_core_port *port;

	/* callbacks */
	void (*register_callback)(struct pnet_rdev_private *pnet_rdev, int evcode);
	unsigned int register_callback_calls;
	void (*data_callback)(struct pnet_rdev_private *pnet_rdev, int evcode);
	unsigned int data_callback_calls;

	/* wait queues */
	wait_queue_head_t open_wait;
	wait_queue_head_t snd_wait;
	wait_queue_head_t rcv_wait;

	/* tasklets */
	struct tasklet_struct worker_tasklet;

	/* private client data */
	void *priv;

#ifdef PNET_RDEV_DEBUG_STATE_COUNTERS
	unsigned int sent_commits;
	unsigned int received_commit_acks;
	unsigned int received_commits;
	unsigned int sent_commit_acks;
	unsigned int sent_flowctrl;
	unsigned int received_flowctrl;
#endif
}pnet_rdev_private_t;

/*
 * callback - event-states for register and data callbacks
 */
#define PNET_RDEV_CALLBACK_EV_CONNECTED			1	/* event -> pnet_rdev connected */
#define PNET_RDEV_CALLBACK_EV_DISCONNECTED		2	/* event -> pnet_rdev disconnected */
#define PNET_RDEV_CALLBACK_EV_RECEIVED			3	/* event -> pnet_rdev received */
#define PNET_RDEV_CALLBACK_EV_TRANSMITTED		4	/* event -> pnet_rdev transmitted */

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
int pnet_rdev_request(int minor, pnet_rdev_private_t **pnet_rdev, void *priv, void (*register_callback)(pnet_rdev_private_t *, int), void (*data_callback)(pnet_rdev_private_t *, int));

/*
 * open pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 * return: <0 .. error; 0 .. ok
 */
int pnet_rdev_open(pnet_rdev_private_t *pnet_rdev);

/*
 * close pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 * return: <0 .. error; 0 .. ok
 */
int pnet_rdev_close(pnet_rdev_private_t *pnet_rdev);

/*
 * get rx-fifo-len of pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 * return: <0 .. error; >0 .. read len
 */
int pnet_rdev_read_len (pnet_rdev_private_t *pnet_rdev);

/*
 * read data of pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	buf .. buffer to read data in
 *	count .. number of bytes to read
 * return: <0 .. error; >0 .. read len
 */
int pnet_rdev_read (pnet_rdev_private_t *pnet_rdev, char *buf, int count);

/*
 * get tx-fifo-space of pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 * return: <0 .. error; >0 .. write size
 */
int pnet_rdev_write_free (pnet_rdev_private_t *pnet_rdev);

/*
 * write data to pnet_rdev minor
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	buf .. buffer to write
 *	count .. number of bytes to write
 * return: <0 .. error; >0 .. write size
 */
int pnet_rdev_write (pnet_rdev_private_t *pnet_rdev, const char *buf, unsigned int count);

/*
 * reset temp registers
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	wait .. 0 .. no wait; >0 .. wait max jiffies
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_rdev_register_reset(pnet_rdev_private_t *pnet_rdev, int wait);

/*
 * read register
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	regnr .. register-number to read
 *	val .. pointer to save value
 * return: <0 .. error; 0 .. not changed, 1 .. changed
 */
int pnet_rdev_register_read(pnet_rdev_private_t *pnet_rdev, unsigned int regnr, pnet_rdev_register_t *val);

/*
 * get changed status of register
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	regnr .. register-number
 * return: <0 .. error; 0 .. not changed, 1 .. changed
 */
int pnet_rdev_register_is_changed(pnet_rdev_private_t *pnet_rdev, unsigned int regnr);

/*
 * wait until a remote commit
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	wait .. 0 .. no wait; >0 .. wait max jiffies
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 1 .. changed
 */
int pnet_rdev_register_wait_remote_commit(pnet_rdev_private_t *pnet_rdev, int wait);

/*
 * write register
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	regnr .. regsiter-number to write
 *	val .. value to write to register
 *	wait .. 0 .. no wait; >0 .. wait max jiffies
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_rdev_register_write(pnet_rdev_private_t *pnet_rdev, unsigned int regnr, pnet_rdev_register_t val, int wait);

/*
 * commit registers
 * parameters:
 *	pnet_rdev .. pnet_rdev instance
 *	wait .. 0 .. no wait; >0 .. wait max jiffies
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_rdev_register_commit(pnet_rdev_private_t *pnet_rdev, int wait);

#endif /* __PNET_RDEV_H__ */

