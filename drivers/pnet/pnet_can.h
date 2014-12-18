#ifndef __LIBPNET_CAN_H__
#define __LIBPNET_CAN_H__

/***********************************************************************
 *  @History:
 *	2010-KW10 manfred.schlaegl@gmx.at
 *		* new api-functions (pnet_can_mbox_set_enabled/pnet_can_mbox_set_disabled)
 *		* comments changed
 ***********************************************************************/
/***********************************************************************
 *  @TODO:
 ***********************************************************************/

#ifdef __KERNEL__
#define PNET_CAN_VERSION_STR					"pnet-can-1.0"
#else
#define PNET_CAN_VERSION_STR					"libpnet-can-1.0"
#endif

/* pnet rdev minor-nr */
#define PNET_CAN_RDEV_MINOR						2

#define PNET_CAN_MASK_MAX						32
#define PNET_CAN_MAILBOX_MAX					32

/* default timeout for operations on linux-driver-interface (userspace) */
#define PNET_CAN_LIN_DEFAULT_TIMEOUT			PNET_RDEV_LIN_DEFAULT_TIMEOUT

/*
 **************************************************************************************************
 * Begin: IOCTL-Definitions
 **************************************************************************************************
 */

/*
 * get info
 */
#define PNET_CAN_IOCTL_CMD_INFO					0

/* info-struct */
struct pnet_can_info {
	unsigned short version_lo;	/* driver version-nr (lo)		*/
	unsigned short version_hi;	/* driver version-nr (hi)		*/
	unsigned short mbox_n;		/* number of mboxes 			*/
	unsigned short mask_n;		/* number of acceptance masks	*/
};

/*
 * enable can
 */
#define PNET_CAN_IOCTL_CMD_ENABLE				1

/*
 * config can
 */
#define PNET_CAN_IOCTL_CMD_CONFIG				2

/* config-struct */
struct pnet_can_config {
	unsigned int bitrate;					/* bitrate 					*/
	unsigned int sample_point;				/* sample-point in percent	*/
	unsigned int edge_resync_mode;			/* enable resync mode 		*/
	unsigned int sync_jump_width;			/* sync jump width 			*/
	unsigned int byte_order;				/* byte-order				*/
	unsigned int mask[PNET_CAN_MASK_MAX];	/* acceptance masks			*/
};

/* mask-flags */
#define PNET_CAN_CONFIG_MASK_AMI	(1<<31)	/* acceptance mask extendet id bit */

/*
 * config mbox
 */
#define PNET_CAN_IOCTL_CMD_MBOX_CONFIG			3

/* types */
#define CAN_MBOX_TYPE_TX 			(1<<0)
#define CAN_MBOX_TYPE_RX			(1<<1)
#define CAN_MBOX_TYPE_REQUEST		(CAN_MBOX_TYPE_RX|(1<<2))
#define CAN_MBOX_TYPE_REPLY			(CAN_MBOX_TYPE_TX|(1<<3))

/* config-struct */
struct pnet_can_mbox_config {
	unsigned int mbox;						/* mbox-number 					*/
	unsigned int type;						/* mbox-type					*/
	unsigned int message_id;				/* message-flags + message_id 	*/
	unsigned int data_len;					/* len of data-load				*/
};

/* message_id - flags */
#define PNET_CAN_MBOX_CONFIG_IDE	(1<<31)	/* extended 22bit identifier 	*/
#define PNET_CAN_MBOX_CONFIG_AME	(1<<30)	/* enable acceptance mask		*/

/*
 * enable mboxes
 */
#define PNET_CAN_IOCTL_CMD_MBOX_ENABLE			4
#define PNET_CAN_IOCTL_CMD_MBOX_SET_ENABLED		14
#define PNET_CAN_IOCTL_CMD_MBOX_SET_DISABLED	15

/*
 * async send
 */
#define PNET_CAN_IOCTL_MSG_SND_ASYNC			5
#define PNET_CAN_IOCTL_DATA_SND_ASYNC			6
#define PNET_CAN_IOCTL_UPDATE_SND_ASYNC			7
#define PNET_CAN_IOCTL_REQUEST_SND_ASYNC		8

/*
 * sync send (broken)
 */
#define PNET_CAN_IOCTL_MSG_SND_SYNC				9
#define PNET_CAN_IOCTL_MSG_RCV_SYNC				10
#define PNET_CAN_IOCTL_DATA_SND_SYNC			11
#define PNET_CAN_IOCTL_UPDATE_SND_SYNC			12
#define PNET_CAN_IOCTL_REQUEST_SND_SYNC			13

/* message-struct */
struct pnet_can_msg {
	unsigned short mbox;					/* sender/receiver mbox 	*/
	unsigned short data_len;				/* worload					*/
	unsigned int message_id;				/* message-id				*/
	unsigned char data[8];					/* data						*/
};

/*
 **************************************************************************************************
 * End: IOCTL-Definitions
 **************************************************************************************************
 */

#ifdef __KERNEL__
/***********************************************************************
 *  Begin: kernel api
 ***********************************************************************/

/* protocol definition */
#define PNET_CAN_MSGTYPE_MSG		0	/* mbox, message_id, data */
#define PNET_CAN_MSGTYPE_DATA		1	/* mbox, data */
#define PNET_CAN_MSGTYPE_UPDATE		2	/* mbox, data */
#define PNET_CAN_MSGTYPE_REQUEST	3	/* mbox */

/* mailbox data */
typedef char can_data[8];
#define can_data_cp(__md_dest__,__md_src__)		\
	do {						\
		(__md_dest__)[0]=(__md_src__)[0];	\
		(__md_dest__)[1]=(__md_src__)[1];	\
		(__md_dest__)[2]=(__md_src__)[2];	\
		(__md_dest__)[3]=(__md_src__)[3];	\
		(__md_dest__)[4]=(__md_src__)[4];	\
		(__md_dest__)[5]=(__md_src__)[5];	\
		(__md_dest__)[6]=(__md_src__)[6];	\
		(__md_dest__)[7]=(__md_src__)[7];	\
	} while(0)
#define can_data_icp(__md_dest__,__md_src__)		\
	do {						\
		(__md_dest__)[0]=(__md_src__)[7];	\
		(__md_dest__)[1]=(__md_src__)[6];	\
		(__md_dest__)[2]=(__md_src__)[5];	\
		(__md_dest__)[3]=(__md_src__)[4];	\
		(__md_dest__)[4]=(__md_src__)[3];	\
		(__md_dest__)[5]=(__md_src__)[2];	\
		(__md_dest__)[6]=(__md_src__)[1];	\
		(__md_dest__)[7]=(__md_src__)[0];	\
	} while(0)

struct pnet_can_package_header {
	unsigned char mbox_nr;
	unsigned char hdr;			/* high-nibble: message_type; low-nibble: load_len */
} __attribute__((__packed__));	/* 2 bytes */

struct pnet_can_package_load {
	unsigned int message_id;	/* message_id */
	can_data md;				/* data */
}; /* 12 bytes (12%4=0 -> no packed attribute neccessary) */

struct pnet_can_package {
	struct pnet_can_package_header head;
	struct pnet_can_package_load load;
} __attribute__((__packed__));	/* 14 bytes */

struct pnet_can_private;

/*
 * update info-struct of can
 * parameters:
 *	pnet_can .. pnet_can instance
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_update_info(struct pnet_can_private *pnet_can, int wait);

/*
 * enable/disable can controller
 * parametesr:
 *	pnet_can .. pnet_can instance
 *	ena .. enable/disable can (0 .. disable; 1 .. enable)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_enable(struct pnet_can_private *pnet_can, int ena, int wait);

/*
 * configure can controller
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_cfg .. can-configuration
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok; 1 .. invalid settings
 */
int pnet_can_config(struct pnet_can_private *pnet_can, struct pnet_can_config can_cfg, int wait);

/*
 * configures a specified mailbox (mailbox is left disabled)
 * parameter:
 *	pnet_can .. pnet_can instance
 *	can_mbox_cfg .. can-mailbox-configuration
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_config(struct pnet_can_private *pnet_can, struct pnet_can_mbox_config can_mbox_cfg, int wait);

/*
 * set mailboxes enabled or disabled
 * parameters:
 *	pnet_can .. pnet_can instance
 *	ena .. bitfield of mboxes (1..enable/0..disable)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_enable(struct pnet_can_private *pnet_can, unsigned int ena, int wait);

/*
 * set mailboxes enabled
 * parameters:
 *	pnet_can .. pnet_can instance
 *	ena .. bitfield of mboxes (1..enable/0..do nothing)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_set_enabled(struct pnet_can_private *pnet_can, unsigned int ena, int wait);

/*
 * set mailboxes disabled
 * parameters:
 *	pnet_can .. pnet_can instance
 *	dis .. bitfield of mboxes (1..disable/0..do nothing)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_set_disabled(struct pnet_can_private *pnet_can, unsigned int dis, int wait);

/*
 * update data in reply-mailbox - unqueued
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. mailbox-data
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok; 1.. busy
 */
int pnet_can_mbox_update_snd_async(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait);

/*
 * do request on mailbox - unqueued
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok; 1.. busy
 */
int pnet_can_mbox_request_snd_async(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait);

/*
 * send data on mailbox with configured message_id - unqueued
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok; 1.. busy
 */
int pnet_can_mbox_data_snd_async(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait);

/*
 * send can message on mailbox - unqueued
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.message_id .. message_id
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok; 1.. busy
 */
int pnet_can_mbox_msg_snd_async(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait);

/*
 * send can package - queued
 * parameters:
 *	pnet_can .. pnet_can instance
 * 	package .. can-message to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_package_snd_sync(struct pnet_can_private *pnet_can, struct pnet_can_package package, int wait);

/*
 * receive can package - queued
 * parameters:
 *	pnet_can .. pnet_can intance
 * 	package .. can-message received
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_package_rcv_sync(struct pnet_can_private *pnet_can, struct pnet_can_package *package, int wait);

/*
 * send can message on mailbox - queued
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.message_id .. message_id
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_msg_snd_sync(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait);

/*
 * receive can message - queued
 *	pnet_can .. pnet_can instance
 *	can_msg->mbox .. mailbox-number
 *	can_msg->message_id .. message_id
 *	can_msg->data .. data to send
 *	can_msg->data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_msg_rcv_sync(struct pnet_can_private *pnet_can, struct pnet_can_msg *can_msg, int wait);

/*
 * send data on mailbox with configured message_id - queued
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_data_snd_sync(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait);

/*
 * update data in reply-mailbox - queued
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_update_snd_sync(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait);

/*
 * do request on mailbox - queued (NOT RECOMMENDED)
 * parameters:
 *	pnet_can .. pnet_can instance
 *	can_msg.mbox .. mailbox-number
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_mbox_request_snd_sync(struct pnet_can_private *pnet_can, struct pnet_can_msg can_msg, int wait);

/*
 * reset can controller
 * parameters:
 * 	pnet_can .. pnet_can instance
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_reset(struct pnet_can_private *pnet_can, int wait);

/*
 * open can controller
 * parameters:
 * 	minor .. pnet_can-minor
 * 	pnet_can .. pnet_can instance (set by open)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_open(int minor, struct pnet_can_private **pnet_can, int wait);

/*
 * close can controller
 * parameters:
 * 	minor .. pnet_can-minor
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_can_close(int minor, int wait);


/***********************************************************************
 *  End: kernel api
 ***********************************************************************/
#else

#define PNET_CAN_DEVICE	"/dev/pnet_can"

/*
 * update/get info-struct of can (OK)
 * parameters:
 *	fd .. descriptor to can-device
 *	can_info .. pointer to info-structure
 * return: 0 .. ok; !0 .. error (errno .. errorcode)
 */
int pnet_can_update_info(int fd, struct pnet_can_info *can_info);

/*
 * enable/disable can controller (OK)
 * parametesr:
 *	fd .. descriptor to can-device
 *	ena .. enable/disable can (0 .. disable; 1 .. enable)
 * return: 0 .. ok; !0 .. error (errno .. errorcode)
 */
int pnet_can_enable(int fd, int ena);

/*
 * configure can controller (OK)
 * parameters:
 *	fd .. descriptor to can-device
 *	can_cfg .. can-configuration
 * return: 0 .. ok; !0 .. error (errno .. errorcode)
 *
 * tested configurations:
 * bitrates: 20000, 33333, 47619, 50000, 83333, 95238, 100000, 125000, 250000, 500000
 * sample_pt: 87%
 * erm: 0
 * sjw: 1
 * byteorder: 0
 */
int pnet_can_config(int fd, struct pnet_can_config can_cfg);

/*
 * configures a specified mailbox (OK)
 *
 * mailbox will not be enabled
 * parameter:
 *	fd .. descriptor to can-device
 *	can_mbox_cfg .. mailbox-configuration
 * return: 0 .. ok; !0 .. error (errno .. errorcode)
 */
int pnet_can_mbox_config(int fd, struct pnet_can_mbox_config can_mbox_cfg);

/*
 * set mailbox enabled or disabled (OK)
 * parameters:
 *	fd .. descriptor to can-device
 *	ena .. bitfield of mboxes (1..enable/0..disable)
 * return: 0 .. ok; !0 .. error (errno .. errorcode)
 */
int pnet_can_mbox_enable(int fd, unsigned int ena);

/*
 * set mailbox enabled (OK)
 * parameters:
 *	fd .. descriptor to can-device
 *	ena .. bitfield of mboxes (1..enable/0..do nothing)
 * return: 0 .. ok; !0 .. error (errno .. errorcode)
 */
int pnet_can_mbox_set_enabled(int fd, unsigned int ena);

/*
 * set mailbox disabled (OK)
 * parameters:
 *	fd .. descriptor to can-device
 *	dis .. bitfield of mboxes (1..disable/0..do nothing)
 * return: 0 .. ok; !0 .. error (errno .. errorcode)
 */
int pnet_can_mbox_set_disabled(int fd, unsigned int dis);

/*
 * update data in reply-mailbox - unqueued (RECOMMENDED)
 * parameters:
 *	fd .. descriptor to can-device
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. mailbox-data
 * return: 0 .. ok; !0 .. error (errno .. errorcode)
 */
int pnet_can_mbox_update_snd_async(int fd, struct pnet_can_msg can_msg);

/*
 * do request on mailbox - unqueued (RECOMMENDED)
 * parameters:
 *	fd .. descriptor to can-device
 *	can_msg.mbox .. mailbox-number
 * return: 0 .. ok; !0 .. error (errno .. errorcode)
 */
int pnet_can_mbox_request_snd_async(int fd, struct pnet_can_msg can_msg);

/*
 * send data on mailbox with configured message_id - unqueued
 * parameters:
 *	fd .. descriptor to can-device
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 * return: 0 .. ok; !0 .. error (errno .. errorcode)
 */
int pnet_can_mbox_data_snd_async(int fd, struct pnet_can_msg can_msg);

/*
 * send can message on mailbox - unqueued
 * parameters:
 *	fd .. descriptor to can-device
 *	can_msg.mbox .. mailbox-number
 *	can_msg.message_id .. message_id
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 * return: 0 .. ok; !0 .. error (errno .. errorcode)
 */
int pnet_can_mbox_msg_snd_async(int fd, struct pnet_can_msg can_msg);

/*
 * send can message on mailbox - queued
 *
 * put can message to queue and send in background
 * parameters:
 *	fd .. descriptor to can-device
 *	can_msg.mbox .. mailbox-number
 *	can_msg.message_id .. message_id
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 * return: 0 .. ok; !0 .. error (errno: .. errorcode: EAGAIN .. not complete try-again)
 */
int pnet_can_mbox_msg_snd_sync(int fd, struct pnet_can_msg can_msg);

/*
 * receive can message - queued
 * parameters:
 *	fd .. descriptor to can-device
 *	can_msg->mbox .. mailbox-number
 *	can_msg->message_id .. message_id
 *	can_msg->data .. data to send
 *	can_msg->data_len .. len of data to send
 * return: 0 .. ok; !0 .. error (errno: .. errorcode: EAGAIN .. not complete try-again)
 */
int pnet_can_mbox_msg_rcv_sync(int fd, struct pnet_can_msg *can_msg);

/*
 * send data on mailbox with configured message_id - queued
 * parameters:
 *	fd .. descriptor to can-device
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 * return: 0 .. ok; !0 .. error (errno: .. errorcode: EAGAIN .. not complete try-again)
 */
int pnet_can_mbox_data_snd_sync(int fd, struct pnet_can_msg can_msg);

/*
 * update data in reply-mailbox - queued (NOT RECOMMENDED)
 * parameters:
 *	fd .. descriptor to can-device
 *	can_msg.mbox .. mailbox-number
 *	can_msg.data .. data to send
 *	can_msg.data_len .. len of data to send
 * return: 0 .. ok; !0 .. error (errno: .. errorcode: EAGAIN .. not complete try-again)
 */
int pnet_can_mbox_update_snd_sync(int fd, struct pnet_can_msg can_msg);

/*
 * do request on mailbox - queued (NOT RECOMMENDED)
 * parameters:
 *	fd .. descriptor to can-device
 *	can_msg.mbox .. mailbox-number
 * return: 0 .. ok; !0 .. error (errno: .. errorcode: EAGAIN .. not complete try-again)
 */
int pnet_can_mbox_request_snd_sync(int fd, struct pnet_can_msg can_msg);

/*
 * open can controller (OK)
 * parameters:
 * 	minor .. pnet_can-minor
 *	wait .. wait-flag (0 .. no wait(not supported!); 1 .. wait allowed)
 * return: 0 .. ok; !0 .. error (errno: .. errorcode: EAGAIN .. not complete try-again)
 */
int pnet_can_open(int minor, int wait);

/*
 * close can controller (OK)
 * parameters:
 *	fd .. descriptor to can-device
 */
void pnet_can_close(int fd);

#endif


#endif /* __LIBPNET_CAN_H__ */

