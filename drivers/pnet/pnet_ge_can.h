#ifndef __GE_CAN_H__
#define __GE_CAN_H__

/***********************************************************************
 *
 * @Authors: Manfred Schlägl, Ginzinger electronic systems GmbH
 * @Descr: ge_can driver for pnet
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
 *	2010-KW12 manfred.schlaegl@gmx.at
 *		* change in design
 *		* support for
 *			* rpc send
 *			* broadcast receive
 *			* sequence send
 *	2010-KW11 manfred.schlaegl@gmx.at
 *		* port to linux / error-corrections
 *	2010KW07 - manfred.schlaegl
 *		* detailed concept (buildable but untested)
 *	2010KW06 - manfred.schlaegl
 *		* begin/concept
 ***********************************************************************/
/***********************************************************************
 *  @TODO:
 *		* validate concept
 *		* module<->module communication on single node
 ***********************************************************************/

/*
 **************************************************************************************************
 * BEGIN: Definitions
 **************************************************************************************************
 */

#ifdef __KERNEL__
#define PNET_GE_CAN_VERSION_STR					"ge-can-1.0"
#else
#define PNET_GE_CAN_VERSION_STR					"libge-can-1.0"
#endif

/*
 **************************************************************************************************
 * END: Definitions
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * BEGIN: Driver-API
 **************************************************************************************************
 */

struct pnet_ge_can_rpc_data {
	unsigned int timeout;				/* timeout in milliseconds */
	unsigned short message_id;			/* message_id to send */
	unsigned char data[8];				/* data sent & received */
};

/*
 * argument: struct pnet_ge_can_rpc_data
 * 	timeout(in/out): 	if timeout==0 -> default-timeout is set
 *	message_id(in): 	message_id to send
 *	data(in/out): 		data to send and received data
 */
#define PNET_GE_CAN_IOCTL_CMD_RPC_SND			1

/* ge_can header */
struct pnet_ge_can_hdr {
	unsigned int timeout;				/* timeout in milliseconds */
	unsigned char len;					/* len of data-elements */
	unsigned char node_id;				/* id of node */
	unsigned char module_id;			/* id of module */
	unsigned char module_instance_id;	/* id of module-instance */
};

/* ge_can data */
struct pnet_ge_can_data {
	unsigned short parameter_id;		/* id of parameter */
	unsigned char dlen;
	unsigned char data[7];
};

/* seqence */
struct pnet_ge_can_sequence {
	struct pnet_ge_can_hdr hdr;
	struct pnet_ge_can_data *data;
};

/*
 * argument: struct pnet_ge_can_sequence
 *	hdr:
 * 		timeout(in/out):	 		if timeout==0 -> default-timeout is set
 *		len(in): 					has to be >1, but only one broadcast will be received
 *		node_id(out):				node_id of received broadcast
 *		module_id(out): 			module_id of received broadcast
 *		module_instance_id(out):	module_instance_id of received broadcast
 *	data: (broadcast element array)
 *		parameter_id(out): 			parameter_id of received broadcast
 *		dlen(out):					data-len of received broadcast
 *		data(out): 					data of received broadcast
 */
#define PNET_GE_CAN_IOCTL_CMD_BROADCAST_RCV		2

/*
 * argument: struct pnet_ge_can_sequence
 *	hdr:
 * 		timeout(in/out):	 		if timeout==0 -> default-timeout is set
 *		len(in): 					number of data-elements in sequence (has to be >1)
 *		node_id(in):				node_id to sequence to send
 *		module_id(in): 				module_id to sequence to send
 *		module_instance_id(in):		module_instance_id to sequence to send
 *	data: (sequence element array)
 *		parameter_id(in):	 		parameter_id to sequence to send
 *		dlen(in/out):				data-len to send; data-len received
 *		data(in/out): 				data to send; data received
 */
#define PNET_GE_CAN_IOCTL_CMD_SEQUENCE_SND		3

/*
 **************************************************************************************************
 * END: Driver-API
 **************************************************************************************************
 */

#ifdef __KERNEL__

struct pnet_ge_can_private;

/*
 * send rpc
 * parameters:
 *	ge_can .. ge-can structure
 *	ge_can_rpc .. rpc to send
 * 		timeout(in/out): 	if timeout==0 -> default-timeout is set
 *		message_id(in): 	message_id to send
 *		data(in/out): 		data to send and received data
 * return: <0 .. error, 0 .. ok
 */

int pnet_ge_can_rpc_snd(struct pnet_ge_can_private *ge_can, struct pnet_ge_can_rpc_data *ge_can_rpc);

/*
 * receive broadcast
 * parameters:
 *	ge_can .. ge-can structure
 *	ge_can_bc .. pointer to save broadcast-data
 *		hdr:
 * 			timeout(in/out):	 		if timeout==0 -> default-timeout is set
 *			len(in): 					has to be >1, but only one broadcast will be received
 *			node_id(out):				node_id of received broadcast
 *			module_id(out): 			module_id of received broadcast
 *			module_instance_id(out):	module_instance_id of received broadcast
 *		data: (broadcast element array)
 *			parameter_id(out): 			parameter_id of received broadcast
 *			dlen(out):					data-len of received broadcast
 *			data(out): 					data of received broadcast
 * return: <0 .. error, 0 .. ok
 */
int pnet_ge_can_broadcast_rcv(struct pnet_ge_can_private *ge_can, struct pnet_ge_can_sequence *ge_can_bc);


/*
 * send sequence
 * parameters:
 *	ge_can .. ge-can structure
 *	ge_can_seq .. sequence to send
 *		hdr:
 * 			timeout(in/out):	 		if timeout==0 -> default-timeout is set
 *			len(in): 					number of data-elements in sequence (has to be >1)
 *			node_id(in):				node_id to sequence to send
 *			module_id(in): 				module_id to sequence to send
 *			module_instance_id(in):		module_instance_id to sequence to send
 *		data: (sequence element array)
 *			parameter_id(in):	 		parameter_id to sequence to send
 *			dlen(in/out):				data-len to send; data-len received
 *			data(in/out): 				data to send; data received
 * return: <0 .. error; 0 .. ok
 */
int pnet_ge_can_sequence_snd(struct pnet_ge_can_private *ge_can, struct pnet_ge_can_sequence *ge_can_seq);

/*
 * open ge-can device
 * parameters:
 * 	minor .. pnet_ge_can-minor
 *	ge_can .. returned structure
 * return: 0 .. ok; <0 .. error
 */
int pnet_ge_can_open(int minor, struct pnet_ge_can_private **ge_can);

/*
 * close ge-can device
 * parameters:
 * 	minor .. pnet_ge_can-minor
 * return: 0 .. ok; <0 .. error
 */
int pnet_ge_can_close(int minor);

#else /* __KERNEL__ */

#define PNET_GE_CAN_DEVICE	"/dev/pnet_ge_can"

/*
 * send rpc
 * parameters:
 *	fd .. descriptor of ge-can_device
 *	ge_can_rpc .. rpc to send
 * 		timeout(in/out): 	if timeout==0 -> default-timeout is set
 *		message_id(in): 	message_id to send
 *		data(in/out): 		data to send and received data
 * return: <0 .. error, 0 .. ok
 */
int pnet_ge_can_rpc_send(int fd, struct pnet_ge_can_rpc_data *ge_can_rpc);

/*
 * receive broadcasts
 * parameters:
 *	fd .. descriptor of ge-can-device
 *	bc .. broadcast received
 *		hdr:
 * 			timeout(in/out):	 		if timeout==0 -> default-timeout is set
 *			len(in): 					has to be >1, but only one broadcast will be received
 *			node_id(out):				node_id of received broadcast
 *			module_id(out): 			module_id of received broadcast
 *			module_instance_id(out):	module_instance_id of received broadcast
 *		data: (broadcast element array)
 *			parameter_id(out): 			parameter_id of received broadcast
 *			dlen(out):					data-len of received broadcast
 *			data(out): 					data of received broadcast
 * return: 0 .. received; <0 .. error
 */
int pnet_ge_can_broadcast_receive(int fd, struct pnet_ge_can_sequence *bc);

/*
 * send sequence
 * parameters:
 *	fd .. descriptor of ge-can-device
 *	seq .. seqence to send
 *		hdr:
 * 			timeout(in/out):	 		if timeout==0 -> default-timeout is set
 *			len(in): 					number of data-elements in sequence (has to be >1)
 *			node_id(in):				node_id to sequence to send
 *			module_id(in): 				module_id to sequence to send
 *			module_instance_id(in):		module_instance_id to sequence to send
 *		data: (sequence element array)
 *			parameter_id(in):	 		parameter_id to sequence to send
 *			dlen(in/out):				data-len to send; data-len received
 *			data(in/out): 				data to send; data received
 * return: 0 .. ok; 1 .. busy -> try again
 */
int pnet_ge_can_sequence_send(int fd, struct pnet_ge_can_sequence *seq);


/*
 * open ge-can device
 * parameters:
 * 	minor .. pnet_ge_can-minor
 * return: 0 .. ok; !0 .. error (errno: .. errorcode)
 */
int pnet_ge_can_open(int minor);

/*
 * close ge-can device
 * parameters:
 *	fd .. descriptor to can-device
 */
void pnet_ge_can_close(int fd);

#endif /* __KERNEL__ */

#endif /* __GE_CAN_H__ */

