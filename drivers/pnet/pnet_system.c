/***********************************************************************
 *
 * @Authors: Manfred Schlägl, Ginzinger electronic systems GmbH
 * @Descr: system-driver over pnet
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
 *		* interruptible waits replaced by timeout waits
 *		* better api-comments
 *		* error-correction: proc-output
 *	2010-KW03 manfred.schlaegl@gmx.at
 *		* Begin
 *		* seperate semaphores for fifo_read, fifo_write and registers, 
 *			instead of semaphores for read and write
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
#include "pnet_rdev.h"
#include "pnet_system.h"

/*
 **************************************************************************************************
 * Begin: Basic Definitions
 **************************************************************************************************
 */

#define DRV_MAJOR  										247
#define DRV_NAME										"pnet_system"
#define PNET_SYSTEM_DEV_N								1

/* enable debug output */
#ifdef CONFIG_PNET_SYSTEM_DEBUG
#define DEBUG
#endif
//#define DEBUG

/*
 **************************************************************************************************
 * End: Basic Definitions
 **************************************************************************************************
 */


/*
 **************************************************************************************************
 * Begin: Driver data
 **************************************************************************************************
 */

#define PNET_SYSTEM_DEBUG_STATE_COUNTERS

/* state of pnet_system receiver */
#define PNET_SYSTEM_RCV_STATE_RCV_HEADER	0
#define PNET_SYSTEM_RCV_STATE_RCV_LOAD		1

/* state of pnet_system sender */
#define PNET_SYSTEM_SND_STATE_SND_HEADER	0
#define PNET_SYSTEM_SND_STATE_SND_LOAD		1

typedef struct pnet_system_private
{
	/* driver data */
	/* associated linux device */
	struct device *device;
	int read_count;				/* read-open counter */
	int write_count;			/* write-open counter */
	int disconnected;			/* if set device got disconnected after open */
	int connection;				/* connection-state */

	/* wait queues */
	wait_queue_head_t open_wait;
	wait_queue_head_t snd_wait;
	wait_queue_head_t rcv_wait;

	/* receiver-state */
	int rcv_state;

	/* sender-state */
	int snd_state;

	/* tasklets */
	struct tasklet_struct worker_tasklet;

	/* data spinlock */
	spinlock_t data_lock;

	/* associated pnet_rdev */
	pnet_rdev_private_t *pnet_rdev;
	struct semaphore reg_sem;		/* semaphore for registers(lin-ioctl) functions */
	struct semaphore fifo_rsem;		/* semaphore for fifo(lin-write) read-functions */
	struct semaphore fifo_wsem;		/* semaphore for fifo(lin-write) write-functions */

#ifdef PNET_SYSTEM_DEBUG_STATE_COUNTERS
#endif
} pnet_system_private_t;

static struct pnet_system_private pnet_system_private[PNET_SYSTEM_DEV_N];

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

/*
 **************************************************************************************************
 * End: Misc
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Debug
 **************************************************************************************************
 */
#ifdef DEBUG
#define dbg(format, arg...) printk(KERN_INFO "%s:%i: %s: " format "\n" , __FILE__, __LINE__, __FUNCTION__, ## arg)
//#define dbg(format, arg...) printk(KERN_DEBUG "%s: " format "\n" , __FILE__ , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif

#define err(format, arg...) printk(KERN_ERR "%s: %s: " format "\n" , __FILE__, __FUNCTION__, ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: %s: " format "\n" , __FILE__, __FUNCTION__, ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: %s: " format "\n", __FILE__, __FUNCTION__, ## arg)

/*
 **************************************************************************************************
 * End: Debug
 **************************************************************************************************
 */



/************************************************************************
 * BEGIN: pnet_rdev interface
 ************************************************************************/

void pnet_system_rdev_data_callback(pnet_rdev_private_t *pnet_rdev, int evcode)
{
	/* get pnet_system */
	struct pnet_system_private *pnet_system=(struct pnet_system_private*)pnet_rdev->priv;

	if(evcode==PNET_RDEV_CALLBACK_EV_CONNECTED) {
		/* pnet_rdev connected */
		dbg("PNET_RDEV_CALLBACK_EV_CONNECTED");
	} else if(evcode==PNET_RDEV_CALLBACK_EV_DISCONNECTED) {
		/* pnet_rdev disconnected */
		dbg("PNET_RDEV_CALLBACK_EV_DISCONNECTED");
	} else if(evcode==PNET_RDEV_CALLBACK_EV_RECEIVED) {
		/* data received */
		dbg("PNET_RDEV_CALLBACK_EV_RECEIVED -> wakeup rcv");
		wake_up(&pnet_system->rcv_wait);
	} else if(evcode==PNET_RDEV_CALLBACK_EV_TRANSMITTED) {
		/* data sent */
		dbg("PNET_RDEV_CALLBACK_EV_TRANSMITTED -> wakeup snd");
		wake_up_interruptible(&pnet_system->snd_wait);
	}
}

void pnet_system_rdev_register_callback(pnet_rdev_private_t *pnet_rdev, int evcode)
{
	/* get pnet_system */
	struct pnet_system_private *pnet_system=(struct pnet_system_private*)pnet_rdev->priv;

	if(evcode==PNET_RDEV_CALLBACK_EV_CONNECTED) {
		dbg("PNET_RDEV_CALLBACK_EV_CONNECTED");

		/* pnet_rdev connected */
		pnet_system->connection=1;

		/* reset data and states */
		pnet_system->rcv_state=PNET_SYSTEM_RCV_STATE_RCV_HEADER;

		/* wakeup all */
		wake_up(&pnet_system->rcv_wait);
		wake_up(&pnet_system->snd_wait);
		wake_up(&pnet_system->open_wait);
	} else if(evcode==PNET_RDEV_CALLBACK_EV_DISCONNECTED) {
		dbg("PNET_RDEV_CALLBACK_EV_DISCONNECTED");

		/* pnet_rdev disconnected */
		pnet_system->connection=0;

		/* set disconnect-flag */
		pnet_system->disconnected=1;

		/* wakeup all */
		wake_up(&pnet_system->rcv_wait);
		wake_up(&pnet_system->snd_wait);
		wake_up(&pnet_system->open_wait);
	} else if(evcode==PNET_RDEV_CALLBACK_EV_RECEIVED) {
		/* registers received (remote commit) */
		dbg("PNET_RDEV_CALLBACK_EV_RECEIVED -> wakeup rcv");
		wake_up(&pnet_system->rcv_wait);
	} else if(evcode==PNET_RDEV_CALLBACK_EV_TRANSMITTED) {
		/* registers sent (local commit done) */
		dbg("PNET_RDEV_CALLBACK_EV_TRANSMITTED -> wakeup snd");
		wake_up(&pnet_system->snd_wait);
	}
}

/*
 **************************************************************************************************
 * End: pnet_rdev interface
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: pnet_system driver
 **************************************************************************************************
 */

/***** Low-Level functions: rx/tx-fifo *****/

/*
 * read a fixed amount of data from pnet_rdev
 * parameters:
 *	pnet_system .. pnet_system instance
 *	buf .. buffer to read in
 *	count .. number of bytes to read
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
static inline int pnet_system_fifo_rcv(struct pnet_system_private *pnet_system, char *buf, int count, int wait)
{
	int ret;

	dbg("buflen: %i",count);

	/* check for received data */
	while(pnet_rdev_read_len(pnet_system->pnet_rdev)<count) {
		if(wait) {
			dbg("sleep");
			if (!wait_event_timeout(pnet_system->rcv_wait,
					(pnet_rdev_read_len(pnet_system->pnet_rdev)>=count) || pnet_system->disconnected,
					wait) 
			) {
				return -ETIME;
			}
			dbg("wakeup");
			if(pnet_system->disconnected)
				return -EPIPE;
		} else {
			return -EAGAIN;
		}
	}

	if(pnet_system->disconnected)
		return -EPIPE;

	/* read data */
	ret=pnet_rdev_read(pnet_system->pnet_rdev,buf,count);
	if(ret<0)
		return ret;
	if(ret!=count) {
		/* error */
		return -EPIPE;
	}

	return 0;
}


/*
 * write a fixed amount of data to pnet_rdev
 * parameters:
 *	pnet_system .. pnet_system instance
 *	buf .. buffer to write from
 *	count .. number of bytes to write
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
static inline int pnet_system_fifo_snd(struct pnet_system_private *pnet_system, char *buf, int count, int wait)
{
	int ret;

	dbg("buflen: %i",count);

	/* check for enough space */
	while(pnet_rdev_write_free(pnet_system->pnet_rdev)<count) {
		if(wait) {
			dbg("sleep");
			if (!wait_event_timeout(pnet_system->snd_wait,
					(pnet_rdev_write_free(pnet_system->pnet_rdev)>=count) || pnet_system->disconnected,
					wait)
			) {
				return -ETIME;
			}
			dbg("wakeup");
			if(pnet_system->disconnected)
				return -EPIPE;
		} else {
			return -EAGAIN;
		}
	}

	/* write data */
	ret=pnet_rdev_write(pnet_system->pnet_rdev,buf,count);
	if(ret<0)
		return ret;
	if(ret!=count) {
		/* error */
		return -EPIPE;
	}

	return 0;
}

/***** Low-Level functions: async-commands (ioctl) *****/

/*
 * do command/ioctl over pnet_rdev
 * parameters:
 *	pnet_system .. pnet_system instance
 *	cmd .. command-code
 *	arg .. argument list
 *  argnr .. number of arguments in list
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); >=0 .. ok(number of return arguments)
 */
static inline int pnet_system_cmd(struct pnet_system_private *pnet_system, int cmd, int arg[], int argc, int wait)
{
	int rcmd,rargc;
	int i,ret;
#ifdef DEBUG
	char str[512];
#endif

#ifdef DEBUG
	sprintf(str, "command sent: cmd: 0x%X ",cmd);
	for(i=0;i<argc;i++)
		sprintf(str, "%s arg%i: 0x%X ",str,i,arg[i]);
	dbg("%s",str);
#endif

	/* set cmd & argc */
	i=cmd<<16|argc;

	dbg("reset");

	/* reset registers */
	ret=pnet_rdev_register_reset(pnet_system->pnet_rdev, wait);
	if(ret<0)
		return ret;

	dbg("write");

	/* set args */
	ret=pnet_rdev_register_write(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_CMD, (unsigned int)i, wait);
	if(ret<0)
		goto __ret_reset;	/* reset registers */
	for(i=0;i<argc;i++) {
		ret=pnet_rdev_register_write(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_ARG(i), (unsigned int)arg[i], wait);
		if(ret<0)
			goto __ret_reset;	/* reset registers */
	}

	dbg("read");

	/* dummy read -> reset register changed flag */
	pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_CMDRET, (unsigned int*)&i);

	dbg("commit");

	/* do command */
	ret=pnet_rdev_register_commit(pnet_system->pnet_rdev, wait);
	if(ret<0)
		goto __ret_reset;	/* reset registers */

	dbg("wait remote commit");

	/* wait for return */
	do {
		dbg("wait (%i)",wait);

		/* wait for remote commit */
		ret=pnet_rdev_register_wait_remote_commit(pnet_system->pnet_rdev, wait);
		if(ret<0)
			return ret;

		dbg("wakeup");

		/* dummy read -> reset register changed flag */
		ret=pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_CMDRET, (unsigned int*)&i);
		if(ret<0)
			return ret;
		if(ret==1) {
			/* return received -> stop loop */
			dbg("ret");
			break;
		}
		dbg("no ret");
	} while(1);

	/* return was received */
	
	/* get cmd & argc */
	rargc=i&0xffff;
	rcmd=(i>>16)&0xffff;

	/* check return */
	if(rcmd!=cmd) {
		err("wong return-cmd for command (sent -> cmd: 0x%X, argc: 0x%X, received -> cmd: 0x%X, argc: 0x%X)",cmd,argc,rcmd,rargc);
		return -EPIPE;
	}

	dbg("read");

	/* copy return arguments */
	for(i=0;i<rargc;i++)
		pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_ARG(i), (unsigned int*)&arg[i]);

#ifdef DEBUG
	sprintf(str, "command return: cmd: 0x%X ",rcmd);
	for(i=0;i<rargc;i++)
		sprintf(str, "%s arg%i: 0x%X ",str,i,arg[i]);
	dbg("%s",str);
#endif

	/* ok */
	return rargc;

__ret_reset:
	/* reset registers */
	ret=pnet_rdev_register_reset(pnet_system->pnet_rdev, wait);
	return ret;
}








/***** High-level functions: async-commands (ioctl) *****/


/*
 * update registers
 *	pnet_system .. pnet_system instance
 *	pnet_system_info .. info-structure
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_update(struct pnet_system_private *pnet_system, struct pnet_system_info *info, int wait)
{
	int ret, data;
	int arg[1];

	/* do command */
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_UPDATE, arg, 0, wait);
	if(ret<0)
		return ret;

	/* return on remote error */
	if(arg[0])
		return arg[0];

	/* evcore-version */
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_EVCORE_VERSION,&data);
	info->evcore.version.hi=  				(data>>24)	&0xff;
	info->evcore.version.med= 				(data>>16)	&0xff;
	info->evcore.version.lo= 				(data>>8)	&0xff;
	info->evcore.version.rc= 				(data>>0)	&0xff;

	dbg("ret: %i",ret);
	if(ret<0)
		return ret;

	/* system-cpu */
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_SYS_CPUINFO1,&data);
	info->cpu.type=								(data>>24)	&0xff;
	info->cpu.powersave_mode=					(data>>16)	&0xff;
	info->cpu.oscclk=							(data>>0)	&0xffff;

	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_SYS_CPUINFO2,&data);
	info->cpu.sysclk=							(data>>16)	&0xffff;
	info->cpu.iclk=								(data)	&0xffff;

	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_SYS_CPUINFO2,&data);	dbg("ret: %i",ret);

	dbg("ret: %i",ret);
	if(ret<0)
		return ret;

	/* system-software */
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_SYS_SOFTWARE1,&data);
	info->evcore.cfg.tasklet_biglock=			(data>>31)	&1;
	info->evcore.cfg.tasklet_absolute=			(data>>30)	&1;
	info->evcore.cfg.exc_irq_nested=			(data>>29)	&1;
	info->evcore.cfg.debug=						(data>>28)	&1;
	info->evcore.cfg.tasklet_debug=				(data>>27)	&1;
	info->evcore.cfg.tasklet_debug_trace=		(data>>26)	&1;
	info->evcore.cfg.tasklet_debug_callstack=	(data>>25)	&1;

	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_SYS_SOFTWARE1,&data);
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_SYS_SOFTWARE1,&data);
	dbg("ret: %i",ret);
	if(ret<0)
		return ret;


	/* system-software-states */
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_IRQ_CALLS, &data);
	info->evcore.state.irq_calls=				data;
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_IRQ_MAX_LEVEL, &data);
	info->evcore.state.irq_max_level=			data;
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_BH_CALLS, &data);
	info->evcore.state.bh_calls=				data;
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_TL_MAX_LEVEL, &data);
	info->evcore.state.tl_max_level=			data;
	dbg("ret: %i",ret);
	if(ret<0)
		return ret;

	/* system-timer */
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_SYS_TIME_HI,&data);
	info->evcore.time.seconds=					((unsigned long long)data<<12);
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_SYS_TIME_LO,&data);
	info->evcore.time.seconds|=					((unsigned long long)data>>20)&((1<<12)-1);
	info->evcore.time.useconds=					data		&((1<<20)-1);
	dbg("ret: %i",ret);
	if(ret<0)
		return ret;

	/* load messurement-struct registers */
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_CONFIG, &data);
	info->evcore.load.enabled=					(data>>16)	&0xffff;
	info->evcore.load.duration=					(data>>0)	&0xffff;
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_COUNT, &data);
	info->evcore.load.count=					data;
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_CUR, &data);
	info->evcore.load.cur=						data;
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_MIN, &data);
	info->evcore.load.min=						data;
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev, PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_MAX, &data);
	info->evcore.load.max=						data;
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_AVG,&data);
	info->evcore.load.avg=						data;
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_STDDEV,&data);
	info->evcore.load.stddev=					data;
	dbg("ret: %i",ret);
	if(ret<0)
		return ret;


	/* application version */
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_APPLICATION_VERSION,&data);
	info->application.version.hi=  				(data>>24)	&0xff;
	info->application.version.med= 				(data>>16)	&0xff;
	info->application.version.lo= 				(data>>8)	&0xff;
	info->application.version.rc= 				(data>>0)	&0xff;
	dbg("ret: %i",ret);
	if(ret<0)
		return ret;

	/* arm9_suspend */
	ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_ARM9_SUSPEND_SUPPORT,&data);
	info->application.arm9_suspend.support=data;
	dbg("ret: %i",ret);
	if(ret<0)
		return ret;

	/* if supported */
	if(info->application.arm9_suspend.support) {
		ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_ARM9_SUSPEND_STATUS,&data);
		info->application.arm9_suspend.status=data;
		ret=pnet_rdev_register_read(pnet_system->pnet_rdev,PNET_SYSTEM_RDEV_REGISTER_ARM9_SUSPEND_MASK,&data);
		info->application.arm9_suspend.mask=data;
	}

	dbg("ret: %i",ret);

	/* error? */
	if(ret<0)
		return ret;

	/* ok */
	return 0;
}
EXPORT_SYMBOL(pnet_system_update);

/*
 * reset the system
 *	pnet_system .. pnet_system instance
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_sys_reset(struct pnet_system_private *pnet_system, int wait)
{
	int ret;
	int arg[1];

	/* do command */
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_SYS_RESET, arg, 0, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_sys_reset);

/*
 * sleep us microseconds with all enabled
 *	pnet_system .. pnet_system instance
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_this(struct pnet_system_private *pnet_system, unsigned int us, int wait)
{
	int ret;
	int arg[1];

	/* do command */
	arg[0]=us;
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_PAUSE_THIS, arg, 1, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_pause_this);

/*
 * sleep us microseconds with tasklets locked
 *	pnet_system .. pnet_system instance
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_tasklet(struct pnet_system_private *pnet_system, unsigned int us, int wait)
{
	int ret;
	int arg[1];

	/* do command */
	arg[0]=us;
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_PAUSE_TASKLET, arg, 1, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_pause_tasklet);

/*
 * sleep us microseconds with irqs and tasklets locked
 *	pnet_system .. pnet_system instance
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_irq(struct pnet_system_private *pnet_system, unsigned int us, int wait)
{
	int ret;
	int arg[1];

	/* do command */
	arg[0]=us;
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_PAUSE_IRQ, arg, 1, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_pause_irq);

/*
 * sleep us microseconds with all(fiq,irq and tasklets) locked
 *	pnet_system .. pnet_system instance
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_all(struct pnet_system_private *pnet_system, unsigned int us, int wait)
{
	int ret;
	int arg[1];

	/* do command */
	arg[0]=us;
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_PAUSE_ALL, arg, 1, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_pause_all);

/*
 * set/reset system-states
 *	pnet_system .. pnet_system instance
 *	evcore_state .. evcore_state-struct
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_sys_state(struct pnet_system_private *pnet_system, struct pnet_system_evcore_state evcore_state, int wait)
{
	int ret;
	int arg[4];

	/* do command */
	arg[0]=evcore_state.irq_calls;
	arg[1]=evcore_state.irq_max_level;
	arg[2]=evcore_state.bh_calls;
	arg[3]=evcore_state.tl_max_level;
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_SET_SYS_STATE, arg, 4, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_set_sys_state);

/*
 * set/reset system-load
 *	pnet_system .. pnet_system instance
 *	evcore_load .. evcore_load-struct
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_sys_load(struct pnet_system_private *pnet_system, struct pnet_system_evcore_load evcore_load, int wait)
{
	int ret;
	int arg[3];

	/* do command */
	arg[0]=evcore_load.count;
	arg[1]=(evcore_load.max<<16) | (evcore_load.min&0xffff);
	arg[2]=(evcore_load.avg<<16) | (evcore_load.stddev&0xffff);
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_SET_EVCORE_LOAD, arg, 3, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_set_sys_load);

/*
 * get the mask of events which caused the last wakeup
 *	pnet_system .. pnet_system instance
 *	status .. pointer to status (wakeup-status (bits represent wakup-events))
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_get_wakeup_status(struct pnet_system_private *pnet_system, int *status, int wait)
{
	int ret;
	int arg[2];

	/* do command */
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_GET_WAKEUP_STATUS, arg, 0, wait);
	if(ret<0)
		return ret;

	(*status)=arg[1];

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_get_wakeup_status);

/*
 * get the mask of events which caused the last wakeup
 *	pnet_system .. pnet_system instance
 *	mask .. pointer to mask (wakeup-status (bits represent wakup-events))
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_get_wakeup_mask(struct pnet_system_private *pnet_system, int *mask, int wait)
{
	int ret;
	int arg[2];

	/* do command */
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_GET_WAKEUP_MASK, arg, 0, wait);
	if(ret<0)
		return ret;

	(*mask)=arg[1];

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_get_wakeup_mask);

/*
 * set the mask of wakeup-events that will cause a wakeup
 *	pnet_system .. pnet_system instance
 *	mask .. mask to set (wakeup-status (bits represent wakup-events))
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_wakeup_mask(struct pnet_system_private *pnet_system, int mask, int wait)
{
	int ret;
	int arg[2];

	/* do command */
	arg[0]=mask;
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_SET_WAKEUP_MASK, arg, 1, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_set_wakeup_mask);

/*
 * enable suspend-mode
 *	pnet_system .. pnet_system instance
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_suspend_enable(struct pnet_system_private *pnet_system, int wait)
{
	int ret;
	int arg[2];

	/* do command */
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_SUSPEND_ENABLE, arg, 0, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_suspend_enable);

/*
 * set timer for timed wakeup event in seconds
 *	pnet_system .. pnet_system instance
 *	sec .. seconds to wait before wakeup
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_wakeup_timer(struct pnet_system_private *pnet_system, unsigned int sec, int wait)
{
	int ret;
	int arg[2];

	/* do command */
	arg[0]=sec;
	ret=pnet_system_cmd(pnet_system, PNET_SYSTEM_CMD_SET_WAKEUP_TIMER, arg, 1, wait);
	if(ret<0)
		return ret;

	/* normal return */
	return arg[0];
}
EXPORT_SYMBOL(pnet_system_set_wakeup_timer);

/***** High-level functions: sync-commands (fifo) *****/


/*
 * read a fixed amount of data
 * parameters:
 *	pnet_system .. pnet_system instance
 *	buf .. buffer to read in
 *	count .. number of bytes to read
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_read(struct pnet_system_private *pnet_system, char *buf, int count, int wait)
{
	return pnet_system_fifo_rcv(pnet_system, buf, count, wait);
}
EXPORT_SYMBOL(pnet_system_read);


/*
 * write a fixed amount of data
 * parameters:
 *	pnet_system .. pnet_system instance
 *	buf .. buffer to write from
 *	count .. number of bytes to write
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_write(struct pnet_system_private *pnet_system, char *buf, int count, int wait)
{
	return pnet_system_fifo_snd(pnet_system, buf, count, wait);
}
EXPORT_SYMBOL(pnet_system_write);

/*
 **************************************************************************************************
 * End: pnet_system driver
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: API-Helper - Functions
 **************************************************************************************************
 */

/*
 * low-level api-system-open
 * parameters:
 *	minor .. system minor (same as chrdev-minor)
 *	pnet_system .. pointer will to pnet_system instance-pointer (will be set on success)
 *	accmod .. like file ACCMODE (O_RDWR, O_RDONLY, O_WRONLY)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system__open(int minor, struct pnet_system_private **pnet_system, int accmode, int wait)
{
	unsigned long iflags;
	int ret=0;

	if (minor>=PNET_SYSTEM_DEV_N)
		return -EINVAL;

	dbg("minor: %i",minor);

	spin_lock_irqsave(&pnet_system_private[minor].data_lock,iflags);

	if (accmode==O_RDWR) {
		if (pnet_system_private[minor].write_count>0 || pnet_system_private[minor].read_count>0) {
			ret=-EBUSY;
			goto busy_error;
		}
		pnet_system_private[minor].write_count++;
		pnet_system_private[minor].read_count++;
	}
	else if (accmode==O_WRONLY) {
		if (pnet_system_private[minor].write_count>0) {
			ret=-EBUSY;
			goto busy_error;
		}
		pnet_system_private[minor].write_count++;
	}
	else if (accmode==O_RDONLY) {
		if (pnet_system_private[minor].read_count>0) {
			ret=-EBUSY;
			goto busy_error;
		}
		pnet_system_private[minor].read_count++;
	}

	spin_unlock_irqrestore(&pnet_system_private[minor].data_lock,iflags);

	/* TODO: open with O_NONBLOCK */
	if(!wait)
		return -EPERM;

	/* reset disconnected before wait */
	pnet_system_private[minor].disconnected=0;
	/* wait for connected */
	if (pnet_system_private[minor].connection) {
		if (!wait) {
			ret=-EPIPE;
			goto open_error;
		}

		if (wait_event_interruptible(pnet_system_private[minor].open_wait,(pnet_system_private[minor].connection))) {
			ret=-ERESTARTSYS;
			goto open_error;
		}
	}

	/* open pnet_rdev for system */
	ret=pnet_rdev_open(pnet_system_private[minor].pnet_rdev);
	if(ret)
		goto open_error;

	/* ok */
	(*pnet_system)=&pnet_system_private[minor];
	ret=0;
	goto ret;

	pnet_rdev_close(pnet_system_private[minor].pnet_rdev);
open_error:
	spin_lock_irqsave(&pnet_system_private[minor].data_lock,iflags);
	if (accmode==O_RDWR || accmode==O_WRONLY) 
		if (pnet_system_private[minor].write_count>0) pnet_system_private[minor].write_count--;
	if (accmode==O_RDWR || accmode==O_RDONLY) 
		if (pnet_system_private[minor].read_count>0) pnet_system_private[minor].read_count--;
busy_error:
	spin_unlock_irqrestore(&pnet_system_private[minor].data_lock,iflags);
ret:
	return ret;
}

/*
 * low-level api-system-close
 * parameters:
 *	minor .. system minor (same as chrdev-minor)
 *	accmod .. like file ACCMODE (O_RDWR, O_RDONLY, O_WRONLY)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system__close(int minor, int accmode, int wait)
{
	unsigned long iflags;

	if (minor>=PNET_SYSTEM_DEV_N)
		return -EINVAL;

	dbg("minor: %i",minor);

	spin_lock_irqsave(&pnet_system_private[minor].data_lock,iflags);

	if (accmode==O_RDWR || accmode==O_WRONLY) 
		if (pnet_system_private[minor].write_count>0) pnet_system_private[minor].write_count--;
	if (accmode==O_RDWR || accmode==O_RDONLY) 
		if (pnet_system_private[minor].read_count>0) pnet_system_private[minor].read_count--;

	spin_unlock_irqrestore(&pnet_system_private[minor].data_lock,iflags);

	/* close pnet_rdev */
	pnet_rdev_close(pnet_system_private[minor].pnet_rdev);

	/* ok */
	return 0;
}
/*
 **************************************************************************************************
 * End: API-Helper - Functions
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Kernel API driver only - Functions
 **************************************************************************************************
 */

/*
 * open system-interface
 * parameters:
 * 	minor .. pnet_system-minor
 * 	pnet_system .. pnet_system instance (set by open)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_open(int minor, struct pnet_system_private **pnet_system, int wait)
{
	dbg("");

	/* raw open */
	return pnet_system__open(minor, pnet_system, O_RDWR, wait);
}
EXPORT_SYMBOL(pnet_system_open);

/*
 * close system-interface
 * parameters:
 * 	minor .. pnet_system-minor
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_close(int minor, int wait)
{
	dbg("");

	/* raw close */
	return pnet_system__close(minor, O_RDWR, wait);
}
EXPORT_SYMBOL(pnet_system_close);

/*
 **************************************************************************************************
 * End: Kernel API driver only - Functions
 **************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: Linux-Driver-Functions
 **************************************************************************************************
 */
static int pnet_system_lin_open(struct inode *inode, struct file *file)
{
	int minor,wait;
	int ret=0;
	struct pnet_system_private *pnet_system=NULL;

	minor=iminor(inode);
	if (minor>=PNET_SYSTEM_DEV_N)
		return -EINVAL;

	dbg("minor: %i",minor);

	if (down_interruptible(&pnet_system_private[minor].reg_sem)) {
		ret=-ERESTARTSYS;
		goto __ret;
	}
	if (down_interruptible(&pnet_system_private[minor].fifo_rsem)) {
		ret=-ERESTARTSYS;
		goto __ret_up_reg_sem;
	}
	if (down_interruptible(&pnet_system_private[minor].fifo_wsem)) {
		ret=-ERESTARTSYS;
		goto __ret_up_fifo_rsem;
	}

	/* get wait-status */
	wait=(file->f_flags & O_NONBLOCK) ? 0 : PNET_SYSTEM_LIN_DEFAULT_TIMEOUT;

	/* open */
	ret=pnet_system__open(
		minor, 
		&pnet_system, 
		(file->f_flags&O_ACCMODE), 
		wait
	);

	up(&pnet_system_private[minor].fifo_wsem);
__ret_up_fifo_rsem:
	up(&pnet_system_private[minor].fifo_rsem);
__ret_up_reg_sem:
	up(&pnet_system_private[minor].reg_sem);
__ret:
	return ret;
}

static int pnet_system_lin_close(struct inode *inode, struct file *file)
{
	int minor,wait;
	int ret=0;

	minor=iminor(inode);
	if (minor>=PNET_SYSTEM_DEV_N)
		return -EINVAL;

	dbg("minor: %i",minor);

	if (down_interruptible(&pnet_system_private[minor].reg_sem)) {
		ret=-ERESTARTSYS;
		goto __ret;
	}
	if (down_interruptible(&pnet_system_private[minor].fifo_rsem)) {
		ret=-ERESTARTSYS;
		goto __ret_up_reg_sem;
	}
	if (down_interruptible(&pnet_system_private[minor].fifo_wsem)) {
		ret=-ERESTARTSYS;
		goto __ret_up_fifo_rsem;
	}

	/* get wait-status */
	wait=(file->f_flags & O_NONBLOCK) ? 0 : PNET_SYSTEM_LIN_DEFAULT_TIMEOUT;

	/* close */
	ret=pnet_system__close(
		minor, 
		(file->f_flags&O_ACCMODE), 
		wait
	);

	up(&pnet_system_private[minor].fifo_wsem);
__ret_up_fifo_rsem:
	up(&pnet_system_private[minor].fifo_rsem);
__ret_up_reg_sem:
	up(&pnet_system_private[minor].reg_sem);
__ret:
	return ret;
}

static int pnet_system_lin_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct pnet_system_private *pnet_system;
	int minor,wait,carg;
	int ret=0;
	struct pnet_system_info info;

	/* get pnet_rdev struct */
	minor = iminor(file->f_dentry->d_inode);
	if (minor>=PNET_SYSTEM_DEV_N)
		return -EINVAL;

	pnet_system =&pnet_system_private[minor];
	dbg("%i, %i",minor,cmd);

	/* get wait-status */
	wait=(file->f_flags & O_NONBLOCK) ? 0 : PNET_SYSTEM_LIN_DEFAULT_TIMEOUT;

	/* error EPIPE when disconnected */
	if (pnet_system->disconnected)
		return -EPIPE;

	/* parse command */
	switch(cmd)
	{
		case PNET_SYSTEM_CMD_UPDATE:
			/* check read-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_RDONLY)
				return -EINVAL;

			dbg("PNET_SYSTEM_CMD_UPDATE: do");

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* get peer-info */
			ret=pnet_system_update(pnet_system,&info,wait);

			/* up */
			up(&pnet_system->reg_sem);

			if(ret)
				return ret;

			/* put structure */
			ret=copy_to_user (argp, &info, sizeof(struct pnet_system_info));
			if(ret)
				return ret;
		break;

		case PNET_SYSTEM_CMD_SYS_RESET:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			dbg("PNET_SYSTEM_CMD_SYS_RESET: do");

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* get peer-info */
			ret=pnet_system_sys_reset(pnet_system,wait);

			/* up */
			up(&pnet_system->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_SYSTEM_CMD_PAUSE_THIS:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameter */
			if (get_user(carg, (int __user *)arg))
				return -EFAULT;

			dbg("PNET_SYSTEM_CMD_PAUSE_THIS: sleep: %ius",carg);

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* do command */
			ret=pnet_system_pause_this(pnet_system, carg, wait);

			/* up */
			up(&pnet_system->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_SYSTEM_CMD_PAUSE_TASKLET:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameter */
			if (get_user(carg, (int __user *)arg))
				return -EFAULT;

			dbg("PNET_SYSTEM_CMD_PAUSE_TASKLET: sleep: %ius",carg);

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* do command */
			ret=pnet_system_pause_tasklet(pnet_system, carg, wait);

			/* up */
			up(&pnet_system->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_SYSTEM_CMD_PAUSE_IRQ:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameter */
			if (get_user(carg, (int __user *)arg))
				return -EFAULT;

			dbg("PNET_SYSTEM_CMD_PAUSE_IRQ: sleep: %ius",carg);

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* do command */
			ret=pnet_system_pause_irq(pnet_system, carg, wait);

			/* up */
			up(&pnet_system->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_SYSTEM_CMD_PAUSE_ALL:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameter */
			if (get_user(carg, (int __user *)arg))
				return -EFAULT;

			dbg("PNET_SYSTEM_CMD_PAUSE_ALL: sleep: %ius",carg);

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* do command */
			ret=pnet_system_pause_all(pnet_system, carg, wait);

			/* up */
			up(&pnet_system->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_SYSTEM_CMD_SET_SYS_STATE:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameters */
			ret=copy_from_user (&info.evcore.state, argp, sizeof(struct pnet_system_evcore_state));
			if(ret)
				return ret;

			dbg("PNET_SYSTEM_CMD_SET_SYS_STATE: do");

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* do command */
			ret=pnet_system_set_sys_state(pnet_system, info.evcore.state, wait);

			/* up */
			up(&pnet_system->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_SYSTEM_CMD_SET_EVCORE_LOAD:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameters */
			ret=copy_from_user (&info.evcore.load, argp, sizeof(struct pnet_system_evcore_load));
			if(ret)
				return ret;

			dbg("PNET_SYSTEM_CMD_SET_SYS_STATE: do");

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* do command */
			ret=pnet_system_set_sys_load(pnet_system, info.evcore.load, wait);

			/* up */
			up(&pnet_system->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_SYSTEM_CMD_GET_WAKEUP_STATUS:
			/* check read-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR)
				return -EINVAL;

			dbg("PNET_SYSTEM_CMD_GET_WAKEUP_STATUS: do");

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* do command */
			ret=pnet_system_get_wakeup_status(pnet_system, &carg, wait);

			/* up */
			up(&pnet_system->reg_sem);

			dbg("PNET_SYSTEM_CMD_GET_WAKEUP_STATUS done (ret: %i, status: 0x%X)",ret, carg);
			if(ret)
				return ret;

			/* put ret */
			if (put_user(carg, (int __user *)arg))
				return -EFAULT;
		break;

		case PNET_SYSTEM_CMD_GET_WAKEUP_MASK:
			/* check read-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR)
				return -EINVAL;

			dbg("PNET_SYSTEM_CMD_GET_WAKEUP_MASK: do");

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* do command */
			ret=pnet_system_get_wakeup_mask(pnet_system, &carg, wait);

			/* up */
			up(&pnet_system->reg_sem);

			dbg("PNET_SYSTEM_CMD_GET_WAKEUP_MASK: done (ret: %i, mask: 0x%X)",ret, carg);
			if(ret)
				return ret;

			/* put ret */
			if (put_user(carg, (int __user *)arg))
				return -EFAULT;
		break;

		case PNET_SYSTEM_CMD_SET_WAKEUP_MASK:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameter */
			if (get_user(carg, (int __user *)arg))
				return -EFAULT;

			dbg("PNET_SYSTEM_CMD_SET_WAKEUP_MASK: mask: 0x%X",carg);

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* do command */
			ret=pnet_system_set_wakeup_mask(pnet_system, carg, wait);

			/* up */
			up(&pnet_system->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_SYSTEM_CMD_SUSPEND_ENABLE:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			dbg("PNET_SYSTEM_CMD_SUSPEND_ENABLE");

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* do command */
			ret=pnet_system_suspend_enable(pnet_system, wait);

			/* up */
			up(&pnet_system->reg_sem);

			if(ret)
				return ret;
		break;

		case PNET_SYSTEM_CMD_SET_WAKEUP_TIMER:
			/* check write-permission */
			if ((file->f_flags&O_ACCMODE)!=O_RDWR && (file->f_flags&O_ACCMODE)!=O_WRONLY)
				return -EINVAL;

			/* get parameter */
			if (get_user(carg, (int __user *)arg))
				return -EFAULT;

			dbg("PNET_SYSTEM_CMD_SET_WAKEUP_TIMER: time: %is",carg);

			/* down register semaphore */
			if (down_interruptible(&pnet_system->reg_sem))
				return -ERESTARTSYS;

			/* do command */
			ret=pnet_system_set_wakeup_timer(pnet_system, carg, wait);

			/* up */
			up(&pnet_system->reg_sem);

			if(ret)
				return ret;
		break;

		default:
			dbg("invalid command");
			return -ENOTTY;
	}

	dbg("done: ret: %i",ret);

	return ret;
}

static struct file_operations pnet_system_fops = {
	.owner = THIS_MODULE,
	.ioctl = pnet_system_lin_ioctl,
	.open = pnet_system_lin_open,
	.release = pnet_system_lin_close,
};

static struct class *pnet_system_class;

/*
 **************************************************************************************************
 * End: Linux-Driver-Functions
 **************************************************************************************************
 */



/***********************************************************************
 *  Begin: procfs statistics
 ***********************************************************************/
#ifdef CONFIG_PROC_FS

static int pnet_system_info_show(struct seq_file *m, void *v)
{
	int i;
	struct pnet_system_info info;

	seq_printf(m,"%s\n",PNET_SYSTEM_VERSION_STR);

	for (i=0;i<PNET_SYSTEM_DEV_N;i++) {
		seq_printf(m,"%s%i:\n",DRV_NAME,i);

		down(&pnet_system_private[i].reg_sem);

		/* reset disconnected if first open */
		if( pnet_system_private[i].connection && ( !(pnet_system_private[i].read_count|pnet_system_private[i].write_count) ) ) {
			/* open pnet_rdev for pnet_system */
			if(pnet_rdev_open(pnet_system_private[i].pnet_rdev)) {
				up(&pnet_system_private[i].reg_sem);
				continue;
			}
			pnet_system_private[i].disconnected=0;
		}


		seq_printf(m," - Open:\n");
		seq_printf(m,"   - read:                       %i\n",pnet_system_private[i].read_count);
		seq_printf(m,"   - write:                      %i\n",pnet_system_private[i].write_count);
		seq_printf(m," - States:\n");
		seq_printf(m,"   - disconnected:               %i\n",pnet_system_private[i].disconnected);
		seq_printf(m,"   - connection:                 %i\n",pnet_system_private[i].connection);

		/* stop output for this port here, if not connected */
		if(!pnet_system_private[i].connection) {
			pnet_rdev_close(pnet_system_private[i].pnet_rdev);
			up(&pnet_system_private[i].reg_sem);
			continue;
		}

		/* update data */
		if(pnet_system_update(&pnet_system_private[i],&info,PNET_SYSTEM_LIN_DEFAULT_TIMEOUT)) {
			err("error on update");
			pnet_rdev_close(pnet_system_private[i].pnet_rdev);
			up(&pnet_system_private[i].reg_sem);
			return -1;
		}

		seq_printf(m," - System:\n");
		seq_printf(m,"   - cpu:\n");
		seq_printf(m,"     - type:                     %i\n",info.cpu.type);
		seq_printf(m,"     - type:                     ");
		switch(info.cpu.type) {
			case 1:	seq_printf(m,"TMS470R1A64"); break;
			case 2:	seq_printf(m,"TMS470R1A128"); break;
			case 3:	seq_printf(m,"TMS470R1A256"); break;
			case 4:	seq_printf(m,"TMS470R1A288"); break;
			case 5:	seq_printf(m,"TMS470R1A384"); break;
			case 6:	seq_printf(m,"TMS470R1B512"); break;
			case 7:	seq_printf(m,"TMS470R1B768"); break;
			case 8:	seq_printf(m,"TMS470R1B1MB"); break;
			case 9:	seq_printf(m,"LPC2148"); break;
			default:seq_printf(m,"unknown"); break;
		}
		seq_printf(m," (%i)\n",info.cpu.type);
		seq_printf(m,"     - powersave_mode:           %i\n",info.cpu.powersave_mode);
		seq_printf(m,"     - oscillator clock:         %ikHz\n",info.cpu.oscclk);
		seq_printf(m,"     - system clock:             %ikHz\n",info.cpu.sysclk);
		seq_printf(m,"     - interface clock:          %ikHz\n",info.cpu.iclk);
		seq_printf(m,"   - evcore:\n");
		if(info.evcore.version.rc) {
			seq_printf(m,"     - version:                  evcore-%i.%i.%i-rc%i (unstable)\n",	\
					info.evcore.version.hi,														\
					info.evcore.version.med,													\
					info.evcore.version.lo,														\
					info.evcore.version.rc
			);
		} else {
			seq_printf(m,"     - version:                  evcore-%i.%i.%i (stable)\n",			\
					info.evcore.version.hi,														\
					info.evcore.version.med,													\
					info.evcore.version.lo														\
			);
		}
		seq_printf(m,"     - config:\n");
		seq_printf(m,"       - tasklet_biglock:        %c\n",info.evcore.cfg.tasklet_biglock ? 'y' : 'n');
		seq_printf(m,"       - tasklet_absolute:       %c\n",info.evcore.cfg.tasklet_biglock ? 'y' : 'n');
		seq_printf(m,"       - exc_irq_nested:         %c\n",info.evcore.cfg.exc_irq_nested ? 'y' : 'n');
		seq_printf(m,"       - debug:                  %c\n",info.evcore.cfg.debug ? 'y' : 'n');
		seq_printf(m,"       - tasklet_debug:          %c\n",info.evcore.cfg.tasklet_debug ? 'y' : 'n');
		seq_printf(m,"       - tasklet_debug_trace:    %c\n",info.evcore.cfg.tasklet_debug_trace ? 'y' : 'n');
		seq_printf(m,"       - tasklet_debug_callstack:%c\n",info.evcore.cfg.tasklet_debug_callstack ? 'y' : 'n');
		seq_printf(m,"     - state:\n");
		seq_printf(m,"       - irq_calls:              %i\n",info.evcore.state.irq_calls);
		seq_printf(m,"       - irq_max_level:          %i\n",info.evcore.state.irq_max_level);
		seq_printf(m,"       - bh_calls:               %i\n",info.evcore.state.bh_calls);
		seq_printf(m,"       - tl_max_level:           %i\n",info.evcore.state.tl_max_level);
		seq_printf(m,"     - time:                     %llu.%.6is\n",info.evcore.time.seconds,info.evcore.time.useconds);
		seq_printf(m,"     - load:\n");
		seq_printf(m,"       - enabled:                %c\n",info.evcore.load.enabled ? 'y' : 'n');
		seq_printf(m,"       - messurement-duration:   %ims\n",info.evcore.load.duration);
		seq_printf(m,"       - count:                  %i\n",info.evcore.load.count);
		seq_printf(m,"       - current:                %i.%.1i\n",info.evcore.load.cur/10,info.evcore.load.cur-(info.evcore.load.cur/10)*10);
		seq_printf(m,"       - maximum:                %i.%.1i\n",info.evcore.load.max/10,info.evcore.load.max-(info.evcore.load.max/10)*10);
		seq_printf(m,"       - minimum:                %i.%.1i\n",info.evcore.load.min/10,info.evcore.load.min-(info.evcore.load.min/10)*10);
		seq_printf(m,"       - average:                %i.%.1i\n",info.evcore.load.avg/10,info.evcore.load.avg-(info.evcore.load.avg/10)*10);
		seq_printf(m,"       - standard deviation:     %i.%.1i\n",info.evcore.load.stddev/10,info.evcore.load.stddev-(info.evcore.load.stddev/10)*10);
		seq_printf(m,"   - application:\n");
		if(info.application.version.rc) {
			seq_printf(m,"     - version:                  pnet_rdev-%i.%i.%i-rc%i (unstable)\n",	\
					info.application.version.hi,												\
					info.application.version.med,												\
					info.application.version.lo,												\
					info.application.version.rc
			);
		} else {
			seq_printf(m,"     - version:                  pnet_rdev-%i.%i.%i (stable)\n",		\
					info.application.version.hi,												\
					info.application.version.med,												\
					info.application.version.lo													\
			);
		}

		if(info.application.arm9_suspend.support) {
			seq_printf(m,"   - arm9_suspend:\n");
			seq_printf(m,"     - wakeup-status:            0x%X\n",info.application.arm9_suspend.status);
			seq_printf(m,"       - RESET:                  %i\n",info.application.arm9_suspend.status&(1<<PNET_SYSTEM_WAKEUP_EVENT_RESET) ? 1 : 0);
			seq_printf(m,"       - TIMER:                  %i\n",info.application.arm9_suspend.status&(1<<PNET_SYSTEM_WAKEUP_EVENT_TIMER) ? 1 : 0);
			seq_printf(m,"       - TOUCH:                  %i\n",info.application.arm9_suspend.status&(1<<PNET_SYSTEM_WAKEUP_EVENT_TOUCH) ? 1 : 0);
			seq_printf(m,"     - wakeup-mask:              0x%X\n",info.application.arm9_suspend.mask);
			seq_printf(m,"       - RESET:                  %i\n",info.application.arm9_suspend.mask&(1<<PNET_SYSTEM_WAKEUP_EVENT_RESET) ? 1 : 0);
			seq_printf(m,"       - TIMER:                  %i\n",info.application.arm9_suspend.mask&(1<<PNET_SYSTEM_WAKEUP_EVENT_TIMER) ? 1 : 0);
			seq_printf(m,"       - TOUCH:                  %i\n",info.application.arm9_suspend.mask&(1<<PNET_SYSTEM_WAKEUP_EVENT_TOUCH) ? 1 : 0);
		} else {
			seq_printf(m,"   - arm9_suspend:               no support\n");
		}

		pnet_rdev_close(pnet_system_private[i].pnet_rdev);
		up(&pnet_system_private[i].reg_sem);
	}

	return 0;
}

static int pnet_system_info_open( struct inode *inode, struct file *file)
{
	return single_open(file, pnet_system_info_show, NULL);
}

static struct file_operations pnet_system_info_fops = {
	.owner = THIS_MODULE,
	.open = pnet_system_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static struct proc_dir_entry *pnet_system_proc_dir,*pnet_system_proc_info;
static int pnet_system_proc_create(void)
{
	pnet_system_proc_dir=proc_mkdir("driver/"DRV_NAME,NULL);
	pnet_system_proc_info=create_proc_entry("info",S_IRUGO,pnet_system_proc_dir);
	if (pnet_system_proc_info) {
		pnet_system_proc_info->proc_fops = &pnet_system_info_fops;
	}
	return 0;
}
static void pnet_system_proc_remove(void)
{
	if (pnet_system_proc_info) remove_proc_entry("info", pnet_system_proc_dir);
	if (pnet_system_proc_dir) remove_proc_entry("driver/"DRV_NAME,NULL);
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

static int __init pnet_system_init(void)
{
	int i;
	int ret=0;

	dbg("");
	
	memset(&pnet_system_private, 0, sizeof(struct pnet_system_private)*PNET_SYSTEM_DEV_N);
	logs_init();

	/* init */
	for (i=0;i<PNET_SYSTEM_DEV_N;i++) {
		/* init */
		init_MUTEX(&pnet_system_private[i].reg_sem);
		init_MUTEX(&pnet_system_private[i].fifo_rsem);
		init_MUTEX(&pnet_system_private[i].fifo_wsem);
		init_waitqueue_head(&pnet_system_private[i].open_wait); 
		init_waitqueue_head(&pnet_system_private[i].snd_wait);
		init_waitqueue_head(&pnet_system_private[i].rcv_wait); 
		spin_lock_init(&pnet_system_private[i].data_lock);

		/* request pnet_rdev */
		if(pnet_rdev_request(PNET_SYSTEM_RDEV_MINOR, &pnet_system_private[i].pnet_rdev, &pnet_system_private[i], &pnet_system_rdev_register_callback, &pnet_system_rdev_data_callback))
			goto pnet_system_init_err_pnet_rdev;
	}


	/* register chrdev */
	ret=register_chrdev(DRV_MAJOR,DRV_NAME,&pnet_system_fops);
	if (ret) {
		err("register chrdev failed");
		goto pnet_system_init_err_cdev;
	}

	/* register device class */
	pnet_system_class=class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(pnet_system_class)) {
		err("class_create failed");
		ret = PTR_ERR(pnet_system_class);
		goto pnet_system_init_err_class;
	}

	/* register devices */
	for (i=0;i<PNET_SYSTEM_DEV_N;i++) {
		pnet_system_private[i].device=device_create(pnet_system_class,NULL,MKDEV(DRV_MAJOR,i),NULL,"%s%d",DRV_NAME,i);
		if (IS_ERR(pnet_system_private[i].device)) {
			err("device_create failed");
			ret=PTR_ERR(pnet_system_private[i].device);
			/* unregister devices */
			i--;
			while(i--)
				device_destroy(pnet_system_class,MKDEV(DRV_MAJOR,i));
			goto pnet_system_init_err_class_device;
		}	
	}		

#ifdef CONFIG_PROC_FS
	pnet_system_proc_create();
#endif

	return ret;

pnet_system_init_err_class_device:
	class_destroy(pnet_system_class);
pnet_system_init_err_class:
	unregister_chrdev(DRV_MAJOR,DRV_NAME);
pnet_system_init_err_cdev:
pnet_system_init_err_pnet_rdev:
	return ret;
}

static void __exit pnet_system_exit(void)
{
	int i;

	dbg("");

#ifdef CONFIG_PROC_FS
	pnet_system_proc_remove();
#endif
	for (i=0;i<PNET_SYSTEM_DEV_N;i++)
		device_destroy(pnet_system_class,MKDEV(DRV_MAJOR,i));
	class_destroy(pnet_system_class);
	unregister_chrdev(DRV_MAJOR,DRV_NAME);
}

module_init( pnet_system_init );
module_exit( pnet_system_exit );

MODULE_AUTHOR("Manfred Schlaegl");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PNET system driver over pnet_rdev");

/*
 **************************************************************************************************
 * End: Module-Stuff
 **************************************************************************************************
 */
