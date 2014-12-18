#ifndef __LIBPNET_SYSTEM_H__
#define __LIBPNET_SYSTEM_H__

#ifdef __KERNEL__
#define PNET_SYSTEM_VERSION_STR					"pnet-system-1.1"
#else
#define PNET_SYSTEM_VERSION_STR					"libpnet-system-1.1"
#endif

/* pnet rdev minor-nr */
#define PNET_SYSTEM_RDEV_MINOR					0

/* default timeout for operations on linux-driver-interface (userspace) */
#define PNET_SYSTEM_LIN_DEFAULT_TIMEOUT			PNET_RDEV_LIN_DEFAULT_TIMEOUT

/*
 *********************************************************************************************************
 * BEGIN: Register - Definition
 *********************************************************************************************************
 */

/*
 * system-command
 * 	command is given by the command register
 *	arguments and return-values are given by the argument registers
 *
 * PNET_SYSTEM_RDEV_REGISTER_SYS_CMD						(write-only)
 *	u16 .. cmd				- command-code
 *	u16 .. argc				- number of arguments
 * PNET_SYSTEM_RDEV_REGISTER_SYS_ARG(x)						(read-only)
 *	u32 .. arg[x]	(5 arguments: x=0-4)
 */
#define PNET_SYSTEM_RDEV_REGISTER_CMD				0			/* 0 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_CMDRET			1			/* 1 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_ARG(__arg__)			(2+__arg__)		/* 2 - 7 -> 5		*/
#define PNET_SYSTEM_RDEV_REGISTER_ARG_MAX			5

/*
 *	commands
 *
 *	cmd=0xbe00	..	void pnet_system_update(void)
 *		update registers
 *	cmd=0xbe01	..	void sys_reset(void)
 *		reset the system
 *	cmd=0xbe10	..	void pause_this(u32 us)
 *		sleep us microseconds with all enabled
 *	cmd=0xbe11	..	void pause_tasklet(u32 us)
 *		sleep us microseconds with tasklets locked
 *	cmd=0xbe12	..	void pause_irq(u32 us)
 *		sleep us microseconds with irqs and tasklets locked
 *	cmd=0xbe13	..	void pause_all(u32 us)
 *		sleep us microseconds with all(fiq,irq and tasklets) locked
 *	cmd=0xbe20	..	void set_sys_state(u32 r_irq_calls, u32 r_max_level, u32 r_bh_calls, u32 r_tl_max_level)
 *		reset-system-states (0 .. no reset; 1 .. reset)
 *			r_irq_calls 	.. reset irq-call counter
 *			r_max_level 	.. reset irq max reentrance level
 *			r_bh_calls	.. reset bottom-half counter
 *			r_tl_max_level 	.. reset tl max reentrance level
 *	cmd=0xbe21	..	void set_evcore_load(u32 r_load_cnt, u16 load_min, u16 load_max, u16 r_load_avg, u16 r_load_stddev)
 *		set-evcore-load
 *			r_load_count 	.. reset load-exec counter
 *			load_min	.. set minimum-value (0xffff .. ignore)
 *			load_max	.. set maximum-value (0xffff .. ignore)
 *			r_load_avg  	.. reset load-average
 *			r_load_stddev 	.. reset load-standard-deviation
 *	cmd=0xbe50	..	unsinged int get_wakeup_status(void)
 *		get the mask of events which caused the last wakeup
 *			return .. wakeup-status (bits represent wakup-events)
 *	cmd=0xbe51	..	unsinged int get_wakeup_mask(void)
 *		get the mask of wakeup-events that will cause a wakeup
 *			return .. wakeup-status (bits represent wakup-events)
 *	cmd=0xbe52	..	void set_wakeup_status(unsigned int mask)
 *		set the mask of wakeup-events that will cause a wakeup
 *			mask .. wakeup-status (bits represent wakup-events)
 *	cmd=0xbe53	..	unsinged int suspend_enable(void)
 *		enable suspend-mode
 *			return .. 0 .. ok; <0 .. error
 *	cmd=0xbe54	..	void set_wakeup_timer(unsigned int seconds)
 *		set timer for timed wakeup event in seconds
 *			seconds .. seconds to wait before wakeup
 */
#define PNET_SYSTEM_CMD_UPDATE			0xbe00
#define PNET_SYSTEM_CMD_SYS_RESET		0xbe01
#define PNET_SYSTEM_CMD_PAUSE_THIS		0xbe10
#define PNET_SYSTEM_CMD_PAUSE_TASKLET		0xbe11
#define PNET_SYSTEM_CMD_PAUSE_IRQ		0xbe12
#define PNET_SYSTEM_CMD_PAUSE_ALL		0xbe13
#define PNET_SYSTEM_CMD_SET_SYS_STATE		0xbe20
#define PNET_SYSTEM_CMD_SET_EVCORE_LOAD		0xbe21

#define PNET_SYSTEM_CMD_GET_WAKEUP_STATUS	0xbe50
#define PNET_SYSTEM_CMD_GET_WAKEUP_MASK		0xbe51
#define PNET_SYSTEM_CMD_SET_WAKEUP_MASK		0xbe52
#define PNET_SYSTEM_CMD_SUSPEND_ENABLE		0xbe53
#define PNET_SYSTEM_CMD_SET_WAKEUP_TIMER	0xbe54


#define PNET_SYSTEM_WAKEUP_EVENT_RESET		0
#define PNET_SYSTEM_WAKEUP_EVENT_TIMER		1
#define PNET_SYSTEM_WAKEUP_EVENT_TOUCH		2

/*
 * system version
 *
 * PNET_SYSTEM_RDEV_REGISTER_EVCORE_VERSION (read-only)
 *	u8 .. version_hi
 *	u8 .. version_med
 *	u8 .. version_lo
 * 	u8 .. release-candidate-version (0 - stable)
 */
#define PNET_SYSTEM_RDEV_REGISTER_EVCORE_VERSION		11		/* 11 -> 1 		*/


/*
 * system-cpu
 *
 * PNET_SYSTEM_RDEV_REGISTER_SYS_CPUINFO1	 				(read-only)
 *	u8  .. Processor-Code (TMS470R1A64, ...)
 *	u8  .. Powersave-Mode
 * 	u16 .. OSC_HZ in kHz
 * PNET_SYSTEM_RDEV_REGISTER_SYS_CPUINFO2					(read-only)
 * 	u16 .. SYSCLK_HZ in kHz
 * 	u16 .. ICLK_HZ in kHz
 * PNET_SYSTEM_RDEV_REGISTER_SYS_CPUINFO3					(read-only)
 *	u32 .. reserved
 */
#define PNET_SYSTEM_RDEV_REGISTER_SYS_CPUINFO1				12		/* 12 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_SYS_CPUINFO2				13		/* 13 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_SYS_CPUINFO3				14		/* 14 -> 1 		*/

/*
 * system-software
 *
 * PNET_SYSTEM_RDEV_REGISTER_SYS_SOFTWARE1	 				(read-only)
 *	u1  .. defined(TASKLET_BIGLOCK)		- tasklet-biglock enabled
 *	u1  .. defined(TASKLET_ABSOLUTE)	- absolute tasklet timing
 *	u1  .. defined(EXC_IRQ_NESTED)		- nested irqs enabled
 *	u1  .. defined(DEBUG)			- debug enabled
 *	u1  .. defined(TASKLET_DEBUG)		- tasklet-debug enabled
 *	u1  .. defined(TASKLET_DEBUG_TRACE)	- tasklet-debug-trace enabled
 *	u1  .. defined(TASKLET_DEBUG_CALLSTACK)	- tasklet-debug-callstack eanabled
 *	u25 .. reserved
 * PNET_SYSTEM_RDEV_REGISTER_SYS_SOFTWARE2					(read-only)
 *	u32 .. reserved
 * PNET_SYSTEM_RDEV_REGISTER_SYS_SOFTWARE3					(read-only)
 *	u32 .. reserved
 */
#define PNET_SYSTEM_RDEV_REGISTER_SYS_SOFTWARE1				15		/* 15 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_SYS_SOFTWARE2				16		/* 16 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_SYS_SOFTWARE3				17		/* 17 -> 1 		*/

/*
 * system-software-states
 *
 * PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_IRQ_CALLS				(read-only)
 *	u32 .. exception_stat_irq		- number of irq exceptions occured
 *
 * PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_IRQ_MAX_LEVEL				(read-only)
 *	u32  .. irq_max_level			- maximum nested irq-level
 *
 * PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_BH_CALLS					(read-only)
 *	u32 .. exception_stat_bottom_half	- number of bottom_halfs occured
 *							if EXC_IRQ_NESTED is defined
 *
 * PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_TL_MAX_LEVEL				(read-only)
 *	u32  .. tl_debug_callstack_pointer_max	- maximum nested tasklet-level
 *							if TASKLET_DEBUG_TRACE is defined
 */
#define PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_IRQ_CALLS			18		/* 18 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_IRQ_MAX_LEVEL		19		/* 19 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_BH_CALLS			20		/* 20 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_SYS_STATE_TL_MAX_LEVEL		21		/* 21 -> 1 		*/

/*
 * system-timer
 * PNET_SYSTEM_RDEV_REGISTER_SYS_TIME						(read-only)
 *	u64 .. system-time in microseconds
 *
 * PNET_SYSTEM_RDEV_REGISTER_SYS_TIME_HI
 *	u32 .. higher 32bits of system-time in seconds
 *
 * PNET_SYSTEM_RDEV_REGISTER_SYS_TIME_LO
 *	u12 .. lower 12bits of system-time in seconds
 *	u20 .. offset to system-time in microseconds
 */
#define PNET_SYSTEM_RDEV_REGISTER_SYS_TIME_HI				22		/* 22 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_SYS_TIME_LO				23		/* 23 -> 1 		*/

/*
 * load messurement-struct registers
 *
 * PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_CONFIG					(read-only)
 *	u16 .. enabled
 *	u16 .. messurement-duration in milliseconds
 * PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_COUNT	 				(read-only)
 *	number of runs
 * PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_CUR	 				(read-only)
 *	u32 .. current sys-load in tenth of a percent
 * PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_MIN	 				(read-only)
 *	u32 .. max sys-load in tenth of a percent
 * PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_MAX	 				(read-only)
 *	u32 .. min sys-load in tenth of a percent
 * PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_AVG	 				(read-only)
 *	u32 .. avg sys-load in tenth of a percent
 * PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_STDDEV 				(read-only)
 *	u32 .. standard deviation of sys-load in tenth of a percent
 */
#define PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_CONFIG			24		/* 24 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_COUNT	 		25		/* 25 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_CUR 			26		/* 26 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_MIN 			27		/* 27 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_MAX 			28		/* 28 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_AVG 			29		/* 29 -> 1 		*/
#define PNET_SYSTEM_RDEV_REGISTER_EVCORE_LOAD_STDDEV 			30		/* 30 -> 1 		*/

/*
 * application version
 *
 * PNET_SYSTEM_RDEV_REGISTER_APPLICATION_VERSION (read-only)
 *	u8 .. version_hi
 *	u8 .. version_med
 *	u8 .. version_lo
 * 	u8 .. release-candidate-version (0 - stable)
 */
#define PNET_SYSTEM_RDEV_REGISTER_APPLICATION_VERSION			31		/* 31 -> 1 		*/

/*
 * arm9 suspend support
 *
 * PNET_SYSTEM_RDEV_REGISTER_ARM9_SUSPEND_SUPPORT
 *	u1 .. supported
 *	u31 .. reserved
 * PNET_SYSTEM_RDEV_REGISTER_ARM9_SUSPEND_STATUS
 *	u32 .. status
 * PNET_SYSTEM_RDEV_REGISTER_ARM9_SUSPEND_MASK
 *	u32 .. mask
 */
#define PNET_SYSTEM_RDEV_REGISTER_ARM9_SUSPEND_SUPPORT			32		/* 32 -> 1		*/
#define PNET_SYSTEM_RDEV_REGISTER_ARM9_SUSPEND_STATUS			33		/* 33 -> 1		*/
#define PNET_SYSTEM_RDEV_REGISTER_ARM9_SUSPEND_MASK				34		/* 34 -> 1		*/

/*
 *********************************************************************************************************
 * END: Register - Definition
 *********************************************************************************************************
 */

/*
 **************************************************************************************************
 * Begin: IOCTL-Definitions
 **************************************************************************************************
 */

/* version */
struct pnet_system_version {
	char hi;
	char med;
	char lo;
	char rc;
};

/* time */
struct pnet_system_time {
	unsigned long long seconds;
	unsigned int useconds;
};

/* system-cpu */
struct pnet_system_cpu {
	char type;
	char powersave_mode;
	int oscclk;
	int sysclk;
	int iclk;
};

/* system-software */
struct pnet_system_evcore_cfg {
	unsigned int tasklet_biglock:1;
	unsigned int tasklet_absolute:1;
	unsigned int exc_irq_nested:1;
	unsigned int debug:1;
	unsigned int tasklet_debug:1;
	unsigned int tasklet_debug_trace:1;
	unsigned int tasklet_debug_callstack:1;
};

/* system-software-states */
struct pnet_system_evcore_state {
	unsigned int irq_calls;
	unsigned int irq_max_level;
	unsigned int bh_calls;
	unsigned int tl_max_level;
};

/* system-software-load */
struct pnet_system_evcore_load {
	unsigned short enabled;
	unsigned short duration;
	unsigned int count;
	unsigned int cur;
	unsigned int max;
	unsigned int min;
	unsigned int avg;
	unsigned int stddev;
};

/* evcore */
struct pnet_system_evcore {
	struct pnet_system_version version;
	struct pnet_system_evcore_cfg cfg;
	struct pnet_system_evcore_state state;
	struct pnet_system_evcore_load load;
	struct pnet_system_time time;
};

/* arm9-suspend */
struct pnet_system_arm9_suspend {
	unsigned int support;
	unsigned int status;
	unsigned int mask;
};

/* application */
struct pnet_system_application {
	struct pnet_system_version version;
	struct pnet_system_arm9_suspend arm9_suspend;
};

/* system-info */
struct pnet_system_info {
	struct pnet_system_cpu cpu;
	struct pnet_system_evcore evcore;
	struct pnet_system_application application;
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

struct pnet_system_private;

/*
 * update registers
 *	pnet_system .. pnet_system instance
 *	pnet_system_info .. info-structure
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_update(struct pnet_system_private *pnet_system, struct pnet_system_info *info, int wait);

/*
 * reset the system
 *	pnet_system .. pnet_system instance
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_sys_reset(struct pnet_system_private *pnet_system, int wait);

/*
 * sleep us microseconds with all enabled
 *	pnet_system .. pnet_system instance
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_this(struct pnet_system_private *pnet_system, unsigned int us, int wait);

/*
 * sleep us microseconds with tasklets locked
 *	pnet_system .. pnet_system instance
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_tasklet(struct pnet_system_private *pnet_system, unsigned int us, int wait);

/*
 * sleep us microseconds with irqs and tasklets locked
 *	pnet_system .. pnet_system instance
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_irq(struct pnet_system_private *pnet_system, unsigned int us, int wait);

/*
 * sleep us microseconds with all(fiq,irq and tasklets) locked
 *	pnet_system .. pnet_system instance
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_all(struct pnet_system_private *pnet_system, unsigned int us, int wait);

/*
 * set/reset system-states
 *	pnet_system .. pnet_system instance
 *	evcore_state .. evcore_state-struct
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_sys_state(struct pnet_system_private *pnet_system, struct pnet_system_evcore_state evcore_state, int wait);

/*
 * set/reset system-load
 *	pnet_system .. pnet_system instance
 *	evcore_load .. evcore_load-struct
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_sys_load(struct pnet_system_private *pnet_system, struct pnet_system_evcore_load evcore_load, int wait);

/*
 * get the mask of events which caused the last wakeup
 *	pnet_system .. pnet_system instance
 *	status .. pointer to status (wakeup-status (bits represent wakup-events))
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_get_wakeup_status(struct pnet_system_private *pnet_system, int *status, int wait);

/*
 * get the mask of events which caused the last wakeup
 *	pnet_system .. pnet_system instance
 *	mask .. pointer to mask (wakeup-status (bits represent wakup-events))
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_get_wakeup_mask(struct pnet_system_private *pnet_system, int *mask, int wait);

/*
 * set the mask of wakeup-events that will cause a wakeup
 *	pnet_system .. pnet_system instance
 *	mask .. mask to set (wakeup-status (bits represent wakup-events))
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_wakeup_mask(struct pnet_system_private *pnet_system, int mask, int wait);

/*
 * enable suspend-mode
 *	pnet_system .. pnet_system instance
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_suspend_enable(struct pnet_system_private *pnet_system, int wait);

/*
 * set timer for timed wakeup event in seconds
 *	pnet_system .. pnet_system instance
 *	sec .. seconds to wait before wakeup
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_wakeup_timer(struct pnet_system_private *pnet_system, unsigned int sec, int wait);

/*
 * read a fixed amount of data
 * parameters:
 *	pnet_system .. pnet_system instance
 *	buf .. buffer to read in
 *	count .. number of bytes to read
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_read(struct pnet_system_private *pnet_system, char *buf, int count, int wait);

/*
 * write a fixed amount of data
 * parameters:
 *	pnet_system .. pnet_system instance
 *	buf .. buffer to write from
 *	count .. number of bytes to write
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_write(struct pnet_system_private *pnet_system, char *buf, int count, int wait);

/*
 * open system-interface
 * parameters:
 * 	minor .. pnet_system-minor
 * 	pnet_system .. pnet_system instance (set by open)
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_open(int minor, struct pnet_system_private **pnet_system, int wait);

/*
 * close system-interface
 * parameters:
 * 	minor .. pnet_system-minor
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_close(int minor, int wait);


/***********************************************************************
 *  End: kernel api
 ***********************************************************************/
#else

#define PNET_SYSTEM_DEVICE	"/dev/pnet_system"

/*
 * update registers
 *	fd .. descriptor to system-device
 *	pnet_system_info .. info-structure
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_update(int fd, struct pnet_system_info *info, int wait);

/*
 * reset the system
 *	fd .. descriptor to system-device
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_sys_reset(int fd, int wait);

/*
 * sleep us microseconds with all enabled
 *	fd .. descriptor to system-device
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_this(int fd, unsigned int us, int wait);

/*
 * sleep us microseconds with tasklets locked
 *	fd .. descriptor to system-device
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_tasklet(int fd, unsigned int us, int wait);

/*
 * sleep us microseconds with irqs and tasklets locked
 *	fd .. descriptor to system-device
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_irq(int fd, unsigned int us, int wait);

/*
 * sleep us microseconds with all(fiq,irq and tasklets) locked
 *	fd .. descriptor to system-device
 *	us .. microseconds to pause
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_pause_all(int fd, unsigned int us, int wait);

/*
 * set/reset system-states
 *	fd .. descriptor to system-device
 *	evcore_state .. evcore_state-struct
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_sys_state(int fd, struct pnet_system_evcore_state evcore_state, int wait);

/*
 * set/reset system-load
 *	fd .. descriptor to system-device
 *	evcore_load .. evcore_load-struct
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_sys_load(int fd, struct pnet_system_evcore_load evcore_load, int wait);

/*
 * get the mask of events which caused the last wakeup
 *	fd .. descriptor to system-device
 *	status .. pointer to status (wakeup-status (bits represent wakup-events))
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_get_wakeup_status(int fd, int *status, int wait);

/*
 * get the mask of events which caused the last wakeup
 *	fd .. descriptor to system-device
 *	mask .. pointer to mask (wakeup-status (bits represent wakup-events))
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_get_wakeup_mask(int fd, int *mask, int wait);

/*
 * set the mask of wakeup-events that will cause a wakeup
 *	fd .. descriptor to system-device
 *	mask .. mask to set (wakeup-status (bits represent wakup-events))
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_wakeup_mask(int fd, int mask, int wait);

/*
 * enable suspend-mode
 *	fd .. descriptor to system-device
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_suspend_enable(int fd, int wait);

/*
 * set timer for timed wakeup event in seconds
 *	fd .. descriptor to system-device
 *	sec .. seconds to wait before wakeup
 *	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_set_wakeup_timer(int fd, unsigned int sec, int wait);

/*
 * open system-interface
 * parameters:
 * 	minor .. pnet_system-minor
 * 	wait .. wait-flag (0 .. no wait; >0 .. wait jiffies)
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_open(int minor, int wait);

/*
 * close system-interface
 * parameters:
 *	fd .. descriptor to system-device
 * return: <0 .. error(-ETIME .. timeout; -EAGAIN .. try again); 0 .. ok
 */
int pnet_system_close(int fd);

#endif


#endif /* __LIBPNET_SYSTEM_H__ */


