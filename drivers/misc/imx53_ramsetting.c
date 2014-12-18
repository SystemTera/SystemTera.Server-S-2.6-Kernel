/***********************************************************************
 *
 * @Authors: Manfred Schlägl, Ginzinger electronic systems GmbH (C) 2007
 * @Descr: Driver to read the register values of the current
 * 		active ram setting on Freescale IMX53.
 * @TODO:
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
 ***********************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <mach/hardware.h>

/***********************************************************************
 *  Driver Data
 ***********************************************************************/

#define IMX53_RAMSETTING_DRIVER_NAME 		"imx53_ramsetting"
#define IMX53_RAMSETTING_DRIVER_AUTHOR 		"Manfred Schlaegl jun. <manfred.schlaegl@gmx.at>"
#define IMX53_RAMSETTING_DRIVER_DESC 		"IMX53 Ram setting driver"

/***********************************************************************
 *  Debug-Messages
 ***********************************************************************/

/* flag for debug-messages */
#ifdef DEBUG
static int debug=1;
#else
static int debug=0;
#endif

#define _KERN_DEBUG_ KERN_INFO /* for debug */

#define MY_NAME	IMX53_RAMSETTING_DRIVER_NAME

/* kernel Output */
#define err(format, arg...) printk(KERN_ERR "%s: %s: " format "\n" , MY_NAME , __FUNCTION__ , ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: %s: " format "\n" , MY_NAME , __FUNCTION__ , ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: %s: " format "\n" , MY_NAME , __FUNCTION__ , ## arg)

/* debug messages */
#define dbg_raw(fmt, arg...) 						\
	do {								\
		if (debug)						\
			printk (_KERN_DEBUG_ fmt "\n", ## arg);		\
	} while (0)
/* debug messages with module- and function-information */
#define dbg(fmt, arg...) 						\
	dbg_raw("%s: %s:\t\t" fmt, MY_NAME , __FUNCTION__ , ## arg)	

#ifdef CONFIG_PROC_FS

/***********************************************************************
 *  procfs statistics
 ***********************************************************************/

static const unsigned int imx53_ramsetting_addr_table[] = {
	0x63F84004,
	0x63F84000,
	0x63F8400C,
	0x63F84010,
	0x63F84008,
	0x53FD4018,
	0x53FD4014,
	0x63F84000,
	0x53FD4014,
	0x53FD4018,
	0x63F84004,
	0x53FD4068,
	0x53FD406C,
	0x53FD4070,
	0x53FD4074,
	0x53FD4078,
	0x53FD407C,
	0x53FD4080,
	0x53FD4084,
	0x53FA8554,
	0x53FA8558,
	0x53FA8560,
	0x53FA8564,
	0x53FA8568,
	0x53FA8570,
	0x53FA8574,
	0x53FA8578,
	0x53FA857C,
	0x53FA8580,
	0x53FA8584,
	0x53FA8588,
	0x53FA8590,
	0x53FA8594,
	0x53FA86F0,
	0x53FA86F4,
	0x53FA86FC,
	0x53FA8714,
	0x53FA8718,
	0x53FA871C,
	0x53FA8720,
	0x53FA8724,
	0x53FA8728,
	0x53FA872C,
	0x63FD9098,
	0x63FD9088,
	0x63FD9090,
	0x63FD904C,
	0x63FD9050,
	0x63FD90F8,
	0x63FD907C,
	0x63FD9080,
	0x63FD9018,
	0x63FD9000,
	0x63FD900C,
	0x63FD9010,
	0x63FD9014,
	0x63FD902C,
	0x63FD9030,
	0x63FD9008,
	0x63FD9004,
	0x63FD901C,
	0x63FD901C,
	0x63FD901C,
	0x63FD901C,
	0x63FD901C,
	0x63FD901C,
	0x63FD901C,
	0x63FD901C,
	0x63FD901C,
	0x63FD901C,
	0x63FD901C,
	0x63FD9020,
	0x63FD9058,
	0x63FD901C,
	0x63FD9040,
	0x53FA8004
};


static int imx53_ramsetting_show(struct seq_file *m, void *v)
{
	int i;
	for(i=0;i<ARRAY_SIZE(imx53_ramsetting_addr_table);i++) {
		seq_printf(m,"0x%.8X = 0x%.8X\n",
				imx53_ramsetting_addr_table[i],
				readl(IO_ADDRESS(imx53_ramsetting_addr_table[i]+0x20000000)));
	}
	return 0;
}

static int imx53_ramsetting_open( struct inode *inode, struct file *file)
{
	return single_open(file, imx53_ramsetting_show, NULL);
}

static struct file_operations imx53_ramsetting_fops = {
	.owner = THIS_MODULE,
	.open = imx53_ramsetting_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct proc_dir_entry *imx53_ramsetting_proc_dir, *imx53_ramsetting_proc;
static int imx53_ramsetting_proc_create(void)
{
	imx53_ramsetting_proc_dir = proc_mkdir(IMX53_RAMSETTING_DRIVER_NAME, NULL);
	if(!imx53_ramsetting_proc_dir)
		return 0;

	imx53_ramsetting_proc = create_proc_entry("ramsetting",S_IRUGO,imx53_ramsetting_proc_dir);
	if(!imx53_ramsetting_proc) {
		remove_proc_entry(IMX53_RAMSETTING_DRIVER_NAME, NULL);
		return 0;
	}

	imx53_ramsetting_proc->proc_fops = &imx53_ramsetting_fops;

	return 0;
}
static void imx53_ramsetting_proc_remove(void)
{
	if (imx53_ramsetting_proc) 
		remove_proc_entry("ramsetting", imx53_ramsetting_proc_dir);
	if (imx53_ramsetting_proc_dir) 
		remove_proc_entry(IMX53_RAMSETTING_DRIVER_NAME, NULL);
}
#endif	


/* init pnet_ts - driver */	
static int __init imx53_ramsetting_init( void )
{
	dbg();
#ifdef CONFIG_PROC_FS
	imx53_ramsetting_proc_create();
#endif

	return 0;
}

static void __exit imx53_ramsetting_exit( void )
{
	dbg();
#ifdef CONFIG_PROC_FS
	imx53_ramsetting_proc_remove();
#endif
}

module_init(imx53_ramsetting_init);
module_exit(imx53_ramsetting_exit);

MODULE_AUTHOR(IMX53_RAMSETTING_DRIVER_AUTHOR);
MODULE_DESCRIPTION(IMX53_RAMSETTING_DRIVER_DESC);
MODULE_LICENSE("GPL");
