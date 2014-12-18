/***********************************************************************
 *
 * @Authors: Manfred Schlägl, Ginzinger electronic systems GmbH (C) 2008
 * @Descr: Booting alternative System
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
/***********************************************************************
 *  @History:
 *	2013KW15 - melchior.franz
 *		* fix overflow bug in checksumming progress display
 *		* introduce CONFIG_ALTERNATIVE_ROOTFS_PANIC_ON_FAILURE option
 *	2013KW11 - melchior.franz
 *		* shorten alt_boot_info_printf lines for smaller displays
 *  	2012KW33 - manfred.schlaegl
 *  		* support for forced search of alt-boot images
 *  	2012KW23 - manfred.schlaegl@ginzinger.com
 *  		* support for mmc-cards
 *  		* cleanup
 *	2010-KW22 manfred.schlaegl@gmx.at
 *		* port to linux-2.6.32
 ***********************************************************************/
/***********************************************************************
 *  @TODO:
 *	* provide clean interface for alt_boot_request_force switch
 ***********************************************************************/

#include <linux/delay.h>
#include <linux/loop.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/errno.h>
#include "do_mounts.h"

/*
 * force search for alternative boot images
 */
unsigned int alt_boot_request_force=0;


#define NR_ALT_SYS_DEVICES_SCSI			255
#define NR_ALT_SYS_DEVICES_MMCBLK		255
#define NR_ALT_SYS_DEVICES			(NR_ALT_SYS_DEVICES_SCSI+NR_ALT_SYS_DEVICES_MMCBLK)

/* if flagfs is rootfs, flag-files are located in /boot */
#ifdef CONFIG_ALTERNATIVE_ROOTFS_FLAGFS_IS_ROOTFS
#define VALID_FILE_PATH				"/boot/SYS_VALID"
#define NO_ALT_SYS_FILE_PATH			"/boot/NO_ALT_BOOT"
#else
/* if flagfs is an extra fs, flag-files are located in / */
#define VALID_FILE_PATH				"/SYS_VALID"
#define NO_ALT_SYS_FILE_PATH			"/NO_ALT_BOOT"
#endif

#define ALT_SYS_IMAGE_PATH			CONFIG_ALTERNATIVE_ROOTFS_IMG_FILE

/* maximum update retry */
#define ALT_SYS_MAX_RETRY			CONFIG_ALTERNATIVE_ROOTFS_MAX_RETRIES

/* info output device */
#ifdef CONFIG_ALTERNATIVE_ROOTFS_INFO_OUTPUT
#define ALT_SYS_INFO_OUTPUT			1
#define ALT_SYS_INFO_OUTPUT_CHRDEV_NAME 	"/dev/info_chrdev"
#define ALT_SYS_INFO_OUTPUT_CHRDEV_MAJOR	CONFIG_ALTERNATIVE_ROOTFS_INFO_OUTPUT_CHRDEV_MAJOR
#define ALT_SYS_INFO_OUTPUT_CHRDEV_MINOR	CONFIG_ALTERNATIVE_ROOTFS_INFO_OUTPUT_CHRDEV_MINOR
#endif

/* definition of used crypt-sum */
#define USED_CRYPT_SUM				"md5"
#define CRYPT_SUM_SIZE				(128>>3)

#define READ_BUF_SIZE				1024
#define SEEK_SET 0 /* TODO */
#define SEEK_END 2 /* TODO */

/* convert hex-nibble */
static inline int convert_hex_nibble(char c)
{
	if((c>='A')&&(c<='Z'))
		return (10+c-'A');

	if((c>='a')&&(c<='z'))
		return (10+c-'a');

	if((c>='0')&&(c<='9'))
		return (c-'0');

	return -ERANGE;
}

#ifdef ALT_SYS_INFO_OUTPUT
/*
 * open/init info output
 * parameters: -
 * return: <0 .. error; 0 .. ok
 */
static int alt_boot_info_fd=-1;
static int __init alt_boot_info_output_open(void)
{
	dev_t info_device;

	/* create device */
	info_device = MKDEV(ALT_SYS_INFO_OUTPUT_CHRDEV_MAJOR, ALT_SYS_INFO_OUTPUT_CHRDEV_MINOR);
	sys_unlink(ALT_SYS_INFO_OUTPUT_CHRDEV_NAME);
	if(sys_mknod(ALT_SYS_INFO_OUTPUT_CHRDEV_NAME,S_IFCHR|0600,new_encode_dev(info_device))==-1) {
		printk(KERN_INFO "Error mknod chrdev \"%s\" major=%i, minor=%i\n",ALT_SYS_INFO_OUTPUT_CHRDEV_NAME,ALT_SYS_INFO_OUTPUT_CHRDEV_MAJOR,ALT_SYS_INFO_OUTPUT_CHRDEV_MINOR);		
		return -1;
	}

	/* open */
	if((alt_boot_info_fd=sys_open(ALT_SYS_INFO_OUTPUT_CHRDEV_NAME, O_RDWR, 0600)) < 0) {
		printk(KERN_INFO "Error open chrdev \"%s\" major=%i, minor=%i\n",ALT_SYS_INFO_OUTPUT_CHRDEV_NAME,ALT_SYS_INFO_OUTPUT_CHRDEV_MAJOR,ALT_SYS_INFO_OUTPUT_CHRDEV_MINOR);
		return -3;
	}

	/* ok */
	return 0;
}

/*
 * uninit/close info output
 * parameters: -
 * return: <0 .. error; 0 .. ok
 */
static int __init alt_boot_info_output_close(void)
{
	/* close, if opened */
	if(alt_boot_info_fd)
		sys_close(alt_boot_info_fd);

	/* ok */
	return 0;
}

/*
 * info output macros
 */
static void __init alt_boot_info_write(char *str)
{
	if(alt_boot_info_fd>=0) {
		sys_write(alt_boot_info_fd,str,strlen(str));
	}
}

#define alt_boot_info_printf(format, arg...)		\
	do {						\
		char str___buf[512];			\
		sprintf(str___buf, format, ## arg);	\
		alt_boot_info_write(str___buf);		\
	} while(0)

#else /* ALT_SYS_INFO_OUTPUT */

/* dummy */
#define alt_boot_info_output_open() 			while(0)
#define alt_boot_info_output_close() 			while(0)
#define alt_boot_info_output_write(__str__) 		while(0)
#define alt_boot_info_printf(format, arg...) 		while(0)

#endif /* ALT_SYS_INFO_OUTPUT */

extern int get_filesystem_list(char * buf);
static void __init get_fs_names(char *page)
{
	char *s = page;

	int len = get_filesystem_list(page);
	char *p, *next;

	page[len] = '\0';
	for (p = page-1; p; p = next) {
		next = strchr(++p, '\n');
		if (*p++ != '\t')
			continue;
		while ((*s++ = *p++) != '\n')
			;
		s[-1] = '\0';
	}
	*s = '\0';
}

static int __init auto_mount(char *name, char *mountpoint, int flags, void *data)
{
	char *fs_names = __getname();
	char *p;
	int ret=0;

	/* get fs-names */
	get_fs_names(fs_names);
	/* try mounts */
	for (p = fs_names; *p; p += strlen(p)+1) {
		ret = sys_mount(name,mountpoint,p,flags,data);
		if(ret==-EINVAL)
			continue;

		if(ret==0) {
			/* success */
			printk("%s mounted on %s with %s\n",name,mountpoint,p);
			ret=0;
			goto out;
		} else {
#if 0
			/* debug */
			printk("tried: ");
			for (p = fs_names; *p; p += strlen(p)+1)
				printk(" %s", p);
			printk("\n");
#endif
			/* error */
			ret=-1;
			goto out;
		}
	}

out:
	putname(fs_names);
	return ret;
}
		
/* check boot-image */
#define CRYPT_SUM_MAX_SIZE (2*CRYPT_SUM_SIZE+255+10)
static int __init check_boot_image(char *path)
{
	int fd;
	struct crypto_hash *tfm;
	struct hash_desc hdesc;
	char *data;
	char crypt_sum_calc[CRYPT_SUM_SIZE];
	char crypt_sum_read[CRYPT_SUM_SIZE];
	struct scatterlist sg;
	int size,i,j,ret;
	unsigned long img_size, file_size, rest_size;

	alt_boot_info_printf("CHECKING BOOT IMAGE:   0%%");

	/* allocate buffer */
	data=kmalloc(READ_BUF_SIZE*sizeof(char), GFP_KERNEL);
	if (!data) {
		printk(KERN_INFO "failed to allocate data buffer\n");
		ret=-ENOMEM;
		goto out;
	}

	/* check existing of file and crypt_sum */
	/* open image file */
	printk(KERN_INFO "checking for alternative image (\"%s\") ... \n",path);
	if((fd=sys_open(path, O_RDONLY, 0600)) < 0) {
		printk(KERN_INFO "\tfailed to open \"%s\"\n",path);
		ret=fd;
		goto out_free;
	}


	/* get crypt_sum at end of alternative-image */


	/* read crypt_sum */
	/* read last READ_BUF_SIZE bytes */
	ret=sys_lseek(fd,-CRYPT_SUM_MAX_SIZE,SEEK_END);
	if(ret==(-CRYPT_SUM_MAX_SIZE-1)) {
		printk(KERN_INFO "error lseek\n");
		ret=-1;
		goto out_close;
	}
	/* image size without last CRYPT_SUM_MAX_SIZE bytes */
	img_size=(ret<0 ? 0 : ret);

	/* read last part of file */
	ret=sys_read(fd,data,CRYPT_SUM_MAX_SIZE);
	if(ret<0) {
		printk(KERN_INFO "error reading\n");
		ret=-1;
		goto out_close;
	}
	/* calculate file-size */
	file_size=img_size+ret;

	/* find end of image / begin of crypt_sum (backwards) */
	for(i=ret-1;i>=0;i--) {
		if(data[i]==' ') {
			/* image size = start of checksum */
			img_size=file_size-(2*CRYPT_SUM_SIZE)-(ret-i);
			break;
		}
	}
	if(i<0) {
		printk(KERN_INFO "error: crypt_sum not found\n");
		ret=-EIO;
		goto out_close;
	}
	printk(KERN_INFO "file size: %lu, image_size: %lu\n",file_size,img_size);

	/* convert read crypt_sum */
	i-=CRYPT_SUM_SIZE*2;
	for(j=0;j<CRYPT_SUM_SIZE;j++) {
		crypt_sum_read[j]=
			(16*convert_hex_nibble(data[i+j*2+0]))+
			convert_hex_nibble(data[i+j*2+1]);
	}

	/* output read crypt_sum */
	printk(KERN_INFO "read crypt_sum:\t");
	for(i=0;i<CRYPT_SUM_SIZE;i++)
		printk(KERN_INFO "0x%X ",crypt_sum_read[i]);
	printk(KERN_INFO "\n");


	/* calculate crypt_sum from image */

	/* start at begin of file */
	if(sys_lseek(fd,0,SEEK_SET)==-1) {
		printk(KERN_INFO "error lseek\n");
		ret=-1;
		goto out_close;
	}

	/* alloc crypto */
	tfm = crypto_alloc_hash(USED_CRYPT_SUM, 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(tfm)) {
		printk(KERN_INFO "failed to load transform for \""USED_CRYPT_SUM"\"\n");
		ret=-EINVAL;
		goto out_close;
	}

	hdesc.tfm = tfm;
	hdesc.flags = 0;

	if(crypto_hash_init(&hdesc)) {
		printk(KERN_INFO "failed to init crypto\n");
		ret=-EINVAL;
		goto out_close;
	}

	rest_size=img_size;
	while((size=sys_read(fd,data,READ_BUF_SIZE))>0) {
		/* cut checksum-part at end of image */
		if(rest_size>=size)
			rest_size-=size;
		else
			size=rest_size;

		/* calc */
		sg_set_buf(&sg, data, size);
		crypto_hash_update(&hdesc, &sg, size);

		/* "100 - 100 * rest_size / img_size" seems more natural, but causes an
		 * overflow if rest_size > 42949672 on systems where sizeof(long) == 4
		 */
		if (img_size >= 100)
			alt_boot_info_printf("\rCHECKING BOOT IMAGE: %3.lu%%   ",
					100 - rest_size / (img_size / 100));
	}
	if(size<0) {
		printk(KERN_INFO "read error\n");
		ret=size;
		goto out_free_close;
	}
	/* get result */
	crypto_hash_final(&hdesc, crypt_sum_calc);

	/* output calculated crypt_sum */	
	printk(KERN_INFO "calc crypt_sum:\t");
	for(i=0;i<CRYPT_SUM_SIZE;i++)
		printk(KERN_INFO "0x%X ",crypt_sum_calc[i]);
	printk(KERN_INFO "\n");

	/* compare crypt_sum */
	if(memcmp(crypt_sum_calc,crypt_sum_read,CRYPT_SUM_SIZE)) {
		printk(KERN_INFO "error: crypt sums differ\n");
		ret=-EPERM;
	} else {
		printk(KERN_INFO "crypt sums are equal\n");
		ret=0;
	}

out_free_close:
	crypto_free_hash(tfm);
out_close:
	sys_close(fd);
out_free:
	kfree(data);
out:
	if(ret) {
		alt_boot_info_printf("\rCHECKING BOOT IMAGE: FAILED! \n");
	} else {
		alt_boot_info_printf("\rCHECKING BOOT IMAGE: OK!     \n");
	}
	return ret;
}


/* loopback mount */
int loop_fd, image_fd;
char loop_dev[]="/dev/loopX";
static int __init loop_mount(char *dev_name, char *mount_point, char *fs, unsigned long flags, char *data)
{
	int i,ret;
	struct loop_info loopinfo;
	dev_t loop_device;

	/* find a free loop device */
	for(i=0; i<8; i++) {
		loop_dev[9]='0'+i;

		/* check if device exists */
		loop_device=name_to_dev_t(loop_dev);
		if(!MAJOR(loop_device)) {
			printk(KERN_INFO "No such device \"%s\"\n",loop_dev);
			continue;	
		}
		if(create_dev(loop_dev, loop_device)<0) {
			printk(KERN_INFO "Error creating device \"%s\"\n",loop_dev);
			continue;
		}
		if((loop_fd = sys_open (loop_dev, O_RDONLY, 0600)) >= 0) {
			ret=sys_ioctl(loop_fd, LOOP_GET_STATUS, (unsigned long)&loopinfo);
			if(ret == -ENXIO)
				break; /* got one! */
			sys_close(loop_fd);
		}
		loop_fd=-1;
	}
	if(loop_fd==-1) {
		printk(KERN_INFO "couldn't find free loop device\n");
		return -ENODEV;
	}

	/* open image file */
	if((image_fd=sys_open(dev_name, O_RDONLY, 0600)) < 0) {
		printk(KERN_INFO "failed to open \"%s\"\n",dev_name);
		ret=-1;
		goto out_close;
	}

	/* set up the loop device */
	memset(&loopinfo, 0, sizeof(loopinfo));
	strncpy(loopinfo.lo_name, dev_name, LO_NAME_SIZE);
	loopinfo.lo_name[LO_NAME_SIZE-1] = 0;
	loopinfo.lo_offset = 0;
	loopinfo.lo_encrypt_key_size = 0;
	if (sys_ioctl(loop_fd, LOOP_SET_FD, image_fd) < 0) {
		printk(KERN_INFO "ioctl failed on \"%s\"\n", loop_dev);
		ret=-1;
		goto out_close_close;
	}
	if (sys_ioctl(loop_fd, LOOP_SET_STATUS, (unsigned long)&loopinfo) < 0) {
		printk(KERN_INFO "ioctl failed on \"%s\"\n", loop_dev);
		ret=-1;
		goto out_clr_close_close;
	}

	/* do the mount */
	if(sys_mount(loop_dev, mount_point, fs, flags, data)) {
		printk(KERN_INFO "failed to mount \"%s\" on \"%s\"\n", loop_dev, mount_point);
		ret=-1;
		goto out_clr_close_close;
	}

	ret=0;
	goto out;

out_clr_close_close:
	if (sys_ioctl (loop_fd, LOOP_CLR_FD, 0))
		printk(KERN_INFO "failed to clear \"%s\"\n", loop_dev);
out_close_close:
	sys_close(image_fd);
out_close:
	sys_close(loop_fd);
out:
	return ret;
}

#if 0 /* unused */
/* loopback unmount */
static int __init loop_umount(char *mount_point, int i)
{
	int ret;

	/* do the umount */
	if(sys_umount(mount_point,i)) {
		ret=-1;
		printk(KERN_ERR "failed to umount \"%s\" from \"%s\"\n", loop_dev, mount_point);
	    if (sys_ioctl (loop_fd, LOOP_CLR_FD, 0))
			printk(KERN_ERR "failed to clear \"%s\"\n", loop_dev);
    	return ret;
	}

	/* clean up */
 	sys_close(image_fd);
	ret=sys_ioctl (loop_fd, LOOP_CLR_FD, 0);
	if (ret<0) {
 		printk(KERN_ERR "failed to clear \"%s\"\n", loop_dev);
		return ret;
 	}
 	sys_close(loop_fd);

	return 0;
}
#endif

/* check flags */
#ifdef CONFIG_ALTERNATIVE_ROOTFS_FLAGFS_IS_ROOTFS
/* flagfs is rootfs */
#define FLAGFS_MOUNTPOINT	"/root"
#else
/* extern flagfs */
#define FLAGFS_MOUNTPOINT	"/flagfs"
#define FLAGFS_DEVICE		CONFIG_ALTERNATIVE_ROOTFS_FLAGFS_DEVICE
#define FLAGFS_FSTYPE		CONFIG_ALTERNATIVE_ROOTFS_FLAGFS_FSTYPE

#endif
static void __init alt_boot_check_flags(int *rootfs_invalid, int *alt_boot_requested)
{
#ifdef CONFIG_ALTERNATIVE_ROOTFS_RECREATE_ALT_BOOT_FILE
	int fd;
#endif


/* mount flagfs if other than rootfs
 * rootfs is already mounted
 */
#ifndef CONFIG_ALTERNATIVE_ROOTFS_FLAGFS_IS_ROOTFS
	dev_t mtd_device;

	/* set invalid for returns */
	(*rootfs_invalid)=1;
	(*alt_boot_requested)=1;

	/* check if device exists */
	mtd_device=name_to_dev_t(FLAGFS_DEVICE);
	if(!MAJOR(mtd_device)) {
		printk(KERN_INFO "No such device \"%s\"\n",FLAGFS_DEVICE);
		return;	/* invalid if no device */
	}
	if(create_dev(FLAGFS_DEVICE, mtd_device)<0) {
		printk(KERN_INFO "Error creating device \"%s\"\n",FLAGFS_DEVICE);
		return;	/* invalid if error creating device */
	}


	/* create mountpoint */
	if(sys_mkdir(FLAGFS_MOUNTPOINT,0600)==-1) {
		/* error if not already exists */
		printk(KERN_ERR "error creating dir \""FLAGFS_MOUNTPOINT"\"\n");
		panic("error creating dir \""FLAGFS_MOUNTPOINT"\" -> root-tmpfs ro");
		return;
	}

	/* mount flag-partition */
	printk(KERN_INFO "mount flagfs partition (\""FLAGFS_DEVICE"\" as \""FLAGFS_FSTYPE"\" on \""FLAGFS_MOUNTPOINT"\")\n");
	if(sys_mount(FLAGFS_DEVICE,FLAGFS_MOUNTPOINT,FLAGFS_FSTYPE,MS_MGC_VAL, NULL)!=0) {
		printk(KERN_ERR "\terror mounting\n");
		return;	/* invalid if we are not able to mount */
	}
#endif

	/* reset request flag */
	(*alt_boot_requested)=0;

	/* check for sys-valid */
	printk(KERN_INFO "check flagfs(\""FLAGFS_MOUNTPOINT"\") for \""VALID_FILE_PATH"\" ... \n");
	/* check if alternative boot necessary */
	if (sys_access((const char __user *) FLAGFS_MOUNTPOINT"/"VALID_FILE_PATH, 0) == 0) {
		(*rootfs_invalid)=0;
		printk(KERN_INFO "\tsystem valid\n");
	} else {
		(*rootfs_invalid)=1;
		printk(KERN_INFO "\tsystem invalid\n");
	}

	/* check no_alt_boot flag */
	printk(KERN_INFO "check flagfs(\""FLAGFS_MOUNTPOINT"\") for \""NO_ALT_SYS_FILE_PATH"\" ... \n");
	/* check if alt-boot to do */
	if (sys_access((const char __user *) FLAGFS_MOUNTPOINT"/"NO_ALT_SYS_FILE_PATH, 0) == 0) {
		printk(KERN_INFO "\tno alternative boot requested\n");
	} else {
		(*alt_boot_requested)=1;
		printk(KERN_INFO "\talternative boot requested\n");

#ifdef CONFIG_ALTERNATIVE_ROOTFS_RECREATE_ALT_BOOT_FILE
		printk(KERN_INFO "recreate \""NO_ALT_SYS_FILE_PATH"\" in flagfs(\""FLAGFS_MOUNTPOINT"\")\n");
		/* recreate */
		fd=sys_creat((const char __user *) FLAGFS_MOUNTPOINT"/"NO_ALT_SYS_FILE_PATH, 0);
		if (fd < 0) {
			printk(KERN_ERR "\terror recreating\n");
			goto __ret_umount;
		} else {
			sys_close(fd);
		}
#endif
	}

	/* check alt_boot_request force switch */
	printk(KERN_INFO "check alternative boot request force switch\n");
	if(!alt_boot_request_force) {
		printk(KERN_INFO "\talternative boot request not forced\n");
	} else {
		(*alt_boot_requested)=1;
		printk(KERN_INFO "\talternative boot request forced\n");
	}


/*
 * umount flagfs if other than rootfs
 * rootfs should not get unmounted
 */
#ifdef CONFIG_ALTERNATIVE_ROOTFS_RECREATE_ALT_BOOT_FILE
__ret_umount:
#endif

#ifndef CONFIG_ALTERNATIVE_ROOTFS_FLAGFS_IS_ROOTFS
	/* umount flag-partition */
	if(sys_umount(FLAGFS_MOUNTPOINT, 0)==-1) {
		printk(KERN_ERR "umount flagfs failed\n");
		panic("umount flagfs failed.");
		return;
	}

	/* remove mountpoint */
	if(sys_unlink(FLAGFS_MOUNTPOINT)==-1) {
		printk(KERN_INFO "unlink \""FLAGFS_MOUNTPOINT"\" failed\n");
		return;
	}
#endif
	return;
}

/* do alternative system mount */
int __init do_alt_boot(void)
{
	int ret;
	unsigned long device_nr,retry,max_retry;
	char tmp[256];
	dev_t blk_dev;
	int alt_boot_requested, rootfs_invalid, alt_img_valid;

	/* reset valid flag */
	alt_img_valid=0;

	/* check flags */
	alt_boot_check_flags(&rootfs_invalid,&alt_boot_requested);

	/* alt-boot to do ? */
	if((rootfs_invalid||alt_boot_requested) == 0)
		return 0;

	/* unmount invalid rootfs */
	sys_chdir("/");
	sys_mkdir("/mnt",0777);

	/* open info-output */
	alt_boot_info_output_open();

	/* search for alt-image */
	ssleep(2);
	printk(KERN_INFO "Try to find block device containing alternative boot image\n");
	device_nr=0;
	retry=1;
	if(rootfs_invalid) {
		printk(KERN_INFO "rootfs invalid -> retry ever\n");
		max_retry=0;
		alt_boot_info_printf("SYSTEM DAMAGED!\n");
	} else {
		max_retry=ALT_SYS_MAX_RETRY;
	}

	alt_boot_info_printf("SEARCHING FOR ALTERNATIVE BOOT IMAGE ");
	do {
		if(device_nr==0) {
			alt_boot_info_printf(".");
			if(max_retry)
				printk(KERN_INFO "Try alt-boot %lu/%lu\n",retry,max_retry);
			else
				printk(KERN_INFO "Try alt-boot %lu/inf\n",retry);
		}

		if(device_nr>=NR_ALT_SYS_DEVICES) {
			/* no block-device with an valid alt-boot-img found */
			/* restart */
			device_nr=0;
			printk(KERN_INFO "no block device found -> restart\n");
			ssleep(2);

			/* count max retries */
			retry++;
			if(max_retry&&(retry>max_retry)) {
				alt_boot_info_printf("\nGIVING UP!\nALT. BOOT IMAGE MISSING OR INVALID\n");
				break;
			} else {
				printk(KERN_INFO "retry\n");
			}

			continue;
		}

		/* generate device string */
		if(device_nr<NR_ALT_SYS_DEVICES_SCSI) {
			/* scsi-device */
			int scsi_device_nr=device_nr-0;
			if(scsi_device_nr%16)
				sprintf(tmp,"/dev/sd%c%i",'a'+(char)(scsi_device_nr/16),scsi_device_nr%16);
			else
				sprintf(tmp,"/dev/sd%c",'a'+(char)(scsi_device_nr/16));
		} else { 
			/* mmcblk-device */
			int mmcblk_device_nr=device_nr-NR_ALT_SYS_DEVICES_SCSI;
			if(mmcblk_device_nr%16)
				sprintf(tmp,"/dev/mmcblk%ip%i",(mmcblk_device_nr/16),mmcblk_device_nr%16);
			else
				sprintf(tmp,"/dev/mmcblk%i",(mmcblk_device_nr/16));
		}
		device_nr++;

		/* check if device exists */
		blk_dev=name_to_dev_t(tmp);
		if(!MAJOR(blk_dev))
			continue;	

		printk(KERN_INFO "create device: \"%s\"\n",tmp);
		/* unlink & create device */
		if(create_dev(tmp, blk_dev)<0) {
			printk(KERN_INFO "Error creating device \"%s\"\n",tmp);
			continue;
		}

		printk(KERN_INFO "try mount \"%s\" ... \n",tmp);
		/* try mount */
		ret=auto_mount(tmp,"/mnt", MS_RDONLY | MS_MGC_VAL, NULL);
		if(ret<0) {
			printk(KERN_INFO "\tfailed %i\n",ret);
			continue;
		}
		printk(KERN_INFO "\tsuccess\n");
		alt_boot_info_printf("\nFOUND!\n");
		
		printk(KERN_INFO "Block device \"%s\" found and mounted\n",tmp);

		/* check alt-boot-image */
		printk(KERN_INFO "checking alternative boot image (\"/mnt/"ALT_SYS_IMAGE_PATH"\") ... \n");
		if (check_boot_image("/mnt/"ALT_SYS_IMAGE_PATH) != 0) {
			printk(KERN_INFO "\tfailed\n");

			printk(KERN_INFO "umount\n\tsuccess\n");
			sys_umount("/mnt",0);	/* problems */

			alt_boot_info_printf("SEARCHING FOR ALTERNATIVE BOOT IMAGE ");
			continue;
		}
		printk(KERN_INFO "\tsuccess\n");

		/* try to mount alt-boot-image */
		printk(KERN_INFO "try to mount alternative boot image (\"/mnt/"ALT_SYS_IMAGE_PATH"\") ... \n");
		ret=loop_mount("/mnt/"ALT_SYS_IMAGE_PATH,"/mnt","cramfs", MS_RDONLY | MS_MGC_VAL, NULL);
		if(ret<0) {
			printk(KERN_INFO "\tfailed -> try next device\n");

			printk(KERN_INFO "umount\n\tsuccess\n");
			sys_umount("/mnt",0);	/* problems */

			alt_boot_info_printf("SEARCHING FOR ALTERNATIVE BOOT IMAGE ");
			continue;
		}
		printk(KERN_INFO "\tsuccess\n");

		/* boot-img is started with init */
		alt_img_valid=1;
		break;
	} while(1);

	if(alt_img_valid) {
		alt_boot_info_printf("STARTING ALTERNATIVE SYSTEM\n");
		printk(KERN_INFO "Starting alternative system\n");
		sys_umount("/root",0);
		sys_mount("/mnt", "/root", NULL, MS_MOVE, NULL);
	} else {
		alt_boot_info_printf("\n");
		if(!rootfs_invalid) {
#ifdef CONFIG_ALTERNATIVE_ROOTFS_PANIC_ON_FAILURE
			panic("alternative boot image (\"/mnt/"ALT_SYS_IMAGE_PATH"\") missing or invalid\n");
#else
			printk(KERN_INFO "rootfs is valid, boot of alternative system was requested but image was not found! -> starting default system\n");
			alt_boot_info_printf("NO ALTERNATIVE BOOT IMAGE FOUND\nSTARTING DEFAULT SYSTEM\n");
#endif
		} else {
			/* never reached: rootfs invalid && no update */
		}
	}

	sys_chdir("/root");

	/* open info-output */
	alt_boot_info_output_close();

	return 0;
}


