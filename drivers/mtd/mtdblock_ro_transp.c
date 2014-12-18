/*
 * (C) 2010 Manfred Schlaegl <manfred.schlaegl@gmx.at>
 * Based on:
 *	 mtdblock_ro.c,v 1.19 2004/11/16 18:28:59 dwmw2 Exp
 *   (C) 2003 David Woodhouse <dwmw2@infradead.org>
 *
 * Simple read-only mtdblock driver with Bad-Blocks masked out.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
/***********************************************************************
 *  @History:
 *	2010-KW35 manfred.schlaegl@gmx.at
 *		* dynamic bad-block-table per device
 *		* port to linux-2.6.35
 *	2010-KW22 manfred.schlaegl@gmx.at
 *		* port to linux-2.6.32
 ***********************************************************************/


#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/blktrans.h>

/*
 * private data for each mtd-device
 */
struct mtdblock_ro_transp_priv {
	/* bad-block table for transparency 
	 * the start-address of each bad block is saved in this array
	 * on read (and write) the offset-address is searched (divide and conquer)
	 * in the array. the index corresponds to the eraseblocks to skip
	 *
	*/
	unsigned long *bbt;

	/* bad block table_size */
	unsigned long bbt_size;
};

/* build the table */
static void mtdblock_bbt_build(struct mtd_blktrans_dev *dev)
{
	struct mtd_info *mtd = dev->mtd;
	struct mtdblock_ro_transp_priv *priv = (struct mtdblock_ro_transp_priv *)dev->priv;
	unsigned long offset;
	int bbt_idx=0;
	unsigned long *tmp;

	if (!mtd->block_isbad) {
		return;
	}

	/* prealloc space */
	priv->bbt=kmalloc(10*sizeof(unsigned long),GFP_KERNEL);
	if(!priv->bbt) {
		printk(KERN_ERR "error allocating %d bytes\n",10*sizeof(unsigned long));
		panic("mtdblock_ro_transp");
	}
	priv->bbt_size=10;

	/* search mtd */
	for(offset=0;offset<mtd->size;offset+=mtd->erasesize) {
		/* find bad block */
		if(mtd->block_isbad(mtd, offset)) {
			printk(KERN_INFO "mask out bad-block at 0x%lX on mtdblock%i\n",offset,mtd->index);
			/* found bad block -> move to bbt */
			if(bbt_idx>=priv->bbt_size) {
				/* realloc bbt if neccessary */
				tmp=priv->bbt;
				priv->bbt=kmalloc((priv->bbt_size+10)*sizeof(unsigned long), GFP_KERNEL);
				if(!priv->bbt) {
					printk(KERN_ERR "error allocating %lu bytes\n",(priv->bbt_size+10)*sizeof(unsigned long));
					panic("mtdblock_ro_transp");
				}
				memcpy(priv->bbt,tmp,priv->bbt_size*sizeof(unsigned long));
				priv->bbt_size+=10;
				kfree(tmp);
			}
			/* add bad block addr to bbt */
			priv->bbt[bbt_idx++]=offset;
		}
	}

	/* save size */
	priv->bbt_size=bbt_idx;
	/* free space, if not neccessary */
	if(!bbt_idx)
		kfree(priv->bbt);
}

/* free the table */
static void mtdblock_bbt_free(struct mtd_blktrans_dev *dev)
{
	struct mtdblock_ro_transp_priv *priv = (struct mtdblock_ro_transp_priv *)dev->priv;

	if(priv && priv->bbt)
		kfree(priv->bbt);
}

/* correct offset (hide bad-blocks) */
inline unsigned long mtdblock_transp_bad(struct mtd_blktrans_dev *dev, unsigned long offset)
{
	struct mtd_info *mtd = dev->mtd;
	struct mtdblock_ro_transp_priv *priv = (struct mtdblock_ro_transp_priv *)dev->priv;
	int bbt_idx;

	/* simple alg (todo: divide and conquer) */
	for(bbt_idx=0;bbt_idx<priv->bbt_size;bbt_idx++) {
		if(priv->bbt[bbt_idx]>offset)
			break;
		offset+=mtd->erasesize;
	}

	return offset;
}

static int mtdblock_readsect(struct mtd_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	size_t retlen;
	/* read with mapped block */
	if (dev->mtd->read(dev->mtd, mtdblock_transp_bad(dev,block * 512), 512, &retlen, buf))
		return 1;
	return 0;
}

static int mtdblock_writesect(struct mtd_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	size_t retlen;
	/* write with mapped block */
	if (dev->mtd->write(dev->mtd, mtdblock_transp_bad(dev,block * 512), 512, &retlen, buf))
		return 1;
	return 0;
}


static void mtdblock_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	struct mtd_blktrans_dev *dev;
	struct mtdblock_ro_transp_priv *priv;

	/* allocate device */
	dev=kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return;

	/* allocate private-data */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		kfree(dev);
		return;
	}

	/* link */
	dev->mtd = mtd;
	dev->devnum = mtd->index;
	dev->priv = priv;

	/* settings */
	dev->size = mtd->size >> 9;
	dev->tr = tr;
	dev->readonly = 1;

	/* allocate and create bbt */
	mtdblock_bbt_build(dev);

	/* register device */
	add_mtd_blktrans_dev(dev);
}

static void mtdblock_remove_dev(struct mtd_blktrans_dev *dev)
{
	/* unregister */
	del_mtd_blktrans_dev(dev);
	/* free bbt */
	mtdblock_bbt_free(dev);
	/* free private data */
	kfree(dev->priv);
	/* free device */
	kfree(dev);
}

static struct mtd_blktrans_ops mtdblock_tr = {
	.name		= "mtdblock",
	.major		= 31,
	.part_bits	= 0,
	.blksize 	= 512,
	.readsect	= mtdblock_readsect,
	.writesect	= mtdblock_writesect,
	.add_mtd	= mtdblock_add_mtd,
	.remove_dev	= mtdblock_remove_dev,
	.owner		= THIS_MODULE,
};

static int __init mtdblock_init(void)
{
	return register_mtd_blktrans(&mtdblock_tr);
}

static void __exit mtdblock_exit(void)
{
	deregister_mtd_blktrans(&mtdblock_tr);
}

module_init(mtdblock_init);
module_exit(mtdblock_exit);

MODULE_AUTHOR("Manfred Schlaegl <manfred.schlaegl@gmx.at>");
MODULE_DESCRIPTION("Simple read-only block device emulation access to MTD devices with Bad-Blocks masked out");
MODULE_LICENSE("GPL");

