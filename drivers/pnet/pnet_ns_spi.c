/***********************************************************************
 *
 * @Authors: Andreas Schrattenecker, Ginzinger electronic systems GmbH
 * @Descr: Netsilicon SPI Packet Driver for NS9360.
 * @References: [1] NS9360 Hardware Reference, March 2005
 * @TODO:
 *    o optimize spinlock in spi_init() to minimum  
 *    o ...
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
 *	2011KW11: manfred.schlaegl
 *		support for kap_bed-board (A9M9_BOARD_KAP_BED)
 *	2010-KW22 manfred.schlaegl@gmx.at
 *		* port to linux-2.6.32
 *	11.11.2009: manfred.schlaegl
 *		* Support for bhz_b-board
 ***********************************************************************/

#include <linux/fs.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <mach/irqs.h>
#include <mach/ns9xxx_spi.h>
#include <mach/ns9xxx_sys.h>
#include <mach/hardware.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "pnet_core.h"

#define DRV_NAME		"pnet_ns_spi"

inline void ns9xxx_gpio_setpin_raw(unsigned int pin, unsigned int val)
{
	void __iomem *addr = (void __iomem *)(NS9XXX_CTRL_ADDR_BASE_VA + NS9XXX_CTRL_PIN_OFFSET(pin));
	u32 ctrlreg;
	u32 mask = (1<<((pin)%32));

	ctrlreg = __raw_readl(addr) & ~mask;
	if(val) 
		ctrlreg |= mask;
	__raw_writel(ctrlreg, addr);
}

#define ns9xxx_gpio_getpin_raw(pin)	\
	( __raw_readl((void __iomem *)(NS9XXX_STAT_ADDR_BASE_VA + NS9XXX_STAT_PIN_OFFSET(pin))) & (1<<((pin)%32)) )


//#define DEBUG


typedef struct
{
	unsigned int* src;
	unsigned int len;       
	unsigned int* dest;	/* unused */
	union {
		unsigned int reg;
		struct {
			unsigned status : 16;
			unsigned res : 12;
			unsigned full : 1;
			unsigned last : 1;
			unsigned intr : 1;
			unsigned wrap : 1;
		} bits;
	} s;
}spi_buffer_desc_t;

#define SPI_STATE_INIT			0
#define SPI_STATE_RESET_REQUEST		1
#define SPI_STATE_RESET			2
#define SPI_STATE_WAIT_CONNECTED	3
#define SPI_STATE_CONNECTED		4

#define SPI_WAIT_RESET_REQUEST_T	1*10	/* 1sek RESET_REQUEST */
int spi_state=SPI_STATE_INIT;
static spinlock_t spi_state_lock=SPIN_LOCK_UNLOCKED;

#if		defined(CONFIG_A9M9_BOARD_VALI) 		|| \
		defined(CONFIG_A9M9_BOARD_GEA9A7DEV) 	|| \
		defined(CONFIG_A9M9_BOARD_ETABEB)

#define SPI_GPIO_SLAVE_REQ		10		/* GPIO10 */
#define SPI_GPIO_MASTER_REQ		11		/* GPIO11 = EXT IRQ2 */
#define SPI_GPIO_EXT_INT_CTRL	NS_SYS_EXT_INT_CTRL(2)
#define SPI_GPIO_EXT_INT		IRQ_EXT2

#elif 	defined(CONFIG_A9M9_BOARD_ETA_BEP) 		|| \
		defined(CONFIG_A9M9_BOARD_CMCX)			|| \
		defined(CONFIG_A9M9_BOARD_BHZ_B)		|| \
		defined(CONFIG_A9M9_BOARD_LWE_CU)		|| \
		defined(CONFIG_A9M9_BOARD_KAP_BED)

#define SPI_GPIO_SLAVE_REQ		11		/* GPIO11 */
#define SPI_GPIO_MASTER_REQ		13		/* GPIO13 = EXT IRQ0 */
#define SPI_GPIO_EXT_INT_CTRL	NS_SYS_EXT_INT_CTRL(0)
#define SPI_GPIO_EXT_INT		IRQ_EXT0

#endif


#define SPI_SLAVE_REQ_NS		0		/* 0 ns, as fast as possible */
#define SPI_RESET_REQ_SAMPLES	5		/* count REQUEST line samples */
#define SPI_MASTER_REQ_MAX_NS	1000		/* 1000 ns, REQUEST > SPI_MASTER_REQ_MAX_US = REQUEST_RESET */

#define SPI_PACKET_TXRDY_BITS	6
#define SPI_PACKET_PRI_BITS	2
#define SPI_PACKET_PORT_BITS	4
#define SPI_PACKET_LEN_BITS	5

#define SPI_PACKET_PRI_N	(1<<SPI_PACKET_PRI_BITS)
#define SPI_PACKET_PORT_N	(1<<SPI_PACKET_PORT_BITS)
#define	SPI_PACKET_DATA_LEN	16


/* sizeof(spi_packet_t) have to be multiple of 4 */
typedef struct
{
	union {
		unsigned int header;
		struct {
			unsigned pri : SPI_PACKET_PRI_BITS;
			unsigned txrdy : SPI_PACKET_TXRDY_BITS;

			unsigned port: SPI_PACKET_PORT_BITS;
			unsigned res1: 8-SPI_PACKET_PORT_BITS;

			unsigned stop: SPI_PACKET_PRI_N;
			unsigned res2: 8-SPI_PACKET_PRI_N;
				
			unsigned len : SPI_PACKET_LEN_BITS;
			unsigned res3: 8-SPI_PACKET_LEN_BITS-1;
			unsigned sys:  1;


		} __attribute__((packed)) bits;
	} __attribute__((packed)) h;
	unsigned char data[SPI_PACKET_DATA_LEN];
} __attribute__((packed)) spi_packet_t;


#define SPI_PACKET_LEN		sizeof(spi_packet_t)
#define SPI_BUFFER_DESC_N	8
static ns_spi_dev spi_dev;

#define SPI_TX_BUFFER_DESC_N	SPI_BUFFER_DESC_N
static spi_buffer_desc_t *spi_tx_buffer_desc_virt;
static dma_addr_t spi_tx_buffer_desc_bus;

#define SPI_TX_BUFFER_LEN	SPI_PACKET_LEN
static spi_packet_t *spi_tx_buffer_virt;
static dma_addr_t spi_tx_buffer_bus;

#define SPI_RX_BUFFER_DESC_N	SPI_BUFFER_DESC_N
static spi_buffer_desc_t *spi_rx_buffer_desc_virt;
static dma_addr_t spi_rx_buffer_desc_bus;

#define SPI_RX_BUFFER_LEN	SPI_PACKET_LEN
static spi_packet_t *spi_rx_buffer_virt;
static dma_addr_t spi_rx_buffer_bus;

static int spi_rxtx_head;
static int spi_rxtx_tail;
static int spi_rxtx_len;

static int spi_txrdy_master;
static atomic_t spi_master_request = ATOMIC_INIT(0);

static int spi_pri_stop_master;
static int spi_pri_stop_slave;
static int spi_force_transfer;

#define SPI_RX_PACKET_BUFFER_LEN	80
#define SPI_TX_PACKET_BUFFER_LEN	64

/* max SPI_PACKET_BUFFER_STOP_LEN = (SPI_PACKET_BUFFER_LEN-2*SPI_BUFFER_DESC_N)! */
#define SPI_RX_PACKET_BUFFER_STOP_LEN	(SPI_RX_PACKET_BUFFER_LEN-2*SPI_BUFFER_DESC_N)
#define SPI_RX_PACKET_BUFFER_START_LEN	(SPI_RX_PACKET_BUFFER_LEN-3*SPI_BUFFER_DESC_N)

typedef struct
{
	spi_packet_t buffer[SPI_RX_PACKET_BUFFER_LEN];
	int head;
	int tail;
	int len;
} spi_rx_packet_buffer_t;

typedef struct
{
	spi_packet_t buffer[SPI_TX_PACKET_BUFFER_LEN];
	int head;
	int tail;
	int len;
} spi_tx_packet_buffer_t;


static spi_rx_packet_buffer_t spi_rx_packet_buffer[SPI_PACKET_PRI_N];
static spi_tx_packet_buffer_t spi_tx_packet_buffer[SPI_PACKET_PRI_N];

static spinlock_t spi_buffer_lock=SPIN_LOCK_UNLOCKED;

static int spi_rx_transfer(void);
static int spi_tx_transfer(void);
static void spi_reset(void);
static void spi_reset_request(void);
static void spi_state_timer(void);
static void spi_connected(void);

static int spi_stat_master_req;
static int spi_stat_slave_req;
static int spi_stat_tx;
static int spi_stat_tx_null;
static int spi_stat_rx;
static int spi_stat_rx_null;
static int spi_stat_dma_irq_rx;
static int spi_stat_dma_irq_tx;

/* periodic timer */
#define SPI_TIMER_HZ		(HZ/10)		/* 10 Hz */
static struct timer_list spi_timer;
static void spi_timer_function(unsigned long);
static atomic_t spi_timer_stop = ATOMIC_INIT(0);
DECLARE_COMPLETION(spi_timer_stopped);


#define DMA_CE 			0x80000000
#define DMA_MODE_MEMTOPER	0x04000000
#define DMA_MODE_PERTOMEM	0x00000000
#define DMA_BBUS_BRIDGE_RESET	0x00010000
#define DMA_BBUS_DMA_RESET	0x00040000

#ifndef MIN
#define MIN(a,b)	((a)>(b) ? (b) : (a))
#endif
 
#ifndef MAX
#define MAX(a,b)	((a)>(b) ? (a) : (b))
#endif

#define LOGS
#ifdef LOGS
#define logs(str)		logs_str(str)
#define logs20(val)		logs_20(val)
#define logs8(val)		logs_8(val)
#define logsmem(mem,c)		logs_mem(mem,c)
#ifdef DEBUG
#define	LOGS_MEM_SIZE 		16384
#else
#define	LOGS_MEM_SIZE 		4096
#endif
typedef struct {
	unsigned char 	mem[LOGS_MEM_SIZE];
	int		i;
	spinlock_t		lock;
} logs_t;
static logs_t	logs;

static void logs_init(void)
{
	logs.i=0;
	memset(logs.mem,0,LOGS_MEM_SIZE);
	spin_lock_init(&logs.lock);
}
static void logs_char(unsigned char c)
{
	logs.mem[logs.i]=c;
	if (logs.i>=LOGS_MEM_SIZE-1) 
		logs.i=0;
	else
		logs.i++;
}
static void logs_str(unsigned char *s)
{
	while(*s) {
		logs_char(*s++);
	}
}

#ifdef DEBUG
static void logs_8(unsigned int val)
{
	unsigned char c;
	c=(val>>4)&0xf; if (c<10) logs_char(c+'0'); else logs_char(c-10+'a');
	c=(val>>0)&0xf; if (c<10) logs_char(c+'0'); else logs_char(c-10+'a');
}
static void logs_mem(unsigned char *mem, int count )
{
	int i;
	for (i=0;i<count;i++) logs_8(mem[i]);
}
#else
static void logs_20(unsigned int val)
{
	unsigned char c;
	c=(val>>16)&0xf;if (c<10) logs_char(c+'0'); else logs_char(c-10+'a');
	c=(val>>12)&0xf;if (c<10) logs_char(c+'0'); else logs_char(c-10+'a');
	c=(val>>8)&0xf; if (c<10) logs_char(c+'0'); else logs_char(c-10+'a');
	c=(val>>4)&0xf; if (c<10) logs_char(c+'0'); else logs_char(c-10+'a');
	c=(val>>0)&0xf; if (c<10) logs_char(c+'0'); else logs_char(c-10+'a');
}
#endif

static void logs_copy(logs_t *logscpy)
{
	unsigned long iflags;
	spin_lock_irqsave(&logs.lock,iflags);
	memcpy(logscpy, &logs, sizeof(logs_t));
	spin_unlock_irqrestore(&logs.lock,iflags);
}

#else	/* LOGS not defined */
#define logs(str)		do {} while (0)
#define logs20(val)		do {} while (0)
#define logs8(val)		do {} while (0)
#define logsmem(mem,c)		do {} while (0)
#define logs_init()		do {} while (0)
#define logs_print()		do {} while (0)
#endif 


#ifdef DEBUG
#define dbg(format, arg...) printk(format "\n" , ## arg)
//#define dbg(format, arg...) printk(KERN_DEBUG "%s: " format "\n" , __FILE__ , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif

#define err(format, arg...) printk(KERN_ERR "%s: " format "\n" , __FILE__ , ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: " format "\n" , __FILE__ , ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: " format "\n" , __FILE__ , ## arg)


static int ns_spi_send_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet);
static int ns_spi_receive_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet);
static struct pnet_core_dev ns_spi_pnet_core_dev = {
	.owner = THIS_MODULE,
	.adr = 0,
	.pri_n = SPI_PACKET_PRI_N,
	.port_n = SPI_PACKET_PORT_N,
	.data_n = SPI_PACKET_DATA_LEN,
	.send_packet = ns_spi_send_pnet_core_packet,
	.receive_packet = ns_spi_receive_pnet_core_packet,
};


static void spi_packet_buffer_init(void)
{
	memset(spi_rx_packet_buffer,0,sizeof(spi_rx_packet_buffer_t)*SPI_PACKET_PRI_N);
	memset(spi_tx_packet_buffer,0,sizeof(spi_tx_packet_buffer_t)*SPI_PACKET_PRI_N);
}

static irqreturn_t spi_rx_dma_irq_handler( int irq, void *dev_id )
{
	int rx=0,tx=0;

	/* clear status by writing 1 into status bits */
	writel(0xf8000000,BBUS_DMA_STAT_VA(0,2));

	logs("dma\n");
	dbg("spi_rx_dma_irq_handler called");

	if (spi_state!=SPI_STATE_CONNECTED)
		return IRQ_HANDLED;

	spi_stat_dma_irq_rx++;
	/* process transfered packets */
	spin_lock(&spi_buffer_lock);
	rx=spi_rx_transfer();
	tx=spi_tx_transfer();
	if (rx||tx) pnet_core_device_transfer(&ns_spi_pnet_core_dev);
	spin_unlock(&spi_buffer_lock);

	return IRQ_HANDLED;
}

static irqreturn_t spi_ext_irq_handler( int irq, void *dev_id )
{
	int rx=0,tx=0,sample;

	/* interrupt ext edge-sensitive, falling edge, clear edge ext interrupt */
	writel( NS_SYS_EXT_INT_CTRL_LVEDG|NS_SYS_EXT_INT_CTRL_PLTY|NS_SYS_EXT_INT_CTRL_CLR, io_p2v(NS_SYS_BASE_PA)+SPI_GPIO_EXT_INT_CTRL );
	writel( NS_SYS_EXT_INT_CTRL_LVEDG|NS_SYS_EXT_INT_CTRL_PLTY, io_p2v(NS_SYS_BASE_PA)+SPI_GPIO_EXT_INT_CTRL );
	
	logs("ext\n");
	dbg("spi_ext_irq_handler called");
	
	/* detect reset request */
	ndelay(SPI_MASTER_REQ_MAX_NS);
	if (!ns9xxx_gpio_getpin_raw(SPI_GPIO_MASTER_REQ)) {
		/* sample request line more times to prevent false reset detection */
		for (sample=0;sample<SPI_RESET_REQ_SAMPLES;sample++) {
			ndelay(SPI_MASTER_REQ_MAX_NS);
			if (ns9xxx_gpio_getpin_raw(SPI_GPIO_MASTER_REQ)) goto spi_ext_irq_request;
		}
		logs("reset\n");
		spi_reset_request();
		return IRQ_HANDLED;
	}
spi_ext_irq_request:
	/* a request when in wait connected state implies connected */
	if (spi_state==SPI_STATE_WAIT_CONNECTED)
		spi_connected();
	
	if (spi_state!=SPI_STATE_CONNECTED)
		return IRQ_HANDLED;
		
	spi_stat_master_req++;
	/* process transfered packets */
	atomic_set(&spi_master_request,1);
	spin_lock(&spi_buffer_lock);
	rx=spi_rx_transfer();
	tx=spi_tx_transfer();
	if (rx||tx) pnet_core_device_transfer(&ns_spi_pnet_core_dev);
	spin_unlock(&spi_buffer_lock);

	return IRQ_HANDLED;
}

static int spi_dma_alloc(void)
{
	int ret=0;

	/* alloc RX DMA descriptors */
	spi_rx_buffer_desc_virt = (spi_buffer_desc_t*) dma_alloc_coherent( NULL,
							sizeof(spi_buffer_desc_t)*SPI_RX_BUFFER_DESC_N,
							&spi_rx_buffer_desc_bus, GFP_DMA );
	if( spi_rx_buffer_desc_virt == NULL ) {
		ret=-ENOMEM;
		goto dma_alloc_rx_desc_error;
	}
	dbg(" spi_rx_buffer_desc_virt: %x", (unsigned int) spi_rx_buffer_desc_virt);
	dbg(" spi_rx_buffer_desc_bus: %x", (unsigned int) spi_rx_buffer_desc_bus);

	/* alloc RX DMA buffers  */
	spi_rx_buffer_virt = (spi_packet_t*) dma_alloc_coherent( NULL,
						SPI_RX_BUFFER_DESC_N*SPI_RX_BUFFER_LEN,
						&spi_rx_buffer_bus, GFP_DMA );
	if( spi_rx_buffer_virt == NULL ) {
		ret=-ENOMEM;
		goto dma_alloc_rx_buffer_error;
	}
	/* alloc TX DMA descriptors */
	spi_tx_buffer_desc_virt = (spi_buffer_desc_t*) dma_alloc_coherent( NULL,
							sizeof(spi_buffer_desc_t)*SPI_TX_BUFFER_DESC_N,
							&spi_tx_buffer_desc_bus, GFP_DMA );
	if( spi_tx_buffer_desc_virt == NULL ) {
		ret=-ENOMEM;
		goto dma_alloc_tx_desc_error;
	}
	dbg(" spi_tx_buffer_desc_virt: %x", (unsigned int) spi_tx_buffer_desc_virt);
	dbg(" spi_tx_buffer_desc_bus: %x", (unsigned int) spi_tx_buffer_desc_bus);

	/* alloc TX DMA buffers */
	spi_tx_buffer_virt = (spi_packet_t*) dma_alloc_coherent( NULL,
						SPI_TX_BUFFER_DESC_N*SPI_TX_BUFFER_LEN,
						&spi_tx_buffer_bus, GFP_DMA );
	if( spi_tx_buffer_virt == NULL ) {
		ret=-ENOMEM;
		goto dma_alloc_tx_buffer_error;
	}

	return ret;

dma_alloc_tx_buffer_error:
	dma_free_coherent( NULL, sizeof(spi_buffer_desc_t)*SPI_TX_BUFFER_DESC_N,
		   	spi_tx_buffer_desc_virt, spi_tx_buffer_desc_bus );
dma_alloc_tx_desc_error:
	dma_free_coherent( NULL, SPI_RX_BUFFER_DESC_N*SPI_RX_BUFFER_LEN,
		   	spi_rx_buffer_virt, spi_rx_buffer_bus );
dma_alloc_rx_buffer_error:
	dma_free_coherent( NULL, sizeof(spi_buffer_desc_t)*SPI_RX_BUFFER_DESC_N,
		   	spi_rx_buffer_desc_virt, spi_rx_buffer_desc_bus );
dma_alloc_rx_desc_error:
	return ret;
}

static void spi_dma_free(void)
{
	/* disable and reset DMA operations */
	writel(0,BBUS_DMA_CTRL_VA(0,2));
	writel(0,BBUS_DMA_CTRL_VA(0,3));
	writel(DMA_BBUS_DMA_RESET,BBUS_DMA_CTRL_VA(0,2));
	writel(DMA_BBUS_DMA_RESET,BBUS_DMA_CTRL_VA(0,3));
	
	free_irq(IRQ_BBUS_DMA_3, &spi_dev);

	/* free RX DMA descriptors and buffers */
	dma_free_coherent( NULL, sizeof(spi_buffer_desc_t)*SPI_RX_BUFFER_DESC_N,
		   	spi_rx_buffer_desc_virt, spi_rx_buffer_desc_bus );
	dma_free_coherent( NULL, SPI_RX_BUFFER_DESC_N*SPI_RX_BUFFER_LEN,
		   	spi_rx_buffer_virt, spi_rx_buffer_bus );
	/* free TX DMA descriptors and buffers */
	dma_free_coherent( NULL, sizeof(spi_buffer_desc_t)*SPI_TX_BUFFER_DESC_N,
		   	spi_tx_buffer_desc_virt, spi_tx_buffer_desc_bus );
	dma_free_coherent( NULL, SPI_TX_BUFFER_DESC_N*SPI_TX_BUFFER_LEN,
		   	spi_tx_buffer_virt, spi_tx_buffer_bus );


}

static int spi_dma_init_irq(void)
{
	/* request IRQ for RX DMA
	 * disable all SPI DMA irqs, and reset all status bits by writing 1
	 * SPI DMA irqs are enabled by setting the intr bit in dma descriptors 
	 */
	writel(0xf8000000,BBUS_DMA_STAT_VA(0,2));
	if( request_irq( IRQ_BBUS_DMA_3, spi_rx_dma_irq_handler, IRQF_DISABLED,
		DRV_NAME, &spi_dev ) ) {
		printk( KERN_ERR DRV_NAME ": Could not allocate irq: %d\n", IRQ_BBUS_DMA_3 );
		spi_dma_free();
		return -EIO;
	}
	return 0;
}

static void spi_dma_setup(void)
{
	int i;

	/* disable DMA operations and reset DMA */
	writel(0,BBUS_DMA_CTRL_VA(0,2));
	writel(0,BBUS_DMA_CTRL_VA(0,3));
	writel(DMA_BBUS_DMA_RESET,BBUS_DMA_CTRL_VA(0,2));
	writel(DMA_BBUS_DMA_RESET,BBUS_DMA_CTRL_VA(0,3));

	/* initialize RX buffers */
	memset(spi_rx_buffer_virt,0,sizeof(spi_packet_t)*SPI_RX_BUFFER_DESC_N);
	
	/* initialize RX descriptor rings */
	for (i=0;i<SPI_RX_BUFFER_DESC_N;i++) {
		spi_rx_buffer_desc_virt[i].src=(unsigned int*)((char *)spi_rx_buffer_bus+i*SPI_RX_BUFFER_LEN);
		spi_rx_buffer_desc_virt[i].len=SPI_RX_BUFFER_LEN;
		spi_rx_buffer_desc_virt[i].s.bits.wrap = 0;
		spi_rx_buffer_desc_virt[i].s.bits.intr = 0;
		spi_rx_buffer_desc_virt[i].s.bits.last = 0;
		/* set full bit otherwise RX DMA would leave IDLE state and read first descriptor
		 * subsequent changing of descriptor (set intr flag ..) would not be possible
		 */
		spi_rx_buffer_desc_virt[i].s.bits.full = 1;
	}
	spi_rx_buffer_desc_virt[SPI_RX_BUFFER_DESC_N-1].s.bits.wrap = 1;

	/* setup RX DMA controller */
	writel(spi_rx_buffer_desc_bus,BBUS_DMA_DESCR_VA(0,2));
	writel(DMA_CE|DMA_MODE_PERTOMEM,BBUS_DMA_CTRL_VA(0,2));
	ns9xx0_spi_start_rx_dma(&spi_dev);
	
	/* initialize TX buffers */
	memset(spi_tx_buffer_virt,0,sizeof(spi_packet_t)*SPI_TX_BUFFER_DESC_N);


	/* initialize TX descriptor rings */
	for (i=0;i<SPI_TX_BUFFER_DESC_N;i++) {
		spi_tx_buffer_desc_virt[i].src=(unsigned int*)((char *)spi_tx_buffer_bus+i*SPI_TX_BUFFER_LEN);
		spi_tx_buffer_desc_virt[i].len=SPI_TX_BUFFER_LEN;
		spi_tx_buffer_desc_virt[i].s.bits.wrap = 0;
		spi_tx_buffer_desc_virt[i].s.bits.intr = 0;
		spi_tx_buffer_desc_virt[i].s.bits.last = 0;
		spi_tx_buffer_desc_virt[i].s.bits.full = 0;
	}
	spi_tx_buffer_desc_virt[SPI_TX_BUFFER_DESC_N-1].s.bits.wrap = 1;

	spi_rxtx_head=0;
	spi_rxtx_tail=0;
	spi_rxtx_len=0;

	spi_txrdy_master=0;
	atomic_set(&spi_master_request,0);

	/* setup TX DMA controller */
	writel(spi_tx_buffer_desc_bus,BBUS_DMA_DESCR_VA(0,3));
	writel(DMA_CE|DMA_MODE_MEMTOPER,BBUS_DMA_CTRL_VA(0,3));
	ns9xx0_spi_start_tx_dma(&spi_dev);
}

static int spi_dma_init(void)
{
	int ret=0;
	ret=spi_dma_alloc();
	if (ret) return ret;
	ret=spi_dma_init_irq();
	if (ret) return ret;
	spi_dma_setup();
	return 0;
}


static int spi_tx_packet_buffers_empty(void)
{
	int i;
	/* only used in ns_spi_send_pnet_core_packet, no locking necessary */
	for (i=0;i<SPI_PACKET_PRI_N;i++) {
		if (spi_tx_packet_buffer[i].len && !(spi_pri_stop_master&(1<<i))) {
			return 0;
		}
	}
	return 1;
}
static void spi_rx_remove_packet(unsigned int pri)
{
	spi_rx_packet_buffer_t *pb;
	int rx=0,tx=0;

	/* only used in ns_spi_receive_pnet_core_packet, no locking necessary */
	pb=&spi_rx_packet_buffer[pri];
	pb->tail=(pb->tail+1)%SPI_RX_PACKET_BUFFER_LEN;
	pb->len--;

	if (pb->len<SPI_RX_PACKET_BUFFER_START_LEN && spi_pri_stop_slave&(1<<pri)) {
		spi_pri_stop_slave&=~(1<<pri);
		/* force to transmit at least one packet */
		spi_force_transfer=1;
		rx=spi_rx_transfer();
		tx=spi_tx_transfer();
		if (rx||tx) pnet_core_device_transfer(&ns_spi_pnet_core_dev);
	}
}

static int ns_spi_send_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet)
{
	unsigned long iflags;
	spi_packet_t *spi_packet;
	int tx_buffers_empty; 
	spi_tx_packet_buffer_t *pb;
	int rx=0,tx=0;

	/* no need to send if no data */
	if(packet->len==0) return 0;

	/* insert packet */
	pb=&spi_tx_packet_buffer[packet->pri];
	spin_lock_irqsave(&spi_buffer_lock,iflags);

	if (spi_state!=SPI_STATE_CONNECTED) {
		spin_unlock_irqrestore(&spi_buffer_lock,iflags);
		return -1;
	}	

	if(pb->len<SPI_TX_PACKET_BUFFER_LEN) {
		tx_buffers_empty=spi_tx_packet_buffers_empty();
		spi_packet=&pb->buffer[pb->head];
		spi_packet->h.bits.pri=packet->pri;
		spi_packet->h.bits.port=packet->port;
		spi_packet->h.bits.len=packet->len;
		memcpy(spi_packet->data,packet->data,packet->len);
		pb->head=(pb->head+1)%SPI_TX_PACKET_BUFFER_LEN;
		pb->len++;
		dbg("spi_send_packet, spi_rxtx_len: %d",spi_rxtx_len);
		if (spi_rxtx_len==0 && tx_buffers_empty) {
			/* initiate transfer */
			rx=spi_rx_transfer();
			tx=spi_tx_transfer();
			if (rx||tx) pnet_core_device_transfer(&ns_spi_pnet_core_dev);
		}
		spin_unlock_irqrestore(&spi_buffer_lock,iflags);
		return 0;
	}
	spin_unlock_irqrestore(&spi_buffer_lock,iflags);

	return -1;	

}

static int ns_spi_receive_pnet_core_packet(struct pnet_core_dev *dev, struct pnet_core_packet *packet)
{

	spi_packet_t *spi_packet;
	unsigned long iflags;
	unsigned int pri;
	spi_rx_packet_buffer_t *pb;
	int ret;

	spin_lock_irqsave(&spi_buffer_lock,iflags);

	if (spi_state!=SPI_STATE_CONNECTED) {
		spin_unlock_irqrestore(&spi_buffer_lock,iflags);
		return -ENODATA;
	}	
	pri=0;
	while (pri<SPI_PACKET_PRI_N) {
		pb=&spi_rx_packet_buffer[pri];
		if(pb->len) {
			spi_packet=&pb->buffer[pb->tail];
			packet->pri=spi_packet->h.bits.pri;
			packet->port=spi_packet->h.bits.port;
			packet->len=spi_packet->h.bits.len;
			/* verify that pnet core will take the packet */
			ret=pnet_core_packet_receive_ready(dev, packet);
			switch (ret) {
				case 0:
					/* copy packet data */
					memcpy(packet->data,spi_packet->data, packet->len);
					/* remove packet from buffer */
					spi_rx_remove_packet(pri);
					/* return with packet */
					spin_unlock_irqrestore(&spi_buffer_lock,iflags);
					return 0;
				case -ENOPROTOOPT:
					/* remove packet from buffer, proceed with same pri in case
					   there are more packets to remove
					 */
					spi_rx_remove_packet(pri);
					pri++;
					break;
				case -EBUSY:
				default:
					/* leave packet in buffer, proceed with next pri */
					pri++;
					break;
			}
		} else {
			/* no packet in buffer, proceed with next pri */
			pri++;
		}
	}
	/* no packet to return */
	spin_unlock_irqrestore(&spi_buffer_lock,iflags);
	return -ENODATA;
}


#ifdef DEBUG
static void spi_print_packet(spi_packet_t *packet)
{
        int i;
        printk("pri: %x ",packet->h.bits.pri);
        printk("port: %x ",packet->h.bits.port);
        printk("len: %x ",packet->h.bits.len);
        printk("txrdy: %x ",packet->h.bits.txrdy);
		printk("stop: %x ",packet->h.bits.stop);
        printk("data: ");
        for (i=0;i<SPI_PACKET_DATA_LEN;i++)
                printk("%x ",packet->data[i]);
        printk("\n");
}
#endif

#undef SPI_RX_CHECK_BUFFER_OVERFLOW

static int spi_rx_transfer(void)
{

	spi_rx_packet_buffer_t *pb;
	int rx=0;
	#ifdef SPI_RX_CHECK_BUFFER_OVERFLOW
	int pri=0;
	#endif
	
	logs("RX:");
	do {
		/* check that no rx_packet_buffer is full
		 * with stop bit not needed anymore
		 */
		#ifdef SPI_RX_CHECK_BUFFER_OVERFLOW
		for (pri=0;pri<SPI_PACKET_PRI_N;pri++) {
			if (spi_rx_packet_buffer[pri].len>=SPI_PACKET_BUFFER_LEN) {
				dbg("spi_rx_transfer: buffer full\n");
				logs("\n");
				return rx;
			}
		}
		#endif
		if (spi_rxtx_len && !spi_tx_buffer_desc_virt[spi_rxtx_tail].s.bits.full && 
		    spi_rx_buffer_desc_virt[spi_rxtx_tail].s.bits.full) {
			#ifdef DEBUG
			printk("RX: "); spi_print_packet(&spi_rx_buffer_virt[spi_rxtx_tail]);
			#endif
			/* if packet contains data insert it into packet_buffer */
			if (spi_rx_buffer_virt[spi_rxtx_tail].h.bits.len>0) {
				pb=&spi_rx_packet_buffer[spi_rx_buffer_virt[spi_rxtx_tail].h.bits.pri];
				BUG_ON(pb->len>=SPI_RX_PACKET_BUFFER_LEN);
				memcpy(&pb->buffer[pb->head],&spi_rx_buffer_virt[spi_rxtx_tail],SPI_PACKET_LEN);
				pb->head=(pb->head+1)%SPI_RX_PACKET_BUFFER_LEN;
				pb->len++;
				if (pb->len>=SPI_RX_PACKET_BUFFER_STOP_LEN)
					spi_pri_stop_slave|=1<<spi_rx_buffer_virt[spi_rxtx_tail].h.bits.pri;
				spi_stat_rx++;
				rx++;
				#ifdef DEBUG
				logs("rx:");logsmem((unsigned char*)&spi_rx_buffer_virt[spi_rxtx_tail],SPI_PACKET_LEN);
				#else
				logs("rx:");logs20(spi_rx_buffer_virt[spi_rxtx_tail].h.header);
				#endif				
			}
			else
			{			
				spi_stat_rx_null++;	
				#ifdef DEBUG
				logs("r0:");logsmem((unsigned char*)&spi_rx_buffer_virt[spi_rxtx_tail],SPI_PACKET_LEN);
				#else
				logs("r0:");logs20(spi_rx_buffer_virt[spi_rxtx_tail].h.header);
				#endif
			}
			/* update txrdy from master */
			spi_txrdy_master=spi_rx_buffer_virt[spi_rxtx_tail].h.bits.txrdy;
			/* update pri_stop from master */ 
			#ifdef DEBUG
			if (spi_pri_stop_master != spi_rx_buffer_virt[spi_rxtx_tail].h.bits.stop)
				printk("stop:%x\n", spi_rx_buffer_virt[spi_rxtx_tail].h.bits.stop);
			#endif 
			spi_pri_stop_master=spi_rx_buffer_virt[spi_rxtx_tail].h.bits.stop;

			/* remove rx packet from DMA descriptor ring */
			spi_rx_buffer_desc_virt[spi_rxtx_tail].s.bits.intr = 0;
			spi_rx_buffer_desc_virt[spi_rxtx_tail].s.bits.last = 0;
			/* dont clear full bit
			 * spi_rx_buffer_desc_virt[spi_rxtx_tail].s.bits.full = 0;
			 * modified full bit ([1] page 430)
			 * writel(DMA_CE|DMA_MODE_PERTOMEM,NS_BBUS_DMA_CTRL(0,2));
			 */
			spi_rxtx_tail=(spi_rxtx_tail+1)%SPI_BUFFER_DESC_N;
			spi_rxtx_len--;
		}
		else {
			logs("\n");
			return rx;
		}
	}while(1);
}


static void spi_slave_request(void)
{
	logs("req\n");
	dbg("spi_slave_request");

	/* generate falling edge 
	 * interrupts are always locked when spi_slave_request ist called
	 * shortest REQUEST impulse = 900ns, netsilicon@177MHz, SPI_SLAVE_REQ_NS=0
	 */
	ns9xxx_gpio_setpin_raw(SPI_GPIO_SLAVE_REQ, 0);
	#if (SPI_SLAVE_REQ_NS>0)
	ndelay(SPI_SLAVE_REQ_NS);
	#endif
	ns9xxx_gpio_setpin_raw(SPI_GPIO_SLAVE_REQ, 1);
	#if (SPI_SLAVE_REQ_NS>0)
	ndelay(SPI_SLAVE_REQ_NS);
	#endif
	spi_stat_slave_req++;
}

static int spi_tx_transfer(void)
{

	spi_packet_t *tx_packet=NULL;
	spi_tx_packet_buffer_t *pb;
	int spi_rxtx_last_sent=-1;
	int pkt_send_avail_n=0;
	int pkt_send_n=0;
	int pkt_recv_n=0;
	int pkt_null_n=0;
	int pri;
	int tx=0;
	int i;

	logs("TX:");
	if (spi_rxtx_len==0) {
		if (atomic_read(&spi_master_request)) {
			/* no problem if interrupt sets spi_master_request between atomic_read and atomic_set */
			atomic_set(&spi_master_request,0);
			spi_txrdy_master=MAX(spi_txrdy_master,1);
		}
		/* calculate how many packets to send */
		for(pri=0;pri<SPI_PACKET_PRI_N;pri++)
			if (!(spi_pri_stop_master&(1<<pri)))
				pkt_send_avail_n+=spi_tx_packet_buffer[pri].len;

		pkt_send_n=MIN(pkt_send_avail_n,SPI_BUFFER_DESC_N);
		/* calculate how many RX DMA descriptors to prepare */
		pkt_recv_n=MAX(pkt_send_n,spi_txrdy_master);
		/* if spi_force_transfer is set, at least transmit one packet. */
		pkt_recv_n=MAX(pkt_recv_n,spi_force_transfer);
		spi_force_transfer=0;
		pkt_recv_n=MIN(pkt_recv_n,SPI_BUFFER_DESC_N);	

		if (pkt_recv_n>0) {

			/* calculate how many null-packets to insert */
			pkt_null_n=MAX(pkt_recv_n-pkt_send_n,0);
			/* insert pkt_send_n packets in TX DMA descriptor ring, highest priority first */
			for(i=0;i<pkt_send_n;i++) {
				/* get packet for tx with highest priority */
				for (pri=0;pri<SPI_PACKET_PRI_N;pri++) {
					pb=(&spi_tx_packet_buffer[pri]);
					if (pb->len && !(spi_pri_stop_master&(1<<pri))) {
						tx_packet=&pb->buffer[pb->tail];
						break;
					}
					#ifdef DEBUG
					else {
						if (pb->len) printk("txno:%d\n",pri);
					}
					#endif
					
				}
				/* insert packet in DMA descriptor ring and remove from packet buffer */
				memcpy(&spi_tx_buffer_virt[spi_rxtx_head],tx_packet,SPI_PACKET_LEN);
				pb->tail=(pb->tail+1)%SPI_TX_PACKET_BUFFER_LEN;
				pb->len--;
				/* update txrdy field */
				spi_tx_buffer_virt[spi_rxtx_head].h.bits.txrdy=pkt_recv_n-i-1;
				/* update stop field */
				spi_tx_buffer_virt[spi_rxtx_head].h.bits.stop=spi_pri_stop_slave;
				#ifdef DEBUG
				printk("TX: "); spi_print_packet(&spi_tx_buffer_virt[spi_rxtx_head]);
				logs("tx:");logsmem((unsigned char*)&spi_tx_buffer_virt[spi_rxtx_head],SPI_PACKET_LEN);
				#else
				logs("tx:");logs20(spi_tx_buffer_virt[spi_rxtx_head].h.header);
				#endif
				/* insert packet in DMA descriptor ring */
				spi_rx_buffer_desc_virt[spi_rxtx_head].s.bits.intr = 0;
				spi_rx_buffer_desc_virt[spi_rxtx_head].s.bits.last = 0;
				spi_rx_buffer_desc_virt[spi_rxtx_head].s.bits.full = 0;

				spi_tx_buffer_desc_virt[spi_rxtx_head].s.bits.full = 1;
				spi_tx_buffer_desc_virt[spi_rxtx_head].s.bits.last = 0;
				spi_rxtx_last_sent=spi_rxtx_head;
				spi_rxtx_head=(spi_rxtx_head+1)%SPI_BUFFER_DESC_N;
				spi_rxtx_len++;
				spi_stat_tx++;
				tx++;
			}
			/* insert pkt_null_n null-packets in TX DMA descriptor ring */
			for(i=0;i<pkt_null_n;i++) {
				memset(&spi_tx_buffer_virt[spi_rxtx_head],0,SPI_PACKET_LEN);
				/* update txrdy field */
				spi_tx_buffer_virt[spi_rxtx_head].h.bits.txrdy=pkt_null_n-i-1;
				/* update stop field */
				spi_tx_buffer_virt[spi_rxtx_head].h.bits.stop=spi_pri_stop_slave;
				#ifdef DEBUG
				printk("TX: "); spi_print_packet(&spi_tx_buffer_virt[spi_rxtx_head]);
				logs("t0:");logsmem((unsigned char*)&spi_tx_buffer_virt[spi_rxtx_head],SPI_PACKET_LEN);
				#else
				logs("t0:");logs20(spi_tx_buffer_virt[spi_rxtx_head].h.header);
				#endif
				/* insert packet in DMA descriptor ring */
				spi_rx_buffer_desc_virt[spi_rxtx_head].s.bits.intr = 0;
				spi_rx_buffer_desc_virt[spi_rxtx_head].s.bits.last = 0;
				spi_rx_buffer_desc_virt[spi_rxtx_head].s.bits.full = 0;

				spi_tx_buffer_desc_virt[spi_rxtx_head].s.bits.full = 1;
				spi_tx_buffer_desc_virt[spi_rxtx_head].s.bits.last = 0;
				spi_rxtx_last_sent=spi_rxtx_head;
				spi_rxtx_head=(spi_rxtx_head+1)%SPI_BUFFER_DESC_N;
				spi_rxtx_len++;
				spi_stat_tx_null++;
			}
			/* set interrupt and last bit for last inserted packet */
			spi_rx_buffer_desc_virt[spi_rxtx_last_sent].s.bits.intr = 1;
			spi_rx_buffer_desc_virt[spi_rxtx_last_sent].s.bits.last = 1;
			spi_tx_buffer_desc_virt[spi_rxtx_last_sent].s.bits.last = 1;
			/* modified full bit ([1] page 430) */
			writel(DMA_CE|DMA_MODE_MEMTOPER,BBUS_DMA_CTRL_VA(0,3));
			writel(DMA_CE|DMA_MODE_PERTOMEM,BBUS_DMA_CTRL_VA(0,2));
			/* trigger external interrupt on spi master */
			logs("\n");
			spi_slave_request();
			return tx;
		}
	}
	logs("\n");
	return tx;							

}

#ifdef CONFIG_PROC_FS
static int spi_stat_show(struct seq_file *m, void *v)
{
	seq_printf(m,"spi_stat_master_req: %d\n",spi_stat_master_req);
	seq_printf(m,"spi_stat_slave_req: %d\n",spi_stat_slave_req);
	seq_printf(m,"spi_stat_tx: %d\n",spi_stat_tx);
	seq_printf(m,"spi_stat_tx_null: %d\n",spi_stat_tx_null);
	seq_printf(m,"spi_stat_rx: %d\n",spi_stat_rx);
	seq_printf(m,"spi_stat_rx_null: %d\n",spi_stat_rx_null);
	seq_printf(m,"spi_stat_dma_irq_rx: %d\n",spi_stat_dma_irq_rx);
	seq_printf(m,"spi_stat_dma_irq_tx: %d\n",spi_stat_dma_irq_tx);
	seq_printf(m,"spi_pri_stop_slave: %d\n",spi_pri_stop_slave);
	seq_printf(m,"spi_pri_stop_master: %d\n",spi_pri_stop_master);
	return 0;
}
static int spi_stat_open( struct inode *inode, struct file *file)
{
	return single_open(file, spi_stat_show, NULL);
}

static struct file_operations spi_stat_fops = {
	.owner = THIS_MODULE,
	.open = spi_stat_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static void spi_packet_show(struct seq_file *m,spi_packet_t *packet)
{
	int i;
	seq_printf(m,"pri: %x ",packet->h.bits.pri);
	seq_printf(m,"port: %x ",packet->h.bits.port);
	seq_printf(m,"len: %x ",packet->h.bits.len);
	seq_printf(m,"txrdy: %x ",packet->h.bits.txrdy);
	seq_printf(m,"stop: %x ",packet->h.bits.stop);
	seq_printf(m,"data: ");
	for (i=0;i<SPI_PACKET_DATA_LEN;i++)
		seq_printf(m,"%x ",packet->data[i]);
	seq_printf(m,"\n");
}
static void spi_rx_buffer_show(struct seq_file *m, spi_rx_packet_buffer_t *pb)
{
	int i;
	for(i=0;i<pb->len;i++) {
		spi_packet_show(m,&pb->buffer[(pb->tail+i)%SPI_RX_PACKET_BUFFER_LEN]);
	}
}
static void spi_tx_buffer_show(struct seq_file *m, spi_tx_packet_buffer_t *pb)
{
	int i;
	for(i=0;i<pb->len;i++) {
		spi_packet_show(m,&pb->buffer[(pb->tail+i)%SPI_TX_PACKET_BUFFER_LEN]);
	}
}
static int spi_buffers_show(struct seq_file *m, void *v)
{
	int pri;

	seq_printf(m,"Transmitbuffers:\n");
	for (pri=0;pri<SPI_PACKET_PRI_N;pri++) {
		spi_tx_buffer_show(m,&spi_tx_packet_buffer[pri]);
	}
	seq_printf(m,"Receivebuffers:\n");
	for (pri=0;pri<SPI_PACKET_PRI_N;pri++) {
		spi_rx_buffer_show(m,&spi_rx_packet_buffer[pri]);
	}
	return 0;
}
static int spi_buffers_open( struct inode *inode, struct file *file)
{
	return single_open(file, spi_buffers_show, NULL);
}

static struct file_operations spi_buffers_fops = {
	.owner = THIS_MODULE,
	.open = spi_buffers_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void spi_dmainfo_desc_show(struct seq_file *m, spi_buffer_desc_t *desc )
{	

	seq_printf(m,"dma desc: %p len: %x ", desc->src, desc->len);
	seq_printf(m,"reg: %x ",desc->s.reg);
	if (desc->s.bits.full) seq_printf(m,"full ");
	if (desc->s.bits.last) seq_printf(m,"last ");
	if (desc->s.bits.intr) seq_printf(m,"intr ");
	if (desc->s.bits.wrap) seq_printf(m,"wrap ");
	seq_printf(m,"\n");

}
static int spi_dmainfo_show(struct seq_file *m, void *v)
{	
	int i;

	seq_printf(m,"head: %d\n",spi_rxtx_head);
	seq_printf(m,"tail: %d\n",spi_rxtx_tail);
	seq_printf(m,"len: %d\n",spi_rxtx_len);

	seq_printf(m,"TX: NS_BBUS_DMA_CTRL(0,3): %x\n",readl(BBUS_DMA_CTRL_VA(0,3)));
	seq_printf(m,"    NS_BBUS_DMA_STAT(0,3): %x\n",readl(BBUS_DMA_STAT_VA(0,3)));
	seq_printf(m,"RX: NS_BBUS_DMA_CTRL(0,2): %x\n",readl(BBUS_DMA_CTRL_VA(0,2)));
	seq_printf(m,"    NS_BBUS_DMA_STAT(0,2): %x\n",readl(BBUS_DMA_STAT_VA(0,2)));

	seq_printf(m,"DMA Transmitbuffers:\n");
	for (i=0;i<SPI_TX_BUFFER_DESC_N;i++) {
		spi_dmainfo_desc_show(m,&spi_tx_buffer_desc_virt[i]);
		spi_packet_show(m,&spi_tx_buffer_virt[i]);
	}
	seq_printf(m,"DMA Receivebuffers:\n");
	for (i=0;i<SPI_RX_BUFFER_DESC_N;i++) {
		spi_dmainfo_desc_show(m,&spi_rx_buffer_desc_virt[i]);
		spi_packet_show(m,&spi_rx_buffer_virt[i]);
	}
	return 0;
}
static int spi_dmainfo_open( struct inode *inode, struct file *file)
{
	return single_open(file, spi_dmainfo_show, NULL);
}

static struct file_operations spi_dmainfo_fops = {
	.owner = THIS_MODULE,
	.open = spi_dmainfo_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static int spi_logs_show(struct seq_file *m, void *v)
{
#ifdef LOGS
	logs_t *logs_bak;
	int i;

	logs_bak=kmalloc(sizeof(logs_t), GFP_KERNEL);
	if (logs_bak==NULL)
		return -ENOMEM;
	logs_copy(logs_bak);
	if (logs_bak->mem[LOGS_MEM_SIZE-1]) {
		/* logs wrapped, print i to end of mem */
		for (i=logs_bak->i;i<LOGS_MEM_SIZE;i++)
			seq_putc(m,logs_bak->mem[i]);	
	}
	for (i=0;i<logs_bak->i;i++)
		seq_putc(m,logs_bak->mem[i]);	
	seq_printf(m,"\n");
	kfree(logs_bak);
#else
	seq_printf(m,"disabled\n");
#endif		
	return 0;
}
static int spi_logs_open( struct inode *inode, struct file *file)
{
	return single_open(file, spi_logs_show, NULL);
}

static struct file_operations spi_logs_fops = {
	.owner = THIS_MODULE,
	.open = spi_logs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static struct proc_dir_entry *spi_proc_dir,*spi_proc_stats,*spi_proc_dmainfo,*spi_proc_buffers,*spi_proc_logs;
static int spi_proc_create(void)
{
	spi_proc_dir=proc_mkdir("driver/"DRV_NAME,NULL);
	spi_proc_stats=create_proc_entry("stats",S_IRUGO,spi_proc_dir);
	spi_proc_dmainfo=create_proc_entry("dmainfo",S_IRUGO,spi_proc_dir);
	spi_proc_buffers=create_proc_entry("buffers",S_IRUGO,spi_proc_dir);
	spi_proc_logs=create_proc_entry("logs",S_IRUGO,spi_proc_dir);
	if (spi_proc_stats) {
		spi_proc_stats->proc_fops = &spi_stat_fops;
	}
	if (spi_proc_dmainfo) {
		spi_proc_dmainfo->proc_fops = &spi_dmainfo_fops;
	}
	if (spi_proc_buffers) {
		spi_proc_buffers->proc_fops = &spi_buffers_fops;
	}
	if (spi_proc_logs) {
		spi_proc_logs->proc_fops = &spi_logs_fops;
	}
	return 0;
}
static void spi_proc_remove(void)
{
	if (spi_proc_stats) remove_proc_entry("stats", spi_proc_dir);
	if (spi_proc_dmainfo) remove_proc_entry("dmainfo", spi_proc_dir);
	if (spi_proc_buffers) remove_proc_entry("buffers", spi_proc_dir);
	if (spi_proc_logs) remove_proc_entry("logs", spi_proc_dir);
	if (spi_proc_dir) remove_proc_entry("driver/"DRV_NAME,NULL);
}
#endif	

static void spi_reset(void)
{
	/* reinit spi */
	ns9xx0_spi_port_setup(&spi_dev);

	/* setup dma and descriptor rings */
	spi_dma_setup();

	/* init packet buffers */
	spi_packet_buffer_init();

	/* clear priority stop bits */
	spi_pri_stop_master=0;
	spi_pri_stop_slave=0;
	spi_force_transfer=0;

	/* clear EXT and DMA interrupts
	 * interrupt ext edge-sensitive, falling edge, clear edge ext interrupt
	 */
	writel( NS_SYS_EXT_INT_CTRL_LVEDG|NS_SYS_EXT_INT_CTRL_PLTY|NS_SYS_EXT_INT_CTRL_CLR, io_p2v(NS_SYS_BASE_PA)+SPI_GPIO_EXT_INT_CTRL );
	writel( NS_SYS_EXT_INT_CTRL_LVEDG|NS_SYS_EXT_INT_CTRL_PLTY, io_p2v(NS_SYS_BASE_PA)+SPI_GPIO_EXT_INT_CTRL );
	/* clear status by writing 1 into status bits */
	writel(0xf8000000,BBUS_DMA_STAT_VA(0,2));

}

static void spi_reset_request(void)
{
	unsigned long iflags;
	dbg("spi_reset_requested called");

	spin_lock_irqsave(&spi_state_lock,iflags);

	if (spi_state > SPI_STATE_RESET_REQUEST)
		spi_state=SPI_STATE_INIT;

	spin_unlock_irqrestore(&spi_state_lock,iflags);
}

static void spi_state_timer(void)
{
	static int reset_request_t=0;
	unsigned long iflags;

	spin_lock_irqsave(&spi_state_lock,iflags);
	switch (spi_state) {
		
		case SPI_STATE_INIT:
			dbg("spi_state=SPI_STATE_INIT");
			ns9xxx_gpio_setpin_raw(SPI_GPIO_SLAVE_REQ, 0);
			reset_request_t=SPI_WAIT_RESET_REQUEST_T;
			pnet_core_device_state(&ns_spi_pnet_core_dev, PNET_CORE_DEVICE_DISCONNECTED);
			spi_state=SPI_STATE_RESET_REQUEST;
			break;

		case SPI_STATE_RESET_REQUEST:
			dbg("spi_state=SPI_STATE_RESET_REQUEST");
			if (reset_request_t)
				reset_request_t--;
			else
				spi_state=SPI_STATE_RESET;
			break;

		case SPI_STATE_RESET:
			dbg("spi_state=SPI_STATE_RESET");
			spi_reset();
			ns9xxx_gpio_setpin_raw(SPI_GPIO_SLAVE_REQ, 1);
			spi_state=SPI_STATE_WAIT_CONNECTED;
			break;

		case SPI_STATE_WAIT_CONNECTED:
			dbg("spi_state=SPI_STATE_WAIT_CONNECTED");
			if (ns9xxx_gpio_getpin_raw(SPI_GPIO_MASTER_REQ))
				spi_connected();
			break;
		case SPI_STATE_CONNECTED:
			/* dbg("spi_state=SPI_STATE_CONNECTED"); */
			break;
	}
	spin_unlock_irqrestore(&spi_state_lock,iflags);
}
static void spi_connected(void)
{
	dbg("spi_connected called");	
	spi_state=SPI_STATE_CONNECTED;
	pnet_core_device_state(&ns_spi_pnet_core_dev, PNET_CORE_DEVICE_CONNECTED);
}

static void spi_timer_function(unsigned long arg)
{

	/* dbg("spi_timer_function called"); */

	if (atomic_read(&spi_timer_stop)) {
		complete(&spi_timer_stopped);
		return;
	} else {
		spi_timer.expires = jiffies + SPI_TIMER_HZ; 
		add_timer(&spi_timer);	
	}
	spi_state_timer();
}

static int __init spi_init(void)
{
	unsigned long iflags;

	dbg("spi_init called");

	/* initialize spi */
	ns9xx0_spi_init_port(&spi_dev,SPI_PORTA);

	if (ns9xx0_spi_request_port(&spi_dev)!=0) {
		printk( KERN_ERR DRV_NAME ": Selected SPI port %d busy\n", spi_dev.port );
		return -EBUSY;
	}
	if (ns9xx0_spi_set_master_slave(&spi_dev,SPI_SLAVE)!=0) {
		printk( KERN_ERR DRV_NAME ": Error configuring SPI as slave\n");
		return -EIO;
	}
	if (ns9xx0_spi_set_mode(&spi_dev,SPI_MODE_1)!=0) {
		printk( KERN_ERR DRV_NAME ": Error configuring SPI mode %d\n", SPI_MODE_1);
		return -EIO;
	}
	if (ns9xx0_spi_set_bitorder(&spi_dev,SPI_MSBF)!=0) {
		printk( KERN_ERR DRV_NAME ": Error configuring SPI bitorder %d\n",SPI_MSBF);
		return -EIO;
	}
	if (ns9xx0_spi_port_setup(&spi_dev)!=0) {
		printk( KERN_ERR DRV_NAME ": Error SPI port setup\n");
		return -EIO;
	}

	#ifdef DEBUG
	ns9xx0_spi_dump_regs(&spi_dev);
	#endif

	/* clear statistics and init packet buffers and logs before interrupts get enabled */
	spi_stat_master_req=0;
	spi_stat_slave_req=0;
	spi_stat_tx=0;
	spi_stat_tx_null=0;
	spi_stat_rx=0;
	spi_stat_rx_null=0;
	spi_stat_dma_irq_rx=0;
	spi_stat_dma_irq_tx=0;
	logs_init();
	spi_packet_buffer_init();

	/* clear priority stop bits */
	spi_pri_stop_master=0;
	spi_pri_stop_slave=0;
	spi_force_transfer=0;

	/* lock irqs till interrupts and dma buffers are initialized for safety */
	/* TODO: shorten lock to minimum */
	spin_lock_irqsave(&spi_buffer_lock,iflags);

	if (spi_dma_init())
	{
		ns9xx0_spi_release_port(&spi_dev);
		spin_unlock_irqrestore(&spi_buffer_lock,iflags);
		return -ENOMEM;
	}


	/* use SPI_GPIO_SLAVE_REQ for output */
	ns9xxx_request_gpio( SPI_GPIO_SLAVE_REQ, DRV_NAME);
	/* ctrl after reset = 0; set 1 before setting as output to prevent spike */
	ns9xxx_gpio_setpin_raw(SPI_GPIO_SLAVE_REQ, 1);
	ns9xxx_gpio_cfgpin( SPI_GPIO_SLAVE_REQ, GPIO_CFG_OUTPUT);

	ns9xxx_request_gpio( SPI_GPIO_MASTER_REQ, DRV_NAME);
	ns9xxx_gpio_cfgpin( SPI_GPIO_MASTER_REQ, GPIO_CFG_FUNC_1 );

	/* interrupt ext edge-sensitive, falling edge, clear edge ext interrupt */
	writel( NS_SYS_EXT_INT_CTRL_LVEDG|NS_SYS_EXT_INT_CTRL_PLTY|NS_SYS_EXT_INT_CTRL_CLR, io_p2v(NS_SYS_BASE_PA)+SPI_GPIO_EXT_INT_CTRL );
	writel( NS_SYS_EXT_INT_CTRL_LVEDG|NS_SYS_EXT_INT_CTRL_PLTY, io_p2v(NS_SYS_BASE_PA)+SPI_GPIO_EXT_INT_CTRL );

	if( request_irq( SPI_GPIO_EXT_INT, spi_ext_irq_handler, IRQF_DISABLED, DRV_NAME, &spi_dev ) ) {
		printk( KERN_ERR DRV_NAME ": Could not allocate irq: %d\n", SPI_GPIO_EXT_INT );
		spi_dma_free();
		spin_unlock_irqrestore(&spi_buffer_lock,iflags);
		ns9xxx_release_gpio(SPI_GPIO_SLAVE_REQ);
		ns9xxx_release_gpio(SPI_GPIO_MASTER_REQ);
		ns9xx0_spi_release_port(&spi_dev);
		return -EIO;
	}

	if (pnet_core_device_register(&ns_spi_pnet_core_dev)!=0)
	{
		free_irq(SPI_GPIO_EXT_INT, &spi_dev);
		spi_dma_free();
		ns9xxx_release_gpio(SPI_GPIO_SLAVE_REQ);
		ns9xxx_release_gpio(SPI_GPIO_MASTER_REQ);
		ns9xx0_spi_release_port(&spi_dev);
		return -EIO;
	}

	spi_state=SPI_STATE_INIT;

	/* init periodic timer */
	init_timer(&spi_timer);
	spi_timer.function=spi_timer_function;
	spi_timer.data=0;
	spi_timer.expires = jiffies + SPI_TIMER_HZ; 
	add_timer(&spi_timer);

	spin_unlock_irqrestore(&spi_buffer_lock,iflags);

	#ifdef CONFIG_PROC_FS
		spi_proc_create();
	#endif

	return 0;
}

static void __exit spi_exit(void)
{

	dbg("spi_exit called");

	/* reset request, REQUEST as input = low */
	spi_state=SPI_STATE_INIT;
	ns9xxx_gpio_cfgpin( SPI_GPIO_SLAVE_REQ, GPIO_CFG_INPUT);

	#ifdef CONFIG_PROC_FS
		spi_proc_remove();
	#endif

	/* remove periodic timer */
	atomic_set(&spi_timer_stop,1);
	wait_for_completion(&spi_timer_stopped);
	del_timer_sync(&spi_timer);

	pnet_core_device_unregister(&ns_spi_pnet_core_dev);
	free_irq(SPI_GPIO_EXT_INT, &spi_dev);
	ns9xxx_release_gpio(SPI_GPIO_SLAVE_REQ);
	ns9xxx_release_gpio(SPI_GPIO_MASTER_REQ);
	spi_dma_free();
	ns9xx0_spi_release_port(&spi_dev);

}

module_init( spi_init );
module_exit( spi_exit );

MODULE_AUTHOR("Andreas Schrattenecker");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Netsilicon SPI Packet Driver");
