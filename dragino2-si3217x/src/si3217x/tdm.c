/*
 * DRAGINO v2 si3217x FXS Daughter Board TDM driver - based on tdm.c from David Rowe
 *
 * Copyright (C) 2013-2014 Village Telco Ltd.
 * Copyright (C) 2013-2014 Dimitar Penev <dpn at switchvoice dot com>
 * Copyright (C) 2014-2015 Vittorio Gambaletta <openwrt@vittgam.net>
 *
 * This program is free software, distributed under the terms of
 * the GNU General Public License Version 2 as published by the
 * Free Software Foundation. See the LICENSE file included with
 * this program for more details.
 */

/*
 * The TDM code is designed to deliver small arrays of size samples
 * every 1ms. A ping-pong arrangement is used, so the address of
 * the buffer will alternate every call between two possible values.
 *
 * The interrupt service routine interfaces with DAHDI in order to
 * exchange audio data with it. In the code, read means the data was
 * just read from the TDM, so this is the "received" TDM samples,
 * while write is the TDM data to be written to the si3217x.
 *
 * Once the init function returns successfully the SLIC/DMA will be up
 * and running, and calls to DAHDI will start as soon as it is
 * initialized in the DFXS subdriver.
 *
 * If module parameter debug is 1 then "cat /proc/dragino2_si3217x" will
 * display some debug information, something like:
 *
 * ........ SLIC ........
 * DFXS initialized.....: yes
 * isr_cycles_average...: 513
 * isr_cycles_worst.....: 1096
 * rx_swap_issues.......: 2
 * rx_swap_issues_0.....: 0
 * rx_swap_issues_1.....: 0
 * tx_swap_issues.......: 0
 * tx_swap_issues_0.....: 0
 * tx_swap_issues_1.....: 0
 *
 * If all is well (when DAHDI is not initialized) then the swap_issues
 * will be static and some very small number. DAHDI will add quite some
 * overhead so the swap_issues will grow, especially at DAHDI load and
 * hook switch.
 *
 * isr_cycles_worst/isr_cycles_average shows the biggest/average amount
 * of CPU clocks used for the ISR and DAHDI functions. Note that for 400 MHz
 * core we cannot have more than 400000 CPU cycles, after that we will
 * miss samples.
 *
 * "cat /proc/interrupts" will provide information about the ar9331-slic
 * interruptions count. 1000 RX and 1000 TX (2000) interrupts/second is
 * what should be observed.
 *
 * One channel is currently supported and DAHDI_CHUNKSIZE (8) bytes long
 * buffers are returned.
 */

#include <linux/slab.h>
#include <linux/device.h>

#include <linux/fs.h>
#include <linux/proc_fs.h>

#include <linux/seq_file.h>

#include <linux/delay.h>

#include <linux/dmapool.h>
#include <linux/interrupt.h>

#include "defs.h"
#include "tdm.h"

static struct tdm_device tdmdata;
static spinlock_t tdm_lock;

/*-------------------------- INIT SLIC INTERFACE ----------------------------
 *
 * For the Dragino we use the AR9331 SLIC to interface a single Si32176 slic
 * on the time slot 0,
 * internal TDM clock = 512 KHz
 * internal Long TDM FSYNC, 8 clocks long - 8KHz
 * Send FS at the positive edge of the BLT_CLK
 * 8bit word length
 * RX samples the first negative edge after FS positive edge
 * Tx data will be sent along with the FS positive edge
 */
static void init_slic(void)
{
	volatile uint32_t temp;

	/* Reset SLIC */
	temp = AR9331_REG_READ(AR9331_RST_RESET);
	AR9331_REG_WRITE(AR9331_RST_RESET, temp | AR9331_RST_RESET_SLIC);
	udelay(50);
	AR9331_REG_WRITE(AR9331_RST_RESET, temp & ~AR9331_RST_RESET_SLIC);

	/* One time slot */
	/* AR9331_REG_WRITE(AR9331_SLIC_SLOT, 32); */ /* For 2.048Mhz system */
	AR9331_REG_WRITE(AR9331_SLIC_SLOT, 8); /* For 512Khz system */

	/* 2.048 Mhz clock */
	AR9331_REG_WRITE(AR9331_SLIC_CLOCK_CONTROL, 63);

	/* Internal clock, MASTER mode */
	AR9331_REG_WRITE(AR9331_SLIC_CTRL, AR9331_SLIC_CTRL_INTR_EN | AR9331_SLIC_CTRL_CLK_EN | AR9331_SLIC_CTRL_MASTER);

	/* Time slot 0 is the only active TX time slot */
	AR9331_REG_WRITE(AR9331_SLIC_TX_SLOTS1, 0x00000001);
	AR9331_REG_WRITE(AR9331_SLIC_TX_SLOTS2, 0x00000000);

	/* Time slot 0 is the only active RX time slot */
	AR9331_REG_WRITE(AR9331_SLIC_RX_SLOTS1, 0x00000001);
	AR9331_REG_WRITE(AR9331_SLIC_RX_SLOTS2, 0x00000000);

	/*
	 * RX samples the first negative edge after FS
	 * Tx data will be sent along in the next positive edge after FS
	 * Send FS at the positive edge of the BLT_CLK
	 * one clock long FS.
	 * The above settings
	 * It seems the SLIC engine doesn't support well this mode
	 *
	 * Miss the Most Significant Bit
	 * AR9331_REG_WRITE(AR9331_SLIC_TIMING_CTRL, AR9331_SLIC_TIMING_CTRL_RXDATA_SAMPLE_NEG2 | AR9331_SLIC_TIMING_CTRL_TXDATA_FS_SYNC1_POS | \
	 * 											 AR9331_SLIC_TIMING_CTRL_FS_POS | \
	 * 											 AR9331_SLIC_TIMING_CTRL_LONG_FSCLKS1 | AR9331_SLIC_TIMING_CTRL_LONG_FS);
	 *
	 * The above mode corresponds to the "Short FSYNC (TXS/RXS = 1)"
	 * for the si3217x device but it seems the AR9331 SLIC engine
	 * misses the most significant bit if configured like this.
	 * We will use TXS/RXS = 0 , FS 8 clocks long
	 *
	 * RX samples the first negative edge after FS positive edge
	 * Tx data will be sent along with the FS positive edge
	 * Send FS at the positive edge of the BLT_CLK eight clock long FS.
	 */
	AR9331_REG_WRITE(AR9331_SLIC_TIMING_CTRL, AR9331_SLIC_TIMING_CTRL_RXDATA_SAMPLE_NEG | AR9331_SLIC_TIMING_CTRL_TXDATA_FS_SYNC0_POS | \
											  AR9331_SLIC_TIMING_CTRL_FS_POS | \
											  AR9331_SLIC_TIMING_CTRL_LONG_FSCLKS8 | AR9331_SLIC_TIMING_CTRL_LONG_FS);

	/* SLIC interrupts ienable all */
	AR9331_REG_WRITE(AR9331_SLIC_INTR, 0x1f);

	/*
	 * Set the I2S clock which is used by the SLIC
	 * 512 Khz = 400MHz / (390 + 40960/65536) / 2
	 * 2.048 Mhz = 400MHz / (97 + 43008/65536) / 2
	 */
	AR9331_REG_WRITE(0x180B001C, (390 << 16) + 40960L);

	printk_dbg(PFX "SLIC interface has been initialized\n");
}

/* init SLIC MBOX engine */
static int init_slic_mbox(struct spi_device *spidev)
{
	struct dma_pool *slic_descr_pool, *slic_buffer_pool;
	dma_addr_t dma_ptr;
	volatile uint32_t temp;

	/* Reset MBOX */
	temp = AR9331_REG_READ(AR9331_RST_RESET);
	AR9331_REG_WRITE(AR9331_RST_RESET, temp | AR9331_RST_RESET_MBOX);
	udelay(50);
	AR9331_REG_WRITE(AR9331_RST_RESET, temp & ~AR9331_RST_RESET_MBOX);

	/* Create DMA pool for the MBOX 4-byte-aligned descriptors. */
	slic_descr_pool = dma_pool_create("slic_descr_pool", &spidev->dev, sizeof(struct ar9331_slic_mbox_desc), 4, 0);

	/* Create two RX and two TX descriptors. Each will hold DAHDI_CHUNKSIZE bytes of data from/to the SLIC. */

	tdmdata.rx_descr0 = dma_pool_alloc(slic_descr_pool, GFP_KERNEL, &dma_ptr);
	if (!tdmdata.rx_descr0)
		return -ENOMEM;
	memset(tdmdata.rx_descr0, 0, sizeof(struct ar9331_slic_mbox_desc));
	tdmdata.rx_descr0->phy_addr = dma_ptr;
	tdmdata.rx_descr0->OWN = 1;
	tdmdata.rx_descr0->EOM = 0;
	tdmdata.rx_descr0->size = tdmdata.rx_descr0->length = DAHDI_CHUNKSIZE;

	tdmdata.rx_descr1 = dma_pool_alloc(slic_descr_pool, GFP_KERNEL, &dma_ptr);
	if (!tdmdata.rx_descr1)
		return -ENOMEM;
	memset(tdmdata.rx_descr1, 0, sizeof(struct ar9331_slic_mbox_desc));
	tdmdata.rx_descr1->phy_addr = dma_ptr;
	tdmdata.rx_descr1->OWN = 1;
	tdmdata.rx_descr1->EOM = 0;
	tdmdata.rx_descr1->size = tdmdata.rx_descr1->length = DAHDI_CHUNKSIZE;

	/* Link the two RX descriptors in the circular manner */
	tdmdata.rx_descr0->NextPtr = tdmdata.rx_descr1->phy_addr;
	tdmdata.rx_descr1->NextPtr = tdmdata.rx_descr0->phy_addr;

	tdmdata.tx_descr0 = dma_pool_alloc(slic_descr_pool, GFP_KERNEL, &dma_ptr);
	if (!tdmdata.tx_descr0)
		return -ENOMEM;
	memset(tdmdata.tx_descr0, 0, sizeof(struct ar9331_slic_mbox_desc));
	tdmdata.tx_descr0->phy_addr = dma_ptr;
	tdmdata.tx_descr0->OWN = 1;
	tdmdata.tx_descr0->EOM = 0;
	tdmdata.tx_descr0->size = tdmdata.tx_descr0->length = DAHDI_CHUNKSIZE;

	tdmdata.tx_descr1 = dma_pool_alloc(slic_descr_pool, GFP_KERNEL, &dma_ptr);
	if (!tdmdata.tx_descr1)
		return -ENOMEM;
	memset(tdmdata.tx_descr1, 0, sizeof(struct ar9331_slic_mbox_desc));
	tdmdata.tx_descr1->phy_addr = dma_ptr;
	tdmdata.tx_descr1->OWN = 1;
	tdmdata.tx_descr1->EOM = 0;
	tdmdata.tx_descr1->size = tdmdata.tx_descr1->length = DAHDI_CHUNKSIZE;

	/* Link the two TX descriptors in the circular manner */
	tdmdata.tx_descr0->NextPtr = tdmdata.tx_descr1->phy_addr;
	tdmdata.tx_descr1->NextPtr = tdmdata.tx_descr0->phy_addr;

	/*
	 * Create DMA pool for the MBOX SLIC 1-byte-aligned buffers.
	 * The first half of the buffer will contain DAHDI_CHUNKSIZE bytes for the descr0
	 * The second half of the buffer will contain DAHDI_CHUNKSIZE bytes for the descr1
	 */
	slic_buffer_pool = dma_pool_create("slic_buffer_pool", &spidev->dev, 2 * DAHDI_CHUNKSIZE, 1, 0);

	tdmdata.rx_data_0 = dma_pool_alloc(slic_buffer_pool, GFP_KERNEL, &dma_ptr);
	tdmdata.rx_data_1 = tdmdata.rx_data_0 + DAHDI_CHUNKSIZE;
	tdmdata.rx_descr0->BufPtr = dma_ptr;
	tdmdata.rx_descr1->BufPtr = dma_ptr + DAHDI_CHUNKSIZE;

	tdmdata.tx_data_0 = dma_pool_alloc(slic_buffer_pool, GFP_KERNEL, &dma_ptr);
	tdmdata.tx_data_1 = tdmdata.tx_data_0 + DAHDI_CHUNKSIZE;
	tdmdata.tx_descr0->BufPtr = dma_ptr;
	tdmdata.tx_descr1->BufPtr = dma_ptr + DAHDI_CHUNKSIZE;

	/* Set the MBOX SLIC registers */

	/* Clear interrupt flags */
	AR9331_REG_WRITE(AR9331_SLIC_MBOX_INT_STATUS, 0x7f);
	/* 8 bytes required so the TX chan in the queue */
	AR9331_REG_WRITE(AR9331_SLIC_MBOX_DMA_POLICY, (2 << 4) + 10);
	/* Assign the RX descriptor chain head */
	AR9331_REG_WRITE(AR9331_SLIC_MBOX_DMA_RX_DESCRIPTOR_BASE, tdmdata.rx_descr0->phy_addr);
	/* Assign the TX descriptor chain head */
	AR9331_REG_WRITE(AR9331_SLIC_MBOX_DMA_TX_DESCRIPTOR_BASE, tdmdata.tx_descr0->phy_addr);
	/* Enable both TX (0x10) and RX (0x40) interrupts */
	AR9331_REG_WRITE(AR9331_SLIC_MBOX_INT_ENABLE, 0x50);
	/* Reset both RX and TX FIFOs */
	AR9331_REG_WRITE(AR9331_SLIC_MBOX_FIFO_RESET, 3);

	printk_dbg(PFX "SLIC MBOX engine has been initialized\n");

	return 0;
}

static irqreturn_t ar9331_slic_interrupt(int irq, void *dev_id)
{
	volatile uint32_t status;//, temp;
	unsigned isr_cycles;
	static int rx_index = -1, tx_index = -1;
	int currdebug = debug;

	spin_lock(&tdm_lock);

	/* get the CPU cycles */
	if (currdebug)
		isr_cycles = cycles();

	status = AR9331_REG_READ(AR9331_SLIC_MBOX_INT_STATUS);

	#define INTERRUPT_TXRX_HANDLER(DIRECTION, direction, action) \
		if (status & AR9331_SLIC_MBOX_INT_STATUS_ ## DIRECTION ## _COMPLETE) { \
			/* Clear TX/RX flag */ \
			AR9331_REG_WRITE(AR9331_SLIC_MBOX_INT_STATUS, AR9331_SLIC_MBOX_INT_STATUS_ ## DIRECTION ## _COMPLETE); \
			/*temp = AR9331_REG_READ(AR9331_SLIC_MBOX_INT_STATUS);*/ \
			\
			if (!tdmdata.direction ## _descr0->OWN && tdmdata.direction ## _descr1->OWN) { /* CPU owns descriptor 0 */ \
				if (wc.initialized) { \
					action(tdmdata.direction ## _data_0); \
				} \
				if (currdebug) { \
					if (direction ## _index == 0) \
						tdmdata.direction ## _swap_issues_0++; \
					direction ## _index = 0; \
				} \
			} else if (tdmdata.direction ## _descr0->OWN && !tdmdata.direction ## _descr1->OWN) { /* CPU owns descriptor 1 */ \
				if (wc.initialized) { \
					action(tdmdata.direction ## _data_1); \
				} \
				if (currdebug) { \
					if (direction ## _index == 1) \
						tdmdata.direction ## _swap_issues_1++; \
					direction ## _index = 1; \
				} \
			} else if (currdebug) { \
				tdmdata.direction ## _swap_issues++; \
			} \
			\
			/* Needs to be before RESUME */ \
			tdmdata.direction ## _descr0->OWN = 1; \
			tdmdata.direction ## _descr1->OWN = 1; \
			\
			/* dfxs stops the interrupt if we don't resume ?? */ \
			AR9331_REG_WRITE(AR9331_SLIC_MBOX_DMA_ ## DIRECTION ## _CONTROL, AR9331_SLIC_MBOX_DMA_RESUME); /* Resume, Why do we need resume??? */ \
		}

	/* TX_COMPLETE is interrupt servicing data coming in from SLIC engine */
	#define INTERRUPT_TX_ACTION(tx_data) \
		memcpy(wc.chan.readchunk, tx_data, DAHDI_CHUNKSIZE * sizeof(u8)); \
		dahdi_ec_chunk(&wc.chan, wc.chan.readchunk, wc.chan.writechunk); \
		dahdi_receive(&wc.span);
	INTERRUPT_TXRX_HANDLER(TX, tx, INTERRUPT_TX_ACTION);

	/* RX_COMPLETE is interrupt servicing data going out to SLIC engine */
	#define INTERRUPT_RX_ACTION(rx_data) \
		dahdi_transmit(&wc.span); \
		memcpy(rx_data, wc.chan.writechunk, DAHDI_CHUNKSIZE * sizeof(u8));
	INTERRUPT_TXRX_HANDLER(RX, rx, INTERRUPT_RX_ACTION);

	#undef INTERRUPT_TXRX_HANDLER
	#undef INTERRUPT_TX_ACTION
	#undef INTERRUPT_RX_ACTION

	/*
	 * some stats to help monitor the cycles used by ISR processing:
	 *
	 * Simple IIR averager: (TC is time constant for example 1024)
	 * y(n) = (1 - 1/TC)*y(n) + (1/TC)*x(n)
	 *
	 * After conversion to fixed point:
	 * 2*y(n) = ((TC-1)*2*y(n) + 2*x(n) + half_lsb ) >> LTC
	 *
	 * Where TC is time constant for example 1024 and LTC is log2(TC)
	*/
	if (currdebug) {
		isr_cycles = cycles() - isr_cycles;
		tdmdata.isr_cycles_average = ((u32)(TC-1) * tdmdata.isr_cycles_average + (((u32)isr_cycles) << 1) + TC) >> LTC;
		if (tdmdata.isr_cycles_worst < isr_cycles)
			tdmdata.isr_cycles_worst = isr_cycles;
 	}

	spin_unlock(&tdm_lock);
	return IRQ_HANDLED;
}

static int init_interrupts(void)
{
	volatile uint32_t temp;

	/* Some common initialization */
	tdmdata.irq_mbox = ATH79_MISC_IRQ(7); /* ATH79_MISC_IRQ_DMA */
	tdmdata.irq_mbox_name = "ar9331-slic";

	/* hook the interrupt */
	if (request_irq(tdmdata.irq_mbox, ar9331_slic_interrupt, 0, tdmdata.irq_mbox_name, &tdmdata) != 0) {
		printk(KERN_ERR PFX "failed to hook %s ISR\n", tdmdata.irq_mbox_name);
		return -EBUSY;
	}

	/* Enable global MBOX_INTERRUPTs */
	temp = AR9331_REG_READ(AR9331_RST_MISC_INTERRUPT_MASK);
	AR9331_REG_WRITE(AR9331_RST_MISC_INTERRUPT_MASK, temp | AR9331_RST_MISC_INTERRUPT_MBOX);
	/* printk_dbg(PFX "%s ISR installed OK\n", tdmdata.irq_mbox_name); */

	return 0;
}


static void enable_slic(void)
{
	volatile uint32_t temp;

	/* enable SLIC and interrupts */
	temp = AR9331_REG_READ(AR9331_SLIC_CTRL);
	AR9331_REG_WRITE(AR9331_SLIC_CTRL, temp | AR9331_SLIC_CTRL_SLIC_EN | AR9331_SLIC_CTRL_INTR_EN);

	/* Enable SLIC on GPIO18..22, SLIC data in and out are on separate pins. */
	temp = AR9331_REG_READ(AR9331_GPIO_FUNCTION_2);
	AR9331_REG_WRITE(AR9331_GPIO_FUNCTION_2, (temp & ~AR9331_GPIO_SLIC_DIO_MUX_EN) | AR9331_GPIO_SLIC_18_22 | AR9331_GPIO_SLIC_EN);
	//temp = AR9331_REG_READ(AR9331_GPIO_FUNCTION_2);

	/* Start the RX/TX DMA */
	AR9331_REG_WRITE(AR9331_SLIC_MBOX_DMA_RX_CONTROL, AR9331_SLIC_MBOX_DMA_START);
	AR9331_REG_WRITE(AR9331_SLIC_MBOX_DMA_TX_CONTROL, AR9331_SLIC_MBOX_DMA_START);

	/* printk_dbg(PFX "SLIC has been enabled\n"); */ /* This print stops the interrupts !!!!!!!!!!!!!!!!!!!!! */
}

static void disable_slic(void)
{
	volatile uint32_t temp;

	/* disable SLIC and interrupts */
	temp = AR9331_REG_READ(AR9331_SLIC_CTRL);
	AR9331_REG_WRITE(AR9331_SLIC_CTRL, (temp & ~AR9331_SLIC_CTRL_SLIC_EN) & ~AR9331_SLIC_CTRL_INTR_EN);

	printk_dbg(PFX "SLIC has been disabled\n");
}

static int tdm_proc_show(struct seq_file *sfile, void *not_used)
{
	if (debug) {
		seq_printf(sfile,
			"........ SLIC ........\n"
			"DFXS initialized.....: %s\n"
			"isr_cycles_average...: %d\n"
			"isr_cycles_worst.....: %d\n"
			"rx_swap_issues.......: %d\n"
			"rx_swap_issues_0.....: %d\n"
			"rx_swap_issues_1.....: %d\n"
			"tx_swap_issues.......: %d\n"
			"tx_swap_issues_0.....: %d\n"
			"tx_swap_issues_1.....: %d\n",
			wc.initialized ? "yes" : "no",
			tdmdata.isr_cycles_average >> 1,
			tdmdata.isr_cycles_worst,
			tdmdata.rx_swap_issues,
			tdmdata.rx_swap_issues_0,
			tdmdata.rx_swap_issues_1,
			tdmdata.tx_swap_issues,
			tdmdata.tx_swap_issues_0,
			tdmdata.tx_swap_issues_1);
	} else {
		seq_printf(sfile,
			"........ SLIC ........\n"
			"DFXS initialized.....: %s\n",
			wc.initialized ? "yes" : "no");
	}
	return 0;
}

static int tdm_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tdm_proc_show, PDE_DATA(inode));
}

static const struct file_operations tdm_proc_ops = {
	.owner		= THIS_MODULE,
	.open		= tdm_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
#define proc_remove(proc_entry) remove_proc_entry((proc_entry)->name, NULL)
#endif

int si3217x_tdm_init(struct spi_device *spidev)
{
	int err;

	memset(&tdmdata, 0, sizeof(struct tdm_device));

	tdmdata.proc_entry = proc_create_data(DRV_NAME, 0400, NULL, &tdm_proc_ops, NULL);
	if (!tdmdata.proc_entry) {
		printk(KERN_ERR PFX "Failed to create proc entry\n");
		return -ENOMEM;
	}

	init_slic();

	err = init_slic_mbox(spidev);
	if (err) {
		printk(KERN_ERR PFX "Failed to initialize SLIC MBOX engine\n");
		free_irq(tdmdata.irq_mbox, &tdmdata);
		proc_remove(tdmdata.proc_entry);
		return err;
	}

	enable_slic();

	/* printk after this point stops TX interrupt !!!!!!???? */
	err = init_interrupts();
	if (err) {
		printk(KERN_ERR PFX "Failed to initialize interrupts\n");
		disable_slic();
		free_irq(tdmdata.irq_mbox, &tdmdata);
		proc_remove(tdmdata.proc_entry);
		return err;
	}

	return 0;
}

/* shut down SLIC operation cleanly */
void si3217x_tdm_exit(void)
{
	disable_slic();
	free_irq(tdmdata.irq_mbox, &tdmdata);
	proc_remove(tdmdata.proc_entry);
}
