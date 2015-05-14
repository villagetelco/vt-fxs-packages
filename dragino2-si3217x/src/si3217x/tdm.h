/*
 * DRAGINO v2 si3217x FXS Daughter Board TDM definitions
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

#ifndef __SI3217X_TDM
#define __SI3217X_TDM

#include "dfxs.h"

/* AR9331 registers access macros */
#define SZ_4K						0x00001000
#define AR9331_REG_READ(addr)		readl(ioremap((addr), SZ_4K))
#define AR9331_REG_WRITE(addr, val)	writel((u32)(val), ioremap((addr), SZ_4K))

/* GPIO SLIC pin multiplexing */
#define AR9331_GPIO_FUNCTION_2		0x18040030

#define AR9331_GPIO_SLIC_EN			0x00002000 /* Enable (gpio bits 18,19,20,21,and 22) or (gpio bits 6,7,8,11,12) as SLIC interface pins. Note: I2S0_EN should be disable when SLIC are enable. */
#define AR9331_GPIO_SLIC_18_22		0x00004000 /* If this bit is set, then SLIC pins are mapped on the GPIOs 18,...,22. Else, GPIO 6,7,8,11 and 12 are used for SLIC. This is needed if we need to support SLIC in the same package as python 1.0 without any additional pins. In this case, this bit needs to be set to 0. */
#define AR9331_GPIO_SLIC_DIO_MUX_EN	0x00008000 /* If this bit is set, SLIC data in and data out are mux to single pin. [GPIO21 or GPIO11] */

/* SLIC Registers */
#define AR9331_SLIC_SLOT			0x18090000
#define AR9331_SLIC_CLOCK_CONTROL	0x18090004
#define AR9331_SLIC_CTRL			0x18090008
#define AR9331_SLIC_TX_SLOTS1		0x1809000C
#define AR9331_SLIC_TX_SLOTS2		0x18090010
#define AR9331_SLIC_RX_SLOTS1		0x18090014
#define AR9331_SLIC_RX_SLOTS2		0x18090018
#define AR9331_SLIC_TIMING_CTRL		0x1809001C
#define AR9331_SLIC_INTR			0x18090020
#define AR9331_SLIC_SWAP			0x18090024

#define AR9331_SLIC_CTRL_CLK_EN		0x00000008
#define AR9331_SLIC_CTRL_MASTER		0x00000004
#define AR9331_SLIC_CTRL_SLIC_EN	0x00000002
#define AR9331_SLIC_CTRL_INTR_EN	0x00000001

/* SLIC flags */
#define AR9331_SLIC_TIMING_CTRL_RXDATA_SAMPLE_NEG	0x00000000 /* RX data samples at the negative clock edge after FS positive edge */
#define AR9331_SLIC_TIMING_CTRL_RXDATA_SAMPLE_POS	0x00000100 /* RX data samples at the positive clock edge after FS positive edge */
#define AR9331_SLIC_TIMING_CTRL_RXDATA_SAMPLE_NEG2	0x00000180 /* RX data samples at the second negative clock edge after FS positive edge */
#define AR9331_SLIC_TIMING_CTRL_TXDATA_FS_SYNC0_POS	0x00000000 /* Tx data will be sent along with a frame sync positive edge. */
#define AR9331_SLIC_TIMING_CTRL_TXDATA_FS_SYNC1_POS	0x00000040 /* Tx data will be sent in the next positive edge of the BLT_CLK after a frame sync positive edge. */
#define AR9331_SLIC_TIMING_CTRL_TXDATA_FS_SYNC1_NEG	0x00000060 /* Tx data will be sent along in the next negative edge of the BLT_CLK after a frame sync positive edge */
#define AR9331_SLIC_TIMING_CTRL_LONG_FSCLKS1		0x00000000 /* FS - one BLT_CLK long */
#define AR9331_SLIC_TIMING_CTRL_LONG_FSCLKS8		0x0000001C /* FS - eight BLT_CLKs long */
#define AR9331_SLIC_TIMING_CTRL_FS_NEG				0x00000000 /* Send FS at the negative edge of the BLT_CLK */
#define AR9331_SLIC_TIMING_CTRL_FS_POS				0x00000002 /* Send FS at the positive edge of the BLT_CLK */
#define AR9331_SLIC_TIMING_CTRL_LONG_FS				0x00000001 /* FS is high for more than 1 BLT_CLK duration */


/* MBOX engine associated with the SLIC */
#define AR9331_SLIC_MBOX_FIFO_STATUS			0x180A000C
#define AR9331_SLIC_MBOX_DMA_POLICY				0x180A0014
#define AR9331_SLIC_MBOX_DMA_RX_DESCRIPTOR_BASE	0x180A0028
#define AR9331_SLIC_MBOX_DMA_RX_CONTROL			0x180A002C
#define AR9331_SLIC_MBOX_DMA_TX_DESCRIPTOR_BASE	0x180A0030
#define AR9331_SLIC_MBOX_DMA_TX_CONTROL			0x180A0034
#define AR9331_SLIC_MBOX_FRAME					0x180A003C
#define AR9331_SLIC_MBOX_INT_STATUS				0x180A0048
#define AR9331_SLIC_MBOX_INT_ENABLE				0x180A0050
#define AR9331_SLIC_MBOX_FIFO_RESET				0x180A005C

/* SLIC MBOX flags */
#define AR9331_SLIC_MBOX_INT_STATUS_RX_COMPLETE		(1<<6)
#define AR9331_SLIC_MBOX_INT_STATUS_TX_COMPLETE		(1<<4)
#define AR9331_SLIC_MBOX_INT_STATUS_TX_EOM_COMPLETE	(1<<5)

#define AR9331_SLIC_MBOX_DMA_STOP			(1<<0)
#define AR9331_SLIC_MBOX_DMA_START			(1<<1)
#define AR9331_SLIC_MBOX_DMA_RESUME			(1<<2)

/* Reset Control Registers and flags*/
#define AR9331_RST_MISC_INTERRUPT_MASK	0x18060014
#define AR9331_RST_RESET				0x1806001C
#define AR9331_RST_MISC_INTERRUPT_MBOX	(1<<7)
#define AR9331_RST_RESET_SLIC			(1<<6)
#define AR9331_RST_RESET_MBOX			(1<<1)

/*DMA descriptor for the SLIC engine */
struct ar9331_slic_mbox_desc {
	unsigned int OWN:1,			/* bit 31 */
				 EOM:1,			/* bit 30 */
				 rsvd1:6,		/* bit 29-24 */
				 size:12,		/* bit 23-12 */
				 length:12,		/* bit 11-00 */
				 rsvd2:4,		/* bit 31-28 */
				 BufPtr:28,		/* bit 27-00 */
				 rsvd3:4,		/* bit 31-28 */
				 NextPtr:28;	/* bit 27-00 */

	unsigned int Va[6]; /* SPDIF data, not used by SLIC DMA */
	unsigned int Ua[6];
	unsigned int Ca[6];
	unsigned int Vb[6];
	unsigned int Ub[6];
	unsigned int Cb[6];

	/* Software specific data, DMA doesn't access those */
	dma_addr_t phy_addr; /* Physical address of the descriptor */
};

/* constants for isr cycle averaging */
#define TC 1024	/* time constant */
#define LTC 10	/* base 2 log of TC */

struct tdm_device {
	/* DMA number and name */
	int irq_mbox;
	char *irq_mbox_name;

	/* SLIC DMA descriptors */
	struct ar9331_slic_mbox_desc *rx_descr0, *rx_descr1, *tx_descr0, *tx_descr1;

	/* DMA buffers 2*DAHDI_CHUNK_SIZE long, the first half of the buffers is own by the descr0 and second half by descr1 */
	u8 *rx_data_0, *rx_data_1, *tx_data_0, *tx_data_1;

	/* data buffers */
	unsigned char *iTxBuffer1;
	unsigned char *iRxBuffer1;

	struct proc_dir_entry *proc_entry;

	/* average and worst case number of cycles we took to process an interrupt */
	int rx_swap_issues, rx_swap_issues_0, rx_swap_issues_1;
	int tx_swap_issues, tx_swap_issues_0, tx_swap_issues_1;
	unsigned int isr_cycles_worst;
	unsigned int isr_cycles_average; /* scaled up by 2x */
};

/* sample cycles for MIPS */
static inline unsigned cycles(void)
{
	unsigned cc;

	__asm__ __volatile__
	(
	"mfc0 %0, $9"
	: "=r" (cc)
	);

	return (cc << 1);
}

#endif /* __SI3217X_TDM */
