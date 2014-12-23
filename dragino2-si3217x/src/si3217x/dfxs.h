/*
 * DRAGINO v2 si3217x FXS Daughter Board DAHDI driver definitions
 *
 * Copyright (C) 2013-2014 Village Telco Ltd.
 * Copyright (C) 2013-2014 Dimitar Penev <dpn at switchvoice dot com>
 * Copyright (C) 2014 Vittorio Gambaletta <openwrt@vittgam.net>
 *
 * This program is free software, distributed under the terms of
 * the GNU General Public License Version 2 as published by the
 * Free Software Foundation. See the LICENSE file included with
 * this program for more details.
 */

#ifndef __SI3217X_DFXS
#define __SI3217X_DFXS

#include <dahdi/kernel.h>

#define NUM_REGS	129
#define NUM_RAMS 	1024

struct dfxs {
	int initialized;

	struct dahdi_device *ddev;
	struct dahdi_span span;
	struct dahdi_chan chan;
	struct dahdi_chan *_chan;

	int linefeed_status;
	int last_hook_status;
	int onhooktransmission;
	int reverse_linefeed_polarity;

	struct dahdi_vmwi_info vmwisetting;
	int vmwi_message_count;

	volatile u8 *writechunk;	/* Double-word aligned write memory */
	volatile u8 *readchunk;		/* Double-word aligned read memory */
};

struct dfxs_stats {
	int tipvolt;	/* TIP voltage (mV) */
	int ringvolt;	/* RING voltage (mV) */
	int batvolt;	/* VBAT voltage (mV) */
};

struct dfxs_mem {
	u8 reg[NUM_REGS];
	u32 ram[NUM_RAMS];
};

struct dfxs_regop {
	u8 address;
	u8 val;
};

struct dfxs_ramop {
	u16 address;
	u32 val;
};

#define DFXS_GET_STATS	_IOR(DAHDI_CODE, 60, struct dfxs_stats)
#define DFXS_GET_REGS	_IOR(DAHDI_CODE, 61, struct dfxs_mem)
#define DFXS_SET_REG	_IOR(DAHDI_CODE, 62, struct dfxs_regop)
#define DFXS_GET_REG	_IOWR(DAHDI_CODE, 63, struct dfxs_regop)
#define DFXS_SET_RAM	_IOR(DAHDI_CODE, 64, struct dfxs_ramop)
#define DFXS_GET_RAM	_IOWR(DAHDI_CODE, 65, struct dfxs_ramop)

/* some registers/ram locations */
#define SI3217X_MADC_VTIPC	1
#define SI3217X_MADC_VRINGC	2
#define SI3217X_MADC_VBAT	3
#define SI3217X_MADC_ILONG	7
#define SI3217X_MADC_ILOOP	10
#define SI3217X_LINEFEED	30

#define SI3217X_VOLTRES	977L /* 931.323e-6 scaled by 2^20 */

#endif /* __SI3217X_DFXS */
