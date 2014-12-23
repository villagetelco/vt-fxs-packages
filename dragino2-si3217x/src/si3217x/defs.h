/*
 * DRAGINO v2 si3217x FXS Daughter Board definitions
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

#ifndef __SI3217X_DEFS
#define __SI3217X_DEFS

#include <linux/spi/spi.h>

#define DEVICE_DESC	"DRAGINO v2 si3217x FXS Daughter Board"

#define DRV_NAME	"dragino2_si3217x"
#define DRV_DESC	DEVICE_DESC " driver"
#define DRV_VERSION	"0.2"

#define PFX		DRV_NAME ": "

/* proslic */
extern void *si3217x_blobwrapper_proslic_init(
	int,
	u8 (*)(void *, u8, u8),
	int (*)(void *, u8, u8, u8),
	u32 (*)(void *, u8, u16),
	int (*)(void *, u8, u16, u32 data),
	int (*)(void *, int),
	int (*)(void *, int),
	int (*)(void *, void *, int *),
	int (*)(void *, void *)
);
extern void si3217x_blobwrapper_proslic_free(void *);
extern void si3217x_blobwrapper_set_linefeed_status(void *, u8);
extern int si3217x_blobwrapper_get_hook_status(void *);

/* main */
extern int debug;
extern int si3217x_proslic_init(void);
extern void si3217x_proslic_free(void);
extern void si3217x_set_linefeed_status(u8);
extern int si3217x_get_hook_status(void);

/* spi-platform-device */
extern int __init si3217x_spi_platform_device_init(void);
extern void si3217x_spi_platform_device_exit(void);

/* spi */
extern struct spi_driver si3217x_spi_driver;
extern int si3217x_read_reg(u8 address, u8 *val);
extern int si3217x_write_reg(u8 address, u8 val);
extern int si3217x_read_ram(u16 address, u32 *val);
extern int si3217x_write_ram(u16 address, u32 val);
extern int si3217x_reset(int state);

/* tdm */
extern int __init si3217x_tdm_init(void);
extern void si3217x_tdm_exit(void);

/* dfxs */
extern struct dfxs wc;
extern void si3217x_dfxs_spidev_callback(struct spi_device *);

/* #define printk_dbg(format, ...) printk("%s" format, (debug ? KERN_INFO : KERN_DEBUG), ## __VA_ARGS__) */
#define printk_dbg(format, ...) if (debug) printk(KERN_INFO format, ## __VA_ARGS__)

#endif /* __SI3217X_DEFS */
