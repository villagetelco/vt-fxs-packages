/*
 * DRAGINO v2 si3217x FXS Daughter Board register definitions
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

#ifndef __SI3217X_SPI
#define __SI3217X_SPI

/* AR9331 registers access macros */
#define SZ_4K						0x00001000
#define AR9331_REG_READ(addr)		readl(ioremap((addr), SZ_4K))
#define AR9331_REG_WRITE(addr, val)	writel((u32)(val), ioremap((addr), SZ_4K))

/* SPI definitions */
#define SPI_BRDCST					0x80	/* Indicates a broadcast operation */
#define SPI_READ					0x40	/* Read operation */
#define SPI_WRITE					0x00	/* Write operation */
#define SPI_REG						0x20	/* Register access */
#define SPI_RAM						0x00	/* RAM access */
#define SPI_CID0					0x00	/* Channel 0 is the nearest to the controller in the SPI chain */

#define USER_TIMEOUT_VAL			100		/* Time out counter for the RAM access */

/* Bit definitions for LTV_IFCTL */
#define LTV_IM						(1 << 15)
#define LTV_NMD						(1 << 14)
#define LTV_SSMD					(1 << 13)
#define LTV_REV						(1 << 7)
#define LTV_NL(x)					(((x) & 0x001f) << 0)

enum {
	SI3217X_COM_REG_RAMSTAT		= 4,
	SI3217X_COM_REG_RAM_ADDR_HI	= 5,
	SI3217X_COM_REG_RAM_DATA_B0	= 6,
	SI3217X_COM_REG_RAM_DATA_B1	= 7,
	SI3217X_COM_REG_RAM_DATA_B2	= 8,
	SI3217X_COM_REG_RAM_DATA_B3	= 9,
	SI3217X_COM_REG_RAM_ADDR_LO	= 10,
};

#endif /* __SI3217X_SPI */
