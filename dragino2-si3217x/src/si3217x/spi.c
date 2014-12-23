/*
 * DRAGINO v2 si3217x FXS Daughter Board SPI control driver
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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/spi/spi.h>

#include "defs.h"
#include "spi.h"

static struct spi_device *spi = NULL;

/*
 * Register read operation via an 8-Bit SPI Port
 * Check Silabs AN340 Rev 1.0 Chapter 2.2.2. SPI Control Interface
 */
int si3217x_read_reg(u8 address, u8 *val)
{
	u8 ctrl_byte = SPI_CID0 | SPI_READ | SPI_REG;
	struct spi_message msg;
	struct spi_transfer tctrl = {
		.len		= 1,
		.cs_change	= 1,
	};
	struct spi_transfer taddr = {
		.len		= 1,
		.cs_change	= 1,
	};
	struct spi_transfer tval = {
		.len		= 1,
		.cs_change	= 0,
	};

	*val = 0;

	if (!spi)
		return -ENODEV;

	spi_message_init(&msg);

	tctrl.tx_buf = &ctrl_byte;
	spi_message_add_tail(&tctrl, &msg);
	taddr.tx_buf = &address;
	spi_message_add_tail(&taddr, &msg);
	tval.rx_buf = val;
	spi_message_add_tail(&tval, &msg);

	return spi_sync(spi, &msg);
}

/*
 * Register write operation via an 8-Bit SPI Port
 * Check Silabs AN340 Rev 1.0 Chapter 2.2.2. SPI Control Interface
 */
int si3217x_write_reg(u8 address, u8 val)
{
	u8 ctrl_byte = SPI_CID0 | SPI_WRITE | SPI_REG;
	struct spi_message msg;
	struct spi_transfer tctrl = {
		.len		= 1,
		.cs_change	= 1,
	};
	struct spi_transfer taddr = {
		.len		= 1,
		.cs_change	= 1,
	};
	struct spi_transfer tval = {
		.len		= 1,
		.cs_change	= 0
	};

	if (!spi)
		return -ENODEV;

	spi_message_init(&msg);

	tctrl.tx_buf = &ctrl_byte;
	spi_message_add_tail(&tctrl, &msg);
	taddr.tx_buf = &address;
	spi_message_add_tail(&taddr, &msg);
	tval.tx_buf = &val;
	spi_message_add_tail(&tval, &msg);

	return spi_sync(spi, &msg);
}

/*
 * RAM read operation via an 8-Bit SPI Port
 */
int si3217x_read_ram(u16 address, u32 *val)
{
	u8 dataByte;
	u8 addrByte;
	u32 userTimeout = USER_TIMEOUT_VAL; /* user defined timeout counter */

	*val = 0;

	if (!spi)
		return -ENODEV;

	/* Wait for RAM to finish */
	do {
		si3217x_read_reg(SI3217X_COM_REG_RAMSTAT, &dataByte);
	}
	while ((dataByte & 0x01) && userTimeout--);

	/* RAM_ADR_HI[7:5] = ramAddr[10:8] */
	addrByte = (u8)((address >> 3) & 0x00E0);
	si3217x_write_reg(SI3217X_COM_REG_RAM_ADDR_HI, addrByte);

	/* RAM_ADR_LO[7:0] = ramAddr[7:0] */
	addrByte = (u8)(address & 0x00FF);
	si3217x_write_reg(SI3217X_COM_REG_RAM_ADDR_LO, addrByte);

	/* Wait for RAM to finish */
	do {
		si3217x_read_reg(SI3217X_COM_REG_RAMSTAT, &dataByte);
	} while ((dataByte & 0x01) && userTimeout--);

	/* ramData[4:0] = RAM_DATA_B0[7:3] */
	si3217x_read_reg(SI3217X_COM_REG_RAM_DATA_B0, &dataByte);
	*val = ((dataByte >> 3) & 0x0000001FL);

	/* ramData[12:5] = RAM_DATA_B1[7:0] */
	si3217x_read_reg(SI3217X_COM_REG_RAM_DATA_B1, &dataByte);
	*val |= (((u32)dataByte << 5) & 0x000001FE0L);

	/* ramData[20:13] = RAM_DATA_B2[7:0] */
	si3217x_read_reg(SI3217X_COM_REG_RAM_DATA_B2, &dataByte);
	*val |= (((u32)dataByte << 13) & 0x0001FE000L);

	/* ramData[28:21] = RAM_DATA_B3[7:0] */
	si3217x_read_reg(SI3217X_COM_REG_RAM_DATA_B3, &dataByte);
	*val |= (((u32)dataByte << 21) & 0x1FE00000L);

	if (userTimeout <= 0)
		return -1;
	else
		return 0;
}

/*
 * RAM write operation via an 8-Bit SPI Port
 */
int si3217x_write_ram(u16 address, u32 val)
{
	u8 dataByte;
	u8 addrByte;
	u32 userTimeout = USER_TIMEOUT_VAL; /* User defined timeout counter */

	if (!spi)
		return -ENODEV;

	/* Wait for RAM to finish */
	do {
		si3217x_read_reg(SI3217X_COM_REG_RAMSTAT, &dataByte);
	} while ((dataByte & 0x01) && userTimeout--);

	/* RAM_ADR_HI[7:5] = ramAddr[10:8] */
	addrByte = (u8)((address >> 3) & 0x00E0);
	si3217x_write_reg(SI3217X_COM_REG_RAM_ADDR_HI, addrByte);

	/* RAM_DATA_B0[7:3] = ramData[4:0] */
	dataByte = (u8)((val << 3) & 0x000000F1L);
	si3217x_write_reg(SI3217X_COM_REG_RAM_DATA_B0, dataByte);

	/* RAM_DATA_B1[7:0] = ramData[12:5] */
	dataByte = (u8)((val >> 5) & 0x000000FFL);
	si3217x_write_reg(SI3217X_COM_REG_RAM_DATA_B1, dataByte);

	/* RAM_DATA_B2[7:0] = ramData[20:13] */
	dataByte = (u8)((val >> 13) & 0x000000FFL);
	si3217x_write_reg(SI3217X_COM_REG_RAM_DATA_B2, dataByte);

	/* RAM_DATA_B3[7:0] = ramData[28:21] */
	dataByte = (u8)((val >> 21) & 0x000000FFL);
	si3217x_write_reg(SI3217X_COM_REG_RAM_DATA_B3, dataByte);

	/* RAM_ADR_LO[7:0] = ramAddr[7:0] */
	addrByte = (u8)(address & 0x00FF);
	si3217x_write_reg(SI3217X_COM_REG_RAM_ADDR_LO, addrByte);

	if (userTimeout <= 0)
		return -1;
	else
		return 0;
}

/*
 * Reset the SLIC chip
 * state = 1: reset asserted
 * state = 0: reset released
 */
int si3217x_reset(int state)
{
	volatile uint32_t temp;

	/* Implement software reset of the slic. Better go to full hardware reset */
	/* return si3217x_write_reg(SI3217X_COM_REG_RESET, state); */

	/* Reset the SLIC using GPIO23 */
	/* Set GPIO23 as output */
	temp = AR9331_REG_READ(0x18040000);
	AR9331_REG_WRITE(0x18040000, temp | (1 << 23));

	/* GPIO23 is not SPDIF output */
	temp = AR9331_REG_READ(0x18040030);
	AR9331_REG_WRITE(0x18040030, temp & ~(1 << 2));

	/* GPIO23 is not connected to WLAN */
	temp = AR9331_REG_READ(0x18040034);
	AR9331_REG_WRITE(0x18040034, temp & ~(1 << 23));

	if (state)
		AR9331_REG_WRITE(0x18040010, 1 << 23); /* Set Reset pin LOW */
	else
		AR9331_REG_WRITE(0x1804000C, 1 << 23); /* Set Reset pin HIGH */

	return(0);
}

static int si3217x_spi_probe(struct spi_device *spidev)
{
	spi = spidev;
	si3217x_dfxs_spidev_callback(spidev);
	return 0;
}

static int si3217x_spi_remove(struct spi_device *spidev)
{
	spi = NULL;
	si3217x_dfxs_spidev_callback(NULL);
	return 0;
}

struct spi_driver si3217x_spi_driver = {
	.driver = {
		.name	= "dragino2_si3217x",
		.owner	= THIS_MODULE
	},
	.probe	= si3217x_spi_probe,
	.remove	= si3217x_spi_remove
};
