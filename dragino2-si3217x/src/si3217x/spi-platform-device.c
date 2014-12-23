/*
 * DRAGINO v2 si3217x FXS Daughter Board support via a GPIO bitbanged SPI Master with ID-number 1
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

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>

#include "defs.h"

/*
 * GPIO resources used by for the Si3217x-based FXS SPI slave module:
 *
 * CS (Chip select) for FXS board, SPI slave device #0, selected by GPIO_24 @Router_header_PIN12
 * FXS_RESET = GPIO_23 = @Router_header_PIN9
 * FXS_INT   = GPIO_27 = @JTAG_header_PIN2
 * SCLK      = GPIO_15 = @JTAG_header_PIN9
 * MOSI      = GPIO_16 = @JTAG_header_PIN10
 * MISO      = GPIO_26 = @JTAG_header_PIN1
 */

static struct spi_gpio_platform_data si3217x_spi_gpio_data = {
	.sck		= 15,
	.mosi		= 16,
	.miso		= 26,
	.num_chipselect	= 1
};

static void dummy_platform_device_release(struct device *dev)
{
	printk_dbg(PFX "Releasing platform_device\n");
}

static struct platform_device si3217x_spi = {
	.name			= "spi_gpio",
	.id			= 1,
	.dev.platform_data	= &si3217x_spi_gpio_data,
	.dev.release 		= dummy_platform_device_release
};

/* TODO: Add SPI client support for the FXS submodule */
/*static struct si3217x_platform_data si3217x_spi_data [] = {
	{
		.reset	= 23
	}
};*/

static struct spi_board_info si3217x_spi_info = {
	.bus_num			= 1,
	.max_speed_hz		= 10000000,
	.modalias			= "dragino2_si3217x",
	.chip_select		= 0,
	.controller_data	= (void *) 24,
	.mode				= 3,
	.irq				= 27
};

int __init si3217x_spi_platform_device_init(void)
{
	int err;
	struct spi_master *master;
	struct spi_device *slave;

	err = platform_device_register(&si3217x_spi);
	if (err) {
		printk(KERN_ERR PFX "platform_device_register failed with return code %d\n", err);
		return err;
	}

	master = spi_busnum_to_master(si3217x_spi_info.bus_num);
	if (!master) {
		printk(KERN_ERR PFX "Unable to get master for bus %d; spi-gpio has not been loaded yet\n", si3217x_spi_info.bus_num);
		platform_device_unregister(&si3217x_spi);
		return -EINVAL;
	}
	slave = spi_new_device(master, &si3217x_spi_info);
	spi_master_put(master);
	if (!slave) {
		printk(KERN_ERR PFX "Unable to create si3217x slave for bus %d\n", si3217x_spi_info.bus_num);
		platform_device_unregister(&si3217x_spi);
		return -EINVAL;
	}

	return 0;
}

void si3217x_spi_platform_device_exit(void)
{
	platform_device_unregister(&si3217x_spi);
}
