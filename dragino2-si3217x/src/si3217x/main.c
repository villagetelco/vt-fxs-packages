/*
 * DRAGINO v2 si3217x FXS Daughter Board driver Linux Kernel Module
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

#include "defs.h"
#include "proslic_time_glue.h"

int debug = 0;

static char *opermode = "FCC";
static int _opermode = 0;
static void *ProObj = NULL;

static u8 ctrl_ReadRegisterWrapper(void *dummy, u8 dummy2, u8 regAddr)
{
	u8 val;
	si3217x_read_reg(regAddr, &val);
	return val;
}

static int ctrl_WriteRegisterWrapper(void *dummy, u8 dummy2, u8 regAddr, u8 data)
{
	return si3217x_write_reg(regAddr, data);
}

static u32 ctrl_ReadRAMWrapper(void *dummy, u8 dummy2, u16 ramAddr)
{
	u32 val;
	si3217x_read_ram(ramAddr, &val);
	return val;
}

static int ctrl_WriteRAMWrapper(void *dummy, u8 dummy2, u16 ramAddr, u32 data)
{
	return si3217x_write_ram(ramAddr, data);
}

static int ctrl_ResetWrapper(void *dummy, int in_reset)
{
	return si3217x_reset(in_reset);
}

int si3217x_proslic_init()
{
	if (ProObj)
		return -EEXIST;

	ProObj = si3217x_blobwrapper_proslic_init(_opermode, ctrl_ReadRegisterWrapper, ctrl_WriteRegisterWrapper,
					ctrl_ReadRAMWrapper, ctrl_WriteRAMWrapper, ctrl_ResetWrapper,
					time_DelayWrapper, time_TimeElapsedWrapper, time_GetTimeWrapper);

	if (!ProObj) {
		printk(KERN_ERR PFX "ProSLIC API initialization failed\n");
		return -ENODEV;
	}

	return 0;
}

void si3217x_proslic_free(void)
{
	if (ProObj) {
		si3217x_blobwrapper_proslic_free(ProObj);
		ProObj = NULL;
	}
}

void si3217x_set_linefeed_status(u8 newLinefeed)
{
	if (ProObj)
		si3217x_blobwrapper_set_linefeed_status(ProObj, newLinefeed);
}

int si3217x_get_hook_status(void)
{
	if (!ProObj)
		return -1;

	return si3217x_blobwrapper_get_hook_status(ProObj);
}

static int __init si3217x_init(void)
{
	int error;

	printk(KERN_INFO PFX DRV_DESC " version " DRV_VERSION "\n");
	printk(KERN_INFO PFX "Copyright (C) 2013-2014 Village Telco Ltd.\n");
	printk(KERN_INFO PFX "Copyright (C) 2013-2014 Dimitar Penev <dpn at switchvoice dot com>\n");
	printk(KERN_INFO PFX "Copyright (C) 2014 Vittorio Gambaletta <openwrt@vittgam.net>\n");

	if (!strcmp(opermode, "FCC"))
		_opermode = 0;
	else if (!strcmp(opermode, "TBR21"))
		_opermode = 1;
	else if (!strcmp(opermode, "TN12"))
		_opermode = 2;
	else if (!strcmp(opermode, "BT3"))
		_opermode = 3;
	else {
		printk(KERN_ERR PFX "Error: The opermode parameter should be one of: FCC, TBR21, TN12, BT3\n");
		return -ENODEV;
	}

	printk_dbg(PFX "Selected operation mode %s\n", opermode);

	error = si3217x_tdm_init();
	if (error < 0) {
		printk(KERN_ERR PFX "Failed to initialize the si3217x TDM driver\n");
		return error;
	}

	error = si3217x_spi_platform_device_init();
	if (error < 0) {
		printk(KERN_ERR PFX "Failed to register the si3217x SPI platform device driver\n");
		si3217x_tdm_exit();
		return error;
	}

	error = spi_register_driver(&si3217x_spi_driver);
	if (error < 0) {
		printk(KERN_ERR PFX "Failed to register the si3217x SPI protocol driver\n");
		si3217x_spi_platform_device_exit();
		si3217x_proslic_free();
		si3217x_tdm_exit();
		return error;
	}

	return 0;
}

static void __exit si3217x_exit(void)
{
	printk_dbg(PFX "Unloading module...\n");

	si3217x_dfxs_spidev_callback(NULL);

	spi_unregister_driver(&si3217x_spi_driver);
	si3217x_spi_platform_device_exit();

	si3217x_proslic_free();

	si3217x_tdm_exit();
}

module_init(si3217x_init);
module_exit(si3217x_exit);

module_param(debug, int, 0600);
module_param(opermode, charp, 0400);

MODULE_AUTHOR("Village Telco Ltd.");
MODULE_AUTHOR("Dimitar Penev <dpn at switchvoice dot com>");
MODULE_AUTHOR("Vittorio Gambaletta <openwrt@vittgam.net>");
MODULE_DESCRIPTION(DRV_DESC);
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:dragino2_si3217x");
