/*
 * DRAGINO v2 si3217x FXS Daughter Board DAHDI driver
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

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>

#include "defs.h"
#include "dfxs.h"

struct dfxs wc;

/*
 * The work interrupt is a soft IRQ that can properly communicate with
 * the SPI kernel drivers and sleep without crashing the kernel
 */
static void work_hook_check_handler(struct work_struct *test);
static DECLARE_DELAYED_WORK(work_hook_check, work_hook_check_handler);

static void work_hook_check_handler(struct work_struct *test)
{
	int current_hook_status;

	current_hook_status = si3217x_get_hook_status();

	if (wc.last_hook_status != current_hook_status) {
		if (current_hook_status) {
			printk_dbg(PFX "Going off hook\n");
			dahdi_hooksig(&wc.chan, DAHDI_RXSIG_OFFHOOK);
		} else {
			printk_dbg(PFX "Going on hook\n");
			dahdi_hooksig(&wc.chan, DAHDI_RXSIG_ONHOOK);
		}

		wc.last_hook_status = current_hook_status;
	}

	/*u32 temp;
	si3217x_read_ram(SI3217X_MADC_ILOOP, &temp);
	printk_dbg(PFX "VittGam: madc_iloop: 0x%08x\n", temp);*/

	/* TODO VittGam: implement si3217x hook interrupt, if possible */
	schedule_delayed_work(&work_hook_check, msecs_to_jiffies(20));
}

/*
 * Schedule bit-bang SPI for si3217x out as they need to sleep
 */
static void work_set_hook_status_handler(struct work_struct *test);
static DECLARE_WORK(work_set_hook_status, work_set_hook_status_handler);

static void work_set_hook_status_handler(struct work_struct *test)
{
	int new_linefeed_status = wc.linefeed_status;
	u8 temp;

	if (wc.onhooktransmission) {
		if (new_linefeed_status == 1)
			new_linefeed_status = 2;
		else if (new_linefeed_status == 5)
			new_linefeed_status = 6;
	}

	if ((wc.vmwi_message_count && (wc.vmwisetting.vmwi_type & DAHDI_VMWI_LREV)) ? !wc.reverse_linefeed_polarity : wc.reverse_linefeed_polarity) {
		if (new_linefeed_status == 1)
			new_linefeed_status = 5;
		else if (new_linefeed_status == 2)
			new_linefeed_status = 6;
		else if (new_linefeed_status == 3)
			new_linefeed_status = 7;
		else if (new_linefeed_status == 5)
			new_linefeed_status = 1;
		else if (new_linefeed_status == 6)
			new_linefeed_status = 2;
		else if (new_linefeed_status == 7)
			new_linefeed_status = 3;
	}

	si3217x_read_reg(SI3217X_LINEFEED, &temp);

	if ((temp & 0xf) != ((temp >> 4) & 0xf)) {
		printk_dbg(PFX "Linefeed status is 0x%02x, rescheduling to set status to 0x%01x\n", temp, new_linefeed_status);
		schedule_work(&work_set_hook_status);
	/*} else if ((temp & 0xf) == new_linefeed_status) {
		printk_dbg(PFX "Linefeed status is 0x%02x, skipping update to same status = 0x%01x\n", temp, new_linefeed_status);
	*/} else {
		si3217x_set_linefeed_status(new_linefeed_status);
		printk_dbg(PFX "Linefeed status was 0x%02x, is now 0x%01x\n", temp, new_linefeed_status);
	}
}

static void work_reset_oht_handler(struct work_struct *test)
{
	printk_dbg(PFX "resetting OHT condition\n");

	wc.onhooktransmission = 0;
	schedule_work(&work_set_hook_status);
}
static DECLARE_DELAYED_WORK(work_reset_oht, work_reset_oht_handler);

static int dfxs_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data)
{
	if (!wc.initialized)
		return -ENODEV;

	switch (cmd) {
		case DAHDI_ONHOOKTRANSFER:
		{
			int i;

			if (get_user(i, (__user int *) data))
				return -EFAULT;

			if (wc.last_hook_status) {
				printk_dbg(PFX "ioctl DAHDI_ONHOOKTRANSFER received while off-hook, ohttimer = %d\n", i);
				return -EINVAL;
			}

			printk_dbg(PFX "ioctl DAHDI_ONHOOKTRANSFER, ohttimer = %d\n", i);

			/*
			 * Note: this ioctl is meant for transmitting data (like
			 * DTMF caller ID) while being on-hook. So we need to
			 * modify the state of the SLIC to On-Hook Transmission
			 * for the time specified by the parameter in milliseconds.
			 */

			wc.onhooktransmission = 1;
			schedule_work(&work_set_hook_status);
			schedule_delayed_work(&work_reset_oht, msecs_to_jiffies(i)); /* use ohttimer to reset it after i milliseconds */

			break;
		}

		case DAHDI_VMWI_CONFIG:
		{
			if (copy_from_user(&(wc.vmwisetting), (__user void *) data, sizeof(wc.vmwisetting)))
				return -EFAULT;
			printk_dbg(PFX "ioctl DAHDI_VMWI_CONFIG, vmwi_type = %u\n", wc.vmwisetting.vmwi_type);
			schedule_work(&work_set_hook_status);
			break;
		}

		case DAHDI_VMWI:
		{
			int i;
			if (get_user(i, (__user int *) data))
				return -EFAULT;
			if (i < 0)
				return -EFAULT;
			printk_dbg(PFX "ioctl DAHDI_VMWI, message count = %d\n", i);
			wc.vmwi_message_count = i;
			schedule_work(&work_set_hook_status);
			break;
		}

		case DAHDI_SET_HWGAIN:
		{
			printk_dbg(PFX "ioctl DAHDI_SET_HWGAIN, not supported\n");
			return -ENOSYS;
		}

		case DAHDI_TONEDETECT:
		{
			printk_dbg(PFX "ioctl DAHDI_TONEDETECT, not supported\n");
			return -ENOSYS;
		}

		case DAHDI_SETPOLARITY:
		{
			int i;
			if (get_user(i, (__user int *) data))
				return -EFAULT;
			printk_dbg(PFX "ioctl DAHDI_SETPOLARITY, status = %d\n", i);
			wc.reverse_linefeed_polarity = i ? 1 : 0;
			schedule_work(&work_set_hook_status);
			break;
		}

		case DFXS_GET_STATS:
		{
			struct dfxs_stats stats;
			u32 temp;

			printk_dbg(PFX "ioctl DFXS_GET_STATS\n");

			si3217x_read_ram(SI3217X_MADC_VTIPC, &temp);
			stats.tipvolt = (int)((temp*SI3217X_VOLTRES)>>20);

			si3217x_read_ram(SI3217X_MADC_VRINGC, &temp);
			stats.ringvolt = (int)((temp*SI3217X_VOLTRES)>>20);

			si3217x_read_ram(SI3217X_MADC_VBAT, &temp);
			stats.batvolt = (int)((temp*SI3217X_VOLTRES)>>20);

			if (copy_to_user((struct dfxs_stats *)data, &stats, sizeof(stats)))
				return -EFAULT;

			break;
		}

		case DFXS_GET_REGS:
		{
			struct dfxs_mem mem;
			int i;

			printk_dbg(PFX "ioctl DFXS_GET_REGS\n");

			for (i = 0; i < NUM_REGS; i++)
				si3217x_read_reg(i, &(mem.reg[i]));

			for (i = 0; i < NUM_RAMS; i++)
				si3217x_read_ram(i, &(mem.ram[i]));

			if (copy_to_user((struct mem *)data, &mem, sizeof(mem)))
				return -EFAULT;

			break;
		}

		case DFXS_GET_REG:
		{
			struct dfxs_regop regop;

			if (copy_from_user(&regop, (__user struct dfxs_regop *) data, sizeof(regop)))
				return -EFAULT;

			si3217x_read_reg(regop.address, &regop.val);

			printk_dbg(PFX "ioctl DFXS_GET_REG, got registry %d value 0x%02x\n", regop.address, regop.val);

			if (copy_to_user((struct dfxs_regop *)data, &regop, sizeof(regop)))
				return -EFAULT;

			break;
		}

		case DFXS_SET_REG:
		{
			struct dfxs_regop regop;

			if (copy_from_user(&regop, (__user struct dfxs_regop *) data, sizeof(regop)))
				return -EFAULT;

			printk_dbg(PFX "ioctl DFXS_SET_REG, setting registry %d to 0x%02x\n", regop.address, regop.val);

			si3217x_write_reg(regop.address, regop.val);

			break;
		}

		case DFXS_GET_RAM:
		{
			struct dfxs_ramop ramop;

			if (copy_from_user(&ramop, (__user struct dfxs_ramop *) data, sizeof(ramop)))
				return -EFAULT;

			si3217x_read_ram(ramop.address, &ramop.val);

			printk_dbg(PFX "ioctl DFXS_GET_RAM, got RAM %d value 0x%08x\n", ramop.address, ramop.val);

			if (copy_to_user((struct dfxs_ramop *)data, &ramop, sizeof(ramop)))
				return -EFAULT;

			break;
		}

		case DFXS_SET_RAM:
		{
			struct dfxs_ramop ramop;

			if (copy_from_user(&ramop, (__user struct dfxs_ramop *) data, sizeof(ramop)))
				return -EFAULT;

			printk_dbg(PFX "ioctl DFXS_SET_RAM, setting RAM %d to 0x%08x\n", ramop.address, ramop.val);

			si3217x_write_ram(ramop.address, ramop.val);

			break;
		}

		default:
			printk_dbg(PFX "ioctl %u not implemented\n", cmd);
			return -ENOTTY;
	}

	return 0;
}

static int dfxs_open(struct dahdi_chan *chan)
{
	if (!wc.initialized)
		return -ENODEV;

	return 0;
}

static int dfxs_watchdog(struct dahdi_span *span, int event)
{
	printk_dbg(PFX "dfxs_watchdog called\n");
	return 0;
}

static int dfxs_close(struct dahdi_chan *chan)
{
	return 0;
}

static int dfxs_hooksig(struct dahdi_chan *chan, enum dahdi_txsig txsig)
{
	if (!wc.initialized)
		return -ENODEV;

	switch (txsig) {
		case DAHDI_TXSIG_ONHOOK:
			printk_dbg(PFX "DAHDI_TXSIG_ONHOOK\n");

			switch (chan->sig) {
				case DAHDI_SIG_EM:
				case DAHDI_SIG_FXOKS:
				case DAHDI_SIG_FXOLS:
					wc.linefeed_status = 1; /* Forward Active */
					break;
				case DAHDI_SIG_FXOGS:
					wc.linefeed_status = 3; /* Tip Open */
					break;
			}
			break;

		case DAHDI_TXSIG_OFFHOOK:
			printk_dbg(PFX "DAHDI_TXSIG_OFFHOOK\n");

			switch (chan->sig) {
				case DAHDI_SIG_EM:
					wc.linefeed_status = 6; /* Reverse OHT */
					break;
				default:
					wc.linefeed_status = 2; /* Forward OHT */
					break;
			}
			break;

		case DAHDI_TXSIG_START:
			printk_dbg(PFX "DAHDI_TXSIG_START\n");

			if (wc.last_hook_status) {
				printk_dbg(PFX "Tried to set ringing state while off-hook\n");
				return -EINVAL;
			}

			wc.linefeed_status = 4; /* Ringing */
			break;
		case DAHDI_TXSIG_KEWL:
			printk_dbg(PFX "DAHDI_TXSIG_KEWL\n");

			wc.linefeed_status = 0; /* Open */
			break;
		default:
			printk_dbg(PFX "Unrecognized tx state %d\n", txsig);
			return -ENOTTY;
	}

	/* Schedule bit-bang SPI for si3217x out of this context as they need to sleep */
	schedule_work(&work_set_hook_status);

	return 0;
}

static const struct dahdi_span_ops dfxs_span_ops = {
	.owner = THIS_MODULE,
	.hooksig = dfxs_hooksig,
	.open = dfxs_open,
	.close = dfxs_close,
	.ioctl = dfxs_ioctl,
	.watchdog = dfxs_watchdog
};

static int dfxs_init_one(struct spi_device *spidev)
{
	printk_dbg(PFX "Initializing DAHDI driver...\n");

	memset(&wc, 0, sizeof(struct dfxs));
	wc.last_hook_status = 1; /* Force detection to notify DAHDI of the first onhook state */

	wc.ddev = dahdi_create_device();

	snprintf(wc.span.name, sizeof(wc.span.name) - 1, "DFXS/0");
	snprintf(wc.span.desc, sizeof(wc.span.desc) - 1, DEVICE_DESC);
	wc.span.deflaw = DAHDI_LAW_MULAW;

	wc.ddev->location = "DRAGINO v2 Daughter Board slot";

	snprintf(wc.chan.name, sizeof(wc.chan.name) - 1, "DFXS/0/0");
	wc.chan.sigcap = DAHDI_SIG_FXOKS | DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS | DAHDI_SIG_SF | DAHDI_SIG_EM | DAHDI_SIG_CLEAR;
	wc.chan.chanpos = 1;
	wc._chan = &(wc.chan);

	wc.ddev->devicetype = "DRAGINO v2 FXS Daughter Board";
	wc.ddev->manufacturer = "Switchvoice, VittGam";

	wc.span.chans = &(wc._chan);
	wc.span.channels = 1;
	wc.span.flags = DAHDI_FLAG_RBS;
	wc.span.spantype = SPANTYPE_ANALOG_FXS;
	wc.span.linecompat = 0;

	wc.span.ops = &dfxs_span_ops;

	list_add_tail(&wc.span.device_node, &wc.ddev->spans);
	if (dahdi_register_device(wc.ddev, &spidev->dev)) {
		printk(KERN_ERR PFX "Unable to register span with DAHDI\n");
		return -EIO;
	}

	if (si3217x_proslic_init()) {
		printk(KERN_ERR PFX "Unable to initialize si3217x hardware\n");
		dahdi_unregister_device(wc.ddev);
		return -EIO;
	}

	/* Put the SLIC in Forward Active mode */
	wc.linefeed_status = 1;
	schedule_work(&work_set_hook_status);

	schedule_delayed_work(&work_hook_check, msecs_to_jiffies(20));

	wc.initialized = 1;

	printk(KERN_INFO PFX DEVICE_DESC " initialized successfully\n");

	return 0;
}

static void dfxs_release(void)
{
	printk_dbg(PFX "Releasing...\n");

	wc.initialized = 0;

	cancel_delayed_work(&work_hook_check);
	cancel_work_sync(&work_set_hook_status);
	cancel_delayed_work(&work_reset_oht);

	dahdi_unregister_device(wc.ddev);

	si3217x_proslic_free();

	printk(KERN_INFO PFX DEVICE_DESC " released successfully\n");
}

void si3217x_dfxs_spidev_callback(struct spi_device *spidev)
{
	if (spidev && !wc.initialized)
		dfxs_init_one(spidev);
	else if (!spidev && wc.initialized)
		dfxs_release();
}
