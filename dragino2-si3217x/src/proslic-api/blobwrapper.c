/*
 * DRAGINO v2 si3217x FXS Daughter Board - ProSLIC API binary blob wrapper
 * Copyright (C) 2014 Village Telco Ltd. All rights reserved.
 *
 * This wrapper is meant to give the end user the possibility to
 * use the ProSLIC API binary blob on his/her own compiled kernel,
 * without the user needing to sign an NDA with Silicon Labs in
 * order to get access to the relative source code.
 * It is the same method used by eg. video card makers to
 * distribute binary blob drivers in a kernel-agnostic way.
 * The ProSLIC API is merely defining the way to talk to the
 * ProSLIC devices, and has nothing to do with the Linux kernel;
 * so it clearly is not a derived work of the kernel.
 * The other parts of the MP2 FXS driver that are in the
 * ../si3217x folder are all open sourced under the GPLv2.
 */

#include <linux/module.h>
#include <linux/slab.h>

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

void *si3217x_proslicglue_memalloc(int size)
{
	return kzalloc(size, GFP_KERNEL);
}

void si3217x_proslicglue_memfree(const void *p)
{
	if (p)
		kfree(p);
}

int si3217x_proslicglue_logprint(const char *f, ...)
{
	int ret;
	va_list valist;
	va_start(valist, f);
	ret = vprintk(f, valist);
	va_end(valist);
	return ret;
}

MODULE_AUTHOR("Silicon Labs (ProSLIC API)");
MODULE_DESCRIPTION("ProSLIC API for DRAGINO v2 si3217x FXS Daughter Board");
MODULE_LICENSE("Proprietary");
EXPORT_SYMBOL(si3217x_blobwrapper_proslic_init);
EXPORT_SYMBOL(si3217x_blobwrapper_proslic_free);
EXPORT_SYMBOL(si3217x_blobwrapper_set_linefeed_status);
EXPORT_SYMBOL(si3217x_blobwrapper_get_hook_status);
