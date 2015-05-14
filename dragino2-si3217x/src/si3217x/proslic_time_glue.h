/*
 * DRAGINO v2 si3217x FXS Daughter Board time functions
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

#ifndef __SI3217X_TIME
#define __SI3217X_TIME

#define UNREFERENCED_PARAMETER(P) (void)(P)
#include <linux/time.h>
#include <linux/delay.h>

static struct timespec diff(struct timespec *start, struct timespec *end)
{
	struct timespec temp;
	if ((end->tv_nsec-start->tv_nsec)<0) {
		temp.tv_sec = end->tv_sec-start->tv_sec-1;
		temp.tv_nsec = 1000000000+end->tv_nsec-start->tv_nsec;
	} else {
		temp.tv_sec = end->tv_sec-start->tv_sec;
		temp.tv_nsec = end->tv_nsec-start->tv_nsec;
	}

	return temp;
}

static int time_DelayWrapper(void *hTimer, int timeInMs)
{
	UNREFERENCED_PARAMETER(hTimer);

	mdelay(timeInMs);

	return 0;
}

static int time_TimeElapsedWrapper(void *hTimer, void *startTime, int *timeInMs)
{
	struct timespec now, time_diff;
	UNREFERENCED_PARAMETER(hTimer);

	do_posix_clock_monotonic_gettime(&now);
	time_diff=diff(&now, (struct timespec *)startTime);

	*timeInMs = (int)((time_diff.tv_sec)*1000+(time_diff.tv_nsec)/1000);

	return 0;
}

static int time_GetTimeWrapper(void *hTimer, void *time)
{
	struct timespec now, *time_ptr;
	UNREFERENCED_PARAMETER(hTimer);

	do_posix_clock_monotonic_gettime(&now);

	time_ptr = (struct timespec *)time;

	time_ptr->tv_sec = now.tv_sec;
	time_ptr->tv_nsec = now.tv_nsec;

	return 0;
}

#endif /* __SI3217X_TIME */
