/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_rtc

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/spinlock.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/timeutil.h>

#include <rtc.h>
#include <time.h>

#define MSEC_TO_NSEC(x) (x * 1000000)

struct rtc_max32_data {
	struct k_spinlock lock;
	uint16_t alarms_count;
	uint16_t mask;
	bool alarm_pending;
	rtc_alarm_callback cb;
	void *cb_data;
	rtc_update_callback update_cb;
	void *update_cb_data;
};

static int rtc_max32_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	ARG_UNUSED(dev);
	if (timeptr == NULL) {
		// validate time method would be better
		return -EINVAL;
	}
	/* Convert rtc_time to sec and subsec*/
	MXC_RTC_Init(timeutil_timegm((struct tm *)timeptr), 0);
	MXC_RTC_Start();

	return 0;
}

static int rtc_max32_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	ARG_UNUSED(dev);
	uint32_t sec, subsec_reg;
	double subsec;
	int ret;

	do {
		ret = MXC_RTC_GetTime(&sec, &subsec_reg);
	} while (ret != E_NO_ERROR);
	subsec = subsec_reg / 4096.0;

	gmtime_r((time_t *)&sec, (struct tm *)(timeptr));
	timeptr->tm_nsec = MSEC_TO_NSEC(subsec);
	timeptr->tm_isdst = -1;

	return 0;
}

struct rtc_driver_api rtc_max32_driver_api = {
	.set_time = rtc_max32_set_time,
	.get_time = rtc_max32_get_time,
};

static int rtc_max32_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* Set time to initial time */
	return MXC_RTC_Init(0, 0);
}

#define RTC_MAX32_INIT(n)                                                                          \
	static struct rtc_max32_data rtc_data_##n = {                                              \
		.alarms_count = DT_INST_PROP(n, alarms_count),                                     \
		.mask = 0,                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &rtc_max32_init, NULL, &rtc_data_##n, NULL, POST_KERNEL,          \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &rtc_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_MAX32_INIT)
