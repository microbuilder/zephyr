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

#include <rtc.h>

#include <time.h>

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
	ARG_UNUSED(timeptr);

	return 0;
}

static int rtc_max32_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeptr);

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
