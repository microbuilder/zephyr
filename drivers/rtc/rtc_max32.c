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
	struct rtc_time alarm_time;
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

#if defined(CONFIG_RTC_ALARM)
static int rtc_max32_alarm_get_supported_fields(const struct device *dev, uint16_t id,
						uint16_t *mask)
{
	struct rtc_max32_data * const dev_data = dev->data;

	if (dev_data->alarms_count <= id) {
		return -EINVAL;
	}

	(*mask) = (RTC_ALARM_TIME_MASK_SECOND
		   | RTC_ALARM_TIME_MASK_MINUTE
		   | RTC_ALARM_TIME_MASK_HOUR
		   | RTC_ALARM_TIME_MASK_MONTHDAY
		   | RTC_ALARM_TIME_MASK_MONTH);

	return 0;
}

static int rtc_max32_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				   const struct rtc_time *timeptr)
{
	struct rtc_max32_data * const dev_data = dev->data;

	if (dev_data->alarms_count <= id) {
		return -EINVAL;
	}

	if ((mask > 0) && (timeptr == NULL)) {
		return -EINVAL;
	}

	dev_data->mask = mask;
	dev_data->alarm_time = *timeptr;

	struct rtc_time *now_time;
	struct time_t now_secs, secs, diff_secs;

	rtc_max32_get_time(dev, now_time);
	gmtime_r((time_t *)&now_secs, (struct tm *)(now_time));
	gmtime_r((time_t *)&secs, (struct tm *)(timeptr));

	diff_secs = difftime(secs, now_secs);

	MXC_RTC_SetTimeofdayAlarm((uint32_t)diff_secs);

	return 0;
}

static int rtc_max32_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
				   struct rtc_time *timeptr)
{
	struct rtc_max32_data * const dev_data = dev->data;

	if (dev_data->alarms_count <= id) {
		return -EINVAL;
	}

	if (timeptr == NULL) {
		return -EINVAL
	}

	*timeptr = dev_data->alarm_time;
	*mask = dev_data->mask;

	return 0;
}

static int rtc_max32_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct rtc_max32_data * const dev_data = dev->data;
	int ret;

	if (dev_data->alarms_count <= id) {
		return -EINVAL;
	}

	ret = (dev_data->alarm_pending == true) ? 1 : 0;
	dev_data->alarm_pending = false;

	return ret;
}
#endif /* CONFIG_RTC_ALARM */

static void rtc_max32_isr(const struct device *dev)
{
	struct rtc_max32_data * const dev_data = dev->data;

	ARG_UNUSED(dev_data);

#if defined(CONFIG_RTC_ALARM)
	if (!MXC_RTC_RevA_GetBusyFlag()) {
		if (dev_data->cb) {
			dev_data->cb(dev, 0, dev_data->cb_data);
		}
		dev_data->alarm_pending = false;
	}
#endif

}

struct rtc_driver_api rtc_max32_driver_api = {
	.set_time = rtc_max32_set_time,
	.get_time = rtc_max32_get_time,
#if defined(CONFIG_RTC_ALARM)
	.alarm_get_supported_fields = rtc_max32_alarm_get_supported_fields,
	.alarm_set_time = rtc_max32_alarm_set_time,
	.alarm_get_time = rtc_max32_alarm_get_time,
	.alarm_is_pending = rtc_max32_alarm_is_pending,
#endif /* CONFIG_RTC_ALARM */
};

static int rtc_max32_init(const struct device *dev)
{
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
