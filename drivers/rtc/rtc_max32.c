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

#define RTC_CFG(dev) ((struct max32_rtc_config *) ((dev)->config))
#define RTC_DATA(dev) ((struct max32_rtc_data *) ((dev)->data))

#define MSEC_TO_NSEC(x) ((x) * 1000000)

/* Converts a time in milleseconds to the equivalent RSSA register value. */
#define NSEC_TO_MSEC(x)  (x / 1000000) 
#define NSEC_TO_RSSA(x)  (0 - ((NSEC_TO_MSEC(x) * 4096) /  1000)) 

struct max32_rtc_data {
	struct k_spinlock lock;
	uint16_t alarms_count;
	uint16_t mask;
	bool alarm_pending;
	rtc_alarm_callback cb;
	void *cb_data;
	struct rtc_time alarm_time;
};

struct max32_rtc_config {
	mxc_rtc_regs_t *regs;
#ifdef CONFIG_RTC_ALARM
	void (*irq_func)(void);
#endif /* CONFIG_RTC_ALARM */
};


static inline time_t convert_to_second(const struct rtc_time *timeptr)
{
	struct tm stm;
	time_t sec;

	stm.tm_sec = timeptr->tm_sec;
	stm.tm_min = timeptr->tm_min;
	stm.tm_hour = timeptr->tm_hour;
	stm.tm_mday = timeptr->tm_mday;
	stm.tm_mon = timeptr->tm_mon;
	stm.tm_year = timeptr->tm_year;
	stm.tm_wday = timeptr->tm_wday;	
	stm.tm_yday = timeptr->tm_yday;	
	stm.tm_isdst = timeptr->tm_isdst;

	sec = timeutil_timegm(&stm); /* Convert rtc_time to seconds*/
	
	return sec;
}

static inline void convert_to_rtc_time(uint32_t sec, uint32_t subsec, struct rtc_time *timeptr)
{
	time_t tm_t;
	struct tm stm;

	tm_t = sec;
	gmtime_r(&tm_t, &stm);

	timeptr->tm_sec = stm.tm_sec;
	timeptr->tm_min = stm.tm_min;
	timeptr->tm_hour = stm.tm_hour;
	timeptr->tm_mday = stm.tm_mday;
	timeptr->tm_mon = stm.tm_mon;
	timeptr->tm_year = stm.tm_year;
	timeptr->tm_wday = stm.tm_wday;
	timeptr->tm_yday = stm.tm_yday;
	//timeptr->tm_isdst = stm.tm_isdst;
	timeptr->tm_isdst = -1;
	// add subsec too
	timeptr->tm_nsec = MSEC_TO_NSEC(subsec / 4.096); // 4096 count equal to 1sec
}

static int api_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	ARG_UNUSED(dev);
	if (timeptr == NULL) {
		return -EINVAL;
	}

	time_t sec = convert_to_second(timeptr);
	if (sec == -1) {
		return -1;
	}

	MXC_RTC_Init((uint32_t)sec, NSEC_TO_RSSA(timeptr->tm_nsec));
	MXC_RTC_Start();

	return 0;
}

static int api_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	ARG_UNUSED(dev);
	uint32_t sec, subsec;

	while (E_NO_ERROR != MXC_RTC_GetTime(&sec, &subsec)) {
		;
	} 

	convert_to_rtc_time(sec, subsec, timeptr);

	return 0;
}

#ifdef CONFIG_RTC_ALARM
static int api_alarm_get_supported_fields(const struct device *dev, uint16_t id,
						uint16_t *mask)
{
	if (RTC_DATA(dev)->alarms_count <= id) {
		return -EINVAL;
	}

	(*mask) = (RTC_ALARM_TIME_MASK_SECOND
		   | RTC_ALARM_TIME_MASK_MINUTE
		   | RTC_ALARM_TIME_MASK_HOUR
		   | RTC_ALARM_TIME_MASK_MONTHDAY
		   | RTC_ALARM_TIME_MASK_MONTH);

	return 0;
}

static int api_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				   const struct rtc_time *timeptr)
{
	struct max32_rtc_data * const dev_data = dev->data;
	uint32_t crr_sec, crr_subsec;

	if (dev_data->alarms_count <= id) {
		return -EINVAL;
	}

	if (timeptr == NULL) {
		return -EINVAL;
	}

	if (mask == 0) { // means disable alarm
		while (MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG) == E_BUSY) {
			;
		}
		return 0;
	}

	dev_data->mask = mask;
	dev_data->alarm_time = *timeptr;

	while (E_NO_ERROR != MXC_RTC_GetTime(&crr_sec, &crr_subsec)) {
		;
	} 

	time_t alarm_sec = convert_to_second(timeptr);

	if ((uint32_t)alarm_sec <= crr_sec) {
		return EINVAL;
	}

	while (MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG) == E_BUSY) {}

	MXC_RTC_SetTimeofdayAlarm(alarm_sec);

	while (MXC_RTC_EnableInt(MXC_RTC_INT_EN_LONG) == E_BUSY) {}

	return 0;
}

static int api_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
				   struct rtc_time *timeptr)
{
	struct max32_rtc_data * const dev_data = dev->data;

	if (dev_data->alarms_count <= id) {
		return -EINVAL;
	}

	if (timeptr == NULL) {
		return -EINVAL;
	}

	*timeptr = dev_data->alarm_time;
	*mask = dev_data->mask;

	return 0;
}

static int api_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct max32_rtc_data * const dev_data = dev->data;
	int ret;

	if (dev_data->alarms_count <= id) {
		return -EINVAL;
	}

	ret = (dev_data->alarm_pending == true) ? 1 : 0;
	dev_data->alarm_pending = false;

	return ret;
}

static void rtc_max32_isr(const struct device *dev)
{
	struct max32_rtc_data * const dev_data = dev->data;
	int flags;

	flags = MXC_RTC_GetFlags();

	if (dev_data->cb) {
		dev_data->cb(dev, 0, dev_data->cb_data);
	}

	dev_data->alarm_pending = false;
	MXC_RTC_ClearFlags(flags);
}
#endif

struct rtc_driver_api rtc_max32_driver_api = {
	.set_time = api_set_time,
	.get_time = api_get_time,
#if defined(CONFIG_RTC_ALARM)
	.alarm_get_supported_fields = api_alarm_get_supported_fields,
	.alarm_set_time = api_alarm_set_time,
	.alarm_get_time = api_alarm_get_time,
	.alarm_is_pending = api_alarm_is_pending,
#endif /* CONFIG_RTC_ALARM */
};

static int rtc_max32_init(const struct device *dev)
{
	// start timer
	while(MXC_RTC_Start() == E_BUSY) {
        ;
    }

#ifdef CONFIG_RTC_ALARM
	RTC_CFG(dev)->irq_func();
#endif
	return 0;
}

#define RTC_MAX32_INIT(_num) \
	IF_ENABLED(CONFIG_RTC_ALARM, \
	(static void max32_rtc_irq_init_##_num(void) \
	{ \
		IF_ENABLED(CONFIG_RTC_ALARM, ( \
			IRQ_CONNECT(\
				DT_INST_IRQN(_num), \
			    DT_INST_IRQ(_num, priority), \
			    rtc_max32_isr, \
			    DEVICE_DT_INST_GET(_num), \
			    0); \
			irq_enable(DT_INST_IRQN(_num))) \
		); \
	})); \
	static const struct max32_rtc_config rtc_max32_config_##_num = { \
		.regs = (mxc_rtc_regs_t *)DT_INST_REG_ADDR(_num), \
		IF_ENABLED(CONFIG_RTC_ALARM, \
			(.irq_func = max32_rtc_irq_init_##_num,)) \
	}; \
	static struct max32_rtc_data rtc_data_##_num = { \
		.alarms_count = DT_INST_PROP(_num, alarms_count), \
		.mask = 0, \
	}; \
	\
	DEVICE_DT_INST_DEFINE(_num, \
		&rtc_max32_init, \
		NULL, \
		&rtc_data_##_num, \
		&rtc_max32_config_##_num, \
		POST_KERNEL, \
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
		&rtc_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_MAX32_INIT)
