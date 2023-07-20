/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_timer

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>
#include <soc.h>

// void sys_clock_set_timeout(int32_t ticks, bool idle)
// {

// }

// uint32_t sys_clock_elapsed(void)
// {
// 	uint32_t ret = 0;

// 	return ret;
// }

// uint32_t sys_clock_cycle_get_32(void)
// {
// 	uint32_t ret = 0;

// 	return ret;
// }

uint64_t sys_clock_cycle_get_64(void)
{
	uint64_t ret = 0;

	return ret;
}

static int sys_clock_driver_init(void)
{

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
