/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for MAX32xxx MCUs
 */

#include <zephyr/device.h>
#include <zephyr/init.h>

#include <wrap_max32_sys.h>

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int max32xxx_init(void)
{
	/* Apply device related preinit configuration */
	max32xx_system_init();

	return 0;
}

SYS_INIT(max32xxx_init, PRE_KERNEL_1, 0);
