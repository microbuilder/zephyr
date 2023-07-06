/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ADI_MAX32_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ADI_MAX32_CLOCK_CONTROL_H_

#include <zephyr/drivers/clock_control.h>

#include <zephyr/dt-bindings/clock/adi_max32_clock.h>

/** Driver structure definition */

struct max32_perclk {
	uint32_t bus;
	uint32_t bit;
};



#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ADI_MAX32_CLOCK_CONTROL_H_ */
