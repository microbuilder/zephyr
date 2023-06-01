/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <mxc_sys.h>


#define DT_DRV_COMPAT adi_max32_gcr

#define DT_GCR_CLOCK_SOURCE DT_CLOCKS_CTLR(DT_NODELABEL(gcr))

#define MAX32_CLOCK_BUS_PERCLKCN0 0
#define MAX32_CLOCK_BUS_PERCLKCN1 1


static inline int max32_clock_control_on(const struct device *dev,
					 clock_control_subsys_t clkcfg)
{
	uint32_t bus = (uint32_t)clkcfg >> 16;
	uint32_t bit = (uint32_t)clkcfg & 0xFFFF;

	switch (bus) {
	case MAX32_CLOCK_BUS_PERCLKCN0:
		// do nothing
		break;
	case MAX32_CLOCK_BUS_PERCLKCN1:
		bit += 32; // to go perckcn1 register
		break;
	default:
		return -EINVAL;
	}

	MXC_SYS_ClockEnable((mxc_sys_periph_clock_t)bit);

	return 0;
}

static inline int max32_clock_control_off(const struct device *dev,
					  clock_control_subsys_t clkcfg)
{
	uint32_t bus = (uint32_t)clkcfg >> 16;
	uint32_t bit = (uint32_t)clkcfg & 0xFFFF;

	switch (bus) {
	case MAX32_CLOCK_BUS_PERCLKCN0:
		// do nothing
		break;
	case MAX32_CLOCK_BUS_PERCLKCN1:
		bit += 32; // to go perckcn1 register
		break;
	default:
		return -EINVAL;
	}

	MXC_SYS_ClockDisable((mxc_sys_periph_clock_t)bit);

	return 0;
}

static struct clock_control_driver_api max32_clock_control_api = {
	.on = max32_clock_control_on,
	.off = max32_clock_control_off,
};

static int max32_clock_control_init(const struct device *dev)
{
	/* Enable desired clocks */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_hso), okay)
	MXC_SYS_Clock_Select(MXC_SYS_CLOCK_HIRC96);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_obrc), okay)
	MXC_SYS_Clock_Select(MXC_SYS_CLOCK_HIRC8);
#endif

	MXC_SYS_Clock_Div(MXC_SYS_SYSTEM_DIV_1);

	return 0;
}

DEVICE_DT_INST_DEFINE(0, max32_clock_control_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &max32_clock_control_api);
