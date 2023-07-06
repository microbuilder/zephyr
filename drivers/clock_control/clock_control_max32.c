/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <mxc_sys.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>

#define DT_DRV_COMPAT adi_max32_gcr

static inline int max32_clock_control_on(const struct device *dev,
					 clock_control_subsys_t clkcfg)
{
	struct max32_perclk *perclk = (struct max32_perclk *)(clkcfg);

	ARG_UNUSED(dev);

	switch (perclk->bus) {
	case ADI_MAX32_CLOCK_BUS0:
		MXC_SYS_ClockEnable((mxc_sys_periph_clock_t)perclk->bit);
		break;
	case ADI_MAX32_CLOCK_BUS1:
		// to go perckcn1 register
		MXC_SYS_ClockEnable((mxc_sys_periph_clock_t)(perclk->bit + 32));
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static inline int max32_clock_control_off(const struct device *dev,
					  clock_control_subsys_t clkcfg)
{
	struct max32_perclk *perclk = (struct max32_perclk *)(clkcfg);

	ARG_UNUSED(dev);

	switch (perclk->bus) {
	case ADI_MAX32_CLOCK_BUS0:
		MXC_SYS_ClockDisable((mxc_sys_periph_clock_t)perclk->bit);
		break;
	case ADI_MAX32_CLOCK_BUS1:
		// to go perckcn1 register
		MXC_SYS_ClockDisable((mxc_sys_periph_clock_t)(perclk->bit + 32));
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct clock_control_driver_api max32_clock_control_api = {
	.on = max32_clock_control_on,
	.off = max32_clock_control_off,
};

static int max32_clock_control_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Enable desired clocks */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_hirc96m), okay)
	MXC_SYS_Clock_Select(MXC_SYS_CLOCK_HIRC96);
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(clk_hirc8m), okay)
	MXC_SYS_Clock_Select(MXC_SYS_CLOCK_HIRC8);
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(clk_hirc), okay)
	MXC_SYS_Clock_Select(MXC_SYS_CLOCK_HIRC);
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(clk_x32m), okay)
	MXC_SYS_Clock_Select(MXC_SYS_CLOCK_XTAL32M);
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(clk_x32k), okay)
	MXC_SYS_Clock_Select(MXC_SYS_CLOCK_XTAL32K);
#endif

	MXC_SYS_Clock_Div(MXC_SYS_SYSTEM_DIV_1);

	return 0;
}

DEVICE_DT_INST_DEFINE(0, max32_clock_control_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &max32_clock_control_api);
