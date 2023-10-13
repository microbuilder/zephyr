/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>

#include <wrap_max32_sys.h>

#define DT_DRV_COMPAT adi_max32_gcr

/** Get prescaler value if it defined  */
#define ADI_MAX32_SYSCLK_PRESCALER DT_INST_PROP_OR(0, sysclk_prescaler, 0)

static inline int api_on(const struct device *dev, clock_control_subsys_t clkcfg)
{
	ARG_UNUSED(dev);
	struct max32_perclk *perclk = (struct max32_perclk *)(clkcfg);

	switch (perclk->bus) {
	case ADI_MAX32_CLOCK_BUS0:
		MXC_SYS_ClockEnable((mxc_sys_periph_clock_t)perclk->bit);
		break;
	case ADI_MAX32_CLOCK_BUS1:
		MXC_SYS_ClockEnable((mxc_sys_periph_clock_t)(perclk->bit + 32));
		break;
	case ADI_MAX32_CLOCK_BUS2:
		MXC_SYS_ClockEnable((mxc_sys_periph_clock_t)(perclk->bit + 64));
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static inline int api_off(const struct device *dev, clock_control_subsys_t clkcfg)
{
	ARG_UNUSED(dev);
	struct max32_perclk *perclk = (struct max32_perclk *)(clkcfg);

	switch (perclk->bus) {
	case ADI_MAX32_CLOCK_BUS0:
		MXC_SYS_ClockDisable((mxc_sys_periph_clock_t)perclk->bit);
		break;
	case ADI_MAX32_CLOCK_BUS1:
		MXC_SYS_ClockDisable((mxc_sys_periph_clock_t)(perclk->bit + 32));
		break;
	case ADI_MAX32_CLOCK_BUS2:
		MXC_SYS_ClockDisable((mxc_sys_periph_clock_t)(perclk->bit + 64));
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct clock_control_driver_api max32_clkctrl_api = {
	.on = api_on,
	.off = api_off,
};

static void setup_fixed_clocks(void)
{
#if DT_NODE_HAS_COMPAT(DT_NODELABEL(clk_extclk), fixed_clock)
	MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_EXTCLK);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_ipo), okay)
	MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_IPO);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_erfo), okay)
	MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_ERFO);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_ibro), okay)
	MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_IBRO);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_iso), okay)
	MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_ISO);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_inro), okay)
	MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_INRO);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_ertco), okay)
	MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_ERTCO);
#endif

/* Some device does not support external clock */
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_extclk), fixed_clock, okay)
	MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_EXTCLK);
#endif
}

static int max32_clkctrl_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Setup fixed clocks if enabled */
	setup_fixed_clocks();

	/* Setup device clock source */
	MXC_SYS_Clock_Select(ADI_MAX32_SYSCLK_SRC);

#if DT_NODE_HAS_PROP(DT_NODELABEL(gcr), sysclk_prescaler)
	/* Setup divider */
	Wrap_MXC_SYS_SetClockDiv(sysclk_prescaler(ADI_MAX32_SYSCLK_PRESCALER));
#endif

	return 0;
}

DEVICE_DT_INST_DEFINE(0, max32_clkctrl_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &max32_clkctrl_api);
