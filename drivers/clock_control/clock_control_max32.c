/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <mxc_sys.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>

#define DT_DRV_COMPAT adi_max32_gcr


#if defined(CONFIG_SOC_MAX32665) || (CONFIG_SOC_MAX32666)
#define z_sysclk_prescaler(v) MXC_SYS_SYSTEM_DIV_ ## v
#else
#define z_sysclk_prescaler(v) MXC_SYS_CLOCK_DIV_ ## v
#endif
#define sysclk_prescaler(v) z_sysclk_prescaler(v)


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

static void setup_fixed_clocks(void)
{
	if (IS_ENABLED(ADI_MAX32_CLK_IPO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_IPO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_IPO);
	}

	if (IS_ENABLED(ADI_MAX32_CLK_ERFO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_ERFO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_ERFO);
	}

	if (IS_ENABLED(ADI_MAX32_CLK_IBRO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_IBRO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_IBRO);
	}

	if (IS_ENABLED(ADI_MAX32_CLK_ISO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_ISO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_ISO);
	}

	if (IS_ENABLED(ADI_MAX32_CLK_INRO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_INRO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_INRO);
	}

	if (IS_ENABLED(ADI_MAX32_CLK_ERTCO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_ERTCO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_ERTCO);
	}

#ifndef CONFIG_SOC_MAX32666
	if (IS_ENABLED(ADI_MAX32_CLK_EXTCLK_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_EXTCLK);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_EXTCLK);
	}
#endif

	return;
}

static int max32_clock_control_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	// Setup fixed clocks if enabled
	setup_fixed_clocks();

	// Setup device clock source
	MXC_SYS_Clock_Select(ADI_MAX32_SYSCLK_SRC);
	// Setup divider
#if defined(CONFIG_SOC_MAX32665) || (CONFIG_SOC_MAX32666)
	MXC_SYS_Clock_Div(sysclk_prescaler(ADI_MAX32_SYSCLK_PRESCALER));
#elif defined(CONFIG_SOC_MAX32655)
	MXC_SYS_SetClockDiv(sysclk_prescaler(ADI_MAX32_SYSCLK_PRESCALER));
#endif

	return 0;
}

DEVICE_DT_INST_DEFINE(0,\
	max32_clock_control_init,\
	NULL,\
	NULL,\
	NULL,\
	PRE_KERNEL_1,\
	CONFIG_CLOCK_CONTROL_INIT_PRIORITY,\
	&max32_clock_control_api);
