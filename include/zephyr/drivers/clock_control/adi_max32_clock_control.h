/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ADI_MAX32_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ADI_MAX32_CLOCK_CONTROL_H_

#include <zephyr/drivers/clock_control.h>

#include <zephyr/dt-bindings/clock/adi_max32_clock.h>

#include <wrap_max32_sys.h>

/** Driver structure definition */

struct max32_perclk {
	uint32_t bus;
	uint32_t bit;
};

#define ADI_MAX32_CLK_IPO_FREQ    DT_PROP(DT_NODELABEL(clk_ipo), clock_frequency)
#define ADI_MAX32_CLK_ERFO_FREQ   DT_PROP(DT_NODELABEL(clk_erfo), clock_frequency)
#define ADI_MAX32_CLK_IBRO_FREQ   DT_PROP(DT_NODELABEL(clk_ibro), clock_frequency)
#define ADI_MAX32_CLK_ISO_FREQ    DT_PROP(DT_NODELABEL(clk_iso), clock_frequency)
#define ADI_MAX32_CLK_INRO_FREQ   DT_PROP(DT_NODELABEL(clk_inro), clock_frequency)
#define ADI_MAX32_CLK_ERTCO_FREQ  DT_PROP(DT_NODELABEL(clk_ertco), clock_frequency)
/* External clock may not be defined so _OR is used */
#define ADI_MAX32_CLK_EXTCLK_FREQ DT_PROP_OR(DT_NODELABEL(clk_extclk), clock_frequency, 0)

#define DT_GCR_CLOCKS_CTRL DT_CLOCKS_CTLR(DT_NODELABEL(gcr))

#if DT_SAME_NODE(DT_GCR_CLOCKS_CTRL, DT_NODELABEL(clk_ipo))
#define ADI_MAX32_SYSCLK_SRC ADI_MAX32_CLK_IPO
#endif
#if DT_SAME_NODE(DT_GCR_CLOCKS_CTRL, DT_NODELABEL(clk_erfo))
#define ADI_MAX32_SYSCLK_SRC ADI_MAX32_CLK_ERFO
#endif
#if DT_SAME_NODE(DT_GCR_CLOCKS_CTRL, DT_NODELABEL(clk_ibro))
#define ADI_MAX32_SYSCLK_SRC ADI_MAX32_CLK_IBRO
#endif
#if DT_SAME_NODE(DT_GCR_CLOCKS_CTRL, DT_NODELABEL(clk_iso))
#define ADI_MAX32_SYSCLK_SRC ADI_MAX32_CLK_ISO
#endif
#if DT_SAME_NODE(DT_GCR_CLOCKS_CTRL, DT_NODELABEL(clk_inro))
#define ADI_MAX32_SYSCLK_SRC ADI_MAX32_CLK_INRO
#endif
#if DT_SAME_NODE(DT_GCR_CLOCKS_CTRL, DT_NODELABEL(clk_ertco))
#define ADI_MAX32_SYSCLK_SRC ADI_MAX32_CLK_ERTCO
#endif
#if DT_SAME_NODE(DT_GCR_CLOCKS_CTRL, DT_NODELABEL(clk_extclk))
#define ADI_MAX32_SYSCLK_SRC ADI_MAX32_CLK_EXTCLK
#endif

#ifndef ADI_MAX32_SYSCLK_SRC
#define ADI_MAX32_SYSCLK_SRC ADI_MAX32_CLK_IPO
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ADI_MAX32_CLOCK_CONTROL_H_ */
