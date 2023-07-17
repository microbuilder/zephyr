/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ADI_MAX32_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ADI_MAX32_CLOCK_CONTROL_H_

#include <zephyr/drivers/clock_control.h>

#include <zephyr/dt-bindings/clock/adi_max32_clock.h>

#include <mxc_sys.h>

/** Driver structure definition */

struct max32_perclk {
    uint32_t bus;
    uint32_t bit;
};

/*
 *  Which clock enabled?
 */
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_ipo), fixed_clock, okay)
#define ADI_MAX32_CLK_IPO_ENABLED 1
#define ADI_MAX32_CLK_IPO_FREQ DT_PROP(DT_NODELABEL(clk_ipo), clock_frequency)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_erfo), fixed_clock, okay)
#define ADI_MAX32_CLK_ERFO_ENABLED 1
#define ADI_MAX32_CLK_ERFO_FREQ DT_PROP(DT_NODELABEL(clk_erfo), clock_frequency)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_ibro), fixed_clock, okay)
#define ADI_MAX32_CLK_IBRO_ENABLED 1
#define ADI_MAX32_CLK_IBRO_FREQ DT_PROP(DT_NODELABEL(clk_ibro), clock_frequency)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_iso), fixed_clock, okay)
#define ADI_MAX32_CLK_ISO_ENABLED 1
#define ADI_MAX32_CLK_ISO_FREQ DT_PROP(DT_NODELABEL(clk_iso), clock_frequency)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_inro), fixed_clock, okay)
#define ADI_MAX32_CLK_INRO_ENABLED 1
#define ADI_MAX32_CLK_INRO_FREQ DT_PROP(DT_NODELABEL(clk_inro), clock_frequency)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_ertco), fixed_clock, okay)
#define ADI_MAX32_CLK_ERTCO_ENABLED 1
#define ADI_MAX32_CLK_ERTCO_FREQ DT_PROP(DT_NODELABEL(clk_ertco), clock_frequency)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_extclk), fixed_clock, okay)
#define ADI_MAX32_CLK_EXTCLK_ENABLED 1
#define ADI_MAX32_CLK_EXTCLK_FREQ DT_PROP(DT_NODELABEL(clk_extclk), clock_frequency)
#endif

/*
 *  Map the clock macros depend on the device
 */
#if defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666)

#define ADI_MAX32_CLK_IPO MXC_SYS_CLOCK_HIRC96
#define ADI_MAX32_CLK_ERFO MXC_SYS_CLOCK_XTAL32M
#define ADI_MAX32_CLK_IBRO MXC_SYS_CLOCK_HIRC8 
#define ADI_MAX32_CLK_ISO MXC_SYS_CLOCK_HIRC
#define ADI_MAX32_CLK_INRO MXC_SYS_CLOCK_LIRC8K
#define ADI_MAX32_CLK_ERTCO MXC_SYS_CLOCK_XTAL32K

#elif defined(CONFIG_SOC_MAX32690)

#define ADI_MAX32_CLK_IPO MXC_SYS_CLOCK_IPO
#define ADI_MAX32_CLK_ERFO MXC_SYS_CLOCK_ERFO
#define ADI_MAX32_CLK_IBRO MXC_SYS_CLOCK_IBRO 
#define ADI_MAX32_CLK_ISO MXC_SYS_CLOCK_ISO
#define ADI_MAX32_CLK_INRO MXC_SYS_CLOCK_INRO
#define ADI_MAX32_CLK_ERTCO MXC_SYS_CLOCK_ERTCO
#define ADI_MAX32_CLK_EXTCLK MXC_SYS_CLOCK_EXTCLK

#endif 


#define DT_GCR_CLOCKS_CTRL  DT_CLOCKS_CTLR(DT_NODELABEL(gcr))

/* To enable use of IS_ENABLED utility macro, these symbols
 * should not be defined directly using DT_SAME_NODE.
 */
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


/** GCR node related properties */
#define ADI_MAX32_SYSCLK_PRESCALER DT_PROP(DT_NODELABEL(gcr), sysclk_prescaler)


#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ADI_MAX32_CLOCK_CONTROL_H_ */
