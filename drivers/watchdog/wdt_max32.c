/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_watchdog

#include <zephyr/drivers/watchdog.h>
#include <zephyr/irq.h>
#include <soc.h>
#include <errno.h>

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_max32);

#include <wdt.h>

#define WDT_CFG(dev) ((struct wdt_max32_config *) ((dev)->config))
#define WDT_DATA(dev) ((struct wdt_max32_data *) ((dev)->data))

struct wdt_max32_config {
	mxc_wdt_regs_t *regs;
	const struct device *clock_dev;
	void (*connect_irq)(void);
	int irq_source;
};

struct wdt_max32_data {
	uint32_t timeout;
	wdt_callback_t callback;
};

static void wdt_max32_enable(const struct device *dev)
{
	mxc_wdt_regs_t *regs = WDT_CFG(dev)->regs;
	struct wdt_max32_data *data = dev->data;
	
	MXC_WDT_Enable(regs);
}

static int wdt_max32_disable(const struct device *dev)
{
	mxc_wdt_regs_t *regs = WDT_CFG(dev)->regs;
	struct wdt_max32_data *data = dev->data;

	MXC_WDT_Disable(regs);

	return 0;
}

static int wdt_max32_feed(const struct device *dev, int channel_id)
{
	mxc_wdt_regs_t *regs = WDT_CFG(dev)->regs;
	struct wdt_max32_data *data = dev->data;

	MXC_WDT_ResetTimer(regs);

	return 0;
}

static int wdt_max32_set_config(const struct device *dev, uint8_t options)
{
	mxc_wdt_regs_t *regs = WDT_CFG(dev)->regs;
	struct wdt_max32_data *data = dev->data;

	MXC_WDT_SetResetPeriod(regs, (mxc_wdt_period_t) data->timeout);
	wdt_max32_enable(dev);
	wdt_max32_feed(dev, 0);

	return 0;
}

static int wdt_max32_install_timeout(const struct device *dev,
				     const struct wdt_timeout_cfg *cfg)
{
	mxc_wdt_regs_t *regs = WDT_CFG(dev)->regs;
	struct wdt_max32_data *data = dev->data;

	MXC_WDT_SetResetPeriod(regs, (mxc_wdt_period_t)(data->timeout));

	if (cfg->window.min != 0U || cfg->window.max == 0U) {
		return -EINVAL;
	}

	data->timeout = cfg->window.max;
	data->callback = cfg->callback;

	return 0;
}

static int wdt_max32_init(const struct device *dev)
{
	mxc_wdt_regs_t *regs = WDT_CFG(dev)->regs;
	const struct wdt_max32_config *const config = dev->config;
	struct wdt_max32_data *data = dev->data;

	// WDT Disable
	MXC_WDT_Disable(regs);
    MXC_WDT_Init(regs);

	MXC_WDT_SetResetPeriod(regs, (mxc_wdt_period_t)(data->timeout));

	MXC_WDT_ResetTimer(regs);

    // WDT Enable RESET
    MXC_WDT_EnableReset(regs);
    
    // WDT Enable
    MXC_WDT_Enable(regs);  

	return 0;
}

static const struct wdt_driver_api wdt_max32_api = {
	.setup = wdt_max32_set_config,
	.disable = wdt_max32_disable,
	.install_timeout = wdt_max32_install_timeout,
	.feed = wdt_max32_feed
};

#define MAX32_WDT_INIT(idx)							                   \
	static struct wdt_max32_data wdt_max32_data##idx;				   \
	static struct wdt_max32_config wdt_max32_config##idx = {		   \
		.regs = (mxc_wdt_regs_t *)DT_INST_REG_ADDR(idx),               \
		.irq_source = DT_IRQN(DT_NODELABEL(wdt##idx)),			       \
	};									                               \
										                               \
	DEVICE_DT_INST_DEFINE(idx,						                   \
			      wdt_max32_init,					                   \
			      NULL,						                           \
			      &wdt_max32_data##idx,					               \
			      &wdt_max32_config##idx,				               \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	   \
			      &wdt_max32_api)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(wdt0), okay)
MAX32_WDT_INIT(0);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(wdt1), okay)
MAX32_WDT_INIT(1);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(wdt2), okay)
MAX32_WDT_INIT(2);
#endif
