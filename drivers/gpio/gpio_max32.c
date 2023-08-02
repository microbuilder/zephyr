/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <pinctrl_soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>

#include <gpio.h>

#define DT_DRV_COMPAT adi_max32_gpio

LOG_MODULE_REGISTER(gpio_max32, CONFIG_GPIO_LOG_LEVEL);

#define GPIO_CFG(dev) ((struct max32_gpio_config *) ((dev)->config))
#define GPIO_DATA(dev) ((struct max32_gpio_data *) ((dev)->data))

struct max32_gpio_config {
	struct gpio_driver_config common;
	mxc_gpio_regs_t *regs;
	const struct pinctrl_dev_config *pctrl;
	const struct device *clock;
	void (*irq_func)(void);
	struct max32_perclk perclk;
};

struct max32_gpio_data {
	struct gpio_driver_data common;
	sys_slist_t cb_list;
};

static int gpio_max32_port_get_raw(const struct device *dev, uint32_t *value)
{
	*value = MXC_GPIO_InGet(GPIO_CFG(dev)->regs, (unsigned int)-1);
	return 0;
}

static int gpio_max32_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	MXC_GPIO_OutPut(GPIO_CFG(dev)->regs, mask, value);
	return 0;
}

static int gpio_max32_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	MXC_GPIO_OutSet(GPIO_CFG(dev)->regs, pins);
	return 0;
}

static int gpio_max32_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	MXC_GPIO_OutClr(GPIO_CFG(dev)->regs, pins);
	return 0;
}

static int gpio_max32_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	MXC_GPIO_OutToggle(GPIO_CFG(dev)->regs, pins);
	return 0;
}

int gpio_max32_config_pinmux(const struct device *dev, int pin, int pinmux, int pincfg)
{
	mxc_gpio_cfg_t gpio_cfg;

	gpio_cfg.port = GPIO_CFG(dev)->regs;
	gpio_cfg.mask = BIT(pin);

	if (pincfg & BIT(MAX32_BIAS_PULL_UP_SHIFT)) {
		gpio_cfg.pad = MXC_GPIO_PAD_PULL_UP;
	} else if (pincfg & BIT(MAX32_BIAS_PULL_DOWN_SHIFT)) {
		gpio_cfg.pad = MXC_GPIO_PAD_NONE;
	}

	if (pincfg & BIT(MAX32_INPUT_ENABLE_SHIFT)) {
		gpio_cfg.func = MXC_GPIO_FUNC_IN;
	} else if (pincfg & BIT(MAX32_OUTPUT_ENABLE_SHIFT)) {
		gpio_cfg.func = MXC_GPIO_FUNC_OUT;
	} else {
		gpio_cfg.func = (mxc_gpio_func_t)(MAX32_PINMUX_MODE(pinmux) + 1);// Add +1 to index match
	}

	if (pincfg & BIT(MAX32_POWER_SOURCE_SHIFT)) {
		gpio_cfg.vssel = MXC_GPIO_VSSEL_VDDIOH;
	} else {
		gpio_cfg.vssel = MXC_GPIO_VSSEL_VDDIO;
	}

	MXC_GPIO_Config(&gpio_cfg);

	return 0;
}

static int gpio_max32_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	mxc_gpio_regs_t *gpio = GPIO_CFG(dev)->regs;
	mxc_gpio_cfg_t gpio_cfg;

	gpio_cfg.port = gpio;
	gpio_cfg.mask = BIT(pin);

	if (flags & GPIO_PULL_UP) {
		gpio_cfg.pad  = MXC_GPIO_PAD_PULL_UP;
	} else if (flags & GPIO_PULL_DOWN) {
		gpio_cfg.pad  = MXC_GPIO_PAD_PULL_DOWN;
	} else {
		gpio_cfg.pad  = MXC_GPIO_PAD_NONE;
	}

	if (flags & GPIO_OUTPUT) {
		gpio_cfg.func = MXC_GPIO_FUNC_OUT;
	} else if (flags & GPIO_INPUT) {
		gpio_cfg.func = MXC_GPIO_FUNC_IN;
	} else {
		// this case will not occur this function call for gpio mode in/out
		gpio_cfg.func = MXC_GPIO_FUNC_ALT1;// TODO: Think on it
	}

	// TODO: Set it VDDIO/VDDIOH depend on the params
	gpio_cfg.vssel = MXC_GPIO_VSSEL_VDDIO;

	MXC_GPIO_Config(&gpio_cfg);

	if (flags & GPIO_OUTPUT) {
		if (flags & GPIO_OUTPUT_INIT_LOW) {
			MXC_GPIO_OutClr(gpio, BIT(pin));
		} else if (flags & GPIO_OUTPUT_INIT_HIGH) {
			MXC_GPIO_OutSet(gpio, BIT(pin));
		}
	}

	return 0;
}

static int gpio_max32_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
						  enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	mxc_gpio_regs_t *gpio = GPIO_CFG(dev)->regs;
	mxc_gpio_cfg_t gpio_cfg;

	gpio_cfg.port = gpio;
	gpio_cfg.mask = BIT(pin);
	// rest of the parameters not necessary

	if (mode == GPIO_INT_MODE_DISABLED) {
		MXC_GPIO_DisableInt(gpio, gpio_cfg.mask);
		return 0;
	}

	switch (mode) {
	case GPIO_INT_MODE_LEVEL:
		if (trig == GPIO_INT_TRIG_LOW) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_LOW);
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_HIGH);
		} else if (trig == GPIO_INT_TRIG_BOTH) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_BOTH);
		} else {
			return -EINVAL;
		}
		break;
	case GPIO_INT_MODE_EDGE:
		if (trig == GPIO_INT_TRIG_LOW) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_FALLING);
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_RISING);
		} else if (trig == GPIO_INT_TRIG_BOTH) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_BOTH);
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	MXC_GPIO_EnableInt(gpio, gpio_cfg.mask);

	return 0;
}

static int gpio_max32_manage_callback(const struct device *dev, struct gpio_callback *callback,
					  bool set)
{
	return gpio_manage_callback(&(GPIO_DATA(dev)->cb_list), callback, set);
}

static const struct gpio_driver_api gpio_max32_driver = {
	.pin_configure = gpio_max32_config,
	.port_get_raw = gpio_max32_port_get_raw,
	.port_set_masked_raw = gpio_max32_port_set_masked_raw,
	.port_set_bits_raw = gpio_max32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_max32_port_clear_bits_raw,
	.port_toggle_bits = gpio_max32_port_toggle_bits,
	.pin_interrupt_configure = gpio_max32_pin_interrupt_configure,
	.manage_callback = gpio_max32_manage_callback,
};

static void gpio_max32_isr(const void *param)
{
	const struct device *dev = param;

	unsigned int flags = MXC_GPIO_GetFlags(GPIO_CFG(dev)->regs);
	// clear interrupt flags
	MXC_GPIO_ClearFlags(GPIO_CFG(dev)->regs, flags);

	gpio_fire_callbacks(&(GPIO_DATA(dev)->cb_list), dev, flags);
}

static int gpio_max32_init(const struct device *dev)
{
	const struct max32_gpio_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->clock)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* enable clock */
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)&(cfg->perclk));
	if (ret != 0) {
		LOG_ERR("cannot enable GPIO clock");
		return ret;
	}

	cfg->irq_func();

	return 0;
}

#define MAX32_GPIO_INIT(n)	\
	static void gpio_max32_irq_init_##n(void) \
	{ \
		IRQ_CONNECT(DT_INST_IRQN(n), \
					DT_INST_IRQ(n, priority), \
					gpio_max32_isr, \
					DEVICE_DT_INST_GET(n), 0); \
		irq_enable(DT_INST_IRQN(n)); \
	} \
	static struct max32_gpio_data max32_gpio_data_##n; \
	static const struct max32_gpio_config max32_gpio_config_##n = { \
		.regs = (mxc_gpio_regs_t *)DT_INST_REG_ADDR(n), \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
		.perclk.bus = DT_INST_CLOCKS_CELL(n, offset), \
		.perclk.bit = DT_INST_CLOCKS_CELL(n, bit), \
		.irq_func = &gpio_max32_irq_init_##n, \
	}; \
	DEVICE_DT_INST_DEFINE(n,\
						  gpio_max32_init, \
						  NULL, \
						  &max32_gpio_data_##n, \
						  &max32_gpio_config_##n, \
						  PRE_KERNEL_1, \
						  CONFIG_GPIO_INIT_PRIORITY, \
						  (void *)&gpio_max32_driver);

DT_INST_FOREACH_STATUS_OKAY(MAX32_GPIO_INIT)
