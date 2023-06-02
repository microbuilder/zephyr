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

#include <gpio.h>

#define DT_DRV_COMPAT adi_max32_gpio

LOG_MODULE_REGISTER(gpio_max32, CONFIG_GPIO_LOG_LEVEL);

struct gpio_max32_data {
	struct gpio_driver_data common;
	sys_slist_t cb_list;
};

struct gpio_max32_config {
	struct gpio_driver_config common;
	volatile mxc_gpio_regs_t *gpio;
	const struct pinctrl_dev_config *pctrl;
	const struct device *clock;
	void (*irq_func)(void);
	uint32_t clock_bus;
	uint32_t clock_bit;
};

static int gpio_max32_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_max32_config *cfg = dev->config;
	*value = MXC_GPIO_InGet((mxc_gpio_regs_t *)(cfg->gpio), (unsigned int)-1);
	return 0;
}

static int gpio_max32_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct gpio_max32_config *cfg = dev->config;
	MXC_GPIO_OutPut((mxc_gpio_regs_t *)(cfg->gpio), mask, value);
	return 0;
}

static int gpio_max32_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_max32_config *cfg = dev->config;
	MXC_GPIO_OutSet((mxc_gpio_regs_t *)(cfg->gpio), pins);
	return 0;
}

static int gpio_max32_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_max32_config *cfg = dev->config;
	MXC_GPIO_OutClr((mxc_gpio_regs_t *)(cfg->gpio), pins);
	return 0;
}

static int gpio_max32_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_max32_config *cfg = dev->config;
	MXC_GPIO_OutToggle((mxc_gpio_regs_t *)(cfg->gpio), pins);
	return 0;
}

int gpio_max32_config_pinmux(const struct device *dev, int pin, int pinmux, int pincfg)
{
	const struct gpio_max32_config *cfg = dev->config;
	mxc_gpio_regs_t *gpio = (mxc_gpio_regs_t *)cfg->gpio;

	int mode = MAX32_PINMUX_MODE(pinmux);

	switch (mode) {
	case MAX32_GPIO:
		break;
	case MAX32_AF1:
		gpio->en_clr = BIT(pin);
		gpio->en1_clr = BIT(pin);
		gpio->en2_clr = BIT(pin);
		break;
	case MAX32_AF2:
		gpio->en_clr = BIT(pin);
		gpio->en1_set = BIT(pin);
		gpio->en2_clr = BIT(pin);
		break;
	case MAX32_AF3:
		gpio->en_clr = BIT(pin);
		gpio->en1_clr = BIT(pin);
		gpio->en2_set = BIT(pin);
		break;
	case MAX32_AF4:
		gpio->en_clr = BIT(pin);
		gpio->en1_set = BIT(pin);
		gpio->en2_set = BIT(pin);
		break;
	default:
		return -EINVAL;
	}

	if (pincfg & BIT(MAX32_INPUT_ENABLE_SHIFT)) {
		gpio->in_en |= BIT(pin);
	}

	if (pincfg & BIT(MAX32_OUTPUT_ENABLE_SHIFT)) {
		gpio->out_en_set = BIT(pin);
	}

	/* select strong pull */
	gpio->ps |= BIT(pin);

	if (pincfg & BIT(MAX32_BIAS_PULL_UP_SHIFT)) {
		gpio->pad_cfg1 |= BIT(pin);
	} else if (pincfg & BIT(MAX32_BIAS_PULL_DOWN_SHIFT)) {
		gpio->pad_cfg2 |= BIT(pin);
	}

	if (pincfg & BIT(MAX32_OUTPUT_HIGH_SHIFT)) {
		gpio->out_set = BIT(pin);
	}

	if (pincfg & BIT(MAX32_POWER_SOURCE_SHIFT)) {
		gpio->vssel |= BIT(pin);
	}

	return 0;
}

static int gpio_max32_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_max32_config *cfg = dev->config;
	mxc_gpio_regs_t *gpio = (mxc_gpio_regs_t *)cfg->gpio;

	if (flags & GPIO_INPUT) {
		gpio->in_en |= BIT(pin);
	} else {
		gpio->in_en &= ~BIT(pin);
	}

	if (flags & GPIO_OUTPUT) {
		gpio->out_en_set = BIT(pin);
	} else {
		gpio->out_en_clr = BIT(pin);
	}

	if (flags & GPIO_OUTPUT_INIT_LOW) {
		gpio->out_clr |= BIT(pin);
	} else if (flags & GPIO_OUTPUT_INIT_HIGH) {
		gpio->out_set |= BIT(pin);
	}

	return 0;
}

static int gpio_max32_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					      enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_max32_config *cfg = dev->config;
	volatile mxc_gpio_regs_t *gpio = cfg->gpio;

	if (mode == GPIO_INT_MODE_DISABLED) {
		gpio->int_en_clr = BIT(pin);
		return 0;
	}

	switch (trig) {
	case GPIO_INT_TRIG_LOW:
		gpio->int_pol &= ~BIT(pin);
		break;
	case GPIO_INT_TRIG_HIGH:
		gpio->int_pol |= BIT(pin);
		break;
	case GPIO_INT_TRIG_BOTH:
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	switch (mode) {
	case GPIO_INT_MODE_LEVEL:
		gpio->int_mod &= ~BIT(pin);
		break;
	case GPIO_INT_MODE_EDGE:
		gpio->int_mod |= BIT(pin);
		break;
	default:
		return -EINVAL;
	}

	gpio->int_en_set = BIT(pin);

	return 0;
}

static int gpio_max32_manage_callback(const struct device *dev, struct gpio_callback *callback,
				      bool set)
{
	struct gpio_max32_data *data = dev->data;

	return gpio_manage_callback(&data->cb_list, callback, set);
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
	const struct gpio_max32_config *cfg = dev->config;
	struct gpio_max32_data *data = dev->data;

	uint32_t pins = cfg->gpio->int_stat;

	cfg->gpio->int_clr = pins;

	gpio_fire_callbacks(&data->cb_list, dev, pins);
}

static int gpio_max32_init(const struct device *dev)
{
	const struct gpio_max32_config *cfg = dev->config;
	uint32_t clkcfg;
	int ret;

	if (!device_is_ready(cfg->clock)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* enable clock */
	clkcfg = (cfg->clock_bus << 16) | cfg->clock_bit;
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)clkcfg);
	if (ret != 0) {
		LOG_ERR("cannot enable GPIO clock");
		return ret;
	}

	cfg->irq_func();

	return 0;
}

#define MAX32_GPIO_INIT(n)                                                                         \
	static void gpio_max32_irq_init_##n(void)                                                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), gpio_max32_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static struct gpio_max32_data gpio_max32_data_##n;                                         \
	static const struct gpio_max32_config gpio_max32_config_##n = {                            \
		.gpio = (mxc_gpio_regs_t *)DT_INST_REG_ADDR(n),                                    \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                    \
		.clock_bus = DT_INST_CLOCKS_CELL(n, offset),                                       \
		.clock_bit = DT_INST_CLOCKS_CELL(n, bit),                                          \
		.irq_func = &gpio_max32_irq_init_##n,                                              \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, gpio_max32_init, NULL, &gpio_max32_data_##n,                      \
			      &gpio_max32_config_##n, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,     \
			      (void *)&gpio_max32_driver);

DT_INST_FOREACH_STATUS_OKAY(MAX32_GPIO_INIT)
