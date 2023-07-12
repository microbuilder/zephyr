/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>

#include <uart.h>

#define DT_DRV_COMPAT adi_max32_uart

LOG_MODULE_REGISTER(uart_max32, CONFIG_UART_LOG_LEVEL);

struct uart_max32_config {
	mxc_uart_regs_t *uart;
	int clock_source;
	const struct pinctrl_dev_config *pctrl;
	const struct device *clock;
	struct max32_perclk perclk;
	struct uart_config uart_conf;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct uart_max32_data {
	uart_irq_callback_user_data_t irq_cb; /* Interrupt Callback */
	void *irq_cb_data;                    /* Interrupt Callback Arg */
    struct uart_config conf; // baudrate, stopbits, ...
};

static void uart_max32_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *uart = cfg->uart;

	MXC_UART_WriteCharacter(uart, c);
}

static int uart_max32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *uart = cfg->uart;
	int val;

	val = MXC_UART_ReadCharacterRaw(uart);
	if (val < 0) {
		return -1;
	} else {
		*c = (unsigned char)val;
	}

	return 0;
}

static int uart_max32_err_check(const struct device *dev)
{
    int err = 0;
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *uart = cfg->uart;
	uint32_t flags;

    flags = MXC_UART_GetFlags(uart);

	if (flags & MXC_F_UART_INT_EN_RX_FRAME_ERROR) {
		err |= UART_ERROR_FRAMING;
	}

	if (flags & MXC_F_UART_INT_EN_RX_PARITY_ERROR) {
		err |= UART_ERROR_PARITY;
	}

	if (flags & MXC_F_UART_INT_EN_RX_OVERRUN) {
		err |= UART_ERROR_OVERRUN;
	}

	return err;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE

#define CONVERT_TO_MXC_DATABITS(x) (x + 5)

static int uart_max32_configure(const struct device *dev, const struct uart_config *uart_cfg)
{
	int err;
    const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *uart = cfg->uart;
	struct uart_max32_data *uart_priv_data = (struct uart_max32_data *)(dev->data);

    /*
     *  Set parity
     */
    mxc_uart_parity_t mxc_parity;

    switch (uart_cfg->parity) {
       case UART_CFG_PARITY_NONE:
           mxc_parity = MXC_UART_PARITY_DISABLE;
       case UART_CFG_PARITY_ODD:
           mxc_parity = MXC_UART_PARITY_ODD;
       case UART_CFG_PARITY_EVEN:
           mxc_parity = MXC_UART_PARITY_EVEN;
       case UART_CFG_PARITY_MARK:
           mxc_parity = MXC_UART_PARITY_MARK;
       case UART_CFG_PARITY_SPACE:
           mxc_parity = MXC_UART_PARITY_SPACE;
       default:
           mxc_parity = MXC_UART_PARITY_DISABLE;
   }

	err = MXC_UART_SetParity(uart, mxc_parity);
	if (err < 0) {
		return -ENOTSUP;
	}
	// incase of success keep configuration
	uart_priv_data->conf.parity = uart_cfg->parity;


    /*
     *  Set stop bit
     */
	if (uart_cfg->stop_bits == UART_CFG_STOP_BITS_1) {
		err = MXC_UART_SetStopBits(uart, MXC_UART_STOP_1);
	} else if (uart_cfg->stop_bits == UART_CFG_STOP_BITS_1_5 ||
		   cfg->uart_conf.stop_bits == UART_CFG_STOP_BITS_2) {
		err = MXC_UART_SetStopBits(uart, MXC_UART_STOP_2);
	} else {
		return -ENOTSUP;
	}
	if (err < 0) {
		return -ENOTSUP;
	}
	// incase of success keep configuration
	uart_priv_data->conf.stop_bits = uart_cfg->stop_bits;


    /*
     *  Set data bit
     */
	err = MXC_UART_SetDataSize(uart, CONVERT_TO_MXC_DATABITS(uart_cfg->data_bits));
	if (err < 0) {
		return -ENOTSUP;
	}
	// incase of success keep configuration
	uart_priv_data->conf.data_bits = uart_cfg->data_bits;



    /*
     *  TODO: Set flow control
     */

    // ...


    /*
     *  Set frequency
     */
#if defined(CONFIG_SOC_MAX32690)
    err = MXC_UART_SetFrequency(uart, uart_cfg->baudrate, (mxc_uart_clock_t)cfg->clock_source);
#else
    err = MXC_UART_SetFrequency(uart, uart_cfg->baudrate);
#endif
	if (err < 0) {
		return -ENOTSUP;
	}

	// incase of success keep configuration
	uart_priv_data->conf.baudrate = uart_cfg->baudrate;

	return 0;
}

static int uart_max32_config_get(const struct device *dev, struct uart_config *uart_cfg)
{
    struct uart_max32_data *uart_priv_data = (struct uart_max32_data *)(dev->data);

	// copy configs from global setting
	*uart_cfg = uart_priv_data->conf;

	return 0;
}

#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uart_max32_init(const struct device *dev)
{
	int ret;
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *uart = cfg->uart;
    struct uart_max32_data *uart_priv_data = (struct uart_max32_data *)(dev->data);

	if (!device_is_ready(cfg->clock)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

    ret = MXC_UART_Shutdown(uart);
    if (ret) {
        return ret;
    }

	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)&(cfg->perclk));
	if (ret != 0) {
		LOG_ERR("cannot enable UART clock");
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pctrl, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

#if defined(CONFIG_SOC_MAX32690)
	ret = MXC_UART_Init(uart, cfg->uart_conf.baudrate, (mxc_uart_clock_t)cfg->clock_source);
#else
    ret = MXC_UART_Init(uart, cfg->uart_conf.baudrate);
#endif
    if (ret) {
        return ret;
    }

    // store configuration at the global
    uart_priv_data->conf = cfg->uart_conf;
    
	return ret;
}

static const struct uart_driver_api uart_max32_driver_api = {
	.poll_in = uart_max32_poll_in,
	.poll_out = uart_max32_poll_out,
	.err_check = uart_max32_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_max32_configure,
	.config_get = uart_max32_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
};

#define MAX32_UART_INIT(_num)                                              		    \
	PINCTRL_DT_INST_DEFINE(_num);                                          		    \
	static const struct uart_max32_config uart_max32_config_##_num = {     		    \
		.uart = (mxc_uart_regs_t *)DT_INST_REG_ADDR(_num),                 		    \
		.pctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(_num),                              \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(_num)),                          \
		.perclk.bus = DT_INST_CLOCKS_CELL(_num, offset),                            \
		.perclk.bit = DT_INST_CLOCKS_CELL(_num, bit),                               \
		.clock_source = DT_INST_ENUM_IDX(_num, clock_source),                       \
        .uart_conf.baudrate = DT_INST_PROP(_num, current_speed),                    \
        .uart_conf.parity = DT_INST_ENUM_IDX_OR(_num, parity, UART_CFG_PARITY_NONE),\
		.uart_conf.data_bits = DT_INST_ENUM_IDX(_num, data_bits),                   \
	};                                                                              \
                                                                                    \
                                                                                    \
    static struct uart_max32_data uart_max32_data##_num;                            \
                                                                                    \
	DEVICE_DT_INST_DEFINE(_num, \
		uart_max32_init, \
		NULL, \
		&uart_max32_data##_num, \
		&uart_max32_config_##_num, \
		PRE_KERNEL_1, \
		CONFIG_SERIAL_INIT_PRIORITY, \
		(void *)&uart_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX32_UART_INIT)
