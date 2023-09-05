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

#include <wrap_max32_uart.h>

#define DT_DRV_COMPAT adi_max32_uart

LOG_MODULE_REGISTER(uart_max32, CONFIG_UART_LOG_LEVEL);

struct uart_max32_config {
	mxc_uart_regs_t *regs;
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
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t  cb; /* Interrupt Callback */
	void *cb_data;                 /* Interrupt Callback Arg */
#endif
    struct uart_config conf; // baudrate, stopbits, ...
};

static void uart_max32_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *regs = cfg->regs;

	MXC_UART_WriteCharacter(regs, c);
}

static int uart_max32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *regs = cfg->regs;
	int val;

	val = MXC_UART_ReadCharacterRaw(regs);
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
	mxc_uart_regs_t *regs = cfg->regs;
	uint32_t flags;

    flags = MXC_UART_GetFlags(regs);

	if (flags & ADI_MAX32_UART_ERROR_FRAMING) {
		err |= UART_ERROR_FRAMING;
	}

	if (flags & ADI_MAX32_UART_ERROR_PARITY) {
		err |= UART_ERROR_PARITY;
	}

	if (flags & ADI_MAX32_UART_ERROR_OVERRUN) {
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
	mxc_uart_regs_t *regs = cfg->regs;
	struct uart_max32_data *uart_priv_data = (struct uart_max32_data *)(dev->data);

    /*
     *  Set parity
     */
    mxc_uart_parity_t mxc_parity;

    switch (uart_cfg->parity) {
       case UART_CFG_PARITY_NONE:
           mxc_parity = ADI_MAX32_UART_CFG_PARITY_NONE;
           break;
       case UART_CFG_PARITY_ODD:
           mxc_parity = ADI_MAX32_UART_CFG_PARITY_ODD;
           break;
       case UART_CFG_PARITY_EVEN:
           mxc_parity = ADI_MAX32_UART_CFG_PARITY_EVEN;
           break;
 	#if defined(CONFIG_SOC_MAX32665) || (CONFIG_SOC_MAX32666)
       case UART_CFG_PARITY_MARK:
           mxc_parity = ADI_MAX32_UART_CFG_PARITY_MARK;
           break;
       case UART_CFG_PARITY_SPACE:
           mxc_parity = ADI_MAX32_UART_CFG_PARITY_SPACE;
           break;
     #endif
       default:
           mxc_parity = ADI_MAX32_UART_CFG_PARITY_NONE;
           break;
   }

	err = MXC_UART_SetParity(regs, mxc_parity);
	if (err < 0) {
		return -ENOTSUP;
	}
	// incase of success keep configuration
	uart_priv_data->conf.parity = uart_cfg->parity;


    /*
     *  Set stop bit
     */
	if (uart_cfg->stop_bits == UART_CFG_STOP_BITS_1) {
		err = MXC_UART_SetStopBits(regs, MXC_UART_STOP_1);
	} else if (uart_cfg->stop_bits == UART_CFG_STOP_BITS_2) {
		err = MXC_UART_SetStopBits(regs, MXC_UART_STOP_2);
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
	err = MXC_UART_SetDataSize(regs, CONVERT_TO_MXC_DATABITS(uart_cfg->data_bits));
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
    err = Wrap_MXC_UART_SetFrequency(regs, uart_cfg->baudrate, cfg->clock_source);
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
	mxc_uart_regs_t *regs = cfg->regs;
    struct uart_max32_data *uart_priv_data = (struct uart_max32_data *)(dev->data);

	if (!device_is_ready(cfg->clock)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

    ret = MXC_UART_Shutdown(regs);
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

	ret = Wrap_MXC_UART_Init(regs, cfg->uart_conf.baudrate, cfg->clock_source);
    if (ret) {
        return ret;
    }

    // store configuration at the global
    uart_priv_data->conf = cfg->uart_conf;
    
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	// Clear any pending UART RX/TX interrupts
	MXC_UART_ClearFlags(regs, (ADI_MAX32_UART_INT_TX | ADI_MAX32_UART_INT_TX));
	cfg->irq_config_func(dev);
#endif

	return ret;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_max32_fifo_fill(const struct device *dev,
				 const uint8_t *tx_data,
				 int size)
{
	const struct uart_max32_config *const cfg = dev->config;
	unsigned int num_tx = 0;

	num_tx = MXC_UART_WriteTXFIFO(cfg->regs, (unsigned char *)tx_data, size);

	return (int)num_tx;
}

static int uart_max32_fifo_read(const struct device *dev, uint8_t *rx_data,
				 const int size)
{
	const struct uart_max32_config *const cfg = dev->config;
	unsigned int num_rx = 0;

	num_rx = MXC_UART_ReadRXFIFO(cfg->regs, (unsigned char *)rx_data, size);

	return num_rx;
}

static void uart_max32_irq_tx_enable(const struct device *dev)
{
	const struct uart_max32_config *const cfg = dev->config;

	MXC_UART_EnableInt(cfg->regs, ADI_MAX32_UART_INT_TX);
}

static void uart_max32_irq_tx_disable(const struct device *dev)
{
	const struct uart_max32_config *const cfg = dev->config;

	MXC_UART_DisableInt(cfg->regs, ADI_MAX32_UART_INT_TX);
}

static int uart_max32_irq_tx_ready(const struct device *dev)
{
	const struct uart_max32_config *const cfg = dev->config;

	return (MXC_UART_GetFlags(cfg->regs) & ADI_MAX32_UART_INT_TX);
}

static void uart_max32_irq_rx_enable(const struct device *dev)
{
	const struct uart_max32_config *const cfg = dev->config;

	MXC_UART_EnableInt(cfg->regs, ADI_MAX32_UART_INT_RX);
}

static void uart_max32_irq_rx_disable(const struct device *dev)
{
	const struct uart_max32_config *const cfg = dev->config;

	MXC_UART_DisableInt(cfg->regs, ADI_MAX32_UART_INT_RX);
}

static int uart_max32_irq_tx_complete(const struct device *dev)
{
    const struct uart_max32_config *const cfg = dev->config;

    if (E_BUSY == MXC_UART_GetActive(cfg->regs)) {
        return 0;
    } else {
        return 1; // tranmission completed
    }
}

static int uart_max32_irq_rx_ready(const struct device *dev)
{
	const struct uart_max32_config *const cfg = dev->config;

	return (MXC_UART_GetFlags(cfg->regs) & ADI_MAX32_UART_INT_RX);
}

static void uart_max32_irq_err_enable(const struct device *dev)
{
    //...
}

static void uart_max32_irq_err_disable(const struct device *dev)
{
    //...
}

static int uart_max32_irq_is_pending(const struct device *dev)
{
	const struct uart_max32_config *const cfg = dev->config;

	return (MXC_UART_GetFlags(cfg->regs) & (ADI_MAX32_UART_INT_RX | ADI_MAX32_UART_INT_TX));
}

static int uart_max32_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_max32_irq_callback_set(const struct device *dev,
					 uart_irq_callback_user_data_t cb,
					 void *cb_data)
{
    struct uart_max32_data * const dev_data = (struct uart_max32_data *)(dev->data);

	dev_data->cb = cb;
	dev_data->cb_data = cb_data;
}

static void uart_max32_isr(const struct device *dev)
{
	const struct uart_max32_config *const cfg = dev->config;
    struct uart_max32_data *dev_data = (struct uart_max32_data *)(dev->data);
    unsigned int int_status;

    int_status = MXC_UART_GetFlags(cfg->regs);

	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	}

    // Clear RX/TX interrupts flag after cb called
	MXC_UART_ClearFlags(cfg->regs, int_status);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_max32_driver_api = {
	.poll_in = uart_max32_poll_in,
	.poll_out = uart_max32_poll_out,
	.err_check = uart_max32_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_max32_configure,
	.config_get = uart_max32_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_max32_fifo_fill,
	.fifo_read = uart_max32_fifo_read,
	.irq_tx_enable = uart_max32_irq_tx_enable,
	.irq_tx_disable = uart_max32_irq_tx_disable,
	.irq_tx_ready = uart_max32_irq_tx_ready,
	.irq_rx_enable = uart_max32_irq_rx_enable,
	.irq_rx_disable = uart_max32_irq_rx_disable,
	.irq_tx_complete = uart_max32_irq_tx_complete,
	.irq_rx_ready = uart_max32_irq_rx_ready,
	.irq_err_enable = uart_max32_irq_err_enable,
	.irq_err_disable = uart_max32_irq_err_disable,
	.irq_is_pending = uart_max32_irq_is_pending,
	.irq_update = uart_max32_irq_update,
	.irq_callback_set = uart_max32_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#define MAX32_UART_INIT(_num) \
	PINCTRL_DT_INST_DEFINE(_num); \
	IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, \
	(static void uart_max32_irq_init_##_num(const struct device *dev) \
	{ \
		IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, ( \
			IRQ_CONNECT(\
				DT_INST_IRQN(_num), \
			    DT_INST_IRQ(_num, priority), \
			    uart_max32_isr, \
			    DEVICE_DT_INST_GET(_num), \
			    0); \
			irq_enable(DT_INST_IRQN(_num))) \
		); \
	})); \
	static const struct uart_max32_config uart_max32_config_##_num = { \
		.regs = (mxc_uart_regs_t *)DT_INST_REG_ADDR(_num), \
		.pctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(_num), \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(_num)), \
		.perclk.bus = DT_INST_CLOCKS_CELL(_num, offset), \
		.perclk.bit = DT_INST_CLOCKS_CELL(_num, bit), \
		.clock_source = DT_INST_PROP(_num, clock_source), \
        .uart_conf.baudrate = DT_INST_PROP(_num, current_speed), \
        .uart_conf.parity = DT_INST_ENUM_IDX_OR(_num, parity, UART_CFG_PARITY_NONE),\
		.uart_conf.data_bits = DT_INST_ENUM_IDX(_num, data_bits), \
		IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, \
		    (.irq_config_func = uart_max32_irq_init_##_num,)) \
	}; \
	static struct uart_max32_data uart_max32_data##_num = { \
		IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, (.cb = NULL,)) \
	}; \
	DEVICE_DT_INST_DEFINE(_num, \
		uart_max32_init, \
		NULL, \
		&uart_max32_data##_num, \
		&uart_max32_config_##_num, \
		PRE_KERNEL_1, \
		CONFIG_SERIAL_INIT_PRIORITY, \
		(void *)&uart_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX32_UART_INIT)
