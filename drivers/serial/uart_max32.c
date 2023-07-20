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


#if defined(CONFIG_SOC_MAX32665) || (CONFIG_SOC_MAX32666)
// error flags
#define ADI_MAX32_UART_ERROR_OVERRUN  MXC_F_UART_INT_FL_RX_OVERRUN
#define ADI_MAX32_UART_ERROR_PARITY   MXC_F_UART_INT_FL_RX_PARITY_ERROR
#define ADI_MAX32_UART_ERROR_FRAMING  MXC_F_UART_INT_FL_RX_FRAME_ERROR
// parity
#define	ADI_MAX32_UART_CFG_PARITY_NONE MXC_UART_PARITY_DISABLE
#define	ADI_MAX32_UART_CFG_PARITY_ODD MXC_UART_PARITY_ODD
#define	ADI_MAX32_UART_CFG_PARITY_EVEN MXC_UART_PARITY_EVEN
#define	ADI_MAX32_UART_CFG_PARITY_MARK MXC_UART_PARITY_MARK
#define	ADI_MAX32_UART_CFG_PARITY_SPACE MXC_UART_PARITY_SPACE

#elif defined(CONFIG_SOC_MAX32690) || (CONFIG_SOC_MAX32655)
// error flags
#define ADI_MAX32_UART_ERROR_OVERRUN  MXC_F_UART_INT_FL_RX_OV
#define ADI_MAX32_UART_ERROR_PARITY   MXC_F_UART_INT_FL_RX_PAR
#define ADI_MAX32_UART_ERROR_FRAMING  MXC_F_UART_INT_FL_RX_FERR
// parity
#define	ADI_MAX32_UART_CFG_PARITY_NONE MXC_UART_PARITY_DISABLE
#define	ADI_MAX32_UART_CFG_PARITY_ODD MXC_UART_PARITY_ODD_0
#define	ADI_MAX32_UART_CFG_PARITY_EVEN MXC_UART_PARITY_EVEN_0
//#define	ADI_MAX32_UART_CFG_PARITY_MARK
//#define	ADI_MAX32_UART_CFG_PARITY_SPACE
#endif 


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
	uart_irq_callback_user_data_t irq_cb; /* Interrupt Callback */
	void *irq_cb_data;                    /* Interrupt Callback Arg */
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
#if defined(CONFIG_SOC_MAX32690) || (CONFIG_SOC_MAX32655)
    err = MXC_UART_SetFrequency(regs, uart_cfg->baudrate, (mxc_uart_clock_t)cfg->clock_source);
#else
    err = MXC_UART_SetFrequency(regs, uart_cfg->baudrate);
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

#if defined(CONFIG_SOC_MAX32690) || (CONFIG_SOC_MAX32655)
	ret = MXC_UART_Init(regs, cfg->uart_conf.baudrate, (mxc_uart_clock_t)cfg->clock_source);
#else
    ret = MXC_UART_Init(regs, cfg->uart_conf.baudrate);
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

#define MAX32_UART_INIT(_num) \
	PINCTRL_DT_INST_DEFINE(_num); \
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
	}; \
    static struct uart_max32_data uart_max32_data##_num; \
	DEVICE_DT_INST_DEFINE(_num, \
		uart_max32_init, \
		NULL, \
		&uart_max32_data##_num, \
		&uart_max32_config_##_num, \
		PRE_KERNEL_1, \
		CONFIG_SERIAL_INIT_PRIORITY, \
		(void *)&uart_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX32_UART_INIT)
