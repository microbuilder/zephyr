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
#include <uart_reva.h>

#define DT_DRV_COMPAT adi_max32_uart

LOG_MODULE_REGISTER(uart_max32, CONFIG_UART_LOG_LEVEL);
/* Helper function to calculate the shift value based on the mask */
static inline unsigned int __mxc_shift_mask(unsigned int mask)
{
	unsigned int shift = 0;
	while ((mask & 0x01) == 0) {
		mask >>= 1;
		shift++;
	}
	return shift;
}

#define MXC_GETFIELD(reg, mask) (((reg) & (mask)) >> __mxc_shift_mask(mask))

struct uart_max32_config {
	volatile mxc_uart_regs_t *uart;
	uint32_t baud_rate;
	uint8_t parity;
	const struct pinctrl_dev_config *pctrl;
	const struct device *clock;
	struct max32_perclk perclk;
	uint8_t data_bits;
	bool use_obrc;
	uint8_t stop_bits;
	uint8_t flow_ctrl;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct uart_max32_data {
	uart_irq_callback_user_data_t irq_cb; /* Interrupt Callback */
	void *irq_cb_data;                    /* Interrupt Callback Arg */
};

static void uart_max32_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *uart = (mxc_uart_regs_t *)cfg->uart;
	MXC_UART_WriteCharacter(uart, c);
}

static int uart_max32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *uart = (mxc_uart_regs_t *)cfg->uart;
	if (uart->status & MXC_F_UART_REVA_STATUS_RX_EMPTY) {
		return -1;
	} else {
		*c = (unsigned char)(MXC_UART_ReadCharacterRaw(uart) & 0xFF);
	}

	return 0;
}

static int uart_max32_err_check(const struct device *dev)
{
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *uart = (mxc_uart_regs_t *)cfg->uart;
	uint32_t flags = MXC_UART_GetFlags(uart);
	int err = 0;

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

/* Helper function to convert mxc stopbits to zephyr stopbits */
static inline unsigned int __mxc_to_zephyr_stopbits(unsigned int stopbits, unsigned int datasize)
{
	switch (stopbits) {
	case MXC_UART_STOP_1:
		return UART_CFG_STOP_BITS_1;
	case MXC_UART_STOP_2:
		if (datasize == UART_CFG_DATA_BITS_5) {
			return UART_CFG_STOP_BITS_1_5;
		} else {
			return UART_CFG_STOP_BITS_2;
		}
	default:
		return -ENOTSUP;
	}
}

#define CONVERT_MXC_TO_ZEPHYR_STOPBITS(stopbits, datasize)                                         \
	__mxc_to_zephyr_stopbits(stopbits, datasize)

static int MXC_UART_RevA_GetStopBits(mxc_uart_reva_regs_t *uart)
{
	if (MXC_UART_GET_IDX((mxc_uart_regs_t *)uart) < 0) {
		return E_BAD_PARAM;
	}

	return MXC_GETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_STOPBITS);
}

static int MXC_UART_GetStopBits(mxc_uart_regs_t *uart)
{
	return MXC_UART_RevA_GetStopBits((mxc_uart_reva_regs_t *)uart);
}

#define CONVERT_ZEPHYR_DATABITS(x) (x + 5)

static int MXC_UART_RevA_GetDataSize(mxc_uart_reva_regs_t *uart)
{
	if (MXC_UART_GET_IDX((mxc_uart_regs_t *)uart) < 0) {
		return E_BAD_PARAM;
	}

	return MXC_GETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_CHAR_SIZE);
}

int MXC_UART_GetDataSize(mxc_uart_regs_t *uart)
{
	return MXC_UART_RevA_GetDataSize((mxc_uart_reva_regs_t *)uart);
}

/* Helper function to convert zephyr parity to mxc parity */
static inline unsigned int __zephyr_to_mxc_parity(unsigned int parity)
{
	switch (parity) {
	case UART_CFG_PARITY_NONE:
		return MXC_UART_PARITY_DISABLE;
	case UART_CFG_PARITY_ODD:
		return MXC_UART_PARITY_ODD;
	case UART_CFG_PARITY_EVEN:
		return MXC_UART_PARITY_EVEN;
	case UART_CFG_PARITY_MARK:
		return MXC_UART_PARITY_MARK;
	case UART_CFG_PARITY_SPACE:
		return MXC_UART_PARITY_SPACE;
	default:
		return -ENOTSUP;
	}
}

#define CONVERT_ZEPHYR_TO_MXC_PARITY(x) __zephyr_to_mxc_parity(x)

/* Helper function to convert mxc parity to zephyr parity */
static inline unsigned int __mxc_to_zephyr_parity(unsigned int parity)
{
	switch (parity) {
	case MXC_UART_PARITY_DISABLE:
		return UART_CFG_PARITY_NONE;
	case MXC_UART_PARITY_ODD:
		return UART_CFG_PARITY_ODD;
	case MXC_UART_PARITY_EVEN:
		return UART_CFG_PARITY_EVEN;
	case MXC_UART_PARITY_MARK:
		return UART_CFG_PARITY_MARK;
	case MXC_UART_PARITY_SPACE:
		return UART_CFG_PARITY_SPACE;
	default:
		return -ENOTSUP;
	}
}

#define CONVERT_MXC_TO_ZEPHYR_PARITY(x) __mxc_to_zephyr_parity(x)

static int MXC_UART_RevA_GetParity(mxc_uart_reva_regs_t *uart)
{
	if (MXC_UART_GET_IDX((mxc_uart_regs_t *)uart) < 0) {
		return E_BAD_PARAM;
	}

	unsigned int parity_en;
	parity_en = MXC_GETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY_EN);
	if (parity_en == 0) {
		return MXC_UART_PARITY_DISABLE;
	}

	switch (MXC_GETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY)) {
	case 0x00:
		return MXC_UART_PARITY_EVEN;
	case 0x01:
		return MXC_UART_PARITY_ODD;
	case 0x02:
		return MXC_UART_PARITY_MARK;
	case 0x03:
		return MXC_UART_PARITY_SPACE;
	default:
		return -ENOTSUP;
	}
}

static int MXC_UART_GetDataParity(mxc_uart_regs_t *uart)
{
	return MXC_UART_RevA_GetParity((mxc_uart_reva_regs_t *)uart);
}

static int uart_max32_configure(const struct device *dev, const struct uart_config *uart_cfg)
{
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *uart = (mxc_uart_regs_t *)cfg->uart;
	int err;
	err = MXC_UART_SetParity(uart, CONVERT_ZEPHYR_TO_MXC_PARITY(uart_cfg->parity));
	if (err < 0) {
		return -ENOTSUP;
	}

	if (uart_cfg->stop_bits == UART_CFG_STOP_BITS_1) {
		err = MXC_UART_SetStopBits(uart, MXC_UART_STOP_1);
	} else if (uart_cfg->stop_bits == UART_CFG_STOP_BITS_1_5 ||
		   cfg->stop_bits == UART_CFG_STOP_BITS_2) {
		err = MXC_UART_SetStopBits(uart, MXC_UART_STOP_2);
	} else {
		return -ENOTSUP;
	}
	if (err < 0) {
		return -ENOTSUP;
	}

	err = MXC_UART_SetDataSize(uart, CONVERT_ZEPHYR_DATABITS(uart_cfg->data_bits));
	if (err < 0) {
		return -ENOTSUP;
	}

	// Flow control should be written here.

	err = MXC_UART_SetFrequency(uart, uart_cfg->baudrate);
	if (err < 0) {
		return -ENOTSUP;
	}

	return 0;
}

static int uart_max32_config_get(const struct device *dev, struct uart_config *uart_cfg)
{
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *uart = (mxc_uart_regs_t *)cfg->uart;

	uart_cfg->parity = CONVERT_MXC_TO_ZEPHYR_PARITY(MXC_UART_GetDataParity(uart));
	uart_cfg->stop_bits =
		CONVERT_MXC_TO_ZEPHYR_STOPBITS(MXC_UART_GetStopBits(uart), uart_cfg->data_bits);
	uart_cfg->data_bits = MXC_UART_GetDataSize(uart);
	// Flow control should be written here.
	uart_cfg->baudrate = MXC_UART_GetFrequency(uart);

	return 0;
}

#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uart_max32_init(const struct device *dev)
{
	const struct uart_max32_config *const cfg = dev->config;
	mxc_uart_regs_t *uart = (mxc_uart_regs_t *)cfg->uart;
	int ret;

	if (!device_is_ready(cfg->clock)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
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

	MXC_UART_RevA_Init((mxc_uart_reva_regs_t *)uart, cfg->baud_rate);

	return 0;
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

#define MAX32_UART_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct uart_max32_config uart_max32_config_##n = {                            \
		.uart = (mxc_uart_regs_t *)DT_INST_REG_ADDR(n),                                    \
		.baud_rate = DT_INST_PROP(n, current_speed),                                       \
		.parity = DT_INST_ENUM_IDX_OR(n, parity, UART_CFG_PARITY_NONE),                    \
		.pctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                        \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                    \
		.perclk.bus = DT_INST_CLOCKS_CELL(n, offset),                                       \
		.perclk.bit = DT_INST_CLOCKS_CELL(n, bit),                                          \
		.use_obrc = DT_INST_ENUM_IDX(n, clock_source),                                     \
		.data_bits = DT_INST_ENUM_IDX(n, data_bits),                                       \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, uart_max32_init, NULL, (mxc_uart_regs_t *)DT_INST_REG_ADDR(n),    \
			      &uart_max32_config_##n, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,   \
			      (void *)&uart_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX32_UART_INIT)
