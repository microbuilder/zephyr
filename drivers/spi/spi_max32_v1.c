/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_spi

#include <string.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>

#include <spi.h>

struct max32_spi_config {
	mxc_spi_regs_t *spi;
	const struct pinctrl_dev_config *pctrl;
	const struct device *clock;
	struct max32_perclk perclk;
};

static int spi_max32_transceive(const struct device *dev, const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	int ret = 0;
	const struct max32_spi_config *const cfg = dev->config;
	mxc_spi_regs_t *spi = cfg->spi;
	mxc_spi_req_t req;
	int i;
	int nb_tx_packet = tx_bufs->count;
	int nb_rx_packet = rx_bufs->count;
	int nb_of_packet = MAX(nb_tx_packet, nb_rx_packet);


	req.spi = spi;

	for (i=0; i < nb_of_packet; i++) {
	
		if (tx_bufs->buffers && tx_bufs->buffers[i].buf) {
			req.txData = (uint8_t *)tx_bufs->buffers[i].buf;
			req.txLen = tx_bufs->buffers[i].len;
		} else {
			req.txData = NULL;
			req.txLen = 0;
		}

		if (rx_bufs->buffers && rx_bufs->buffers[i].buf) {
			req.rxData = (uint8_t *)rx_bufs->buffers[i].buf;
			req.rxLen = rx_bufs->buffers[i].len;
		} else {
			req.rxData = NULL;
			req.rxLen = 0;
		}
		
		req.ssIdx = 0;
		req.ssDeassert = 1;
		req.txCnt = 0;
		req.rxCnt = 0;

		ret = MXC_SPI_MasterTransaction(&req); 
		if (ret) {
			//ret = -EIO;
			break; // error occurs
		}
	}

	return ret;
}

static int spi_max32_release(const struct device *dev, const struct spi_config *config)
{

	return 0;
}

/* API implementation: init */
static int spi_max32_init(const struct device *dev)
{
	int ret = 0;
	const struct max32_spi_config *const cfg = dev->config;
	mxc_spi_regs_t *spi = cfg->spi;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}

	MXC_SPI_Shutdown(spi);

	/* enable clock */
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)&(cfg->perclk));
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pctrl, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	int masterMode   = 1;
    int quadModeUsed = 0;
    int numSlaves    = 1;
    int ssPolarity   = 0;
    unsigned int spi_speed = 1000000;

    ret = MXC_SPI_Init(spi, masterMode, quadModeUsed, numSlaves, ssPolarity, spi_speed);
    if (ret) {
    	return ret;
    }

	ret = MXC_SPI_SetDataSize(spi, 8);
    if (ret) {
    	return ret;
    }

    ret = MXC_SPI_SetWidth(spi, SPI_WIDTH_STANDARD);
	if (ret) {
    	return ret;
    }

	return ret;
}

/* SPI driver APIs structure */
static struct spi_driver_api spi_max32_api = {
	.transceive = spi_max32_transceive,
	.release = spi_max32_release,
};

/* SPI driver registration */
#define DEFINE_SPI_MAX32(_num)                                       \
	PINCTRL_DT_INST_DEFINE(_num);                                    \
																	 \
	static const struct max32_spi_config max32_spi_config_##_num = { \
		.spi = (mxc_spi_regs_t *)DT_INST_REG_ADDR(_num),             \
		.pctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(_num),               \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(_num)),           \
		.perclk.bus = DT_INST_CLOCKS_CELL(_num, offset),             \
		.perclk.bit = DT_INST_CLOCKS_CELL(_num, bit),                \
	};                                                               \
																	 \
	DEVICE_DT_INST_DEFINE( \
		_num, \
		spi_max32_init, \
		NULL, \
		(mxc_spi_req_t *)DT_INST_REG_ADDR(_num), \
		&max32_spi_config_##_num, \
		PRE_KERNEL_2, \
		CONFIG_SPI_INIT_PRIORITY, \
		&spi_max32_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_SPI_MAX32)
