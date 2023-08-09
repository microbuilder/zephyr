/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/util.h"

#define DT_DRV_COMPAT adi_max32_trng

#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>

#include "trng.h"

struct max32_trng_cfg {
	mxc_trng_regs_t *regs;
};

static int entropy_max32_get_entropy(const struct device *dev, uint8_t *buffer,
				   uint16_t length)
{
        return MXC_TRNG_Random(buffer, length);
}

static int entropy_max32_get_entropy_isr(const struct device *dev,
				       uint8_t *buffer,
				       uint16_t length, uint32_t flags)
{
	return 0;
}

static int entropy_max32_init(const struct device *dev)
{
	return MXC_TRNG_Init();
}

static const struct entropy_driver_api entropy_max32_api = {
	.get_entropy = entropy_max32_get_entropy,
	.get_entropy_isr = entropy_max32_get_entropy_isr
};

static const struct max32_trng_cfg max32_trng_cfg = {
	.regs = (mxc_trng_regs_t *)DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0,
		    entropy_max32_init, 
		    NULL,
		    NULL, 
		    &max32_trng_cfg,
		    PRE_KERNEL_1, 
		    CONFIG_ENTROPY_INIT_PRIORITY,
		    &entropy_max32_api);
