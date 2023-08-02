/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_flash_controller
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

#define FLASH_MAX32_BASE_ADDRESS 0x10000000

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>

#include "flc.h"

struct flash_max32_dev_data {
	struct k_sem sem;
};

struct flash_max32_dev_config {
	uint32_t base;
	uint32_t size;
};

static const struct flash_parameters flash_max32_parameters = {
	.write_block_size = FLASH_WRITE_BLK_SZ,
	.erase_value = 0xff,
};

static int flash_max32_read(const struct device *dev, off_t address, void *buffer, size_t length)
{
	int ret = 0;

	MXC_FLC_Read(address + FLASH_MAX32_BASE_ADDRESS, buffer, length);

	return ret;
}

static int flash_max32_write(const struct device *dev, off_t address, const void *buffer, size_t length)
{
	int ret = 0;
	
	ret = MXC_FLC_Write(address + FLASH_MAX32_BASE_ADDRESS, length, buffer);

	return ret;
}

static int flash_max32_erase(const struct device *dev, off_t start, size_t len)
{
	int ret = 0;

	ret = MXC_FLC_PageErase(start + FLASH_MAX32_BASE_ADDRESS);
	
	return ret;
}

#if CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_max32_pages_layout = {
	.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) / FLASH_ERASE_BLK_SZ,
	.pages_size = DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
};

void flash_max32_page_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	*layout = &flash_max32_pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *
flash_max32_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_max32_parameters;
}

static int flash_max32_init(const struct device *dev)
{
	struct flash_max32_dev_data *const dev_data = dev->data;

	return MXC_FLC_Init();
}

static const struct flash_driver_api flash_max32_driver_api = {
	.read = flash_max32_read,
	.write = flash_max32_write,
	.erase = flash_max32_erase,
	.get_parameters = flash_max32_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_max32_page_layout,
#endif
};

static struct flash_max32_dev_data flash_max32_data;

static struct flash_max32_dev_config flash_max32_config = {
	.base = DT_REG_ADDR(DT_INST(0, soc_nv_flash)),
	.size = DT_REG_SIZE(DT_INST(0, soc_nv_flash))};

DEVICE_DT_INST_DEFINE(0, flash_max32_init,
		      NULL,
		      &flash_max32_data, &flash_max32_config,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &flash_max32_driver_api);
