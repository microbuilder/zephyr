/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>

#include <dma.h>

#define DT_DRV_COMPAT adi_max32_dma

LOG_MODULE_REGISTER(max32_dma, CONFIG_DMA_LOG_LEVEL);

static int is_valid_dma_width(uint32_t width)
{
    int valid = 0;

    switch(width) {
        case MXC_DMA_WIDTH_BYTE:
        case MXC_DMA_WIDTH_HALFWORD:
        case MXC_DMA_WIDTH_WORD:
            valid = 1;
    };
    return valid;
}

static int is_valid_dma_ch_prio(uint32_t ch_prio)
{
    /* mxc_dma_priority_t is limited to values 0-3 */
    return ((ch_prio >= 0 && ch_prio < 4) ? 1 : 0);
}

static mxc_dma_width_t dma_width_z_to_mxc(uint32_t width)
{
    switch(width)
    {
        case 1:
            return MXC_DMA_WIDTH_BYTE;
            break;
        case 2:
            return MXC_DMA_WIDTH_HALFWORD;
            break;
        case 4:
            return MXC_DMA_WIDTH_WORD;
            break;
        default:
            return 0;
    }
}

struct max32_dma_config {
	mxc_dma_regs_t *regs;
	const struct device *clock;
	struct max32_perclk perclk;
};

/*
 * APIs
 */

static inline int max32_dma_config(const struct device *dev,
					     uint32_t channel,
					     struct dma_config *config)
{
    int ret = 0;

    /* DMA Channel Config */
    mxc_dma_config_t dma_cfg;
    dma_cfg.ch = channel;
    dma_cfg.reqsel = MXC_DMA_REQUEST_MEMTOMEM; // initial test
    if (!(is_valid_dma_width(config->source_data_size) \
                        || !(is_valid_dma_width(config->dest_data_size))))
    {
        LOG_ERR("Invalid DMA width - must be byte (1), halfword (2) or word (4) !");
        return -ENOTSUP;
	}
    dma_cfg.srcwd = dma_width_z_to_mxc(config->source_data_size);
    dma_cfg.dstwd = dma_width_z_to_mxc(config->dest_data_size);
    // dma_cfg.srcinc_en = config->source_chaining_en;
    // dma_cfg.dstinc_en = config->dest_chaining_en;
    dma_cfg.srcinc_en = 1;
    dma_cfg.dstinc_en = 1;

    /* DMA Channel Advanced Config */
    mxc_dma_adv_config_t dma_cfg_adv;
    dma_cfg_adv.ch = channel;
    if (!is_valid_dma_ch_prio(config->channel_priority))
    {
        LOG_ERR("Invalid DMA priority - must comply with type mxc_dma_priority_t (0 - 3)");
        return -ENOTSUP;
    }
    dma_cfg_adv.prio = config->channel_priority;
    dma_cfg_adv.reqwait_en = 0;
    dma_cfg_adv.tosel = MXC_DMA_TIMEOUT_4_CLK;
    dma_cfg_adv.pssel = MXC_DMA_PRESCALE_DISABLE;
    dma_cfg_adv.burst_size = config->source_burst_length;

    /* DMA Transfer Config */
    mxc_dma_srcdst_t txfer;
    txfer.ch = channel;
    txfer.source = (void *)config->head_block->source_address;
    txfer.dest = (void *)config->head_block->dest_address;
    txfer.len = config->head_block->block_size;

    /* Call required by SDK DMA driver, 
     * but will use channel passed as argument */
    MXC_DMA_AcquireChannel(); 

    ret = MXC_DMA_ConfigChannel(dma_cfg, txfer);
    if (ret != E_NO_ERROR) {
		return ret;
	}

    ret = MXC_DMA_AdvConfigChannel(dma_cfg_adv);
    if (ret) {
		return ret;
	}

    ret = MXC_DMA_EnableInt(channel); 
    if (ret != E_NO_ERROR) {
		return ret;
	}

    ret = MXC_DMA_ChannelEnableInt(channel, MXC_F_DMA_CTRL_CTZ_IE);
    if (ret != E_NO_ERROR) {
		return ret;
	}
    return ret;
}

static inline int max32_dma_reload(const struct device *dev, uint32_t channel, 
                        uint32_t src, uint32_t dst, size_t size)
{
    return 0;
}

int max32_dma_start(const struct device *dev, uint32_t channel)
{
    return MXC_DMA_Start(channel);
}

int max32_dma_stop(const struct device *dev, uint32_t channel)
{
    return MXC_DMA_ReleaseChannel(channel);
}

static inline int max32_dma_get_status(const struct device *dev, 
                        uint32_t channel, struct dma_status *stat)
{
    return 0;
}

static void max32_dma_isr(const struct device *dev)
{
    printk("In max32_dma_isr\n");
    int flags;
    flags = MXC_DMA_ChannelGetFlags(0);
    MXC_DMA_ChannelClearFlags(0, flags);
    flags = MXC_DMA_ChannelGetFlags(1);
    MXC_DMA_ChannelClearFlags(1, flags);
}
    
#define MAX32_DMA_IRQ_CONNECT(n, inst)				\
        IRQ_CONNECT(DT_IRQ_BY_IDX(inst, n, irq),    \
                DT_IRQ_BY_IDX(inst, n, priority),   \
                max32_dma_isr, NULL, 0);            \
        irq_enable(DT_IRQ_BY_IDX(inst, n, irq));    \

#define CONFIGURE_ALL_IRQS(n) LISTIFY(n, MAX32_DMA_IRQ_CONNECT, (), DT_NODELABEL(dma))

static int max32_dma_init(const struct device *dev)
{
    int ret = 0;
	const struct max32_dma_config *const cfg = dev->config;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}

	/* Enable peripheral clock */
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)&(cfg->perclk));
	if (ret) {
		return ret;
	}

    CONFIGURE_ALL_IRQS(DT_NUM_IRQS(DT_NODELABEL(dma)));

    return MXC_DMA_Init();
}

static const struct dma_driver_api max32_dma_driver_api = {
	.config = max32_dma_config,
	.reload = max32_dma_reload,
	.start = max32_dma_start,
	.stop = max32_dma_stop,
	.get_status = max32_dma_get_status,
	//.chan_filter = dma_max32_api_chan_filter,
};

static const struct max32_dma_config dma_cfg = { 
    .regs = (mxc_dma_regs_t *)DT_INST_REG_ADDR(0), 
    .clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)), 
    .perclk.bus = DT_INST_CLOCKS_CELL(0, offset), 
    .perclk.bit = DT_INST_CLOCKS_CELL(0, bit), 
};

DEVICE_DT_DEFINE(DT_NODELABEL(dma), 
            &max32_dma_init, 
            NULL, NULL, 
            &dma_cfg, POST_KERNEL, 
            CONFIG_DMA_INIT_PRIORITY, 
            &max32_dma_driver_api);
