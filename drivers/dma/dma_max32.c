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

struct max32_dma_config {
	mxc_dma_regs_t *regs;
	const struct device *clock;
	struct max32_perclk perclk;
    uint8_t channels;
};

struct max32_dma_data {
    dma_callback_t callback;
    void *cb_data;
    uint32_t err_cb_en;
};

static int is_valid_dma_width(uint32_t width)
{
    switch(width) {
        case MXC_DMA_WIDTH_BYTE:
        case MXC_DMA_WIDTH_HALFWORD:
        case MXC_DMA_WIDTH_WORD:
            return 1;
    };
    return 0;
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

/*
 * APIs
 */

static inline int max32_dma_config(const struct device *dev,
					     uint32_t channel,
					     struct dma_config *config)
{
    int ret = 0;
    const struct max32_dma_config *cfg = dev->config;
    struct max32_dma_data *data = dev->data;

    if (channel >= cfg->channels) {
		LOG_ERR("Invalid DMA channel - must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, channel);
		return -EINVAL;
	}

    /* DMA Channel Config */
    mxc_dma_config_t mxc_dma_cfg;
    mxc_dma_cfg.ch = channel;
    mxc_dma_cfg.reqsel = MXC_DMA_REQUEST_MEMTOMEM; // initial test
    if (!(is_valid_dma_width(config->source_data_size) \
                        || !(is_valid_dma_width(config->dest_data_size))))
    {
        LOG_ERR("Invalid DMA width - must be byte (1), halfword (2) or word (4) !");
        return -EINVAL;
	}
    mxc_dma_cfg.srcwd = dma_width_z_to_mxc(config->source_data_size);
    mxc_dma_cfg.dstwd = dma_width_z_to_mxc(config->dest_data_size);
    // mxc_dma_cfg.srcinc_en = config->source_chaining_en;
    // mxc_dma_cfg.dstinc_en = config->dest_chaining_en;
    mxc_dma_cfg.srcinc_en = 1;
    mxc_dma_cfg.dstinc_en = 1;

    /* DMA Channel Advanced Config */
    mxc_dma_adv_config_t mxc_dma_cfg_adv;
    mxc_dma_cfg_adv.ch = channel;
    if (!is_valid_dma_ch_prio(config->channel_priority))
    {
        LOG_ERR("Invalid DMA priority - must comply with type mxc_dma_priority_t (0 - 3)");
        return -EINVAL;
    }
    mxc_dma_cfg_adv.prio = config->channel_priority;
    mxc_dma_cfg_adv.reqwait_en = 0;
    mxc_dma_cfg_adv.tosel = MXC_DMA_TIMEOUT_4_CLK;
    mxc_dma_cfg_adv.pssel = MXC_DMA_PRESCALE_DISABLE;
    mxc_dma_cfg_adv.burst_size = config->source_burst_length;

    /* DMA Transfer Config */
    mxc_dma_srcdst_t txfer;
    txfer.ch = channel;
    txfer.source = (void *)config->head_block->source_address;
    txfer.dest = (void *)config->head_block->dest_address;
    txfer.len = config->head_block->block_size;

    /* Call required by SDK DMA driver, 
     * but we will use channel passed as argument */
    MXC_DMA_AcquireChannel(); 

    ret = MXC_DMA_ConfigChannel(mxc_dma_cfg, txfer);
    if (ret != E_NO_ERROR) {
		return ret;
	}

    ret = MXC_DMA_AdvConfigChannel(mxc_dma_cfg_adv);
    if (ret) {
		return ret;
	}

    /* Enable interrupts for the DMA peripheral */
    ret = MXC_DMA_EnableInt(channel); 
    if (ret != E_NO_ERROR) {
		return ret;
	}

    /* Enable complete and count-to-zero interrupts for the channel */
    ret = MXC_DMA_ChannelEnableInt(channel, MXC_F_DMA_CTRL_DIS_IE | MXC_F_DMA_CTRL_CTZ_IE);
    if (ret != E_NO_ERROR) {
		return ret;
	}

    data[channel].callback = config->dma_callback;
    data[channel].cb_data = config->user_data;
    data[channel].err_cb_en = config->error_callback_en;

    return ret;
}

static inline int max32_dma_reload(const struct device *dev, uint32_t channel, 
                        uint32_t src, uint32_t dst, size_t size)
{
    mxc_dma_srcdst_t reload;
    reload.ch = channel;
    reload.source = (void *)src;
    reload.dest = (void *)dst;
    reload.len = size;
    return MXC_DMA_SetSrcReload(reload);
}

int max32_dma_start(const struct device *dev, uint32_t channel)
{
    const struct max32_dma_config *cfg = dev->config;

    if (channel >= cfg->channels) {
		LOG_ERR("Invalid DMA channel - must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, channel);
		return -EINVAL;
	}
    return MXC_DMA_Start(channel);
}

int max32_dma_stop(const struct device *dev, uint32_t channel)
{
    const struct max32_dma_config *cfg = dev->config;
    
    if (channel >= cfg->channels) {
		LOG_ERR("Invalid DMA channel - must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, channel);
		return -EINVAL;
	}
    return MXC_DMA_Stop(channel);
}

static inline int max32_dma_get_status(const struct device *dev, 
                        uint32_t channel, struct dma_status *stat)
{
    const struct max32_dma_config *cfg = dev->config;

    if (channel >= cfg->channels) {
        LOG_ERR("Invalid DMA channel - must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, channel);
        return -EINVAL;
	}

    int flags = 0;
    flags = MXC_DMA_ChannelGetFlags(channel);

    /* Channel is busy if no interrupt pending */
    stat->busy = !(flags & MXC_F_DMA_STATUS_IPEND);

    return 0;
}

static void max32_dma_isr(const struct device *dev)
{
    const struct max32_dma_config *cfg = dev->config;
    struct max32_dma_data *data = dev->data;
    mxc_dma_regs_t *regs = cfg->regs;
    int ch;
    int flags;
    int status = 0;

    for (ch = 0; ch < cfg->channels; ch++) {
        flags = MXC_DMA_ChannelGetFlags(ch);

        /* Check if channel is in use, if not, move to next channel */
        if (flags <= 0) {
            continue;
        }

        /* Check for error interrupts */
        if (flags & (MXC_F_DMA_STATUS_BUS_ERR | MXC_F_DMA_STATUS_TO_IF))
        {
            status = -EIO;
        }

        if (data[ch].callback) {
            /* Only call error callback if enabled during DMA config */
            if (status < 0 && (!data[ch].err_cb_en)) { break; }
            data[ch].callback(dev, data[ch].cb_data, ch, status);
        }

        MXC_DMA_ChannelClearFlags(ch, flags);

        /* No need to check rest of the channels if no interrupt flags set */
        if (regs->intfl == 0)
            break;
    }
}

#define dma DT_NODELABEL(dma)
    
#define MAX32_DMA_IRQ_CONNECT(n, inst)				\
        IRQ_CONNECT(DT_IRQ_BY_IDX(inst, n, irq),    \
                DT_IRQ_BY_IDX(inst, n, priority),   \
                max32_dma_isr,                      \
                DEVICE_DT_GET(inst), 0);            \
        irq_enable(DT_IRQ_BY_IDX(inst, n, irq));    

#define CONFIGURE_ALL_IRQS(n) LISTIFY(n, MAX32_DMA_IRQ_CONNECT, (), dma)

static int max32_dma_init(const struct device *dev)
{
    int ret = 0;
	const struct max32_dma_config *cfg = dev->config;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}

	/* Enable peripheral clock */
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)&(cfg->perclk));
	if (ret) {
		return ret;
	}

    CONFIGURE_ALL_IRQS(DT_NUM_IRQS(dma));

    return MXC_DMA_Init();
}

static const struct dma_driver_api max32_dma_driver_api = {
	.config = max32_dma_config,
	.reload = max32_dma_reload,
	.start = max32_dma_start,
	.stop = max32_dma_stop,
	.get_status = max32_dma_get_status,
};


static const struct max32_dma_config dma_cfg = { 
    .regs = (mxc_dma_regs_t *)DT_REG_ADDR(dma), 
    .clock = DEVICE_DT_GET(DT_CLOCKS_CTLR(dma)), 
    .perclk.bus = DT_CLOCKS_CELL(dma, offset), 
    .perclk.bit = DT_CLOCKS_CELL(dma, bit), 
    .channels = DT_PROP(dma, dma_channels), 
};

/* Callback data for each channel */
static struct max32_dma_data dma_data[DT_PROP(dma, dma_channels)];

DEVICE_DT_DEFINE(dma,                   
            &max32_dma_init, 
            NULL, &dma_data, 
            &dma_cfg, POST_KERNEL, 
            CONFIG_DMA_INIT_PRIORITY, 
            &max32_dma_driver_api);
