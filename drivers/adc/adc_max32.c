/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_adc

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>

#include <zephyr/logging/log.h>
#include <adc.h>

LOG_MODULE_REGISTER(adc_max32, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

struct max32_adc_conf {
	uint8_t channel_count;
	mxc_adc_regs_t *regs;
	void (*irq_config_func)(const struct device *dev);
};

struct max32_adc_data {
	const struct device *dev;
	struct adc_context ctx;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
	uint8_t channel_id;
};

void adc_complete_cb(void *req, int error)
{
    return;
}

static void adc_max32_start_channel(const struct device *dev)
{
	struct max32_adc_data *data = dev->data;
	data->channel_id = find_lsb_set(data->channels) - 1;

	LOG_DBG("Starting channel %d", data->channel_id);
	// Use dummy callbackk?
	MXC_ADC_StartConversionAsync(data->channel_id, adc_complete_cb);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct max32_adc_data *data = CONTAINER_OF(ctx, struct max32_adc_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	adc_max32_start_channel(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct max32_adc_data *data = CONTAINER_OF(ctx, struct max32_adc_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int start_read(const struct device *dev, const struct adc_sequence *seq)
{
	struct max32_adc_data *data = dev->data;

	if (seq->resolution != 10) {
		LOG_ERR("Unsupported resolution (%d)", seq->resolution);
		return -ENOTSUP;
	}

	data->buffer = seq->buffer;
	data->channel_id = seq->channels;

	adc_context_start_read(&data->ctx, seq);

	return adc_context_wait_for_completion(&data->ctx);
}

static void adc_max32_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct max32_adc_data *data = dev->data;
	uint32_t flags;

	flags = MXC_ADC_GetFlags();
	MXC_ADC_GetData(data->buffer);
	LOG_DBG("Finished channel %d. Result is 0x%04x",
		data->channel_id, *data->buffer);
	MXC_ADC_ClearFlags(flags);

	data->buffer++;
	data->channels &= ~BIT(data->channel_id);
	if (data->channels) {
		adc_max32_start_channel(dev);
	} else {
		adc_context_on_sampling_done(&data->ctx, dev);
	}
}

static int adc_max32_read(const struct device *dev, const struct adc_sequence *seq)
{

	struct max32_adc_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = start_read(dev, seq);
	adc_context_release(&data->ctx, error);

	return error;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_max32_read_async(const struct device *dev, const struct adc_sequence *seq,
				struct k_poll_signal *async)
{
	struct max32_adc_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, true, async);
	error = start_read(dev, seq);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif /* CONFIG_ADC_ASYNC */

static int adc_max32_channel_setup(const struct device *dev, const struct adc_channel_cfg *cfg)
{
	const struct max32_adc_conf *conf = (const struct max32_adc_conf *)dev->config;

	if (cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Gain is not valid");
		return -ENOTSUP;
	}

	if (cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Reference is not valid");
		return -ENOTSUP;
	}

	if (cfg->differential) {
		LOG_ERR("Differential sampling not supported");
		return -ENOTSUP;
	}

	if (cfg->channel_id >= conf->channel_count) {
		LOG_ERR("Invalid channel (%u)", cfg->channel_id);
		return -EINVAL;
	}

	if (cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Invalid channel acquisition time");
		return -EINVAL;
	}

	if (cfg->channel_id <= 0x7) { // MACRO is in the reva so keep it val
		MXC_ADC_SetExtScale(MXC_ADC_SCALE_1);
	}

	return 0;
}

static int adc_max32_init(const struct device *dev)
{
	const struct max32_adc_conf *config = dev->config;
	struct max32_adc_data *data = dev->data;

	MXC_ADC_Init();

	config->irq_config_func(dev);
	data->dev = dev;

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api api_max32_driver_api = {
	.channel_setup = adc_max32_channel_setup,
	.read = adc_max32_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_max32_read_async,
#endif /* CONFIG_ADC_ASYNC */
};

#define MAX32_ADC_INIT(inst)                                                                       \
	static void max32_adc_config_func_##inst(const struct device *dev);                        \
                                                                                                   \
	static const struct max32_adc_conf max32_adc_conf_##inst = {                               \
		.channel_count = DT_PROP(DT_DRV_INST(inst), channel_count),                        \
		.regs = (mxc_adc_regs_t *)DT_INST_REG_ADDR(inst),                                  \
		.irq_config_func = max32_adc_config_func_##inst,                                   \
	};                                                                                         \
                                                                                                   \
	static struct max32_adc_data max32_adc_data_##inst = {                                     \
		ADC_CONTEXT_INIT_TIMER(max32_adc_data_##inst, ctx),                                \
		ADC_CONTEXT_INIT_LOCK(max32_adc_data_##inst, ctx),                                 \
		ADC_CONTEXT_INIT_SYNC(max32_adc_data_##inst, ctx),                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &adc_max32_init, NULL, &max32_adc_data_##inst,                 \
			      &max32_adc_conf_##inst, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,       \
			      &api_max32_driver_api);                                              \
	static void max32_adc_config_func_##inst(const struct device *dev)                         \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), adc_max32_isr,        \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
                                                                                                   \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

DT_INST_FOREACH_STATUS_OKAY(MAX32_ADC_INIT)
