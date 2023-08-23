/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_pwm

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/sys/util_macro.h>

#include <tmr.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_max32, CONFIG_PWM_LOG_LEVEL);

#define TMR_CFG(dev) ((struct max32_pwm_config *) ((dev)->config))
#define TMR_DATA(dev) ((struct max32_pwm_data *) ((dev)->data))

/** PWM configuration. */
struct max32_pwm_config {
	/** Timer register. */
	mxc_tmr_regs_t *regs;
	mxc_tmr_cfg_t *cfg;
};

/** PWM data. */
struct max32_pwm_data {
	/** Timer clock (Hz). */
	uint32_t tim_clk;
};

static int pwm_max32_set_cycles(const struct device *dev, uint32_t channel,
			      uint32_t period_cycles, uint32_t pulse_cycles,
			      pwm_flags_t flags)
{
	mxc_tmr_regs_t *regs = TMR_CFG(dev)->regs;
	mxc_tmr_cfg_t cfg;
	int ret;

	cfg.pres = TMR_PRES_1;
    cfg.mode = TMR_MODE_PWM;
    cfg.cmp_cnt = period_cycles;
	cfg.pol = 1;

	MXC_TMR_Shutdown(regs);

	MXC_TMR_Init(regs, &cfg);

	ret = MXC_TMR_SetPWM(regs, pulse_cycles);
	if (ret != E_NO_ERROR) {
		// Failed TMR_PWMConfig
		return ret;
    }

	MXC_TMR_Start(regs);

	return 0;
}

static int pwm_max32_get_cycles_per_sec(const struct device *dev,
				       uint32_t channel, uint64_t *cycles)
{
	*cycles = (uint64_t)(PeripheralClock / (TMR_PRES_1 + 1));

	return 0;
}

static const struct pwm_driver_api pwm_max32_driver_api = {
	.set_cycles = pwm_max32_set_cycles,
	.get_cycles_per_sec = pwm_max32_get_cycles_per_sec,
};

static int pwm_max32_init(const struct device *dev)
{
	mxc_tmr_regs_t *regs = TMR_CFG(dev)->regs;
	mxc_tmr_cfg_t cfg;
	int ret;

	MXC_TMR_Shutdown(regs);

	cfg.pres = TMR_PRES_1;
    cfg.mode = TMR_MODE_PWM;
    cfg.cmp_cnt = PeripheralClock;
	cfg.pol = 1;

	MXC_TMR_Init(regs, &cfg);

	ret = MXC_TMR_SetPWM(regs, 0);
	if (ret != E_NO_ERROR) {
		// Failed TMR_PWMConfig
		return ret;
    }

    MXC_TMR_Start(regs);

	return 0;
}

#define PWM_MAX32_DEFINE(i)						                             \
	static struct max32_pwm_data max32_pwm_data_##i;			             \
									                                         \
	static const struct max32_pwm_config max32_pwm_config_##i = {	         \
		.regs = (mxc_tmr_regs_t *)DT_REG_ADDR(DT_INST_PARENT(i)),		     \
	};								                                         \
									                                         \
	DEVICE_DT_INST_DEFINE(i, &pwm_max32_init, NULL, &max32_pwm_data_##i,     \
			      &max32_pwm_config_##i, POST_KERNEL,	                     \
			      CONFIG_PWM_INIT_PRIORITY,			                         \
			      &pwm_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_MAX32_DEFINE)
