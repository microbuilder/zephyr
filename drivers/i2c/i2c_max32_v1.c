/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_i2c

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>

#include <i2c.h>


/* Driver config */
struct i2c_max32_config {
	mxc_i2c_regs_t *i2c;
	const struct pinctrl_dev_config *pctrl;
	const struct device *clock;
	struct max32_perclk perclk;
	uint32_t bitrate;
};

static int i2c_max32_configure(const struct device *dev, uint32_t dev_cfg)
{
	int ret = 0;
	const struct i2c_max32_config *const cfg = dev->config;
	mxc_i2c_regs_t *i2c = cfg->i2c;

	switch( I2C_SPEED_GET(dev_cfg) ) {
		case I2C_SPEED_STANDARD: /** I2C Standard Speed: 100 kHz */
		ret = MXC_I2C_SetFrequency(i2c, MXC_I2C_STD_MODE);
		break;

		case I2C_SPEED_FAST: /** I2C Fast Speed: 400 kHz */
		ret = MXC_I2C_SetFrequency(i2c, MXC_I2C_FAST_SPEED);
		break;

		case I2C_SPEED_FAST_PLUS: /** I2C Fast Plus Speed: 1 MHz */
		ret = MXC_I2C_SetFrequency(i2c, MXC_I2C_FASTPLUS_SPEED);
		break;

		case I2C_SPEED_HIGH: /** I2C High Speed: 3.4 MHz */
		ret = MXC_I2C_SetFrequency(i2c, MXC_I2C_HIGH_SPEED);
		break;

		//case I2C_SPEED_ULTRA: /** I2C Ultra Fast Speed: 5 MHz */
		//break;		

		default:
		// Speed not supported
		return -ENOTSUP;
	}

	/* Only i2c master mode support */
	if (!(dev_cfg & I2C_MODE_CONTROLLER)) {
		//Only I2C master mode supported // Todo add slave too
		return -ENOTSUP;
	}

	return ret;
}

static int i2c_max32_target_register(const struct device *dev, struct i2c_target_config *cfg)
{

	return 0;
}

static int i2c_max32_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{

	return 0;
}

#ifdef CONFIG_I2C_CALLBACK
static int i2c_max32_transfer_cb(const struct device *dev, struct i2c_msg *msgs,
				 uint8_t num_msgs, uint16_t addr,
				 i2c_callback_t cb, void *userdata)
{

	return 0;
}

#endif /* CONFIG_I2C_CALLBACK */

#if defined(CONFIG_I2C_RTIO)
static void i2c_max32_iodev_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{

}
#endif /* CONFIG_I2C_RTIO */

static int i2c_max32_recover_bus(const struct device *dev)
{
	int ret;
	const struct i2c_max32_config *const cfg = dev->config;
	mxc_i2c_regs_t *i2c = cfg->i2c;
	
	ret = MXC_I2C_Recover(i2c, 3);

	return ret;
}

static int i2c_max32_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			      uint16_t slave_address)
{
	int ret;
	const struct i2c_max32_config *const cfg = dev->config;
	mxc_i2c_regs_t *i2c = cfg->i2c;
	unsigned int i = 0;
    mxc_i2c_req_t reqMaster;

	reqMaster.i2c = i2c;
    reqMaster.addr = slave_address;
	reqMaster.callback = NULL;

	for(i=0; i<num_msgs; i++) {

	    if (msgs[i].flags & I2C_MSG_READ) {
	    	reqMaster.rx_buf = (unsigned char *)msgs[i].buf;
	    	reqMaster.rx_len = msgs[i].len;	
	    	reqMaster.tx_buf = NULL;
	    	reqMaster.tx_len = 0;

	    } else {
	    	reqMaster.tx_buf = (unsigned char *)msgs[i].buf;
	    	reqMaster.tx_len = msgs[i].len;
	    	reqMaster.rx_buf = NULL;
	    	reqMaster.rx_len = 0;	
	    }

	    if (msgs[i].flags & I2C_MSG_STOP) {
	    	reqMaster.restart = 0;
		} else {
			reqMaster.restart = 1;
		}

		ret = MXC_I2C_MasterTransaction(&reqMaster);
		if (ret) {
			break;
		}
	} 

	return ret;
}

static struct i2c_driver_api api = {
	.configure = i2c_max32_configure,
	.transfer = i2c_max32_transfer,

	.target_register = i2c_max32_target_register,
	.target_unregister = i2c_max32_target_unregister,

#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = i2c_max32_transfer_cb,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_max32_iodev_submit,
#endif
	.recover_bus = i2c_max32_recover_bus,
};

static int i2c_max32_init(const struct device *dev)
{
	const struct i2c_max32_config *const cfg = dev->config;
	mxc_i2c_regs_t *i2c = cfg->i2c;
	int ret = 0;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}

	MXC_I2C_Shutdown(i2c); // Clear everything out

	/* enable clock */
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)&(cfg->perclk));
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pctrl, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	ret = MXC_I2C_Init(i2c, 1, 0); // configure as master
	// set frequency
	MXC_I2C_SetFrequency(i2c, cfg->bitrate);

	return ret;
}

#define DEFINE_I2C_MAX32(_num)                                                            \
	PINCTRL_DT_INST_DEFINE(_num);                                                         \
                                                                                          \
	static const struct i2c_max32_config i2c_max32_dev_cfg_##_num = {                     \
		.i2c = (mxc_i2c_regs_t *)DT_INST_REG_ADDR(_num),                                  \
		.pctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(_num),                                    \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(_num)),                                \
		.perclk.bus = DT_INST_CLOCKS_CELL(_num, offset),                                  \
		.perclk.bit = DT_INST_CLOCKS_CELL(_num, bit),                                     \
		.bitrate = DT_INST_PROP(_num, clock_frequency),                                   \
	};                                                                                    \
                                                                                          \
	I2C_DEVICE_DT_INST_DEFINE( \
		_num, \
		i2c_max32_init, \
		NULL,  \
		(mxc_i2c_regs_t *)DT_INST_REG_ADDR(_num), \
		&i2c_max32_dev_cfg_##_num, \
		PRE_KERNEL_2, \
		CONFIG_I2C_INIT_PRIORITY, \
		&api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_I2C_MAX32)
