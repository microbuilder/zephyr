/*
 * Copyright (c) 2021-2022 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include "psa/service.h"
#include "psa_manifest/tfm_tflm_service.h"
#include "psa/crypto.h"
#include "tfm_sp_log.h"
#include "tfm_plat_test.h"
#include "target_cfg.h"

#include "constants.h"
#include "tfm_huk_key_derivation_service_api.h"
#include <string.h>
// #include "Driver_I2C.h"
#include "platform_regs.h"

#include "main_functions.h"

typedef enum {
	CLIENT_TLS      = 0x5001,       // Client TLS key id
	C_SIGN          = 0x5002,       // COSE SIGN key id
	C_ENCRYPT       = 0x5003,       // COSE ENCRYPT key id
} key_type_t;

// These are the extra-label string passed as a seed to key derivation
// for generating 3 ec unique keys which are going to be used for the
// different use cases.
const uint8_t *label[3] = { (const uint8_t *)"CLIENT_TLS",
			    (const uint8_t *)"C_SIGN",
			    (const uint8_t *)"C_ENCRYPT" };

// /* I2C driver name for LSM303 peripheral */
// extern ARM_DRIVER_I2C LSM303_DRIVER;

// /* I2C address of LSM303 peripheral */
// extern const uint8_t lsm303_addr;

// /**
//  * \brief Send data to I2C peripheral
//  */

// static psa_status_t i2c_send_data(uint8_t* i2c_data,
//                                  uint32_t i2c_data_len,
//                                  bool xfer_pending)
// {
//     psa_status_t status = PSA_SUCCESS;

//     LSM303_DRIVER.MasterTransmit (lsm303_addr, i2c_data, i2c_data_len, xfer_pending);

//     /* Wait until transfer completed */
//     while (LSM303_DRIVER.GetStatus().busy);
//     /* Check if all data transferred */
//     if (LSM303_DRIVER.GetDataCount () != i2c_data_len) {
//         LOG_INFFMT("[Example partition] Master transmit data count didn't match...\r\n");
//         status = PSA_ERROR_HARDWARE_FAILURE;
//     }

//     return status;
// }

// /**
//  * \brief Receive data from I2C peripheral
//  */

// static psa_status_t i2c_receive_data(uint8_t* i2c_data,
//                                     uint32_t i2c_data_len,
//                                     bool xfer_pending)
// {
//     psa_status_t status = PSA_SUCCESS;

//     LSM303_DRIVER.MasterReceive (lsm303_addr, i2c_data, i2c_data_len, false);

//     /* Wait until transfer completed */
//     while (LSM303_DRIVER.GetStatus().busy);
//     /* Check if all data transferred */
//     if (LSM303_DRIVER.GetDataCount () != i2c_data_len) {
//         LOG_INFFMT("[Example partition] Master receive data count didn't match...\r\n");
//         status = PSA_ERROR_HARDWARE_FAILURE;
//     }

//     return status;
// }

// /**
//  * \brief Convert LSM303 data
//  */
// static inline double lsm303_data_to_double(const uint16_t data)
// {
//      int32_t tmp_data1;
//     int32_t tmp_data2;

//     tmp_data1 = data / 1100;
//      tmp_data2 = (1000000 * data / 1100) % 1000000;
//     return (double)tmp_data1 + (double)tmp_data2 / 1000000;
// }

// /**
//  * \brief Read from LSM303 peripheral
//  */
// static void tfm_example_read_lsm303(uint8_t hw_initialised)
// {
//     psa_status_t status;
//     psa_msg_t msg;

//     uint8_t i2c_reg_addr;
//     uint8_t i2c_reg_data[6] = {0};
//     // double lsm303_data[3] = {0};

//     /* Retrieve the message corresponding to the example service signal */
//     status = psa_get(TFM_EXAMPLE_READ_LSM303_SIGNAL, &msg);
//     if (status != PSA_SUCCESS) {
//         return;
//     }

//     /* Decode the message */
//     switch (msg.type) {
//     /* Any setup or teardown on IPC connect or disconnect goes here. If
//      * nothing then just reply with success.
//      */
//     case PSA_IPC_CONNECT:
//     case PSA_IPC_DISCONNECT:
//         /* This service does not require any setup or teardown on connect or
//          * disconnect, so just reply with success.
//          */
//         status = PSA_SUCCESS;
//         break;

//     case PSA_IPC_CALL:
//         // Check size of outvec parameter
//         if (msg.out_size[0] != sizeof(i2c_reg_data)) {
//             status = PSA_ERROR_PROGRAMMER_ERROR;
//             break;
//         }

//         /* Hardware init failed, return failure status to unblock the client */
//         if(!hw_initialised) {
//             psa_reply(msg.handle, PSA_ERROR_HARDWARE_FAILURE);
//         }

//         /* Read from LSM303 peripheral */
//         LOG_INFFMT("[Example partition] Start reading LSM303 peripheral...\r\n");

//         /* Check data ready */
//         i2c_reg_addr = 0x9;
//         i2c_send_data(&i2c_reg_addr, sizeof(i2c_reg_addr), true);

//         i2c_receive_data(&i2c_reg_data[0], 1, false);
//         if(!(i2c_reg_data[0] & 1)) {
//             LOG_INFFMT("[Example partition] Sensor data not available...\r\n");
//             status = PSA_ERROR_INSUFFICIENT_DATA;
//             break;
//         }

//         /* Read magnetic sensor data */
//         i2c_reg_addr = 0x3;
//         i2c_send_data(&i2c_reg_addr, sizeof(i2c_reg_addr), true);

//         i2c_receive_data(i2c_reg_data, sizeof(i2c_reg_data), false);

//         // lsm303_data[0] = lsm303_data_to_double((uint16_t)((i2c_reg_data[0] << 8) | i2c_reg_data[1]));
//         // lsm303_data[1] = lsm303_data_to_double((uint16_t)((i2c_reg_data[4] << 8) | i2c_reg_data[5]));
//         // lsm303_data[2] = lsm303_data_to_double((uint16_t)((i2c_reg_data[2] << 8) | i2c_reg_data[3]));

//         psa_write(msg.handle, 0, i2c_reg_data, sizeof(i2c_reg_data));
//         status = PSA_SUCCESS;
//         break;
//     default:
//         /* Invalid message type */
//         status = PSA_ERROR_PROGRAMMER_ERROR;
//         break;
//     }

//     /* Reply with the message result status to unblock the client */
//     psa_reply(msg.handle, status);
// }

/**
 * \brief Run inference using Tensorflow lite-micro
 */
void tfm_tflm_service_hello(void)
{
	psa_status_t status;
	psa_msg_t msg;
	float x_value, y_value;
	uint8_t inf_val_encoded_buf[256];
	size_t inf_val_encoded_buf_len = 0;
	cose_cbor_config_t cose_enc_cfg;

	/* Retrieve the message corresponding to the TFLM hello service signal */
	status = psa_get(TFM_TFLM_SERVICE_HELLO_SIGNAL, &msg);
	if (status != PSA_SUCCESS) {
		return;
	}

	/* Decode the message */
	switch (msg.type) {
	/* Any setup or teardown on IPC connect or disconnect goes here. If
	 * nothing then just reply with success.
	 */
	case PSA_IPC_CONNECT:
	case PSA_IPC_DISCONNECT:
		/* This service does not require any setup or teardown on connect or
		 * disconnect, so just reply with success.
		 */
		status = PSA_SUCCESS;
		break;

	case PSA_IPC_CALL:
		// Check size of invec/outvec parameter
		if (msg.in_size[0] != sizeof(psa_key_id_t) ||
		    msg.in_size[1] != sizeof(x_value) ||
		    msg.out_size[0] != sizeof(inf_val_encoded_buf)) {

			status = PSA_ERROR_PROGRAMMER_ERROR;
			break;
		}

		psa_read(msg.handle, 0, &cose_enc_cfg.key_id, sizeof(psa_key_id_t));
		psa_read(msg.handle, 1, &x_value, sizeof(x_value));
		cose_enc_cfg.max_buf_size = msg.out_size[0];
		if (cose_enc_cfg.key_id == C_SIGN) {
			cose_enc_cfg.cbor_encode_and_sign_pld = true;
		}
		/* This constant kXrange represents the range of x values our model
		 * was trained on, which is from 0 to (2 * Pi). We approximate Pi
		 * to avoid requiring additional libraries.
		 */
		if ((kXrange < x_value) || (x_value < 0.0f)) {
			status = PSA_ERROR_PROGRAMMER_ERROR;
			break;
		}

		/* Run inference */
		LOG_INFFMT("[TFLM service] Starting secure inferencing...\r\n");
		y_value = loop(x_value);

		LOG_INFFMT("[TFLM service] Starting CBOR encoding and COSE signing...\
				\r\n");
		psa_huk_key_derivation_cose_cbor_encode_and_sign(&y_value,
								 &cose_enc_cfg,
								 inf_val_encoded_buf,
								 &inf_val_encoded_buf_len);

		psa_write(msg.handle, 0, inf_val_encoded_buf, inf_val_encoded_buf_len);
		psa_write(msg.handle, 1, &inf_val_encoded_buf_len, sizeof(inf_val_encoded_buf_len));
		status = PSA_SUCCESS;
		break;
	default:
		/* Invalid message type */
		status = PSA_ERROR_PROGRAMMER_ERROR;
		break;
	}

	/* Reply with the message result status to unblock the client */
	psa_reply(msg.handle, status);
}

/**
 * \brief Create EC key needed by COSE.
 */
static void tfm_tflm_cose_create_ec_key(const uint8_t  *label,
					size_t label_len,
					psa_key_id_t ec_key_id,
					psa_key_usage_t key_usage_flag)
{
	psa_status_t status;

	status = psa_huk_key_derivation_ec_key(&ec_key_id,
					       label,
					       label_len,
					       &key_usage_flag);

	if (status != PSA_SUCCESS) {
		LOG_ERRFMT("[TFLM service] HUK key derivation failed with status %d\n" \
			   , status);
	} else {
		LOG_INFFMT("[TFLM service] Successfully derived the key from HUK for");
		LOG_INFFMT(" %s\n", label);
	}
}

/**
 * \brief The TFLM service partition's entry function.
 */
void tfm_tflm_service_req_mngr_init(void)
{
	psa_signal_t signals;

	// uint8_t i2c_reg_data[2] = {0};
	// uint8_t lsm303_init_completed = 1;

	// /* LSM303 DLHC */
	// LOG_INFFMT("[Example partition] Configuring I2C3 as secure peripheral...\r\n");
	// ppc_configure_to_secure(PPC_SP_APB_PPC_EXP1, CMSDK_I2C3_APB_PPC_POS);

	// LOG_INFFMT("[Example partition] Initialising I2C bus...\r\n");
	// LSM303_DRIVER.Initialize(NULL);
	// LSM303_DRIVER.PowerControl (ARM_POWER_FULL);
	// LSM303_DRIVER.Control      (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
	// LSM303_DRIVER.Control      (ARM_I2C_BUS_CLEAR, 0);

	// /* Set magnetometer output data rate */
	// i2c_reg_data[0] = 0x0;
	// i2c_reg_data[1] = 0x0;
	// if(i2c_send_data(i2c_reg_data, sizeof(i2c_reg_data), false) != PSA_SUCCESS) {
	//     lsm303_init_completed = 0;
	// }

	// /* Set magnetometer full scale range */
	// i2c_reg_data[0] = 0x1;
	// i2c_reg_data[1] = 1 << 5;
	// if(i2c_send_data(i2c_reg_data, sizeof(i2c_reg_data), false) != PSA_SUCCESS) {
	//     lsm303_init_completed = 0;
	// }

	// /* Continuous update */
	// i2c_reg_data[0] = 0x2;
	// i2c_reg_data[1] = 0x0;
	// if(i2c_send_data(i2c_reg_data, sizeof(i2c_reg_data), false) != PSA_SUCCESS) {
	//     lsm303_init_completed = 0;
	// }

	// LOG_INFFMT("[Example partition] Initialisation of I2C bus completed\r\n");

	/* Create 3 EC keys

	 | Key Name            | Added Label | Resulting Label              |
	 |---------------------|-------------|------------------------------|
	 | Device Client TLS   | CLIENT_TLS  | CLIENT_TLS_EC_PRIV_KEY_HI    |
	 | Device COSE SIGN    | C_SIGN      | C_SIGN_TLS_EC_PRIV_KEY_HI    |
	 | Device COSE ENCRYPT | C_ENCRYPT   | C_ENCRYPT_TLS_EC_PRIV_KEY_HI |
	 */
	tfm_tflm_cose_create_ec_key(label[0],
				    strlen((char *)label[0]),
				    CLIENT_TLS,
				    PSA_KEY_USAGE_VERIFY_MESSAGE);
	tfm_tflm_cose_create_ec_key(label[1],
				    strlen((char *)label[1]),
				    C_SIGN,
				    (PSA_KEY_USAGE_SIGN_HASH | PSA_KEY_USAGE_VERIFY_HASH));
	tfm_tflm_cose_create_ec_key(label[2],
				    strlen((char *)label[2]),
				    C_ENCRYPT,
				    PSA_KEY_USAGE_ENCRYPT);

	/* Tensorflow lite-micro initialisation */
	setup();

	LOG_INFFMT("[TFLM service] TFLM initalisation completed\r\n");

	/* Continually wait for one or more of the partition's RoT Service or
	 * interrupt signals to be asserted and then handle the asserted signal(s).
	 */
	while (1) {
		signals = psa_wait(PSA_WAIT_ANY, PSA_BLOCK);

		// if (signals & TFM_READ_LSM303_SIGNAL) {
		//     tfm_example_read_lsm303(lsm303_init_completed);
		// }

		if (signals & TFM_TFLM_SERVICE_HELLO_SIGNAL) {
			tfm_tflm_service_hello();
		}
	}
}
