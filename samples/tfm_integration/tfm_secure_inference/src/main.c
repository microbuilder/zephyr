/*
 * Copyright (c) 2021-2022 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr.h>
#include <logging/log_ctrl.h>
#include <logging/log.h>

#include <tfm_veneers.h>
#include <tfm_ns_interface.h>
#include "tfm_tflm_service_api.h"
#include "cose_verify.h"
#include "util_app_log.h"
#include "pk_import_verify_sign.h"
#include <math.h>

// TODO: Replace with getting the public key from S-side
const uint8_t temp_ecdsaPublic[66] = {
	0x4, 0x18, 0x4d, 0xc2, 0x5c, 0xb, 0x32,
	0x2f, 0xfb, 0xff, 0xd, 0xdf, 0x9b, 0x55,
	0x87, 0x32, 0xf3, 0x53, 0xf8, 0x9a, 0xf1,
	0x1b, 0x1c, 0x89, 0x3a, 0x8f, 0xd5, 0xb1,
	0x4d, 0x9d, 0x5a, 0xed, 0x8e, 0x92, 0xea,
	0xda, 0x95, 0x24, 0xdf, 0xd4, 0xcc, 0xcc,
	0x4b, 0xe3, 0x3c, 0x1, 0xc8, 0x2c, 0xb3,
	0xbf, 0xb9, 0x21, 0x68, 0x71, 0x5a, 0x5b,
	0xbc, 0xc4, 0xa, 0x24, 0x9d, 0x74, 0xad,
	0xc, 0x68
};

// TODO: Replace with getting cose payload from S-side
uint8_t cose_encoded_payload[256] = {
	0xd2, 0x84, 0x43, 0xa1, 0x1, 0x26, 0xa0, 0x4b,
	0xa1, 0x3a, 0x0, 0x1, 0x38, 0x7f, 0x44, 0x1f,
	0x85, 0xab, 0x3f, 0x58, 0x40, 0xb7, 0x61, 0x7c,
	0x38, 0x29, 0x4b, 0xe, 0x78, 0xbf, 0x92, 0xb5,
	0x93, 0x74, 0x9c, 0x6c, 0x40, 0x72, 0x13, 0x71,
	0xb0, 0x6a, 0x8a, 0x2, 0x49, 0x4f, 0xa4, 0xad,
	0x7b, 0x15, 0x8, 0x10, 0x4a, 0x37, 0xc6, 0x26,
	0x17, 0x31, 0xee, 0xcf, 0x60, 0x89, 0xa7, 0xfc,
	0x46, 0x71, 0xfd, 0x6e, 0xe1, 0x63, 0xe5, 0x13,
	0x33, 0xcb, 0x57, 0x2f, 0x7e, 0x75, 0x75, 0x1a,
	0x25, 0xc1, 0xd2, 0x75, 0xd6
};

/** Declare a reference to the application logging interface. */
LOG_MODULE_DECLARE(app, CONFIG_LOG_DEFAULT_LEVEL);

int tflm_inference_value_decode_and_verify_sign(uint8_t *inf_val_encoded_buf,
						size_t inf_val_encoded_buf_len,
						const unsigned char *pkey,
						float *y_val)
{
	uint8_t *dec;
	size_t len_dec;
	cose_sign_context_t ctx;
	int status;

	status = mbedtls_ecp_load_representation(&ctx.pk,
						 pkey, strlen(pkey)
						 );
	if (status != 0) {
		LOG_ERR("Load the public key failed\n");
		goto err;
	}

	status = cose_sign_init(&ctx);
	if (status != COSE_ERROR_NONE) {
		LOG_ERR("Failed to initialize COSE signing context.\n");
	}
	goto err;

	status = cose_verify_sign1(&ctx,
				   inf_val_encoded_buf,
				   inf_val_encoded_buf_len,
				   (const uint8_t **) &dec,
				   &len_dec);
	if (status != COSE_ERROR_NONE) {
		LOG_ERR("Failed to authenticate signature.\n");
		goto err;
	}

	// Print the decoded payload
	for (int i = 0; i < len_dec; i++)
		LOG_INF("%x ", dec[i]);
	LOG_INF("\n");

	return status;
err:
	al_dump_log();
	cose_sign_free(&ctx);
	return status;
}

void main(void)
{
	psa_status_t status;

	const float PI = 3.14159265359f;
	float deg = PI / 180.0;

	float x_value, y_value;

	/* Initialise the logger subsys and dump the current buffer. */
	log_init();

	if (tflm_inference_value_decode_and_verify_sign(&cose_encoded_payload[0],
							sizeof(cose_encoded_payload),
							temp_ecdsaPublic,
							&y_value) != 0) {
		LOG_ERR("Failed to verify signature.\n");
	} else {
		LOG_INF("Verified the signature using the public key.\n");
	}

	for (int i = 0; i <= 20; i++) {

		x_value = (float)i * deg;
		status = al_psa_status(
			psa_secure_inference_tflm_hello(&x_value,
							sizeof(x_value),
							&y_value,
							sizeof(y_value)),
			__func__);

		if (status != PSA_SUCCESS) {
			LOG_ERR("Failed to get sine value using secure inference");
			goto err;
		}

		printf("Model: Sine of %d deg is: %f\t", i, y_value);
		printf("C Mathlib: Sine of %d deg is: %f\t", i, sin(x_value));
		printf("Deviation: %f\n", fabs(sin(x_value) - y_value));
		al_dump_log();

		k_msleep(500);
	}

err:
	/* Dump any queued log messages, and wait for system events. */
	al_dump_log();
}
