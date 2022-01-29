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
#include "tfm_secure_infer_partitions_service_api.h"
#include "cose_verify.h"
#include "util_app_log.h"
#include "util_sformat.h"
#include "pk_import_verify_sign.h"
#include <math.h>

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
						 pkey,
						 strlen(pkey));
	if (status != 0) {
		LOG_ERR("Load the public key failed\n");
		goto err;
	}

	status = cose_sign_init(&ctx);
	if (status != COSE_ERROR_NONE) {
		LOG_ERR("Failed to initialize COSE signing context.\n");
		goto err;
	}

	status = cose_verify_sign1(&ctx,
				   inf_val_encoded_buf,
				   inf_val_encoded_buf_len,
				   (const uint8_t **) &dec,
				   &len_dec);
	if (status != COSE_ERROR_NONE) {
		LOG_ERR("Failed to authenticate signature.\n");
		goto err;
	}

	status = cose_payload_decode(dec, len_dec, y_val);
	if (status != COSE_ERROR_NONE) {
		LOG_ERR("Failed to authenticate signature.\n");
		goto err;
	}
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

	psa_key_id_t key_id = C_SIGN; // Using Device COSE SIGN key
	uint8_t inf_val_encoded_buf[256];
	size_t inf_val_encoded_buf_len = 0;
	uint8_t public_key[65] = { 0 }; /* EC public key = 65 bytes. */
	size_t public_key_len = sizeof(public_key);
	float x_value, y_value;

	/* Initialise the logger subsys and dump the current buffer. */
	log_init();

	/* String format output config. */
	struct sf_hex_tbl_fmt fmt = {
		.ascii = true,
		.addr_label = true,
		.addr = 0
	};

	status = al_psa_status(
		psa_huk_key_derivation_export_public_key(&key_id,
							 public_key,
							 public_key_len),
		__func__);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Failed to export the_public_key");
		goto err;
	}

	for (int i = 1; i <= 20; i++) {

		x_value = (float)i * deg;
		status = al_psa_status(
			psa_secure_inference_tflm_hello(&key_id,
							&x_value,
							sizeof(x_value),
							inf_val_encoded_buf,
							sizeof(inf_val_encoded_buf),
							&inf_val_encoded_buf_len),
			__func__);

		if (status != PSA_SUCCESS) {
			LOG_ERR("Failed to get sine value using secure inference");
			goto err;
		}

		LOG_INF("CBOR encoded and COSE signed inference value:");
		sf_hex_tabulate_16(&fmt, inf_val_encoded_buf, inf_val_encoded_buf_len);

		if (tflm_inference_value_decode_and_verify_sign(
			    &inf_val_encoded_buf[0],
			    inf_val_encoded_buf_len,
			    public_key,
			    &y_value) != 0) {
			LOG_ERR("NS: Failed to verify signature.\n");
		} else {
			LOG_INF("NS: Verified the signature using the public key.\n");
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
