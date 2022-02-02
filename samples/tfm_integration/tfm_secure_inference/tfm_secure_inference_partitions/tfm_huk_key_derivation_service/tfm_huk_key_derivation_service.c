/*
 * Copyright (c) 2021-2022 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <psa/crypto.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "tfm_secure_api.h"
#include "tfm_api.h"

#include "cbor_cose_api.h"
#include "tfm_sp_log.h"
#include "tfm_crypto_defs.h"
#include "psa/crypto.h"
#include "psa/service.h"
#include "psa_manifest/tfm_huk_key_derivation_service.h"
#include "tfm_huk_key_derivation_service_api.h"

#define KEY_LEN_BYTES  16
#define LABEL_HI "_EC_PRIV_KEY_HI"
#define LABEL_LO "_EC_PRIV_KEY_LO"

typedef psa_status_t (*signal_handler_t)(psa_msg_t *);

const char hex_digits[] = { '0', '1', '2', '3', '4', '5', '6', '7',
			    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

#define UUID_STR_LEN ((KEY_LEN_BYTES * 2) + 4 + 1)
#define UUID_7TH_BYTE_MASK  64U         /* 0b0100_0000*/
#define UUID_9TH_BYTE_MASK  128U        /* 0b1000_0000*/

static psa_status_t tfm_encode_random_bytes_to_uuid(uint8_t *random_bytes,
						    size_t random_bytes_len,
						    uint8_t *uuid_buf,
						    size_t uuid_buf_len)
{
	int j = 0;
	int hyphen_index = 8;

	if (random_bytes_len != KEY_LEN_BYTES) {
		return PSA_ERROR_INSUFFICIENT_DATA;
	}

	if (uuid_buf_len != UUID_STR_LEN) {
		return PSA_ERROR_BUFFER_TOO_SMALL;
	}

	for (int i = 0; i < random_bytes_len; i++) {
		if (i == 6) {
			random_bytes[i] |= UUID_7TH_BYTE_MASK;
		}
		if (i == 8) {
			random_bytes[i] |= UUID_9TH_BYTE_MASK;
		}

		if (random_bytes[i] <= 0x0f) {
			uuid_buf[j++] = '0';
		}

		do {
			uuid_buf[j++] = hex_digits[random_bytes[i] & 0x0f];
			random_bytes[i] >>= 4;
		} while (random_bytes[i]);

		if (j == hyphen_index) {
			uuid_buf[j++] = '-';
			if (hyphen_index == 23) {
				hyphen_index = 0;
			} else {
				hyphen_index += 5;
			}
		}
	}
	uuid_buf[j] = '\0';
}

static psa_status_t tfm_huk_key_derivation(uint8_t *key_data,
					   size_t key_data_size,
					   size_t *key_data_len,
					   uint8_t *label,
					   size_t label_size)
{
	psa_status_t status;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_derivation_operation_t op = PSA_KEY_DERIVATION_OPERATION_INIT;
	psa_key_id_t derived_key_id;

	if (key_data_size < KEY_LEN_BYTES) {
		return PSA_ERROR_BUFFER_TOO_SMALL;
	}

	if (label == NULL || label_size == 0) {
		return PSA_ERROR_INVALID_ARGUMENT;
	}

	/* Currently, MbedTLS does not support key derivation for Elliptic curves.
	 * There is a PR https://github.com/ARMmbed/mbedtls/pull/5139 in progress
	 * though. Once this PR is merged, TF-M updates MbedTLS and finally, once
	 * Zephyr updates to latest TF-M, then we can use derive key/s for Elliptic
	 * curve instead of using symmetric keys as starting point for Elliptic
	 * curve key derivation.
	 */

	/* Set the key attributes for the key */
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT |
				PSA_KEY_USAGE_DECRYPT |
				PSA_KEY_USAGE_EXPORT);

	/* Set the algorithm, key type and the number of bits of the key. This is
	 * mandatory for key derivation. Setting these attributes will ensure that
	 * derived key is in accordance with the standard, if any.
	 */
	psa_set_key_algorithm(&attributes, PSA_ALG_GCM);
	psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attributes, PSA_BYTES_TO_BITS(KEY_LEN_BYTES));

	/* Set up a key derivation operation with HUK derivation as the alg */
	status = psa_key_derivation_setup(&op, TFM_CRYPTO_ALG_HUK_DERIVATION);
	if (status != PSA_SUCCESS) {
		return status;
	}

	/* Supply the UUID label as an input to the key derivation */
	status = psa_key_derivation_input_bytes(&op,
						PSA_KEY_DERIVATION_INPUT_LABEL,
						label,
						label_size);
	if (status != PSA_SUCCESS) {
		goto err_release_op;
	}

	/* Create the storage key from the key derivation operation */
	status = psa_key_derivation_output_key(&attributes, &op, &derived_key_id);
	if (status != PSA_SUCCESS) {
		goto err_release_op;
	}

	// status = psa_key_derivation_output_bytes(&op, uuid, sizeof(uuid));

	status =  psa_export_key(derived_key_id, key_data, key_data_size, key_data_len);

	if (status != PSA_SUCCESS) {
		goto err_release_op;
	}

	/* Free resources associated with the key derivation operation */
	status = psa_key_derivation_abort(&op);
	if (status != PSA_SUCCESS) {
		return status;
	}

	status = psa_destroy_key(derived_key_id);
	if (status != PSA_SUCCESS) {
		LOG_INFFMT("psa_destroy_key returned: %d \n", status);
		return status;
	}

	return PSA_SUCCESS;

err_release_op:
	(void)psa_key_derivation_abort(&op);

	return status;
}

static psa_status_t tfm_huk_key_derivation_ec_key(psa_msg_t *msg)
{
	psa_status_t status = PSA_SUCCESS;
	uint8_t ec_priv_key_data[KEY_LEN_BYTES * 2] = { 0 };
	size_t ec_priv_key_data_len = 0;

	// Check size of invec parameters
	if (msg->in_size[0] == 0 ||
	    msg->in_size[1] != sizeof(psa_key_id_t)) {
		return PSA_ERROR_PROGRAMMER_ERROR;
	}

	uint8_t label_hi[32] = { 0 };
	uint8_t label_lo[32] = { 0 };
	uint8_t rx_label[32] = { 0 };
	psa_key_id_t key_id;

	psa_read(msg->handle, 0, &rx_label, msg->in_size[0]);
	psa_read(msg->handle, 1, &key_id, msg->in_size[1]);

	// Add _EC_PRIV_KEY_HI to rx_label to create unique label_hi
	sprintf((char *)label_hi, "%s%s", rx_label, LABEL_HI);

	// Add _EC_PRIV_KEY_LO to rx_label to create unique label_lo
	sprintf((char *)label_lo, "%s%s", rx_label, LABEL_LO);

	/* For MPS2 AN521 platform, TF-M always returns a 16-byte sample key
	 * as the HUK derived key. But the size of EC private key is 32-bytes.
	 * Therefore, we decided to call HUK based key derivation twice.
	 */
	status = tfm_huk_key_derivation(ec_priv_key_data,
					KEY_LEN_BYTES,
					&ec_priv_key_data_len,
					label_hi,
					strlen((char *)label_hi));

	if (status != PSA_SUCCESS) {
		return status;
	}

	status = tfm_huk_key_derivation(&ec_priv_key_data[ec_priv_key_data_len],
					KEY_LEN_BYTES,
					&ec_priv_key_data_len,
					label_lo,
					strlen((char *)label_lo));

	if (status != PSA_SUCCESS) {
		return status;
	}

	psa_key_attributes_t key_attributes = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_type_t key_type =
		PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1);
	psa_algorithm_t alg = PSA_ALG_ECDSA(PSA_ALG_SHA_256);
	psa_key_handle_t tflm_cose_key_handle = 0;
	/* Setup the key's attributes before the creation request. */
	psa_set_key_id(&key_attributes, key_id);
	psa_set_key_usage_flags(&key_attributes,
				PSA_KEY_USAGE_SIGN_HASH | PSA_KEY_USAGE_VERIFY_HASH);
	psa_set_key_lifetime(&key_attributes, PSA_KEY_LIFETIME_PERSISTENT);
	psa_set_key_algorithm(&key_attributes, alg);
	psa_set_key_type(&key_attributes, key_type);

	status = psa_import_key(&key_attributes,
				ec_priv_key_data,
				sizeof(ec_priv_key_data),
				&tflm_cose_key_handle);
	if (status != PSA_SUCCESS) {
		LOG_ERRFMT("psa_import_key returned: %d \n", status);
	}
	status = psa_close_key(tflm_cose_key_handle);
	if (status != PSA_SUCCESS) {
		LOG_ERRFMT("psa_close_key returned: %d \n", status);
	}

	return status;
}


static psa_status_t tfm_huk_key_derivation_export_public_key(psa_msg_t *msg)
{
	psa_status_t status = PSA_SUCCESS;
	psa_key_id_t key_id = 0;
	psa_key_handle_t key_handle;
	uint8_t data_out[65] = { 0 };         /* EC public key = 65 bytes. */
	size_t data_len;

	psa_read(msg->handle, 0, &key_id, msg->in_size[0]);
	status = psa_open_key(key_id, &key_handle);
	if (status != PSA_SUCCESS) {
		LOG_ERRFMT("psa_open_key returned: %d \n", status);
	}

	status = psa_export_public_key(key_handle,
				       data_out,
				       sizeof(data_out),
				       &data_len);
	if (status != PSA_SUCCESS) {
		LOG_ERRFMT("psa_export_public_key returned: %d \n", status);
	}

	status = psa_close_key(key_handle);
	if (status != PSA_SUCCESS) {
		LOG_ERRFMT("psa_close_key returned: %d \n", status);
	}
	psa_write(msg->handle, 0, data_out, data_len);
	return status;
}

static psa_status_t tfm_huk_key_derivation_cose_cbor_enc_and_sign
	(psa_msg_t *msg)
{
	psa_status_t status = PSA_SUCCESS;
	cose_cbor_config_t cose_cbor_cfg;

	psa_read(msg->handle, 1, &cose_cbor_cfg, msg->in_size[1]);

	if (cose_cbor_cfg.cbor_encode_and_sign_pld) {
		uint8_t inf_val_encoded_buf[cose_cbor_cfg.max_buf_size];
		size_t inf_val_encoded_buf_len = 0;
		float inf_value = 0;
		psa_read(msg->handle, 0, &inf_value, msg->in_size[0]);
		status = tflm_inference_value_encode_and_sign(cose_cbor_cfg.key_id,
							      inf_value,
							      inf_val_encoded_buf,
							      cose_cbor_cfg.max_buf_size,
							      &inf_val_encoded_buf_len);
		if (status != PSA_SUCCESS) {
			LOG_ERRFMT("tflm_inference_value_encode_and_sign returned: %d \n",
				   status);
			return status;
		}
		psa_write(msg->handle,
			  0,
			  inf_val_encoded_buf,
			  inf_val_encoded_buf_len);
		psa_write(msg->handle,
			  1,
			  &inf_val_encoded_buf_len,
			  sizeof(inf_val_encoded_buf_len));
	} else {
		LOG_ERRFMT("Normal sign payload without CBOR encode is not supported");
		return PSA_ERROR_PROGRAMMER_ERROR;
	}
	return status;
}

static psa_status_t tfm_huk_key_derivation_gen_uuid(psa_msg_t *msg)
{
	psa_status_t status = PSA_SUCCESS;
	size_t uuid_length;
	uint8_t uuid_encoded[37] = {0};
	uint8_t uuid[16] = {0};
	uint8_t *uuid_label = (uint8_t *)"UUID";

	status = tfm_huk_key_derivation(uuid,
					sizeof(uuid),
					&uuid_length,
					uuid_label,
					strlen((char *)uuid_label));

	if (status != PSA_SUCCESS) {
		return status;
	}
	tfm_encode_random_bytes_to_uuid(uuid,
					sizeof(uuid),
					uuid_encoded,
					sizeof(uuid_encoded));
	psa_write(msg->handle, 0, uuid_encoded, sizeof(uuid_encoded));
	return status;
}

static void tfm_huk_key_derivation_signal_handle(psa_signal_t signal, signal_handler_t pfn)
{
	psa_status_t status;
	psa_msg_t msg;

	status = psa_get(signal, &msg);
	switch (msg.type) {
	case PSA_IPC_CONNECT:
		psa_reply(msg.handle, PSA_SUCCESS);
		break;
	case PSA_IPC_CALL:
		status = pfn(&msg);
		psa_reply(msg.handle, status);
		break;
	case PSA_IPC_DISCONNECT:
		psa_reply(msg.handle, PSA_SUCCESS);
		break;
	default:
		psa_panic();
	}
}

psa_status_t tfm_huk_key_derivation_req_mngr_init(void)
{
	psa_signal_t signals = 0;

	while (1) {
		signals = psa_wait(PSA_WAIT_ANY, PSA_BLOCK);
		if (signals & TFM_HUK_KEY_DERIVATION_EC_KEY_SIGNAL) {
			tfm_huk_key_derivation_signal_handle(
				TFM_HUK_KEY_DERIVATION_EC_KEY_SIGNAL,
				tfm_huk_key_derivation_ec_key);
		} else if (signals & TFM_HUK_KEY_DERIVATION_EXPORT_PUBLIC_KEY_SIGNAL) {
			tfm_huk_key_derivation_signal_handle(
				TFM_HUK_KEY_DERIVATION_EXPORT_PUBLIC_KEY_SIGNAL,
				tfm_huk_key_derivation_export_public_key);
		} else if (signals &
			   TFM_HUK_KEY_DERIVATION_COSE_CBOR_ENC_SIGN_SIGNAL) {
			tfm_huk_key_derivation_signal_handle(
				TFM_HUK_KEY_DERIVATION_COSE_CBOR_ENC_SIGN_SIGNAL,
				tfm_huk_key_derivation_cose_cbor_enc_and_sign);
		} else if (signals & TFM_HUK_KEY_DERIVATION_GENERATE_UUID_SIGNAL) {
			tfm_huk_key_derivation_signal_handle(
				TFM_HUK_KEY_DERIVATION_GENERATE_UUID_SIGNAL,
				tfm_huk_key_derivation_gen_uuid);
		} else {
			psa_panic();
		}
	}

	return PSA_SUCCESS;
}
