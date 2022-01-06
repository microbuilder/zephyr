/*
 * Copyright (c) 2021 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <psa/crypto.h>
#include <stdbool.h>
#include <stdint.h>
#include "tfm_secure_api.h"
#include "tfm_api.h"

#include "tfm_sp_log.h"
#include "tfm_crypto_defs.h"
#include "psa/crypto.h"
#include "psa/service.h"
#include "psa_manifest/tfm_huk_key_derivation_service.h"

#define KEY_LEN_BYTES  16

typedef psa_status_t (*signal_handler_t)(psa_msg_t *);

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
	status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_LABEL,
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
	uint8_t label_hi[] = "EC_PRIV_KEY_HI";
	uint8_t label_lo[] = "EC_PRIV_KEY_LO";

	if (msg->out_size[0] != sizeof(ec_priv_key_data)) {
		/* The size of the argument is incorrect */
		return PSA_ERROR_PROGRAMMER_ERROR;
	}

	/* For MPS2 AN521 platform, TF-M always returns a 16-byte sample key
	 * as the HUK derived key. But the size of EC private key is 32-bytes.
	 * Therefore, we decided to call HUK based key derivation twice.
	*/
	status = tfm_huk_key_derivation(ec_priv_key_data,
					KEY_LEN_BYTES,
					&ec_priv_key_data_len,
					label_hi,
					sizeof(label_hi));

	if (status != PSA_SUCCESS) {
		return status;
	}

	status = tfm_huk_key_derivation(&ec_priv_key_data[ec_priv_key_data_len],
					KEY_LEN_BYTES,
					&ec_priv_key_data_len,
					label_lo,
					sizeof(label_lo));

	if (status != PSA_SUCCESS) {
		return status;
	}

	psa_write(msg->handle, 0, ec_priv_key_data, sizeof(ec_priv_key_data));

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
			tfm_huk_key_derivation_signal_handle(TFM_HUK_KEY_DERIVATION_EC_KEY_SIGNAL,
							     tfm_huk_key_derivation_ec_key);
		} else {
			psa_panic();
		}
	}

	return PSA_SUCCESS;
}
