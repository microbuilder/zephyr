/*
 * Copyright (c) 2021-2022 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tfm_huk_key_derivation_service_api.h"

psa_status_t psa_huk_key_derivation_ec_key(psa_key_id_t *ec_key_id,
					   const uint8_t *label,
					   size_t label_size)
{
	psa_status_t status;
	psa_handle_t handle;

	psa_invec in_vec[] = {
		{ .base = label, .len = label_size },
		{ .base = ec_key_id, .len = sizeof(psa_key_id_t) },
	};

	handle = psa_connect(TFM_HUK_KEY_DERIVATION_EC_KEY_SID,
			     TFM_HUK_KEY_DERIVATION_EC_KEY_VERSION);
	if (!PSA_HANDLE_IS_VALID(handle)) {
		return PSA_ERROR_GENERIC_ERROR;
	}

	status = psa_call(handle,
			  PSA_IPC_CALL,
			  in_vec,
			  IOVEC_LEN(in_vec),
			  NULL,
			  0);

	psa_close(handle);

	return status;
}

psa_status_t psa_huk_key_derivation_cose_cbor_encode_and_sign(float *inf_value,
							      cose_cbor_config_t *cfg,
							      uint8_t *encoded_buf,
							      size_t *encoded_buf_len)
{
	psa_status_t status;
	psa_handle_t handle;

	psa_invec in_vec[] = {
		{ .base = inf_value, .len = sizeof(float) },
		{ .base = cfg, .len = sizeof(cose_cbor_config_t) },
	};

	psa_outvec out_vec[] = {
		{ .base = encoded_buf, .len = cfg->max_buf_size },
		{ .base = encoded_buf_len, .len = sizeof(size_t) },
	};

	handle = psa_connect(TFM_HUK_KEY_DERIVATION_COSE_CBOR_ENC_SIGN_SID,
			     TFM_HUK_KEY_DERIVATION_COSE_CBOR_ENC_SIGN_VERSION);
	if (!PSA_HANDLE_IS_VALID(handle)) {
		return PSA_ERROR_GENERIC_ERROR;
	}

	status = psa_call(handle,
			  PSA_IPC_CALL,
			  in_vec,
			  IOVEC_LEN(in_vec),
			  out_vec,
			  IOVEC_LEN(out_vec));

	psa_close(handle);

	return status;
}
