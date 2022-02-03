/*
 * Copyright (c) 2021-2022 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>

#include "psa/client.h"
#include "psa_manifest/sid.h"
#include "psa/crypto.h"
#include <stdbool.h>

typedef struct {
	psa_key_id_t key_id;
	_Bool cbor_encode_and_sign_pld;
	size_t max_buf_size;
} cose_cbor_config_t;

/**
 * \brief Generate EC Key
 *
 * Generates an EC Key
 *
 * \param[in] ec_key_id         EC key id for persistent key
 * \param[in] label             Unique label string for the key derivation seed value
 * \param[in] label_size        Unique label string size
 * \param[in] key_usage_flag    Pointer to key usage flag
 *
 * \return A status indicating the success/failure of the operation
 */
psa_status_t psa_huk_key_derivation_ec_key(psa_key_id_t *ec_key_id,
					   const uint8_t *label,
					   size_t label_size,
					   psa_key_usage_t *key_usage_flag);
/**
 * \brief COSE CBOR encode and sign
 *
 * COSE CBOR encode and sign
 *
 * \param[in]  inf_value        Tflm inference value to encode and sign
 * \param[in]  cfg              Pointer to COSE CBOR config
 * \param[out] encoded_buf      Buffer to which encoded data
 *                              is written into
 * \param[out] encoded_buf_len  Encoded buffer len in bytes
 *
 * \return A status indicating the success/failure of the operation
 */
psa_status_t psa_huk_key_derivation_cose_cbor_encode_and_sign(float *inf_value,
							      cose_cbor_config_t *cfg,
							      uint8_t *encoded_buf,
							      size_t *encoded_buf_len);
