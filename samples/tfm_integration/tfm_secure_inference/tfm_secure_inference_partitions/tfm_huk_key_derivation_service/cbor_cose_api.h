/*
 * Copyright (c) 2021-2022 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CBOR_COSE_API_H__
#define __CBOR_COSE_API_H__

#include <stdint.h>
#include "psa/service.h"

#ifdef __cplusplus
extern "C" {
#endif

psa_status_t tflm_inference_value_encode_and_sign(psa_key_handle_t key_handle,
						  float inv_val,
						  uint8_t *inf_val_encoded_buf,
						  size_t inf_val_encoded_buf_size,
						  size_t *inf_val_encoded_buf_len);

#ifdef __cplusplus
}
#endif

#endif /* __CBOR_COSE_API_H__ */
