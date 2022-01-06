/*
 * Copyright (c) 2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "tfm_tflm_service_api.h"

#include "psa/client.h"
#include "psa_manifest/sid.h"

// psa_status_t example_read_lsm303(uint8_t *data,
// 				 size_t data_size)
// {
// 	psa_status_t status;
// 	psa_handle_t handle;

// 	psa_outvec out_vec[] = {
// 		{ .base = data, .len = data_size },
// 	};

// 	handle = psa_connect(TFM_EXAMPLE_READ_LSM303_SID, TFM_EXAMPLE_READ_LSM303_VERSION);
// 	if (!PSA_HANDLE_IS_VALID(handle)) {
// 		return PSA_HANDLE_TO_ERROR(handle);
// 	}

// 	status = psa_call(handle, PSA_IPC_CALL, NULL, 0, out_vec, 1);

// 	psa_close(handle);

// 	return status;
// }

psa_status_t psa_secure_inference_tflm_hello(const float *input,
					     size_t input_length,
					     float *sine_value_buf,
					     size_t sine_value_buf_len)
{
	psa_status_t status;
	psa_handle_t handle;
	psa_invec in_vec[] = {
		{ .base = input, .len = input_length },
	};

	psa_outvec out_vec[] = {
		{ .base = sine_value_buf, .len = sine_value_buf_len },
	};

	handle = psa_connect(TFM_TFLM_SERVICE_HELLO_SID, TFM_TFLM_SERVICE_HELLO_VERSION);
	if (!PSA_HANDLE_IS_VALID(handle)) {
		return PSA_HANDLE_TO_ERROR(handle);
	}

	status = psa_call(handle, PSA_IPC_CALL, in_vec, 1, out_vec, 1);

	psa_close(handle);

	return status;
}
