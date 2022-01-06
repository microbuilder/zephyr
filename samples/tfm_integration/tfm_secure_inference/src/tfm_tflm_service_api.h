/*
 * Copyright (c) 2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __TFM_EXAMPLE_PARTITION_API_H__
#define __TFM_EXAMPLE_PARTITION_API_H__

#include <stdint.h>
#include <stddef.h>

#include "psa/error.h"

#ifdef __cplusplus
extern "C" {
#endif

// /**
//  * \brief Read magnetometer (LSM303) data.
//  *
//  * \param[out]  data            Buffer to which magnetometer data is
//  *                              written into
//  * \param[out]   data_size      Size of magnetometer data in bytes
//  *
//  * \return Returns error code as specified in \ref psa_status_t
//  */
// psa_status_t example_read_lsm303(uint8_t *data,
//                                 size_t data_size);

/**
 * \brief Run secure inference to get the sine value of input
 *
 * \param[in]   input               Angle in degrees
 * \param[in]   input_length        Length of input in bytes
 * \param[out]  sine_value_buf      Buffer to which calculated sine value
 *                                  is written into
 * \param[in]   sine_value_buf_len  Size of sine_value_buf in bytes
 *
 * \return Returns error code as specified in \ref psa_status_t
 */
psa_status_t psa_secure_inference_tflm_hello(const float *input,
					     size_t input_length,
					     float *sine_value_buf,
					     size_t sine_value_buf_len);

#ifdef __cplusplus
}
#endif

#endif /* __TFM_EXAMPLE_PARTITION_API_H__ */
