/*
 * Copyright (c) 2021 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>

#include "psa/client.h"
#include "psa_manifest/sid.h"
#include "psa/crypto.h"

/**
 * \brief Generate EC Key
 *
 * Generates an EC Key
 *
 * \param[in] key_id          Key slot id for persistent key
 *
 * \return A status indicating the success/failure of the operation
 */
psa_status_t psa_huk_key_derivation_ec_key(uint8_t *ec_priv_key_data,
					   size_t ec_priv_key_data_size);
