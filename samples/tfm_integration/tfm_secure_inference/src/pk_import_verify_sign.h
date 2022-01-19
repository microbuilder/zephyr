/*
 * Copyright (c) 2022 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PK_IMPORT_VERIFY_SIGN_H
#define PK_IMPORT_VERIFY_SIGN_H

#include <stdio.h>
#include "mbedtls/pk.h"
#include "mbedtls/error.h"
#include <zephyr.h>

/**
 * @brief              Load the contents of a public key buffer
 *                     into an internal ECP representation.
 *
 * @note               This function does the pk setup, group load
 *                     (mbedtls_ecp_group_load()) ecp point read binary.
 *
 * @param ctx          Pointer to the uninitialized public-key context
 * @param data         The buffer from which to load the representation.
 * @param data_length  The size in bytes of \p data.
 *
 * @return Returns error code as specified in \ref MbedTLS error code.
 */
int mbedtls_ecp_load_representation(mbedtls_pk_context *ctx,
				    const uint8_t *data,
				    size_t data_length);

/**
 * \brief           Verify signature in non-ASN container format signed
 *                  payload.
 *
 * \param ctx       The PK context to use. It must have been set up.
 * \param hash      Hash of the message to sign
 * \param hash_len  Hash length
 * \param sig       Signature to verify
 * \param sig_len   Signature length
 *
 * \return          0 on success (signature is valid),
 *                  #MBEDTLS_ERR_PK_SIG_LEN_MISMATCH if there is a valid
 *                  signature in sig but its length is less than \p siglen,
 *                  or a specific error code.
 */
int mbedtls_ecdsa_pk_verify(mbedtls_pk_context ctx,
			    const unsigned char *hash,
			    size_t hash_len,
			    const unsigned char *sig,
			    size_t sig_len);
#endif /* PK_IMPORT_VERIFY_SIGN_H */
