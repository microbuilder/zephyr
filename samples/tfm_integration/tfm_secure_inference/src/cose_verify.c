/*
 * Copyright (c) 2022 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "cose_verify.h"
#include "pk_import_verify_sign.h"

#define HASH_TSTR(md_ctx, nc, buf, len_buf, str)		    \
	nanocbor_encoder_init(&nc, buf, len_buf);		    \
	nanocbor_fmt_tstr(&nc, strlen(str));			    \
	mbedtls_md_update(&md_ctx, buf, nanocbor_encoded_len(&nc)); \
	mbedtls_md_update(&md_ctx, str, strlen(str));

#define HASH_BSTR(md_ctx, nc, buf, len_buf, bstr, len_bstr)	    \
	nanocbor_encoder_init(&nc, buf, len_buf);		    \
	nanocbor_fmt_bstr(&nc, len_bstr);			    \
	mbedtls_md_update(&md_ctx, buf, nanocbor_encoded_len(&nc)); \
	mbedtls_md_update(&md_ctx, bstr, len_bstr);

static int cose_encode_prot(nanocbor_encoder_t *nc)
{
	nanocbor_fmt_map(nc, 1);
	nanocbor_fmt_int(nc, cose_header_algorithm);
	nanocbor_fmt_int(nc, COSE_ALG_ECDSA_SHA256);
	return nanocbor_encoded_len(nc);
}

int cose_sign_init(cose_sign_context_t *ctx)
{
	mbedtls_ecp_group_id grp_id =
		mbedtls_pk_ec(ctx->pk)->MBEDTLS_PRIVATE(grp).id;

	if (grp_id == MBEDTLS_ECP_DP_SECP256R1) {
		ctx->len_hash = 32;
		ctx->len_sig = 72;
	} else {
		return COSE_ERROR_UNSUPPORTED;
	}

	return COSE_ERROR_NONE;
}

int cose_sign1_hash(cose_sign_context_t *ctx,
		    const uint8_t *pld,
		    const size_t len_pld,
		    uint8_t *hash)
{
	mbedtls_md_context_t md_ctx;

	mbedtls_md_setup(&md_ctx,
			 mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
			 0);
	mbedtls_md_starts(&md_ctx);
	nanocbor_encoder_t nc;

	/* serialize body_protected */
	nanocbor_encoder_init(&nc, NULL, 0);
	size_t len_prot = cose_encode_prot(&nc);
	uint8_t prot[len_prot];
	nanocbor_encoder_init(&nc, prot, len_prot);
	cose_encode_prot(&nc);

	/* serialize and hash ToBeSigned */
	size_t len_buf = 8;
	uint8_t buf[len_buf];

	nanocbor_encoder_init(&nc, buf, len_buf);
	nanocbor_fmt_array(&nc, 4);
	mbedtls_md_update(&md_ctx, buf, nanocbor_encoded_len(&nc));

	HASH_TSTR(md_ctx, nc, buf, len_buf, COSE_CONTEXT_SIGN1)
	HASH_BSTR(md_ctx, nc, buf, len_buf, prot, len_prot)
	/* external_aad. There is none so an empty bstr */
	HASH_BSTR(md_ctx, nc, buf, len_buf, NULL, 0)
	HASH_BSTR(md_ctx, nc, buf, len_buf, pld, len_pld)

	if (mbedtls_md_finish(&md_ctx, hash)) {
		return COSE_ERROR_HASH;
	}
	return COSE_ERROR_NONE;
}


int cose_sign1_decode(cose_sign_context_t *ctx,
		      const uint8_t *obj, const size_t len_obj,
		      const uint8_t **pld, size_t *len_pld,
		      const uint8_t **sig, size_t *len_sig,
		      uint8_t *hash)
{
	nanocbor_value_t nc, arr;

	nanocbor_decoder_init(&nc, obj, len_obj);
	nanocbor_skip(&nc);

	if (nanocbor_enter_array(&nc, &arr) < 0) {
		return COSE_ERROR_DECODE;
	}

	nanocbor_skip(&arr);
	nanocbor_skip(&arr);
	nanocbor_get_bstr(&arr, pld, len_pld);

	cose_sign1_hash(ctx, *pld, *len_pld, hash);

	nanocbor_get_bstr(&arr, sig, len_sig);
	return COSE_ERROR_NONE;
}

int cose_verify_sign1(cose_sign_context_t *ctx,
		      const uint8_t *obj,
		      const size_t len_obj,
		      const uint8_t **pld,
		      size_t *len_pld)
{
	uint8_t hash[ctx->len_hash];
	uint8_t *sig;
	size_t len_sig;

	if (cose_sign1_decode(ctx,
			      obj, len_obj,
			      pld, len_pld,
			      (const uint8_t **) &sig,
			      &len_sig, hash)) {
		return COSE_ERROR_DECODE;
	}

	if (mbedtls_ecdsa_pk_verify(
		    ctx->pk,
		    hash, ctx->len_hash, sig, len_sig) != 0) {
		return COSE_ERROR_AUTHENTICATE;
	}

	return COSE_ERROR_NONE;
}

void cose_sign_free(cose_sign_context_t *ctx)
{
	mbedtls_pk_free(&ctx->pk);
}
