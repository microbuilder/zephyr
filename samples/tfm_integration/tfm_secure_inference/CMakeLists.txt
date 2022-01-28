#
# Copyright (c) 2021-2022 Linaro Limited
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

get_target_property(TFM_BINARY_DIR tfm TFM_BINARY_DIR)
configure_file(
  ${CMAKE_CURRENT_LIST_DIR}/tfm_secure_inference_partitions/tfm_manifest_list.yaml.in
  ${CMAKE_CURRENT_BINARY_DIR}/tfm_secure_inference_partitions/tfm_manifest_list.yaml
)

set_property(TARGET zephyr_property_target
  APPEND PROPERTY TFM_CMAKE_OPTIONS
  -DTFM_EXTRA_MANIFEST_LIST_FILES=${CMAKE_CURRENT_BINARY_DIR}/tfm_secure_inference_partitions/tfm_manifest_list.yaml
  -DTFM_EXTRA_PARTITION_PATHS="${CMAKE_CURRENT_LIST_DIR}/tfm_secure_inference_partitions/tfm_huk_key_derivation_service\\\;${CMAKE_CURRENT_LIST_DIR}/tfm_secure_inference_partitions/tfm_tflm_service"
  )

# set_property(TARGET zephyr_property_target
#   APPEND_STRING PROPERTY TFM_PARTITION_PATHS
#   ${PARTITION_PATHS}
#   )

project(tfm_secure_provisioning)

target_sources(app PRIVATE
  src/main.c
  src/pk_import_verify_sign.c
  src/cose_verify.c
  src/tfm_secure_infer_partitions_service_api.c
  src/util_app_log.c
  src/util_sformat.c
  ext/NanoCBOR/src/decoder.c
  ext/NanoCBOR/src/encoder.c
)

target_include_directories(app PRIVATE
  ${ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR}/interface/include
  ext/NanoCBOR/include
)

target_compile_definitions(app PRIVATE
  TFM_PARTITION_HUK_KEY_DERIVATION
  TFM_PARTITION_TFLM_SERVICE
  NANOCBOR_BYTEORDER_HEADER=<sys/byteorder.h>
  NANOCBOR_BE64TOH_FUNC=sys_be64_to_cpu
  NANOCBOR_HTOBE64_FUNC=sys_cpu_to_be64
  NANOCBOR_HTOBE32_FUNC=sys_cpu_to_be32
)

# In TF-M, default value of CRYPTO_ENGINE_BUF_SIZE is 0x2080. It causes
# insufficient memory failure while verifying signature. Increase it to 0x2400.
set_property(TARGET zephyr_property_target
             APPEND PROPERTY TFM_CMAKE_OPTIONS
             -DCRYPTO_ENGINE_BUF_SIZE=0x2400
)

zephyr_include_directories(${APPLICATION_SOURCE_DIR}/src/tls_config)
