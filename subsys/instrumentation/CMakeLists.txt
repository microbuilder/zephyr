# Copyright 2022 Linaro
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(
  CONFIG_INSTRUMENTATION
  common/instr_common.c
  )

if (CONFIG_INSTRUMENTATION)
  if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    zephyr_sources(handlers/instr_handlers_gcc.c)
  else()
    message(FATAL_ERROR "CONFIG_INSTRUMENTATION unsupported by this compiler.")
  endif()
endif()

zephyr_include_directories_ifdef(CONFIG_INSTRUMENTATION include)
