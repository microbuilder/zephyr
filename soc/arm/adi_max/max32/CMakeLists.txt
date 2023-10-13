# Copyright (c) 2023 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_include_directories(common)
zephyr_sources(soc.c)
