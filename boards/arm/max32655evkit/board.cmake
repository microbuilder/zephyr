# Copyright (c) 2023 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_MAX32655EVKIT)
board_runner_args(openocd --cmd-pre-init "source [find interface/cmsis-dap.cfg]")
board_runner_args(openocd --cmd-pre-init "source [find target/max32655.cfg]")
board_runner_args(jlink "--device=MAX32655" "--reset-after-load")
endif()

# Make OpenOCD the default runner
set_ifndef(BOARD_DEBUG_RUNNER openocd)
set_ifndef(BOARD_FLASH_RUNNER openocd)

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
