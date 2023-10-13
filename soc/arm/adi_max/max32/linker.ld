/* linker.ld - Linker command/script file */

/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>

SECTIONS
{
	SECTION_DATA_PROLOGUE(.flashprog,, SUBALIGN(4))
	{
		KEEP(*(.flashprog*)) /* Flash program */
	}
}
