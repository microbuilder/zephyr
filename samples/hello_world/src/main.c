/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/instrumentation/instrumentation.h>

extern void instr_dump_buffer(void);

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);
	instr_dump_buffer();
}
