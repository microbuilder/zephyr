/*
 * Copyright 2022 Linaro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/instrumentation/instrumentation.h>

__no_instrumentation__
void __cyg_profile_func_enter(void *func, void *caller)
{
	/* TODO: Implement profiling handler, currently only tracing supported. */
	instr_event_handler(INSTR_OP_ENTRY, func, caller);
}

__no_instrumentation__
void __cyg_profile_func_exit(void *func, void *caller)
{
	/* TODO: Implement profiling handler, currently only tracing supported. */
	instr_event_handler(INSTR_OP_EXIT, func, caller);
}
