/*
 * Copyright 2022 Linaro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/instrumentation/instrumentation.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>

/* Memory buffer to store instrumentation event records.

   TODO: This should be configurable depending on the instrumentation
         method selected:
		 - Callgraph ring buffer (default): Replace oldest entry when buffer
		   full, at the cost of processing stray entry events that have been
		   truncated.
		 - Callgraph fixed buffer: Stop buffering events when the buffer is
		   full, ensuring we have a callgraph from reset or from whenever the
		   trigger event was entered.
		 - Statistical: Buffer functions until out of memory
 */
static uint8_t _instr_buffer[CONFIG_INSTRUMENTATION_BUFFER_SIZE] = { 0 };

static uint8_t _instr_initialised = 0;
static uint8_t _instr_disabled = 0;
static int _instr_buffer_pos = 0;

/* TODO: Replace with Zephyr statistics API. */
struct instr_event_stats {
	uint64_t event_entry;
	uint64_t event_exit;
};

static struct instr_event_stats _instr_event_stats = {
	.event_entry = 0,
	.event_exit = 0,
};

__no_instrumentation__
void instr_init(void)
{
	if (_instr_disabled) {
		return;
	}

	/* ToDo: Publish metadata, config settings, etc. */

	/* Initialisation complete. */
	_instr_initialised = 1;
}

__no_instrumentation__
void instr_dump_buffer(void)
{
	/* Disable instrumentation just in case buffer isn't full yet. */
	_instr_disabled = 1;

	printk("\n");
	printk("DIR C M THR FUNCTION   CALLER     DELTA\n");
	printk("--- - - --- ---------- ---------- -----");

	/* Dump event buffer in 14-byte chunks (TODO: Remove hard coding!). */
	for (int i = 0; i < _instr_buffer_pos;
	     i += sizeof(struct instr_event) + sizeof(struct instr_header)) {
		struct instr_header hdr = {
			.raw = _instr_buffer[i],
		};

		/* Make sure we're reading an ENTRY/EXIT event record. */
		if ((hdr.opcode.op != INSTR_OP_ENTRY) && (hdr.opcode.op != INSTR_OP_EXIT)) {
			continue;
		}

		/* Reconstruct event record. */
		/* TODO: Assumes LE, make portable! */
		/* TODO: Handle 64-bit addresses, and dynamic delta sizes. */
		struct instr_event event = {
			.ctxt.raw = *((uint16_t *)(_instr_buffer + (i + 1))),
			.address = *((uint32_t *)(_instr_buffer + (i + 3))),
			.caller = *((uint32_t *)(_instr_buffer + (i + 7))),
			.delta = *((uint32_t *)(_instr_buffer + (i + 11))),
		};

		/* Render direction, address, ticks */
		switch (hdr.event.op) {
		case INSTR_OP_ENTRY:
		case INSTR_OP_EXIT:
			/* Render direction. */
			printk("\n%s", hdr.event.op == INSTR_OP_ENTRY ? "--> " : "<-- ");

			/* Render context data if present. */
			if (hdr.event.ctxt) {
				printk("%d %d %03d ", event.ctxt.fields.cpu, event.ctxt.fields.mode,
				       event.ctxt.fields.thread_id);
			} else {
				printk("- - -- ");
			}

			/* Render func address. */
			printk("0x%08X ", event.address);

			/* Render caller address if present. */
			if (hdr.event.caller) {
				printk("0x%08X ", event.caller);
			} else {
				printk("---------- ");
			}

			/* Render delta. */
			printk("%d", event.delta);

			break;
		default:
			printk("unhandled packet\n");
			break;
		}
	}

	/* Display current thread priority. */
	/* TODO: Figure out how to access current cpu, etc., without func. calls. */
	/* TODO: Figure out how to generate an 8-bit ID per thread. */
	printk("\n\n");
	printk("current cpu: %d\n", _kernel.cpus[0].id);
	printk("thread prio: %d\n", _kernel.cpus[0].current->base.prio);
	/* Thread state:
	   _THREAD_DUMMY (BIT(0))		// Not a real thread
	   _THREAD_PENDING (BIT(1))		// Thread is waiting on an object
	   _THREAD_PRESTART (BIT(2))	// Thread has not yet started
	   _THREAD_DEAD (BIT(3))		// Thread has terminated
	   _THREAD_SUSPENDED (BIT(4))	// Thread is suspended
	   _THREAD_ABORTING (BIT(5))	// Thread is being aborted
	   _THREAD_QUEUED (BIT(7))		// Thread is present in the ready queue
	*/
	printk("thread state: %d\n", _kernel.cpus[0].current->base.thread_state);
	printk("thread order key: %d\n", _kernel.cpus[0].current->base.order_key);
#if defined(CONFIG_THREAD_NAME)
	printk("thread name: %s\n", _kernel.cpus[0].current->name);
#endif

	/* Dump stats. */
	printk("\n");
	printk("entry: %lld\n", _instr_event_stats.event_entry);
	printk("exit:  %lld\n", _instr_event_stats.event_exit);
}

__no_instrumentation__
void instr_event_handler(enum instr_op_codes opcode, void *func, void *caller)
{
	if (_instr_disabled) {
		return;
	}

	/* Make sure we've published config settings. */
	if (!_instr_initialised) {
		instr_init();
		if ((_instr_disabled) || (!_instr_initialised)) {
			return;
		}
	}

	/* Currently only handles ENTRY and EXIT op codes. */
	if ((opcode != INSTR_OP_ENTRY) && (opcode != INSTR_OP_EXIT)) {
		return;
	}

	/* Increment stats for debug purposes. */
	if (opcode == INSTR_OP_ENTRY) {
		_instr_event_stats.event_entry++;
	}
	if (opcode == INSTR_OP_EXIT) {
		_instr_event_stats.event_exit++;
	}

	/* Hard-code sensible values for now. */
	struct instr_header hdr = {
		.event.op = opcode,
		.event.ctxt = 1,     /* Context data provided. */
		.event.addr64 = 0,   /* 32-bit addresses. */
		.event.caller = 1,   /* Caller address provided. */
		.event.delta_sz = 3, /* 32-bits timestamps (n+1 bytes). */
	};

	struct instr_event event = {
		.ctxt.raw = 0,
		.address = ((uint32_t)func) - 1,  /* Subtract one for Thumb-2, make portable! */
		.caller = ((uint32_t)caller) - 1, /* Subtract one for Thumb-2, make portable! */
		.delta = 1000,
	};

	/* Set the CPU number if relevant. */
	/* NOTE: Requires the following exclusions when building:
	   "-finstrument-functions-exclude-function-list=arch" */
	IF_ENABLED(CONFIG_SMP, (event.ctxt.fields.cpu = arch_curr_cpu()->id));

	/* Set the thread ID. */
	/* TODO: Convert thread to lookup table when we have a unique ID here. */
	/* See kernel/thread.h for struct. */
	/* NOTE: Requires the following exclusions when building:
	   "-finstrument-functions-exclude-function-list=arch" */
	/* Call below doesn't resolve to the expected value, but printk works?!? */
	event.ctxt.fields.thread_id = arch_curr_cpu()->current->base.prio;

	/* Set mode bit(s). Meaning here is arch-specific. */
	event.ctxt.fields.mode = ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) ? 1 : 0);

	/* Publish the entry event. */
	if (_instr_buffer_pos < (CONFIG_INSTRUMENTATION_BUFFER_SIZE -
				 (sizeof(struct instr_event) + sizeof(struct instr_header)))) {
		_instr_buffer[_instr_buffer_pos++] = hdr.raw;
		for (int i = 0; i < sizeof(struct instr_event); i++) {
			_instr_buffer[_instr_buffer_pos++] = event.raw[i];
		}
	} else {
		/* Buffer full ... disable further writes. */
		_instr_disabled = 1;
	}
}
