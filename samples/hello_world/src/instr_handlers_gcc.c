#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>

#define _NO_INSTRUMENT_FUNC_ __attribute__((__no_instrument_function__))

/**
 * @brief Magic word placed at the beginning of an instrumentation buffer.
 */
#define INSTR_MAGIC_START (0x1575ECAB)

/**
 * @brief Magic word placed at the end of the an instrumentation buffer.
 */
#define INSTR_MAGIC_STOP (0xBACE5751)

/**
 * @brief Version indicator for the payloads.
 */
#define INSTR_VERSION (0x0302)

/* TODO: Move to kconfig. */
#define INSTR_BUFFER_SZ (8192)

/**
 * @brief Memory buffer to store instrumentation event records.
 */
static uint8_t _instr_buffer[INSTR_BUFFER_SZ] = {0};

static uint8_t _instr_initialised = 0;
static uint8_t _instr_disabled = 0;
static int _instr_buffer_pos = 0;

/**
 * @brief Instrumentation record op codes.
 */
enum instr_op_codes {
	INSTR_OP_META,	/**< Metadata record, defined in TLV format. */
	INSTR_OP_ENTRY, /**< Func. entry event record, followed by instr_event. */
	INSTR_OP_EXIT,	/**< Func. exit event record, followed by instr_event. */
	INSTR_OP_RESET	/**< Reset event marker. */
};

/**
 * @brief Record types for INSTR_OP_META TLV records.
 */
enum instr_meta_datatypes {
	INSTR_META_INSTR_OVER,	 /**< Estimated tick overhead for instr. handlers. */
	INSTR_META_COEF_US2TICK, /**< Ticks per us coefficient. */
	INSTR_META_THREAD,	 /**< Details for a single thread context. */
	INSTR_META_ENDIANNESS,	 /**< Indiciates big or little endian. */
	INSTR_META_LAST = 64,	 /**< Limit meta record types to 6 bits (0..63). */
};

/**
 * @brief Packet header for individual records.
 */
struct instr_header {
	union {
		/** Generic opcode-only header, to determine packet type for further processing */
		struct {
			/** Op code. */
			uint8_t op : 2;
			/** Reserved, only relevant for known packet types. */
			uint8_t rsvd : 6;
		} opcode;
		/** Metadata packet header. */
		struct {
			/** OP code (0 for METADATA). */
			uint8_t op : 2;
			/** Data type (values TBD). */
			uint8_t type : 6;
		} metadata;
		/** Event packet header. */
		struct {
			/** Op code (1 = ENTRY, 2 = EXIT). */
			uint8_t op : 2;
			/** instr_event_ctxt data present = 1 */
			uint8_t ctxt : 1;
			/** 64-bit addresses for func and caller values. */
			uint8_t addr64 : 1;
			/**< Caller address present = 1 */
			uint8_t caller : 1;
			/**< Timestamp delta size in n+1 bytes */
			uint8_t delta_sz : 3;
		} event;
		/** Raw packet header value. */
		uint8_t raw;
	};
};

struct instr_event_ctxt {
	union {
		/* Execution context information about the event that follows. */
		struct {
// #if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
// 			/** Thread ID (correlate values with thread lookup table). */
// 			uint8_t thread_id;
// 			/** Arch-specific mode indicator (thread mode, interrupt mode, etc.). */
// 			uint8_t mode : 3;
// 			/** CPU number. */
// 			uint8_t cpu : 3;
// 			/** Rserved for future use. */
// 			uint8_t rsvd : 2;
// #else /* LITTLE and PDP */
			/** Arch-specific mode indicator (thread mode, interrupt mode, etc.). */
			uint8_t mode : 3;
			/** CPU number. */
			uint8_t cpu : 3;
			/** Rserved for future use. */
			uint8_t rsvd : 2;
			/** Thread ID (correlate values with thread lookup table). */
			uint8_t thread_id;
// #endif
		} fields;
		/** Event context value. */
		uint16_t raw;
	};
};

/**
 * @brief Instrumentation event payload.
 *
 * Currently hard-coded to 32-bit addresses, context enabled, 32-bit delta.
 */
struct instr_event {
	union {
		struct {
			struct instr_event_ctxt ctxt;
			uint32_t address;
			uint32_t caller;
			uint32_t delta;
		} __packed;
		uint8_t raw[14];
	};
};

struct instr_event_stats {
	uint64_t event_entry;
	uint64_t event_exit;
};

static struct instr_event_stats _instr_event_stats = {
	.event_entry = 0,
	.event_exit = 0,
};

_NO_INSTRUMENT_FUNC_
void instr_init(void)
{
	if (_instr_disabled) {
		return;
	}

	/* ToDo: Publish metadata, config settings, etc. */

	/* Initialisation complete. */
	_instr_initialised = 1;
}

_NO_INSTRUMENT_FUNC_
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
			.ctxt.raw = *((uint16_t*)(_instr_buffer + (i + 1))),
			.address= *((uint32_t*)(_instr_buffer + (i + 3))),
			.caller = *((uint32_t*)(_instr_buffer + (i + 7))),
			.delta = *((uint32_t*)(_instr_buffer + (i + 11))),
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
	/* Test with:
	   $ rm -rf build && west build -p auto -b mps2_an521 \
	     zephyr/samples/hello_world  -t run -- \
		 -DCMAKE_C_FLAGS="-finstrument-functions \
		 -finstrument-functions-exclude-function-list=arch" \
		 -DCONFIG_MAIN_THREAD_PRIORITY=2 -DCONFIG_THREAD_NAME=y
	 */
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

_NO_INSTRUMENT_FUNC_
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
		.event.addr64 = 0,  /* 32-bit addresses. */
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
	if (_instr_buffer_pos <
	    (INSTR_BUFFER_SZ - (sizeof(struct instr_event) + sizeof(struct instr_header)))) {
		_instr_buffer[_instr_buffer_pos++] = hdr.raw;
		for (int i = 0; i < sizeof(struct instr_event); i++) {
			_instr_buffer[_instr_buffer_pos++] = event.raw[i];
		}
	} else {
		/* Buffer full ... disable further writes. */
		_instr_disabled = 1;
	}
}

_NO_INSTRUMENT_FUNC_
void __cyg_profile_func_enter(void *func, void *caller)
{
	/* TODO: Implement profiling handler, currently only tracing supported. */
	instr_event_handler(INSTR_OP_ENTRY, func, caller);
}

_NO_INSTRUMENT_FUNC_
void __cyg_profile_func_exit(void *func, void *caller)
{
	/* TODO: Implement profiling handler, currently only tracing supported. */
	instr_event_handler(INSTR_OP_EXIT, func, caller);
}
