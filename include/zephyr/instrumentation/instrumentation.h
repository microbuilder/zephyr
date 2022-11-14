/*
 * Copyright (c) 2022 Linaro
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_INSTRUMENTATION_INSTRUMENTATION_H_
#define ZEPHYR_INCLUDE_INSTRUMENTATION_INSTRUMENTATION_H_

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(__no_instrumentation__)
#error "No toolchain support for __no_instrumentation__"
#endif

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
			// 			/** Thread ID (correlate values with thread lookup table).
			// */ 			uint8_t thread_id;
			// 			/** Arch-specific mode indicator (thread mode, interrupt
			// mode, etc.). */ 			uint8_t mode : 3;
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

/**
 * @brief Performs any initialisation required by the system.
 */
void instr_init(void);

/**
 * @brief Dumps the buffered contents via printk.
 */
void instr_dump_buffer(void);

/**
 * @brief Shared callabck handler to process entry/exit events.
 *
 * @param opcode The type of event to process.
 * @param func   Address of the function being called.
 * @param caller Address of the function caller.
 */
void instr_event_handler(enum instr_op_codes opcode, void *func, void *caller);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_INSTRUMENTATION_INSTRUMENTATION_H_ */
