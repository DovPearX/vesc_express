/*
	Copyright 2023, 2024 Benjamin Vedder benjamin@vedder.se
	Copyright 2022, 2024 Joel Svensson   svenssonjoel@yahoo.se
	Copyright 2023 Rasmus Söderhielm     rasmus.soderhielm@gmail.com

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "driver/gpio.h"

#include "comm_can.h"
#include "hw.h"
#include "lispbm.h"
#include "lispif_vesc_extensions.h"
#include "main.h"
#include "can_extensions.h"

static lbm_value ext_can_local_id(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_i(backup.config.controller_id);
}

static lbm_value ext_can_ping(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int id = lbm_dec_as_i32(args[0]);
	if (id < 0 || id > 253) {
		return ENC_SYM_TERROR;
	}

	HW_TYPE hw = HW_TYPE_VESC;
	bool res = comm_can_ping(id, &hw);

	return res ? lbm_enc_i(hw) : ENC_SYM_NIL;
}

static lbm_value start_can(unsigned bus, int tx, int rx, CAN_BAUD baud) {
	if (tx < 0 || tx >= GPIO_NUM_MAX || rx < 0 || rx >= GPIO_NUM_MAX ||
			!GPIO_IS_VALID_OUTPUT_GPIO(tx) || !GPIO_IS_VALID_GPIO(rx)) {
		lbm_set_error_reason("Invalid CAN pins");
		return ENC_SYM_EERROR;
	}
	return comm_can_start_bus(bus, tx, rx, baud) ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static lbm_value ext_can_start(lbm_value *args, lbm_uint argn) {
	if (argn > 2) {
		lbm_set_error_reason((char *)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}
	LBM_CHECK_NUMBER_ALL();

#if defined(CAN_TX_GPIO_NUM) && defined(CAN_RX_GPIO_NUM)
	int tx = CAN_TX_GPIO_NUM;
	int rx = CAN_RX_GPIO_NUM;
#else
	int tx = -1;
	int rx = -1;
#endif
	if (argn >= 1) {
		tx = lbm_dec_as_i32(args[0]);
	}
	if (argn >= 2) {
		rx = lbm_dec_as_i32(args[1]);
	}
	return start_can(0, tx, rx, backup.config.can_baud_rate);
}

static lbm_value ext_can_stop(lbm_value *args, lbm_uint argn) {
	(void)args;
	LBM_CHECK_ARGN(0);
	comm_can_stop();
	return ENC_SYM_TRUE;
}

static lbm_value can_send(lbm_value *args, lbm_uint argn, unsigned bus, bool extended) {
	if (argn != 2 || !lbm_is_number(args[0])) {
		return ENC_SYM_EERROR;
	}

	uint32_t id = lbm_dec_as_u32(args[0]);
	uint8_t bytes[8];
	const uint8_t *data = bytes;
	lbm_uint len = 0;
	lbm_array_header_t *array = lbm_dec_array_r(args[1]);
	if (array) {
		data = (const uint8_t *)array->data;
		len = array->size;
	} else {
		lbm_value list = args[1];
		while (lbm_is_cons(list) && len < sizeof(bytes)) {
			lbm_value byte = lbm_car(list);
			if (!lbm_is_number(byte)) {
				return ENC_SYM_TERROR;
			}
			bytes[len++] = lbm_dec_as_u32(byte);
			list = lbm_cdr(list);
		}
		if (!lbm_is_symbol_nil(list)) {
			return ENC_SYM_EERROR;
		}
	}
	if (len > sizeof(bytes) || id > (extended ? 0x1fffffffU : 0x7ffU)) {
		return ENC_SYM_EERROR;
	}
	return comm_can_transmit_bus(bus, id, data, len, extended) ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_can_send_sid(lbm_value *args, lbm_uint argn) {
	return can_send(args, argn, 0, false);
}

static lbm_value ext_can_send_eid(lbm_value *args, lbm_uint argn) {
	return can_send(args, argn, 0, true);
}

static lbm_value ext_can_use_vesc(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);
	if (!lbm_is_symbol_true(args[0]) && !lbm_is_symbol_nil(args[0])) {
		return ENC_SYM_TERROR;
	}
	comm_can_use_vesc_decoder(lbm_is_symbol_true(args[0]));
	return ENC_SYM_TRUE;
}
#if SOC_TWAI_CONTROLLER_NUM > 1
static lbm_value ext_can2_start(lbm_value *args, lbm_uint argn) {
	if (argn < 2 || argn > 3) {
		lbm_set_error_reason((char *)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}
	LBM_CHECK_NUMBER_ALL();

	int tx = lbm_dec_as_i32(args[0]);
	int rx = lbm_dec_as_i32(args[1]);
	int kbits = argn == 3 ? lbm_dec_as_i32(args[2]) : 500;
	return start_can(1, tx, rx, comm_can_kbits_to_baud(kbits));
}

static lbm_value ext_can2_stop(lbm_value *args, lbm_uint argn) {
	(void)args;
	LBM_CHECK_ARGN(0);
	comm_can2_stop();
	return ENC_SYM_TRUE;
}

static lbm_value ext_can2_send_sid(lbm_value *args, lbm_uint argn) {
	return can_send(args, argn, 1, false);
}

static lbm_value ext_can2_send_eid(lbm_value *args, lbm_uint argn) {
	return can_send(args, argn, 1, true);
}

static lbm_value ext_can2_use_vesc(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);
	if (!lbm_is_symbol_true(args[0]) && !lbm_is_symbol_nil(args[0])) {
		return ENC_SYM_TERROR;
	}
	comm_can2_use_vesc_decoder(lbm_is_symbol_true(args[0]));
	return ENC_SYM_TRUE;
}
#endif
#if SOC_TWAI_CONTROLLER_NUM > 2
static lbm_value ext_can3_start(lbm_value *args, lbm_uint argn) {
	if (argn < 2 || argn > 3) {
		lbm_set_error_reason((char *)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}
	LBM_CHECK_NUMBER_ALL();

	int tx = lbm_dec_as_i32(args[0]);
	int rx = lbm_dec_as_i32(args[1]);
	int kbits = argn == 3 ? lbm_dec_as_i32(args[2]) : 500;
	return start_can(2, tx, rx, comm_can_kbits_to_baud(kbits));
}

static lbm_value ext_can3_stop(lbm_value *args, lbm_uint argn) {
	(void)args;
	LBM_CHECK_ARGN(0);
	comm_can3_stop();
	return ENC_SYM_TRUE;
}

static lbm_value ext_can3_send_sid(lbm_value *args, lbm_uint argn) {
	return can_send(args, argn, 2, false);
}

static lbm_value ext_can3_send_eid(lbm_value *args, lbm_uint argn) {
	return can_send(args, argn, 2, true);
}

static lbm_value ext_can3_use_vesc(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);
	if (!lbm_is_symbol_true(args[0]) && !lbm_is_symbol_nil(args[0])) {
		return ENC_SYM_TERROR;
	}
	comm_can3_use_vesc_decoder(lbm_is_symbol_true(args[0]));
	return ENC_SYM_TRUE;
}
#endif

static lbm_uint sym_event_can_sid;
static lbm_uint sym_event_can_eid;
#if SOC_TWAI_CONTROLLER_NUM > 1
static lbm_uint sym_event_can2_sid;
static lbm_uint sym_event_can2_eid;
#endif
#if SOC_TWAI_CONTROLLER_NUM > 2
static lbm_uint sym_event_can3_sid;
static lbm_uint sym_event_can3_eid;
#endif

static lisp_event_t can_events[SOC_TWAI_CONTROLLER_NUM][2] = {
	{{.symbol = &sym_event_can_sid}, {.symbol = &sym_event_can_eid}},
#if SOC_TWAI_CONTROLLER_NUM > 1
	{{.symbol = &sym_event_can2_sid}, {.symbol = &sym_event_can2_eid}},
#endif
#if SOC_TWAI_CONTROLLER_NUM > 2
	{{.symbol = &sym_event_can3_sid}, {.symbol = &sym_event_can3_eid}},
#endif
};

bool lispif_can_enable_event(lbm_uint symbol, bool enabled) {
	for (unsigned bus = 0; bus < SOC_TWAI_CONTROLLER_NUM; bus++) {
		for (unsigned extended = 0; extended < 2; extended++) {
			lisp_event_t *event = &can_events[bus][extended];
			if (symbol == *event->symbol) {
				event->enabled = enabled;
				return true;
			}
		}
	}
	return false;
}

static lbm_value ext_can_recv_sid(lbm_value *args, lbm_uint argn) {
	return lispif_event_wait(&can_events[0][0], args, argn);
}

static lbm_value ext_can_recv_eid(lbm_value *args, lbm_uint argn) {
	return lispif_event_wait(&can_events[0][1], args, argn);
}
#if SOC_TWAI_CONTROLLER_NUM > 1
static lbm_value ext_can2_recv_sid(lbm_value *args, lbm_uint argn) {
	return lispif_event_wait(&can_events[1][0], args, argn);
}

static lbm_value ext_can2_recv_eid(lbm_value *args, lbm_uint argn) {
	return lispif_event_wait(&can_events[1][1], args, argn);
}
#endif
#if SOC_TWAI_CONTROLLER_NUM > 2
static lbm_value ext_can3_recv_sid(lbm_value *args, lbm_uint argn) {
	return lispif_event_wait(&can_events[2][0], args, argn);
}

static lbm_value ext_can3_recv_eid(lbm_value *args, lbm_uint argn) {
	return lispif_event_wait(&can_events[2][1], args, argn);
}
#endif

void lispif_can_context_done(lbm_cid cid) {
	for (unsigned bus = 0; bus < SOC_TWAI_CONTROLLER_NUM; bus++) {
		lispif_event_forget_waiter(&can_events[bus][0], cid);
		lispif_event_forget_waiter(&can_events[bus][1], cid);
	}
}

bool lispif_can_event_enabled(unsigned bus, bool extended) {
	lisp_event_t *event = &can_events[bus][extended];
	return event->enabled || event->waiter >= 0;
}

typedef struct {
	uint32_t id;
	uint8_t *data;
	uint8_t len;
} lisp_can_frame_t;

static bool pack_can_frame(lbm_flat_value_t *flat, bool waiting, const void *payload) {
	const lisp_can_frame_t *frame = payload;
	if (!f_cons(flat) || !f_i32(flat, frame->id)) {
		return false;
	}
	if (waiting) {
		return f_cons(flat) && f_lbm_array(flat, frame->len, frame->data) && f_sym(flat, SYM_NIL);
	}
	return f_lbm_array(flat, frame->len, frame->data);
}

void lispif_can_event(unsigned bus, uint32_t id, uint8_t *data, uint8_t len, bool extended) {
	lisp_can_frame_t frame = {.id = id, .data = data, .len = len};
	lispif_event_post(&can_events[bus][extended], len, pack_can_frame, &frame);
}

void lispif_load_can_extensions(void) {
	lbm_add_symbol_const("event-can-sid", &sym_event_can_sid);
	lbm_add_symbol_const("event-can-eid", &sym_event_can_eid);
#if SOC_TWAI_CONTROLLER_NUM > 1
	lbm_add_symbol_const("event-can2-sid", &sym_event_can2_sid);
	lbm_add_symbol_const("event-can2-eid", &sym_event_can2_eid);
#endif
#if SOC_TWAI_CONTROLLER_NUM > 2
	lbm_add_symbol_const("event-can3-sid", &sym_event_can3_sid);
	lbm_add_symbol_const("event-can3-eid", &sym_event_can3_eid);
#endif

	for (unsigned bus = 0; bus < SOC_TWAI_CONTROLLER_NUM; bus++) {
		lispif_event_reset(&can_events[bus][0]);
		lispif_event_reset(&can_events[bus][1]);
	}

	lbm_add_extension("can-local-id", ext_can_local_id);
	lbm_add_extension("can-ping", ext_can_ping);
	lbm_add_extension("can-start", ext_can_start);
	lbm_add_extension("can-stop", ext_can_stop);
	lbm_add_extension("can-use-vesc", ext_can_use_vesc);
	lbm_add_extension("can-send-sid", ext_can_send_sid);
	lbm_add_extension("can-send-eid", ext_can_send_eid);
	lbm_add_extension("can-recv-sid", ext_can_recv_sid);
	lbm_add_extension("can-recv-eid", ext_can_recv_eid);
#if SOC_TWAI_CONTROLLER_NUM > 1
	lbm_add_extension("can2-start", ext_can2_start);
	lbm_add_extension("can2-stop", ext_can2_stop);
	lbm_add_extension("can2-use-vesc", ext_can2_use_vesc);
	lbm_add_extension("can2-send-sid", ext_can2_send_sid);
	lbm_add_extension("can2-send-eid", ext_can2_send_eid);
	lbm_add_extension("can2-recv-sid", ext_can2_recv_sid);
	lbm_add_extension("can2-recv-eid", ext_can2_recv_eid);
#endif
#if SOC_TWAI_CONTROLLER_NUM > 2
	lbm_add_extension("can3-start", ext_can3_start);
	lbm_add_extension("can3-stop", ext_can3_stop);
	lbm_add_extension("can3-use-vesc", ext_can3_use_vesc);
	lbm_add_extension("can3-send-sid", ext_can3_send_sid);
	lbm_add_extension("can3-send-eid", ext_can3_send_eid);
	lbm_add_extension("can3-recv-sid", ext_can3_recv_sid);
	lbm_add_extension("can3-recv-eid", ext_can3_recv_eid);
#endif
}
