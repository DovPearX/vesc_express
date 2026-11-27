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

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lispbm.h"
#include "lispif_vesc_extensions.h"
#include "modules/can_extensions.h"
#include "modules/eeprom_extensions.h"
#include "modules/system_extensions.h"
#include "extensions/array_extensions.h"
#include "extensions/lbm_dyn_lib.h"
#include "extensions/math_extensions.h"
#include "extensions/mutex_extensions.h"
#include "extensions/string_extensions.h"

static portMUX_TYPE waiter_lock = portMUX_INITIALIZER_UNLOCKED;
static lbm_uint sym_event_data_rx;
static lisp_event_t data_event = {.symbol = &sym_event_data_rx};

void lispif_event_reset(lisp_event_t *event) {
	event->enabled = false;
	event->waiter = -1;
}

static lbm_value ext_event_enable(lbm_value *args, lbm_uint argn) {
	if ((argn != 1 && argn != 2) || !lbm_is_symbol(args[0]) ||
			(argn == 2 && !lbm_is_number(args[1]))) {
		return ENC_SYM_EERROR;
	}

	lbm_uint symbol = lbm_dec_sym(args[0]);
	bool enabled = argn == 1 || lbm_dec_as_i32(args[1]) != 0;
	if (symbol == *data_event.symbol) {
		data_event.enabled = enabled;
		return ENC_SYM_TRUE;
	}

	return lispif_can_enable_event(symbol, enabled) ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

lbm_value lispif_event_wait(lisp_event_t *event, lbm_value *args, lbm_uint argn) {
	if (argn > 1 || (argn == 1 && !lbm_is_number(args[0]))) {
		return ENC_SYM_EERROR;
	}
	float timeout = argn == 1 ? lbm_dec_as_float(args[0]) : 0.0f;
	if (!isfinite(timeout) || timeout >= UINT32_MAX / 1000000U) {
		return ENC_SYM_EERROR;
	}
	TickType_t now = xTaskGetTickCount();
	lbm_cid cid = lbm_get_current_cid();
	portENTER_CRITICAL(&waiter_lock);
	event->started = now;
	event->timeout = timeout > 0.0f ?
			(TickType_t)(timeout * configTICK_RATE_HZ + 1.0f) : 0;
	event->waiter = cid;
	portEXIT_CRITICAL(&waiter_lock);
	if (timeout > 0.0f) {
		lbm_block_ctx_from_extension_timeout(timeout);
	} else {
		lbm_block_ctx_from_extension();
	}
	return ENC_SYM_TRUE;
}

static lbm_value ext_recv_data(lbm_value *args, lbm_uint argn) {
	return lispif_event_wait(&data_event, args, argn);
}

static void load_event_extensions(void) {
	lbm_add_symbol_const("event-data-rx", &sym_event_data_rx);
	lispif_event_reset(&data_event);
	lbm_add_extension("event-enable", ext_event_enable);
	lbm_add_extension("recv-data", ext_recv_data);
}

void lispif_event_forget_waiter(lisp_event_t *event, lbm_cid cid) {
	atomic_compare_exchange_strong(&event->waiter, &cid, -1);
}

void lispif_events_context_done(lbm_cid cid) {
	lispif_can_context_done(cid);
	lispif_event_forget_waiter(&data_event, cid);
}

void lispif_event_post(lisp_event_t *event, size_t len, lispif_event_pack_t pack, const void *payload) {
	TickType_t now = xTaskGetTickCount();
	portENTER_CRITICAL(&waiter_lock);
	lbm_cid waiter = event->waiter;
	if (waiter >= 0 && event->timeout && now - event->started >= event->timeout) {
		lispif_event_forget_waiter(event, waiter);
		waiter = -1;
	}
	portEXIT_CRITICAL(&waiter_lock);
	if (waiter < 0 && (!event->enabled || !lbm_event_handler_exists())) {
		return;
	}

	lbm_flat_value_t flat;
	if (!lbm_start_flatten(&flat, len + 32)) {
		return;
	}

	bool ok = waiter >= 0 || (f_cons(&flat) && f_sym(&flat, *event->symbol));
	ok = ok && pack(&flat, waiter >= 0, payload);

	if (ok && lbm_finish_flatten(&flat)) {
		if (waiter >= 0 && lbm_unblock_ctx(waiter, &flat)) {
			lispif_event_forget_waiter(event, waiter);
			return;
		}
		if (waiter < 0 && lbm_event(&flat)) {
			return;
		}
	}
	lbm_free(flat.buf);
}

typedef struct {
	uint8_t *data;
	unsigned int len;
} app_data_t;

static bool pack_app_data(lbm_flat_value_t *flat, bool waiting, const void *payload) {
	(void)waiting;
	const app_data_t *message = payload;
	return f_lbm_array(flat, message->len, message->data);
}

void lispif_process_custom_app_data(uint8_t *data, unsigned int len) {
	app_data_t message = {.data = data, .len = len};
	lispif_event_post(&data_event, len, pack_app_data, &message);
}

static const char *const dyn_functions[] = {
	/* These helpers require the uart-read extension.
	"(defun uart-read-bytes (buffer n ofs)"
	"(let ((rd (uart-read buffer n ofs)))"
	"(if (= rd n)"
	"(bufset-u8 buffer (+ ofs rd) 0)"
	"(progn (yield 4000) (uart-read-bytes buffer (- n rd) (+ ofs rd)))"
	")))",

	"(defun uart-read-until (buffer n ofs end)"
	"(let ((rd (uart-read buffer n ofs end)))"
	"(if (and (> rd 0) (or (= rd n) (= (bufget-u8 buffer (+ ofs (- rd 1))) end)))"
	"(bufset-u8 buffer (+ ofs rd) 0)"
	"(progn (yield 10000) (uart-read-until buffer (- n rd) (+ ofs rd) end))"
	")))",
	*/
	"(defun img-buffer-from-bin (x) x)",
};

static bool dynamic_loader(const char *name, const char **code) {
	size_t len = strlen(name);
	for (size_t i = 0; i < sizeof(dyn_functions) / sizeof(dyn_functions[0]); i++) {
		const char *definition = dyn_functions[i] + sizeof("(defun ") - 1;
		if (strncmp(definition, name, len) == 0 && definition[len] == ' ') {
			*code = dyn_functions[i];
			return true;
		}
	}
	return lbm_dyn_lib_find(name, code);
}

void lispif_load_vesc_extensions(void) {
	load_event_extensions();
	lbm_array_extensions_init();
	lbm_math_extensions_init();
	lbm_mutex_extensions_init();
	lbm_string_extensions_init();
	lbm_dyn_lib_init();
	lbm_set_dynamic_load_callback(dynamic_loader);

	lispif_load_system_extensions();
	lispif_load_eeprom_extensions();
	lispif_load_can_extensions();
}
