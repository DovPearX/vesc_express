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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "buffer.h"
#include "comm_can.h"
#include "comm_uart.h"
#include "comm_usb.h"
#include "commands.h"
#include "crc.h"
#include "flash_helper.h"
#include "lispbm.h"
#include "packet.h"
#include "system_extensions.h"

static lbm_value ext_print(lbm_value *args, lbm_uint argn) {
	const int str_len = 256;
	char *print_val_buffer = lbm_malloc_reserve(str_len + 1);
	if (!print_val_buffer) {
		return ENC_SYM_MERROR;
	}
	print_val_buffer[str_len] = '\0';

	for (lbm_uint i = 0; i < argn; i ++) {
		lbm_print_value(print_val_buffer, str_len, args[i]);
		commands_printf_lisp("%s", print_val_buffer);
	}

	lbm_free(print_val_buffer);

	return ENC_SYM_TRUE;
}

static lbm_value ext_puts(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);

	if (!lbm_is_array_r(args[0])) {
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = lbm_dec_array_r(args[0]);
	commands_printf_lisp("%.*s", (int)array->size, (const char *)array->data);

	return ENC_SYM_TRUE;
}

static lbm_value ext_systime(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_u32(xTaskGetTickCount());
}

static lbm_value ext_secs_since(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	return lbm_enc_float((float)(xTaskGetTickCount() - lbm_dec_as_u32(args[0])) / configTICK_RATE_HZ);
}

static lbm_value ext_crc16(lbm_value *args, lbm_uint argn) {
	if ((argn != 1 && argn != 2) || !lbm_is_array_r(args[0])) {
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
	unsigned int len = array->size;
	if (argn == 2) {
		if (!lbm_is_number(args[1])) {
			return ENC_SYM_TERROR;
		}

		len = lbm_dec_as_u32(args[1]);
		if (len > array->size) {
			len = array->size;
		}
	}

	return lbm_enc_i(crc16((uint8_t*)array->data, len));
}

static lbm_value ext_crc32(lbm_value *args, lbm_uint argn) {
	if ((argn != 2 && argn != 3) || !lbm_is_array_r(args[0]) || !lbm_is_number(args[1])) {
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
	unsigned int len = array->size;
	if (argn == 3) {
		if (!lbm_is_number(args[2])) {
			return ENC_SYM_TERROR;
		}

		len = lbm_dec_as_u32(args[2]);
		if (len > array->size) {
			len = array->size;
		}
	}

	return lbm_enc_u32(crc32_with_init((uint8_t*)array->data, len, lbm_dec_as_u32(args[1])));
}

// VESC Tool appends a table of named imports after the stored source.
static lbm_value ext_import(lbm_value *args, lbm_uint argn) {
	if (argn != 2 || !lbm_is_array_r(args[0]) || !lbm_is_symbol(args[1])) {
		return ENC_SYM_TERROR;
	}

	const char *symbol = lbm_get_name_by_symbol(lbm_dec_sym(args[1]));
	const uint8_t *code = flash_helper_code_data_ptr(CODE_IND_LISP);
	if (!code || !symbol) {
		return ENC_SYM_EERROR;
	}
	int32_t size = flash_helper_code_size(CODE_IND_LISP);
	int32_t index = strnlen((const char *)code, size) + 1;
	if (size - index < 2) {
		lbm_set_error_reason("No bundled imports found");
		return ENC_SYM_EERROR;
	}

	uint16_t count = buffer_get_uint16(code, &index);
	for (uint16_t i = 0; i < count && index < size; i++) {
		const char *name = (const char *)code + index;
		size_t name_len = strnlen(name, size - index);
		if (name_len >= size - index || size - index - name_len - 1 < 8) {
			break;
		}
		index += name_len + 1;
		uint32_t offset = buffer_get_uint32(code, &index);
		uint32_t len = buffer_get_uint32(code, &index);
		if (strcmp(name, symbol) != 0) {
			continue;
		}
		if (offset > size || len > size - offset) {
			lbm_set_error_reason("Bundled import has an invalid offset or length");
			return ENC_SYM_EERROR;
		}
		lbm_value value;
		if (!lbm_share_array_const(&value, (char *)code + offset, len)) {
			return ENC_SYM_MERROR;
		}
		lbm_uint index = lbm_dec_sym(args[1]) & GLOBAL_ENV_MASK;
		lbm_value *env = lbm_get_global_env();
		lbm_value entry = lbm_env_set(env[index], args[1], value);
		if (lbm_is_symbol(entry)) {
			return entry == ENC_SYM_MERROR ? entry : ENC_SYM_EERROR;
		}
		env[index] = entry;
		return ENC_SYM_TRUE;
	}

	lbm_set_error_reason("Symbol not found among bundled imports");
	return ENC_SYM_EERROR;
}

static lbm_value ext_send_data(lbm_value *args, lbm_uint argn) {
	if (argn < 1 || argn > 3) {
		lbm_set_error_reason((char *)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}
	if ((argn >= 2 && !lbm_is_number(args[1])) ||
			(argn >= 3 && !lbm_is_number(args[2]))) {
		lbm_set_error_reason((char *)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}
	int interface = argn >= 2 ? lbm_dec_as_i32(args[1]) : 0;
	int can_id = argn >= 3 ? lbm_dec_as_i32(args[2]) : 0;

	uint8_t packet[PACKET_MAX_PL_LEN] = {COMM_CUSTOM_APP_DATA};
	unsigned int len = 1;
	lbm_array_header_t *array = lbm_dec_array_r(args[0]);
	if (array) {
		if (array->size > sizeof(packet) - len) {
			return ENC_SYM_EERROR;
		}
		memcpy(packet + len, array->data, array->size);
		len += array->size;
	} else {
		lbm_value list = args[0];
		if (!lbm_is_cons(list)) {
			lbm_set_error_reason((char *)lbm_error_str_incorrect_arg);
			return ENC_SYM_TERROR;
		}
		// Keep the VESC list payload limit of 100 bytes.
		while (lbm_is_cons(list) && len <= 100 && len < sizeof(packet)) {
			lbm_value byte = lbm_car(list);
			if (!lbm_is_number(byte)) {
				return ENC_SYM_EERROR;
			}
			packet[len++] = lbm_dec_as_u32(byte);
			list = lbm_cdr(list);
		}
	}

	switch (interface) {
	case 0:
		commands_send_packet(packet, len);
		break;
#if SOC_USB_SERIAL_JTAG_SUPPORTED
	case 1:
		comm_usb_send_packet(packet, len);
		break;
#endif
	case 2:
		if (can_id < 0 || can_id > UINT8_MAX) {
			lbm_set_error_reason("Invalid CAN ID");
			return ENC_SYM_EERROR;
		}
		comm_can_send_buffer(can_id, packet, len, 3);
		break;
	case 3:
		comm_uart_send_packet(packet, len, 0);
		break;
	case 4:
		comm_uart_send_packet(packet, len, 1);
		break;
	/*
	case 6:
		comm_wifi_send_packet_local(packet, len);
		break;
	case 7:
		comm_wifi_send_packet_hub(packet, len);
		break;
	case 8:
		comm_ble_send_packet(packet, len);
		break;
	*/
	default:
		lbm_set_error_reason("Unsupported send-data interface");
		return ENC_SYM_EERROR;
	}
	return ENC_SYM_TRUE;
}

void lispif_load_system_extensions(void) {
	lbm_add_extension("print", ext_print);
	lbm_add_extension("puts", ext_puts);
	lbm_add_extension("systime", ext_systime);
	lbm_add_extension("secs-since", ext_secs_since);
	lbm_add_extension("crc16", ext_crc16);
	lbm_add_extension("crc32", ext_crc32);
	lbm_add_extension("import", ext_import);
	lbm_add_extension("send-data", ext_send_data);
}
