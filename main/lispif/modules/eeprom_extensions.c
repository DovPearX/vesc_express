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

#include "flash_helper.h"
#include "lispbm.h"
#include "eeprom_extensions.h"

static bool check_eeprom_addr(int addr) {
	if (addr < 0 || addr >= EEPROM_VARS) {
		lbm_set_error_reason("Address must be 0 to 511");
		return false;
	}

	return true;
}

static lbm_value ext_eeprom_store_f(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int addr = lbm_dec_as_i32(args[0]);
	if (!check_eeprom_addr(addr)) {
		return ENC_SYM_EERROR;
	}

	eeprom_var v;
	v.as_float = lbm_dec_as_float(args[1]);
	return store_eeprom_var(&v, addr, 1) ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_eeprom_read_f(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int addr = lbm_dec_as_i32(args[0]);
	if (!check_eeprom_addr(addr)) {
		return ENC_SYM_EERROR;
	}

	eeprom_var v;
	bool res = read_eeprom_var(&v, addr, 1);
	return res ? lbm_enc_float(v.as_float) : ENC_SYM_NIL;
}

static lbm_value ext_eeprom_store_i(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int addr = lbm_dec_as_i32(args[0]);
	if (!check_eeprom_addr(addr)) {
		return ENC_SYM_EERROR;
	}

	eeprom_var v;
	v.as_i32 = lbm_dec_as_i32(args[1]);
	return store_eeprom_var(&v, addr, 1) ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_eeprom_read_i(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int addr = lbm_dec_as_i32(args[0]);
	if (!check_eeprom_addr(addr)) {
		return ENC_SYM_EERROR;
	}

	eeprom_var v;
	bool res = read_eeprom_var(&v, addr, 1);
	return res ? lbm_enc_i32(v.as_i32) : ENC_SYM_NIL;
}

static lbm_value ext_eeprom_erase(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int addr = lbm_dec_as_i32(args[0]);
	if (!check_eeprom_addr(addr)) {
		return ENC_SYM_EERROR;
	}

	if (!erase_eeprom_var(addr, 1)) {
		return ENC_SYM_EERROR;
	}
	return ENC_SYM_TRUE;
}

void lispif_load_eeprom_extensions(void) {
	lbm_add_extension("eeprom-store-f", ext_eeprom_store_f);
	lbm_add_extension("eeprom-read-f", ext_eeprom_read_f);
	lbm_add_extension("eeprom-store-i", ext_eeprom_store_i);
	lbm_add_extension("eeprom-read-i", ext_eeprom_read_i);
	lbm_add_extension("eeprom-erase", ext_eeprom_erase);
}
