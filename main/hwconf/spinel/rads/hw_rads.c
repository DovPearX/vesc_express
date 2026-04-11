/*
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

#include "hw_rads.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lispif_disp_extensions.h"
#include "disp_sh8601.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_sleep.h"

#include "lispif.h"
#include "lispbm.h"
#include "extensions/display_extensions.h"
#include "terminal.h"
#include "commands.h"
#include "utils.h"

static lbm_value ext_disp_init(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	disp_sh8601_init(
			DISP_SD0,
			DISP_CLK,
			DISP_CS,
			DISP_RESET,
			DISP_DC,
			40);

	lbm_display_extensions_set_callbacks(
			disp_sh8601_render_image,
			disp_sh8601_clear,
			disp_sh8601_reset
	);

	disp_sh8601_reset();

	return ENC_SYM_TRUE;
}

static void load_extensions(bool main_found) {
	if (!main_found) {
		lbm_add_extension("disp-init", ext_disp_init);
	}
}

void hw_init(void) {
	lispif_add_ext_load_callback(load_extensions);
}
