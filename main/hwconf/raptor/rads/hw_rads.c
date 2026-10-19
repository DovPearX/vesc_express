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
#include "driver/ledc.h"
#include "esp_log.h"
#include "lispif_disp_extensions.h"
#include "disp_sh8601.h"
#include "esp_wifi.h"
#include "esp_bt.h"
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

static bool backlight_pwm_initialized = false;

// Set the active-low display backlight. Brightness is expressed as a
// percentage: 0 is dark (GPIO high), 100 is brightest (GPIO low).
static lbm_value ext_set_brightness(lbm_value *args, lbm_uint argn) {
	if (argn != 1 || !lbm_is_number(args[0])) {
		return ENC_SYM_NIL;
	}

	float brightness = lbm_dec_as_float(args[0]);
	if (brightness < 0.0f) {
		brightness = 0.0f;
	} else if (brightness > 100.0f) {
		brightness = 100.0f;
	}

	if (!backlight_pwm_initialized) {
		ledc_timer_config_t timer = {
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.duty_resolution = LEDC_TIMER_10_BIT,
			.timer_num = LEDC_TIMER_0,
			.freq_hz = 20000,
			.clk_cfg = LEDC_AUTO_CLK,
		};
		if (ledc_timer_config(&timer) != ESP_OK) {
			return ENC_SYM_NIL;
		}

		ledc_channel_config_t channel = {
			.gpio_num = DISP_BACKLIGHT_GPIO,
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.channel = LEDC_CHANNEL_0,
			.intr_type = LEDC_INTR_DISABLE,
			.timer_sel = LEDC_TIMER_0,
			.duty = 1024,
			.hpoint = 0,
		};
		if (ledc_channel_config(&channel) != ESP_OK) {
			return ENC_SYM_NIL;
		}
		backlight_pwm_initialized = true;
	}

	// The backlight input is active-low, so reverse the PWM duty cycle.
	uint32_t duty = (uint32_t)((100.0f - brightness) * 1024.0f / 100.0f + 0.5f);
	if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty) != ESP_OK ||
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0) != ESP_OK) {
		return ENC_SYM_NIL;
	}
	return ENC_SYM_TRUE;
}

static void load_extensions(bool main_found) {
	if (!main_found) {
		lbm_add_extension("disp-init", ext_disp_init);
		lbm_add_extension("ext-set-brightness", ext_set_brightness);
	}
}

void hw_init(void) {
	lispif_add_ext_load_callback(load_extensions);
}
