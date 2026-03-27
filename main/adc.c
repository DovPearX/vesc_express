/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "adc.h"
#include "terminal.h"
#include "commands.h"

#include <math.h>

// Private variables
static bool cal_ok = false;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle;

static void adc_config_channel(adc_channel_t ch) {
	adc_oneshot_chan_cfg_t chan_cfg = {
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	adc_oneshot_config_channel(adc1_handle, ch, &chan_cfg);
}

void adc_init(void) {
	adc_oneshot_unit_init_cfg_t init_cfg = {
		.unit_id = ADC_UNIT_1,
		.clk_src = 0,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	if (adc_oneshot_new_unit(&init_cfg, &adc1_handle) != ESP_OK) {
		return;
	}

#ifdef HW_ADC_CH0
	adc_config_channel(HW_ADC_CH0);
#endif
#ifdef HW_ADC_CH1
	adc_config_channel(HW_ADC_CH1);
#endif
#ifdef HW_ADC_CH2
	adc_config_channel(HW_ADC_CH2);
#endif
#ifdef HW_ADC_CH3
	adc_config_channel(HW_ADC_CH3);
#endif
#ifdef HW_ADC_CH4
	adc_config_channel(HW_ADC_CH4);
#endif

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
	adc_cali_curve_fitting_config_t cali_cfg = {
		.unit_id = ADC_UNIT_1,
		.chan = ADC_CHANNEL_0,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	cal_ok = adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc1_cali_handle) == ESP_OK;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
	adc_cali_line_fitting_config_t cali_cfg = {
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	cal_ok = adc_cali_create_scheme_line_fitting(&cali_cfg, &adc1_cali_handle) == ESP_OK;
#else
	cal_ok = false;
#endif
}

float adc_get_voltage(adc1_channel_t ch) {
	float res = -1.0;

	int raw = 0;
	if (adc_oneshot_read(adc1_handle, ch, &raw) == ESP_OK) {
		if (cal_ok) {
			int mv = 0;
			if (adc_cali_raw_to_voltage(adc1_cali_handle, raw, &mv) == ESP_OK) {
				res = (float)mv / 1000.0;
			}
		} else {
			// Fallback if calibration eFuse data/scheme is unavailable.
			res = ((float)raw / 4095.0) * 3.3;
		}
	}

	return res;
}
