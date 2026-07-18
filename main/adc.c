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

#include "adc.h"
#include "terminal.h"
#include "commands.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <math.h>

// Private variables
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool cal_ok = false;

void adc_init(void) {
	adc_oneshot_unit_init_cfg_t unit_config = {
		.unit_id = ADC_UNIT_1,
	};
	adc_oneshot_new_unit(&unit_config, &adc1_handle);

	adc_oneshot_chan_cfg_t channel_config = {
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};

#ifdef HW_ADC_CH0
	adc_oneshot_config_channel(adc1_handle, HW_ADC_CH0, &channel_config);
#endif
#ifdef HW_ADC_CH1
	adc_oneshot_config_channel(adc1_handle, HW_ADC_CH1, &channel_config);
#endif
#ifdef HW_ADC_CH2
	adc_oneshot_config_channel(adc1_handle, HW_ADC_CH2, &channel_config);
#endif
#ifdef HW_ADC_CH3
	adc_oneshot_config_channel(adc1_handle, HW_ADC_CH3, &channel_config);
#endif
#ifdef HW_ADC_CH4
	adc_oneshot_config_channel(adc1_handle, HW_ADC_CH4, &channel_config);
#endif

	#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
	adc_cali_curve_fitting_config_t cali_config = {
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};

	if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle) == ESP_OK) {
		cal_ok = true;
	}
	#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
	adc_cali_line_fitting_config_t cali_config = {
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
		.default_vref = 0,
	};
	if (adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle) == ESP_OK) {
		cal_ok = true;
	}
	#endif
}

float adc_get_voltage(adc_channel_t ch) {
	float res = -1.0;

	if (cal_ok) {
		int voltage_mv = 0;
		if (adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, ch, &voltage_mv) == ESP_OK) {
			res = (float)voltage_mv / 1000.0;
		}
	}

	return res;
}
