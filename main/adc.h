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

#ifndef MAIN_ADC_H_
#define MAIN_ADC_H_

#include "conf_general.h"
#include "hal/adc_types.h"

// Compatibility aliases for hardware configs that still use legacy ADC1 names.
typedef adc_channel_t adc1_channel_t;

#ifndef ADC1_CHANNEL_0
#define ADC1_CHANNEL_0 ADC_CHANNEL_0
#endif
#ifndef ADC1_CHANNEL_1
#define ADC1_CHANNEL_1 ADC_CHANNEL_1
#endif
#ifndef ADC1_CHANNEL_2
#define ADC1_CHANNEL_2 ADC_CHANNEL_2
#endif
#ifndef ADC1_CHANNEL_3
#define ADC1_CHANNEL_3 ADC_CHANNEL_3
#endif
#ifndef ADC1_CHANNEL_4
#define ADC1_CHANNEL_4 ADC_CHANNEL_4
#endif
#ifndef ADC1_CHANNEL_5
#define ADC1_CHANNEL_5 ADC_CHANNEL_5
#endif
#ifndef ADC1_CHANNEL_6
#define ADC1_CHANNEL_6 ADC_CHANNEL_6
#endif
#ifndef ADC1_CHANNEL_7
#define ADC1_CHANNEL_7 ADC_CHANNEL_7
#endif
#ifndef ADC1_CHANNEL_8
#define ADC1_CHANNEL_8 ADC_CHANNEL_8
#endif
#ifndef ADC1_CHANNEL_9
#define ADC1_CHANNEL_9 ADC_CHANNEL_9
#endif

// Functions
void adc_init(void);
float adc_get_voltage(adc1_channel_t ch);

#endif /* MAIN_ADC_H_ */
