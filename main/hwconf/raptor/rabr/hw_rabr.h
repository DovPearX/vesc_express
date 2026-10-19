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

#ifndef HWCONF_RABR_H_
#define HWCONF_RABR_H_

#define HW_NAME				"RABR"
#define HW_TARGET           "esp32c3"

#define HW_DEFAULT_ID 10

#define HW_INIT_HOOK() 			hw_init()

// CAN
#define CAN_TX_GPIO_NUM			0
#define CAN_RX_GPIO_NUM			1

// Other pins
#define PIN_LED_SYSTEM 8
#define PIN_LED_DATA 9
#define PIN_HV_INPUT 7

// Functions
void hw_init(void);

#endif /* HWCONF_RABR_H_ */
