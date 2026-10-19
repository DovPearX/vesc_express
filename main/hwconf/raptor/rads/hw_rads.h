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

#ifndef HWCONF_RADS_H_
#define HWCONF_RADS_H_

#include "datatypes.h"

#define HW_NAME				"RADS"
#define HW_TARGET           "esp32s3_n16r8"
#define HW_INTERNAL_FS

#define HW_DEFAULT_ID 20

#define HW_INIT_HOOK() 			hw_init()

// CAN
#define CAN_TX_GPIO_NUM			6
#define CAN_RX_GPIO_NUM			5

// Display
#define DISP_SD0			13
#define DISP_CLK			10
#define DISP_CS				12
#define DISP_RESET			9
#define DISP_DC				11
#define DISP_BACKLIGHT_GPIO	14

// Touch CST816
#define TOUCH_I2C_SDA		47
#define TOUCH_I2C_SCL		48
#define TOUCH_RST			17
#define TOUCH_INT			21

// QMI8658 IMU
#define QMI_I2C_SDA			47
#define QMI_I2C_SCL			48
#define QMI_INT1		 	8
#define QMI_INT2		 	7
#define QMI_I2C_ADDR		0x6B

// Functions
void hw_init(void);

#endif /* HWCONF_RADS_H_ */
