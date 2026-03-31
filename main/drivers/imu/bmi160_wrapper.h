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

#ifndef BMI160_WRAPPER_H_
#define BMI160_WRAPPER_H_

#include "imu.h"

void bmi160_wrapper_set_rate_hz(int hz);
void bmi160_wrapper_set_filter(IMU_FILTER f);
void bmi160_wrapper_init(void);
void bmi160_wrapper_set_read_callback(void(*func)(float *accel, float *gyro, float *mag));
void bmi160_wrapper_stop(void);

#endif /* BMI160_WRAPPER_H_ */