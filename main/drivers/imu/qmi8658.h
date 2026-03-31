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

#ifndef QMI8658_H_
#define QMI8658_H_

#include "imu.h"

void qmi8658_set_rate_hz(int hz);
void qmi8658_set_filter(IMU_FILTER f);
void qmi8658_init(void);
void qmi8658_set_read_callback(void(*func)(float *accel, float *gyro, float *mag));
void qmi8658_stop(void);

#endif /* QMI8658_H_ */
