/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se
	Copyright 2022 Joel Svensson    svenssonjoel@yahoo.se

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

#ifndef LISPBM_LISPIF_H_
#define LISPBM_LISPIF_H_

#include <stdint.h>
#include <stdbool.h>

void lispif_init(void);
bool lispif_stop(void);
void lispif_process_cmd(unsigned char *data, unsigned int len,
		void (*reply_func)(unsigned char *data, unsigned int len));
void lispif_process_can(unsigned bus, uint32_t id, uint8_t *data, uint8_t len, bool extended);

#endif /* LISPBM_LISPIF_H_ */
