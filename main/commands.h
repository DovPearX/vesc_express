/*
	Copyright 2022 Benjamin Vedder      benjamin@vedder.se
	Copyright 2023 Rasmus Söderhielm    rasmus.soderhielm@gmail.com

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

#ifndef MAIN_COMMANDS_H_
#define MAIN_COMMANDS_H_

#include <stdint.h>
#include <stdbool.h>
#include "datatypes.h"

typedef void (*send_func_t)(unsigned char *, unsigned int);

void commands_process_can_packet(unsigned char *data, unsigned int len, unsigned bus, uint8_t sender, uint8_t response_type);
void commands_queue_can_forward(unsigned bus, unsigned char *data, unsigned int len);
void commands_queue_can_baud(CAN_BAUD baud, int delay_ms);
uint32_t commands_get_dropped_count(void);

// Functions
void commands_init(void);
void commands_process_packet(
	unsigned char *data, unsigned int len, send_func_t reply_func
);
void commands_send_packet(unsigned char *data, unsigned int len);
void commands_send_packet_can_last(unsigned char *data, unsigned int len);
int commands_printf(const char *format, ...);

#endif /* MAIN_COMMANDS_H_ */
