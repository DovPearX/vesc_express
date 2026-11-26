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

#ifndef MAIN_COMM_CAN_H_
#define MAIN_COMM_CAN_H_

#include "datatypes.h"
#include "soc/soc_caps.h"

typedef struct {
	bool enabled;
	uint32_t rx_overflow;
	uint32_t tx_rejected;
	uint32_t bus_off;
	uint32_t recoveries;
} comm_can_stats_t;

bool comm_can_start_bus(unsigned bus, int pin_tx, int pin_rx, CAN_BAUD baud);
void comm_can_stop_bus(unsigned bus);
bool comm_can_ping_bus(unsigned bus, uint8_t controller_id, HW_TYPE *hw_type);
void comm_can_transmit_bus(unsigned bus, uint32_t id, const uint8_t *data, uint8_t len, bool ext);
bool comm_can_get_stats(unsigned bus, comm_can_stats_t *stats);
void comm_can_send_buffer_bus(unsigned bus, uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send);

void comm_can_start(int pin_tx, int pin_rx);
void comm_can_stop(void);
void comm_can_use_vesc_decoder(bool use_vesc_dec);
CAN_BAUD comm_can_kbits_to_baud(int kbits);
void comm_can_update_baudrate(int delay_msec);
void comm_can_change_pins(int tx, int rx);
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send);
bool comm_can_ping(uint8_t controller_id, HW_TYPE *hw_type);

void comm_can_send_update_baud(int kbits, int delay_msec);

#if SOC_TWAI_CONTROLLER_NUM > 1
bool comm_can2_start(int pin_tx, int pin_rx, int baud_kbits);
void comm_can2_stop(void);
void comm_can2_use_vesc_decoder(bool use);
void comm_can2_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can2_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can2_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send_type);
#endif

#endif /* MAIN_COMM_CAN_H_ */
