/*
	Copyright 2022 - 2024 Benjamin Vedder	benjamin@vedder.se

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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "datatypes.h"
#include "buffer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "comm_can.h"
#include "datatypes.h"
#include "conf_general.h"
#include "main.h"
#include "crc.h"
#include "packet.h"
#include "commands.h"
#include "lispif.h"
#include <string.h>

// Status messages

#define RX_BUFFER_NUM				3
#define RX_BUFFER_SIZE				PACKET_MAX_PL_LEN
#define RXBUF_LEN					50

#define TXBUF_LEN 21

typedef struct {
	uint32_t identifier;
	uint8_t data_length_code;
	bool extd;
	uint8_t data[8];
} can_message_t;

typedef struct {
	twai_frame_t frame;
	uint8_t data[8];
} can_tx_message_t;

typedef struct {
	twai_node_handle_t node;
	QueueHandle_t rx_queue;
	QueueHandle_t tx_free;
	SemaphoreHandle_t send_mutex;
	SemaphoreHandle_t packet_mutex;
	can_tx_message_t tx_buf[TXBUF_LEN];
	int pin_tx;
	int pin_rx;
	bool bus_off;
	bool recover_requested;
	comm_can_stats_t stats;
	SemaphoreHandle_t ping_sem;
	HW_TYPE ping_hw;
} can_state_t;

static DRAM_ATTR can_state_t buses[SOC_TWAI_CONTROLLER_NUM];

static volatile bool init_done;
static TaskHandle_t can_worker;
static portMUX_TYPE stats_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
	uint8_t buffer[RX_BUFFER_NUM][RX_BUFFER_SIZE];
	int offset[RX_BUFFER_NUM];
} can_rx_state_t;

static can_rx_state_t rx_states[SOC_TWAI_CONTROLLER_NUM];
static bool decoder_disabled[SOC_TWAI_CONTROLLER_NUM];
static bool can_transmit(can_state_t *state, uint32_t id, const uint8_t *data, uint8_t len, bool ext, TickType_t wait);

static void decode_msg(unsigned bus, can_message_t *msg, bool is_replaced) {
	can_rx_state_t *rx = &rx_states[bus];
	uint8_t *data8 = msg->data;
	int len = msg->data_length_code;
	int32_t ind = 0;
	uint8_t crc_low;
	uint8_t crc_high;
	uint8_t commands_send;

	uint8_t id = msg->identifier & 0xFF;
	CAN_PACKET_ID cmd = msg->identifier >> 8;

	if (id == 255 || id == backup.config.controller_id) {
		switch (cmd) {
		case CAN_PACKET_FILL_RX_BUFFER: {
			if (len < 2) {
				break;
			}
			int buf_ind = -1;
			int offset = data8[0];
			data8++;
			len--;

			for (int i = 0; i < RX_BUFFER_NUM;i++) {
				if ((rx->offset[i]) == offset ) {
					buf_ind = i;
					break;
				}
			}

			if (buf_ind < 0) {
				if (offset == 0) {
					buf_ind = 0;
				} else {
					break;
				}
			}

			memcpy(rx->buffer[buf_ind] + offset, data8, len);
			rx->offset[buf_ind] = offset + len;
		} break;

		case CAN_PACKET_FILL_RX_BUFFER_LONG: {
			if (len < 3) {
				break;
			}
			int buf_ind = -1;
			int offset = (int)data8[0] << 8;
			offset |= data8[1];
			data8 += 2;
			len -= 2;

			for (int i = 0; i < RX_BUFFER_NUM;i++) {
				if ((rx->offset[i]) == offset ) {
					buf_ind = i;
					break;
				}
			}

			if (buf_ind < 0) {
				if (offset == 0) {
					buf_ind = 0;
				} else {
					break;
				}
			}

			if ((offset + len) <= RX_BUFFER_SIZE) {
				memcpy(rx->buffer[buf_ind] + offset, data8, len);
				rx->offset[buf_ind] = offset + len;
			}
		} break;

		case CAN_PACKET_PROCESS_RX_BUFFER: {
			if (len < 6) {
				break;
			}
			ind = 0;
			unsigned int last_id = data8[ind++];
			commands_send = data8[ind++];

			int rxbuf_len = (int)data8[ind++] << 8;
			rxbuf_len |= (int)data8[ind++];

			if (rxbuf_len > RX_BUFFER_SIZE) {
				break;
			}

			int buf_ind = -1;
			for (int i = 0; i < RX_BUFFER_NUM;i++) {
				if ((rx->offset[i]) == rxbuf_len ) {
					buf_ind = i;
					break;
				}
			}

			// Something is wrong, reset all buffers
			if (buf_ind < 0) {
				for (int i = 0; i < RX_BUFFER_NUM;i++) {
					rx->offset[i] = 0;
				}
				break;
			}

			rx->offset[buf_ind] = 0;

			crc_high = data8[ind++];
			crc_low = data8[ind++];

			if (crc16(rx->buffer[buf_ind], rxbuf_len)
					== ((unsigned short) crc_high << 8
							| (unsigned short) crc_low)) {

				if (is_replaced) {
					if (rx->buffer[buf_ind][0] == COMM_JUMP_TO_BOOTLOADER ||
							rx->buffer[buf_ind][0] == COMM_ERASE_NEW_APP ||
							rx->buffer[buf_ind][0] == COMM_WRITE_NEW_APP_DATA ||
							rx->buffer[buf_ind][0] == COMM_WRITE_NEW_APP_DATA_LZO ||
							rx->buffer[buf_ind][0] == COMM_ERASE_BOOTLOADER) {
						break;
					}
				}

				switch (commands_send) {
				case 0:
				case 3:
					commands_process_can_packet(rx->buffer[buf_ind], rxbuf_len, bus, last_id, commands_send == 3 ? 0 : 1);
					break;
				case 1:
					commands_queue_can_forward(bus, rx->buffer[buf_ind], rxbuf_len);
					break;
				case 2:
					commands_process_can_packet(rx->buffer[buf_ind], rxbuf_len, bus, last_id, 2);
					break;
				default:
					break;
				}
			}
		} break;

		case CAN_PACKET_PROCESS_SHORT_BUFFER:
			if (len < 3) {
				break;
			}
			ind = 0;
			unsigned int last_id = data8[ind++];
			commands_send = data8[ind++];

			if (is_replaced) {
				if (data8[ind] == COMM_JUMP_TO_BOOTLOADER ||
						data8[ind] == COMM_ERASE_NEW_APP ||
						data8[ind] == COMM_WRITE_NEW_APP_DATA ||
						data8[ind] == COMM_WRITE_NEW_APP_DATA_LZO ||
						data8[ind] == COMM_ERASE_BOOTLOADER) {
					break;
				}
			}

			switch (commands_send) {
			case 0:
			case 3:
				commands_process_can_packet(data8 + ind, len - ind, bus, last_id, commands_send == 3 ? 0 : 1);
				break;
			case 1:
				commands_queue_can_forward(bus, data8 + ind, len - ind);
				break;
			case 2:
				commands_process_can_packet(data8 + ind, len - ind, bus, last_id, 2);
				break;
			default:
				break;
			}
			break;

		case CAN_PACKET_PING: {
			uint8_t buffer[2];
			buffer[0] = backup.config.controller_id;
			buffer[1] = HW_TYPE_CUSTOM_MODULE;
			if (len >= 1) {
				can_transmit(&buses[bus], data8[0] | ((uint32_t)CAN_PACKET_PONG << 8), buffer, 2, true, 0);
			}
		} break;

		case CAN_PACKET_PONG: {
			can_state_t *state = &buses[bus];
			if (len >= 1) {
				state->ping_hw = len >= 2 ? data8[1] : HW_TYPE_VESC_BMS;
				xSemaphoreGive(state->ping_sem);
			}
		} break;

		default:
			break;
		}
	}

	// The packets below are addressed to all devices, mainly containing status information.

	switch (cmd) {
	case CAN_PACKET_UPDATE_BAUD: {
		if (bus != 0 || len < 4) {
			break;
		}
		ind = 0;
		int kbits = buffer_get_int16(data8, &ind);
		int delay_msec = buffer_get_int16(data8, &ind);
		CAN_BAUD baud = comm_can_kbits_to_baud(kbits);

		if (baud != CAN_BAUD_INVALID) {
			commands_queue_can_baud(baud, delay_msec);
		}
	} break;

	default:
		break;
	}
}

static bool IRAM_ATTR can_rx_done(twai_node_handle_t node, const twai_rx_done_event_data_t *event, void *arg) {
	can_state_t *state = arg;
	can_message_t msg = {0};
	twai_frame_t frame = {.buffer = msg.data, .buffer_len = sizeof(msg.data)};
	BaseType_t wake = pdFALSE;
	if (twai_node_receive_from_isr(node, &frame) == ESP_OK && !frame.header.rtr && !frame.header.fdf
		&& frame.header.dlc <= 8) {
		msg.identifier = frame.header.id;
		msg.data_length_code = frame.header.dlc;
		msg.extd = frame.header.ide;
		if (xQueueSendFromISR(state->rx_queue, &msg, &wake) == pdTRUE) {
			vTaskNotifyGiveFromISR(can_worker, &wake);
		} else {
			portENTER_CRITICAL_ISR(&stats_lock);
			state->stats.rx_overflow++;
			portEXIT_CRITICAL_ISR(&stats_lock);
		}
	}
	return wake == pdTRUE;
}

static bool IRAM_ATTR can_tx_done(twai_node_handle_t node, const twai_tx_done_event_data_t *event, void *arg) {
	can_state_t *state = arg;
	can_tx_message_t *msg = (can_tx_message_t *)event->done_tx_frame;
	BaseType_t wake = pdFALSE;
	xQueueSendFromISR(state->tx_free, &msg, &wake);
	return wake == pdTRUE;
}

static bool IRAM_ATTR can_state_change(
	twai_node_handle_t node, const twai_state_change_event_data_t *event, void *arg) {
	can_state_t *state = arg;
	BaseType_t wake = pdFALSE;
	if (event->new_sta == TWAI_ERROR_BUS_OFF || event->old_sta == TWAI_ERROR_BUS_OFF) {
		portENTER_CRITICAL_ISR(&stats_lock);
		state->bus_off = event->new_sta == TWAI_ERROR_BUS_OFF;
		state->recover_requested = false;
		if (state->bus_off) {
			state->stats.bus_off++;
		} else {
			state->stats.recoveries++;
		}
		portEXIT_CRITICAL_ISR(&stats_lock);
		if (event->new_sta == TWAI_ERROR_BUS_OFF) {
			vTaskNotifyGiveFromISR(can_worker, &wake);
		}
	}
	return wake == pdTRUE;
}

static void can_reset_tx(can_state_t *state) {
	xQueueReset(state->tx_free);
	for (int i = 0; i < TXBUF_LEN; i++) {
		can_tx_message_t *msg = &state->tx_buf[i];
		xQueueSend(state->tx_free, &msg, 0);
	}
}

static void can_node_stop(can_state_t *state) {
	if (state->node) {
		twai_node_disable(state->node);
		twai_node_delete(state->node);
		state->node = NULL;
	}
	portENTER_CRITICAL(&stats_lock);
	state->bus_off = false;
	state->recover_requested = false;
	state->stats.enabled = false;
	portEXIT_CRITICAL(&stats_lock);
}

static bool can_node_start_locked(can_state_t *state, int pin_tx, int pin_rx, uint32_t bitrate) {
	can_node_stop(state);
	xQueueReset(state->rx_queue);
	can_reset_tx(state);

	twai_onchip_node_config_t config = {
		.io_cfg = {.tx = pin_tx, .rx = pin_rx, .quanta_clk_out = -1, .bus_off_indicator = -1},
		.bit_timing = {.bitrate = bitrate},
		.fail_retry_cnt = -1,
		.tx_queue_depth = TXBUF_LEN - 1,
		.flags.no_receive_rtr = true,
	};
	const twai_event_callbacks_t callbacks = {
		.on_rx_done = can_rx_done,
		.on_tx_done = can_tx_done,
		.on_state_change = can_state_change,
	};
	esp_err_t res = twai_new_node_onchip(&config, &state->node);
	if (res == ESP_OK) {
		res = twai_node_register_event_callbacks(state->node, &callbacks, state);
	}
	if (res == ESP_OK) {
		res = twai_node_enable(state->node);
	}
	if (res != ESP_OK) {
		can_node_stop(state);
	} else {
		state->pin_tx = pin_tx;
		state->pin_rx = pin_rx;
		portENTER_CRITICAL(&stats_lock);
		state->stats.enabled = true;
		portEXIT_CRITICAL(&stats_lock);
	}
	return res == ESP_OK;
}

static bool can_node_start(can_state_t *state, int pin_tx, int pin_rx, uint32_t bitrate) {
	if (!state->send_mutex) {
		state->send_mutex = xSemaphoreCreateMutex();
	}
	if (!state->packet_mutex) {
		state->packet_mutex = xSemaphoreCreateMutex();
	}
	if (!state->rx_queue) {
		QueueHandle_t queue = xQueueCreate(RXBUF_LEN, sizeof(can_message_t));
		portENTER_CRITICAL(&stats_lock);
		state->rx_queue = queue;
		portEXIT_CRITICAL(&stats_lock);
	}
	if (!state->tx_free) {
		state->tx_free = xQueueCreate(TXBUF_LEN, sizeof(can_tx_message_t *));
	}
	if (!state->send_mutex || !state->packet_mutex || !state->rx_queue || !state->tx_free) {
		return false;
	}

	xSemaphoreTake(state->send_mutex, portMAX_DELAY);
	bool res = can_node_start_locked(state, pin_tx, pin_rx, bitrate);
	xSemaphoreGive(state->send_mutex);
	return res;
}

static bool can_recover(can_state_t *state) {
	portENTER_CRITICAL(&stats_lock);
	bool pending = state->bus_off && !state->recover_requested;
	portEXIT_CRITICAL(&stats_lock);
	if (!pending) {
		return false;
	}
	if (xSemaphoreTake(state->send_mutex, 0) != pdTRUE) {
		return true;
	}

	portENTER_CRITICAL(&stats_lock);
	pending = state->bus_off && !state->recover_requested;
	if (pending) {
		state->recover_requested = true;
	}
	portEXIT_CRITICAL(&stats_lock);

	bool retry = pending && state->node && twai_node_recover(state->node) != ESP_OK;
	if (retry) {
		portENTER_CRITICAL(&stats_lock);
		state->recover_requested = false;
		portEXIT_CRITICAL(&stats_lock);
	}
	xSemaphoreGive(state->send_mutex);
	return retry;
}

static bool can_transmit(can_state_t *state, uint32_t id, const uint8_t *data, uint8_t len, bool ext, TickType_t wait) {
	bool sent = false;
	if (state->send_mutex && xSemaphoreTake(state->send_mutex, wait) == pdTRUE) {
		can_tx_message_t *msg;
		TickType_t tx_wait = wait ? pdMS_TO_TICKS(5) : 0;
		if (state->node && xQueueReceive(state->tx_free, &msg, tx_wait) == pdTRUE) {
			len = len > 8 ? 8 : len;
			msg->frame = (twai_frame_t){
				.header = {.id = id, .dlc = len, .ide = ext},
				.buffer = msg->data,
				.buffer_len = len,
			};
			memcpy(msg->data, data, len);
			sent = twai_node_transmit(state->node, &msg->frame, 0) == ESP_OK;
			if (!sent) {
				xQueueSend(state->tx_free, &msg, 0);
			}
		}
		xSemaphoreGive(state->send_mutex);
	}
	if (!sent) {
		portENTER_CRITICAL(&stats_lock);
		state->stats.tx_rejected++;
		portEXIT_CRITICAL(&stats_lock);
	}
	return sent;
}

#define RX_FRAMES_PER_TURN 8

static void process_task(void *arg) {
	(void)arg;
	unsigned first = 0;
	TickType_t wait = portMAX_DELAY;
	for (;;) {
		ulTaskNotifyTake(pdTRUE, wait);
		wait = portMAX_DELAY;
		bool saturated = false;

		for (unsigned i = 0; i < SOC_TWAI_CONTROLLER_NUM; i++) {
			unsigned bus = (first + i) % SOC_TWAI_CONTROLLER_NUM;
			can_state_t *state = &buses[bus];
			if (can_recover(state)) {
				wait = pdMS_TO_TICKS(10) + 1;
			}
			portENTER_CRITICAL(&stats_lock);
			QueueHandle_t queue = state->rx_queue;
			bool decode = !decoder_disabled[bus];
			portEXIT_CRITICAL(&stats_lock);
			if (!queue) {
				continue;
			}
			can_message_t msg;
			unsigned count = 0;
			while (count < RX_FRAMES_PER_TURN && xQueueReceive(queue, &msg, 0) == pdTRUE) {
				count++;

				lispif_process_can(bus, msg.identifier, msg.data, msg.data_length_code, msg.extd);
				if (decode && msg.extd) {
					decode_msg(bus, &msg, false);
				}
			}
			saturated |= count == RX_FRAMES_PER_TURN;
		}

		first = (first + 1) % SOC_TWAI_CONTROLLER_NUM;

		if (saturated) {
			wait = 0;
			taskYIELD();
		}
	}
}

static uint32_t baudrate_to_bitrate(CAN_BAUD baudrate) {
	switch (baudrate) {
		case CAN_BAUD_125K:
			return 125000;
		case CAN_BAUD_250K:
			return 250000;
		case CAN_BAUD_1M:
			return 1000000;
		case CAN_BAUD_10K:
			return 10000;
		case CAN_BAUD_20K:
			return 20000;
		case CAN_BAUD_50K:
			return 50000;
		case CAN_BAUD_75K:
			// The ESP-IDF 6.1 timing calculator does not find a valid even
			// prescaler when asked for exactly 75000 bit/s with a 40 or 80 MHz
			// TWAI clock. Requesting 75188 instead gives 19 time quanta per bit
			// with a prescaler of 28 at 40 MHz, or 56 at 80 MHz. Both produce
			// about 75187.97 bit/s, which is 0.25% above the nominal 75 kbit/s.
			// This was tested on C6. The same timing fits the other supported
			// ESP targets with 40/80 MHz TWAI clocks. A different source clock
			// or a change to the IDF timing calculator needs to be checked again.
			return 75188;
		case CAN_BAUD_100K:
			return 100000;
		case CAN_BAUD_500K:
		default:
			return 500000;
	}
}

bool comm_can_start_bus(unsigned bus, int pin_tx, int pin_rx, CAN_BAUD baud) {
	if (bus >= SOC_TWAI_CONTROLLER_NUM || baud == CAN_BAUD_INVALID) {
		return false;
	}
	can_state_t *state = &buses[bus];
	if (!state->ping_sem) {
		state->ping_sem = xSemaphoreCreateBinary();
		if (!state->ping_sem) {
			return false;
		}
	}
	if (!can_worker && xTaskCreate(process_task, "can_proc", 3072, NULL, 7, &can_worker) != pdPASS) {
		return false;
	}

	bool started = can_node_start(state, pin_tx, pin_rx, baudrate_to_bitrate(baud));
	if (bus == 0) {
		init_done = started;
	}
	return started;
}

void comm_can_start(int pin_tx, int pin_rx) {
	if (init_done) {
		comm_can_change_pins(pin_tx, pin_rx);
		return;
	}
	if (!comm_can_start_bus(0, pin_tx, pin_rx, backup.config.can_baud_rate)) {
		ESP_LOGE("can", "Failed to start CAN1");
	}
}

void comm_can_stop_bus(unsigned bus) {
	if (bus >= SOC_TWAI_CONTROLLER_NUM || !buses[bus].send_mutex) {
		return;
	}
	can_state_t *state = &buses[bus];
	xSemaphoreTake(state->send_mutex, portMAX_DELAY);
	if (bus == 0) {
		init_done = false;
	}
	can_node_stop(state);
	if (state->rx_queue) {
		xQueueReset(state->rx_queue);
	}
	xSemaphoreGive(state->send_mutex);
}

void comm_can_stop(void) {
	comm_can_stop_bus(0);
}

void comm_can_use_vesc_decoder(bool use_vesc_dec) {
	portENTER_CRITICAL(&stats_lock);
	decoder_disabled[0] = !use_vesc_dec;
	portEXIT_CRITICAL(&stats_lock);
}

CAN_BAUD comm_can_kbits_to_baud(int kbits) {
	CAN_BAUD new_baud = CAN_BAUD_INVALID;
	switch (kbits) {
	case 125: new_baud = CAN_BAUD_125K; break;
	case 250: new_baud = CAN_BAUD_250K; break;
	case 500: new_baud = CAN_BAUD_500K; break;
	case 1000: new_baud = CAN_BAUD_1M; break;
	case 10: new_baud = CAN_BAUD_10K; break;
	case 20: new_baud = CAN_BAUD_20K; break;
	case 50: new_baud = CAN_BAUD_50K; break;
	case 75: new_baud = CAN_BAUD_75K; break;
	case 100: new_baud = CAN_BAUD_100K; break;
	default: new_baud = CAN_BAUD_INVALID; break;
	}
	return new_baud;
}

void comm_can_update_baudrate(int delay_msec) {
	if (!init_done) {
		return;
	}
	if (delay_msec > 0) {
		vTaskDelay(pdMS_TO_TICKS(delay_msec));
	}
	can_state_t *state = &buses[0];
	xSemaphoreTake(state->send_mutex, portMAX_DELAY);
	if (init_done) {
		init_done = can_node_start_locked(state, state->pin_tx, state->pin_rx, baudrate_to_bitrate(backup.config.can_baud_rate));
	}
	xSemaphoreGive(state->send_mutex);
}

void comm_can_change_pins(int tx, int rx) {
	if (!init_done) {
		return;
	}
	can_state_t *state = &buses[0];
	xSemaphoreTake(state->send_mutex, portMAX_DELAY);
	if (init_done && (state->pin_tx != tx || state->pin_rx != rx)) {
		init_done = can_node_start_locked(state, tx, rx, baudrate_to_bitrate(backup.config.can_baud_rate));
	}
	xSemaphoreGive(state->send_mutex);
}

bool comm_can_transmit_bus(unsigned bus, uint32_t id, const uint8_t *data, uint8_t len, bool ext) {
	return bus < SOC_TWAI_CONTROLLER_NUM && can_transmit(&buses[bus], id, data, len, ext, portMAX_DELAY);
}

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	comm_can_transmit_bus(0, id, data, len, true);
}

void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) {
	comm_can_transmit_bus(0, id, data, len, false);
}

/**
 * Send a buffer up to RX_BUFFER_SIZE bytes as fragments. If the buffer is 6 bytes or less
 * it will be sent in a single CAN frame, otherwise it will be split into
 * several frames.
 *
 * @param controller_id
 * The controller id to send to.
 *
 * @param data
 * The payload.
 *
 * @param len
 * The payload length.
 *
 * @param send
 * 0: Packet goes to commands_process_packet of receiver
 * 1: Packet goes to commands_send_packet of receiver
 * 2: Packet goes to commands_process and send function is set to null
 *    so that no reply is sent back.
 * 3: Same as 0, but the reply is processed locally and not sent out on the last interface.
 */
void comm_can_send_buffer_bus(unsigned bus, uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send) {
	if (bus >= SOC_TWAI_CONTROLLER_NUM || len > RX_BUFFER_SIZE) {
		return;
	}

	can_state_t *state = &buses[bus];
	if (!state->packet_mutex) {
		return;
	}

	xSemaphoreTake(state->packet_mutex, portMAX_DELAY);
	uint8_t buffer[8];

	if (len <= 6) {
		buffer[0] = backup.config.controller_id;
		buffer[1] = send;
		memcpy(buffer + 2, data, len);
		uint32_t id = controller_id | ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8);
		can_transmit(state, id, buffer, len + 2, true, portMAX_DELAY);
		xSemaphoreGive(state->packet_mutex);
		return;
	}

	unsigned offset = 0;
	while (offset < len) {
		bool long_offset = offset > UINT8_MAX;
		unsigned header_len = long_offset ? 2 : 1;
		unsigned chunk_len = sizeof(buffer) - header_len;
		if (chunk_len > len - offset) {
			chunk_len = len - offset;
		}

		if (long_offset) {
			buffer[0] = offset >> 8;
		}
		buffer[header_len - 1] = offset;
		memcpy(buffer + header_len, data + offset, chunk_len);

		CAN_PACKET_ID command = long_offset ? CAN_PACKET_FILL_RX_BUFFER_LONG : CAN_PACKET_FILL_RX_BUFFER;
		uint32_t id = controller_id | ((uint32_t)command << 8);
		if (!can_transmit(state, id, buffer, header_len + chunk_len, true, portMAX_DELAY)) {
			xSemaphoreGive(state->packet_mutex);
			return;
		}
		offset += chunk_len;
	}

	uint16_t crc = crc16(data, len);
	buffer[0] = backup.config.controller_id;
	buffer[1] = send;
	buffer[2] = len >> 8;
	buffer[3] = len;
	buffer[4] = crc >> 8;
	buffer[5] = crc;

	uint32_t id = controller_id | ((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8);
	can_transmit(state, id, buffer, 6, true, portMAX_DELAY);
	xSemaphoreGive(state->packet_mutex);
}

void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send) {
	comm_can_send_buffer_bus(0, controller_id, data, len, send);
}

/**
 * Check if a VESC on the CAN-bus responds.
 *
 * @param controller_id
 * The ID of the VESC.
 *
 * @param hw_type
 * The hardware type of the CAN device.
 *
 * @return
 * True for success, false otherwise.
 */
bool comm_can_ping_bus(unsigned bus, uint8_t controller_id, HW_TYPE *hw_type) {
	if (bus >= SOC_TWAI_CONTROLLER_NUM) {
		return false;
	}
	can_state_t *state = &buses[bus];
	if (!state->ping_sem) {
		return false;
	}

	xSemaphoreTake(state->ping_sem, 0);
	uint8_t controller = backup.config.controller_id;
	if (!can_transmit(state, controller_id | ((uint32_t)CAN_PACKET_PING << 8), &controller, 1, true, portMAX_DELAY)) {
		return false;
	}
	bool received = xSemaphoreTake(state->ping_sem, pdMS_TO_TICKS(10) + 1) == pdTRUE;

	if (received && hw_type) {
		*hw_type = state->ping_hw;
	}
	return received;
}

bool comm_can_ping(uint8_t controller_id, HW_TYPE *hw_type) {
	return init_done && comm_can_ping_bus(0, controller_id, hw_type);
}

void comm_can_send_update_baud(int kbits, int delay_msec) {
	int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_int16(buffer, kbits, &send_index);
	buffer_append_int16(buffer, delay_msec, &send_index);
	comm_can_transmit_eid(255 |
			((uint32_t)CAN_PACKET_UPDATE_BAUD << 8), buffer, send_index);
}

bool comm_can_get_stats(unsigned bus, comm_can_stats_t *stats) {
	if (bus >= SOC_TWAI_CONTROLLER_NUM) {
		return false;
	}
	portENTER_CRITICAL(&stats_lock);
	*stats = buses[bus].stats;
	portEXIT_CRITICAL(&stats_lock);
	return true;
}

#if SOC_TWAI_CONTROLLER_NUM > 1
bool comm_can2_start(int pin_tx, int pin_rx, int baud_kbits) {
	return comm_can_start_bus(1, pin_tx, pin_rx, comm_can_kbits_to_baud(baud_kbits));
}

void comm_can2_stop(void) {
	comm_can_stop_bus(1);
}

void comm_can2_use_vesc_decoder(bool use) {
	portENTER_CRITICAL(&stats_lock);
	decoder_disabled[1] = !use;
	portEXIT_CRITICAL(&stats_lock);
}

void comm_can2_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	comm_can_transmit_bus(1, id, data, len, true);
}

void comm_can2_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) {
	comm_can_transmit_bus(1, id, data, len, false);
}

void comm_can2_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send_type) {
	comm_can_send_buffer_bus(1, controller_id, data, len, send_type);
}

#endif

#if SOC_TWAI_CONTROLLER_NUM > 2
bool comm_can3_start(int pin_tx, int pin_rx, int baud_kbits) {
	return comm_can_start_bus(2, pin_tx, pin_rx, comm_can_kbits_to_baud(baud_kbits));
}

void comm_can3_stop(void) {
	comm_can_stop_bus(2);
}

void comm_can3_use_vesc_decoder(bool use) {
	portENTER_CRITICAL(&stats_lock);
	decoder_disabled[2] = !use;
	portEXIT_CRITICAL(&stats_lock);
}

void comm_can3_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	comm_can_transmit_bus(2, id, data, len, true);
}

void comm_can3_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) {
	comm_can_transmit_bus(2, id, data, len, false);
}

void comm_can3_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send_type) {
	comm_can_send_buffer_bus(2, controller_id, data, len, send_type);
}

#endif
