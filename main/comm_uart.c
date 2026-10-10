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

#include "conf_general.h"
#include "commands.h"
#include "comm_uart.h"
#include "packet.h"
#include "driver/uart.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
	int uart_num;
	PACKET_STATE_t packet_state;
	bool should_stop;
	bool is_running;
} uart_state;

static uart_state *m_state[UART_NUM_MAX] = {0};
static StaticSemaphore_t m_init_mutex_buffer;
static StaticSemaphore_t m_send_mutex_buffer;
static SemaphoreHandle_t m_init_mutex;
static SemaphoreHandle_t m_send_mutex;
static portMUX_TYPE m_mutex_lock = portMUX_INITIALIZER_UNLOCKED;

static void init_mutexes(void) {
	taskENTER_CRITICAL(&m_mutex_lock);
	if (!m_init_mutex) {
		m_init_mutex = xSemaphoreCreateMutexStatic(&m_init_mutex_buffer);
		m_send_mutex = xSemaphoreCreateMutexStatic(&m_send_mutex_buffer);
	}
	taskEXIT_CRITICAL(&m_mutex_lock);
}

static void stop_uart(int uart_num) {
	xSemaphoreTake(m_send_mutex, portMAX_DELAY);
	uart_state *state = m_state[uart_num];
	if (state) {
		m_state[uart_num] = NULL;
		taskENTER_CRITICAL(&m_mutex_lock);
		state->should_stop = true;
		taskEXIT_CRITICAL(&m_mutex_lock);
	}
	xSemaphoreGive(m_send_mutex);

	if (state) {
		for (;;) {
			taskENTER_CRITICAL(&m_mutex_lock);
			bool running = state->is_running;
			taskEXIT_CRITICAL(&m_mutex_lock);
			if (!running) {
				break;
			}
			vTaskDelay(1);
		}

		free(state);
	}

	if (uart_is_driver_installed(uart_num)) {
		uart_driver_delete(uart_num);
	}
}

static void rx_task(void *arg) {
	uart_state *state = (uart_state *)arg;

	for (;;) {
		taskENTER_CRITICAL(&m_mutex_lock);
		bool stop = state->should_stop;
		taskEXIT_CRITICAL(&m_mutex_lock);
		if (stop) {
			break;
		}

		uint8_t buf[64];
		int bytes = uart_read_bytes(state->uart_num, buf, 1, 3);
		if (bytes == 1) {
			int pending = uart_read_bytes(state->uart_num, buf + 1, sizeof(buf) - 1, 0);
			if (pending > 0) {
				bytes += pending;
			}
		}
		for (int i = 0; i < bytes; i++) {
			packet_process_byte(buf[i], &(state->packet_state));
		}

		if (!uart_is_driver_installed(state->uart_num)) {
			break;
		}
	}

	xSemaphoreTake(m_send_mutex, portMAX_DELAY);
	if (m_state[state->uart_num] == state) {
		m_state[state->uart_num] = NULL;
		free(state);
	} else {
		taskENTER_CRITICAL(&m_mutex_lock);
		state->is_running = false;
		taskEXIT_CRITICAL(&m_mutex_lock);
	}
	xSemaphoreGive(m_send_mutex);
	vTaskDelete(NULL);
}

static void send_packet_u0(unsigned char *data, unsigned int len) {
	comm_uart_send_packet(data, len, 0);
}

static void send_packet_u1(unsigned char *data, unsigned int len) {
	comm_uart_send_packet(data, len, 1);
}

static void process_packet_u0(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, send_packet_u0);
}

static void process_packet_u1(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, send_packet_u1);
}

static void send_packet_raw_u0(unsigned char *buffer, unsigned int len) {
	uart_write_bytes(0, buffer, len);
}

static void send_packet_raw_u1(unsigned char *buffer, unsigned int len) {
	uart_write_bytes(1, buffer, len);
}

bool comm_uart_init(int pin_tx, int pin_rx, int uart_num, int baudrate) {
	if (uart_num < 0 || uart_num >= UART_NUM_MAX) {
		return false;
	}

	init_mutexes();
	xSemaphoreTake(m_init_mutex, portMAX_DELAY);
	stop_uart(uart_num);

	uart_state *state;
	state = malloc(sizeof(uart_state));
	if (!state) {
		xSemaphoreGive(m_init_mutex);
		return false;
	}
	memset(state, 0, sizeof(uart_state));
	state->uart_num = uart_num;
	state->is_running = true;

	uart_config_t uart_config = {
		.baud_rate = baudrate,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	if (uart_driver_install(uart_num, 512, 512, 0, 0, 0) != ESP_OK) {
		free(state);
		xSemaphoreGive(m_init_mutex);
		return false;
	}
	if (uart_param_config(uart_num, &uart_config) != ESP_OK
		|| uart_set_pin(uart_num, pin_tx, pin_rx, -1, -1) != ESP_OK) {
		goto error;
	}

	if (uart_num == 0) {
		packet_init(send_packet_raw_u0, process_packet_u0, &(state->packet_state));
	} else {
		packet_init(send_packet_raw_u1, process_packet_u1, &(state->packet_state));
	}

	xSemaphoreTake(m_send_mutex, portMAX_DELAY);
	m_state[uart_num] = state;
	BaseType_t res = xTaskCreatePinnedToCore(rx_task, "uart_rx", 3072, state, 8, NULL, tskNO_AFFINITY);
	if (res != pdPASS) {
		m_state[uart_num] = NULL;
	}
	xSemaphoreGive(m_send_mutex);
	if (res != pdPASS) {
		goto error;
	}

	xSemaphoreGive(m_init_mutex);
	return true;

error:
	uart_driver_delete(uart_num);
	free(state);
	xSemaphoreGive(m_init_mutex);
	return false;
}

void comm_uart_stop(int uart_num) {
	if (uart_num < 0 || uart_num >= UART_NUM_MAX) {
		return;
	}

	init_mutexes();
	xSemaphoreTake(m_init_mutex, portMAX_DELAY);
	stop_uart(uart_num);
	xSemaphoreGive(m_init_mutex);
}

void comm_uart_send_packet(unsigned char *data, unsigned int len, int uart_num) {
	if (uart_num < 0 || uart_num >= UART_NUM_MAX) {
		return;
	}

	init_mutexes();
	xSemaphoreTake(m_send_mutex, portMAX_DELAY);
	if (m_state[uart_num]) {
		packet_send_packet(data, len, &(m_state[uart_num]->packet_state));
	}
	xSemaphoreGive(m_send_mutex);
}
