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

#include "commands.h"
#include "comm_usb.h"
#include "packet.h"
#include "soc/soc_caps.h"

#if SOC_USB_SERIAL_JTAG_SUPPORTED

#include "esp_log.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"

static PACKET_STATE_t packet_state;

static void rx_task(void *arg) {
	uint8_t buf[64];
	for (;;) {
		int bytes = usb_serial_jtag_read_bytes(buf, sizeof(buf), portMAX_DELAY);
		for (int i = 0; i < bytes; i++) {
			packet_process_byte(buf[i], &packet_state);
		}
	}
}

static void process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, comm_usb_send_packet);
}

static void send_packet_raw(unsigned char *buffer, unsigned int len) {
	unsigned int sent = 0;
	int fail_cnt = 0;

	while (sent < len) {
		int to_send = len - sent;
		if (to_send > 150) {
			to_send = 150;
		}

		int sent_now = usb_serial_jtag_write_bytes(buffer + sent, to_send, 10);
		if (sent_now <= 0) {
			fail_cnt++;
		} else {
			sent += sent_now;
			fail_cnt = 0;
		}

		if (fail_cnt >= 3) {
			break;
		}
	}
}

void comm_usb_init(void) {
	usb_serial_jtag_driver_config_t usb_serial_jtag_config;
	usb_serial_jtag_config.rx_buffer_size = 1024;
	usb_serial_jtag_config.tx_buffer_size = 256;
	esp_err_t result = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
	if (result != ESP_OK) {
		ESP_LOGE("usb_jtag", "Driver initialization failed: %s", esp_err_to_name(result));
		return;
	}

	packet_init(send_packet_raw, process_packet, &packet_state);

	if (xTaskCreate(rx_task, "usb_rx", 3072, NULL, 7, NULL) != pdPASS) {
		packet_state.tx_lock = NULL;
		usb_serial_jtag_driver_uninstall();
		ESP_LOGE("usb_jtag", "Failed to create RX task");
	}
}

void comm_usb_send_packet(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, &packet_state);
}
#endif

#if CONFIG_TINYUSB_CDC_ENABLED

#include <stdio.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/task.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_default_config.h"
#include "tusb.h"

enum {
	CDC_RX_DATA = 1U,
	CDC_RX_RESET = 2U,
};

static PACKET_STATE_t cdc_packet_state;
static TaskHandle_t cdc_rx_task;

static void cdc_send_raw(unsigned char *data, unsigned int len) {
	const TickType_t started = xTaskGetTickCount();
	const TickType_t timeout = pdMS_TO_TICKS(20) + 1;

	while (tud_ready() && tud_cdc_n_write_available(TINYUSB_CDC_ACM_0) < len) {
		if (xTaskGetTickCount() - started >= timeout) {
			return;
		}
		vTaskDelay(1);
	}

	if (tud_ready() && tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, data, len) > 0) {
		tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
	}
}

static void cdc_send_packet(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, &cdc_packet_state);
}

static void cdc_process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, cdc_send_packet);
}

static void cdc_on_rx(int itf, cdcacm_event_t *event) {
	(void)itf;
	(void)event;
	xTaskNotify(cdc_rx_task, CDC_RX_DATA, eSetBits);
}

static void cdc_on_event(tinyusb_event_t *event, void *arg) {
	(void)arg;
	if (event->id == TINYUSB_EVENT_DETACHED) {
		xTaskNotify(cdc_rx_task, CDC_RX_RESET, eSetBits);
	}
}

static void cdc_receive(void *arg) {
	(void)arg;
	uint8_t data[512];
	for (;;) {
		uint32_t events;
		xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);
		if (events & CDC_RX_RESET) {
			packet_reset(&cdc_packet_state);
		}
		if (!(events & CDC_RX_DATA)) {
			continue;
		}

		size_t len;
		while (tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, data, sizeof(data), &len) == ESP_OK && len > 0) {
			for (size_t i = 0; i < len; i++) {
				packet_process_byte(data[i], &cdc_packet_state);
			}
		}
	}
}

void comm_usb_cdc_init(void) {
	uint8_t mac[6];
	static char serial_number[sizeof("HS") + 2 * sizeof(mac)];
	static const char english_us[] = {0x09, 0x04};
	static const char *descriptors[] = {
		english_us,
		CONFIG_TINYUSB_DESC_MANUFACTURER_STRING,
		CONFIG_TINYUSB_DESC_PRODUCT_STRING,
		serial_number,
		CONFIG_TINYUSB_DESC_CDC_STRING,
	};

	if (esp_read_mac(mac, ESP_MAC_BASE) != ESP_OK) {
		ESP_LOGE("usb_cdc", "Failed to read the USB serial number");
		return;
	}
	snprintf(serial_number, sizeof(serial_number), "HS%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	packet_init(cdc_send_raw, cdc_process_packet, &cdc_packet_state);
	if (xTaskCreate(cdc_receive, "usb_cdc_rx", 4096, NULL, 7, &cdc_rx_task) != pdPASS) {
		ESP_LOGE("usb_cdc", "Failed to create RX task");
		return;
	}

	tinyusb_config_t usb = TINYUSB_DEFAULT_CONFIG(cdc_on_event);
	usb.descriptor.string = descriptors;
	usb.descriptor.string_count = sizeof(descriptors) / sizeof(descriptors[0]);
	const tinyusb_config_cdcacm_t cdc = {
		.cdc_port = TINYUSB_CDC_ACM_0,
		.callback_rx = cdc_on_rx,
	};

	esp_err_t result = tinyusb_cdcacm_init(&cdc);
	if (result == ESP_OK) {
		result = tinyusb_driver_install(&usb);
		if (result != ESP_OK) {
			tinyusb_cdcacm_deinit(TINYUSB_CDC_ACM_0);
		}
	}
	if (result == ESP_OK) {
		return;
	}

	ESP_LOGE("usb_cdc", "CDC initialization failed: %s", esp_err_to_name(result));
	vTaskDelete(cdc_rx_task);
	cdc_rx_task = NULL;
}

#endif
