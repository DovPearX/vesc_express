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

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "comm_usb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "commands.h"
#include "datatypes.h"
#include "conf_general.h"
#include "comm_can.h"
#include "terminal.h"
#include "mempools.h"
#include "utils.h"

#ifdef OVR_CONF_PARSER_H
#include OVR_CONF_PARSER_H
#else
#include "confparser.h"
#endif

#ifdef OVR_CONF_XML_H
#include OVR_CONF_XML_H
#else
#include "confxml.h"
#endif

#include "packet.h"
#include "buffer.h"
#include "main.h"

#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "soc/rtc.h"

// Settings
#define PRINT_BUFFER_SIZE	400

// For double precision literals
#define D(x) 						((double)x##L)

// Private variables
static const esp_partition_t *update_partition = NULL;
static esp_ota_handle_t update_handle = 0;

typedef struct {
	send_func_t serial;
	bool can;
	unsigned bus;
	uint8_t sender;
	uint8_t response_type;
} reply_route_t;

typedef enum {
	COMMAND_JOB_PACKET,
	COMMAND_JOB_FORWARD,
	COMMAND_JOB_CAN_BAUD,
} command_job_type_t;

typedef struct {
	reply_route_t route;
	command_job_type_t type;
	CAN_BAUD baud;
	CAN_BAUD previous_baud;
	TickType_t queued_at;
	int delay_ms;
	unsigned int len;
	uint8_t payload[PACKET_MAX_PL_LEN + 1];
} command_job_t;

#define COMMAND_QUEUE_DEPTH 4

static QueueHandle_t command_queue;
static TaskHandle_t command_worker;
static reply_route_t worker_route;
static portMUX_TYPE route_lock = portMUX_INITIALIZER_UNLOCKED;
static reply_route_t last_route;
static reply_route_t forward_routes[SOC_TWAI_CONTROLLER_NUM];
static uint32_t dropped_commands;

static void send_reply(reply_route_t route, unsigned char *data, unsigned int len) {
	if (route.can) {
		comm_can_send_buffer_bus(route.bus, route.sender, data, len, route.response_type);
	} else if (route.serial) {
		route.serial(data, len);
	}
}

static void queue_job(command_job_t *job) {
	if (command_queue && xQueueSend(command_queue, job, 0) == pdTRUE) {
		return;
	}

	portENTER_CRITICAL(&route_lock);
	dropped_commands++;
	portEXIT_CRITICAL(&route_lock);
}

static void queue_packet(const unsigned char *data, unsigned int len, reply_route_t route, command_job_type_t type) {
	if (!len || len > PACKET_MAX_PL_LEN) {
		return;
	}
	command_job_t job = {.route = route, .type = type, .len = len};
	memcpy(job.payload, data, len);
	queue_job(&job);
}

static void process_packet(unsigned char *data, unsigned int len, reply_route_t route) {
	if (!len) {
		return;
	}

	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;

	portENTER_CRITICAL(&route_lock);
	last_route = route;

	if (!forward_routes[route.bus].serial && !forward_routes[route.bus].can) {
		forward_routes[route.bus] = route;
	}
	portEXIT_CRITICAL(&route_lock);

	switch (packet_id) {
	case COMM_FW_VERSION: {
		int32_t ind = 0;
		uint8_t send_buffer[65];
		send_buffer[ind++] = COMM_FW_VERSION;
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;

		strcpy((char*)(send_buffer + ind), HW_NAME);
		ind += strlen(HW_NAME) + 1;

		size_t size_bits = esp_efuse_get_field_size(ESP_EFUSE_MAC_FACTORY);
	    esp_efuse_read_field_blob(ESP_EFUSE_MAC_FACTORY, send_buffer + ind, size_bits);
	    ind += 6;
		memset(send_buffer + ind, 0, 6);
		ind += 6;

		send_buffer[ind++] = 0;
		send_buffer[ind++] = FW_TEST_VERSION_NUMBER;

		send_buffer[ind++] = HW_TYPE_CUSTOM_MODULE;
		send_buffer[ind++] = 1;

		send_buffer[ind++] = 0; // No phase filters
		send_buffer[ind++] = 0; // No HW QML

		send_buffer[ind++] = 0; // No application QML

		send_buffer[ind++] = 0; // No NRF flags

		strcpy((char *)send_buffer + ind, FW_NAME);
		ind += strlen(FW_NAME) + 1;

		buffer_append_uint32(send_buffer, main_calc_hw_crc(), &ind);

		send_reply(route, send_buffer, ind);
	} break;

	// TODO: Run crc check on new app, also make sure to skip duplicate packets
	case COMM_JUMP_TO_BOOTLOADER:
		if (update_handle != 0) {
			if (esp_ota_end(update_handle) == ESP_OK) {
				if (esp_ota_set_boot_partition(update_partition) == ESP_OK) {
					vTaskDelay(50 / portTICK_PERIOD_MS);

					// Here we must use esp_restart even though that does not play nicely
					// with USB. That is because we skip image validation in the bootloader
					// after deep sleep to get a faster boot time, allowing less power draw
					// in applications that need to wake up from deep sleep occasionally.
					esp_restart();
//					esp_sleep_enable_timer_wakeup(1000000);
//					esp_deep_sleep_start();
				}
			}
		}
		break;

	case COMM_ERASE_NEW_APP: {
		int32_t ind = 0;

		if (update_handle != 0) {
			esp_ota_abort(update_handle);
			update_handle = 0;
		}

		update_partition = esp_ota_get_next_update_partition(NULL);
		bool ok = false;
		if (update_partition != NULL) {
			esp_err_t res = esp_ota_begin(update_partition, buffer_get_uint32(data, &ind) - 6, &update_handle);
			ok = res == ESP_OK;
		}

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_ERASE_NEW_APP;
		send_buffer[ind++] = ok;
		send_reply(route, send_buffer, ind);
	} break;

	case COMM_WRITE_NEW_APP_DATA: {
		int32_t ind = 0;
		uint32_t new_app_offset = buffer_get_uint32(data, &ind);

		if (new_app_offset < 6) {
			ind += (6 - new_app_offset); // Skip size and crc
			new_app_offset = 0;
		} else {
			new_app_offset -= 6;
		}

		bool ok = false;
		if (update_handle != 0) {
			esp_err_t res = esp_ota_write_with_offset(update_handle, data + ind, len - ind, new_app_offset);
			ok = res == ESP_OK;
		}

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_WRITE_NEW_APP_DATA;
		send_buffer[ind++] = ok;
		buffer_append_uint32(send_buffer, new_app_offset, &ind);
		send_reply(route, send_buffer, ind);
	} break;

	case COMM_REBOOT: {
		// Deep sleep to reboot as that disconnects USB properly
//		esp_restart();
		esp_sleep_enable_timer_wakeup(1000000);
		esp_deep_sleep_start();
	} break;

	case COMM_FORWARD_CAN:
		if (!len) {
			break;
		}
		portENTER_CRITICAL(&route_lock);
		forward_routes[route.bus] = route;
		portEXIT_CRITICAL(&route_lock);
		comm_can_send_buffer_bus(route.bus, data[0], data + 1, len - 1, 0);
		break;

	case COMM_CAN_FWD_FRAME: {
		if (len < 5) {
			break;
		}
		int32_t ind = 0;
		uint32_t id = buffer_get_uint32(data, &ind);
		bool is_ext = data[ind++];

		comm_can_transmit_bus(route.bus, id, data + ind, len - ind, is_ext);
	} break;

	case COMM_GET_CUSTOM_CONFIG:
	case COMM_GET_CUSTOM_CONFIG_DEFAULT: {
		main_config_t *conf = calloc(1, sizeof(main_config_t));

		int conf_ind = data[0];

		if (conf_ind != 0) {
			free(conf);
			break;
		}

		if (packet_id == COMM_GET_CUSTOM_CONFIG) {
			*conf = backup.config;
		} else {
#ifdef OVR_CONF_SET_DEFAULTS
			OVR_CONF_SET_DEFAULTS(conf);
#else
			confparser_set_defaults_main_config_t(conf);
#endif
		}

		uint8_t *send_buffer_global = mempools_get_packet_buffer();
		int32_t ind = 0;
		send_buffer_global[ind++] = packet_id;
		send_buffer_global[ind++] = conf_ind;
#ifdef OVR_CONF_SERIALIZE
		int32_t len = OVR_CONF_SERIALIZE(send_buffer_global + ind, conf);
#else
		int32_t len = confparser_serialize_main_config_t(send_buffer_global + ind, conf);
#endif
		send_reply(route, send_buffer_global, len + ind);
		mempools_free_packet_buffer(send_buffer_global);

		free(conf);
	} break;

	case COMM_SET_CUSTOM_CONFIG: {
		main_config_t *conf = calloc(1, sizeof(main_config_t));
		*conf = backup.config;

		int conf_ind = data[0];

		if (conf_ind != 0) {
			free(conf);
			break;
		}

#ifdef OVR_CONF_DESERIALIZE
		if (conf_ind == 0 && OVR_CONF_DESERIALIZE(data + 1, conf)) {
#else
		if (conf_ind == 0 && confparser_deserialize_main_config_t(data + 1, conf)) {
#endif
			bool baud_changed = backup.config.can_baud_rate != conf->can_baud_rate;
			backup.config = *conf;

			if (baud_changed) {
				comm_can_update_baudrate(0);
			}

			main_store_backup_data();

			int32_t ind = 0;
			uint8_t send_buffer[50];
			send_buffer[ind++] = packet_id;
			send_reply(route, send_buffer, ind);
		} else {
			uint8_t warning[] = "\0Warning: Could not set configuration";
			warning[0] = COMM_PRINT;
			send_reply(route, warning, sizeof(warning) - 1);
		}

		free(conf);
	} break;

	case COMM_GET_CUSTOM_CONFIG_XML: {
		int32_t ind = 0;

		int conf_ind = data[ind++];

		if (conf_ind != 0) {
			break;
		}

		int32_t len_conf = buffer_get_int32(data, &ind);
		int32_t ofs_conf = buffer_get_int32(data, &ind);

		if ((len_conf + ofs_conf) > DATA_MAIN_CONFIG_T__SIZE || len_conf > (PACKET_MAX_PL_LEN - 10)) {
			break;
		}

		uint8_t *send_buffer_global = mempools_get_packet_buffer();
		ind = 0;
		send_buffer_global[ind++] = packet_id;
		send_buffer_global[ind++] = conf_ind;
		buffer_append_int32(send_buffer_global, DATA_MAIN_CONFIG_T__SIZE, &ind);
		buffer_append_int32(send_buffer_global, ofs_conf, &ind);
		memcpy(send_buffer_global + ind, data_main_config_t_ + ofs_conf, len_conf);
		ind += len_conf;
		send_reply(route, send_buffer_global, ind);
		mempools_free_packet_buffer(send_buffer_global);
	} break;

	case COMM_FW_INFO: {
		// Write at most the first max_len characters of str into buffer,
		// followed by a null byte.
		void buffer_append_str_max_len(uint8_t *buffer, char *str, size_t max_len, int32_t *index) {
			size_t str_len = strlen(str);
			if (str_len > max_len) {
				str_len = max_len;
				return;
			}

			memcpy(&buffer[*index], str, str_len);
			*index += str_len;
			buffer[(*index)++] = '\0';
		}

		int32_t ind = 0;
		uint8_t send_buffer[98];

		send_buffer[ind++] = COMM_FW_INFO;

		// This information is technically duplicated with COMM_FW_VERSION, but
		// I don't care.
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;
		send_buffer[ind++] = FW_TEST_VERSION_NUMBER;

		// We don't include the branch name unfortunately
		buffer_append_str_max_len(send_buffer, GIT_COMMIT_HASH, 46, &ind);
#ifdef USER_GIT_COMMIT_HASH
		char *user_commit_hash = USER_GIT_COMMIT_HASH;
#else
		char *user_commit_hash = "";
#endif
		buffer_append_str_max_len(send_buffer, user_commit_hash, 46, &ind);

		send_reply(route, send_buffer, ind);
	} break;

	// Blocking commands
	case COMM_TERMINAL_CMD:
	case COMM_PING_CAN:
	case COMM_CAN_UPDATE_BAUD_ALL:
		queue_packet(data - 1, len + 1, route, COMMAND_JOB_PACKET);
		break;

	default:
		break;
	}
}

static void block_task(void *arg) {
	(void)arg;
	static command_job_t job;
	CAN_BAUD pending_baud = CAN_BAUD_INVALID;
	CAN_BAUD original_baud = CAN_BAUD_INVALID;
	TickType_t queued_at = 0;
	TickType_t delay = 0;

	for (;;) {
		TickType_t wait = portMAX_DELAY;
		if (pending_baud != CAN_BAUD_INVALID) {
			TickType_t elapsed = xTaskGetTickCount() - queued_at;
			wait = elapsed < delay ? delay - elapsed : 0;
			if (wait == 0) {
				if (backup.config.can_baud_rate == original_baud) {
					backup.config.can_baud_rate = pending_baud;
					main_store_backup_data();
					comm_can_update_baudrate(0);
				}
				pending_baud = CAN_BAUD_INVALID;
			}
		}
		if (xQueueReceive(command_queue, &job, wait) != pdTRUE) {
			continue;
		}
		worker_route = job.route;
		if (job.type == COMMAND_JOB_FORWARD) {
			send_reply(job.route, job.payload, job.len);
			continue;
		}
		if (job.type == COMMAND_JOB_CAN_BAUD) {
			CAN_BAUD current_baud = backup.config.can_baud_rate;
			if (job.previous_baud != current_baud) {
				continue;
			}
			if (job.baud == current_baud) {
				pending_baud = CAN_BAUD_INVALID;
			} else if (job.baud != pending_baud) {
				pending_baud = job.baud;
				original_baud = current_baud;
				queued_at = job.queued_at;
				delay = job.delay_ms > 0 ? pdMS_TO_TICKS(job.delay_ms) : 0;
			}
			continue;
		}

		COMM_PACKET_ID packet_id = job.payload[0];
		uint8_t *data = job.payload + 1;
		unsigned int len = job.len - 1;
		static uint8_t send_buffer[256];

		switch (packet_id) {
		case COMM_PING_CAN: {
			int32_t ind = 0;
			send_buffer[ind++] = COMM_PING_CAN;

			for (uint8_t i = 0; i < 255; i++) {
				HW_TYPE hw_type;
				if (comm_can_ping_bus(job.route.bus, i, &hw_type)) {
					send_buffer[ind++] = i;
				}
			}

			send_reply(job.route, send_buffer, ind);
		} break;

		case COMM_TERMINAL_CMD:
			data[len] = '\0';
			terminal_process_string((char *)data);
			break;

		case COMM_CAN_UPDATE_BAUD_ALL: {
			if (len < 4) {
				break;
			}
			int32_t ind = 0;
			uint32_t kbits = buffer_get_int16(data, &ind);
			uint32_t delay_msec = buffer_get_int16(data, &ind);
			CAN_BAUD baud = comm_can_kbits_to_baud(kbits);
			if (baud != CAN_BAUD_INVALID) {
				for (int i = 0; i < 10; i++) {
					comm_can_send_update_baud(kbits, delay_msec);
					vTaskDelay(50 / portTICK_PERIOD_MS);
				}

				backup.config.can_baud_rate = baud;
				main_store_backup_data();
				comm_can_update_baudrate(delay_msec);
			}

			ind = 0;
			send_buffer[ind++] = packet_id;
			send_buffer[ind++] = baud != CAN_BAUD_INVALID;
			send_reply(job.route, send_buffer, ind);
		} break;

		default:
			process_packet(job.payload, job.len, job.route);
			break;
		}
	}
}

void commands_init(void) {
	command_queue = xQueueCreate(COMMAND_QUEUE_DEPTH, sizeof(command_job_t));
	if (!command_queue) {
		ESP_LOGE("commands", "Failed to create command queue");
		return;
	}
	xTaskCreate(block_task, "comm_block", 4096, NULL, 7, &command_worker);
	if (!command_worker) {
		ESP_LOGE("commands", "Failed to create command task");
		vQueueDelete(command_queue);
		command_queue = NULL;
	}
}

void commands_process_packet(unsigned char *data, unsigned int len, send_func_t reply_func) {
	process_packet(data, len, (reply_route_t){.serial = reply_func});
}

void commands_process_can_packet(unsigned char *data, unsigned int len, unsigned bus, uint8_t sender, uint8_t response_type) {
	reply_route_t route = {
		.can = response_type != 2,
		.bus = bus,
		.sender = sender,
		.response_type = response_type,
	};
	queue_packet(data, len, route, COMMAND_JOB_PACKET);
}

void commands_queue_can_forward(unsigned bus, unsigned char *data, unsigned int len) {
	portENTER_CRITICAL(&route_lock);
	reply_route_t route = forward_routes[bus];
	portEXIT_CRITICAL(&route_lock);
	queue_packet(data, len, route, COMMAND_JOB_FORWARD);
}

void commands_queue_can_baud(CAN_BAUD baud, int delay_ms) {
	command_job_t job = {
		.type = COMMAND_JOB_CAN_BAUD,
		.baud = baud,
		.previous_baud = backup.config.can_baud_rate,
		.delay_ms = delay_ms,
		.queued_at = xTaskGetTickCount(),
	};
	queue_job(&job);
}

uint32_t commands_get_dropped_count(void) {
	portENTER_CRITICAL(&route_lock);
	uint32_t count = dropped_commands;
	portEXIT_CRITICAL(&route_lock);
	return count;
}

/**
 * Send a packet using the last can fwd function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void commands_send_packet_can_last(unsigned char *data, unsigned int len) {
	portENTER_CRITICAL(&route_lock);
	reply_route_t route = forward_routes[0];
	portEXIT_CRITICAL(&route_lock);
	send_reply(route, data, len);
}

void commands_send_packet(unsigned char *data, unsigned int len) {
	if (command_worker && xTaskGetCurrentTaskHandle() == command_worker) {
		send_reply(worker_route, data, len);
		return;
	}

	portENTER_CRITICAL(&route_lock);
	reply_route_t route = last_route;
	portEXIT_CRITICAL(&route_lock);
	send_reply(route, data, len);
}

int commands_printf(const char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;

	char print_buffer[PRINT_BUFFER_SIZE];

	print_buffer[0] = COMM_PRINT;
	len = vsnprintf(print_buffer + 1, (PRINT_BUFFER_SIZE - 1), format, arg);
	va_end (arg);

	int len_to_print = (len < (PRINT_BUFFER_SIZE - 1)) ? len + 1 : PRINT_BUFFER_SIZE;

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer, len_to_print);
	}

	return len_to_print - 1;
}