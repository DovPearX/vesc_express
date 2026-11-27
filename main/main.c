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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "soc/soc_caps.h"

#include "conf_general.h"
#include "comm_uart.h"
#include "comm_usb.h"
#include "comm_can.h"
#include "commands.h"
#include "crc.h"
#include "flash_helper.h"
#include "lispif.h"

#ifdef OVR_CONF_XML_H
#include OVR_CONF_XML_H
#else
#include "confxml.h"
#endif

#ifdef OVR_CONF_PARSER_H
#include OVR_CONF_PARSER_H
#else
#include "confparser.h"
#endif

#include "terminal.h"
#include "main.h"
#include "mempools.h"

#include <string.h>
#include <stddef.h>
#include <stdlib.h>

// Global variables
volatile backup_data backup;

// Private variables
volatile static bool init_done = false;

void app_main(void) {
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase();
		nvs_flash_init();
	}

	flash_helper_init();

	{
		memset((void*)&backup, 0, sizeof(backup));
		nvs_handle_t my_handle;
		if (nvs_open("vesc", NVS_READONLY, &my_handle) == ESP_OK) {
			size_t size = 0;
			size_t can_size = offsetof(backup_data, config_init_flag);
			uint8_t *data = NULL;
			if (nvs_get_blob(my_handle, "backup", NULL, &size) == ESP_OK && size >= can_size) {
				data = malloc(size);
			}
			if (data) {
				if (nvs_get_blob(my_handle, "backup", data, &size) == ESP_OK) {
					memcpy((void*)&backup, data, size == sizeof(backup_data) ? size : can_size);
				}
				free(data);
			}
			nvs_close(my_handle);
		}

		if (backup.controller_id_init_flag != VAR_INIT_CODE) {
			backup.controller_id = CONF_CONTROLLER_ID;
			backup.controller_id_init_flag = VAR_INIT_CODE;
		}

		if (backup.can_baud_rate_init_flag != VAR_INIT_CODE) {
			backup.can_baud_rate = CONF_CAN_BAUD_RATE;
			backup.can_baud_rate_init_flag = VAR_INIT_CODE;
		}
		if (backup.config_init_flag != MAIN_CONFIG_T_SIGNATURE) {
#ifdef OVR_CONF_SET_DEFAULTS
			OVR_CONF_SET_DEFAULTS((main_config_t*)(&backup.config));
#else
			confparser_set_defaults_main_config_t((main_config_t*)(&backup.config));
#endif
			backup.config_init_flag = MAIN_CONFIG_T_SIGNATURE;
			backup.config.controller_id = backup.controller_id;
			backup.config.can_baud_rate = backup.can_baud_rate;
		}

	}

#ifdef HW_EARLY_LBM_INIT
	HW_INIT_HOOK();
	lispif_init();
	HW_POST_LISPIF_HOOK();
#endif

	mempools_init();
	commands_init();
#if defined(CAN_TX_GPIO_NUM) && defined(CAN_RX_GPIO_NUM)
	comm_can_start(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM);
#endif
	#if SOC_USB_SERIAL_JTAG_SUPPORTED
	comm_usb_init();
	#endif
#if CONFIG_TINYUSB_CDC_ENABLED
	comm_usb_cdc_init();
#endif

	vTaskDelay(1);

#ifndef HW_EARLY_LBM_INIT
	HW_INIT_HOOK();
	lispif_init();
	HW_POST_LISPIF_HOOK();
#endif

#if !defined(HW_NO_UART) && defined(HW_UART_COMM)
	comm_uart_init(UART_TX, UART_RX, UART_NUM, UART_BAUDRATE);
#endif

	init_done = true;

	// Exit main to free up heap-space
	vTaskDelete(NULL);
}

uint32_t main_calc_hw_crc(void) {
	return crc32_with_init(data_main_config_t_, DATA_MAIN_CONFIG_T__SIZE, 0);
}

void main_store_backup_data(void) {
	nvs_handle_t my_handle;
	backup.controller_id = backup.config.controller_id;
	backup.can_baud_rate = backup.config.can_baud_rate;
	nvs_open("vesc", NVS_READWRITE, &my_handle);
	nvs_set_blob(my_handle, "backup", (void*)&backup, sizeof(backup_data));
	nvs_commit(my_handle);
	nvs_close(my_handle);
}

bool main_init_done(void) {
	return init_done;
}

void main_wait_until_init_done(void) {
	while (!init_done) {
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}
