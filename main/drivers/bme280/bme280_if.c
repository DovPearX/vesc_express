/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se

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

#include "bme280.h"
#include "bme280_if.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include <string.h>

// Private variables
static float m_last_temp = 0.0;
static float m_last_hum = 0.0;
static float m_last_pres = 0.0;

static volatile bool mutex_init = false;
static SemaphoreHandle_t i2c_mutex;
static i2c_master_bus_handle_t bme_i2c_bus = NULL;
static i2c_master_dev_handle_t bme_i2c_dev = NULL;

static esp_err_t bme280_open_device(bool create_bus, int pin_sda, int pin_scl) {
	if (bme_i2c_dev) {
		return ESP_OK;
	}

	if (!bme_i2c_bus) {
		if (create_bus) {
			i2c_master_bus_config_t bus_cfg = {
				.i2c_port = I2C_NUM_0,
				.sda_io_num = pin_sda,
				.scl_io_num = pin_scl,
				.clk_source = I2C_CLK_SRC_DEFAULT,
				.glitch_ignore_cnt = 7,
				.intr_priority = 0,
				.trans_queue_depth = 4,
				.flags.enable_internal_pullup = 1,
			};
			esp_err_t bus_res = i2c_new_master_bus(&bus_cfg, &bme_i2c_bus);
			if (bus_res != ESP_OK) {
				return bus_res;
			}
		} else {
			if (i2c_master_get_bus_handle(I2C_NUM_0, &bme_i2c_bus) != ESP_OK || !bme_i2c_bus) {
				return ESP_ERR_INVALID_STATE;
			}
		}
	}

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = BME280_I2C_ADDR_PRIM,
		.scl_speed_hz = 100000,
		.scl_wait_us = 0,
	};
	return i2c_master_bus_add_device(bme_i2c_bus, &dev_cfg, &bme_i2c_dev);
}

// Private functions
static void bme_task(void *arg);

void bme280_if_init(int pin_sda, int pin_scl) {
	bme280_open_device(true, pin_sda, pin_scl);

	xTaskCreatePinnedToCore(bme_task, "BME280", 1536, NULL, 6, NULL, tskNO_AFFINITY);
}

void bme280_if_init_with_mutex(SemaphoreHandle_t mutex) {
	mutex_init = true;
	i2c_mutex = mutex;
	bme280_open_device(false, 0, 0);
	xTaskCreatePinnedToCore(bme_task, "BME280", 1536, NULL, 6, NULL, tskNO_AFFINITY);
}

float bme280_if_get_hum(void) {
	return m_last_hum;
}

float bme280_if_get_temp(void) {
	return m_last_temp;
}

float bme280_if_get_pres(void) {
	return m_last_pres;
}

static void user_delay_us(uint32_t period, void *intf_ptr) {
	(void)intf_ptr;
	vTaskDelay(period / (1000 * portTICK_PERIOD_MS));
}

static int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	(void)intf_ptr;

	uint8_t txbuf[1];
	txbuf[0] = reg_addr;

	if (mutex_init) {
		xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	}

	esp_err_t res = bme_i2c_dev ? i2c_master_transmit_receive(bme_i2c_dev, txbuf, 1, reg_data, len, 1000 / portTICK_PERIOD_MS) : ESP_ERR_INVALID_STATE;

	if (mutex_init) {
		xSemaphoreGive(i2c_mutex);
	}

	return res == ESP_OK ? 0 : -1; // Return 0 for Success, non-zero for failure
}

static int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	(void)intf_ptr;

	uint8_t txbuf[len + 1];
	txbuf[0] = reg_addr;
	memcpy(txbuf + 1, reg_data, len);

	if (mutex_init) {
		xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	}

	esp_err_t res = bme_i2c_dev ? i2c_master_transmit(bme_i2c_dev, txbuf, len + 1, 1000 / portTICK_PERIOD_MS) : ESP_ERR_INVALID_STATE;

	if (mutex_init) {
		xSemaphoreGive(i2c_mutex);
	}

	return res == ESP_OK ? 0 : -1; // Return 0 for Success, non-zero for failure
}

static void bme_task(void *arg) {
	struct bme280_dev dev;
	uint8_t dev_addr = BME280_I2C_ADDR_PRIM;

	dev.intf_ptr = &dev_addr;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_us = user_delay_us;

	bme280_init(&dev);

	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;

	uint8_t settings_sel;
	uint32_t req_delay;
	struct bme280_data comp_data;

	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
	bme280_set_sensor_settings(settings_sel, &dev);
	req_delay = bme280_cal_meas_delay(&dev.settings);

	for(;;) {
		bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
		vTaskDelay(req_delay / portTICK_PERIOD_MS);

		bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
		m_last_hum = comp_data.humidity;
		m_last_temp = comp_data.temperature;
		m_last_pres = comp_data.pressure;
	}
}
