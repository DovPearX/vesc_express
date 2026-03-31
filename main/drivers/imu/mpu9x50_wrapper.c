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

#include "mpu9x50_wrapper.h"
#include "commands.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MPU9X50_ADDR_0 0x68
#define MPU9X50_ADDR_1 0x69
#define MPU9X50_REG_WHO_AM_I 0x75
#define MPU9X50_REG_PWR_MGMT_1 0x6B
#define MPU9X50_REG_SMPLRT_DIV 0x19
#define MPU9X50_REG_CONFIG 0x1A
#define MPU9X50_REG_GYRO_CONFIG 0x1B
#define MPU9X50_REG_ACCEL_CONFIG 0x1C
#define MPU9X50_REG_ACCEL_XOUT_H 0x3B

static uint8_t m_i2c_addr = MPU9X50_ADDR_0;
static int m_rate_hz = 200;
static IMU_FILTER m_filter = IMU_FILTER_MEDIUM;
static volatile bool m_thd_running = false;
static volatile bool m_should_terminate = false;
static void (*m_read_callback)(float *accel, float *gyro, float *mag) = NULL;

static float m_accel_scale = (1.0f / 16384.0f);
static float m_gyro_scale = (1.0f / 131.0f);

extern bool imu_i2c_tx_rx(uint8_t addr, const uint8_t* write_buffer, size_t write_size,
		uint8_t* read_buffer, size_t read_size);

static int8_t mpu9x50_platform_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	uint8_t txb[16];

	if (len > (sizeof(txb) - 1U)) {
		return -1;
	}

	txb[0] = reg_addr;
	if (len > 0U) {
		memcpy(&txb[1], data, len);
	}

	return imu_i2c_tx_rx(dev_addr, txb, len + 1U, 0, 0) ? 0 : -1;
}

static int8_t mpu9x50_platform_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	return imu_i2c_tx_rx(dev_addr, &reg_addr, 1, data, len) ? 0 : -1;
}

static void mpu9x50_convert_accel(int16_t *raw, float *accel) {
	accel[0] = (float)raw[0] * m_accel_scale;
	accel[1] = (float)raw[1] * m_accel_scale;
	accel[2] = (float)raw[2] * m_accel_scale;
}

static void mpu9x50_convert_gyro(int16_t *raw, float *gyro) {
	gyro[0] = (float)raw[0] * m_gyro_scale;
	gyro[1] = (float)raw[1] * m_gyro_scale;
	gyro[2] = (float)raw[2] * m_gyro_scale;
}

static bool mpu9x50_probe_addr(uint8_t addr, uint8_t *who_am_i) {
	if (mpu9x50_platform_read(addr, MPU9X50_REG_WHO_AM_I, who_am_i, 1) != 0) {
		return false;
	}

	return *who_am_i == 0x68 || *who_am_i == 0x70 || *who_am_i == 0x71;
}

static void mpu9x50_task(void *arg) {
	(void)arg;
	bool initialized = false;
	uint8_t who_am_i = 0;

	for (int i = 0; i < 5; i++) {
		if (mpu9x50_probe_addr(MPU9X50_ADDR_0, &who_am_i)) {
			m_i2c_addr = MPU9X50_ADDR_0;
			initialized = true;
			break;
		}

		if (mpu9x50_probe_addr(MPU9X50_ADDR_1, &who_am_i)) {
			m_i2c_addr = MPU9X50_ADDR_1;
			initialized = true;
			break;
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}

	if (!initialized) {
		commands_printf("MPU9X50: init failed, sensor not found");
		m_thd_running = false;
		vTaskDelete(NULL);
		return;
	}

	commands_printf("MPU9X50: found WHO_AM_I=0x%02x addr=0x%02x", who_am_i, m_i2c_addr);

	uint8_t pwr_mgmt = 0x01;
	(void)mpu9x50_platform_write(m_i2c_addr, MPU9X50_REG_PWR_MGMT_1, &pwr_mgmt, 1);

	uint8_t gyro_config = 0x00;
	(void)mpu9x50_platform_write(m_i2c_addr, MPU9X50_REG_GYRO_CONFIG, &gyro_config, 1);
	m_gyro_scale = (1.0f / 131.0f);

	uint8_t accel_config = 0x00;
	(void)mpu9x50_platform_write(m_i2c_addr, MPU9X50_REG_ACCEL_CONFIG, &accel_config, 1);
	m_accel_scale = (1.0f / 16384.0f);

	uint8_t dlpf = 0;
	switch (m_filter) {
	case IMU_FILTER_LOW:
		dlpf = 6;
		break;
	case IMU_FILTER_MEDIUM:
		dlpf = 3;
		break;
	case IMU_FILTER_HIGH:
	default:
		dlpf = 0;
		break;
	}
	(void)mpu9x50_platform_write(m_i2c_addr, MPU9X50_REG_CONFIG, &dlpf, 1);

	uint8_t srd = 0;
	if (m_rate_hz > 0) {
		int temp_srd = (1000 / m_rate_hz) - 1;
		if (temp_srd > 255) temp_srd = 255;
		if (temp_srd < 0) temp_srd = 0;
		srd = (uint8_t)temp_srd;
	}
	(void)mpu9x50_platform_write(m_i2c_addr, MPU9X50_REG_SMPLRT_DIV, &srd, 1);

	m_thd_running = true;

	for (;;) {
		if (m_should_terminate) {
			break;
		}

		uint8_t data[14];
		if (mpu9x50_platform_read(m_i2c_addr, MPU9X50_REG_ACCEL_XOUT_H, data, 14) == 0) {
			int16_t accel_raw[3];
			int16_t gyro_raw[3];
			float accel_si[3];
			float gyro_si[3];
			float mag[3] = {0.0f, 0.0f, 0.0f};

			accel_raw[0] = ((int16_t)data[0] << 8) | data[1];
			accel_raw[1] = ((int16_t)data[2] << 8) | data[3];
			accel_raw[2] = ((int16_t)data[4] << 8) | data[5];

			gyro_raw[0] = ((int16_t)data[8] << 8) | data[9];
			gyro_raw[1] = ((int16_t)data[10] << 8) | data[11];
			gyro_raw[2] = ((int16_t)data[12] << 8) | data[13];

			mpu9x50_convert_accel(accel_raw, accel_si);
			mpu9x50_convert_gyro(gyro_raw, gyro_si);

			if (m_read_callback) {
				m_read_callback(accel_si, gyro_si, mag);
			}
		}

		int delay_ms = m_rate_hz > 0 ? (1000 / m_rate_hz) : 5;
		if (delay_ms < 1) {
			delay_ms = 1;
		}
		vTaskDelay(pdMS_TO_TICKS(delay_ms));
	}

	m_thd_running = false;
	vTaskDelete(NULL);
}

void mpu9x50_wrapper_init(void) {
	if (m_thd_running) {
		return;
	}

	m_should_terminate = false;
	xTaskCreatePinnedToCore(mpu9x50_task, "MPU9X50", 2048, NULL, 6, NULL, tskNO_AFFINITY);
}

void mpu9x50_wrapper_set_rate_hz(int hz) {
	m_rate_hz = hz;
	if (m_rate_hz < 4) {
		m_rate_hz = 4;
	}
	if (m_rate_hz > 1000) {
		m_rate_hz = 1000;
	}
}

void mpu9x50_wrapper_set_filter(IMU_FILTER f) {
	m_filter = f;
}

void mpu9x50_wrapper_set_read_callback(void(*func)(float *accel, float *gyro, float *mag)) {
	m_read_callback = func;
}

void mpu9x50_wrapper_stop(void) {
	if (m_thd_running) {
		m_should_terminate = true;
		for (int i = 0; i < 100; i++) {
			if (!m_thd_running) {
				break;
			}
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
}
