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

#include "bmi160_wrapper.h"
#include "imu.h"
#include "commands.h"
#include "bmi160/bmi160.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define BMI160_ADDR_PRIMARY 0x68
#define BMI160_ADDR_SECONDARY 0x69

static struct bmi160_dev m_bmi160;
static int m_rate_hz = 200;
static IMU_FILTER m_filter = IMU_FILTER_MEDIUM;
static volatile bool m_thd_running = false;
static volatile bool m_should_terminate = false;
static void (*m_read_callback)(float *accel, float *gyro, float *mag) = 0;

static int8_t bmi160_platform_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	return imu_i2c_tx_rx(dev_addr, &reg_addr, 1, data, len) ? BMI160_OK : BMI160_E_COM_FAIL;
}

static int8_t bmi160_platform_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	uint8_t txb[16];

	if (len > (sizeof(txb) - 1U)) {
		return BMI160_E_READ_WRITE_LENGTH_INVALID;
	}

	txb[0] = reg_addr;
	if (len > 0U) {
		memcpy(&txb[1], data, len);
	}

	return imu_i2c_tx_rx(dev_addr, txb, len + 1U, 0, 0) ? BMI160_OK : BMI160_E_COM_FAIL;
}

static void bmi160_platform_delay_ms(uint32_t period) {
	if (period > 0U) {
		vTaskDelay(pdMS_TO_TICKS(period));
	}
}

static uint8_t bmi160_accel_odr_from_rate(int hz) {
	if (hz <= 25) {
		return BMI160_ACCEL_ODR_25HZ;
	} else if (hz <= 50) {
		return BMI160_ACCEL_ODR_50HZ;
	} else if (hz <= 100) {
		return BMI160_ACCEL_ODR_100HZ;
	} else if (hz <= 200) {
		return BMI160_ACCEL_ODR_200HZ;
	} else if (hz <= 400) {
		return BMI160_ACCEL_ODR_400HZ;
	} else if (hz <= 800) {
		return BMI160_ACCEL_ODR_800HZ;
	}

	return BMI160_ACCEL_ODR_1600HZ;
}

static uint8_t bmi160_gyro_odr_from_rate(int hz) {
	if (hz <= 25) {
		return BMI160_GYRO_ODR_25HZ;
	} else if (hz <= 50) {
		return BMI160_GYRO_ODR_50HZ;
	} else if (hz <= 100) {
		return BMI160_GYRO_ODR_100HZ;
	} else if (hz <= 200) {
		return BMI160_GYRO_ODR_200HZ;
	} else if (hz <= 400) {
		return BMI160_GYRO_ODR_400HZ;
	} else if (hz <= 800) {
		return BMI160_GYRO_ODR_800HZ;
	} else if (hz <= 1600) {
		return BMI160_GYRO_ODR_1600HZ;
	}

	return BMI160_GYRO_ODR_3200HZ;
}

static uint8_t bmi160_accel_bw_from_filter(IMU_FILTER filter) {
	switch (filter) {
	case IMU_FILTER_LOW:
		return BMI160_ACCEL_BW_OSR4_AVG1;
	case IMU_FILTER_HIGH:
		return BMI160_ACCEL_BW_NORMAL_AVG4;
	case IMU_FILTER_MEDIUM:
	default:
		return BMI160_ACCEL_BW_OSR2_AVG2;
	}
}

static uint8_t bmi160_gyro_bw_from_filter(IMU_FILTER filter) {
	switch (filter) {
	case IMU_FILTER_LOW:
		return BMI160_GYRO_BW_OSR4_MODE;
	case IMU_FILTER_HIGH:
		return BMI160_GYRO_BW_NORMAL_MODE;
	case IMU_FILTER_MEDIUM:
	default:
		return BMI160_GYRO_BW_OSR2_MODE;
	}
}

static float bmi160_accel_lsb_per_g(void) {
	switch (m_bmi160.accel_cfg.range) {
	case BMI160_ACCEL_RANGE_2G:
		return 16384.0f;
	case BMI160_ACCEL_RANGE_4G:
		return 8192.0f;
	case BMI160_ACCEL_RANGE_8G:
		return 4096.0f;
	case BMI160_ACCEL_RANGE_16G:
	default:
		return 2048.0f;
	}
}

static float bmi160_gyro_lsb_per_dps(void) {
	switch (m_bmi160.gyro_cfg.range) {
	case BMI160_GYRO_RANGE_125_DPS:
		return 262.4f;
	case BMI160_GYRO_RANGE_250_DPS:
		return 131.2f;
	case BMI160_GYRO_RANGE_500_DPS:
		return 65.6f;
	case BMI160_GYRO_RANGE_1000_DPS:
		return 32.8f;
	case BMI160_GYRO_RANGE_2000_DPS:
	default:
		return 16.4f;
	}
}

static void bmi160_task(void *arg) {
	(void)arg;

	while (!m_should_terminate) {
		struct bmi160_sensor_data acc_raw;
		struct bmi160_sensor_data gyro_raw;
		int8_t res = bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL, &acc_raw, &gyro_raw, &m_bmi160);

		if (res == BMI160_OK) {
			float acc[3];
			float gyro[3];
			float mag[3] = {0.0f, 0.0f, 0.0f};
			float acc_scale = bmi160_accel_lsb_per_g();
			float gyro_scale = bmi160_gyro_lsb_per_dps();

			acc[0] = (float)acc_raw.x / acc_scale;
			acc[1] = (float)acc_raw.y / acc_scale;
			acc[2] = (float)acc_raw.z / acc_scale;

			gyro[0] = (float)gyro_raw.x / gyro_scale;
			gyro[1] = (float)gyro_raw.y / gyro_scale;
			gyro[2] = (float)gyro_raw.z / gyro_scale;

			if (m_read_callback) {
				m_read_callback(acc, gyro, mag);
			}
		}

		int delay_ms = 1;
		if (m_rate_hz > 0) {
			delay_ms = 1000 / m_rate_hz;
			if (delay_ms < 1) {
				delay_ms = 1;
			}
		}

		vTaskDelay(pdMS_TO_TICKS(delay_ms));
	}

	m_thd_running = false;
	vTaskDelete(NULL);
}

void bmi160_wrapper_set_rate_hz(int hz) {
	m_rate_hz = hz;
}

void bmi160_wrapper_set_filter(IMU_FILTER filter) {
	m_filter = filter;
}

void bmi160_wrapper_init(void) {
	memset(&m_bmi160, 0, sizeof(m_bmi160));
	m_bmi160.read = bmi160_platform_read;
	m_bmi160.write = bmi160_platform_write;
	m_bmi160.delay_ms = bmi160_platform_delay_ms;
	m_bmi160.intf = BMI160_I2C_INTF;
	m_bmi160.read_write_len = 8;
	m_bmi160.id = BMI160_ADDR_PRIMARY;

	int8_t res = bmi160_init(&m_bmi160);
	if (res != BMI160_OK) {
		m_bmi160.id = BMI160_ADDR_SECONDARY;
		res = bmi160_init(&m_bmi160);
	}

	if (res != BMI160_OK) {
		commands_printf("BMI160 not found (err: %d)", res);
		return;
	}

	m_bmi160.accel_cfg.odr = bmi160_accel_odr_from_rate(m_rate_hz);
	m_bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
	m_bmi160.accel_cfg.bw = bmi160_accel_bw_from_filter(m_filter);
	m_bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	m_bmi160.gyro_cfg.odr = bmi160_gyro_odr_from_rate(m_rate_hz);
	m_bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	m_bmi160.gyro_cfg.bw = bmi160_gyro_bw_from_filter(m_filter);
	m_bmi160.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	res = bmi160_set_sens_conf(&m_bmi160);
	if (res != BMI160_OK) {
		commands_printf("BMI160 config failed (err: %d)", res);
		return;
	}

	m_should_terminate = false;
	m_thd_running = true;
	xTaskCreatePinnedToCore(bmi160_task, "BMI160", 3072, NULL, 6, NULL, tskNO_AFFINITY);
}

void bmi160_wrapper_set_read_callback(void(*func)(float *accel, float *gyro, float *mag)) {
	m_read_callback = func;
}

void bmi160_wrapper_stop(void) {
	m_should_terminate = true;

	bool thd_was_running = m_thd_running;
	while (m_thd_running) {
		vTaskDelay(1);
	}

	if (thd_was_running) {
		m_bmi160.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
		m_bmi160.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
		bmi160_set_power_mode(&m_bmi160);
	}
}