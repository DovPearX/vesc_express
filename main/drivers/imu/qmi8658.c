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

#include "qmi8658.h"
#include "imu.h"
#include "commands.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdint.h>

#define QMI8658_ADDR_A          0x6A
#define QMI8658_ADDR_B          0x6B
#define QMI8658_WHO_AM_I_VAL    0x05

#define QMI8658_REG_WHO_AM_I    0x00
#define QMI8658_REG_CTRL1       0x02
#define QMI8658_REG_CTRL2       0x03
#define QMI8658_REG_CTRL3       0x04
#define QMI8658_REG_CTRL5       0x06
#define QMI8658_REG_CTRL7       0x08
#define QMI8658_REG_STATUS0     0x2E
#define QMI8658_REG_AX_L        0x35

#define QMI8658_SENSOR_ACC_EN   0x01
#define QMI8658_SENSOR_GYR_EN   0x02

static volatile uint8_t qmi8658_addr = QMI8658_ADDR_B;
static int rate_hz = 1000;
static IMU_FILTER filter = IMU_FILTER_MEDIUM;
static volatile bool thd_running = false;
static volatile bool should_terminate = false;

static void (*read_callback)(float *accel, float *gyro, float *mag) = 0;

static bool qmi8658_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    return imu_i2c_tx_rx(qmi8658_addr, &reg, 1, data, len);
}

static bool qmi8658_write_reg(uint8_t reg, uint8_t val) {
    uint8_t txb[2] = {reg, val};
    return imu_i2c_tx_rx(qmi8658_addr, txb, 2, 0, 0);
}

static uint8_t qmi8658_acc_odr_from_rate(int hz) {
    if (hz <= 31) {
        return 0x08;
    } else if (hz <= 62) {
        return 0x07;
    } else if (hz <= 125) {
        return 0x06;
    } else if (hz <= 250) {
        return 0x05;
    } else if (hz <= 500) {
        return 0x04;
    } else if (hz <= 1000) {
        return 0x03;
    } else if (hz <= 2000) {
        return 0x02;
    } else if (hz <= 4000) {
        return 0x01;
    }

    return 0x00;
}

static uint8_t qmi8658_gyro_odr_from_rate(int hz) {
    if (hz <= 31) {
        return 0x08;
    } else if (hz <= 62) {
        return 0x07;
    } else if (hz <= 125) {
        return 0x06;
    } else if (hz <= 250) {
        return 0x05;
    } else if (hz <= 500) {
        return 0x04;
    } else if (hz <= 1000) {
        return 0x03;
    } else if (hz <= 2000) {
        return 0x02;
    } else if (hz <= 4000) {
        return 0x01;
    }

    return 0x00;
}

static void qmi8658_task(void *arg) {
    (void)arg;

    while (!should_terminate) {
        uint8_t status = 0;
        uint8_t raw[12] = {0};

        bool st_ok = qmi8658_read_reg(QMI8658_REG_STATUS0, &status, 1);
        bool drdy = (status & 0x03) != 0;

        if (st_ok && drdy && qmi8658_read_reg(QMI8658_REG_AX_L, raw, sizeof(raw))) {
            int16_t ax_raw = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
            int16_t ay_raw = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
            int16_t az_raw = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

            int16_t gx_raw = (int16_t)(((uint16_t)raw[7] << 8) | raw[6]);
            int16_t gy_raw = (int16_t)(((uint16_t)raw[9] << 8) | raw[8]);
            int16_t gz_raw = (int16_t)(((uint16_t)raw[11] << 8) | raw[10]);

            float acc[3];
            float gyro[3];
            float mag[3] = {0.0f, 0.0f, 0.0f};

            // 8g full scale => 4096 LSB/g
            acc[0] = (float)ax_raw / 4096.0f;
            acc[1] = (float)ay_raw / 4096.0f;
            acc[2] = (float)az_raw / 4096.0f;

            // 1024 dps full scale => 32 LSB/dps
            gyro[0] = (float)gx_raw / 32.0f;
            gyro[1] = (float)gy_raw / 32.0f;
            gyro[2] = (float)gz_raw / 32.0f;

            if (read_callback) {
                read_callback(acc, gyro, mag);
            }
        }

        int delay_ms = 1;
        if (rate_hz > 0) {
            delay_ms = 1000 / rate_hz;
            if (delay_ms < 1) {
                delay_ms = 1;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    thd_running = false;
    vTaskDelete(NULL);
}

void qmi8658_set_rate_hz(int hz) {
    rate_hz = hz;
}

void qmi8658_set_filter(IMU_FILTER f) {
    filter = f;
}

void qmi8658_init(void) {
    read_callback = 0;

    uint8_t whoami = 0;
    qmi8658_addr = QMI8658_ADDR_A;

    bool ok = qmi8658_read_reg(QMI8658_REG_WHO_AM_I, &whoami, 1);
    if (!ok || whoami != QMI8658_WHO_AM_I_VAL) {
        qmi8658_addr = QMI8658_ADDR_B;
        ok = qmi8658_read_reg(QMI8658_REG_WHO_AM_I, &whoami, 1);
        if (!ok || whoami != QMI8658_WHO_AM_I_VAL) {
            commands_printf("QMI8658 not found (rx: %d)", whoami);
            return;
        }
    }

    // Basic interface setup
    if (!qmi8658_write_reg(QMI8658_REG_CTRL1, 0x40)) {
        commands_printf("QMI8658 CTRL1 config failed");
        return;
    }

    // Accel: 8g + selected ODR
    uint8_t ctrl2 = (0x02 << 4) | qmi8658_acc_odr_from_rate(rate_hz);
    if (!qmi8658_write_reg(QMI8658_REG_CTRL2, ctrl2)) {
        commands_printf("QMI8658 CTRL2 config failed");
        return;
    }

    // Gyro: 1024 dps + selected ODR
    uint8_t ctrl3 = (0x06 << 4) | qmi8658_gyro_odr_from_rate(rate_hz);
    if (!qmi8658_write_reg(QMI8658_REG_CTRL3, ctrl3)) {
        commands_printf("QMI8658 CTRL3 config failed");
        return;
    }

    // Low-pass filter settings
    uint8_t ctrl5 = 0;
    if (filter != IMU_FILTER_LOW) {
        ctrl5 = 0x11;
    }
    if (!qmi8658_write_reg(QMI8658_REG_CTRL5, ctrl5)) {
        commands_printf("QMI8658 CTRL5 config failed");
        return;
    }

    // Enable accelerometer + gyroscope
    if (!qmi8658_write_reg(QMI8658_REG_CTRL7, QMI8658_SENSOR_ACC_EN | QMI8658_SENSOR_GYR_EN)) {
        commands_printf("QMI8658 CTRL7 config failed");
        return;
    }

    should_terminate = false;
    thd_running = true;
    xTaskCreatePinnedToCore(qmi8658_task, "QMI8658", 2048, NULL, 6, NULL, tskNO_AFFINITY);
}

void qmi8658_stop(void) {
    should_terminate = true;

    bool thd_was_running = thd_running;
    while (thd_running) {
        vTaskDelay(1);
    }

    if (thd_was_running) {
        qmi8658_write_reg(QMI8658_REG_CTRL7, 0x00);
    }
}

void qmi8658_set_read_callback(void(*func)(float *accel, float *gyro, float *mag)) {
    read_callback = func;
}
