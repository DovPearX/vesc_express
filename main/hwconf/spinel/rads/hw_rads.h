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

#ifndef HWCONF_RADS_H_
#define HWCONF_RADS_H_

#define HW_NAME				"RADS"
#define HW_TARGET           "esp32s3_n16r8"

#define OVR_CONF_PARSER_C		"rads_confparser.c"
#define OVR_CONF_PARSER_H		"rads_confparser.h"
#define OVR_CONF_XML_C			"rads_confxml.c"
#define OVR_CONF_XML_H			"rads_confxml.h"
#define OVR_CONF_DEFAULT		"rads_conf_default.h"
#define OVR_CONF_SERIALIZE		rads_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE	rads_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS	rads_confparser_set_defaults_main_config_t
#define OVR_CONF_MAIN_CONFIG
#define VAR_INIT_CODE				259763451

typedef struct {
	int controller_id;
	CAN_BAUD can_baud_rate;
	int can_status_rate_hz;
	WIFI_MODE wifi_mode;
	char wifi_sta_ssid[36];
	char wifi_sta_key[26];
	char wifi_ap_ssid[36];
	char wifi_ap_key[26];
	bool use_tcp_local;
	bool use_tcp_hub;
	char tcp_hub_url[36];
	uint16_t tcp_hub_port;
	char tcp_hub_id[26];
	char tcp_hub_pass[26];
	BLE_MODE ble_mode;
	char ble_name[9];
	uint32_t ble_pin;
	uint32_t ble_service_capacity;
	uint32_t ble_chr_descr_capacity;
} main_config_t;

#define HW_DEFAULT_ID 20

#define HW_INIT_HOOK() 			hw_init()

// CAN
#define CAN_TX_GPIO_NUM			6
#define CAN_RX_GPIO_NUM			5

// Display
#define DISP_SD0			13
#define DISP_CLK			10
#define DISP_CS				12
#define DISP_RESET			9
#define DISP_DC				11

// Touch CST816
#define TOUCH_I2C_SDA		47
#define TOUCH_I2C_SCL		48
#define TOUCH_RST			17
#define TOUCH_INT			21

// QMI8658 IMU
#define QMI_I2C_SDA			47
#define QMI_I2C_SCL			48
#define QMI_INT1		 	8
#define QMI_INT2		 	7
#define QMI_I2C_ADDR		0x6B

#define HW_IMU_SDA 47
#define HW_IMU_SCL 48
#define HW_IMU_TYPE IMU_TYPE_EXTERNAL_QMI8658

// Functions
void hw_init(void);

#endif /* HWCONF_RADS_H_ */
