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

#ifndef HWCONF_RABR_H_
#define HWCONF_RABR_H_

#define HW_NAME				"RABR"
#define HW_TARGET           "esp32c3"

#define OVR_CONF_PARSER_C		"rabr_confparser.c"
#define OVR_CONF_PARSER_H		"rabr_confparser.h"
#define OVR_CONF_XML_C			"rabr_confxml.c"
#define OVR_CONF_XML_H			"rabr_confxml.h"
#define OVR_CONF_DEFAULT		"rabr_conf_default.h"
#define OVR_CONF_SERIALIZE		rabr_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE	rabr_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS	rabr_confparser_set_defaults_main_config_t
#define OVR_CONF_MAIN_CONFIG
#define VAR_INIT_CODE				259763435

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

#define HW_DEFAULT_ID 10

#define HW_INIT_HOOK() 			hw_init()

// CAN
#define CAN_TX_GPIO_NUM			0
#define CAN_RX_GPIO_NUM			1

// Other pins
#define PIN_LED_SYSTEM 8
#define PIN_LED_DATA 9
#define PIN_HV_INPUT 7
// Functions
void hw_init(void);

#endif /* HWCONF_RABR_H_ */
