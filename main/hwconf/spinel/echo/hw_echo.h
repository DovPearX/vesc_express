#ifndef HWCONF_ECHO_H_
#define HWCONF_ECHO_H_

#define HW_NAME                 "ECHO"
#define HW_TARGET               "esp32s3"
#define HW_HAS_PSRAM
#define HW_FLASH_16MB
#define HW_INTERNAL_FS

#define OVR_CONF_PARSER_C		"echo_confparser.c"
#define OVR_CONF_PARSER_H		"echo_confparser.h"
#define OVR_CONF_XML_C			"echo_confxml.c"
#define OVR_CONF_XML_H			"echo_confxml.h"
#define OVR_CONF_DEFAULT		"echo_conf_default.h"
#define OVR_CONF_SERIALIZE		echo_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE	echo_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS	echo_confparser_set_defaults_main_config_t
#define OVR_CONF_MAIN_CONFIG
#define VAR_INIT_CODE				259763456

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

#define HW_INIT_HOOK()          hw_init()

#define HW_DEFAULT_ID 30

// CAN
#define CAN_TX_GPIO_NUM			44
#define CAN_RX_GPIO_NUM			43

// SD-card
#define SD_PIN_MOSI				11
#define SD_PIN_MISO				13
#define SD_PIN_SCK				12
#define SD_PIN_CS				10

//Display
#define DISP_BLK				1
#define DISP_SD0				21
#define DISP_SD1				48
#define DISP_SD2				40
#define DISP_SD3				39
#define DISP_CLK				47
#define DISP_CS					45
#define DISP_RESET				-1
#define DISP_TE					38

//Buttons
#define PIN_UP					16
#define PIN_MODE				6
#define PIN_DOWN				5
#define PIN_GND					15
#define PIN_POWER				7

// Functions
void hw_init(void);

#endif /* HWCONF_ECHO_H_ */
