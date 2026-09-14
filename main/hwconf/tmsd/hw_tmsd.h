#ifndef HW_TMSD_H_
#define HW_TMSD_H_

#include "adc.h"
#include "driver/gpio.h"
#include "datatypes.h"

#define HW_NAME                 "TMSD"
#define HW_TARGET               "esp32s3_n16r8"

#define HW_EARLY_LBM_INIT
//#define HW_NO_UART
#define HW_UART_COMM

#define HW_INIT_HOOK()          hw_init()

// Configuration overrides
#define OVR_CONF_PARSER_C		"tmsd_confparser.c"
#define OVR_CONF_PARSER_H		"tmsd_confparser.h"
#define OVR_CONF_XML_C			"tmsd_confxml.c"
#define OVR_CONF_XML_H			"tmsd_confxml.h"
#define OVR_CONF_DEFAULT		"tmsd_conf_default.h"
#define OVR_CONF_SERIALIZE		tmsd_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE	tmsd_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS	tmsd_confparser_set_defaults_main_config_t
#define OVR_CONF_MAIN_CONFIG
#define VAR_INIT_CODE			259763458

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

// Default setting Overrides
#define HW_DEFAULT_ID			20

// CAN
#define CAN_TX_GPIO_NUM			44
#define CAN_RX_GPIO_NUM			43

// SD-card
#define SD_PIN_MOSI				11
#define SD_PIN_MISO				13
#define SD_PIN_SCK				12
#define SD_PIN_CS				10

// Battery
#define VBAT_SENSE              5
#define R1                      33000.0f
#define R2                      100000.0f

// Touch
#define TOUCH_INT               3
#define TOUCH_SCL               4
#define TOUCH_SDA               8

// UART
#define UART_NUM                    1
#define UART_BAUDRATE               19200
#define UART_TX                     17
#define UART_RX                     18

// Display
#define DISPLAY_TE              38
#define DISPLAY_CS              45
#define DISPLAY_SCK             47
#define DISPLAY_D0              21
#define DISPLAY_D1              48
#define DISPLAY_D2              40
#define DISPLAY_D3              39
#define DISPLAY_BLK             1

// HEADER 8P
// IO 5
// IO 6
// IO 7
// IO 15
// IO 16
// IO 46
// IO 9
// IO 14

// HEADER 4P
// GND
// 3V3
// IO 17
// IO 18

// Functions
void hw_init(void);

#endif /* HW_TMSD_H_ */
