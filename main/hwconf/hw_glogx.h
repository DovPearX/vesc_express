#ifndef HWCONF_GLOGX_H_
#define HWCONF_GLOGX_H_

#define HW_NAME						"GlogX"

//#define HW_UART_COMM

#ifndef CONF_BLE_NAME
#define CONF_BLE_NAME "GlogX"
#endif

#define HW_INIT_HOOK()				hw_init()

// LEDs
#define LED_BLUE_PIN				8

#define LED_BLUE_ON()				gpio_set_level(LED_BLUE_PIN, 0)
#define LED_BLUE_OFF()				gpio_set_level(LED_BLUE_PIN, 1)

// CAN
#define CAN_TX_GPIO_NUM				20
#define CAN_RX_GPIO_NUM				21

// SD-card
#define SD_PIN_MOSI					10
#define SD_PIN_MISO					7
#define SD_PIN_SCK					9
#define SD_PIN_CS					3

//UART
#define UART_NUM				0
#define UART_BAUDRATE		    115200
#define UART_TX					1
#define UART_RX					0

// Functions
void hw_init(void);

#endif /* HWCONF_GLOGX_H_ */
