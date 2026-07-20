#ifndef MAIN_HWCONF_ECUBOX_H_
#define MAIN_HWCONF_ECUBOX_H_

#define HW_NAME						"ECU Box"
#define HW_TARGET					"esp32c3"
#define HW_UART_COMM

#define HW_INIT_HOOK()				hw_init()

// CAN
#define CAN_TX_GPIO_NUM			1
#define CAN_RX_GPIO_NUM			0

// UART
#define UART_NUM				0
#define UART_BAUDRATE			115200
#define UART_TX					21
#define UART_RX					20

//TODO START --------------------------------------

//I2C
#define I2C_SCL				    9
#define I2C_SDA					8

//+12V output
#define OUTPUT_12V1             6
#define OUTPUT_12V2             5
#define OUTPUT_12V3             4
#define OUTPUT_12V4             3

//? IO FREE
#define IO10                    10
#define IO7                     7

//! TCA9534A I/O expander
#define TCA9534A_ADDR           0x38

//TODO END --------------------------------------

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_ECUBOX_H_ */
