#ifndef HW_DEVKIT_S3_H_
#define HW_DEVKIT_S3_H_

#define HW_NAME                 "Devkit S3"
#define HW_TARGET               "esp32s3"
#define HW_UART_COMM
#define HW_INIT_HOOK()          hw_init()

// CAN
#define CAN_TX_GPIO_NUM				16
#define CAN_RX_GPIO_NUM				17

// UART
#define UART_NUM                0
#define UART_BAUDRATE           115200
#define UART_TX                 43
#define UART_RX                 44

// Functions
void hw_init(void);

#endif /* HW_DEVKIT_S3_H_ */
