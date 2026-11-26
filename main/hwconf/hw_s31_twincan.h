#ifndef HW_S31_TWINCAN_H_
#define HW_S31_TWINCAN_H_

#define HW_NAME             "S31 TwinCAN"
#define HW_TARGET           "esp32s31_n16r16v"

// UART
#define HW_UART_COMM
#define UART_NUM            0
#define UART_BAUDRATE       115200
#define UART_TX             58
#define UART_RX             59

// CAN1
#define CAN_TX_GPIO_NUM     38
#define CAN_RX_GPIO_NUM     39

// CAN2
#define CAN2_TX_GPIO_NUM    42
#define CAN2_RX_GPIO_NUM    40

#endif
