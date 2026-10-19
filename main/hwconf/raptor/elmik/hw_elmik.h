#ifndef MAIN_HWCONF_ELMIK_H_
#define MAIN_HWCONF_ELMIK_H_

#define HW_NAME                     "ELMik"
#define HW_TARGET                   "esp32c3"

#define HW_INIT_HOOK()              hw_init()

// CAN
#define CAN_TX_GPIO_NUM             20
#define CAN_RX_GPIO_NUM             21

#define CONF_CAN_STATUS_RATE_HZ     0

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_ELMIK_H_ */
