#ifndef MAIN_HWCONF_HW_P4_NRW32N16C6_H_
#define MAIN_HWCONF_HW_P4_NRW32N16C6_H_

#define HW_NAME					"P4 NRW32N16C6"
#define HW_TARGET				"esp32p4_nrw32n16c6"
#define HW_NO_UART

#define HW_INIT_HOOK()			hw_init()

void hw_init(void);

#endif /* MAIN_HWCONF_HW_P4_NRW32N16C6_H_ */
