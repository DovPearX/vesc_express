#include "hw_devkit_c3.h"
#include "lispif.h"
#include "lispbm.h"

void hw_init(void) {
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << LED_BLUE_PIN);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	LED_BLUE_OFF();
}
