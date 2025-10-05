#include "hw_devkit_c3.h"
#include "lispif.h"
#include "lispbm.h"
#include "nmea.h"
#include "ublox.h"

static lbm_value ext_gnss_n_sat(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_i(nmea_get_state()->gga.n_sat);
}

static void load_extensions(bool main_found) {
	if (main_found) {
		return;
	}

	//memset(&syms_vesc, 0, sizeof(syms_vesc));

	// Add GNSS sat number extension
	lbm_add_extension("gnss-n-sat", ext_gnss_n_sat);
}

void hw_init(void) {
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << LED_BLUE_PIN);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	LED_BLUE_OFF();

	lispif_add_ext_load_callback(load_extensions);
}
