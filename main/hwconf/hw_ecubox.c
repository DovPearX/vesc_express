#include "hw_ecubox.h"
#include "lispif.h"
#include "lispbm.h"
#include "nmea.h"
#include "ublox.h"

static lbm_value ext_gnss_n_sat(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_i(nmea_get_state()->gga.n_sat);
}

static lbm_value ext_fix_type(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_i(nmea_get_state()->gga.fix_type);
}

static lbm_value ext_gnss_sat_list(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	const nmea_state_t *state = nmea_get_state();
	const int n = state->gsv.n_sats;

	if (n <= 0) {
		return lbm_enc_sym("nil");
	}

	lbm_value list = lbm_enc_sym("nil");

	for (int i = n - 1; i >= 0; i--) {
		const nmea_gsv_sat_t *sat = &state->gsv.sats[i];

		lbm_value sat_list = lbm_enc_cons(lbm_enc_i(sat->prn),
			lbm_enc_cons(lbm_enc_float(sat->elevation),
			lbm_enc_cons(lbm_enc_float(sat->azimuth),
			lbm_enc_cons(lbm_enc_float(sat->snr),
			lbm_enc_cons(lbm_enc_bool(sat->lock),
			lbm_enc_cons(lbm_enc_float(sat->base_snr),
			lbm_enc_cons(lbm_enc_bool(sat->base_lock),
			lbm_enc_cons(lbm_enc_bool(sat->local_lock),
			lbm_enc_sym("nil"))))))))));

		list = lbm_enc_cons(sat_list, list);
	}

	return list;
}

static void load_extensions(bool main_found) {
	if (main_found) {
		return;
	}

	// Add GNSS sat number extension
	lbm_add_extension("gnss-n-sat", ext_gnss_n_sat);

	// Add GNSS fix type extension
	lbm_add_extension("gnss-fix-type", ext_fix_type);

	// Add GNSS sat list extension
	lbm_add_extension("gnss-sat-list", ext_gnss_sat_list);
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
