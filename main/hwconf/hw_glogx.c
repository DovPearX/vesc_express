#include "hw_glogx.h"
#include "lispif.h"
#include "lispbm.h"
#include "nmea.h"
#include "ublox.h"

#define LBM_BOOL(x) ((x) ? lbm_enc_i(1) : lbm_enc_i(0))

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
    const nmea_gsv_info_t *gsv = &state->gsv;
    const int n = gsv->sat_num;

    if (n <= 0) {
        return ENC_SYM_NIL;
    }

    lbm_value list = ENC_SYM_NIL;

    for (int i = n - 1; i >= 0; i--) {
        const nmea_gsv_sat_t *sat = &gsv->sats[i];

        lbm_value sat_list =
            lbm_cons(lbm_enc_i(sat->prn),
            lbm_cons(lbm_enc_float(sat->elevation),
            lbm_cons(lbm_enc_float(sat->azimuth),
            lbm_cons(lbm_enc_float(sat->snr),
            lbm_cons(lbm_enc_i(sat->lock ? 1 : 0),
            lbm_cons(lbm_enc_float(sat->base_snr),
            lbm_cons(lbm_enc_i(sat->base_lock ? 1 : 0),
            lbm_cons(lbm_enc_i(sat->local_lock ? 1 : 0),
            ENC_SYM_NIL))))))));

        list = lbm_cons(sat_list, list);
    }

    return list;
}

static lbm_value ext_gnss_gsv_info(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	const nmea_gsv_info_t *gsv = &nmea_get_state()->gsv;

	lbm_value info =
		lbm_cons(lbm_enc_i(gsv->sat_num),
		lbm_cons(lbm_enc_i(gsv->sentences),
		lbm_cons(lbm_enc_i(gsv->sat_last),
		lbm_cons(lbm_enc_i(gsv->sat_num_base),
		lbm_cons(lbm_enc_i(gsv->update_time),
		ENC_SYM_NIL)))));

	return info;
}

static void load_extensions(bool main_found) {
	if (main_found) {
		return;
	}

	lbm_add_extension("gnss-n-sat", ext_gnss_n_sat);
	lbm_add_extension("gnss-fix-type", ext_fix_type);
	lbm_add_extension("gnss-sat-list", ext_gnss_sat_list);
	lbm_add_extension("gnss-gsv-info", ext_gnss_gsv_info);
}

void hw_init(void) {
	gpio_config_t io_conf = {0};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << LED_BLUE_PIN);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	LED_BLUE_OFF();

	lispif_add_ext_load_callback(load_extensions);
}
