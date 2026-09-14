#include "hw_tmsd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "lispif_disp_extensions.h"
#include "disp_axs15231.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_sleep.h"

#include "lispif.h"
#include "lispbm.h"
#include "extensions/display_extensions.h"
#include "terminal.h"
#include "commands.h"
#include "utils.h"

static esp_err_t disp_backlight_pwm_init(void);

static lbm_value ext_disp_init(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	disp_axs15231_init(
            DISPLAY_D0,
            DISPLAY_D1,
            DISPLAY_D2,
            DISPLAY_D3,
            DISPLAY_SCK,
            DISPLAY_CS,
            -1,
            40);

	lbm_display_extensions_set_callbacks(
			disp_axs15231_render_image,
			disp_axs15231_clear,
			disp_axs15231_reset
	);

	disp_axs15231_reset();
	(void)disp_backlight_pwm_init();

	return ENC_SYM_TRUE;
}

static const ledc_timer_t disp_backlight_timer = LEDC_TIMER_0;
static const ledc_channel_t disp_backlight_channel = LEDC_CHANNEL_0;
static const int disp_backlight_bits = 10;
static const int disp_backlight_freq_hz = 5000;
static int disp_backlight_max = 1 << disp_backlight_bits;
static bool disp_backlight_initialized = false;

static esp_err_t disp_backlight_pwm_init(void) {
	if (disp_backlight_initialized) {
		return ESP_OK;
	}

	if (!utils_gpio_is_valid(DISPLAY_BLK)) {
		return ESP_ERR_INVALID_ARG;
	}

	ledc_timer_config_t timer = {
		.speed_mode       = LEDC_LOW_SPEED_MODE,
		.timer_num        = disp_backlight_timer,
		.duty_resolution  = disp_backlight_bits,
		.freq_hz          = disp_backlight_freq_hz,
		.clk_cfg          = LEDC_AUTO_CLK
	};

	if (ledc_timer_config(&timer) != ESP_OK) {
		return ESP_FAIL;
	}

	ledc_channel_config_t channel = {
		.speed_mode     = LEDC_LOW_SPEED_MODE,
		.channel        = disp_backlight_channel,
		.timer_sel      = disp_backlight_timer,
		.intr_type      = LEDC_INTR_DISABLE,
		.gpio_num       = DISPLAY_BLK,
		.duty           = 0,
		.hpoint         = 0
	};

	if (ledc_channel_config(&channel) != ESP_OK) {
		return ESP_FAIL;
	}

	disp_backlight_initialized = true;
	return ESP_OK;
}

static lbm_value ext_disp_set_backlight(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	float percent = lbm_dec_as_float(args[0]);
	utils_truncate_number(&percent, 0.0, 100.0);

	if (disp_backlight_pwm_init() != ESP_OK) {
		lbm_set_error_reason("Backlight PWM init failed");
		return ENC_SYM_TERROR;
	}

	int duty_i = (int)(percent * (float)disp_backlight_max / 100.0f);

	if (ledc_set_duty(LEDC_LOW_SPEED_MODE, disp_backlight_channel, duty_i) != ESP_OK) {
		return ENC_SYM_EERROR;
	}

	if (ledc_update_duty(LEDC_LOW_SPEED_MODE, disp_backlight_channel) != ESP_OK) {
		return ENC_SYM_EERROR;
	}

	return lbm_enc_float(percent);
}

static void load_extensions(bool main_found) {
	if (main_found) {
		return;
	}
	
	lbm_add_extension("disp-init", ext_disp_init);
	lbm_add_extension("disp-set-backlight", ext_disp_set_backlight);
}

void hw_init(void) {

    lispif_add_ext_load_callback(load_extensions);

}
