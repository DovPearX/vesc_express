#include "hw_echo.h"

#include "lispif.h"
#include "disp_axs15231.h"
#include "extensions/display_extensions.h"

static lbm_value ext_disp_init(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn;

    disp_axs15231_init(
            DISP_SD0,
            DISP_SD1,
            DISP_SD2,
            DISP_SD3,
            DISP_CLK,
            DISP_CS,
            DISP_RESET,
            40);

    lbm_display_extensions_set_callbacks(
            disp_axs15231_render_image,
            disp_axs15231_clear,
            disp_axs15231_reset
    );

    disp_axs15231_reset();

    return ENC_SYM_TRUE;
}

static void load_extensions(bool main_found) {
    if (!main_found) {
        lbm_add_extension("disp-init", ext_disp_init);
    }
}

void hw_init(void) {
    lispif_add_ext_load_callback(load_extensions);
}
