#ifndef LISPBM_LISPIF_C_LIB_H_
#define LISPBM_LISPIF_C_LIB_H_

#include "eval_cps.h"

void lispif_c_lib_eval_context_done(eval_context_t *ctx);
void lispif_c_lib_eval_prepare_stop(void);
void lispif_c_lib_eval_reset(void);

#endif
