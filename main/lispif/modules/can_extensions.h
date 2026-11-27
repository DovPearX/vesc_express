#ifndef MAIN_LISPIF_MODULES_CAN_EXTENSIONS_H_
#define MAIN_LISPIF_MODULES_CAN_EXTENSIONS_H_

#include <stdbool.h>
#include <stdint.h>

#include "lbm_types.h"

void lispif_load_can_extensions(void);
bool lispif_can_enable_event(lbm_uint symbol, bool enabled);
void lispif_can_context_done(lbm_cid cid);
bool lispif_can_event_enabled(unsigned bus, bool extended);
void lispif_can_event(unsigned bus, uint32_t id, uint8_t *data, uint8_t len, bool extended);

#endif
