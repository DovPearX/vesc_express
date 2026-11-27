#ifndef MAIN_LISPIF_VESC_EXTENSIONS_H_
#define MAIN_LISPIF_VESC_EXTENSIONS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "lbm_flat_value.h"
#include "lbm_types.h"

void lispif_load_vesc_extensions(void);
void lispif_events_context_done(lbm_cid cid);
void lispif_process_custom_app_data(uint8_t *data, unsigned int len);

typedef struct {
	const lbm_uint *symbol;
	atomic_bool enabled;
	atomic_int waiter;
	TickType_t started;
	TickType_t timeout;
} lisp_event_t;

typedef bool (*lispif_event_pack_t)(lbm_flat_value_t *flat, bool waiting, const void *payload);

void lispif_event_reset(lisp_event_t *event);
lbm_value lispif_event_wait(lisp_event_t *event, lbm_value *args, lbm_uint argn);
void lispif_event_forget_waiter(lisp_event_t *event, lbm_cid cid);
void lispif_event_post(lisp_event_t *event, size_t len, lispif_event_pack_t pack, const void *payload);

#endif
