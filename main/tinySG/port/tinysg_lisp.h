#ifndef TINYSG_LISP_H
#define TINYSG_LISP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void lispif_tinysg_init(void);
bool lispif_tinysg_set_buffer(uint8_t *pixels, size_t bytes);

#endif
