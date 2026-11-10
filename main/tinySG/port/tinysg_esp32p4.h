#ifndef TINYSG_ESP32P4_H
#define TINYSG_ESP32P4_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint32_t fills, rotations, fallbacks;
} tinysg_ppa_stats;

bool tinysg_esp32p4_enable(bool enabled);
tinysg_ppa_stats tinysg_esp32p4_stats(void);
void *tinysg_esp32p4_alloc(size_t bytes);
bool tinysg_esp32p4_rotate(const uint16_t *source, uint16_t *destination, uint16_t width, uint16_t height,
                           unsigned rotation, size_t bytes);

#endif
