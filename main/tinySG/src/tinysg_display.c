#include "tinysg.h"

static tinysg_sink render_callback;
static tinysg_clear_callback clear_callback;
static tinysg_reset_callback reset_callback;
static tinysg_accelerator accelerator;
static void *accelerator_context;

void tinysg_set_accelerator(const tinysg_accelerator *callbacks, void *ctx) {
  accelerator = callbacks ? *callbacks : (tinysg_accelerator){0};
  accelerator_context = ctx;
}

bool tinysg_accelerated_fill(uint8_t *pixels, size_t count, uint16_t color) {
  return accelerator.fill && accelerator.fill(accelerator_context, pixels, count, color);
}

void tinysg_display_extensions_set_callbacks(tinysg_sink render, tinysg_clear_callback clear,
                                             tinysg_reset_callback reset) {
  render_callback = render;
  clear_callback = clear;
  reset_callback = reset;
}

bool tinysg_display_write(void *ctx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pixels) {
  if (!render_callback || !pixels || !w || !h) return false;
  return render_callback(ctx, x, y, w, h, pixels);
}

bool tinysg_display_clear(uint32_t rgb888) {
  if (!clear_callback) return false;
  clear_callback(rgb888);
  return true;
}

bool tinysg_display_reset(void) {
  if (!reset_callback) return false;
  reset_callback();
  return true;
}
