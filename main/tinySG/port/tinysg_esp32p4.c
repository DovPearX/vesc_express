#include "tinysg_esp32p4.h"
#include "tinysg.h"
#include "driver/ppa.h"
#include "esp_heap_caps.h"

static ppa_client_handle_t fill_client, rotate_client;
static bool active;
static tinysg_ppa_stats counters;

void *tinysg_esp32p4_alloc(size_t bytes) {
  if (!bytes || bytes > SIZE_MAX - 127) return NULL;
  size_t size = (bytes + 127) / 128 * 128;
  void *memory = heap_caps_aligned_alloc(128, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return memory ? memory : heap_caps_aligned_alloc(128, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

static uint32_t expanded(uint16_t color) {
  uint32_t r = color >> 11, g = (color >> 5) & 63, b = color & 31;
  return 0xff000000u | ((r << 3 | r >> 2) << 16) | ((g << 2 | g >> 4) << 8) | (b << 3 | b >> 2);
}

static bool fill(void *ctx, uint8_t *pixels, size_t count, uint16_t color) {
  (void)ctx;
  if (!active || count < 8192 || count / 64 > 8191 || ((uintptr_t)pixels & 127) || ((count * 2) & 127)) return false;
  uint16_t swapped = (uint16_t)(color << 8 | color >> 8);
  ppa_fill_oper_config_t operation = {.out = {.buffer = pixels,
                                              .buffer_size = count * 2,
                                              .pic_w = 64,
                                              .pic_h = count / 64,
                                              .fill_cm = PPA_FILL_COLOR_MODE_RGB565},
                                      .fill_block_w = 64,
                                      .fill_block_h = count / 64,
                                      .fill_color_val = expanded(swapped),
                                      .mode = PPA_TRANS_MODE_BLOCKING};
  if (ppa_do_fill(fill_client, &operation) != ESP_OK) {
    counters.fallbacks++;
    return false;
  }
  counters.fills++;
  return true;
}

bool tinysg_esp32p4_enable(bool enabled) {
  active = false;
  tinysg_set_accelerator(NULL, NULL);
  if (!enabled) return true;
  ppa_client_config_t config = {.oper_type = PPA_OPERATION_FILL};
  if (!fill_client && ppa_register_client(&config, &fill_client) != ESP_OK) return false;
  config.oper_type = PPA_OPERATION_SRM;
  if (!rotate_client && ppa_register_client(&config, &rotate_client) != ESP_OK) return false;
  active = true;
  const tinysg_accelerator callbacks = {.fill = fill};
  tinysg_set_accelerator(&callbacks, NULL);
  return true;
}

tinysg_ppa_stats tinysg_esp32p4_stats(void) {
  return counters;
}

bool tinysg_esp32p4_rotate(const uint16_t *source, uint16_t *destination, uint16_t width, uint16_t height,
                           unsigned rotation, size_t bytes) {
  if (!active || !source || !destination || source == destination || (size_t)width * height < 8192 || rotation < 1 ||
      rotation > 3 || ((uintptr_t)destination & 127) || (bytes & 127) || bytes < (size_t)width * height * 2)
    return false;
  static const ppa_srm_rotation_angle_t angles[] = {PPA_SRM_ROTATION_ANGLE_0, PPA_SRM_ROTATION_ANGLE_270,
                                                    PPA_SRM_ROTATION_ANGLE_180, PPA_SRM_ROTATION_ANGLE_90};
  bool swapped = rotation & 1;
  ppa_srm_oper_config_t operation = {.in = {.buffer = source,
                                            .pic_w = width,
                                            .pic_h = height,
                                            .block_w = width,
                                            .block_h = height,
                                            .srm_cm = PPA_SRM_COLOR_MODE_RGB565},
                                     .out = {.buffer = destination,
                                             .buffer_size = bytes,
                                             .pic_w = swapped ? height : width,
                                             .pic_h = swapped ? width : height,
                                             .srm_cm = PPA_SRM_COLOR_MODE_RGB565},
                                     .rotation_angle = angles[rotation],
                                     .scale_x = 1,
                                     .scale_y = 1,
                                     .mode = PPA_TRANS_MODE_BLOCKING};
  if (ppa_do_scale_rotate_mirror(rotate_client, &operation) != ESP_OK) {
    counters.fallbacks++;
    return false;
  }
  counters.rotations++;
  return true;
}
