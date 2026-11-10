#include "tinysg.h"
#include <string.h>

static int32_t parameters[56];
static char text[128] = "tinySG";
static uint8_t rgba[320 * 180 * 4];
static uint8_t scratch[1024];
static uint8_t image_pixels[320 * 180 * 2];
static uint8_t font_pixels[262144];
static tinysg_font imported_font;
static bool font_valid;

uint8_t *tinysg_preview_font(void) {
  return font_pixels;
}

int tinysg_preview_load_font(unsigned bytes) {
  font_valid = bytes <= sizeof(font_pixels) && tinysg_font_load(&imported_font, font_pixels, bytes);
  return font_valid;
}

int32_t *tinysg_preview_parameters(void) {
  return parameters;
}

uint8_t *tinysg_preview_image(void) {
  return image_pixels;
}

char *tinysg_preview_text(void) {
  return text;
}

int tinysg_preview_fill_allowed(unsigned kind, unsigned fill) {
  return tinysg_fill_supported((tinysg_kind)kind, (tinysg_fill)fill);
}

static bool output(void *ctx, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *pixels) {
  (void)ctx;
  for (unsigned i = 0; i < (unsigned)width * height; i++) {
    uint16_t color = (uint16_t)(pixels[2 * i] << 8 | pixels[2 * i + 1]);
    size_t index = ((size_t)(y + i / width) * 320 + x + i % width) * 4;
    unsigned r = color >> 11, g = (color >> 5) & 63, b = color & 31;
    rgba[index] = (uint8_t)(r << 3 | r >> 2);
    rgba[index + 1] = (uint8_t)(g << 2 | g >> 4);
    rgba[index + 2] = (uint8_t)(b << 3 | b >> 2);
    rgba[index + 3] = 255;
  }
  return true;
}

uint8_t *tinysg_preview(void) {
  int32_t *p = parameters;
  if (p[0] < 0 || p[0] > TINYSG_IMAGE) return NULL;
  text[127] = 0;
  tinysg_style style = {.fill = (tinysg_fill)p[1],
                        .opacity = (uint8_t)p[2],
                        .quality = (uint8_t)p[3],
                        .flags = TINYSG_STYLE_OPACITY,
                        .stroke_width = (uint8_t)p[10],
                        .dash_length = (uint8_t)p[11],
                        .gap_length = (uint8_t)p[12],
                        .secondary_color = (uint16_t)p[16],
                        .gradient_angle = (int16_t)p[29]};
  tinysg_node nodes[64] = {{.kind = (tinysg_kind)p[0],
                            .x = (int16_t)p[20],
                            .y = (int16_t)p[21],
                            .w = (int16_t)p[22],
                            .h = (int16_t)p[23],
                            .color = (uint16_t)p[15],
                            .style = &style,
                            .angle_start = (int16_t)p[24],
                            .angle_sweep = (int16_t)p[25],
                            .radius = (uint16_t)p[26],
                            .x2 = (int16_t)p[27],
                            .y2 = (int16_t)p[28],
                            .font = p[13] == 3 ? &tinysg_font_3x5 : &tinysg_font_5x7,
                            .text = text,
                            .scale = (uint8_t)p[14]}};
  if (p[0] == TINYSG_TEXT && p[54] && font_valid) {
    nodes[0].font = &imported_font;
    nodes[0].font_size = (uint16_t)p[55];
    nodes[0].scale = 1;
  }
  int32_t pivot_x = p[22] / 2, pivot_y = p[23] / 2;
  if (p[0] == TINYSG_LINE) {
    pivot_x = (p[22] - p[20]) / 2;
    pivot_y = (p[23] - p[21]) / 2;
  }
  if (p[0] == TINYSG_CIRCLE || p[0] == TINYSG_CIRCLE_OUTLINE || p[0] == TINYSG_ELLIPSE || p[0] == TINYSG_ARC ||
      p[0] == TINYSG_PIE || p[0] == TINYSG_TRIANGLE)
    pivot_x = pivot_y = 0;
  if (p[0] == TINYSG_TEXT) {
    const tinysg_font *font = nodes[0].font;
    unsigned size = font->format ? nodes[0].font_size : nodes[0].scale;
    pivot_x = (int32_t)tinysg_text_width(font, text, size) / 2;
    pivot_y =
        (int32_t)(font->format ? (font->height * size + font->nominal - 1) / font->nominal : font->height * size) / 2;
  }
  tinysg_transform transform = {.angle = (int16_t)p[4],
                                .skew = (int16_t)p[5],
                                .zoom_x = (uint16_t)p[6],
                                .zoom_y = (uint16_t)p[6],
                                .pivot_x = (int16_t)pivot_x,
                                .pivot_y = (int16_t)pivot_y,
                                .wave_amplitude = (int16_t)p[7],
                                .wave_period = (uint16_t)p[30],
                                .wave_phase = (int16_t)p[31]};
  if (p[4] || p[5] || p[6] != 256 || p[7]) nodes[0].transform = &transform;
  if (p[0] == TINYSG_IMAGE) {
    nodes[0].pixels = image_pixels;
    nodes[0].pixel_bytes = sizeof(image_pixels);
  }
  tinysg_scene scene = {nodes, 1, 0x0862};
  return tinysg_render(&scene, 320, 180, scratch, sizeof(scratch), output, NULL) ? rgba : NULL;
}

#if defined(__wasm__)
void *memchr(const void *source, int value, size_t count) {
  const uint8_t *bytes = source;
  for (size_t i = 0; i < count; i++)
    if (bytes[i] == (uint8_t)value) return (void *)(bytes + i);
  return NULL;
}

void *memcpy(void *destination, const void *source, size_t count) {
  uint8_t *out = destination;
  const uint8_t *in = source;
  for (size_t i = 0; i < count; i++) out[i] = in[i];
  return destination;
}

void *memset(void *destination, int value, size_t count) {
  uint8_t *out = destination;
  for (size_t i = 0; i < count; i++) out[i] = (uint8_t)value;
  return destination;
}

size_t strlen(const char *value) {
  size_t length = 0;
  while (value[length]) length++;
  return length;
}

int strcmp(const char *a, const char *b) {
  while (*a && *a == *b) {
    a++;
    b++;
  }
  return (unsigned char)*a - (unsigned char)*b;
}

int memcmp(const void *a, const void *b, size_t count) {
  const uint8_t *left = a, *right = b;
  for (size_t i = 0; i < count; i++) {
    if (left[i] != right[i]) return left[i] - right[i];
  }
  return 0;
}
#endif
