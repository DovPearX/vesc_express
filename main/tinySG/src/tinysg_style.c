#include "tinysg_internal.h"

bool tinysg_fill_supported(tinysg_kind kind, tinysg_fill fill) {
  return kind <= TINYSG_IMAGE && fill <= TINYSG_FILL_GRADIENT && (kind != TINYSG_IMAGE || fill == TINYSG_FILL_SOLID);
}

unsigned tinysg_stroke(const tinysg_node *node) {
  unsigned stroke = node->style ? node->style->stroke_width : 1;
  if (!stroke) return 1;
  return stroke > 32 ? 32 : stroke;
}

static int32_t floor_ratio(int64_t value, int64_t divisor) {
  int32_t result = (int32_t)(value / divisor);
  return result - (value % divisor < 0);
}

static uint16_t blend(uint16_t a, uint16_t b, unsigned t, unsigned length) {
  unsigned r = (((uint64_t)(a >> 11) * (length - t)) + ((uint64_t)(b >> 11) * t)) / length;
  unsigned g = (((uint64_t)((a >> 5) & 63) * (length - t)) + ((uint64_t)((b >> 5) & 63) * t)) / length;
  unsigned blue = (((uint64_t)(a & 31) * (length - t)) + ((uint64_t)(b & 31) * t)) / length;
  return (uint16_t)((r << 11) | (g << 5) | blue);
}

static bool dash_visible(const tinysg_node *node, int32_t x, int32_t y) {
  const tinysg_style *style = node->style;
  if (!style->dash_length || node->kind != TINYSG_LINE) return true;
  int32_t dx = x - node->x, dy = y - node->y;
  if (dx < 0) dx = -dx;
  if (dy < 0) dy = -dy;
  unsigned distance = (unsigned)(dx > dy ? dx : dy);
  unsigned period = style->dash_length + style->gap_length;
  return distance % period < style->dash_length;
}

static void gradient_axis(const tinysg_node *node, const tinysg_area *bounds, int32_t x, int32_t y, unsigned *position,
                          unsigned *length, int32_t *step) {
  int32_t cosine = tinysg_cos_deg(node->style->gradient_angle), sine = tinysg_sin_deg(node->style->gradient_angle);
  int64_t width = bounds->width ? bounds->width - 1 : 0, height = bounds->height ? bounds->height - 1 : 0;
  int64_t start = (cosine < 0 ? -cosine * width : 0) + (sine < 0 ? -sine * height : 0);
  int64_t span = (cosine < 0 ? -cosine : cosine) * width + (sine < 0 ? -sine : sine) * height;
  int64_t value = (int64_t)(x - bounds->x) * cosine + (int64_t)(y - bounds->y) * sine + start;
  *length = span > 0 ? (unsigned)span : 1;
  *position = value < 0 ? 0 : value > span ? (unsigned)span : (unsigned)value;
  *step = cosine;
}

static void gradient_pixel(const tinysg_node *node, const tinysg_area *bounds, int32_t x, int32_t y, uint16_t *color) {
  const tinysg_style *style = node->style;
  tinysg_area local;
  if (!bounds) {
    local = tinysg_local_bounds(node);
    bounds = &local;
  }
  if (style->fill == TINYSG_FILL_GRADIENT) {
    unsigned position, length;
    int32_t step;
    gradient_axis(node, bounds, x, y, &position, &length, &step);
    *color = blend(node->color, style->secondary_color, position, length);
    return;
  }
  int32_t position = style->fill == TINYSG_FILL_GRADIENT_X ? x - bounds->x : y - bounds->y;
  unsigned size = style->fill == TINYSG_FILL_GRADIENT_X ? bounds->width : bounds->height;
  if (size < 2) return;
  unsigned t = position < 0 ? 0 : (unsigned)position;
  if (t >= size) t = size - 1;
  *color = blend(node->color, style->secondary_color, t, size - 1);
  return;
}

size_t tinysg_image_size(tinysg_pixel_format format, uint16_t width, uint16_t height) {
  size_t pixels = (size_t)width * height;
  if (format == TINYSG_A1) return (pixels + 7) / 8;
  unsigned step = format == TINYSG_RGB888 ? 3 : format == TINYSG_RGB565 ? 2 : 1;
  return format <= TINYSG_A1 && pixels <= SIZE_MAX / step ? pixels * step : 0;
}

unsigned tinysg_image_alpha(const tinysg_node *node, int32_t x, int32_t y) {
  size_t index = (size_t)(y - node->y) * (uint16_t)node->w + x - node->x;
  if (node->pixel_format == TINYSG_A1) return node->pixels[index / 8] & (128u >> (index & 7)) ? 255 : 0;
  return node->pixel_format == TINYSG_A8 ? node->pixels[index] : 255;
}

uint16_t tinysg_image_pixel(const tinysg_node *node, int32_t x, int32_t y) {
  size_t index = (size_t)(y - node->y) * (uint16_t)node->w + x - node->x;
  if (node->pixel_format == TINYSG_A8 || node->pixel_format == TINYSG_A1) return node->color;
  if (node->pixel_format == TINYSG_RGB888) {
    const uint8_t *p = node->pixels + index * 3;
    return tinysg_rgb(p[0], p[1], p[2]);
  }
  if (node->pixel_format == TINYSG_RGB332) {
    uint8_t p = node->pixels[index];
    return tinysg_rgb((uint8_t)((p >> 5) * 255 / 7), (uint8_t)(((p >> 2) & 7) * 255 / 7), (uint8_t)((p & 3) * 85));
  }
  return (uint16_t)(node->pixels[index * 2] << 8 | node->pixels[index * 2 + 1]);
}

bool tinysg_style_pixel_bounds(const tinysg_node *node, const tinysg_area *bounds, int32_t x, int32_t y,
                               uint16_t *color) {
  const tinysg_style *style = node->style;
  if (!style && node->kind != TINYSG_IMAGE) return true;
  if (style && !dash_visible(node, x, y)) return false;
  if (node->kind == TINYSG_IMAGE) {
    if (x < node->x || y < node->y || x >= node->x + node->w || y >= node->y + node->h) return false;
    *color = tinysg_image_pixel(node, x, y);
    return true;
  }
  if (style->fill == TINYSG_FILL_SOLID) return true;
  gradient_pixel(node, bounds, x, y, color);
  return true;
}

typedef struct {
  int32_t value, step;
  uint32_t error, remainder;
} tinysg_gradient_channel;

static tinysg_gradient_channel gradient_channel(unsigned from, unsigned to, unsigned position, unsigned length,
                                                int32_t advance) {
  int32_t delta = (int32_t)to - (int32_t)from;
  int64_t numerator = (int64_t)from * length + (int64_t)delta * position;
  int64_t increment = (int64_t)delta * advance;
  int32_t step = floor_ratio(increment, length);
  return (tinysg_gradient_channel){.value = (int32_t)(numerator / length),
                                   .error = (uint32_t)(numerator % length),
                                   .step = step,
                                   .remainder = (uint32_t)(increment - (int64_t)step * length)};
}

static void advance_channel(tinysg_gradient_channel *channel, unsigned length) {
  channel->value += channel->step;
  uint64_t error = (uint64_t)channel->error + channel->remainder;
  if (error >= length) {
    error -= length;
    channel->value++;
  }
  channel->error = (uint32_t)error;
}

void tinysg_paint_style(const tinysg_node *node, int32_t x, int32_t y, size_t count, uint8_t *pixels) {
  tinysg_area bounds = tinysg_local_bounds(node);
  tinysg_fill fill = node->style ? node->style->fill : TINYSG_FILL_SOLID;
  unsigned opacity = tinysg_opacity(node);
  unsigned length = bounds.width > 1 ? bounds.width - 1 : 1;
  unsigned position = x > bounds.x ? (unsigned)(x - bounds.x) : 0;
  int32_t step = 1;
  if (fill == TINYSG_FILL_GRADIENT) gradient_axis(node, &bounds, x, y, &position, &length, &step);
  tinysg_gradient_channel red = {0}, green = {0}, blue = {0};
  uint16_t color = node->color;
  if (fill == TINYSG_FILL_GRADIENT_Y) gradient_pixel(node, &bounds, x, y, &color);
  if (fill == TINYSG_FILL_GRADIENT_X || fill == TINYSG_FILL_GRADIENT) {
    uint16_t to = node->style->secondary_color;
    red = gradient_channel(color >> 11, to >> 11, position, length, step);
    green = gradient_channel((color >> 5) & 63, (to >> 5) & 63, position, length, step);
    blue = gradient_channel(color & 31, to & 31, position, length, step);
  }
  for (size_t i = 0; i < count; i++) {
    uint16_t pixel = color;
    if (fill == TINYSG_FILL_GRADIENT_X || fill == TINYSG_FILL_GRADIENT) {
      pixel = (uint16_t)((red.value << 11) | (green.value << 5) | blue.value);
      advance_channel(&red, length);
      advance_channel(&green, length);
      advance_channel(&blue, length);
    }
    int32_t px = x + (int32_t)i;
    if ((node->kind == TINYSG_ARC || node->kind == TINYSG_PIE || node->kind == TINYSG_TRIANGLE ||
         node->kind == TINYSG_TEXT) &&
        !tinysg_shape_pixel(node, px, y))
      continue;
    if (node->style && !dash_visible(node, px, y)) continue;
    if (node->kind == TINYSG_IMAGE) {
      pixel = tinysg_image_pixel(node, px, y);
    }
    unsigned alpha = opacity;
    if (node->kind == TINYSG_IMAGE) alpha = (alpha * tinysg_image_alpha(node, px, y) + 127) / 255;
    if (!alpha) continue;
    if (alpha != 255) {
      uint16_t background = (uint16_t)(pixels[2 * i] << 8 | pixels[2 * i + 1]);
      pixel = tinysg_blend(background, pixel, (uint8_t)alpha);
    }
    pixels[2 * i] = (uint8_t)(pixel >> 8);
    pixels[2 * i + 1] = (uint8_t)pixel;
  }
}
