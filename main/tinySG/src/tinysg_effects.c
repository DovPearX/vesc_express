#include "tinysg_internal.h"
#include <string.h>

typedef struct {
  int64_t xx, xy, yx, yy;
  int32_t pivot_x, pivot_y;
} inverse_map;

typedef struct {
  const tinysg_node *node;
  tinysg_area local;
  inverse_map map;
  int64_t sample_x[16], sample_y[16];
  size_t text_length;
  int32_t wave_step;
  unsigned samples, opacity;
  int16_t offset_x[16], offset_y[16];
} sampler;

static int64_t smaller(int64_t a, int64_t b) {
  return a < b ? a : b;
}

static int64_t larger(int64_t a, int64_t b) {
  return a > b ? a : b;
}

static int32_t floor_q8(int64_t value) {
  int64_t result = value >= 0 ? value / 256 : -((-value + 255) / 256);
  if (result < INT32_MIN) return INT32_MIN;
  return result > INT32_MAX ? INT32_MAX : (int32_t)result;
}

static unsigned zoom(uint16_t value) {
  return value ? value : 256;
}

unsigned tinysg_opacity(const tinysg_node *node) {
  if (!node->style || !(node->style->flags & TINYSG_STYLE_OPACITY)) return 255;
  return node->style->opacity;
}

uint16_t tinysg_blend(uint16_t background, uint16_t foreground, uint8_t opacity) {
  if (!opacity) return background;
  if (opacity == 255) return foreground;
  unsigned inverse = 255 - opacity;
  unsigned r = ((background >> 11) * inverse + (foreground >> 11) * opacity + 127) / 255;
  unsigned g = (((background >> 5) & 63) * inverse + ((foreground >> 5) & 63) * opacity + 127) / 255;
  unsigned b = ((background & 31) * inverse + (foreground & 31) * opacity + 127) / 255;
  return (uint16_t)((r << 11) | (g << 5) | b);
}

bool tinysg_transform_active(const tinysg_node *node) {
  const tinysg_transform *t = node->transform;
  return t && (t->angle % 360 || t->skew || zoom(t->zoom_x) != 256 || zoom(t->zoom_y) != 256 || t->wave_amplitude);
}

static void forward_point(const tinysg_node *node, int64_t x, int64_t y, int64_t *out_x, int64_t *out_y) {
  const tinysg_transform *t = node->transform;
  int64_t px = (int32_t)node->x + t->pivot_x, py = (int32_t)node->y + t->pivot_y;
  x -= px * 256;
  y -= py * 256;
  x = (x + t->skew * y / 256) * zoom(t->zoom_x) / 256;
  y = y * zoom(t->zoom_y) / 256;
  int64_t sine = tinysg_sin_deg(t->angle), cosine = tinysg_cos_deg(t->angle);
  *out_x = px * 256 + (x * cosine - y * sine) / 32767;
  *out_y = py * 256 + (x * sine + y * cosine) / 32767;
}

tinysg_area tinysg_effect_bounds(const tinysg_node *node, tinysg_area area) {
  if (!area.width || !area.height) return area;
  bool active = tinysg_transform_active(node);
  bool aa = node->style && node->style->quality;
  if (!active && !aa) return area;
  int64_t left = area.x, top = area.y, right = left + area.width, bottom = top + area.height;
  if (active) {
    int64_t wave = node->transform->wave_amplitude;
    if (wave < 0) wave = -wave;
    top -= wave;
    bottom += wave;
    int64_t min_x = INT64_MAX, min_y = INT64_MAX, max_x = INT64_MIN, max_y = INT64_MIN;
    for (unsigned i = 0; i < 4; i++) {
      int64_t x, y;
      forward_point(node, (i & 1 ? right : left) * 256, (i & 2 ? bottom : top) * 256, &x, &y);
      min_x = smaller(min_x, x);
      min_y = smaller(min_y, y);
      max_x = larger(max_x, x);
      max_y = larger(max_y, y);
    }
    left = (int64_t)floor_q8(min_x) - 2;
    top = (int64_t)floor_q8(min_y) - 2;
    right = (int64_t)floor_q8(max_x) + 3;
    bottom = (int64_t)floor_q8(max_y) + 3;
  } else {
    left--;
    top--;
    right++;
    bottom++;
  }
  if (right - left > UINT16_MAX) {
    left = larger(left, 0);
    right = smaller(right, UINT16_MAX);
  }
  if (bottom - top > UINT16_MAX) {
    top = larger(top, 0);
    bottom = smaller(bottom, UINT16_MAX);
  }
  if (right <= left || bottom <= top) return (tinysg_area){0};
  left = larger(left, INT32_MIN);
  top = larger(top, INT32_MIN);
  uint16_t width = (uint16_t)(right - left), height = (uint16_t)(bottom - top);
  return (tinysg_area){(int32_t)left, (int32_t)top, width, height};
}

static inverse_map prepare_map(const tinysg_node *node) {
  inverse_map map = {65536, 0, 0, 65536, node->x, node->y};
  const tinysg_transform *t = node->transform;
  if (!t) return map;
  map.pivot_x += t->pivot_x;
  map.pivot_y += t->pivot_y;
  int64_t sine = tinysg_sin_deg(t->angle), cosine = tinysg_cos_deg(t->angle);
  map.yx = -sine * 16777216 / (32767 * (int64_t)zoom(t->zoom_y));
  map.yy = cosine * 16777216 / (32767 * (int64_t)zoom(t->zoom_y));
  map.xx = cosine * 16777216 / (32767 * (int64_t)zoom(t->zoom_x)) - t->skew * map.yx / 256;
  map.xy = sine * 16777216 / (32767 * (int64_t)zoom(t->zoom_x)) - t->skew * map.yy / 256;
  return map;
}

static bool rounded_contains(int64_t x, int64_t y, int64_t width, int64_t height, int64_t radius) {
  if (width <= 0 || height <= 0 || x < -128 || y < -128 || x >= width - 128 || y >= height - 128) return false;
  radius = smaller(radius, smaller(width, height) / 2);
  int64_t dx = larger(radius - x - 128, x - (width - radius - 128));
  int64_t dy = larger(radius - y - 128, y - (height - radius - 128));
  dx = larger(dx, 0);
  dy = larger(dy, 0);
  return dx * dx + dy * dy <= radius * radius;
}

static bool sector_contains(const tinysg_node *node, int64_t x, int64_t y) {
  int32_t sweep = node->angle_sweep, start = node->angle_start;
  if (!sweep) return false;
  if (sweep >= 360 || sweep <= -360) return true;
  if (sweep < 0) {
    start += sweep;
    sweep = -sweep;
  }
  int64_t a = tinysg_cos_deg(start) * y - tinysg_sin_deg(start) * x;
  int64_t b = x * tinysg_sin_deg(start + sweep) - y * tinysg_cos_deg(start + sweep);
  return sweep <= 180 ? a >= 0 && b >= 0 : a >= 0 || b >= 0;
}

static bool line_contains(const tinysg_node *node, int64_t x, int64_t y) {
  int64_t dx = (int32_t)node->w - node->x, dy = (int32_t)node->h - node->y;
  int64_t length = dx * dx + dy * dy;
  int64_t dot = x * dx + y * dy;
  int64_t t = length ? dot * 256 / length : 0;
  t = smaller(larger(t, 0), 65536);
  x -= dx * t / 256;
  y -= dy * t / 256;
  int64_t radius = tinysg_stroke(node) * 128;
  return x * x + y * y <= radius * radius;
}

static bool triangle_contains(const tinysg_node *node, int64_t x, int64_t y) {
  int64_t bx = ((int32_t)node->w - node->x) * 256, by = ((int32_t)node->h - node->y) * 256;
  int64_t cx = ((int32_t)node->x2 - node->x) * 256, cy = ((int32_t)node->y2 - node->y) * 256;
  int64_t a = bx * y - by * x;
  int64_t b = (cx - bx) * (y - by) - (cy - by) * (x - bx);
  int64_t c = cy * x - cx * y;
  return bx * cy != by * cx && ((a >= 0 && b >= 0 && c >= 0) || (a <= 0 && b <= 0 && c <= 0));
}

static bool radial_contains(const tinysg_node *node, int64_t x, int64_t y) {
  int64_t radius = (int32_t)node->w * 256, radius_y = (int32_t)node->h * 256;
  if (radius < 0) return false;
  if (node->kind == TINYSG_ELLIPSE) {
    if (radius_y < 0) return false;
    if (!radius) return x >= -128 && x <= 128 && y >= -radius_y - 128 && y <= radius_y + 128;
    if (!radius_y) return y >= -128 && y <= 128 && x >= -radius - 128 && x <= radius + 128;
    int64_t nx = x * 65536 / radius, ny = y * 65536 / radius_y;
    return nx * nx + ny * ny <= INT64_C(4294967296);
  }
  if (!radius) radius = 128;
  int64_t distance = x * x + y * y;
  if (distance > radius * radius) return false;
  if (node->kind == TINYSG_CIRCLE_OUTLINE || node->kind == TINYSG_ARC) {
    int64_t inner = radius - tinysg_stroke(node) * 256;
    if (inner >= 0 && distance <= inner * inner) return false;
  }
  return node->kind != TINYSG_ARC && node->kind != TINYSG_PIE ? true : sector_contains(node, x, y);
}

static bool text_contains(const tinysg_node *node, int64_t x, int64_t y, size_t length) {
  if (node->font && node->font->format) return tinysg_text_alpha(node, floor_q8(x + 128), floor_q8(y + 128)) != 0;
  const tinysg_font *font = node->font ? node->font : &tinysg_font_5x7;
  unsigned scale = node->scale ? node->scale : 1;
  int32_t px = floor_q8(x + 128) - node->x, py = floor_q8(y + 128) - node->y;
  if (px < 0 || py < 0) return false;
  unsigned cell = font->width + 1, column = (unsigned)px / scale;
  unsigned index = column / cell;
  if (index >= length) return false;
  return tinysg_font_pixel(font, (unsigned char)node->text[index], column % cell, (unsigned)py / scale);
}

static bool contains(const tinysg_node *node, int64_t x, int64_t y, size_t text_length) {
  int64_t dx = x - (int32_t)node->x * 256, dy = y - (int32_t)node->y * 256;
  int64_t width = (int32_t)node->w * 256, height = (int32_t)node->h * 256;
  int64_t stroke = tinysg_stroke(node) * 256;
  switch (node->kind) {
  case TINYSG_RECT:
    return rounded_contains(dx, dy, width, height, 0);
  case TINYSG_RECT_OUTLINE:
    return rounded_contains(dx, dy, width, height, 0) &&
           !rounded_contains(dx - stroke, dy - stroke, width - 2 * stroke, height - 2 * stroke, 0);
  case TINYSG_ROUND_RECT:
    return rounded_contains(dx, dy, width, height, node->radius * 256);
  case TINYSG_ROUND_RECT_OUTLINE:
    return rounded_contains(dx, dy, width, height, node->radius * 256) &&
           !rounded_contains(dx - stroke, dy - stroke, width - 2 * stroke, height - 2 * stroke,
                             larger((int32_t)node->radius - (int32_t)tinysg_stroke(node), 0) * 256);
  case TINYSG_LINE:
    return line_contains(node, dx, dy);
  case TINYSG_TRIANGLE:
    return triangle_contains(node, dx, dy);
  case TINYSG_TEXT:
    return text_contains(node, x, y, text_length);
  default:
    return radial_contains(node, dx, dy);
  }
}

static sampler prepare_sampler(const tinysg_node *node) {
  sampler state = {
      .node = node, .local = tinysg_local_bounds(node), .map = prepare_map(node), .opacity = tinysg_opacity(node)};
  unsigned quality = node->style ? node->style->quality : 0;
  unsigned side = quality == TINYSG_QUALITY_AA4 ? 4 : quality ? 2 : 1;
  state.samples = side * side;
  if (node->kind == TINYSG_TEXT && node->text) state.text_length = strlen(node->text);
  if (node->transform && node->transform->wave_amplitude) {
    unsigned period = node->transform->wave_period ? node->transform->wave_period : 24;
    state.wave_step = 23592960 / period;
  }
  for (unsigned i = 0; i < state.samples; i++) {
    int64_t x = side == 1 ? 0 : ((int64_t)(i % side) * 256 + 128) / side - 128;
    int64_t y = side == 1 ? 0 : ((int64_t)(i / side) * 256 + 128) / side - 128;
    state.offset_x[i] = (int16_t)x;
    state.offset_y[i] = (int16_t)y;
    state.sample_x[i] = x * state.map.xx + y * state.map.xy;
    state.sample_y[i] = x * state.map.yx + y * state.map.yy;
  }
  return state;
}

static bool sample(const sampler *state, int64_t x, int64_t y, uint16_t *color) {
  const tinysg_node *node = state->node;
  if (x < (int64_t)state->local.x * 256 - 128 || x >= ((int64_t)state->local.x + state->local.width) * 256 - 128)
    return false;
  const tinysg_transform *t = node->transform;
  if (state->wave_step) {
    int32_t angle = (int32_t)((x - (int32_t)node->x * 256) * state->wave_step / 16777216) + t->wave_phase;
    y -= (int64_t)tinysg_sin_deg(angle) * t->wave_amplitude * 256 / 32767;
  }
  if (y < (int64_t)state->local.y * 256 - 128 || y >= ((int64_t)state->local.y + state->local.height) * 256 - 128)
    return false;
  if (node->kind != TINYSG_IMAGE && !contains(node, x, y, state->text_length)) return false;
  return tinysg_style_pixel_bounds(node, &state->local, floor_q8(x + 128), floor_q8(y + 128), color);
}

static unsigned sample_alpha(const sampler *state, int64_t x, int64_t y) {
  const tinysg_node *node = state->node;
  if (node->kind == TINYSG_TEXT && node->font && node->font->format)
    return tinysg_text_alpha(node, floor_q8(x + 128), floor_q8(y + 128));
  if (node->kind != TINYSG_IMAGE || (node->pixel_format != TINYSG_A8 && node->pixel_format != TINYSG_A1)) return 255;
  int32_t px = floor_q8(x + 128) - node->x, py = floor_q8(y + 128) - node->y;
  if (px < 0 || py < 0 || px >= node->w || py >= node->h) return 0;
  return tinysg_image_alpha(node, node->x + px, node->y + py);
}

static void paint_coverage(const sampler *state, unsigned hits, uint16_t color, uint8_t *pixels) {
  unsigned divisor = state->samples * 255;
  unsigned alpha = (state->opacity * hits + divisor / 2) / divisor;
  if (!alpha) return;
  if (alpha != 255) {
    uint16_t background = (uint16_t)((pixels[0] << 8) | pixels[1]);
    color = tinysg_blend(background, color, (uint8_t)alpha);
  }
  pixels[0] = (uint8_t)(color >> 8);
  pixels[1] = (uint8_t)color;
}

static void paint_effect_pixel(const sampler *state, int64_t x, int64_t y, uint8_t *pixels) {
  unsigned hits = 0;
  uint16_t color = state->node->color;
  for (unsigned i = 0; i < state->samples; i++) {
    int64_t sx = state->map.pivot_x * INT64_C(256) + (x + state->sample_x[i]) / 65536;
    int64_t sy = state->map.pivot_y * INT64_C(256) + (y + state->sample_y[i]) / 65536;
    if (sample(state, sx, sy, &color)) hits += sample_alpha(state, sx, sy);
  }
  paint_coverage(state, hits, color, pixels);
}

static void triangle_row(const tinysg_node *node, int32_t y, int32_t *left, int32_t *right) {
  int32_t px[3] = {node->x, node->w, node->x2}, py[3] = {node->y, node->h, node->y2};
  int32_t low = INT32_MAX, high = INT32_MIN;
  for (unsigned i = 0; i < 3; i++) {
    unsigned next = (i + 1) % 3;
    if (py[i] >= y - 1 && py[i] <= y + 1) {
      if (px[i] < low) low = px[i];
      if (px[i] > high) high = px[i];
    }
    for (int32_t row = y - 1; row <= y + 1; row += 2) {
      if ((py[i] < row && py[next] < row) || (py[i] > row && py[next] > row) || py[i] == py[next]) continue;
      int32_t x = px[i] + (int32_t)((int64_t)(px[next] - px[i]) * (row - py[i]) / (py[next] - py[i]));
      if (x < low) low = x;
      if (x > high) high = x;
    }
  }
  if (low == INT32_MAX) {
    *right = *left;
    return;
  }
  if (*left < low - 2) *left = low - 2;
  if (*right > high + 3) *right = high + 3;
}

static void paint_flat_row(const sampler *state, int32_t y, int32_t left, int32_t right, int32_t hole_left,
                           int32_t hole_right, tinysg_area tile, uint8_t *pixels) {
  for (int32_t x = left; x < right; x++) {
    if (x >= hole_left && x < hole_right) {
      x = hole_right - 1;
      continue;
    }
    uint16_t color = state->node->color;
    unsigned hits = 0;
    for (unsigned i = 0; i < state->samples; i++) {
      int64_t sx = (int64_t)x * 256 + state->offset_x[i], sy = (int64_t)y * 256 + state->offset_y[i];
      if (sample(state, sx, sy, &color)) hits += sample_alpha(state, sx, sy);
    }
    size_t index = ((size_t)(y - tile.y) * tile.width + x - tile.x) * 2;
    paint_coverage(state, hits, color, pixels + index);
  }
}

static void paint_flat(const sampler *state, tinysg_area bounds, tinysg_area tile, uint8_t *pixels) {
  const tinysg_node *node = state->node;
  int32_t top = (int32_t)larger(tile.y, bounds.y);
  int32_t bottom = (int32_t)smaller((int64_t)tile.y + tile.height, (int64_t)bounds.y + bounds.height);
  for (int32_t y = top; y < bottom; y++) {
    int32_t left = (int32_t)larger(tile.x, bounds.x);
    int32_t right = (int32_t)smaller((int64_t)tile.x + tile.width, (int64_t)bounds.x + bounds.width);
    int32_t hole_left = 0, hole_right = 0;
    bool radial = node->kind == TINYSG_ARC || node->kind == TINYSG_PIE || node->kind == TINYSG_CIRCLE ||
                  node->kind == TINYSG_CIRCLE_OUTLINE;
    if (radial && node->w > 0) {
      int32_t dy = y - node->y;
      if (dy < 0) dy = -dy;
      int32_t near = dy ? dy - 1 : 0, radius = node->w;
      if (near > radius) continue;
      int32_t extent = (int32_t)tinysg_integer_sqrt((uint32_t)(radius * radius - near * near)) + 2;
      if (left < node->x - extent) left = node->x - extent;
      if (right > node->x + extent + 1) right = node->x + extent + 1;
      int32_t inner = radius - (int32_t)tinysg_stroke(node) - 2;
      if ((node->kind == TINYSG_ARC || node->kind == TINYSG_CIRCLE_OUTLINE) && inner > dy + 1) {
        int32_t hole = (int32_t)tinysg_integer_sqrt((uint32_t)(inner * inner - (dy + 1) * (dy + 1)));
        hole_left = node->x - hole;
        hole_right = node->x + hole;
      }
    }
    if (node->kind == TINYSG_TRIANGLE) triangle_row(node, y, &left, &right);
    paint_flat_row(state, y, left, right, hole_left, hole_right, tile, pixels);
  }
}

void tinysg_paint_effect(const tinysg_node *node, tinysg_area tile, uint8_t *pixels) {
  tinysg_area bounds = tinysg_node_bounds(node);
  int32_t left = (int32_t)larger(tile.x, bounds.x), top = (int32_t)larger(tile.y, bounds.y);
  int32_t right = (int32_t)smaller((int64_t)tile.x + tile.width, (int64_t)bounds.x + bounds.width);
  int32_t bottom = (int32_t)smaller((int64_t)tile.y + tile.height, (int64_t)bounds.y + bounds.height);
  if (left >= right || top >= bottom || !tinysg_opacity(node)) return;
  sampler state = prepare_sampler(node);
  if (!tinysg_transform_active(node)) {
    paint_flat(&state, bounds, tile, pixels);
    return;
  }
  for (int32_t y = top; y < bottom; y++) {
    int64_t dx = ((int64_t)left - state.map.pivot_x) * 256, dy = ((int64_t)y - state.map.pivot_y) * 256;
    int64_t sx = dx * state.map.xx + dy * state.map.xy, sy = dx * state.map.yx + dy * state.map.yy;
    for (int32_t x = left; x < right; x++) {
      size_t index = ((size_t)(y - tile.y) * tile.width + x - tile.x) * 2;
      paint_effect_pixel(&state, sx, sy, pixels + index);
      sx += state.map.xx * 256;
      sy += state.map.yx * 256;
    }
  }
}
