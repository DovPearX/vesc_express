#include "tinysg_internal.h"
#include <string.h>

static uint16_t limited_size(uint32_t size) {
  return size > UINT16_MAX ? UINT16_MAX : (uint16_t)size;
}

static int32_t smaller(int32_t a, int32_t b) {
  return a < b ? a : b;
}
static int32_t larger(int32_t a, int32_t b) {
  return a > b ? a : b;
}

static tinysg_area text_bounds(const tinysg_node *node) {
  const tinysg_font *font = node->font ? node->font : &tinysg_font_5x7;
  unsigned scale = node->scale ? node->scale : 1;
  size_t length = node->text ? strlen(node->text) : 0;
  uint32_t width = length > 1024 ? UINT16_MAX : (uint32_t)length * (font->width + 1) * scale;
  uint32_t height = font->height * scale;
  if (font->format) {
    unsigned size = node->font_size ? node->font_size : font->nominal;
    width = tinysg_text_width(font, node->text, size);
    height = (font->height * size + font->nominal - 1) / font->nominal;
  }
  if (node->w > 0) width = (uint32_t)node->w;
  if (node->h > 0) height = (uint32_t)node->h;
  return (tinysg_area){node->x, node->y, limited_size(width), limited_size(height)};
}

static tinysg_area segment_bounds(const tinysg_node *node) {
  int32_t left = smaller(node->x, node->w), top = smaller(node->y, node->h);
  int32_t right = larger(node->x, node->w), bottom = larger(node->y, node->h);
  int32_t pad = (int32_t)tinysg_stroke(node) / 2;
  return (tinysg_area){left - pad, top - pad, limited_size((uint32_t)(right - left + 2 * pad + 1)),
                       limited_size((uint32_t)(bottom - top + 2 * pad + 1))};
}

static tinysg_area triangle_bounds(const tinysg_node *node) {
  int32_t left = smaller(smaller(node->x, node->w), node->x2);
  int32_t top = smaller(smaller(node->y, node->h), node->y2);
  int32_t right = larger(larger(node->x, node->w), node->x2);
  int32_t bottom = larger(larger(node->y, node->h), node->y2);
  return (tinysg_area){left, top, limited_size((uint32_t)(right - left + 1)),
                       limited_size((uint32_t)(bottom - top + 1))};
}

static tinysg_area radial_bounds(const tinysg_node *node) {
  int32_t radius_y = node->kind == TINYSG_ELLIPSE ? node->h : node->w;
  if (node->w < 0 || radius_y < 0) return (tinysg_area){0};
  return (tinysg_area){node->x - node->w, node->y - radius_y, limited_size((uint32_t)node->w * 2 + 1),
                       limited_size((uint32_t)radius_y * 2 + 1)};
}

tinysg_area tinysg_local_bounds(const tinysg_node *node) {
  switch (node->kind) {
  case TINYSG_TEXT:
    return text_bounds(node);
  case TINYSG_LINE:
    return segment_bounds(node);
  case TINYSG_TRIANGLE:
    return triangle_bounds(node);
  case TINYSG_CIRCLE:
  case TINYSG_CIRCLE_OUTLINE:
  case TINYSG_ELLIPSE:
  case TINYSG_ARC:
  case TINYSG_PIE:
    return radial_bounds(node);
  default:
    return (tinysg_area){node->x, node->y, node->w > 0 ? (uint16_t)node->w : 0, node->h > 0 ? (uint16_t)node->h : 0};
  }
}

tinysg_area tinysg_node_bounds(const tinysg_node *node) {
  tinysg_area area = tinysg_effect_bounds(node, tinysg_local_bounds(node));
  if (!node->clip_width) return area;
  int32_t right = smaller(area.x + area.width, node->clip_x + node->clip_width);
  int32_t bottom = smaller(area.y + area.height, node->clip_y + node->clip_height);
  area.x = larger(area.x, node->clip_x);
  area.y = larger(area.y, node->clip_y);
  area.width = right > area.x ? (uint16_t)(right - area.x) : 0;
  area.height = bottom > area.y ? (uint16_t)(bottom - area.y) : 0;
  return area;
}

static bool styles_equal(const tinysg_style *a, const tinysg_style *b) {
  if (a == b) return true;
  if (!a || !b) return false;
  return a->fill == b->fill && a->gradient_angle == b->gradient_angle && a->secondary_color == b->secondary_color &&
         a->stroke_width == b->stroke_width && a->dash_length == b->dash_length && a->gap_length == b->gap_length &&
         a->opacity == b->opacity && a->quality == b->quality && a->flags == b->flags;
}

static bool transforms_equal(const tinysg_transform *a, const tinysg_transform *b) {
  if (a == b) return true;
  if (!a || !b) return false;
  return a->angle == b->angle && a->skew == b->skew && a->pivot_x == b->pivot_x && a->pivot_y == b->pivot_y &&
         a->zoom_x == b->zoom_x && a->zoom_y == b->zoom_y && a->wave_amplitude == b->wave_amplitude &&
         a->wave_phase == b->wave_phase && a->wave_period == b->wave_period;
}

static bool text_equal(const char *a, const char *b) {
  if (a == b) return true;
  return a && b && !strcmp(a, b);
}

static bool fonts_equal(const tinysg_font *a, const tinysg_font *b) {
  if (a == b) return true;
  return a && b && a->rows == b->rows && a->bytes == b->bytes && a->format == b->format && a->width == b->width &&
         a->height == b->height && a->first == b->first && a->count == b->count && a->nominal == b->nominal;
}

bool tinysg_node_equal(const tinysg_node *a, const tinysg_node *b) {
  return a->kind == b->kind && a->x == b->x && a->y == b->y && a->w == b->w && a->h == b->h && a->color == b->color &&
         styles_equal(a->style, b->style) && a->angle_start == b->angle_start && a->angle_sweep == b->angle_sweep &&
         a->radius == b->radius && a->x2 == b->x2 && a->y2 == b->y2 && text_equal(a->text, b->text) &&
         fonts_equal(a->font, b->font) && a->font_size == b->font_size && a->scale == b->scale &&
         a->revision == b->revision && transforms_equal(a->transform, b->transform) && a->clip_x == b->clip_x &&
         a->clip_y == b->clip_y && a->clip_width == b->clip_width && a->clip_height == b->clip_height &&
         a->pixels == b->pixels && a->pixel_bytes == b->pixel_bytes && a->pixel_format == b->pixel_format;
}

static bool rounded_span(int32_t x, int32_t top, int32_t w, int32_t h, int32_t radius, int32_t y, int32_t *left,
                         int32_t *right) {
  if (w <= 0 || h <= 0 || y < top || y >= top + h) return false;
  if (radius > w / 2) radius = w / 2;
  if (radius > h / 2) radius = h / 2;
  int32_t dy = y - top;
  int32_t edge = dy < h - 1 - dy ? dy : h - 1 - dy;
  int32_t inset = 0;
  if (edge < radius) {
    int32_t d = radius - edge;
    inset = radius - (int32_t)tinysg_integer_sqrt((uint32_t)(radius * radius - d * d));
  }
  *left = x + inset;
  *right = x + w - inset;
  return *right > *left;
}

static bool arc_span(const tinysg_node *node, int32_t y, int32_t *left, int32_t *right) {
  if (!node->angle_sweep) return false;
  int32_t dy = y - node->y, r = node->w;
  if (r < 0 || dy < -r || dy > r) return false;
  int32_t extent = (int32_t)tinysg_integer_sqrt((uint32_t)(r * r - dy * dy));
  *left = node->x - extent;
  *right = node->x + extent + 1;
  return true;
}

bool tinysg_extra_span(const tinysg_node *node, int32_t y, int32_t *left, int32_t *right) {
  if (node->kind < TINYSG_ARC || node->kind > TINYSG_IMAGE) return false;
  tinysg_area bounds = tinysg_node_bounds(node);
  if (y < bounds.y || y >= bounds.y + bounds.height) return false;
  if (node->kind == TINYSG_ARC || node->kind == TINYSG_PIE) return arc_span(node, y, left, right);
  if (node->kind == TINYSG_ROUND_RECT || node->kind == TINYSG_ROUND_RECT_OUTLINE) {
    return rounded_span(node->x, node->y, node->w, node->h, node->radius, y, left, right);
  }
  *left = bounds.x;
  *right = bounds.x + bounds.width;
  return bounds.width != 0;
}

static bool arc_hole(const tinysg_node *node, int32_t y, int32_t *left, int32_t *right) {
  int32_t stroke = (int32_t)tinysg_stroke(node);
  int32_t r = node->w - stroke, dy = y - node->y;
  if (r < 0 || dy < -r || dy > r) return false;
  int32_t extent = (int32_t)tinysg_integer_sqrt((uint32_t)(r * r - dy * dy));
  *left = node->x - extent;
  *right = node->x + extent + 1;
  return true;
}

bool tinysg_extra_hole(const tinysg_node *node, int32_t y, int32_t *left, int32_t *right) {
  int32_t stroke = (int32_t)tinysg_stroke(node);
  if (node->kind == TINYSG_ARC) return arc_hole(node, y, left, right);
  if (node->kind != TINYSG_ROUND_RECT_OUTLINE) return false;
  int32_t radius = node->radius > stroke ? node->radius - stroke : 0;
  return rounded_span(node->x + stroke, node->y + stroke, node->w - 2 * stroke, node->h - 2 * stroke, radius, y, left,
                      right);
}

static int64_t cross(int32_t ax, int32_t ay, int32_t bx, int32_t by) {
  return (int64_t)ax * by - (int64_t)ay * bx;
}

static bool angle_visible(const tinysg_node *node, int32_t x, int32_t y) {
  int32_t sweep = node->angle_sweep, start = node->angle_start;
  if (!sweep) return false;
  if (sweep >= 360 || sweep <= -360) return true;
  if (sweep < 0) {
    start += sweep;
    sweep = -sweep;
  }
  int32_t dx = x - node->x, dy = y - node->y;
  int64_t a = cross(tinysg_cos_deg(start), tinysg_sin_deg(start), dx, dy);
  int64_t b = cross(dx, dy, tinysg_cos_deg(start + sweep), tinysg_sin_deg(start + sweep));
  return sweep <= 180 ? a >= 0 && b >= 0 : a >= 0 || b >= 0;
}

static bool triangle_visible(const tinysg_node *node, int32_t x, int32_t y) {
  int64_t a = cross(node->w - node->x, node->h - node->y, x - node->x, y - node->y);
  int64_t b = cross(node->x2 - node->w, node->y2 - node->h, x - node->w, y - node->h);
  int64_t c = cross(node->x - node->x2, node->y - node->y2, x - node->x2, y - node->y2);
  int64_t area = cross(node->w - node->x, node->h - node->y, node->x2 - node->x, node->y2 - node->y);
  return area && ((a >= 0 && b >= 0 && c >= 0) || (a <= 0 && b <= 0 && c <= 0));
}

static bool text_visible(const tinysg_node *node, int32_t x, int32_t y) {
  if (!node->text) return false;
  const tinysg_font *font = node->font ? node->font : &tinysg_font_5x7;
  unsigned scale = node->scale ? node->scale : 1;
  unsigned px = (unsigned)(x - node->x) / scale;
  unsigned py = (unsigned)(y - node->y) / scale;
  unsigned cell = font->width + 1;
  if (px / cell >= strlen(node->text)) return false;
  return tinysg_font_pixel(font, (unsigned char)node->text[px / cell], px % cell, py);
}

bool tinysg_shape_pixel(const tinysg_node *node, int32_t x, int32_t y) {
  if (node->kind == TINYSG_IMAGE) return x >= node->x && y >= node->y && x < node->x + node->w && y < node->y + node->h;
  switch (node->kind) {
  case TINYSG_ARC:
  case TINYSG_PIE:
    return angle_visible(node, x, y);
  case TINYSG_TRIANGLE:
    return triangle_visible(node, x, y);
  case TINYSG_TEXT:
    return text_visible(node, x, y);
  default:
    return true;
  }
}
