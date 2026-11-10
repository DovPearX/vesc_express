#include "tinysg_internal.h"
#include <string.h>

uint16_t tinysg_rgb(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static void fill_span(uint8_t *pixels, size_t count, uint16_t color) {
  if (tinysg_accelerated_fill(pixels, count, color)) return;
  uint8_t high = (uint8_t)(color >> 8);
  uint8_t low = (uint8_t)color;
  for (size_t i = 0; i < count; i++) {
    pixels[2 * i] = high;
    pixels[2 * i + 1] = low;
  }
}

static int32_t ceil_ratio(int64_t value, int32_t divisor) {
  return (int32_t)(value <= 0 ? value / divisor : (value + divisor - 1) / divisor);
}

static bool thin_line_span(const tinysg_node *node, int32_t y, int32_t *left, int32_t *right) {
  int32_t dx = (int32_t)node->w - node->x, dy = (int32_t)node->h - node->y;
  int32_t sx = dx < 0 ? -1 : 1, sy = dy < 0 ? -1 : 1;
  if (dx < 0) dx = -dx;
  if (dy < 0) dy = -dy;
  int32_t row = (y - node->y) * sy;
  if (row < 0 || row > dy) return false;
  if (!dy) {
    *left = node->x < node->w ? node->x : node->w;
    *right = (node->x > node->w ? node->x : node->w) + 1;
    return true;
  }
  if (dy > dx) {
    *left = node->x + sx * (int32_t)(((int64_t)row * dx + dy / 2) / dy);
    *right = *left + 1;
    return true;
  }
  int32_t first = ceil_ratio((int64_t)row * dx - dx / 2, dy);
  int32_t last = ceil_ratio((int64_t)(row + 1) * dx - dx / 2, dy) - 1;
  if (first < 0) first = 0;
  if (last > dx) last = dx;
  *left = node->x + sx * (sx > 0 ? first : last);
  *right = node->x + sx * (sx > 0 ? last : first) + 1;
  return *right > *left;
}

static bool line_span(const tinysg_node *node, int32_t y, int32_t *left, int32_t *right) {
  int32_t before = ((int32_t)tinysg_stroke(node) - 1) / 2;
  int32_t after = (int32_t)tinysg_stroke(node) / 2;
  *left = INT32_MAX;
  *right = INT32_MIN;
  for (int32_t offset = -before; offset <= after; offset++) {
    int32_t a, b;
    if (!thin_line_span(node, y - offset, &a, &b)) continue;
    if (a - before < *left) *left = a - before;
    if (b + after > *right) *right = b + after;
  }
  return *right > *left;
}

static bool node_span(const tinysg_node *node, int32_t y, int32_t *left, int32_t *right) {
  int32_t dy = y - node->y;
  if (node->kind >= TINYSG_ARC) return tinysg_extra_span(node, y, left, right);
  if (node->kind == TINYSG_LINE) {
    return line_span(node, y, left, right);
  }
  if (node->kind == TINYSG_RECT || node->kind == TINYSG_RECT_OUTLINE) {
    *left = node->x;
    *right = (int32_t)node->x + node->w;
    return dy >= 0 && dy < node->h && node->w > 0;
  }
  int32_t radius = node->w;
  int32_t radius_y = node->kind == TINYSG_ELLIPSE ? node->h : radius;
  if ((node->kind != TINYSG_CIRCLE && node->kind != TINYSG_CIRCLE_OUTLINE && node->kind != TINYSG_ELLIPSE) ||
      radius < 0 || radius_y < 0 || dy < -radius_y || dy > radius_y) {
    return false;
  }
  uint32_t square = (uint32_t)(radius * radius);
  if (node->kind != TINYSG_ELLIPSE) {
    square -= (uint32_t)(dy * dy);
  } else if (radius_y) {
    uint32_t square_y = (uint32_t)(radius_y * radius_y);
    square = (uint32_t)((uint64_t)square * (square_y - dy * dy) / square_y);
  }
  int32_t extent = (int32_t)tinysg_integer_sqrt(square);
  *left = node->x - extent;
  *right = node->x + extent + 1;
  return true;
}

static void paint_span(int32_t left, int32_t right, uint16_t x, uint16_t width, uint8_t *pixels,
                       const tinysg_node *node, uint16_t y) {
  if (node->clip_width) {
    if (left < node->clip_x) left = node->clip_x;
    if (right > node->clip_x + node->clip_width) right = node->clip_x + node->clip_width;
  }
  if (left < x) left = x;
  if (right > (int32_t)x + width) right = (int32_t)x + width;
  if (right <= left) return;
  unsigned opacity = tinysg_opacity(node);
  if (!opacity) return;
  if (node->kind == TINYSG_IMAGE && node->pixel_format == TINYSG_RGB565 && opacity == 255) {
    size_t index = ((size_t)(y - node->y) * (uint16_t)node->w + left - node->x) * 2;
    memcpy(pixels + (left - x) * 2, node->pixels + index, (size_t)(right - left) * 2);
    return;
  }
  bool solid = !node->style || (node->style->fill == TINYSG_FILL_SOLID && !node->style->dash_length);
  if (solid &&
      (node->kind < TINYSG_ARC || node->kind == TINYSG_ROUND_RECT || node->kind == TINYSG_ROUND_RECT_OUTLINE) &&
      opacity == 255) {
    fill_span(pixels + (left - x) * 2, (size_t)(right - left), node->color);
    return;
  }
  tinysg_paint_style(node, left, y, (size_t)(right - left), pixels + (left - x) * 2);
}

static bool outline_hole(const tinysg_node *node, uint16_t y, int32_t *left, int32_t *right) {
  if (node->kind >= TINYSG_ARC) return tinysg_extra_hole(node, y, left, right);
  int32_t dy = (int32_t)y - node->y;
  int32_t stroke = (int32_t)tinysg_stroke(node);
  if (node->kind == TINYSG_RECT_OUTLINE && dy >= stroke && dy < node->h - stroke) {
    *left = node->x + stroke;
    *right = node->x + node->w - stroke;
    return *right > *left;
  }
  if (node->kind != TINYSG_CIRCLE_OUTLINE || node->w < stroke) return false;
  int32_t inner = node->w - stroke;
  if (dy < -inner || dy > inner) return false;
  int32_t extent = (int32_t)tinysg_integer_sqrt((uint32_t)(inner * inner - dy * dy));
  *left = node->x - extent;
  *right = node->x + extent + 1;
  return true;
}

static void paint_node(const tinysg_node *node, uint16_t x, uint16_t y, uint16_t width, uint8_t *pixels) {
  int32_t left, right, hole_left, hole_right;
  if (!node_span(node, y, &left, &right)) {
    return;
  }
  if (outline_hole(node, y, &hole_left, &hole_right)) {
    paint_span(left, hole_left, x, width, pixels, node, y);
    paint_span(hole_right, right, x, width, pixels, node, y);
    return;
  }
  paint_span(left, right, x, width, pixels, node, y);
}

static void paint_tile_node(const tinysg_node *node, tinysg_area tile, uint8_t *pixels) {
  if (!tinysg_opacity(node)) return;
  tinysg_area bounds = tinysg_node_bounds(node);
  if ((int64_t)bounds.x + bounds.width <= tile.x || (int64_t)bounds.y + bounds.height <= tile.y ||
      (int64_t)tile.x + tile.width <= bounds.x || (int64_t)tile.y + tile.height <= bounds.y)
    return;
  if (node->kind == TINYSG_TEXT && node->font && node->font->format && !tinysg_transform_active(node)) {
    tinysg_paint_text(node, tile, pixels);
    return;
  }
  if (tinysg_transform_active(node) || (node->style && node->style->quality && node->kind != TINYSG_TEXT)) {
    tinysg_paint_effect(node, tile, pixels);
    return;
  }
  int32_t top = bounds.y > tile.y ? bounds.y : tile.y;
  int32_t bottom = bounds.y + bounds.height < tile.y + tile.height ? bounds.y + bounds.height : tile.y + tile.height;
  for (int32_t y = top; y < bottom; y++) {
    size_t row = (size_t)(y - tile.y) * tile.width * 2;
    paint_node(node, (uint16_t)tile.x, (uint16_t)y, tile.width, pixels + row);
  }
}

static void paint_tile(const tinysg_scene *scene, tinysg_area tile, uint8_t *pixels) {
  fill_span(pixels, (size_t)tile.width * tile.height, scene->background);
  for (size_t i = 0; i < scene->count; i++) paint_tile_node(&scene->nodes[i], tile, pixels);
}

static tinysg_area clip_area(tinysg_area area, uint16_t width, uint16_t height) {
  int64_t right = (int64_t)area.x + area.width;
  int64_t bottom = (int64_t)area.y + area.height;
  int32_t left = area.x < 0 ? 0 : area.x;
  int32_t top = area.y < 0 ? 0 : area.y;
  if (right > width) {
    right = width;
  }
  if (bottom > height) {
    bottom = height;
  }
  if (right <= left || bottom <= top) {
    return (tinysg_area){0};
  }
  return (tinysg_area){left, top, (uint16_t)(right - left), (uint16_t)(bottom - top)};
}

static bool render_strip(const tinysg_scene *scene, tinysg_area strip, uint16_t tile_width, uint8_t *scratch,
                         tinysg_sink sink, void *ctx) {
  uint32_t right = (uint32_t)strip.x + strip.width;
  for (uint32_t x = (uint32_t)strip.x; x < right; x += tile_width) {
    uint16_t w = right - x < tile_width ? (uint16_t)(right - x) : tile_width;
    tinysg_area tile = {(int32_t)x, strip.y, w, strip.height};
    paint_tile(scene, tile, scratch);
    if (!sink(ctx, (uint16_t)x, (uint16_t)strip.y, w, strip.height, scratch)) return false;
  }
  return true;
}

bool tinysg_render_area(const tinysg_scene *scene, uint16_t width, uint16_t height, tinysg_area area, uint8_t *scratch,
                        size_t bytes, tinysg_sink sink, void *ctx) {
  if (!scene || (!scene->nodes && scene->count) || !width || !height || !scratch || bytes < 2 || !sink) {
    return false;
  }
  for (size_t i = 0; i < scene->count; i++) {
    const tinysg_node *node = scene->nodes + i;
    if (node->kind == TINYSG_IMAGE &&
        (!node->pixels || node->w <= 0 || node->h <= 0 || node->pixel_format > TINYSG_A1 ||
         node->pixel_bytes <
             tinysg_image_size((tinysg_pixel_format)node->pixel_format, (uint16_t)node->w, (uint16_t)node->h) ||
         (node->style && node->style->quality)))
      return false;
    if (!node->style) continue;
    if (!tinysg_fill_supported(node->kind, node->style->fill)) return false;
    if (node->style->dash_length && node->kind != TINYSG_LINE) return false;
  }
  area = clip_area(area, width, height);
  if (!area.width || !area.height) {
    return true;
  }
  size_t capacity = bytes / 2;
  uint16_t tile_width = capacity < area.width ? (uint16_t)capacity : area.width;
  size_t rows = capacity / tile_width;
  uint16_t tile_height = rows < area.height ? (uint16_t)rows : area.height;
  uint32_t bottom = (uint32_t)area.y + area.height;
  for (uint32_t y = (uint32_t)area.y; y < bottom; y += tile_height) {
    uint16_t h = bottom - y < tile_height ? (uint16_t)(bottom - y) : tile_height;
    tinysg_area strip = {area.x, (int32_t)y, area.width, h};
    if (!render_strip(scene, strip, tile_width, scratch, sink, ctx)) return false;
  }
  return true;
}

bool tinysg_render(const tinysg_scene *scene, uint16_t width, uint16_t height, uint8_t *scratch, size_t bytes,
                   tinysg_sink sink, void *ctx) {
  return tinysg_render_area(scene, width, height, (tinysg_area){0, 0, width, height}, scratch, bytes, sink, ctx);
}
