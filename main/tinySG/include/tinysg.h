#ifndef TINYSG_H
#define TINYSG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  TINYSG_RECT,
  TINYSG_CIRCLE,
  TINYSG_LINE,
  TINYSG_RECT_OUTLINE,
  TINYSG_CIRCLE_OUTLINE,
  TINYSG_ELLIPSE,
  TINYSG_ARC,
  TINYSG_PIE,
  TINYSG_TRIANGLE,
  TINYSG_ROUND_RECT,
  TINYSG_ROUND_RECT_OUTLINE,
  TINYSG_TEXT,
  TINYSG_IMAGE
} tinysg_kind;

typedef enum { TINYSG_FILL_SOLID, TINYSG_FILL_GRADIENT_X, TINYSG_FILL_GRADIENT_Y, TINYSG_FILL_GRADIENT } tinysg_fill;

typedef enum { TINYSG_STYLE_OPACITY = 1 } tinysg_style_flags;

typedef enum { TINYSG_QUALITY_FAST, TINYSG_QUALITY_AA2, TINYSG_QUALITY_AA4 } tinysg_quality;

typedef struct {
  uint16_t secondary_color;
  int16_t gradient_angle;
  uint8_t fill, stroke_width, dash_length, gap_length;
  uint8_t opacity, quality, flags;
} tinysg_style;

typedef struct {
  int16_t angle, skew;
  int16_t pivot_x, pivot_y;
  uint16_t zoom_x, zoom_y;
  int16_t wave_amplitude, wave_phase;
  uint16_t wave_period;
} tinysg_transform;

typedef struct {
  uint8_t width, height, first, count;
  const uint8_t *rows;
  uint8_t format, nominal;
  size_t bytes;
} tinysg_font;

extern const tinysg_font tinysg_font_3x5;
extern const tinysg_font tinysg_font_5x7;
typedef enum { TINYSG_RGB565, TINYSG_RGB332, TINYSG_RGB888, TINYSG_A8, TINYSG_A1 } tinysg_pixel_format;

typedef struct {
  tinysg_kind kind;
  int16_t x, y, w, h;
  uint16_t color;
  const tinysg_style *style;
  int16_t angle_start, angle_sweep;
  uint16_t radius;
  int16_t x2, y2;
  const char *text;
  const tinysg_font *font;
  uint8_t scale, pixel_format;
  uint16_t font_size;
  uint32_t revision;
  const tinysg_transform *transform;
  const uint8_t *pixels;
  size_t pixel_bytes;
  int16_t clip_x, clip_y;
  uint16_t clip_width, clip_height;
} tinysg_node;

typedef struct {
  const tinysg_node *nodes;
  size_t count;
  uint16_t background;
} tinysg_scene;

typedef struct {
  int32_t x, y;
  uint16_t width, height;
} tinysg_area;

typedef enum { TINYSG_FLEX_ROW, TINYSG_FLEX_COLUMN, TINYSG_FLEX_ROW_WRAP, TINYSG_FLEX_COLUMN_WRAP } tinysg_flow;

typedef enum {
  TINYSG_ALIGN_START,
  TINYSG_ALIGN_CENTER,
  TINYSG_ALIGN_END,
  TINYSG_ALIGN_STRETCH,
  TINYSG_ALIGN_SPACE_BETWEEN
} tinysg_align;

typedef struct {
  uint16_t width, height, grow;
  tinysg_area area;
} tinysg_flex_item;

typedef struct {
  tinysg_area area;
  uint16_t gap, padding;
  tinysg_flow flow;
  tinysg_align justify, align;
} tinysg_flex_layout;

typedef bool (*tinysg_sink)(void *ctx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pixels);

typedef void (*tinysg_clear_callback)(uint32_t rgb888);
typedef void (*tinysg_reset_callback)(void);

typedef struct {
  bool (*fill)(void *ctx, uint8_t *pixels, size_t count, uint16_t color);
} tinysg_accelerator;

void tinysg_set_accelerator(const tinysg_accelerator *accelerator, void *ctx);

void tinysg_display_extensions_set_callbacks(tinysg_sink render, tinysg_clear_callback clear,
                                             tinysg_reset_callback reset);
bool tinysg_display_write(void *ctx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pixels);
bool tinysg_display_clear(uint32_t rgb888);
bool tinysg_display_reset(void);

bool tinysg_render(const tinysg_scene *scene, uint16_t width, uint16_t height, uint8_t *scratch, size_t bytes,
                   tinysg_sink sink, void *ctx);
bool tinysg_render_area(const tinysg_scene *scene, uint16_t width, uint16_t height, tinysg_area area, uint8_t *scratch,
                        size_t bytes, tinysg_sink sink, void *ctx);
bool tinysg_render_update(const tinysg_scene *scene, const tinysg_scene *previous, uint16_t width, uint16_t height,
                          uint8_t *dirty, size_t dirty_bytes, uint8_t *scratch, size_t bytes, tinysg_sink sink,
                          void *ctx);
size_t tinysg_image_size(tinysg_pixel_format format, uint16_t width, uint16_t height);
bool tinysg_fill_supported(tinysg_kind kind, tinysg_fill fill);
uint16_t tinysg_rgb(uint8_t r, uint8_t g, uint8_t b);
uint16_t tinysg_blend(uint16_t background, uint16_t foreground, uint8_t opacity);
bool tinysg_grid_cell(tinysg_area area, uint16_t columns, uint16_t rows, uint16_t gap, uint16_t padding, size_t index,
                      tinysg_area *cell);
bool tinysg_layout_flex(const tinysg_flex_layout *layout, tinysg_flex_item *items, size_t count);

int16_t tinysg_sin_deg(int32_t angle);
int16_t tinysg_cos_deg(int32_t angle);
tinysg_area tinysg_node_bounds(const tinysg_node *node);
bool tinysg_font_load(tinysg_font *font, const uint8_t *data, size_t bytes);
uint16_t tinysg_text_width(const tinysg_font *font, const char *text, unsigned size);
bool tinysg_font_pixel(const tinysg_font *font, unsigned character, unsigned x, unsigned y);
bool tinysg_node_equal(const tinysg_node *a, const tinysg_node *b);
size_t tinysg_scene_damage(const tinysg_node *before, size_t before_count, const tinysg_node *after, size_t after_count,
                           tinysg_area *areas, size_t capacity);
#endif
