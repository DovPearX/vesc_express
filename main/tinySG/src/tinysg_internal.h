#ifndef TINYSG_INTERNAL_H
#define TINYSG_INTERNAL_H
#include "tinysg.h"
uint32_t tinysg_integer_sqrt(uint32_t value);
unsigned tinysg_image_alpha(const tinysg_node *node, int32_t x, int32_t y);
uint16_t tinysg_image_pixel(const tinysg_node *node, int32_t x, int32_t y);
unsigned tinysg_stroke(const tinysg_node *node);
bool tinysg_extra_span(const tinysg_node *node, int32_t y, int32_t *left, int32_t *right);
bool tinysg_extra_hole(const tinysg_node *node, int32_t y, int32_t *left, int32_t *right);
bool tinysg_shape_pixel(const tinysg_node *node, int32_t x, int32_t y);
void tinysg_paint_style(const tinysg_node *node, int32_t x, int32_t y, size_t count, uint8_t *pixels);
bool tinysg_style_pixel_bounds(const tinysg_node *node, const tinysg_area *bounds, int32_t x, int32_t y,
                               uint16_t *color);
unsigned tinysg_opacity(const tinysg_node *node);
bool tinysg_transform_active(const tinysg_node *node);
tinysg_area tinysg_local_bounds(const tinysg_node *node);
tinysg_area tinysg_effect_bounds(const tinysg_node *node, tinysg_area area);
unsigned tinysg_text_alpha(const tinysg_node *node, int32_t x, int32_t y);
void tinysg_paint_text(const tinysg_node *node, tinysg_area tile, uint8_t *pixels);
void tinysg_paint_effect(const tinysg_node *node, tinysg_area tile, uint8_t *pixels);
bool tinysg_accelerated_fill(uint8_t *pixels, size_t count, uint16_t color);
#endif
