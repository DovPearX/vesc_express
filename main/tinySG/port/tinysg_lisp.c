#include "tinysg_lisp.h"
#include "tinysg.h"
#include "tinysg_image.h"
#include "lispbm.h"
#include <stdlib.h>
#include <string.h>
#ifdef TINYSG_ESP32P4
#include "tinysg_esp32p4.h"
#endif

typedef struct {
  tinysg_node *nodes;
  tinysg_style *styles;
  tinysg_transform *transforms;
  char *text;
  size_t count, text_used;
  uint16_t background;
  tinysg_font fonts[8];
  size_t font_count;
} tinysg_frame;

static tinysg_frame frames[2];
static unsigned current_frame;
typedef struct {
  size_t buffer, nodes, text, dirty, layout;
  tinysg_quality quality;
} tinysg_settings;

static const tinysg_settings defaults = {1024, 64, 512, 512, 16, TINYSG_QUALITY_FAST};
static tinysg_settings settings;
static uint8_t *storage, *pixel_buffer, *scratch, *dirty;
static size_t storage_size, scratch_size;
static tinysg_flex_item *layout_items;
static tinysg_area screen;
static bool frame_valid;
static uint32_t transfer_bytes, transfer_tiles;
static const char *const kinds[] = {"rect", "circle", "line",     "rect-outline", "circle-outline",     "ellipse",
                                    "arc",  "pie",    "triangle", "round-rect",   "round-rect-outline", "text",
                                    "image"};
static const char *const fills[] = {"solid", "gradient-x", "gradient-y", "gradient"};

static bool tinysg_take(lbm_value *list, lbm_value *value) {
  if (!lbm_is_cons(*list)) return false;
  *value = lbm_car(*list);
  *list = lbm_cdr(*list);
  return true;
}

static bool tinysg_numbers(lbm_value *list, int32_t *values, size_t count, int32_t low, int32_t high) {
  for (size_t i = 0; i < count; i++) {
    lbm_value value;
    if (!tinysg_take(list, &value) || !lbm_is_number(value)) return false;
    double number = lbm_dec_as_double(value);
    if (!(number >= low && number <= high)) return false;
    values[i] = (int32_t)number;
  }
  return true;
}

static int tinysg_symbol_index(lbm_value value, const char *const *names, size_t count) {
  if (!lbm_is_symbol(value)) return -1;
  const char *name = lbm_get_name_by_symbol(lbm_dec_sym(value));
  if (!name) return -1;
  for (size_t i = 0; i < count; i++) {
    if (!strcmp(name, names[i])) return (int)i;
  }
  return -1;
}

static bool tinysg_parse_style(lbm_value value, tinysg_style *style) {
  lbm_value fill;
  int32_t color, p[5];
  if (!tinysg_take(&value, &fill)) return false;
  int index = tinysg_symbol_index(fill, fills, 4);
  if (index < 0 || !tinysg_numbers(&value, &color, 1, 0, UINT16_MAX) || !tinysg_numbers(&value, p, 4, 0, 255) ||
      !tinysg_numbers(&value, p + 4, 1, 0, TINYSG_QUALITY_AA4))
    return false;
  *style = (tinysg_style){.fill = (tinysg_fill)index,
                          .secondary_color = (uint16_t)color,
                          .stroke_width = (uint8_t)p[0],
                          .dash_length = (uint8_t)p[1],
                          .gap_length = (uint8_t)p[2],
                          .opacity = (uint8_t)p[3],
                          .quality = (uint8_t)p[4],
                          .flags = TINYSG_STYLE_OPACITY};
  if (!lbm_is_symbol_nil(value)) {
    int32_t angle;
    if (!tinysg_numbers(&value, &angle, 1, INT16_MIN, INT16_MAX)) return false;
    style->gradient_angle = (int16_t)angle;
  }
  return lbm_is_symbol_nil(value);
}

static const tinysg_font *tinysg_cached_font(lbm_value value, tinysg_frame *frame) {
  lbm_array_header_t *array = lbm_dec_array_r(value);
  if (!array) return NULL;
  size_t index = 0;
  while (index < frame->font_count && frame->fonts[index].rows != (const uint8_t *)array->data) index++;
  if (index == frame->font_count) {
    if (index == 8 || !tinysg_font_load(frame->fonts + index, (const uint8_t *)array->data, array->size)) return NULL;
    frame->font_count++;
  }
  return frame->fonts + index;
}

static bool tinysg_font_property(lbm_value list, lbm_value *property) {
  lbm_value value;
  int32_t size;
  if (!tinysg_take(&list, &value)) return false;
  if (lbm_is_number(value)) {
    double scale = lbm_dec_as_double(value);
    if (!(scale >= 1 && scale <= 8) || !lbm_is_symbol_nil(list)) return false;
    *property = lbm_enc_i((int32_t)scale);
    return true;
  }
  lbm_array_header_t *array = lbm_dec_array_r(value);
  tinysg_font font;
  if (!array || !tinysg_font_load(&font, (const uint8_t *)array->data, array->size) ||
      !tinysg_numbers(&list, &size, 1, 1, 512) || !lbm_is_symbol_nil(list))
    return false;
  *property = lbm_heap_allocate_list_init(2, value, lbm_enc_i(size));
  return lbm_is_cons(*property);
}

static bool tinysg_parse_text(lbm_value *list, tinysg_node *node, tinysg_frame *frame) {
  lbm_value value;
  char *text;
  size_t capacity;
  int32_t p[2];
  if (!tinysg_take(list, &value) || !lbm_dec_str_size(value, &text, &capacity)) return false;
  const char *end = memchr(text, 0, capacity);
  lbm_value font_value;
  if (!end || !tinysg_take(list, &font_value) || !tinysg_numbers(list, p + 1, 1, 1, 512)) return false;
  if (lbm_is_array_r(font_value)) {
    node->font = tinysg_cached_font(font_value, frame);
    if (!node->font) return false;
    node->font_size = (uint16_t)p[1];
    node->scale = 1;
  } else {
    if (!lbm_is_number(font_value) || p[1] > 8) return false;
    p[0] = lbm_dec_as_i32(font_value);
    if (p[0] != 3 && p[0] != 5) return false;
    node->font = p[0] == 3 ? &tinysg_font_3x5 : &tinysg_font_5x7;
    node->scale = (uint8_t)p[1];
  }
  size_t bytes = (size_t)(end - text) + 1;
  if (bytes > settings.text - frame->text_used) return false;
  node->text = frame->text + frame->text_used;
  memcpy(frame->text + frame->text_used, text, bytes);
  frame->text_used += bytes;
  return true;
}

static bool tinysg_parse_details(lbm_value *list, tinysg_node *node, tinysg_frame *frame) {
  int32_t p[2];
  switch (node->kind) {
  case TINYSG_IMAGE: {
    lbm_value pixels, revision;
    int32_t offset, format;
    if (!tinysg_take(list, &pixels) || !tinysg_take(list, &revision) || !lbm_is_number(revision) ||
        !tinysg_numbers(list, &offset, 1, 0, INT32_MAX) || !tinysg_numbers(list, &format, 1, 0, TINYSG_A1))
      return false;
    lbm_array_header_t *array = lbm_dec_array_r(pixels);
    size_t image_bytes = tinysg_image_size((tinysg_pixel_format)format, (uint16_t)node->w, (uint16_t)node->h);
    if (!array || node->w <= 0 || node->h <= 0 || (size_t)offset > array->size || array->size - offset < image_bytes)
      return false;
    node->pixels = (const uint8_t *)array->data + offset;
    node->pixel_bytes = array->size - offset;
    node->pixel_format = (uint8_t)format;
    node->revision = lbm_dec_as_u32(revision);
    return true;
  }
  case TINYSG_ARC:
  case TINYSG_PIE:
    if (!tinysg_numbers(list, p, 2, INT16_MIN, INT16_MAX)) return false;
    node->angle_start = p[0];
    node->angle_sweep = p[1];
    return true;
  case TINYSG_TRIANGLE:
    if (!tinysg_numbers(list, p, 2, INT16_MIN, INT16_MAX)) return false;
    node->x2 = p[0];
    node->y2 = p[1];
    return true;
  case TINYSG_ROUND_RECT:
  case TINYSG_ROUND_RECT_OUTLINE:
    if (!tinysg_numbers(list, p, 1, 0, INT16_MAX)) return false;
    node->radius = p[0];
    return true;
  case TINYSG_TEXT:
    return tinysg_parse_text(list, node, frame);
  default:
    return true;
  }
}

static bool tinysg_parse_transform(lbm_value list, tinysg_transform *transform) {
  int32_t p[9];
  if (!tinysg_numbers(&list, p, 2, INT16_MIN, INT16_MAX)) return false;
  if (p[1] < -1024 || p[1] > 1024) return false;
  if (!tinysg_numbers(&list, p + 2, 2, 0, 4096)) return false;
  if ((p[2] && p[2] < 16) || (p[3] && p[3] < 16)) return false;
  if (!tinysg_numbers(&list, p + 4, 2, INT16_MIN, INT16_MAX)) return false;
  if (!tinysg_numbers(&list, p + 6, 1, -64, 64)) return false;
  if (!tinysg_numbers(&list, p + 7, 1, 0, 4096)) return false;
  if (!tinysg_numbers(&list, p + 8, 1, INT16_MIN, INT16_MAX) || !lbm_is_symbol_nil(list)) return false;
  *transform = (tinysg_transform){.angle = p[0],
                                  .skew = p[1],
                                  .zoom_x = p[2],
                                  .zoom_y = p[3],
                                  .pivot_x = p[4],
                                  .pivot_y = p[5],
                                  .wave_amplitude = p[6],
                                  .wave_period = p[7],
                                  .wave_phase = p[8]};
  return true;
}

typedef struct {
  tinysg_style style;
  tinysg_transform transform;
  bool filled, rounded, transformed;
  int32_t radius, font, scale;
  lbm_value font_value;
} tinysg_attributes;

static bool tinysg_fill_attribute(lbm_value list, tinysg_style *style) {
  lbm_value fill;
  int32_t color;
  if (!tinysg_take(&list, &fill)) return false;
  int index = tinysg_symbol_index(fill, fills, 4);
  if (index < 0) return false;
  style->fill = (tinysg_fill)index;
  if (index == TINYSG_FILL_SOLID) return lbm_is_symbol_nil(list);
  if (!tinysg_numbers(&list, &color, 1, 0, UINT16_MAX)) return false;
  style->secondary_color = (uint16_t)color;
  if (index == TINYSG_FILL_GRADIENT) {
    if (!tinysg_numbers(&list, &color, 1, INT16_MIN, INT16_MAX)) return false;
    style->gradient_angle = (int16_t)color;
  }
  return lbm_is_symbol_nil(list);
}

static bool tinysg_attribute(lbm_value list, tinysg_attributes *attr, tinysg_kind kind) {
  static const char *const names[] = {"filled", "rounded", "thickness", "dashed", "fill", "opacity", "quality",
                                      "rotate", "pivot",   "skew",      "zoom",   "wave", "font"};
  static const char *const qualities[] = {"fast", "aa2", "aa4"};
  lbm_value name;
  int32_t p[3];
  if (!tinysg_take(&list, &name)) return false;
  int index = tinysg_symbol_index(name, names, sizeof(names) / sizeof(names[0]));
  if (index < 0) return false;
  switch (index) {
  case 0:
    if (kind != TINYSG_RECT && kind != TINYSG_CIRCLE) return false;
    attr->filled = true;
    break;
  case 1:
    if (kind != TINYSG_RECT || !tinysg_numbers(&list, p, 1, 0, INT16_MAX)) return false;
    attr->rounded = true;
    attr->radius = p[0];
    break;
  case 2:
    if (!tinysg_numbers(&list, p, 1, 1, 32)) return false;
    attr->style.stroke_width = (uint8_t)p[0];
    break;
  case 3:
    if (kind != TINYSG_LINE || !tinysg_numbers(&list, p, 2, 0, 255) || !p[0]) return false;
    attr->style.dash_length = (uint8_t)p[0];
    attr->style.gap_length = (uint8_t)p[1];
    break;
  case 4:
    return tinysg_fill_attribute(list, &attr->style);
  case 5:
    if (!tinysg_numbers(&list, p, 1, 0, 255)) return false;
    attr->style.opacity = (uint8_t)p[0];
    break;
  case 6:
    if (!tinysg_take(&list, &name)) return false;
    index = tinysg_symbol_index(name, qualities, 3);
    if (index < 0) return false;
    attr->style.quality = (uint8_t)index;
    break;
  case 7:
    if (!tinysg_numbers(&list, p, 1, INT16_MIN, INT16_MAX)) return false;
    attr->transform.angle = (int16_t)p[0];
    attr->transformed = true;
    break;
  case 8:
    if (!tinysg_numbers(&list, p, 2, INT16_MIN, INT16_MAX)) return false;
    attr->transform.pivot_x = (int16_t)p[0];
    attr->transform.pivot_y = (int16_t)p[1];
    attr->transformed = true;
    break;
  case 9:
    if (!tinysg_numbers(&list, p, 1, -1024, 1024)) return false;
    attr->transform.skew = (int16_t)p[0];
    attr->transformed = true;
    break;
  case 10:
    if (!tinysg_numbers(&list, p, 2, 16, 4096)) return false;
    attr->transform.zoom_x = (uint16_t)p[0];
    attr->transform.zoom_y = (uint16_t)p[1];
    attr->transformed = true;
    break;
  case 11:
    if (!tinysg_numbers(&list, p, 1, -64, 64) || !tinysg_numbers(&list, p + 1, 1, 0, 4096) ||
        !tinysg_numbers(&list, p + 2, 1, INT16_MIN, INT16_MAX))
      return false;
    attr->transform.wave_amplitude = (int16_t)p[0];
    attr->transform.wave_period = (uint16_t)p[1];
    attr->transform.wave_phase = (int16_t)p[2];
    attr->transformed = true;
    break;
  case 12: {
    lbm_value value;
    if (kind != TINYSG_TEXT || !tinysg_take(&list, &value)) return false;
    if (lbm_is_array_r(value)) {
      lbm_array_header_t *array = lbm_dec_array_r(value);
      tinysg_font font;
      if (!tinysg_font_load(&font, (const uint8_t *)array->data, array->size) || !tinysg_numbers(&list, p, 1, 1, 512))
        return false;
      attr->font_value = value;
      attr->scale = p[0];
    } else {
      if (!lbm_is_number(value) || !tinysg_numbers(&list, p + 1, 1, 1, 8)) return false;
      p[0] = lbm_dec_as_i32(value);
      if (p[0] != 3 && p[0] != 5) return false;
      attr->font = p[0];
      attr->scale = p[1];
      attr->font_value = ENC_SYM_NIL;
    }
    break;
  }
  }
  return lbm_is_symbol_nil(list);
}

static bool tinysg_argument(lbm_value value, int32_t *result, int32_t low, int32_t high) {
  if (!lbm_is_number(value)) return false;
  double number = lbm_dec_as_double(value);
  if (!(number >= low && number <= high)) return false;
  *result = (int32_t)number;
  return true;
}

static lbm_value tinysg_make_node(lbm_value *args, lbm_uint argn, tinysg_kind kind) {
  bool sector = kind == TINYSG_ARC || kind == TINYSG_PIE;
  lbm_uint coordinates = 4;
  if (kind == TINYSG_TRIANGLE) coordinates = 6;
  if (sector || kind == TINYSG_CIRCLE) coordinates = 3;
  if (kind == TINYSG_TEXT) coordinates = 2;
  lbm_uint required = coordinates + 1 + (sector ? 2 : kind == TINYSG_TEXT ? 1 : 0);
  if (argn < required) return ENC_SYM_EERROR;
  int32_t p[6] = {0}, color;
  for (lbm_uint i = 0; i < coordinates; i++) {
    if (!tinysg_argument(args[i], &p[i], INT16_MIN, INT16_MAX)) return ENC_SYM_EERROR;
  }
  if (!tinysg_argument(args[coordinates], &color, 0, UINT16_MAX)) return ENC_SYM_EERROR;
  char *text;
  size_t text_bytes;
  if (kind == TINYSG_TEXT && !lbm_dec_str_size(args[3], &text, &text_bytes)) return ENC_SYM_EERROR;
  tinysg_attributes attr = {.style = {.stroke_width = 1, .opacity = 255, .quality = settings.quality},
                            .font = 5,
                            .scale = 1,
                            .font_value = ENC_SYM_NIL};
  for (lbm_uint i = required; i < argn; i++) {
    if (!tinysg_attribute(args[i], &attr, kind)) return ENC_SYM_EERROR;
  }
  tinysg_kind actual = kind;
  if (kind == TINYSG_RECT) actual = attr.filled ? TINYSG_RECT : TINYSG_RECT_OUTLINE;
  if (kind == TINYSG_CIRCLE) actual = attr.filled ? TINYSG_CIRCLE : TINYSG_CIRCLE_OUTLINE;
  if (attr.rounded) actual = attr.filled ? TINYSG_ROUND_RECT : TINYSG_ROUND_RECT_OUTLINE;
  if (!tinysg_fill_supported(actual, attr.style.fill)) return ENC_SYM_EERROR;
  lbm_uint symbol;
  if (!lbm_add_symbol_const(kinds[actual], &symbol)) return ENC_SYM_MERROR;
  lbm_value transform = ENC_SYM_NIL;
  if (attr.transformed) {
    tinysg_transform *t = &attr.transform;
    transform = lbm_heap_allocate_list_init(
        9, lbm_enc_i(t->angle), lbm_enc_i(t->skew), lbm_enc_u(t->zoom_x), lbm_enc_u(t->zoom_y), lbm_enc_i(t->pivot_x),
        lbm_enc_i(t->pivot_y), lbm_enc_i(t->wave_amplitude), lbm_enc_u(t->wave_period), lbm_enc_i(t->wave_phase));
    if (!lbm_is_cons(transform)) return ENC_SYM_MERROR;
  }
  tinysg_style *s = &attr.style;
  lbm_uint fill;
  if (!lbm_add_symbol_const(fills[s->fill], &fill)) return ENC_SYM_MERROR;
  lbm_value style = ENC_SYM_NIL;
  if (s->fill || s->stroke_width != 1 || s->dash_length || s->opacity != 255 || s->quality) {
    style = lbm_heap_allocate_list_init(8, lbm_enc_sym(fill), lbm_enc_u(s->secondary_color), lbm_enc_u(s->stroke_width),
                                        lbm_enc_u(s->dash_length), lbm_enc_u(s->gap_length), lbm_enc_u(s->opacity),
                                        lbm_enc_u(s->quality), lbm_enc_i(s->gradient_angle));
    if (!lbm_is_cons(style)) return ENC_SYM_MERROR;
  }
  lbm_value result = lbm_heap_allocate_list_init(7, lbm_enc_sym(symbol), lbm_enc_i(p[0]), lbm_enc_i(p[1]),
                                                 lbm_enc_i(p[2]), lbm_enc_i(p[3]), lbm_enc_u(color), style);
  if (!lbm_is_cons(result)) return ENC_SYM_MERROR;
  lbm_value tail = result;
  for (unsigned i = 0; i < 6; i++) tail = lbm_cdr(tail);
  lbm_value details = ENC_SYM_NIL;
  if (attr.transformed) {
    details = lbm_cons(transform, ENC_SYM_NIL);
    if (!lbm_is_cons(details)) return ENC_SYM_MERROR;
  }
  if (kind == TINYSG_TEXT) {
    details = lbm_cons(lbm_enc_u(attr.scale), details);
    if (!lbm_is_cons(details)) return ENC_SYM_MERROR;
    details = lbm_cons(lbm_is_array_r(attr.font_value) ? attr.font_value : lbm_enc_u(attr.font), details);
    if (!lbm_is_cons(details)) return ENC_SYM_MERROR;
    details = lbm_cons(args[3], details);
  }
  if (kind == TINYSG_TRIANGLE) {
    details = lbm_cons(lbm_enc_i(p[5]), details);
    if (!lbm_is_cons(details)) return ENC_SYM_MERROR;
    details = lbm_cons(lbm_enc_i(p[4]), details);
  }
  if (sector) {
    int32_t angles[2];
    for (unsigned i = 0; i < 2; i++) {
      if (!lbm_is_number(args[4 + i])) return ENC_SYM_EERROR;
      double value = lbm_dec_as_double(args[4 + i]);
      if (!(value >= INT16_MIN && value <= INT16_MAX)) return ENC_SYM_EERROR;
      angles[i] = (int32_t)value;
    }
    details = lbm_cons(lbm_enc_i(angles[1]), details);
    if (!lbm_is_cons(details)) return ENC_SYM_MERROR;
    details = lbm_cons(lbm_enc_i(angles[0]), details);
  }
  if (attr.rounded) details = lbm_cons(lbm_enc_i(attr.radius), details);
  if (!lbm_is_cons(details) && !lbm_is_symbol_nil(details)) return ENC_SYM_MERROR;
  lbm_set_cdr(tail, details);
  return result;
}

static lbm_value tinysg_rectangle(lbm_value *args, lbm_uint argn) {
  return tinysg_make_node(args, argn, TINYSG_RECT);
}

static lbm_value tinysg_circle(lbm_value *args, lbm_uint argn) {
  return tinysg_make_node(args, argn, TINYSG_CIRCLE);
}

static lbm_value tinysg_line(lbm_value *args, lbm_uint argn) {
  return tinysg_make_node(args, argn, TINYSG_LINE);
}

static lbm_value tinysg_text(lbm_value *args, lbm_uint argn) {
  return tinysg_make_node(args, argn, TINYSG_TEXT);
}

static lbm_value tinysg_arc(lbm_value *args, lbm_uint argn) {
  return tinysg_make_node(args, argn, TINYSG_ARC);
}

static lbm_value tinysg_pie(lbm_value *args, lbm_uint argn) {
  return tinysg_make_node(args, argn, TINYSG_PIE);
}

static lbm_value tinysg_triangle(lbm_value *args, lbm_uint argn) {
  return tinysg_make_node(args, argn, TINYSG_TRIANGLE);
}

static lbm_value tinysg_ellipse(lbm_value *args, lbm_uint argn) {
  return tinysg_make_node(args, argn, TINYSG_ELLIPSE);
}

static lbm_value tinysg_text_size(lbm_value *args, lbm_uint argn) {
  if (argn != 3 || !lbm_is_number(args[2])) return ENC_SYM_EERROR;
  char *text;
  size_t bytes;
  double value = lbm_dec_as_double(args[2]);
  if (!lbm_dec_str_size(args[0], &text, &bytes) || !memchr(text, 0, bytes) || !(value >= 1 && value <= 512))
    return ENC_SYM_EERROR;
  tinysg_font loaded;
  const tinysg_font *font;
  unsigned size = (unsigned)value, height;
  if (lbm_is_array_r(args[1])) {
    lbm_array_header_t *array = lbm_dec_array_r(args[1]);
    if (!tinysg_font_load(&loaded, (const uint8_t *)array->data, array->size)) return ENC_SYM_EERROR;
    font = &loaded;
    height = (font->height * size + font->nominal - 1) / font->nominal;
  } else {
    if (!lbm_is_number(args[1]) || size > 8) return ENC_SYM_EERROR;
    int id = lbm_dec_as_i32(args[1]);
    if (id != 3 && id != 5) return ENC_SYM_EERROR;
    font = id == 3 ? &tinysg_font_3x5 : &tinysg_font_5x7;
    height = font->height * size;
  }
  return lbm_heap_allocate_list_init(2, lbm_enc_u(tinysg_text_width(font, text, size)), lbm_enc_u(height));
}

static lbm_value tinysg_image(lbm_value *args, lbm_uint argn) {
  if (argn < 2) return ENC_SYM_EERROR;
  int32_t p[4] = {0};
  lbm_value box = args[0], pixels = args[1];
  if (!tinysg_numbers(&box, p, 2, INT16_MIN, INT16_MAX)) return ENC_SYM_EERROR;
  if (!lbm_is_symbol_nil(box) && (!tinysg_numbers(&box, p + 2, 2, 1, INT16_MAX) || !lbm_is_symbol_nil(box)))
    return ENC_SYM_EERROR;
  lbm_array_header_t *array = lbm_dec_array_r(pixels);
  if (!array) return ENC_SYM_EERROR;
  tinysg_asset asset;
  size_t offset = 0;
  for (lbm_uint i = 2; i < argn; i++) {
    lbm_value option = args[i], name;
    int32_t number;
    static const char *const source[] = {"source"};
    if (!tinysg_take(&option, &name)) return ENC_SYM_EERROR;
    if (tinysg_symbol_index(name, source, 1) != 0) continue;
    if (!tinysg_numbers(&option, &number, 1, 0, INT32_MAX) || !lbm_is_symbol_nil(option)) return ENC_SYM_EERROR;
    offset = (size_t)number;
  }
  if (offset > array->size) return ENC_SYM_EERROR;
  uint8_t format = TINYSG_RGB565;
  if (tinysg_image_info((const uint8_t *)array->data + offset, array->size - offset, &asset)) {
    if (p[2] && (p[2] != asset.width || p[3] != asset.height)) return ENC_SYM_EERROR;
    p[2] = asset.width;
    p[3] = asset.height;
    format = asset.format;
    if (asset.jpeg) {
      lbm_value decoded;
      if (!lbm_heap_allocate_array(&decoded, (size_t)asset.width * asset.height * 2)) return ENC_SYM_MERROR;
      lbm_array_header_t *out = lbm_dec_array_rw(decoded);
      if (!tinysg_jpeg_decode((const uint8_t *)array->data + offset, array->size - offset, (uint8_t *)out->data,
                              asset.width, asset.height))
        return ENC_SYM_EERROR;
      pixels = decoded;
      array = out;
      offset = 0;
    } else
      offset += 5;
  } else if (!p[2]) {
    lbm_set_error_reason("Expected a RGB .bin image or baseline JPEG");
    return ENC_SYM_EERROR;
  }
  size_t bytes = tinysg_image_size((tinysg_pixel_format)format, (uint16_t)p[2], (uint16_t)p[3]);
  if (offset > array->size || bytes > array->size - offset) return ENC_SYM_EERROR;
  uint32_t revision = 2166136261u;
  const uint8_t *data = (const uint8_t *)array->data + offset;
  for (size_t i = 0; i < bytes; i++) revision = (revision ^ data[i]) * 16777619u;
  int32_t opacity = 255, tint = UINT16_MAX;
  static const char *const names[] = {"opacity", "revision", "tint", "source"};
  for (lbm_uint i = 2; i < argn; i++) {
    lbm_value option = args[i], name;
    int32_t number;
    if (!tinysg_take(&option, &name)) return ENC_SYM_EERROR;
    int index = tinysg_symbol_index(name, names, 4);
    if (index == 3) continue;
    if (index < 0 ||
        !tinysg_numbers(&option, &number, 1, 0,
                        index == 1   ? INT32_MAX
                        : index == 2 ? UINT16_MAX
                                     : 255) ||
        !lbm_is_symbol_nil(option))
      return ENC_SYM_EERROR;
    if (index == 2)
      tint = number;
    else if (index == 1)
      revision = (uint32_t)number;
    else
      opacity = number;
  }
  lbm_uint kind, fill;
  if (!lbm_add_symbol_const("image", &kind) || !lbm_add_symbol_const("solid", &fill)) return ENC_SYM_MERROR;
  lbm_value style = ENC_SYM_NIL;
  if (opacity != 255) {
    style = lbm_heap_allocate_list_init(7, lbm_enc_sym(fill), lbm_enc_i(0), lbm_enc_i(1), lbm_enc_i(0), lbm_enc_i(0),
                                        lbm_enc_i(opacity), lbm_enc_i(0));
    if (!lbm_is_cons(style)) return ENC_SYM_MERROR;
  }
  lbm_value version = lbm_enc_u32(revision);
  if (!lbm_is_number(version)) return ENC_SYM_MERROR;
  return lbm_heap_allocate_list_init(11, lbm_enc_sym(kind), lbm_enc_i(p[0]), lbm_enc_i(p[1]), lbm_enc_i(p[2]),
                                     lbm_enc_i(p[3]), lbm_enc_i(tint), style, pixels, version, lbm_enc_u(offset),
                                     lbm_enc_u(format));
}

static bool tinysg_parse_node(lbm_value list, tinysg_frame *frame) {
  if (frame->count == settings.nodes) return false;
  lbm_value kind, style;
  int32_t p[4], color;
  if (!tinysg_take(&list, &kind)) return false;
  int index = tinysg_symbol_index(kind, kinds, sizeof(kinds) / sizeof(kinds[0]));
  if (index < 0) return false;
  if (!tinysg_numbers(&list, p, 4, INT16_MIN, INT16_MAX)) return false;
  if (!tinysg_numbers(&list, &color, 1, 0, UINT16_MAX) || !tinysg_take(&list, &style)) return false;
  tinysg_node *node = &frame->nodes[frame->count];
  *node = (tinysg_node){.kind = (tinysg_kind)index, .x = p[0], .y = p[1], .w = p[2], .h = p[3], .color = color};
  if (!lbm_is_symbol_nil(style)) {
    if (!tinysg_parse_style(style, &frame->styles[frame->count])) return false;
    node->style = &frame->styles[frame->count];
    if (!tinysg_fill_supported(node->kind, node->style->fill) ||
        (node->style->dash_length && node->kind != TINYSG_LINE))
      return false;
  }
  if (!tinysg_parse_details(&list, node, frame)) return false;
  if (lbm_is_symbol_nil(list)) {
    frame->count++;
    return true;
  }
  lbm_value transform;
  if (!tinysg_take(&list, &transform) || !lbm_is_symbol_nil(list)) return false;
  if (!lbm_is_symbol_nil(transform)) {
    if (!tinysg_parse_transform(transform, &frame->transforms[frame->count])) return false;
    node->transform = &frame->transforms[frame->count];
  }
  frame->count++;
  return true;
}

static lbm_value tinysg_accel(lbm_value *args, lbm_uint argn) {
  static const char *const modes[] = {"auto", "cpu"};
  if (argn != 1) return ENC_SYM_EERROR;
  int mode = tinysg_symbol_index(args[0], modes, 2);
  if (mode < 0) return ENC_SYM_EERROR;
#ifdef TINYSG_ESP32P4
  if (!tinysg_esp32p4_enable(mode == 0)) return ENC_SYM_EERROR;
#endif
  return ENC_SYM_TRUE;
}

static lbm_value tinysg_accel_stats(lbm_value *args, lbm_uint argn) {
  (void)args;
  if (argn) return ENC_SYM_EERROR;
  uint32_t fills = 0, rotations = 0, fallbacks = 0;
#ifdef TINYSG_ESP32P4
  tinysg_ppa_stats stats = tinysg_esp32p4_stats();
  fills = stats.fills;
  rotations = stats.rotations;
  fallbacks = stats.fallbacks;
#endif
  return lbm_heap_allocate_list_init(3, lbm_enc_u32(fills), lbm_enc_u32(rotations), lbm_enc_u32(fallbacks));
}

static size_t aligned_size(size_t bytes) {
  return (bytes + sizeof(void *) - 1) / sizeof(void *) * sizeof(void *);
}

static bool tinysg_allocate(tinysg_settings next) {
  size_t node_bytes = aligned_size(next.nodes * sizeof(tinysg_node));
  size_t style_bytes = aligned_size(next.nodes * sizeof(tinysg_style));
  size_t transform_bytes = aligned_size(next.nodes * sizeof(tinysg_transform));
  size_t text_bytes = aligned_size(next.text);
  size_t frame_bytes = node_bytes + style_bytes + transform_bytes + text_bytes;
  size_t layout_bytes = aligned_size(next.layout * sizeof(tinysg_flex_item));
  size_t bytes = 2 * frame_bytes + layout_bytes + next.dirty;
  bool same = storage && settings.nodes == next.nodes && settings.text == next.text && settings.layout == next.layout &&
              settings.dirty == next.dirty;
  bool same_buffer = pixel_buffer && settings.buffer == next.buffer;
  uint8_t *pixels = pixel_buffer;
  if (!same_buffer) {
#ifdef TINYSG_ESP32P4
    pixels = tinysg_esp32p4_alloc(next.buffer);
#else
    pixels = same ? realloc(pixel_buffer, next.buffer) : malloc(next.buffer);
#endif
  }
  if (!pixels) return false;
  uint8_t *memory = same ? storage : realloc(storage, bytes);
  if (!memory) {
    if (!same_buffer) free(pixels);
    return false;
  }
#ifdef TINYSG_ESP32P4
  if (!same_buffer) free(pixel_buffer);
  tinysg_esp32p4_enable(true);
#else
  if (!same && !same_buffer) free(pixel_buffer);
#endif
  storage = memory;
  storage_size = bytes + next.buffer;
  settings = next;
  for (size_t i = 0; i < 2; i++) {
    frames[i] = (tinysg_frame){.nodes = (tinysg_node *)memory,
                               .styles = (tinysg_style *)(memory + node_bytes),
                               .transforms = (tinysg_transform *)(memory + node_bytes + style_bytes),
                               .text = (char *)(memory + node_bytes + style_bytes + transform_bytes)};
    memory += frame_bytes;
  }
  layout_items = (tinysg_flex_item *)memory;
  dirty = memory + layout_bytes;
  scratch = pixel_buffer = pixels;
  scratch_size = next.buffer;
  current_frame = 0;
  frame_valid = false;
  transfer_bytes = transfer_tiles = 0;
  return true;
}

static bool tinysg_dimensions(lbm_value *args, tinysg_area *area) {
  int32_t p[2];
  for (size_t i = 0; i < 2; i++) {
    if (!lbm_is_number(args[i])) return false;
    double number = lbm_dec_as_double(args[i]);
    if (!(number >= 1 && number <= INT16_MAX) || number != (int32_t)number) return false;
    p[i] = (int32_t)number;
  }
  *area = (tinysg_area){0, 0, p[0], p[1]};
  return true;
}

static bool tinysg_option(lbm_value list, tinysg_settings *next, tinysg_area *area) {
  static const char *const names[] = {"buffer", "nodes", "text", "dirty", "layout", "offset", "quality"};
  lbm_value name;
  int32_t p[2];
  if (!tinysg_take(&list, &name)) return false;
  int index = tinysg_symbol_index(name, names, 7);
  if (index < 0) return false;
  if (index == 6) {
    static const char *const qualities[] = {"fast", "aa2", "aa4"};
    if (!tinysg_take(&list, &name) || !lbm_is_symbol_nil(list)) return false;
    int quality = tinysg_symbol_index(name, qualities, 3);
    if (quality < 0) return false;
    next->quality = (tinysg_quality)quality;
    return true;
  }
  if (index == 5) {
    if (!tinysg_numbers(&list, p, 2, 0, INT16_MAX) || !lbm_is_symbol_nil(list)) return false;
    area->x = p[0];
    area->y = p[1];
    return true;
  }
  static const int32_t limits[] = {1048576, 4096, 65535, 65535, 4096};
  if (!tinysg_numbers(&list, p, 1, index == 0 ? 2 : 1, limits[index]) || !lbm_is_symbol_nil(list)) return false;
  switch (index) {
  case 0:
    next->buffer = (size_t)p[0];
    break;
  case 1:
    next->nodes = (size_t)p[0];
    break;
  case 2:
    next->text = (size_t)p[0];
    break;
  case 3:
    next->dirty = (size_t)p[0];
    break;
  case 4:
    next->layout = (size_t)p[0];
    break;
  }
  return true;
}

static lbm_value tinysg_initialize(lbm_value *args, lbm_uint argn) {
  tinysg_area area;
  tinysg_settings next = defaults;
  if (argn < 2 || !tinysg_dimensions(args, &area)) return ENC_SYM_EERROR;
  for (lbm_uint i = 2; i < argn; i++) {
    if (!tinysg_option(args[i], &next, &area)) return ENC_SYM_EERROR;
  }
  if (area.x + area.width > INT16_MAX || area.y + area.height > INT16_MAX) return ENC_SYM_EERROR;
  if (!tinysg_allocate(next)) {
    lbm_set_error_reason("tinySG: insufficient platform RAM for requested buffers");
    return ENC_SYM_MERROR;
  }
  screen = area;
  return ENC_SYM_TRUE;
}

static lbm_value tinysg_deinitialize(lbm_value *args, lbm_uint argn) {
  (void)args;
  if (argn) return ENC_SYM_EERROR;
#ifdef TINYSG_ESP32P4
  tinysg_esp32p4_enable(false);
#endif
  free(storage);
  free(pixel_buffer);
  storage = pixel_buffer = scratch = dirty = NULL;
  storage_size = scratch_size = 0;
  layout_items = NULL;
  settings = (tinysg_settings){0};
  memset(frames, 0, sizeof(frames));
  screen = (tinysg_area){0};
  frame_valid = false;
  current_frame = 0;
  transfer_bytes = transfer_tiles = 0;
  return ENC_SYM_TRUE;
}

static lbm_value tinysg_clear(lbm_value *args, lbm_uint argn) {
  LBM_CHECK_ARGN_NUMBER(1);
  frame_valid = false;
  return tinysg_display_clear(lbm_dec_as_u32(args[0])) ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static lbm_value tinysg_reset(lbm_value *args, lbm_uint argn) {
  (void)args;
  if (argn) return ENC_SYM_EERROR;
  frame_valid = false;
  return tinysg_display_reset() ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static bool tinysg_flush(void *ctx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pixels) {
  (void)ctx;
  if (!tinysg_display_write(NULL, x + screen.x, y + screen.y, w, h, pixels)) return false;
  transfer_bytes += (uint32_t)w * h * 2;
  transfer_tiles++;
  return true;
}

static bool tinysg_render_frame(tinysg_frame *frame) {
  tinysg_frame *previous = &frames[current_frame ^ 1];
  tinysg_scene scene = {frame->nodes, frame->count, frame->background};
  tinysg_scene old = {previous->nodes, previous->count, previous->background};
  return tinysg_render_update(&scene, frame_valid ? &old : NULL, screen.width, screen.height, dirty, settings.dirty,
                              scratch, scratch_size, tinysg_flush, NULL);
}

static lbm_value tinysg_render_scene(lbm_value *args, lbm_uint argn) {
  if (argn != 2 || !lbm_is_number(args[0]) || !screen.width || !storage) return ENC_SYM_EERROR;
  double background = lbm_dec_as_double(args[0]);
  if (!(background >= 0 && background <= UINT16_MAX)) return ENC_SYM_EERROR;
  tinysg_frame *frame = &frames[current_frame];
  frame->count = frame->text_used = frame->font_count = 0;
  frame->background = (uint16_t)background;
  lbm_value list = args[1], node;
  while (lbm_is_cons(list)) {
    if (!tinysg_take(&list, &node) || !tinysg_parse_node(node, frame)) {
      lbm_set_error_reason("tinySG: invalid scene or configured capacity exceeded");
      return ENC_SYM_EERROR;
    }
  }
  if (!lbm_is_symbol_nil(list)) return ENC_SYM_EERROR;
  transfer_bytes = transfer_tiles = 0;
  frame_valid = tinysg_render_frame(frame);
  if (!frame_valid) {
    lbm_set_error_reason("tinySG: pixel transfer failed; check display initialization, orientation and size");
    return ENC_SYM_EERROR;
  }
  current_frame ^= 1;
  return ENC_SYM_TRUE;
}

static lbm_value tinysg_statistics(lbm_value *args, lbm_uint argn) {
  (void)args;
  if (argn) return ENC_SYM_EERROR;
  return lbm_heap_allocate_list_init(4, lbm_enc_u32(transfer_bytes), lbm_enc_u32(transfer_tiles),
                                     lbm_enc_u32(frames[current_frame ^ 1].count),
                                     lbm_enc_u32(storage_size + (scratch != pixel_buffer ? scratch_size : 0)));
}

static lbm_value tinysg_area_list(tinysg_area area) {
  return lbm_heap_allocate_list_init(4, lbm_enc_i32(area.x), lbm_enc_i32(area.y), lbm_enc_i(area.width),
                                     lbm_enc_i(area.height));
}

static lbm_value tinysg_grid(lbm_value *args, lbm_uint argn) {
  LBM_CHECK_ARGN_NUMBER(9);
  int32_t p[9];
  for (size_t i = 0; i < 9; i++) {
    if (!tinysg_argument(args[i], &p[i], i < 2 ? INT16_MIN : 0, i < 2 ? INT16_MAX : UINT16_MAX)) return ENC_SYM_EERROR;
  }
  tinysg_area cell, area = {p[0], p[1], p[2], p[3]};
  if (!tinysg_grid_cell(area, p[4], p[5], p[6], p[7], (size_t)p[8], &cell)) return ENC_SYM_EERROR;
  return tinysg_area_list(cell);
}

static lbm_value tinysg_flex(lbm_value *args, lbm_uint argn) {
  if (argn != 10 || !storage) return ENC_SYM_EERROR;
  static const char *const flows[] = {"row", "column", "row-wrap", "column-wrap"};
  static const char *const alignments[] = {"start", "center", "end", "stretch", "space-between"};
  int32_t p[6];
  for (size_t i = 0; i < 4; i++) {
    if (!tinysg_argument(args[i], &p[i], i < 2 ? INT16_MIN : 0, i < 2 ? INT16_MAX : UINT16_MAX)) return ENC_SYM_EERROR;
  }
  if (!tinysg_argument(args[5], p + 4, 0, UINT16_MAX) || !tinysg_argument(args[6], p + 5, 0, UINT16_MAX))
    return ENC_SYM_EERROR;
  int flow = tinysg_symbol_index(args[4], flows, 4);
  int justify = tinysg_symbol_index(args[7], alignments, 5), align = tinysg_symbol_index(args[8], alignments, 4);
  if (flow < 0 || justify < 0 || align < 0) return ENC_SYM_EERROR;
  size_t count = 0;
  lbm_value list = args[9], value;
  while (lbm_is_cons(list)) {
    int32_t size[3];
    if (count == settings.layout || !tinysg_take(&list, &value)) return ENC_SYM_EERROR;
    if (!tinysg_numbers(&value, size, 3, 0, UINT16_MAX) || !lbm_is_symbol_nil(value)) return ENC_SYM_EERROR;
    layout_items[count++] = (tinysg_flex_item){.width = size[0], .height = size[1], .grow = size[2]};
  }
  if (!lbm_is_symbol_nil(list)) return ENC_SYM_EERROR;
  tinysg_flex_layout layout = {.area = {p[0], p[1], p[2], p[3]},
                               .gap = p[4],
                               .padding = p[5],
                               .flow = (tinysg_flow)flow,
                               .justify = (tinysg_align)justify,
                               .align = (tinysg_align)align};
  if (!tinysg_layout_flex(&layout, layout_items, count)) return ENC_SYM_EERROR;
  lbm_value result = ENC_SYM_NIL;
  for (size_t i = count; i > 0; i--) {
    lbm_value area = tinysg_area_list(layout_items[i - 1].area);
    if (!lbm_is_cons(area)) return ENC_SYM_MERROR;
    result = lbm_cons(area, result);
    if (!lbm_is_cons(result)) return ENC_SYM_MERROR;
  }
  return result;
}

bool lispif_tinysg_set_buffer(uint8_t *pixels, size_t bytes) {
  if (!storage || (pixels && bytes < 2) || (!pixels && bytes)) return false;
  scratch = pixels ? pixels : pixel_buffer;
  scratch_size = pixels ? bytes : settings.buffer;
  frame_valid = false;
  return true;
}

static lbm_value tinysg_dispatch(lbm_value *args, lbm_uint argn) {
  static const struct {
    const char *name;
    lbm_value (*call)(lbm_value *, lbm_uint);
  } operations[] = {{"render", tinysg_render_scene},
                    {"init", tinysg_initialize},
                    {"deinit", tinysg_deinitialize},
                    {"image", tinysg_image},
                    {"rect", tinysg_rectangle},
                    {"circle", tinysg_circle},
                    {"arc", tinysg_arc},
                    {"pie", tinysg_pie},
                    {"line", tinysg_line},
                    {"triangle", tinysg_triangle},
                    {"ellipse", tinysg_ellipse},
                    {"text", tinysg_text},
                    {"text-size", tinysg_text_size},
                    {"clear", tinysg_clear},
                    {"reset", tinysg_reset},
                    {"stats", tinysg_statistics},
                    {"accel", tinysg_accel},
                    {"accel-stats", tinysg_accel_stats},
                    {"grid-cell", tinysg_grid},
                    {"flex", tinysg_flex}};
  if (!argn || !lbm_is_symbol(args[0])) return ENC_SYM_EERROR;
  const char *name = lbm_get_name_by_symbol(lbm_dec_sym(args[0]));
  if (!name) return ENC_SYM_EERROR;
  for (size_t i = 0; i < sizeof(operations) / sizeof(operations[0]); i++) {
    if (!strcmp(name, operations[i].name)) return operations[i].call(args + 1, argn - 1);
  }
  lbm_set_error_reason("Unknown tinySG operation");
  return ENC_SYM_EERROR;
}

void lispif_tinysg_init(void) {
  tinysg_deinitialize(NULL, 0);
  lbm_add_extension("tinysg", tinysg_dispatch);
}
