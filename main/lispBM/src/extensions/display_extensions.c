/*
  Copyright 2023 - 2025 Benjamin Vedder            benjamin@vedder.se
  Copyright 2023 - 2026 Joel Svensson              svenssonjoel@yahoo.se
  Copyright 2023        Rasmus Söderhielm          rasmus.soderhielm@gmail.com
  Copyright 2025        Joakim Lundborg            joakim.lundborg@gmail.com

  This file is part of LispBM. (Originally a part of the vesc_express FW)

  LispBM is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  LispBM is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>
#include <string.h>

#include <extensions/display_extensions.h>
#include <lbm_utils.h>
#include <lbm_defrag_mem.h>

#ifdef LBM_OPT_DISPLAY_EXTENSIONS_SIZE
#pragma GCC optimize ("-Os")
#endif
#ifdef LBM_OPT_DISPLAY_EXTENSIONS_SIZE_AGGRESSIVE
#pragma GCC optimize ("-Oz")
#endif

#define MAX_WIDTH 32000
#define MAX_HEIGHT 32000

#define SCENE_MAGIC 0x53434E31u
#define SCENE_MAX_SLOTS 256

typedef enum {
  SCENE_RECT,
  SCENE_CIRCLE,
  SCENE_IMAGE,
  SCENE_TEXT,
  SCENE_TTF,
  SCENE_ROUNDRECT,
  SCENE_KIND_COUNT
} display_scene_kind_t;

typedef struct {
  uint32_t magic;
  tinygfx_scene_t scheduler;
  uint32_t background;
  uint16_t slots;
  uint16_t tile_width;
  uint16_t tile_height;
} display_scene_t;

typedef struct {
  display_scene_kind_t kind;
  tinygfx_rect_t bounds;
  lbm_value color;
  lbm_value palette;
  image_buffer_t image;
  int32_t transparent;
  uint8_t alpha;
  int radius;
  const uint8_t *font;
  const char *text;
  int text_scale;
  tinygfx_prepared_font_t prepared;
  int32_t pen_x;
  int32_t pen_y;
} display_scene_node_t;

typedef struct {
  display_scene_t *state;
  lbm_value slots;
} display_scene_render_context_t;

typedef struct {
  image_buffer_t *tile;
  display_scene_node_t *node;
  int tx;
  int ty;
  int x0;
  int y0;
  int x1;
  int y1;
  lbm_value palette[16];
} display_scene_glyph_context_t;


static struct {
  const char *name;
  lbm_uint symbol;
} scene_symbols[SCENE_KIND_COUNT] = {
  {"rect", 0},
  {"circle", 0},
  {"image", 0},
  {"text", 0},
  {"ttf", 0},
  {"roundrect", 0},
};

static lbm_uint symbol_indexed2 = 0;
static lbm_uint symbol_indexed4 = 0;
static lbm_uint symbol_indexed16 = 0;
static lbm_uint symbol_rgb332 = 0;
static lbm_uint symbol_rgb565 = 0;
static lbm_uint symbol_rgb888 = 0;

static lbm_uint symbol_thickness = 0;
static lbm_uint symbol_filled = 0;
static lbm_uint symbol_rounded = 0;
static lbm_uint symbol_dotted = 0;
static lbm_uint symbol_scale = 0;
static lbm_uint symbol_rotate = 0;
static lbm_uint symbol_resolution = 0;
static lbm_uint symbol_tile = 0;
static lbm_uint symbol_clip = 0;
static lbm_uint symbol_palette = 0;
static lbm_uint symbol_alpha_buffer = 0;


static lbm_uint symbol_regular = 0;
static lbm_uint symbol_gradient_x = 0;
static lbm_uint symbol_gradient_y = 0;
static lbm_uint symbol_gradient_x_pre = 0;
static lbm_uint symbol_gradient_y_pre = 0;
static lbm_uint symbol_repeat = 0;
static lbm_uint symbol_mirrored = 0;

static lbm_uint symbol_color_0 = 0;
static lbm_uint symbol_color_1 = 0;
static lbm_uint symbol_width = 0;
static lbm_uint symbol_offset = 0;
static lbm_uint symbol_repeat_type = 0;
static lbm_uint symbol_alpha = 0;

static lbm_uint symbol_down = 0;
static lbm_uint symbol_up = 0;
static lbm_uint symbol_magnify = 0;
static lbm_uint symbol_spacing = 0;
static lbm_uint symbol_align = 0;
static lbm_uint symbol_left = 0;
static lbm_uint symbol_center = 0;
static lbm_uint symbol_right = 0;

bool display_is_symbol_up(lbm_value v) {
  if (lbm_is_symbol(v)) {
    lbm_uint s = lbm_dec_sym(v);
    return (s == symbol_up);
  }
  return false;
}

bool display_is_symbol_down(lbm_value v) {
  if (lbm_is_symbol(v)) {
    lbm_uint s = lbm_dec_sym(v);
    return (s == symbol_down);
  }
  return false;
}

color_format_t sym_to_color_format(lbm_value v) {
  lbm_uint s = lbm_dec_sym(v);
  if (s == symbol_indexed2) return indexed2;
  if (s == symbol_indexed4) return indexed4;
  if (s == symbol_indexed16) return indexed16;
  if (s == symbol_rgb332) return rgb332;
  if (s == symbol_rgb565) return rgb565;
  if (s == symbol_rgb888) return rgb888;
  return format_not_supported;
}

#define COLOR_MAGIC (uint32_t)0x4C4F4300

#define COLOR_PRECALC_LEN	512

static lbm_value image_buffer_lift(uint8_t *buf, color_format_t fmt, uint16_t width, uint16_t height) {
  lbm_value res = ENC_SYM_MERROR;
  lbm_uint size = image_dims_to_size_bytes(fmt, width, height);
  if ( lbm_lift_array(&res, (char*)buf, IMAGE_BUFFER_HEADER_SIZE + size)) {
    buf[0] = (uint8_t)(width >> 8);
    buf[1] = (uint8_t)width;
    buf[2] = (uint8_t)(height >> 8);
    buf[3] = (uint8_t)height;
    buf[4] = color_format_to_byte(fmt);
  }
  return res;
}

static inline bool is_color_sized(lbm_uint size) {
  size_t color_size = sizeof(color_t);
  return (size == color_size);
}

static inline bool is_color(uint8_t *data, lbm_uint size) {
  bool res = false;
  if (is_color_sized(size)) {
    color_t *color = (color_t*)data;
    res = (color->magic == COLOR_MAGIC);
  }
  return res;
}


static lbm_value color_allocate(COLOR_TYPE type, int32_t color1, int32_t color2, uint16_t param1, uint16_t param2, bool mirrored, uint8_t alpha) {
  color_t *color = lbm_malloc(sizeof(color_t));
  if (!color) {
    return ENC_SYM_MERROR;
  }

  uint32_t *pre = 0;
  if (type == COLOR_PRE_X || type == COLOR_PRE_Y) {
    pre = lbm_malloc(COLOR_PRECALC_LEN * sizeof(uint32_t));
    if (!pre) {
      lbm_free(color);
      return ENC_SYM_MERROR;
    }
  }

  lbm_value res = ENC_SYM_MERROR;

  if (lbm_lift_array(&res, (char*)color, sizeof(color_t))) {
    color->magic = COLOR_MAGIC;
    color->type = type;
    color->color1 = color1;
    color->color2 = color2;
    color->param1 = param1;
    color->param2 = param2;
    color->mirrored = mirrored;
    color->alpha = alpha;
    color->precalc = pre;

    if (pre) {
      COLOR_TYPE type_old = color->type;
      if (type == COLOR_PRE_X) {
        color->type = COLOR_GRADIENT_X;
      } else if (type == COLOR_PRE_Y) {
        color->type = COLOR_GRADIENT_Y;
      }

      if (color->param1 > COLOR_PRECALC_LEN) {
        color->param1 = COLOR_PRECALC_LEN;
      }

      for (int i = 0;i < color->param1;i++) {
        pre[i] = lbm_display_rgb888_from_color(*color, i + color->param2, i + color->param2);
      }

      color->type = type_old;
    }
  } else {
    lbm_free(pre);
    lbm_free(color);
  }

  return res;
}

static lbm_value image_buffer_allocate(color_format_t fmt, uint16_t width, uint16_t height) {
  uint32_t size_bytes = image_dims_to_size_bytes(fmt, width, height);

  uint8_t *buf = lbm_malloc(IMAGE_BUFFER_HEADER_SIZE + size_bytes);
  if (!buf) {
    return ENC_SYM_MERROR;
  }
  memset(buf, 0, size_bytes + IMAGE_BUFFER_HEADER_SIZE);
  lbm_value res = image_buffer_lift(buf, fmt, width, height);
  if (lbm_is_symbol(res)) { /* something is wrong, free */
    lbm_free(buf);
  }
  return res;
}

static lbm_value image_buffer_allocate_dm(lbm_uint *dm, color_format_t fmt, uint16_t width, uint16_t height) {
  uint32_t size_bytes = image_dims_to_size_bytes(fmt, width, height);

  lbm_value res = lbm_defrag_mem_alloc(dm, IMAGE_BUFFER_HEADER_SIZE + size_bytes);
  lbm_array_header_t *arr = lbm_dec_array_r(res);
  if (arr) {
    uint8_t *buf = (uint8_t*)arr->data;
    buf[0] = (uint8_t)(width >> 8);
    buf[1] = (uint8_t)width;
    buf[2] = (uint8_t)(height >> 8);
    buf[3] = (uint8_t)height;
    buf[4] = color_format_to_byte(fmt);
  }
  return res;
}

// Exported interface
bool display_is_color(lbm_value v) {
  lbm_array_header_t *array = lbm_dec_array_r(v);
  bool res = false;
  if (array && is_color_sized(array->size)) {
    res = (is_color((uint8_t*)array->data, array->size));
  }
  return res;
}

static color_t *get_color(lbm_value v) {
  color_t *res = NULL;
  lbm_array_header_t *array = lbm_dec_array_r(v);
  if (array && is_color_sized(array->size)
      && (is_color((uint8_t*)array->data, array->size))) {
    res = (color_t*)array->data;
  }
  return res;
}


// Register symbols

static bool register_symbols(void) {
  bool res = true;
  for (unsigned i = 0; i < SCENE_KIND_COUNT; i++) {
    res = res && lbm_add_symbol_const(scene_symbols[i].name, &scene_symbols[i].symbol);
  }
  res = res && lbm_add_symbol_const("indexed2", &symbol_indexed2);
  res = res && lbm_add_symbol_const("indexed4", &symbol_indexed4);
  res = res && lbm_add_symbol_const("indexed16", &symbol_indexed16);
  res = res && lbm_add_symbol_const("rgb332", &symbol_rgb332);
  res = res && lbm_add_symbol_const("rgb565", &symbol_rgb565);
  res = res && lbm_add_symbol_const("rgb888", &symbol_rgb888);

  res = res && lbm_add_symbol_const("thickness", &symbol_thickness);
  res = res && lbm_add_symbol_const("filled", &symbol_filled);
  res = res && lbm_add_symbol_const("rounded", &symbol_rounded);
  res = res && lbm_add_symbol_const("dotted", &symbol_dotted);
  res = res && lbm_add_symbol_const("scale", &symbol_scale);
  res = res && lbm_add_symbol_const("rotate", &symbol_rotate);
  res = res && lbm_add_symbol_const("resolution", &symbol_resolution);
  res = res && lbm_add_symbol_const("tile", &symbol_tile);
  res = res && lbm_add_symbol_const("clip", &symbol_clip);
  res = res && lbm_add_symbol_const("palette", &symbol_palette);
  res = res && lbm_add_symbol_const("alpha-buffer", &symbol_alpha_buffer);

  res = res && lbm_add_symbol_const("regular", &symbol_regular);
  res = res && lbm_add_symbol_const("gradient_x", &symbol_gradient_x);
  res = res && lbm_add_symbol_const("gradient_y", &symbol_gradient_y);
  res = res && lbm_add_symbol_const("gradient_x_pre", &symbol_gradient_x_pre);
  res = res && lbm_add_symbol_const("gradient_y_pre", &symbol_gradient_y_pre);
  res = res && lbm_add_symbol_const("mirrored", &symbol_mirrored);
  res = res && lbm_add_symbol_const("repeat", &symbol_repeat);

  res = res && lbm_add_symbol_const("color-0", &symbol_color_0);
  res = res && lbm_add_symbol_const("color-1", &symbol_color_1);
  res = res && lbm_add_symbol_const("width", &symbol_width);
  res = res && lbm_add_symbol_const("offset", &symbol_offset);
  res = res && lbm_add_symbol_const("repeat-type", &symbol_repeat_type);
  res = res && lbm_add_symbol_const("alpha", &symbol_alpha);

  res = res && lbm_add_symbol_const("down", &symbol_down);
  res = res && lbm_add_symbol_const("up", &symbol_up);
  res = res && lbm_add_symbol_const("magnify", &symbol_magnify);
  res = res && lbm_add_symbol_const("spacing", &symbol_spacing);
  res = res && lbm_add_symbol_const("align", &symbol_align);
  res = res && lbm_add_symbol_const("left", &symbol_left);
  res = res && lbm_add_symbol_const("center", &symbol_center);
  res = res && lbm_add_symbol_const("right", &symbol_right);

  return res;
}

#include "tinygfx_fonts.h"

// returns: 1 parsed, 0 not an attr list, -1 malformed/invalid
static int parse_text_attr(lbm_value v, float *mag, int *spacing, int *align, int *rotation_deg) {
  if (lbm_is_cons(v) && lbm_car(v) == ENC_SYM_QUOTE) {
    lbm_value quoted = lbm_cdr(v);
    if (!lbm_is_cons(quoted) || lbm_cdr(quoted) != ENC_SYM_NIL) {
      return -1;
    }
    v = lbm_car(quoted);
  }

  if (!lbm_is_cons(v)) {
    return 0;
  }

  lbm_value key = lbm_car(v);
  lbm_value rest = lbm_cdr(v);
  if (!lbm_is_symbol(key) || !lbm_is_cons(rest)) {
    return 0;
  }

  lbm_value val = lbm_car(rest);
  if (lbm_cdr(rest) != ENC_SYM_NIL) {
    return -1;
  }

  lbm_uint sym = lbm_dec_sym(key);
  if (sym == symbol_magnify || sym == symbol_scale) {
    if (!lbm_is_number(val)) return -1;
    float m = lbm_dec_as_float(val);
    if (m < 0.1f) m = 0.1f;
    *mag = m;
    return 1;
  }

  if (sym == symbol_spacing) {
    if (!lbm_is_number(val)) return -1;
    *spacing = lbm_dec_as_i32(val);
    return 1;
  }

  if (sym == symbol_align) {
    if (lbm_is_number(val)) {
      int a = lbm_dec_as_i32(val);
      if (a < 0 || a > 2) return -1;
      *align = a;
      return 1;
    }

    if (lbm_is_symbol(val)) {
      lbm_uint av = lbm_dec_sym(val);
      if (av == symbol_left) { *align = 0; return 1; }
      if (av == symbol_center) { *align = 1; return 1; }
      if (av == symbol_right) { *align = 2; return 1; }
      return -1;
    }

    char *astr = lbm_dec_str(val);
    if (!astr) return -1;
    if (strcmp(astr, "left") == 0) { *align = 0; return 1; }
    if (strcmp(astr, "center") == 0) { *align = 1; return 1; }
    if (strcmp(astr, "right") == 0) { *align = 2; return 1; }
    return -1;
  }

  if (sym == symbol_rotate) {
    if (!lbm_is_number(val)) return -1;
    int r = lbm_dec_as_i32(val);
    *rotation_deg = r % 360;
    return 1;
  }

  return 0;
}

// Extensions

#define ATTR_MAX_ARGS	4
#define ARG_MAX_NUM	8

typedef struct {
  bool is_valid;
  uint16_t arg_num;
  lbm_value args[ATTR_MAX_ARGS];
} attr_t;

typedef struct {
  bool is_valid;
  image_buffer_t img;
  lbm_value args[ARG_MAX_NUM];
  attr_t attr_thickness;
  attr_t attr_filled;
  attr_t attr_rounded;
  attr_t attr_dotted;
  attr_t attr_scale;
  attr_t attr_rotate;
  attr_t attr_resolution;
  attr_t attr_tile;
  attr_t attr_clip;
  attr_t attr_palette;
  attr_t attr_alpha;
  attr_t attr_alpha_buffer;
  bool transform_attr_present;
} img_args_t;

static img_args_t decode_args(lbm_value *args, lbm_uint argn, int num_expected) {
  img_args_t res;
  memset(&res, 0, sizeof(res));
  res.is_valid = false;

  lbm_array_header_t *arr;
  if (argn >= 1 && (arr = get_image_buffer(args[0]))) {
    // at least one argument which is an image buffer.
    res.img.width = image_buffer_width((uint8_t*)arr->data);
    res.img.height = image_buffer_height((uint8_t*)arr->data);
    res.img.fmt = image_buffer_format((uint8_t*)arr->data);
    res.img.mem_base = (uint8_t*)arr->data;
    res.img.data = image_buffer_data((uint8_t*)arr->data);


    int num_dec = 0;
    for (unsigned int i = 1;i < argn;i++) {
      if (!lbm_is_number(args[i]) && !lbm_is_cons(args[i])) {
        color_t *clr = get_color(args[i]);
        if (clr && clr->type == COLOR_REGULAR &&
            num_dec == num_expected - 1) {
          res.args[num_dec] = lbm_enc_u32((uint32_t)clr->color1);
          num_dec++;
          if (num_dec >= ARG_MAX_NUM) return res;
          continue;
        }
        return res;
      }

      if (lbm_is_number(args[i])) {
        res.args[num_dec] = args[i];
        num_dec++;

        if (num_dec >= ARG_MAX_NUM) {
          return res;
        }
      } else {
        lbm_value curr = args[i];
        int attr_ind = 0;
        attr_t *attr_now = 0;
        while (lbm_is_cons(curr)) {
          lbm_value  arg = lbm_car(curr);

          if (attr_ind == 0) {
            if (!lbm_is_symbol(arg)) {
              return res;
            }

            if (lbm_dec_sym(arg) == symbol_thickness) {
              attr_now = &res.attr_thickness;
              attr_now->arg_num = 1;
            } else if (lbm_dec_sym(arg) == symbol_filled) {
              attr_now = &res.attr_filled;
              attr_now->arg_num = 0;
            } else if (lbm_dec_sym(arg) == symbol_rounded) {
              attr_now = &res.attr_rounded;
              attr_now->arg_num = 1;
            } else if (lbm_dec_sym(arg) == symbol_dotted) {
              attr_now = &res.attr_dotted;
              attr_now->arg_num = 2;
            } else if (lbm_dec_sym(arg) == symbol_scale) {
              attr_now = &res.attr_scale;
              attr_now->arg_num = 1;
              res.transform_attr_present = true;
            } else if (lbm_dec_sym(arg) == symbol_rotate) {
              attr_now = &res.attr_rotate;
              attr_now->arg_num = 3;
              res.transform_attr_present = true;
            } else if (lbm_dec_sym(arg) == symbol_resolution) {
              attr_now = &res.attr_resolution;
              attr_now->arg_num = 1;
            } else if (lbm_dec_sym(arg) == symbol_tile) {
              attr_now = &res.attr_tile;
              attr_now->arg_num = 0;
              res.transform_attr_present = true;
            } else if (lbm_dec_sym(arg) == symbol_clip) {
              attr_now = &res.attr_clip;
              attr_now->arg_num = 4;
              res.transform_attr_present = true;
            } else if (lbm_dec_sym(arg) == symbol_palette) {
              attr_now = &res.attr_palette;
              attr_now->arg_num = 1;
            } else if (lbm_dec_sym(arg) == symbol_alpha) {
              attr_now = &res.attr_alpha;
              attr_now->arg_num = 1;
            } else if (lbm_dec_sym(arg) == symbol_alpha_buffer) {
              attr_now = &res.attr_alpha_buffer;
              attr_now->arg_num = 1;
            }
            else {
              return res;
            }
          } else {
            if (attr_now == &res.attr_palette) {
              if (!lbm_is_list(arg)) {
                return res;
              }
            } else if (attr_now == &res.attr_alpha_buffer) {
              if (!lbm_dec_array_r(arg)) {
                return res;
              }
            } else if (!lbm_is_number(arg)) {
              return res;
            }

            attr_now->args[attr_ind - 1] = arg;
          }

          attr_ind++;
          if (attr_ind > (ATTR_MAX_ARGS + 1)) {
            return res;
          }

          curr = lbm_cdr(curr);
        }

        // does this really compare the pointer addresses?
        if (attr_now == &res.attr_rounded && attr_ind == 1) {
          attr_now->arg_num = 0; // the `rounded` attribute may be empty
        }


        if ((attr_ind - 1) == attr_now->arg_num) {
          attr_now->is_valid = true;
        } else {
          return res;
        }
      }
    }
    if (num_dec != num_expected) {
      return res;
    }
    // I think this should go here ???
    res.is_valid = true;
  }
  return res;
}

static lbm_value ext_image_dims(lbm_value *args, lbm_uint argn) {
  img_args_t arg_dec = decode_args(args, argn, 0);

  if (!arg_dec.is_valid) {
    return ENC_SYM_TERROR;
  }

  lbm_value dims = lbm_heap_allocate_list(2);
  if (lbm_is_symbol(dims)) {
    return dims;
  }
  lbm_value curr = dims;
  lbm_set_car(curr, lbm_enc_i(arg_dec.img.width));
  curr = lbm_cdr(curr);
  lbm_set_car(curr, lbm_enc_i(arg_dec.img.height));
  return dims;
}

static lbm_value ext_image_buffer(lbm_value *args, lbm_uint argn) {
  lbm_value res = ENC_SYM_TERROR;
  bool args_ok = false;
  color_format_t fmt = indexed2;
  lbm_uint w = 0;
  lbm_uint h = 0;

  if (argn == 4 &&
      lbm_is_defrag_mem(args[0]) &&
      lbm_is_symbol(args[1]) &&
      lbm_is_number(args[2]) &&
      lbm_is_number(args[3])) {
    fmt = sym_to_color_format(args[1]);
    w = lbm_dec_as_u32(args[2]);
    h = lbm_dec_as_u32(args[3]);
    args_ok = true;
  } else if (argn == 3 &&
             lbm_is_symbol(args[0]) &&
             lbm_is_number(args[1]) &&
             lbm_is_number(args[2])) {
    fmt = sym_to_color_format(args[0]);
    w = lbm_dec_as_u32(args[1]);
    h = lbm_dec_as_u32(args[2]);
    args_ok = true;
  }

  if (args_ok && fmt != format_not_supported && w > 0 && h > 0 && w < MAX_WIDTH && h < MAX_HEIGHT) {
    if (argn == 3) {
      res = image_buffer_allocate(fmt, (uint16_t)w, (uint16_t)h);
    } else {
      res = image_buffer_allocate_dm((lbm_uint*)lbm_car(args[0]), fmt, (uint16_t)w, (uint16_t)h);
    }
  }
  return res;
}


static lbm_value ext_is_image_buffer(lbm_value *args, lbm_uint argn) {
  lbm_value res = ENC_SYM_TERROR;

  if (argn == 1) {
    res = ENC_SYM_NIL;
    lbm_array_header_t *array = lbm_dec_array_r(args[0]);
    if (array) {
      uint8_t *data = (uint8_t*)array->data;
      if (image_buffer_is_valid(data, array->size)) {
        res = ENC_SYM_TRUE;;
      }
    }
  }
  return res;
}

static lbm_value ext_color(lbm_value *args, lbm_uint argn) {
  lbm_value res = ENC_SYM_TERROR;

  if (argn >= 2 && argn <= 7 &&
      lbm_is_symbol(args[0]) &&
      lbm_is_number(args[1])) {

    // Color1 and color2 are int in the struct and decoded as i32, why
    // where they stored in uint32_t?
    int32_t color1 = lbm_dec_as_i32(args[1]);

    int32_t color2 = 0;
    if (argn >= 3) {
      if (lbm_is_number(args[2])) {
        color2 = lbm_dec_as_i32(args[2]);
      } else {
        return ENC_SYM_TERROR;
      }
    }

    int32_t param1 = 0;
    if (argn >= 4) {
      if (lbm_is_number(args[3])) {
        param1 = lbm_dec_as_i32(args[3]);
      } else {
        return ENC_SYM_TERROR;
      }
    }

    int32_t param2 = 0;
    if (argn >= 5) {
      if (lbm_is_number(args[4])) {
        param2 = lbm_dec_as_i32(args[4]);
      } else {
        return ENC_SYM_TERROR;
      }
    }

    bool mirrored = false;
    lbm_uint alpha_arg = argn;

    if (argn >= 6) {
      if (lbm_is_symbol(args[5])) {
        lbm_uint sym = lbm_dec_sym(args[5]);
        if (sym == symbol_repeat) {
          mirrored = false;
        } else if (sym == symbol_mirrored) {
          mirrored = true;
        } else {
          return ENC_SYM_TERROR;
        }
        alpha_arg = 6;
      } else if (argn == 6 && lbm_is_number(args[5])) {
        alpha_arg = 5;
      } else {
        return ENC_SYM_TERROR;
      }
    }

    COLOR_TYPE t;
    if (lbm_dec_sym(args[0]) == symbol_regular) {
      t = COLOR_REGULAR;
    } else if (lbm_dec_sym(args[0]) == symbol_gradient_x) {
      t = COLOR_GRADIENT_X;
    } else if (lbm_dec_sym(args[0]) == symbol_gradient_y) {
      t = COLOR_GRADIENT_Y;
    } else if (lbm_dec_sym(args[0]) == symbol_gradient_x_pre) {
      t = COLOR_PRE_X;
    } else if (lbm_dec_sym(args[0]) == symbol_gradient_y_pre) {
      t = COLOR_PRE_Y;
    } else {
      return ENC_SYM_TERROR;
    }

    if (t == COLOR_REGULAR && argn == 3) {
      alpha_arg = 2;
    }

    uint8_t alpha = 255;
    if (alpha_arg < argn) {
      if (lbm_is_number(args[alpha_arg])) {
        int32_t a = lbm_dec_as_i32(args[alpha_arg]);
        if (a < 0) a = 0;
        if (a > 255) a = 255;
        alpha = (uint8_t)a;
      } else {
        return ENC_SYM_TERROR;
      }
    }

    // Maybe check if param is in ranges first ?
    res = color_allocate(t, color1, color2, (uint16_t)param1, (uint16_t)param2, mirrored, alpha);
  }

  return res;
}

static lbm_value ext_color_set(lbm_value *args, lbm_uint argn) {
  color_t *color;
  if (argn != 3 || !(color = get_color(args[0])) || // color assignment
      !lbm_is_symbol(args[1])) {
    return ENC_SYM_TERROR;
  }

  bool is_regular = color->type == COLOR_REGULAR;
  bool is_gradient = color->type == COLOR_GRADIENT_X || color->type == COLOR_GRADIENT_Y;
  bool is_pre = color->type == COLOR_PRE_X || color->type == COLOR_PRE_Y;

  lbm_uint prop = lbm_dec_sym(args[1]);
  if (prop == symbol_color_0) {
    if (!lbm_is_number(args[2]) || !(is_regular || is_gradient)) {
      return ENC_SYM_TERROR;
    }
    color->color1 = lbm_dec_as_i32(args[2]);
  } else if (prop == symbol_color_1) {
    if (!lbm_is_number(args[2]) || !is_gradient) {
      return ENC_SYM_TERROR;
    }
    color->color2 = lbm_dec_as_i32(args[2]);
  } else if (prop == symbol_width) {
    if (!lbm_is_number(args[2]) || !is_gradient) {
      return ENC_SYM_TERROR;
    }
    color->param1 = (uint16_t)lbm_dec_as_u32(args[2]);
  } else if (prop == symbol_offset) {
    if (!lbm_is_number(args[2]) || !(is_gradient || is_pre)) {
      return ENC_SYM_TERROR;
    }
    color->param2 = (uint16_t)lbm_dec_as_u32(args[2]);
  } else if (prop == symbol_repeat_type) {
    if (!lbm_is_symbol(args[2]) || !(is_gradient || is_pre)) {
      return ENC_SYM_TERROR;
    }
    lbm_uint sym = lbm_dec_sym(args[2]);
    if (sym == symbol_repeat) {
      color->mirrored = false;
    } else if (sym == symbol_mirrored) {
      color->mirrored = true;
    } else {
      return ENC_SYM_TERROR;
    }
  } else if (prop == symbol_alpha) {
    if (!lbm_is_number(args[2])) {
      return ENC_SYM_TERROR;
    }
    int32_t a = lbm_dec_as_i32(args[2]);
    if (a < 0) a = 0;
    if (a > 255) a = 255;
    color->alpha = (uint8_t)a;
  } else {
    return ENC_SYM_TERROR;
  }

  return ENC_SYM_TRUE;
}

static lbm_value ext_color_get(lbm_value *args, lbm_uint argn) {
  color_t *color;
  if (argn != 2 || !(color = get_color(args[0])) || // color assignment
      !lbm_is_symbol(args[1])) {
    return ENC_SYM_TERROR;
  }

  bool is_gradient = color->type == COLOR_GRADIENT_X || color->type == COLOR_GRADIENT_Y;
  bool is_pre = color->type == COLOR_PRE_X || color->type == COLOR_PRE_Y;

  lbm_uint prop = lbm_dec_sym(args[1]);
  if (prop == symbol_color_0) {
    // always allowed
    return lbm_enc_u32((uint32_t)color->color1);
  } else if (prop == symbol_color_1) {
    if (!is_gradient && !is_pre) {
      return ENC_SYM_TERROR;
    }
    return lbm_enc_u32((uint32_t)color->color2);
  } else if (prop == symbol_width) {
    if (!is_gradient && !is_pre) {
      return ENC_SYM_TERROR;
    }
    return lbm_enc_i32((int32_t)color->param1);
  } else if (prop == symbol_offset) {
    if (!is_gradient && !is_pre) {
      return ENC_SYM_TERROR;
    }
    return lbm_enc_i32((int32_t)color->param2);
  } else if (prop == symbol_repeat_type) {
    if (!is_gradient && !is_pre) {
      return ENC_SYM_TERROR;
    }
    return lbm_enc_sym(color->mirrored ? symbol_mirrored : symbol_repeat);
  } else if (prop == symbol_alpha) {
    return lbm_enc_u32((uint32_t)color->alpha);
  } else {
    return ENC_SYM_TERROR;
  }

  return ENC_SYM_TRUE;
}

static lbm_value ext_color_setpre(lbm_value *args, lbm_uint argn) {
  color_t *color;
  if (argn != 3 || !(color = get_color(args[0])) ||
      !lbm_is_number(args[1]) || !lbm_is_number(args[2])) {
    return ENC_SYM_TERROR;
  }

  uint32_t pos = lbm_dec_as_u32(args[1]);
  int new_color = lbm_dec_as_i32(args[2]);

  if (color->precalc == 0 || pos >= COLOR_PRECALC_LEN) {
    return ENC_SYM_EERROR;
  }

  color->precalc[pos] = (uint32_t)new_color;

  return ENC_SYM_TRUE;
}

static lbm_value ext_color_getpre(lbm_value *args, lbm_uint argn) {
  color_t *color;
  if (argn != 2 || !(color = get_color(args[0])) ||
      !lbm_is_number(args[1])) {
    return ENC_SYM_TERROR;
  }
  uint32_t pos = lbm_dec_as_u32(args[1]);

  if (color->precalc == 0 || pos >= COLOR_PRECALC_LEN) {
    return ENC_SYM_EERROR;
  }

  return lbm_enc_u32(color->precalc[pos]);
}

static lbm_value ext_clear(lbm_value *args, lbm_uint argn) {

  lbm_value res = ENC_SYM_TERROR;
  lbm_array_header_t *arr;
  if ((argn == 1 || argn == 2) &&
      (arr = get_image_buffer(args[0])) &&   // assignment
      (argn != 2 || lbm_is_number(args[1]))) { // ( argn == 2 -> lbm_is_number(args[1]))
    image_buffer_t img_buf;
    img_buf.width = image_buffer_width((uint8_t*)arr->data);
    img_buf.height = image_buffer_height((uint8_t*)arr->data);
    img_buf.fmt = image_buffer_format((uint8_t*)arr->data);
    img_buf.mem_base = (uint8_t*)arr->data;
    img_buf.data = image_buffer_data((uint8_t*)arr->data);

    uint32_t color = 0;
    if (argn == 2) {
      color = lbm_dec_as_u32(args[1]);
    }

    image_buffer_clear(&img_buf, color);
    res = ENC_SYM_TRUE;
  }
  return res;
}

static lbm_value ext_putpixel(lbm_value *args, lbm_uint argn) {
  img_args_t arg_dec = decode_args(args, argn, 3);

  if (!arg_dec.is_valid) {
    return ENC_SYM_TERROR;
  }

  putpixel(&arg_dec.img,
           lbm_dec_as_i32(arg_dec.args[0]),
           lbm_dec_as_i32(arg_dec.args[1]),
           lbm_dec_as_u32(arg_dec.args[2]));
  return ENC_SYM_TRUE;
}

static lbm_value ext_getpixel(lbm_value *args, lbm_uint argn) {
  img_args_t arg_dec = decode_args(args, argn, 2);

  if (!arg_dec.is_valid) {
    return ENC_SYM_TERROR;
  }

  uint32_t c = getpixel(&arg_dec.img,
                        lbm_dec_as_i32(arg_dec.args[0]),
                        lbm_dec_as_i32(arg_dec.args[1]));
  return lbm_enc_u32(c);
}

// lisp args: img x1 y1 x2 y2 color opt-attr1 ... opt-attrN
static lbm_value ext_line(lbm_value *args, lbm_uint argn) {
  img_args_t arg_dec = decode_args(args, argn, 5);

  if (!arg_dec.is_valid) {
    return ENC_SYM_TERROR;
  }

  tinygfx_line(&arg_dec.img,
       lbm_dec_as_i32(arg_dec.args[0]),
       lbm_dec_as_i32(arg_dec.args[1]),
       lbm_dec_as_i32(arg_dec.args[2]),
       lbm_dec_as_i32(arg_dec.args[3]),
       lbm_dec_as_i32(arg_dec.attr_thickness.args[0]),
       lbm_dec_as_i32(arg_dec.attr_dotted.args[0]),
       lbm_dec_as_i32(arg_dec.attr_dotted.args[1]),
       lbm_dec_as_u32(arg_dec.args[4]));

  return ENC_SYM_TRUE;
}

// Shared by ext_arc/ext_circle_sector/ext_circle_segment, which decode
// identical attribute sets and only differ in sector/segment and which
// args[] slot holds the color.
static arc_params_t make_arc_params(const img_args_t *a, bool rounded, bool sector, bool segment, lbm_value color_arg) {
  arc_params_t p = {
    .thickness  = lbm_dec_as_i32(a->attr_thickness.args[0]),
    .rounded    = rounded,
    .filled     = a->attr_filled.is_valid,
    .sector     = sector,
    .segment    = segment,
    .dot1       = lbm_dec_as_i32(a->attr_dotted.args[0]),
    .dot2       = lbm_dec_as_i32(a->attr_dotted.args[1]),
    .resolution = lbm_dec_as_i32(a->attr_resolution.args[0]),
    .color      = lbm_dec_as_u32(color_arg),
  };
  return p;
}

// lisp args: img cx cy r color opt-attr1 ... opt-attrN
static lbm_value ext_circle(lbm_value *args, lbm_uint argn) {
  img_args_t arg_dec = decode_args(args, argn, 4);

  if (!arg_dec.is_valid) {
    return ENC_SYM_TERROR;
  }

  if (arg_dec.attr_filled.is_valid) {
    tinygfx_fill_circle(&arg_dec.img,
                lbm_dec_as_i32(arg_dec.args[0]),
                lbm_dec_as_i32(arg_dec.args[1]),
                lbm_dec_as_i32(arg_dec.args[2]),
                lbm_dec_as_u32(arg_dec.args[3]));
  } else if (arg_dec.attr_dotted.is_valid) {
    // rounded here currently does nothing as the line function doesn't
    // support square ends.
    arc_params_t p = make_arc_params(&arg_dec, arg_dec.attr_rounded.is_valid, false, false, arg_dec.args[3]);
    tinygfx_arc(&arg_dec.img,
        lbm_dec_as_i32(arg_dec.args[0]),
        lbm_dec_as_i32(arg_dec.args[1]),
        lbm_dec_as_i32(arg_dec.args[2]),
        0, 359.9f,
        &p);
  } else {
    tinygfx_circle(&arg_dec.img,
           lbm_dec_as_i32(arg_dec.args[0]),
           lbm_dec_as_i32(arg_dec.args[1]),
           lbm_dec_as_i32(arg_dec.args[2]),
           lbm_dec_as_i32(arg_dec.attr_thickness.args[0]),
           lbm_dec_as_u32(arg_dec.args[3]));
  }

  return ENC_SYM_TRUE;
}

// lisp args: img cx cy r ang-s ang-e color opt-attr1 ... opt-attrN
static lbm_value ext_arc(lbm_value *args, lbm_uint argn) {
  img_args_t arg_dec = decode_args(args, argn, 6);

  if (!arg_dec.is_valid) {
    return ENC_SYM_TERROR;
  }

  arc_params_t p = make_arc_params(&arg_dec, arg_dec.attr_rounded.is_valid, false, false, arg_dec.args[5]);
  tinygfx_arc(&arg_dec.img,
      lbm_dec_as_i32(arg_dec.args[0]),
      lbm_dec_as_i32(arg_dec.args[1]),
      lbm_dec_as_i32(arg_dec.args[2]),
      lbm_dec_as_float(arg_dec.args[3]),
      lbm_dec_as_float(arg_dec.args[4]),
      &p);

  return ENC_SYM_TRUE;
}

// lisp args: img cx cy r ang-s ang-e color opt-attr1 ... opt-attrN
static lbm_value ext_circle_sector(lbm_value *args, lbm_uint argn) {
  img_args_t arg_dec = decode_args(args, argn, 6);

  if (!arg_dec.is_valid) {
    return ENC_SYM_TERROR;
  }

  arc_params_t p = make_arc_params(&arg_dec, true, true, false, arg_dec.args[5]);
  tinygfx_arc(&arg_dec.img,
      lbm_dec_as_i32(arg_dec.args[0]),
      lbm_dec_as_i32(arg_dec.args[1]),
      lbm_dec_as_i32(arg_dec.args[2]),
      lbm_dec_as_float(arg_dec.args[3]),
      lbm_dec_as_float(arg_dec.args[4]),
      &p);

  return ENC_SYM_TRUE;
}

// lisp args: img cx cy r ang-s ang-e color opt-attr1 ... opt-attrN
static lbm_value ext_circle_segment(lbm_value *args, lbm_uint argn) {
  img_args_t arg_dec = decode_args(args, argn, 6);

  if (!arg_dec.is_valid) {
    return ENC_SYM_TERROR;
  }

  arc_params_t p = make_arc_params(&arg_dec, true, false, true, arg_dec.args[5]);
  tinygfx_arc(&arg_dec.img,
      lbm_dec_as_i32(arg_dec.args[0]),
      lbm_dec_as_i32(arg_dec.args[1]),
      lbm_dec_as_i32(arg_dec.args[2]),
      lbm_dec_as_float(arg_dec.args[3]),
      lbm_dec_as_float(arg_dec.args[4]),
      &p);


  return ENC_SYM_TRUE;
}

// lisp args: img x y width height color opt-attr1 ... opt-attrN
static lbm_value ext_rectangle(lbm_value *args, lbm_uint argn) {
  img_args_t arg_dec = decode_args(args, argn, 5);

  if (!arg_dec.is_valid) {
    return ENC_SYM_TERROR;
  }

  image_buffer_t *img = &arg_dec.img;
  int x = lbm_dec_as_i32(arg_dec.args[0]);
  int y = lbm_dec_as_i32(arg_dec.args[1]);
  int width = lbm_dec_as_i32(arg_dec.args[2]);
  int height = lbm_dec_as_i32(arg_dec.args[3]);
  int rad = lbm_dec_as_i32(arg_dec.attr_rounded.args[0]);
  int thickness = lbm_dec_as_i32(arg_dec.attr_thickness.args[0]);
  uint32_t color = lbm_dec_as_u32(arg_dec.args[4]);
  int dot1 = lbm_dec_as_i32(arg_dec.attr_dotted.args[0]);
  int dot2 = lbm_dec_as_i32(arg_dec.attr_dotted.args[1]);
  int resolution = lbm_dec_as_i32(arg_dec.attr_resolution.args[0]);

  if (arg_dec.attr_rounded.is_valid) {
    if (arg_dec.attr_filled.is_valid) {
      tinygfx_fill_rounded_rectangle(img, x, y, width, height, rad, color);
    } else {
      tinygfx_rounded_rectangle(img, x, y, width, height, rad, thickness, dot1, dot2, resolution, color);
    }
  } else {
    tinygfx_rectangle(img,
              x, y,
              width, height,
              arg_dec.attr_filled.is_valid,
              thickness,
              dot1, dot2,
              color);
  }

  return ENC_SYM_TRUE;
}

// lisp args: img x1 y1 x2 y2 x3 y3 color opt-attr1 ... opt-attrN
static lbm_value ext_triangle(lbm_value *args, lbm_uint argn) {
  img_args_t arg_dec = decode_args(args, argn, 7);

  if (!arg_dec.is_valid) {
    return ENC_SYM_TERROR;
  }

  image_buffer_t *img = &arg_dec.img;
  int x0 = lbm_dec_as_i32(arg_dec.args[0]);
  int y0 = lbm_dec_as_i32(arg_dec.args[1]);
  int x1 = lbm_dec_as_i32(arg_dec.args[2]);
  int y1 = lbm_dec_as_i32(arg_dec.args[3]);
  int x2 = lbm_dec_as_i32(arg_dec.args[4]);
  int y2 = lbm_dec_as_i32(arg_dec.args[5]);
  int thickness = lbm_dec_as_i32(arg_dec.attr_thickness.args[0]);
  int dot1 = lbm_dec_as_i32(arg_dec.attr_dotted.args[0]);
  int dot2 = lbm_dec_as_i32(arg_dec.attr_dotted.args[1]);
  uint32_t color = lbm_dec_as_u32(arg_dec.args[6]);

  if (arg_dec.attr_filled.is_valid) {
    tinygfx_fill_triangle(img, x0, y0, x1, y1, x2, y2, color);
  } else {
    tinygfx_line(img, x0, y0, x1, y1, thickness, dot1, dot2, color);
    tinygfx_line(img, x1, y1, x2, y2, thickness, dot1, dot2, color);
    tinygfx_line(img, x2, y2, x0, y0, thickness, dot1, dot2, color);
  }

  return ENC_SYM_TRUE;
}

static bool font_data_is_valid(const lbm_array_header_t *font) {
  if (font->size < 4) {
    return false;
  }

  const uint8_t *font_data = (const uint8_t*)font->data;
  uint8_t bits_per_pixel = font_data[3];
  if (font_data[0] == 0 || font_data[1] == 0 || font_data[2] == 0 ||
      (bits_per_pixel != 1 && bits_per_pixel != 2)) {
    return false;
  }

  uint32_t bytes_per_char =
    ((uint32_t)font_data[0] * font_data[1] * bits_per_pixel + 7) / 8;
  uint32_t required_size = 4 + bytes_per_char * font_data[2];
  return font->size >= required_size;
}

// lisp args:
//   img x y fg bg font str  [attrs] ['up|'down]
//   img x y fg bg font-id str [attrs] ['up|'down]  uses a compiled-in font
//   img x y fg bg str       [attrs] ['up|'down]   uses TINYGFX_DEFAULT_FONT_ID
//   img x y '(c0..c3) font str [attrs] ['up|'down]  4-color form for 2bpp fonts
// attrs: '(magnify N) '(scale N) '(spacing N) '(align 'left|'center|'right) '(rotate deg)
// orient: 0=normal 1=up/90CCW 2=180 3=down/90CW
static lbm_value ext_text(lbm_value *args, lbm_uint argn) {
  int orient = 0;

  if (argn >= 6 && lbm_is_symbol(args[argn - 1])) {
    lbm_uint sym = lbm_dec_sym(args[argn - 1]);
    if (sym == symbol_up)   { orient = 1; argn--; }
    if (sym == symbol_down) { orient = 3; argn--; }
  }

  if (argn < 6) return ENC_SYM_TERROR;

  float txt_mag = 1.0f;
  int spacing = 0;
  int align   = 0;
  int rot_deg = 0;

  lbm_uint core_argn = argn;
  while (core_argn > 0) {
    int r = parse_text_attr(args[core_argn - 1], &txt_mag, &spacing, &align, &rot_deg);
    if (r == 1) { core_argn--; continue; }
    if (r < 0)  { return ENC_SYM_TERROR; }
    break;
  }

  if (core_argn != 6 && core_argn != 7) return ENC_SYM_TERROR;

  if (rot_deg != 0) {
    orient = (((rot_deg % 360) + 360) % 360) / 90;
  }

  int x = lbm_dec_as_i32(args[1]);
  int y = lbm_dec_as_i32(args[2]);

  int32_t colors[4] = {-1, -1, -1, -1};
  const uint8_t *font_data = NULL;
  char *txt = NULL;

  if (lbm_is_cons(args[3])) {
    // color list form: img x y '(c0..cN) font str
    if (core_argn != 6) return ENC_SYM_TERROR;
    lbm_value curr = args[3];
    int ci = 0;
    while (lbm_is_cons(curr) && ci < 4) {
      lbm_value a = lbm_car(curr);
      if (!lbm_is_number(a)) return ENC_SYM_TERROR;
      colors[ci++] = lbm_dec_as_i32(a);
      curr = lbm_cdr(curr);
    }
    if (lbm_type_of_functional(args[4]) == LBM_TYPE_ARRAY) {
      lbm_array_header_t *fh = (lbm_array_header_t*)lbm_car(args[4]);
      if (!font_data_is_valid(fh)) return ENC_SYM_TERROR;
      const uint8_t *fd = (const uint8_t*)fh->data;
      font_data = fd;
    } else if (lbm_is_number(args[4])) {
      font_data = tinygfx_get_builtin_font(lbm_dec_as_i32(args[4]));
      if (!font_data) return ENC_SYM_TERROR;
    } else {
      return ENC_SYM_TERROR;
    }
    txt = lbm_dec_str(args[5]);
  } else if (lbm_is_number(args[3]) && lbm_is_number(args[4])) {
    colors[0] = lbm_dec_as_i32(args[3]);
    colors[1] = lbm_dec_as_i32(args[4]);
    if (core_argn == 7) {
      if (lbm_type_of_functional(args[5]) == LBM_TYPE_ARRAY) {
        lbm_array_header_t *fh = (lbm_array_header_t*)lbm_car(args[5]);
        if (!font_data_is_valid(fh)) return ENC_SYM_TERROR;
        const uint8_t *fd = (const uint8_t*)fh->data;
        font_data = fd;
      } else if (lbm_is_number(args[5])) {
        font_data = tinygfx_get_builtin_font(lbm_dec_as_i32(args[5]));
        if (!font_data) return ENC_SYM_TERROR;
      } else {
        return ENC_SYM_TERROR;
      }
      txt = lbm_dec_str(args[6]);
    } else {
      font_data = tinygfx_get_builtin_font(TINYGFX_DEFAULT_FONT_ID);
      if (!font_data) return ENC_SYM_TERROR;
      txt = lbm_dec_str(args[5]);
    }
  } else {
    return ENC_SYM_TERROR;
  }

  if (!txt) return ENC_SYM_TERROR;

  if (!array_is_image_buffer(args[0])) return ENC_SYM_TERROR;
  lbm_array_header_t *arr = (lbm_array_header_t *)lbm_car(args[0]);
  image_buffer_t img_buf;
  img_buf.width    = image_buffer_width((uint8_t*)arr->data);
  img_buf.height   = image_buffer_height((uint8_t*)arr->data);
  img_buf.fmt      = image_buffer_format((uint8_t*)arr->data);
  img_buf.mem_base = (uint8_t*)arr->data;
  img_buf.data     = image_buffer_data((uint8_t*)arr->data);

  uint8_t w = font_data[0];

  static const int8_t incx_tbl[4] = { 1,  0, -1,  0};
  static const int8_t incy_tbl[4] = { 0, -1,  0,  1};
  int incx = incx_tbl[orient];
  int incy = incy_tbl[orient];

  float char_step_f = (float)w * txt_mag + (float)spacing;
  int char_step = (int)lroundf(char_step_f);
  int txt_len      = (int)strlen(txt);
  int total        = (int)lroundf((float)txt_len * char_step_f - (float)spacing);
  int align_offset = (align == 1) ? total / 2 : (align == 2) ? total : 0;

  int x0 = x - align_offset * incx;
  int y0 = y - align_offset * incy;

  for (int ind = 0; txt[ind] != 0; ind++) {
    tinygfx_img_putc(&img_buf,
      x0 + ind * char_step * incx,
      y0 + ind * char_step * incy,
      (uint32_t *)colors,
      4,
      font_data,
      (uint8_t)txt[ind],
      orient,
      txt_mag);
  }

  return ENC_SYM_TRUE;
}


// Number of distinct index values an indexed format can hold, 0 if fmt isn't indexed.
static int indexed_depth(color_format_t fmt) {
  switch (fmt) {
  case indexed2: return 2;
  case indexed4: return 4;
  case indexed16: return 16;
  default: return 0;
  }
}

// Decodes a plain numeric list (as opposed to lbm_display_decode_color_list's
// color-or-number list) into out[], up to max_len entries. Reports the exact
// decoded length via out_len so callers can validate it against a required size.
static bool decode_palette_list(lbm_value lst, uint32_t *out, int max_len, int *out_len) {
  int i = 0;
  lbm_value curr = lst;
  while (lbm_is_cons(curr)) {
    if (i >= max_len) return false;
    lbm_value arg = lbm_car(curr);
    if (!lbm_is_number(arg)) return false;
    out[i] = lbm_dec_as_u32(arg);
    i++;
    curr = lbm_cdr(curr);
  }
  if (curr != ENC_SYM_NIL) return false;
  *out_len = i;
  return true;
}

// TODO: Think about way to simplify the blit dispatch
//       based on transform or not.
static lbm_value ext_blit(lbm_value *args, lbm_uint argn) {
  img_args_t arg_dec = decode_args(args + 1, argn - 1, 3);

  lbm_value res = ENC_SYM_TERROR;
  lbm_array_header_t *arr;
  if (arg_dec.is_valid && (arr = get_image_buffer(args[0]))) { //assignment
    image_buffer_t dest_buf;
    dest_buf.width = image_buffer_width((uint8_t*)arr->data);
    dest_buf.height = image_buffer_height((uint8_t*)arr->data);
    dest_buf.fmt = image_buffer_format((uint8_t*)arr->data);
    dest_buf.mem_base = (uint8_t*)arr->data;
    dest_buf.data = image_buffer_data((uint8_t*)arr->data);

    int dest_x = lbm_dec_as_i32(arg_dec.args[0]);
    int dest_y = lbm_dec_as_i32(arg_dec.args[1]);
    int32_t transparent_color = lbm_dec_as_i32(arg_dec.args[2]);

    int src_depth = indexed_depth(arg_dec.img.fmt);
    int dest_depth = indexed_depth(dest_buf.fmt);

    if (src_depth == 0 && dest_depth != 0) {
      lbm_set_error_reason("img-blit: an rgb source cannot be blitted into an indexed destination.");
      return ENC_SYM_EERROR;
    }

    uint32_t palette[16];
    const uint32_t *palette_ptr = NULL;
    int palette_len = 0;

    if (arg_dec.attr_palette.is_valid) {
      if (src_depth == 0 ||
          !decode_palette_list(arg_dec.attr_palette.args[0], palette, 16, &palette_len) ||
          palette_len != src_depth) {
        lbm_set_error_reason("img-blit: palette must be a list with exactly as many entries as the source image's color space.");
        return ENC_SYM_EERROR;
      }
      palette_ptr = palette;
    } else if (src_depth != 0 && dest_depth != 0 && dest_depth >= src_depth) {
      // Indexed source into an indexed dest with at least as many colors:
      // source index i is a valid dest index unchanged, so the
      // getpixel/putpixel fallback below (no compose) is already an
      // identity mapping and needs no palette.
    } else if (src_depth != 0 && arg_dec.img.fmt != dest_buf.fmt) {
      lbm_set_error_reason("img-blit: needs a palette attribute here.");
      return ENC_SYM_EERROR;
    }

    uint8_t alpha = 255;
    if (arg_dec.attr_alpha.is_valid) {
      int32_t a = lbm_dec_as_i32(arg_dec.attr_alpha.args[0]);
      if (a < 0) a = 0;
      if (a > 255) a = 255;
      alpha = (uint8_t)a;
    }

    image_buffer_t alpha_buf_img;
    image_buffer_t *alpha_buf_ptr = NULL;
    if (arg_dec.attr_alpha_buffer.is_valid) {
      lbm_array_header_t *abuf_arr = get_image_buffer(arg_dec.attr_alpha_buffer.args[0]);
      color_format_t abuf_fmt = abuf_arr ? image_buffer_format((uint8_t*)abuf_arr->data) : format_not_supported;
      uint16_t abuf_w = abuf_arr ? image_buffer_width((uint8_t*)abuf_arr->data) : 0;
      uint16_t abuf_h = abuf_arr ? image_buffer_height((uint8_t*)abuf_arr->data) : 0;
      if (!abuf_arr ||
          (abuf_fmt != indexed2 && abuf_fmt != indexed4 && abuf_fmt != indexed16 && abuf_fmt != rgb332) ||
          abuf_w != arg_dec.img.width || abuf_h != arg_dec.img.height) {
        lbm_set_error_reason("img-blit: alpha-buffer must be an indexed2/4/16 or rgb332 image buffer with the same dimensions as the source.");
        return ENC_SYM_EERROR;
      }
      alpha_buf_img.width = abuf_w;
      alpha_buf_img.height = abuf_h;
      alpha_buf_img.fmt = abuf_fmt;
      alpha_buf_img.mem_base = (uint8_t*)abuf_arr->data;
      alpha_buf_img.data = image_buffer_data((uint8_t*)abuf_arr->data);
      alpha_buf_ptr = &alpha_buf_img;
    }

    blit_compose_t compose = {
      .palette = palette_ptr,
      .palette_len = palette_len,
      .alpha = alpha,
      .alpha_buf = alpha_buf_ptr,
    };
    const blit_compose_t *compose_ptr =
      (palette_ptr || alpha != 255 || alpha_buf_ptr) ? &compose : NULL;

    if (!arg_dec.transform_attr_present) {
      tinygfx_blit(&dest_buf, &arg_dec.img, dest_x, dest_y, transparent_color, compose_ptr);
    } else {
      blit_transform_t transform = {
        .rot_x = lbm_dec_as_float(arg_dec.attr_rotate.args[0]),
        .rot_y = lbm_dec_as_float(arg_dec.attr_rotate.args[1]),
        .rot_angle = lbm_dec_as_float(arg_dec.attr_rotate.args[2]),
        .scale = arg_dec.attr_scale.is_valid ? lbm_dec_as_float(arg_dec.attr_scale.args[0]) : 1.0f,
        .tile = arg_dec.attr_tile.is_valid,
        .clip_x = arg_dec.attr_clip.is_valid ? lbm_dec_as_i32(arg_dec.attr_clip.args[0]) : 0,
        .clip_y = arg_dec.attr_clip.is_valid ? lbm_dec_as_i32(arg_dec.attr_clip.args[1]) : 0,
        .clip_w = arg_dec.attr_clip.is_valid ? lbm_dec_as_i32(arg_dec.attr_clip.args[2]) : dest_buf.width,
        .clip_h = arg_dec.attr_clip.is_valid ? lbm_dec_as_i32(arg_dec.attr_clip.args[3]) : dest_buf.height,
      };

      tinygfx_blit_transform(&dest_buf, &arg_dec.img, dest_x, dest_y, transform, transparent_color, compose_ptr);
    }
    res = ENC_SYM_TRUE;
  }
  return res;
}

void display_dummy_reset(void) {
  return;
}

void display_dummy_clear(uint32_t color) {
  (void) color;
  return;
}

bool display_dummy_render_image(image_buffer_t *img, uint16_t x, uint16_t y,  color_t *colors) {
  (void) img;
  (void) x;
  (void) y;
  (void) colors;
  return false;
}

static bool(* volatile disp_render_image)(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) = display_dummy_render_image;
static void(* volatile disp_clear)(uint32_t color) = display_dummy_clear;
static void(* volatile disp_reset)(void) = display_dummy_reset;

static char *msg_not_supported = "Command not supported or display driver not initialized";

static lbm_value ext_disp_reset(lbm_value *args, lbm_uint argn) {
  (void) args;
  (void) argn;

  if (disp_reset == NULL) {
    lbm_set_error_reason(msg_not_supported);
    return ENC_SYM_EERROR;
  }

  disp_reset();

  return ENC_SYM_TRUE;
}

static lbm_value ext_disp_clear(lbm_value *args, lbm_uint argn) {
  if (disp_clear == NULL) {
    lbm_set_error_reason(msg_not_supported);
    return ENC_SYM_EERROR;
  }

  if (argn > 1) {
    return ENC_SYM_TERROR;
  }

  uint32_t clear_color = 0;

  if (argn == 1) {
    if (!lbm_is_number(args[0])) {
      return ENC_SYM_TERROR;
    }

    clear_color = lbm_dec_as_u32(args[0]);
  }

  disp_clear(clear_color);

  return ENC_SYM_TRUE;
}

bool lbm_display_decode_color_list(lbm_value color_list, color_t *colors, int max_colors) {
  memset(colors, 0, sizeof(color_t) * (size_t)max_colors);
  if (!lbm_is_list(color_list)) {
    return true;
  }
  int i = 0;
  lbm_value curr = color_list;
  while (lbm_is_cons(curr) && i < max_colors) {
    lbm_value arg = lbm_car(curr);
    color_t *color;
    if (lbm_is_number(arg)) {
      colors[i].color1 = (int)lbm_dec_as_u32(arg);
    } else if ((color = get_color(arg))) { // color assignment
      colors[i] = *color;
    } else {
      return false;
    }

    curr = lbm_cdr(curr);
    i++;
  }
  return true;
}

static lbm_value ext_disp_render(lbm_value *args, lbm_uint argn) {
  if (disp_render_image == NULL) {
    lbm_set_error_reason(msg_not_supported);
    return ENC_SYM_EERROR;
  }

  lbm_value res = ENC_SYM_TERROR;
  lbm_array_header_t *arr;
  if ((argn == 3 || argn == 4) &&
      (arr = get_image_buffer(args[0])) &&
      lbm_is_number(args[1]) &&
      lbm_is_number(args[2])) {
    image_buffer_t img_buf;
    img_buf.fmt = image_buffer_format((uint8_t*)arr->data);
    img_buf.width = image_buffer_width((uint8_t*)arr->data);
    img_buf.height = image_buffer_height((uint8_t*)arr->data);
    img_buf.mem_base = (uint8_t*)arr->data;
    img_buf.data = image_buffer_data((uint8_t*)arr->data);

    color_t colors[16];
    if (argn == 4 && !lbm_display_decode_color_list(args[3], colors, 16)) {
      return ENC_SYM_TERROR;
    } else if (argn == 3) {
      memset(colors, 0, sizeof(color_t) * 16);
    }

    // img_buf is a stack allocated image_buffer_t.
    bool render_res = disp_render_image(&img_buf, (uint16_t)lbm_dec_as_u32(args[1]), (uint16_t)lbm_dec_as_u32(args[2]), colors);
    if (!render_res) {
      lbm_set_error_reason("Could not render image. Check if the format and location is compatible with the display.");
      return ENC_SYM_EERROR;
    }
    res = ENC_SYM_TRUE;
  }
  return res;
}

// Jpg decoder

int jpg_output_func (	/* 1:Ok, 0:Aborted */
                     JDEC* jd,		/* Decompression object */
                     void* bitmap,	/* Bitmap data to be output */
                     JRECT* rect		/* Rectangular region to output */
                        ) {
  tinygfx_jpg_io_t *dev = (tinygfx_jpg_io_t*)jd->device;

  image_buffer_t img;
  img.mem_base = (uint8_t*)bitmap;
  img.data = (uint8_t*)bitmap;
  img.width = (uint16_t)(rect->right - rect->left + 1);
  img.height = (uint16_t)(rect->bottom - rect->top + 1);
  img.fmt = rgb888;

  disp_render_image(&img, (uint16_t)(rect->left + dev->ofs_x), (uint16_t)(rect->top + dev->ofs_y), 0);

  return 1;
}

static lbm_value ext_disp_render_jpg(lbm_value *args, lbm_uint argn) {

  lbm_array_header_t *array;
  lbm_value res = ENC_SYM_TERROR;

  if (argn == 3 &&
      (array = lbm_dec_array_r(args[0])) && //asignment
      lbm_is_number(args[1]) &&
      lbm_is_number(args[2])) {

    JDEC jd;
    void *jdwork;
    // make a bit of room before the buffer.
    const size_t sz_work = 4096 + IMAGE_BUFFER_HEADER_SIZE;

    jdwork = lbm_malloc(sz_work);
    if (!jdwork) {
      return ENC_SYM_MERROR;
    }



    tinygfx_jpg_io_t iodev;
    iodev.data = (uint8_t*)(array->data);
    iodev.size = array->size;
    iodev.pos = 0;
    iodev.dest = NULL;
    iodev.ofs_x = lbm_dec_as_i32(args[1]);
    iodev.ofs_y = lbm_dec_as_i32(args[2]);
    jd_prepare(&jd, tinygfx_jpg_input, jdwork, sz_work, &iodev);
    jd_decomp(&jd, jpg_output_func, 0);
    lbm_free(jdwork);
    res = ENC_SYM_TRUE;
  }
  return res;
}

static lbm_value ext_img_render_jpg(lbm_value *args, lbm_uint argn) {

  lbm_array_header_t *dest_arr;
  lbm_array_header_t *jpg_arr;
  lbm_value res = ENC_SYM_TERROR;

  if (argn == 4 &&
      (dest_arr = get_image_buffer(args[0])) && //assignment
      (jpg_arr = lbm_dec_array_r(args[1])) &&    //assignment
      lbm_is_number(args[2]) &&
      lbm_is_number(args[3])) {

    image_buffer_t dest_buf;
    dest_buf.width = image_buffer_width((uint8_t*)dest_arr->data);
    dest_buf.height = image_buffer_height((uint8_t*)dest_arr->data);
    dest_buf.fmt = image_buffer_format((uint8_t*)dest_arr->data);
    dest_buf.mem_base = (uint8_t*)dest_arr->data;
    dest_buf.data = image_buffer_data((uint8_t*)dest_arr->data);

    const size_t sz_work = 4096;
    void *jdwork = lbm_malloc(sz_work);
    if (!jdwork) {
      return ENC_SYM_MERROR;
    }

    bool ok = tinygfx_decode_jpg(&dest_buf,
                                  (uint8_t*)jpg_arr->data, jpg_arr->size,
                                  lbm_dec_as_i32(args[2]), lbm_dec_as_i32(args[3]),
                                  jdwork, sz_work);
    lbm_free(jdwork);
    res = ok ? ENC_SYM_TRUE : ENC_SYM_NIL;
  }
  return res;
}

static bool scene_decode_integer(lbm_value value, int32_t *out, int32_t min, int32_t max) {
  if (!lbm_is_number(value)) {
    return false;
  }

  double number = lbm_dec_as_double(value);

  if (!(number >= min && number <= max)) {
    return false;
  }

  *out = (int32_t)number;
  return number == *out;
}

static size_t scene_size_bytes(display_scene_t *scene) {
  return sizeof(*scene) + scene->slots * sizeof(tinygfx_rect_t) +
    (size_t)scene->tile_width * scene->tile_height * 2;
}

static display_scene_t *scene_get_state(lbm_value value) {
  if (!lbm_is_cons(value)) {
    return NULL;
  }

  lbm_array_header_t *array = lbm_dec_array_rw(lbm_car(value));

  if (!array || array->size < sizeof(display_scene_t)) {
    return NULL;
  }

  display_scene_t *scene = (display_scene_t *)array->data;

  if (scene->magic != SCENE_MAGIC || !scene->slots || scene->slots > SCENE_MAX_SLOTS ||
      !scene->tile_width || !scene->tile_height || !scene->scheduler.width ||
      !scene->scheduler.height || scene->scheduler.width > MAX_WIDTH ||
      scene->scheduler.height > MAX_HEIGHT ||
      scene->scheduler.dirty_count > TINYGFX_SCENE_DIRTY_MAX ||
      array->size != scene_size_bytes(scene)) {
    return NULL;
  }

  lbm_value tail = lbm_cdr(value);

  for (unsigned i = 0; i < scene->slots; i++) {
    if (!lbm_is_cons(tail)) {
      return NULL;
    }
    tail = lbm_cdr(tail);
  }

  if (tail != ENC_SYM_NIL) {
    return NULL;
  }

  for (unsigned i = 0; i < scene->scheduler.dirty_count; i++) {
    tinygfx_rect_t bounds = scene->scheduler.dirty[i];
    if (bounds.x < 0 || bounds.y < 0 || bounds.w <= 0 || bounds.h <= 0 || (int64_t)bounds.x + bounds.w > scene->scheduler.width ||
        (int64_t)bounds.y + bounds.h > scene->scheduler.height) {
      return NULL;
    }
  }
  return scene;
}

static bool scene_color_is_valid(lbm_value value) {
  if (lbm_is_number(value)) {
    return true;
  }

  color_t *color = get_color(value);

  return color &&
    (color->type == COLOR_REGULAR ||
     ((color->type == COLOR_GRADIENT_X || color->type == COLOR_GRADIENT_Y) && color->param1));
}

static bool scene_palette_is_valid(lbm_value palette, unsigned count) {
  for (unsigned i = 0; i < count; i++) {
    if (!lbm_is_cons(palette) || !scene_color_is_valid(lbm_car(palette))) {
      return false;
    }
    palette = lbm_cdr(palette);
  }
  return palette == ENC_SYM_NIL;
}

static bool scene_decode_rect(lbm_value *fields, display_scene_node_t *node) {
  if (!scene_decode_integer(fields[3], &node->bounds.w, 1, MAX_WIDTH) ||
      !scene_decode_integer(fields[4], &node->bounds.h, 1, MAX_HEIGHT)) {
    return false;
  }
  unsigned color_index = 5;
  if (node->kind == SCENE_ROUNDRECT) {
    int32_t radius;
    if (!scene_decode_integer(fields[5], &radius, 0, 16000)) {
      return false;
    }
    node->radius = radius;
    color_index = 6;
  }
  if (!scene_color_is_valid(fields[color_index])) {
    return false;
  }
  node->color = fields[color_index];
  return true;
}

static bool scene_decode_circle(lbm_value *fields, display_scene_node_t *node) {
  int32_t radius;
  if (!scene_decode_integer(fields[3], &radius, 0, MAX_WIDTH / 2) ||
      !scene_color_is_valid(fields[4])) {
    return false;
  }
  node->radius = radius;
  node->bounds.x -= radius;
  node->bounds.y -= radius;
  node->bounds.w = node->bounds.h = radius * 2 + 1;
  node->color = fields[4];
  return true;
}

static bool scene_decode_ttf(lbm_value *fields, display_scene_node_t *node) {
  lbm_array_header_t *font = lbm_dec_array_r(fields[7]), *text = lbm_dec_array_r(fields[8]);
  if (!font || !text || !text->size || text->size > 4096 || !memchr(text->data, 0, text->size) ||
      !scene_decode_integer(fields[3], &node->bounds.w, 1, MAX_WIDTH) ||
      !scene_decode_integer(fields[4], &node->bounds.h, 1, MAX_HEIGHT) ||
      !scene_decode_integer(fields[5], &node->pen_x, -MAX_WIDTH, MAX_WIDTH) ||
      !scene_decode_integer(fields[6], &node->pen_y, -MAX_HEIGHT, MAX_HEIGHT) ||
      !tinygfx_prepared_font_init(&node->prepared, (uint8_t *)font->data, font->size)) {
    return false;
  }
  node->text = (const char *)text->data;
  node->palette = fields[9];
  if (!scene_palette_is_valid(node->palette, (1u << node->prepared.fmt))) {
    return false;
  }
  return true;
}

static bool scene_decode_text(lbm_value *fields, display_scene_node_t *node) {
  int32_t scale, font_id;
  lbm_array_header_t *text = lbm_dec_array_r(fields[4]);
  if (!text || !text->size || !scene_color_is_valid(fields[5]) ||
      !scene_decode_integer(fields[6], &scale, 1, 32)) {
    return false;
  }
  const char *end = memchr(text->data, 0, text->size);
  if (!end) {
    return false;
  }
  lbm_array_header_t *font = lbm_dec_array_r(fields[3]);
  if (font) {
    if (!font_data_is_valid(font)) {
      return false;
    }
    node->font = (const uint8_t *)font->data;
  } else {
    if (!scene_decode_integer(fields[3], &font_id, 0, 255) ||
        !(node->font = tinygfx_get_builtin_font(font_id))) {
      return false;
    }
  }
  node->text = (const char *)text->data;
  size_t length = (size_t)(end - node->text);
  if (length > (size_t)MAX_WIDTH / node->font[0] / (unsigned)scale) {
    return false;
  }
  node->text_scale = scale;
  node->bounds.w = (int32_t)length * node->font[0] * scale;
  node->bounds.h = node->font[1] * scale;
  node->color = fields[5];
  return true;
}

static bool scene_decode_image(lbm_value *fields, display_scene_node_t *node) {
  lbm_array_header_t *array = get_image_buffer(fields[3]);
  int32_t alpha;
  if (!array || !scene_decode_integer(fields[4], &node->transparent, -1, 0xFFFFFF) ||
      !scene_decode_integer(fields[6], &alpha, 0, 255)) {
    return false;
  }
  uint8_t *data = (uint8_t *)array->data;
  node->image.fmt = image_buffer_format(data);
  node->image.width = image_buffer_width(data);
  node->image.height = image_buffer_height(data);
  node->image.data = image_buffer_data(data);
  node->image.mem_base = data;
  if (!node->image.width || !node->image.height || node->image.width > MAX_WIDTH ||
      node->image.height > MAX_HEIGHT ||
      !image_dims_to_size_bytes(node->image.fmt, node->image.width, node->image.height) ||
      array->size < IMAGE_BUFFER_HEADER_SIZE +
          image_dims_to_size_bytes(node->image.fmt, node->image.width, node->image.height)) {
    return false;
  }
  node->bounds.w = node->image.width;
  node->bounds.h = node->image.height;
  node->alpha = (uint8_t)alpha;
  node->palette = fields[5];
  unsigned colors = node->image.fmt <= indexed16 ? 1u << node->image.fmt : 0;
  if (!scene_palette_is_valid(node->palette, colors)) {
    return false;
  }
  return true;
}

static bool scene_decode_node(lbm_value value, display_scene_node_t *node) {
  lbm_value fields[11];
  unsigned count = 0;
  while (lbm_is_cons(value) && count < 11) {
    fields[count++] = lbm_car(value);
    value = lbm_cdr(value);
  }
  if (value != ENC_SYM_NIL || count < 5 || !lbm_is_symbol(fields[0])) {
    return false;
  }
  memset(node, 0, sizeof(*node));
  lbm_uint symbol = lbm_dec_sym(fields[0]);
  node->kind = SCENE_KIND_COUNT;
  for (unsigned i = 0; i < SCENE_KIND_COUNT; i++) {
    if (symbol == scene_symbols[i].symbol) {
      node->kind = (display_scene_kind_t)i;
      break;
    }
  }
  node->alpha = 255;
  node->palette = ENC_SYM_NIL;
  if (!scene_decode_integer(fields[1], &node->bounds.x, -MAX_WIDTH, MAX_WIDTH) ||
      !scene_decode_integer(fields[2], &node->bounds.y, -MAX_HEIGHT, MAX_HEIGHT)) {
    return false;
  }
  if ((node->kind == SCENE_RECT && count == 6) ||
      (node->kind == SCENE_ROUNDRECT && count == 7)) {
    return scene_decode_rect(fields, node);
  }
  if (node->kind == SCENE_CIRCLE && count == 5) {
    return scene_decode_circle(fields, node);
  }
  if (node->kind == SCENE_TTF && count == 10) {
    return scene_decode_ttf(fields, node);
  }
  if (node->kind == SCENE_TEXT && count == 7) {
    return scene_decode_text(fields, node);
  }
  if (node->kind == SCENE_IMAGE && count == 7) {
    return scene_decode_image(fields, node);
  }
  return false;
}

static uint32_t scene_get_color(lbm_value value, int x, int y, uint8_t *alpha) {
  color_t *color = get_color(value);
  *alpha = color ? color->alpha : 255;
  return color ? COLOR_TO_RGB888((*color), x, y) : lbm_dec_as_u32(value);
}

static void scene_putpixel(image_buffer_t *tile, int x, int y, uint32_t color, uint8_t alpha) {
  if (!alpha) {
    return;
  }
  if (alpha != 255) {
    uint32_t background = getpixel(tile, x, y), blended = 0;
    for (unsigned shift = 0; shift <= 16; shift += 8) {
      uint32_t channel =
        (((color >> shift) & 255) * alpha + ((background >> shift) & 255) * (255 - alpha) + 127) / 255;
      blended |= channel << shift;
    }
    color = blended;
  }
  putpixel(tile, x, y, color);
}

static bool scene_draw_glyph(image_buffer_t *glyph, int gx, int gy, void *user_data) {
  display_scene_glyph_context_t *context = user_data;
  int px = context->node->pen_x + gx, py = context->node->pen_y + gy;
  int x0 = px > context->x0 ? px : context->x0, y0 = py > context->y0 ? py : context->y0;
  int x1 = px + glyph->width < context->x1 ? px + glyph->width : context->x1;
  int y1 = py + glyph->height < context->y1 ? py + glyph->height : context->y1;
  for (int y = y0; y < y1; y++) {
    for (int x = x0; x < x1; x++) {
      uint32_t index = getpixel(glyph, x - px, y - py);
      if (index) {
        uint8_t alpha;
        uint32_t color = scene_get_color(context->palette[index], x, y, &alpha);
        scene_putpixel(context->tile, x - context->tx, y - context->ty, color, alpha);
      }
    }
  }
  return true;
}

static bool scene_render_ttf(image_buffer_t *tile, display_scene_node_t *node,
                             int tx, int ty, tinygfx_rect_t clip) {
  display_scene_glyph_context_t context = {
    .tile = tile,
    .node = node,
    .tx = tx,
    .ty = ty,
    .x0 = clip.x,
    .y0 = clip.y,
    .x1 = clip.x + clip.w,
    .y1 = clip.y + clip.h
  };
  lbm_value palette = node->palette;
  for (unsigned i = 0; i < (1u << node->prepared.fmt); i++) {
    context.palette[i] = lbm_car(palette);
    palette = lbm_cdr(palette);
  }

  color_t *background = get_color(context.palette[0]);
  if (!background || background->alpha) {
    for (int y = clip.y; y < clip.y + clip.h; y++) {
      for (int x = clip.x; x < clip.x + clip.w; x++) {
        uint8_t alpha;
        uint32_t color = scene_get_color(context.palette[0], x, y, &alpha);
        scene_putpixel(tile, x - tx, y - ty, color, alpha);
      }
    }
  }
  return tinygfx_prepared_text(&node->prepared, node->text, scene_draw_glyph, &context);
}

static void scene_render_image(image_buffer_t *tile, display_scene_node_t *node,
                               int tx, int ty, tinygfx_rect_t clip) {
  lbm_value palette[16];
  if (node->image.fmt <= indexed16) {
    lbm_value entries = node->palette;
    for (unsigned i = 0; i < (1u << node->image.fmt); i++) {
      palette[i] = lbm_car(entries);
      entries = lbm_cdr(entries);
    }
  }

  for (int y = clip.y; y < clip.y + clip.h; y++) {
    for (int x = clip.x; x < clip.x + clip.w; x++) {
      uint32_t color = getpixel(&node->image, x - node->bounds.x, y - node->bounds.y);
      if (node->transparent != -1 && color == (uint32_t)node->transparent) {
        continue;
      }
      uint8_t alpha = 255;
      if (node->image.fmt <= indexed16) {
        color = scene_get_color(palette[color], x, y, &alpha);
      }
      alpha = (uint8_t)(((unsigned)alpha * node->alpha + 127) / 255);
      scene_putpixel(tile, x - tx, y - ty, color, alpha);
    }
  }
}

static unsigned scene_text_coverage(display_scene_node_t *node, int x, int y) {
  unsigned column = (unsigned)(x - node->bounds.x) / (unsigned)node->text_scale;
  unsigned row = (unsigned)(y - node->bounds.y) / (unsigned)node->text_scale;
  unsigned width = node->font[0];
  unsigned height = node->font[1];
  unsigned bits_per_pixel = node->font[3];
  unsigned character = (uint8_t)node->text[column / width];
  unsigned first_character = node->font[2] == 10 ? '0' : ' ';
  if (character < first_character || character - first_character >= node->font[2]) {
    return 0;
  }

  unsigned bit = (row * width + column % width) * bits_per_pixel;
  unsigned stride = (width * height * bits_per_pixel + 7) / 8;
  unsigned offset = 4 + (character - first_character) * stride + bit / 8;
  unsigned mask = (1u << bits_per_pixel) - 1;
  unsigned sample = (node->font[offset] >> (bit % 8)) & mask;
  return sample * 255 / mask;
}

static bool scene_contains_pixel(display_scene_node_t *node, int x, int y) {
  int local_x = x - node->bounds.x;
  int local_y = y - node->bounds.y;
  if (node->kind == SCENE_CIRCLE) {
    int dx = local_x - node->radius;
    int dy = local_y - node->radius;
    return (int64_t)dx * dx + (int64_t)dy * dy <= (int64_t)node->radius * node->radius;
  }
  if (node->kind != SCENE_ROUNDRECT) {
    return true;
  }

  int radius = node->radius;
  if (radius > node->bounds.w / 2) {
    radius = node->bounds.w / 2;
  }
  if (radius > node->bounds.h / 2) {
    radius = node->bounds.h / 2;
  }
  int dx = 0;
  int dy = 0;
  if (local_x < radius) {
    dx = radius - local_x;
  } else if (local_x >= node->bounds.w - radius) {
    dx = local_x - (node->bounds.w - radius - 1);
  }
  if (local_y < radius) {
    dy = radius - local_y;
  } else if (local_y >= node->bounds.h - radius) {
    dy = local_y - (node->bounds.h - radius - 1);
  }
  return !dx || !dy || dx * dx + dy * dy <= radius * radius;
}

static bool scene_render_node(image_buffer_t *tile, display_scene_node_t *node, int tx, int ty) {
  int x0 = node->bounds.x > tx ? node->bounds.x : tx;
  int y0 = node->bounds.y > ty ? node->bounds.y : ty;
  int x1 = node->bounds.x + node->bounds.w;
  int y1 = node->bounds.y + node->bounds.h;
  if (x1 > tx + tile->width) {
    x1 = tx + tile->width;
  }
  if (y1 > ty + tile->height) {
    y1 = ty + tile->height;
  }
  if (x0 >= x1 || y0 >= y1) {
    return true;
  }
  if (node->kind == SCENE_ROUNDRECT && lbm_is_number(node->color)) {
    tinygfx_fill_rounded_rectangle(tile, node->bounds.x - tx, node->bounds.y - ty, node->bounds.w,
                                   node->bounds.h, node->radius, lbm_dec_as_u32(node->color));
    return true;
  }
  tinygfx_rect_t clip = {x0, y0, x1 - x0, y1 - y0};
  if (node->kind == SCENE_TTF) {
    return scene_render_ttf(tile, node, tx, ty, clip);
  }
  if (node->kind == SCENE_IMAGE) {
    scene_render_image(tile, node, tx, ty, clip);
    return true;
  }

  for (int y = y0; y < y1; y++) {
    for (int x = x0; x < x1; x++) {
      unsigned coverage = node->kind == SCENE_TEXT ? scene_text_coverage(node, x, y) : 255;
      if (!coverage || !scene_contains_pixel(node, x, y)) {
        continue;
      }
      uint8_t alpha;
      uint32_t color = scene_get_color(node->color, x, y, &alpha);
      alpha = (uint8_t)((alpha * coverage + 127) / 255);
      scene_putpixel(tile, x - tx, y - ty, color, alpha);
    }
  }
  return true;
}

static bool scene_render_tile(image_buffer_t *tile, int tx, int ty, void *user_data) {
  display_scene_render_context_t *context = user_data;
  image_buffer_clear(tile, context->state->background);
  lbm_value slots = context->slots;
  while (lbm_is_cons(slots)) {
    lbm_value value = lbm_car(slots);
    slots = lbm_cdr(slots);
    if (value != ENC_SYM_NIL) {
      display_scene_node_t node;
      if (!scene_decode_node(value, &node) || !scene_render_node(tile, &node, tx, ty)) {
        return false;
      }
    }
  }
  return true;
}

static bool scene_flush_tile(image_buffer_t *tile, int x, int y, void *context) {
  (void)context;
  return disp_render_image(tile, (uint16_t)x, (uint16_t)y, NULL);
}

static lbm_value ext_scene_create(lbm_value *args, lbm_uint argn) {
  if (argn != 3 && argn != 5 && argn != 6) {
    return ENC_SYM_TERROR;
  }
  int32_t width, height, slots;
  int32_t tile_width = 32;
  int32_t tile_height = 16;
  if (!scene_decode_integer(args[0], &width, 1, MAX_WIDTH) ||
      !scene_decode_integer(args[1], &height, 1, MAX_HEIGHT) ||
      !scene_decode_integer(args[2], &slots, 1, SCENE_MAX_SLOTS)) {
    return ENC_SYM_TERROR;
  }
  if (argn >= 5 &&
      (!scene_decode_integer(args[3], &tile_width, 1, MAX_WIDTH) ||
       !scene_decode_integer(args[4], &tile_height, 1, MAX_HEIGHT))) {
    return ENC_SYM_TERROR;
  }
  if (argn == 6 && !lbm_is_number(args[5])) {
    return ENC_SYM_TERROR;
  }
  if ((size_t)tile_width * tile_height > 32768) {
    return ENC_SYM_TERROR;
  }
  lbm_value result = lbm_heap_allocate_list((lbm_uint)slots + 1);
  if (!lbm_is_cons(result)) {
    return ENC_SYM_MERROR;
  }
  display_scene_t prototype = {0};
  prototype.slots = (uint16_t)slots;
  prototype.tile_width = (uint16_t)tile_width;
  prototype.tile_height = (uint16_t)tile_height;
  lbm_value state;
  if (!lbm_heap_allocate_array(&state, (lbm_uint)scene_size_bytes(&prototype))) {
    return ENC_SYM_MERROR;
  }
  lbm_set_car(result, state);
  lbm_value p = lbm_cdr(result);
  while (lbm_is_cons(p)) {
    lbm_set_car(p, ENC_SYM_NIL);
    p = lbm_cdr(p);
  }
  display_scene_t *scene = (display_scene_t *)lbm_dec_array_rw(state)->data;
  memset(scene, 0, scene_size_bytes(&prototype));
  *scene = prototype;
  scene->magic = SCENE_MAGIC;
  scene->background = argn == 6 ? lbm_dec_as_u32(args[5]) : 0;
  tinygfx_scene_init(&scene->scheduler, (uint16_t)width, (uint16_t)height);
  return result;
}

static lbm_value ext_scene_set(lbm_value *args, lbm_uint argn) {
  display_scene_t *scene;
  int32_t index;
  if (argn != 3 || !(scene = scene_get_state(args[0])) ||
      !scene_decode_integer(args[1], &index, 0, scene->slots - 1)) {
    return ENC_SYM_TERROR;
  }
  display_scene_node_t node;
  bool remove = args[2] == ENC_SYM_NIL;
  if (!remove && !scene_decode_node(args[2], &node)) {
    return ENC_SYM_TERROR;
  }
  if (!remove && node.kind == SCENE_TTF &&
      !tinygfx_prepared_text(&node.prepared, node.text, NULL, NULL)) {
    return ENC_SYM_TERROR;
  }
  tinygfx_rect_t *bounds = (tinygfx_rect_t *)(scene + 1);
  tinygfx_scene_invalidate(&scene->scheduler, bounds[index]);
  bounds[index] = remove ? (tinygfx_rect_t){0} : node.bounds;
  tinygfx_scene_invalidate(&scene->scheduler, bounds[index]);
  lbm_value cell = lbm_cdr(args[0]);
  for (int i = 0; i < index; i++) {
    cell = lbm_cdr(cell);
  }
  lbm_set_car(cell, args[2]);
  return ENC_SYM_TRUE;
}

static lbm_value ext_scene_invalidate(lbm_value *args, lbm_uint argn) {
  display_scene_t *scene;
  if ((argn != 1 && argn != 5) || !(scene = scene_get_state(args[0]))) {
    return ENC_SYM_TERROR;
  }
  tinygfx_rect_t r = {0, 0, scene->scheduler.width, scene->scheduler.height};
  if (argn == 5 &&
      (!scene_decode_integer(args[1], &r.x, -MAX_WIDTH, MAX_WIDTH) ||
       !scene_decode_integer(args[2], &r.y, -MAX_HEIGHT, MAX_HEIGHT) ||
       !scene_decode_integer(args[3], &r.w, 0, MAX_WIDTH) ||
       !scene_decode_integer(args[4], &r.h, 0, MAX_HEIGHT))) {
    return ENC_SYM_TERROR;
  }
  tinygfx_scene_invalidate(&scene->scheduler, r);
  return ENC_SYM_TRUE;
}

static lbm_value ext_scene_get(lbm_value *args, lbm_uint argn) {
  display_scene_t *scene;
  int32_t index;
  if (argn != 2 || !(scene = scene_get_state(args[0])) ||
      !scene_decode_integer(args[1], &index, 0, scene->slots - 1)) {
    return ENC_SYM_TERROR;
  }
  lbm_value cell = lbm_cdr(args[0]);
  for (int i = 0; i < index; i++) {
    cell = lbm_cdr(cell);
  }
  return lbm_car(cell);
}

static lbm_value ext_scene_background(lbm_value *args, lbm_uint argn) {
  display_scene_t *scene;
  if (argn != 2 || !(scene = scene_get_state(args[0])) || !lbm_is_number(args[1])) {
    return ENC_SYM_TERROR;
  }
  scene->background = lbm_dec_as_u32(args[1]);
  tinygfx_scene_invalidate(&scene->scheduler,
                           (tinygfx_rect_t){0, 0, scene->scheduler.width, scene->scheduler.height});
  return ENC_SYM_TRUE;
}

static lbm_value ext_scene_render(lbm_value *args, lbm_uint argn) {
  display_scene_t *scene;
  if (argn != 1 || !(scene = scene_get_state(args[0]))) {
    return ENC_SYM_TERROR;
  }
  if (!disp_render_image || disp_render_image == display_dummy_render_image) {
    lbm_set_error_reason(msg_not_supported);
    return ENC_SYM_EERROR;
  }
  lbm_value p = lbm_cdr(args[0]);
  while (lbm_is_cons(p)) {
    display_scene_node_t node;
    if (lbm_car(p) != ENC_SYM_NIL && !scene_decode_node(lbm_car(p), &node)) {
      return ENC_SYM_TERROR;
    }
    if (lbm_car(p) != ENC_SYM_NIL && node.kind == SCENE_TTF &&
        !tinygfx_prepared_text(&node.prepared, node.text, NULL, NULL)) {
      return ENC_SYM_TERROR;
    }
    p = lbm_cdr(p);
  }
  uint8_t *scratch = (uint8_t *)(scene + 1) + scene->slots * sizeof(tinygfx_rect_t);
  image_buffer_t tile = {rgb565, scene->tile_width, scene->tile_height, scratch, scratch};
  display_scene_render_context_t context = {scene, lbm_cdr(args[0])};
  if (!tinygfx_scene_render(&scene->scheduler, &tile, scene_render_tile, scene_flush_tile,
                            &context)) {
    lbm_set_error_reason("Scene render failed; dirty regions retained for retry.");
    return ENC_SYM_EERROR;
  }
  return ENC_SYM_TRUE;
}

void lbm_display_extensions_init(void) {
  register_symbols();

  disp_render_image = NULL;
  disp_clear = NULL;
  disp_reset = NULL;

  lbm_add_extension("scene-create", ext_scene_create);
  lbm_add_extension("scene-set", ext_scene_set);
  lbm_add_extension("scene-get", ext_scene_get);
  lbm_add_extension("scene-invalidate", ext_scene_invalidate);
  lbm_add_extension("scene-background", ext_scene_background);
  lbm_add_extension("scene-render", ext_scene_render);

  lbm_add_extension("img-buffer", ext_image_buffer);
  lbm_add_extension("img-buffer?", ext_is_image_buffer);
  lbm_add_extension("img-color", ext_color);
  lbm_add_extension("img-color-set", ext_color_set);
  lbm_add_extension("img-color-get", ext_color_get);
  lbm_add_extension("img-color-setpre", ext_color_setpre);
  lbm_add_extension("img-color-getpre", ext_color_getpre);
  lbm_add_extension("img-dims", ext_image_dims);
  lbm_add_extension("img-setpix", ext_putpixel);
  lbm_add_extension("img-getpix", ext_getpixel);
  lbm_add_extension("img-line", ext_line);
  lbm_add_extension("img-text", ext_text);
  lbm_add_extension("img-clear", ext_clear);
  lbm_add_extension("img-circle", ext_circle);
  lbm_add_extension("img-arc", ext_arc);
  lbm_add_extension("img-circle-sector", ext_circle_sector);
  lbm_add_extension("img-circle-segment", ext_circle_segment);
  lbm_add_extension("img-rectangle", ext_rectangle);
  lbm_add_extension("img-triangle", ext_triangle);
  lbm_add_extension("img-blit", ext_blit);

  lbm_add_extension("disp-reset", ext_disp_reset);
  lbm_add_extension("disp-clear", ext_disp_clear);
  lbm_add_extension("disp-render", ext_disp_render);
  lbm_add_extension("disp-render-jpg", ext_disp_render_jpg);
  lbm_add_extension("img-render-jpg", ext_img_render_jpg);
}

void lbm_display_extensions_set_callbacks(
                                          bool(* volatile render_image)(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors),
                                          void(* volatile clear)(uint32_t color),
                                          void(* volatile reset)(void)
                                          ) {
  disp_render_image = render_image ? render_image : display_dummy_render_image;
  disp_clear = clear ? clear : display_dummy_clear;
  disp_reset = reset ? reset : display_dummy_reset;
}
