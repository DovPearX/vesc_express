/*
	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "hw.h"

#ifdef HW_USE_NAV

#include "lispif_nav_extensions.h"
#include "extensions.h"
#include "extensions/display_extensions.h"
#include "lowzip.h"
#include "tinygfx_fonts.h"

#include "esp_heap_caps.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/endian.h>

#define R              6378137.0
#define MAX_LATITUDE   85.0511287798066
#define WORLD_SIZE     (2.0 * M_PI * R)
#define CELL_SIZE      16384
#define QUANTUM        4
#define MAX_CELL_COORD (21000000 / CELL_SIZE + 1)
#define MAX_MAP_SIZE   512000000
#define MAX_CELL_SIZE  (8 * 1024 * 1024)
#define MAX_POINTS     128
#define MAX_IMAGE_DIM  2048
#define CACHE_CELLS    4
#define MAX_LABELS     48
#define MAX_NAME       28

#ifndef HW_NAV_CACHE_SIZE
#define HW_NAV_CACHE_SIZE (8 * 1024 * 1024)
#endif

typedef struct {
	uint32_t background, outline, major_road, local_road;
} palette_t;

static const palette_t day_palette = {0xeceee9, 0xb4b7b8, 0xffcf70, 0xffffff};
static const palette_t night_palette = {0x101a27, 0x253447, 0xc69451, 0x53677e};

typedef struct {
	int32_t x, y;
} point_t;

typedef struct {
	int32_t x;
	int32_t y;
	uint32_t offset;
	uint32_t compressed_size;
	uint32_t size;
	uint32_t crc;
} tile_entry_t;

_Static_assert(sizeof(tile_entry_t) == 24, "NVA tile entry must be 24 bytes");

typedef struct {
	uint8_t *data;
	unsigned index, used;
	size_t size;
} cached_tile_t;

typedef struct {
	FILE *file;
	tile_entry_t *index;
	cached_tile_t cache[CACHE_CELLS];
	uint32_t tile_count;
	unsigned generation;
} map_t;

typedef struct {
	double cx, cy, mpp;
	int zoom;
	palette_t palette;
} viewport_t;

typedef struct {
	uint16_t width, height;
	viewport_t view;
} render_request_t;

static map_t map;
static point_t work[MAX_POINTS];
static SemaphoreHandle_t map_mutex;

static TaskHandle_t render_task;
static image_buffer_t render_image;
static render_request_t render_request;
static enum {
	RENDER_FAILED = -1,
	RENDER_IDLE,
	RENDER_BUSY,
	RENDER_READY,
} render_state;

static double mercator_x(double lon) {
	return R * lon * M_PI / 180.0;
}

static double mercator_y(double lat) {
	return R * log(tan(M_PI / 4 + lat * M_PI / 360));
}

static bool read_delta(const uint8_t **p, const uint8_t *end, int32_t *v) {
	uint32_t n = 0;

	for (unsigned shift = 0; shift <= 28; shift += 7) {
		if (*p >= end) {
			return false;
		}

		uint8_t b = *(*p)++;
		if (shift == 28 && (b & 0xf0)) {
			return false;
		}

		n |= (uint32_t)(b & 127) << shift;
		if (!(b & 128)) {
			*v = (int32_t)((n >> 1) ^ -(n & 1));
			return true;
		}
	}

	return false;
}

static void close_map(map_t *value) {
	if (value->file) {
		fclose(value->file);
	}

	for (unsigned i = 0; i < CACHE_CELLS; i++) {
		free(value->cache[i].data);
	}

	free(value->index);
	*value = (map_t){0};
}

static bool read_directory(map_t *next, long size) {
	uint8_t header[12];
	if (size < (long)sizeof(header) || size > MAX_MAP_SIZE ||
			fread(header, 1, sizeof(header), next->file) != sizeof(header) ||
			memcmp(header, "NVA", 4) != 0) {
		return false;
	}

	next->tile_count = le32dec(header + 4);
	if (!next->tile_count || next->tile_count > 200000) {
		return false;
	}

	size_t bytes = next->tile_count * sizeof(tile_entry_t);
	uint64_t offset = sizeof(header) + bytes;
	next->index = heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
	if (offset > size || !next->index ||
			fread(next->index, 1, bytes, next->file) != bytes ||
			esp_rom_crc32_le(0, (uint8_t *)next->index, bytes) != le32dec(header + 8)) {
		return false;
	}

	for (unsigned i = 0; i < next->tile_count; i++) {
		const tile_entry_t *entry = &next->index[i];
		if (entry->size < 12 || entry->size > MAX_CELL_SIZE ||
				!entry->compressed_size || entry->compressed_size > MAX_CELL_SIZE ||
				entry->offset != offset ||
				llabs(entry->x) > MAX_CELL_COORD || llabs(entry->y) > MAX_CELL_COORD) {
			return false;
		}

		if (i) {
			const tile_entry_t *prev = entry - 1;
			if (prev->x > entry->x || (prev->x == entry->x && prev->y >= entry->y)) {
				return false;
			}
		}

		offset += entry->compressed_size;
	}

	return offset == size;
}

static bool open_map(const char *path) {
	map_t next = {.file = fopen(path, "rb")};
	if (!next.file) {
		return false;
	}

	long size = -1;
	if (fseek(next.file, 0, SEEK_END) == 0) {
		size = ftell(next.file);
		rewind(next.file);
	}

	if (!read_directory(&next, size)) {
		close_map(&next);
		return false;
	}

	close_map(&map);
	map = next;
	return true;
}

typedef struct {
	uint8_t *bytes;
	uint32_t size;
} compressed_tile_t;

static lowzip_state inflater;

static unsigned read_compressed(void *user, unsigned offset) {
	compressed_tile_t *input = user;
	return offset < input->size ? input->bytes[offset] : 0x100;
}

static void trim_cache(size_t needed) {
	size_t cached = 0;
	for (unsigned i = 0; i < CACHE_CELLS; i++) {
		cached += map.cache[i].size;
	}

	while (cached + needed > HW_NAV_CACHE_SIZE) {
		cached_tile_t *oldest = NULL;
		for (unsigned i = 0; i < CACHE_CELLS; i++) {
			cached_tile_t *candidate = map.cache + i;
			if (candidate->data && (!oldest || candidate->used < oldest->used)) {
				oldest = candidate;
			}
		}

		if (!oldest) {
			break;
		}

		cached -= oldest->size;
		free(oldest->data);
		memset(oldest, 0, sizeof(*oldest));
	}
}

static uint8_t *inflate_cell(const tile_entry_t *entry) {
	compressed_tile_t input = {.size = entry->compressed_size};
	input.bytes = heap_caps_malloc(input.size, MALLOC_CAP_8BIT);
	uint8_t *data = heap_caps_malloc(entry->size, MALLOC_CAP_8BIT);

	bool read = input.bytes && data && fseek(map.file, entry->offset, SEEK_SET) == 0 &&
		fread(input.bytes, 1, input.size, map.file) == input.size;
	if (!read) {
		free(input.bytes);
		free(data);
		return NULL;
	}

	memset(&inflater, 0, sizeof(inflater));
	inflater.udata = &input;
	inflater.read_callback = read_compressed;
	inflater.zip_length = input.size;
	inflater.output_start = inflater.output_next = data;
	inflater.output_end = data + entry->size;
	lowzip_inflate_raw(&inflater);
	free(input.bytes);

	if (inflater.have_error || inflater.output_next != inflater.output_end ||
			esp_rom_crc32_le(0, data, entry->size) != entry->crc) {
		free(data);
		return NULL;
	}

	return data;
}

static const uint8_t *get_cell(unsigned index) {
	cached_tile_t *slot = &map.cache[0];

	for (unsigned i = 0; i < CACHE_CELLS; i++) {
		cached_tile_t *cell = &map.cache[i];
		if (cell->data && cell->index == index) {
			cell->used = ++map.generation;
			return cell->data;
		}

		if (!cell->data || cell->used < slot->used) {
			slot = cell;
		}
	}

	const tile_entry_t *entry = &map.index[index];
	if (entry->size > HW_NAV_CACHE_SIZE) {
		return NULL;
	}

	free(slot->data);
	*slot = (cached_tile_t){0};
	trim_cache((size_t)entry->size + entry->compressed_size);
	uint8_t *data = inflate_cell(entry);
	if (!data) {
		return NULL;
	}

	*slot = (cached_tile_t){data, index, ++map.generation, entry->size};
	return data;
}

static bool clip(float *x0, float *y0, float *x1, float *y1, int w, int h) {
	float dx = *x1 - *x0;
	float dy = *y1 - *y0;
	float first = 0;
	float last = 1;
	const float p[] = {-dx, dx, -dy, dy};
	const float q[] = {*x0, w - 1 - *x0, *y0, h - 1 - *y0};

	for (int i = 0; i < 4; i++) {
		if (p[i] == 0) {
			if (q[i] < 0) {
				return false;
			}
			continue;
		}

		float t = q[i] / p[i];
		if (p[i] < 0) {
			first = fmaxf(first, t);
		} else {
			last = fminf(last, t);
		}

		if (first > last) {
			return false;
		}
	}

	*x1 = *x0 + last * dx;
	*y1 = *y0 + last * dy;
	*x0 += first * dx;
	*y0 += first * dy;

	return true;
}

static void line(image_buffer_t *im, float x0, float y0, float x1, float y1,
	int thickness, uint32_t c) {
	x0 += thickness;
	y0 += thickness;
	x1 += thickness;
	y1 += thickness;

	if (!clip(&x0, &y0, &x1, &y1, im->width + 2 * thickness, im->height + 2 * thickness)) {
		return;
	}

	x0 -= thickness;
	y0 -= thickness;
	x1 -= thickness;
	y1 -= thickness;

	if (thickness <= 1) {
		tinygfx_line(im, lroundf(x0), lroundf(y0), lroundf(x1), lroundf(y1),
			thickness, 0, 0, c);
		return;
	}

	float dx = x1 - x0;
	float dy = y1 - y0;
	float len = sqrtf(dx * dx + dy * dy);
	if (len < 0.5f) {
		return;
	}

	float nx = -dy * thickness / len;
	float ny = dx * thickness / len;
	point_t p[] = {
		{lroundf(x0 + nx), lroundf(y0 + ny)},
		{lroundf(x1 + nx), lroundf(y1 + ny)},
		{lroundf(x1 - nx), lroundf(y1 - ny)},
		{lroundf(x0 - nx), lroundf(y0 - ny)},
	};

	tinygfx_fill_triangle(im, p[0].x, p[0].y, p[1].x, p[1].y, p[2].x, p[2].y, c);
	tinygfx_fill_triangle(im, p[0].x, p[0].y, p[2].x, p[2].y, p[3].x, p[3].y, c);
}

static void render_yield(TickType_t *last, unsigned index) {
	if (!(index & 255) && xTaskGetTickCount() - *last >= pdMS_TO_TICKS(50)) {
		vTaskDelay(1);
		*last = xTaskGetTickCount();
	}
}

static void draw_road(image_buffer_t *image, const viewport_t *view,
	unsigned count, bool major, bool fill) {
	int width = major ? 4 : 3;
	uint32_t color = view->palette.outline;
	if (fill) {
		width--;
		color = major ? view->palette.major_road : view->palette.local_road;
	}

	const double half_w = image->width / 2.0;
	const double half_h = image->height / 2.0;
	point_t prev = {
		lround((work[0].x - view->cx) / view->mpp + half_w),
		lround((view->cy - work[0].y) / view->mpp + half_h),
	};

	for (unsigned i = 1; i < count; i++) {
		point_t p = {
			lround((work[i].x - view->cx) / view->mpp + half_w),
			lround((view->cy - work[i].y) / view->mpp + half_h),
		};

		line(image, prev.x, prev.y, p.x, p.y, width, color);
		prev = p;
	}
}

typedef struct {
	double left, bottom, right, top;
} region_t;

static int read_road(const uint8_t **cursor, const uint8_t *end,
	point_t origin, const region_t *region, bool *visible) {
	int32_t count;
	if (!read_delta(cursor, end, &count) || count < 2 || count > MAX_POINTS) {
		return 0;
	}

	int64_t x = origin.x;
	int64_t y = origin.y;
	point_t low = {INT32_MAX, INT32_MAX};
	point_t high = {INT32_MIN, INT32_MIN};

	for (int i = 0; i < count; i++) {
		int32_t dx, dy;
		if (!read_delta(cursor, end, &dx) || !read_delta(cursor, end, &dy)) {
			return 0;
		}

		x += dx;
		y += dy;
		if (llabs(x) > 5250000 || llabs(y) > 5250000) {
			return 0;
		}

		point_t p = {(int32_t)(x * QUANTUM), (int32_t)(y * QUANTUM)};
		work[i] = p;
		low.x = p.x < low.x ? p.x : low.x;
		low.y = p.y < low.y ? p.y : low.y;
		high.x = p.x > high.x ? p.x : high.x;
		high.y = p.y > high.y ? p.y : high.y;
	}

	*visible = high.x >= region->left && low.x <= region->right &&
		high.y >= region->bottom && low.y <= region->top;
	return count;
}

static bool draw_roads(image_buffer_t *image, const viewport_t *view,
	const region_t *region, const tile_entry_t *entry, const uint8_t *data) {
	uint32_t count = le32dec(data);
	uint32_t size = le32dec(data + 4);
	if (size > entry->size - 12 || count > size / 6) {
		return false;
	}

	const uint8_t *start = data + 12;
	const uint8_t *end = start + size;
	point_t origin = {entry->x * (CELL_SIZE / QUANTUM), entry->y * (CELL_SIZE / QUANTUM)};
	TickType_t last = xTaskGetTickCount();

	for (unsigned pass = 0; pass < 2; pass++) {
		const uint8_t *p = start;
		for (unsigned i = 0; i < count; i++) {
			render_yield(&last, i);
			if (p == end || *p > 1) {
				return false;
			}

			bool major = *p++ != 0;
			bool visible;
			int points = read_road(&p, end, origin, region, &visible);
			if (!points) {
				return false;
			}

			if (visible) {
				draw_road(image, view, points, major, pass != 0);
			}
		}

		if (p != end) {
			return false;
		}
	}

	return true;
}

typedef struct {
	int16_t x, y, width;
	uint8_t length, name[MAX_NAME];
} label_t;

typedef struct {
	const uint8_t *font;
	unsigned count;
	label_t labels[MAX_LABELS];
} labels_t;

static void place_label(labels_t *labels, const image_buffer_t *image,
	int x, int y, uint8_t length, const uint8_t *name) {
	if (!labels->font || labels->count == MAX_LABELS) {
		return;
	}

	int width = length * labels->font[0];
	int height = labels->font[1];
	x -= width / 2;
	y -= height / 2;
	if (x < 1 || y < 1 || x + width >= image->width || y + height >= image->height) {
		return;
	}

	for (unsigned i = 0; i < labels->count; i++) {
		const label_t *other = &labels->labels[i];
		if (x < other->x + other->width + 2 && x + width + 2 > other->x &&
				y < other->y + height + 2 && y + height + 2 > other->y) {
			return;
		}
	}

	label_t *label = &labels->labels[labels->count++];
	*label = (label_t){.x = x, .y = y, .width = width, .length = length};
	memcpy(label->name, name, length);
}

static bool read_labels(labels_t *labels, const image_buffer_t *image,
	const viewport_t *view, const tile_entry_t *entry, const uint8_t *data) {
	uint32_t count = le32dec(data + 8);
	const uint8_t *p = data + 12 + le32dec(data + 4);
	const uint8_t *end = data + entry->size;
	if (count > 256) {
		return false;
	}

	for (unsigned i = 0; i < count; i++) {
		if (end - p < 6) {
			return false;
		}

		unsigned dx = le16dec(p), dy = le16dec(p + 2);
		uint8_t zoom = p[4], length = p[5];
		p += 6;
		if (!length || length > MAX_NAME || end - p < length ||
				dx >= CELL_SIZE || dy >= CELL_SIZE || zoom < 14 || zoom > 18) {
			return false;
		}

		if (zoom <= view->zoom) {
			int32_t x = entry->x * CELL_SIZE + (int32_t)dx;
			int32_t y = entry->y * CELL_SIZE + (int32_t)dy;
			int px = lround((x - view->cx) / view->mpp + image->width / 2.0);
			int py = lround((view->cy - y) / view->mpp + image->height / 2.0);
			place_label(labels, image, px, py, length, p);
		}
		p += length;
	}

	return p == end;
}

static void draw_labels(image_buffer_t *image, const viewport_t *view, const labels_t *labels) {
	uint32_t bg = view->palette.background;
	unsigned lightness = ((bg >> 16) & 255) * 3 + ((bg >> 8) & 255) * 6 + (bg & 255);
	uint32_t colors[] = {lightness >= 1280 ? 0x182432 : 0xf0f6fa, bg};

	for (unsigned i = 0; i < labels->count; i++) {
		const label_t *label = &labels->labels[i];
		for (unsigned j = 0; j < label->length; j++) {
			int x = label->x + j * labels->font[0];
			tinygfx_img_putc(image, x, label->y, colors, 2,
				labels->font, label->name[j], 0, 1.0f);
		}
	}
}

static bool render(image_buffer_t *image, const viewport_t *view) {
	image_buffer_clear(image, view->palette.background);

	double half_w = (image->width / 2.0 + 8) * view->mpp;
	double half_h = (image->height / 2.0 + 8) * view->mpp;
	region_t region = {view->cx - half_w, view->cy - half_h,
		view->cx + half_w, view->cy + half_h};
	int left = floor(region.left / CELL_SIZE);
	int right = floor(region.right / CELL_SIZE);
	int bottom = floor(region.bottom / CELL_SIZE);
	int top = floor(region.top / CELL_SIZE);
	labels_t labels = {.font = tinygfx_get_builtin_font(TINYGFX_DEFAULT_FONT_ID)};

	for (unsigned i = 0; i < map.tile_count; i++) {
		const tile_entry_t *entry = &map.index[i];
		if (entry->x < left || entry->x > right || entry->y < bottom || entry->y > top) {
			continue;
		}

		const uint8_t *data = get_cell(i);
		if (!data || !draw_roads(image, view, &region, entry, data) ||
				!read_labels(&labels, image, view, entry, data)) {
			return false;
		}
	}

	draw_labels(image, view, &labels);
	return true;
}

static void render_worker(void *arg) {
	(void)arg;

	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		xSemaphoreTake(map_mutex, portMAX_DELAY);
		if (render_state != RENDER_BUSY) {
			xSemaphoreGive(map_mutex);
			continue;
		}

		size_t size = (size_t)render_request.width * render_request.height * 2;
		uint8_t *data = heap_caps_realloc(render_image.data, size, MALLOC_CAP_8BIT);
		bool ok = data != NULL;
		if (ok) {
			render_image = (image_buffer_t){
				.fmt = rgb565,
				.width = render_request.width,
				.height = render_request.height,
				.data = data,
			};
			ok = render(&render_image, &render_request.view);
		}

		render_state = ok ? RENDER_READY : RENDER_FAILED;
		xSemaphoreGive(map_mutex);
	}
}

static bool lock_map(void) {
	if (!map_mutex || xSemaphoreTake(map_mutex, 0) != pdTRUE) {
		return false;
	}

	if (render_state == RENDER_BUSY) {
		xSemaphoreGive(map_mutex);
		return false;
	}

	return true;
}

static void clear_render(void) {
	free(render_image.data);
	memset(&render_image, 0, sizeof(render_image));
	render_state = RENDER_IDLE;
}

static bool decode_image(lbm_array_header_t *array, image_buffer_t *image) {
	if (!array || array->size < IMAGE_BUFFER_HEADER_SIZE) {
		return false;
	}

	uint8_t *bytes = (uint8_t *)array->data;
	uint16_t width = image_buffer_width(bytes);
	uint16_t height = image_buffer_height(bytes);
	if (image_buffer_format(bytes) != rgb565 || !width || !height ||
			width > MAX_IMAGE_DIM || height > MAX_IMAGE_DIM ||
			!image_buffer_is_valid(bytes, array->size)) {
		return false;
	}

	*image = (image_buffer_t){
		.fmt = rgb565,
		.width = width,
		.height = height,
		.data = image_buffer_data(bytes),
	};
	return true;
}

static lbm_value ext_open(lbm_value *args, lbm_uint count) {
	if (count != 1 || !lbm_dec_str(args[0])) {
		return ENC_SYM_TERROR;
	}

	if (!lock_map()) {
		return ENC_SYM_NIL;
	}

	bool ok = open_map(lbm_dec_str(args[0]));
	if (ok) {
		render_state = RENDER_IDLE;
	}

	xSemaphoreGive(map_mutex);
	return ok ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_close(lbm_value *args, lbm_uint count) {
	(void)args;

	if (count) {
		return ENC_SYM_TERROR;
	}
	if (!lock_map()) {
		return ENC_SYM_NIL;
	}

	clear_render();
	close_map(&map);
	xSemaphoreGive(map_mutex);
	return ENC_SYM_TRUE;
}

static bool decode_viewport(const lbm_value *args, viewport_t *view) {
	for (unsigned i = 0; i < 3; i++) {
		if (!lbm_is_number(args[i])) {
			return false;
		}
	}

	double lat = lbm_dec_as_double(args[0]);
	double lon = lbm_dec_as_double(args[1]);
	double zoom = lbm_dec_as_double(args[2]);
	if (!isfinite(lat) || fabs(lat) > MAX_LATITUDE || !isfinite(lon) || fabs(lon) > 180 ||
			!isfinite(zoom) || zoom < 14 || zoom > 18 || zoom != floor(zoom)) {
		return false;
	}

	view->cx = mercator_x(lon);
	view->cy = mercator_y(lat);
	view->zoom = (int)zoom;
	view->mpp = WORLD_SIZE / (256.0 * (1U << view->zoom));
	return true;
}

static bool decode_palette(lbm_value list, palette_t *palette) {
	if (lbm_is_symbol(list)) {
		const char *name = lbm_get_name_by_symbol(lbm_dec_sym(list));
		if (name && strcmp(name, "day") == 0) {
			*palette = day_palette;
			return true;
		}

		if (name && strcmp(name, "night") == 0) {
			*palette = night_palette;
			return true;
		}

		return false;
	}

	uint32_t colors[4];
	for (unsigned i = 0; i < sizeof(colors) / sizeof(colors[0]); i++) {
		if (!lbm_is_cons(list) || !lbm_is_number(lbm_car(list))) {
			return false;
		}

		double color = lbm_dec_as_double(lbm_car(list));
		if (!isfinite(color) || color < 0 || color > 0xffffff || color != floor(color)) {
			return false;
		}

		colors[i] = (uint32_t)color;
		list = lbm_cdr(list);
	}

	if (list != ENC_SYM_NIL) {
		return false;
	}

	*palette = (palette_t){
		.background = colors[0],
		.outline = colors[1],
		.major_road = colors[2],
		.local_road = colors[3],
	};

	return true;
}

static lbm_value ext_render_async(lbm_value *args, lbm_uint count) {
	image_buffer_t image;
	render_request_t request;
	if ((count != 4 && count != 5) || !decode_image(lbm_dec_array_r(args[0]), &image)) {
		return ENC_SYM_TERROR;
	}

	request.width = image.width;
	request.height = image.height;
	request.view.palette = day_palette;

	if (count == 5 && !decode_palette(args[4], &request.view.palette)) {
		lbm_set_error_reason("Expected day, night or a list of four RGB888 colors");
		return ENC_SYM_TERROR;
	}

	if (!decode_viewport(args + 1, &request.view)) {
		lbm_set_error_reason("Invalid map center or zoom");
		return ENC_SYM_EERROR;
	}

	if (!lock_map()) {
		return ENC_SYM_NIL;
	}

	if (!map.file || (!render_task && xTaskCreatePinnedToCore(
			render_worker,
			"nav_render",
			8192,
			NULL,
			3,
			&render_task,
			(xPortGetCoreID() + 1) % portNUM_PROCESSORS) != pdPASS)) {
		xSemaphoreGive(map_mutex);
		return ENC_SYM_NIL;
	}

	render_request = request;
	render_state = RENDER_BUSY;
	xTaskNotifyGive(render_task);
	xSemaphoreGive(map_mutex);
	return ENC_SYM_TRUE;
}

static lbm_value ext_render_ready(lbm_value *args, lbm_uint count) {
	image_buffer_t buffer;
	if (count != 1 || !decode_image(lbm_dec_array_rw(args[0]), &buffer)) {
		return ENC_SYM_TERROR;
	}

	if (!map_mutex || xSemaphoreTake(map_mutex, 0) != pdTRUE) {
		return lbm_enc_i(0);
	}

	int result = 0;
	if (render_state == RENDER_FAILED) {
		result = -1;
		render_state = RENDER_IDLE;
	} else if (render_state == RENDER_READY) {
		if (buffer.width != render_image.width || buffer.height != render_image.height) {
			xSemaphoreGive(map_mutex);
			lbm_set_error_reason("Image dimensions differ from the queued render");
			return ENC_SYM_EERROR;
		}

		memcpy(buffer.data, render_image.data, (size_t)buffer.width * buffer.height * 2);
		render_state = RENDER_IDLE;
		result = 1;
	}

	xSemaphoreGive(map_mutex);
	return lbm_enc_i(result);
}

void lispif_nav_load_extensions(void) {
	if (!map_mutex) {
		map_mutex = xSemaphoreCreateMutex();
	}

	if (map_mutex) {
		xSemaphoreTake(map_mutex, portMAX_DELAY);
		clear_render();
		close_map(&map);
		xSemaphoreGive(map_mutex);
	}

	lbm_add_extension("nav-open", ext_open);
	lbm_add_extension("nav-close", ext_close);
	lbm_add_extension("nav-render-async", ext_render_async);
	lbm_add_extension("nav-render-ready", ext_render_ready);
}

#endif /* HW_USE_NAV */
