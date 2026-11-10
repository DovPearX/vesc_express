#include "tinysg.h"
#include <string.h>

typedef struct {
  uint8_t *bits;
  uint32_t columns, rows, tile;
  uint16_t width, height;
} tinysg_dirty_map;

static bool marked(const tinysg_dirty_map *map, uint32_t x, uint32_t y) {
  uint32_t bit = y * map->columns + x;
  return (map->bits[bit / 8] & (1u << (bit % 8))) != 0;
}

static void mark_area(tinysg_dirty_map *map, tinysg_area area) {
  int32_t left = area.x < 0 ? 0 : area.x, top = area.y < 0 ? 0 : area.y;
  int32_t right = area.x + area.width, bottom = area.y + area.height;
  if (right > map->width) right = map->width;
  if (bottom > map->height) bottom = map->height;
  if (left >= right || top >= bottom) return;
  for (uint32_t y = (uint32_t)top / map->tile; y <= (uint32_t)(bottom - 1) / map->tile; y++) {
    for (uint32_t x = (uint32_t)left / map->tile; x <= (uint32_t)(right - 1) / map->tile; x++) {
      uint32_t bit = y * map->columns + x;
      map->bits[bit / 8] |= (uint8_t)(1u << (bit % 8));
    }
  }
}

static bool marked_row(const tinysg_dirty_map *map, uint32_t left, uint32_t right, uint32_t y) {
  for (uint32_t x = left; x < right; x++) {
    if (!marked(map, x, y)) return false;
  }
  return true;
}

static tinysg_area take_area(tinysg_dirty_map *map, uint32_t left, uint32_t top) {
  uint32_t right = left + 1, bottom = top + 1;
  while (right < map->columns && marked(map, right, top)) right++;
  while (bottom < map->rows && marked_row(map, left, right, bottom)) bottom++;
  for (uint32_t y = top; y < bottom; y++) {
    for (uint32_t x = left; x < right; x++) {
      uint32_t bit = y * map->columns + x;
      map->bits[bit / 8] &= (uint8_t)~(1u << (bit % 8));
    }
  }
  right *= map->tile;
  bottom *= map->tile;
  if (right > map->width) right = map->width;
  if (bottom > map->height) bottom = map->height;
  return (tinysg_area){(int32_t)(left * map->tile), (int32_t)(top * map->tile), (uint16_t)(right - left * map->tile),
                       (uint16_t)(bottom - top * map->tile)};
}

bool tinysg_render_update(const tinysg_scene *scene, const tinysg_scene *previous, uint16_t width, uint16_t height,
                          uint8_t *dirty, size_t dirty_bytes, uint8_t *scratch, size_t bytes, tinysg_sink sink,
                          void *ctx) {
  if (!scene || (!scene->nodes && scene->count) || !width || !height || !dirty || !dirty_bytes || !scratch ||
      bytes < 2 || !sink || (previous && !previous->nodes && previous->count))
    return false;
  if (!previous || previous->background != scene->background) {
    return tinysg_render(scene, width, height, scratch, bytes, sink, ctx);
  }
  tinysg_dirty_map map = {.bits = dirty, .tile = 8, .width = width, .height = height};
  do {
    map.columns = ((uint32_t)width + map.tile - 1) / map.tile;
    map.rows = ((uint32_t)height + map.tile - 1) / map.tile;
    if ((map.columns * map.rows + 7) / 8 <= dirty_bytes) break;
    map.tile *= 2;
  } while (true);
  memset(dirty, 0, (map.columns * map.rows + 7) / 8);
  size_t total = previous->count > scene->count ? previous->count : scene->count;
  for (size_t i = 0; i < total; i++) {
    if (i < previous->count && i < scene->count && tinysg_node_equal(&previous->nodes[i], &scene->nodes[i])) continue;
    if (i < previous->count) mark_area(&map, tinysg_node_bounds(&previous->nodes[i]));
    if (i < scene->count) mark_area(&map, tinysg_node_bounds(&scene->nodes[i]));
  }
  for (uint32_t y = 0; y < map.rows; y++) {
    for (uint32_t x = 0; x < map.columns; x++) {
      if (!marked(&map, x, y)) continue;
      tinysg_area area = take_area(&map, x, y);
      if (!tinysg_render_area(scene, width, height, area, scratch, bytes, sink, ctx)) return false;
    }
  }
  return true;
}

static bool touches(tinysg_area a, tinysg_area b) {
  return a.x <= b.x + b.width && b.x <= a.x + a.width && a.y <= b.y + b.height && b.y <= a.y + a.height;
}

static tinysg_area united(tinysg_area a, tinysg_area b) {
  int32_t left = a.x < b.x ? a.x : b.x;
  int32_t top = a.y < b.y ? a.y : b.y;
  int32_t right = a.x + a.width > b.x + b.width ? a.x + a.width : b.x + b.width;
  int32_t bottom = a.y + a.height > b.y + b.height ? a.y + a.height : b.y + b.height;
  uint32_t width = (uint32_t)(right - left), height = (uint32_t)(bottom - top);
  return (tinysg_area){left, top, width > UINT16_MAX ? UINT16_MAX : (uint16_t)width,
                       height > UINT16_MAX ? UINT16_MAX : (uint16_t)height};
}

static size_t touching_index(const tinysg_area *areas, size_t count, tinysg_area area) {
  for (size_t i = 0; i < count; i++) {
    if (touches(area, areas[i])) return i;
  }
  return count;
}

static size_t add_area(tinysg_area *areas, size_t count, size_t capacity, tinysg_area area) {
  if (!area.width || !area.height) return count;
  size_t index = touching_index(areas, count, area);
  while (index < count) {
    area = united(area, areas[index]);
    areas[index] = areas[--count];
    index = touching_index(areas, count, area);
  }
  if (count < capacity) {
    areas[count++] = area;
    return count;
  }
  for (size_t i = 0; i < count; i++) area = united(area, areas[i]);
  areas[0] = area;
  return 1;
}

size_t tinysg_scene_damage(const tinysg_node *before, size_t before_count, const tinysg_node *after, size_t after_count,
                           tinysg_area *areas, size_t capacity) {
  if (!areas || !capacity || (!before && before_count) || (!after && after_count)) return SIZE_MAX;
  size_t count = 0, total = before_count > after_count ? before_count : after_count;
  for (size_t i = 0; i < total; i++) {
    if (i < before_count && i < after_count && tinysg_node_equal(&before[i], &after[i])) continue;
    if (i < before_count) count = add_area(areas, count, capacity, tinysg_node_bounds(&before[i]));
    if (i < after_count) count = add_area(areas, count, capacity, tinysg_node_bounds(&after[i]));
  }
  return count;
}
