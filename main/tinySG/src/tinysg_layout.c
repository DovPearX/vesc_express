#include "tinysg.h"

typedef struct {
  size_t first, end;
  uint32_t basis, cross, grow;
} flex_line;

static bool inner_area(tinysg_area area, uint16_t padding, tinysg_area *inner) {
  if ((uint32_t)padding * 2 >= area.width || (uint32_t)padding * 2 >= area.height) return false;
  if ((int64_t)area.x + area.width > INT32_MAX || (int64_t)area.y + area.height > INT32_MAX) return false;
  *inner = (tinysg_area){area.x + padding, area.y + padding, area.width - 2 * padding, area.height - 2 * padding};
  return true;
}

bool tinysg_grid_cell(tinysg_area area, uint16_t columns, uint16_t rows, uint16_t gap, uint16_t padding, size_t index,
                      tinysg_area *cell) {
  if (!cell || !columns || !rows || (uint64_t)index >= (uint64_t)columns * rows) return false;
  tinysg_area inner;
  if (!inner_area(area, padding, &inner)) return false;
  uint32_t gaps_x = (uint32_t)(columns - 1) * gap, gaps_y = (uint32_t)(rows - 1) * gap;
  if (gaps_x >= inner.width || gaps_y >= inner.height) return false;
  uint32_t width = inner.width - gaps_x, height = inner.height - gaps_y;
  if (width < columns || height < rows) return false;
  uint32_t column = (uint32_t)(index % columns), row = (uint32_t)(index / columns);
  uint32_t left = width * column / columns, top = height * row / rows;
  uint32_t right = width * (column + 1) / columns, bottom = height * (row + 1) / rows;
  *cell = (tinysg_area){inner.x + (int32_t)(left + column * gap), inner.y + (int32_t)(top + row * gap),
                        (uint16_t)(right - left), (uint16_t)(bottom - top)};
  return true;
}

static uint16_t main_size(const tinysg_flex_item *item, bool column) {
  return column ? item->height : item->width;
}

static uint16_t cross_size(const tinysg_flex_item *item, bool column) {
  return column ? item->width : item->height;
}

static flex_line find_line(const tinysg_flex_layout *layout, const tinysg_flex_item *items, size_t count, size_t first,
                           uint32_t available, bool column) {
  flex_line line = {.first = first, .end = first};
  bool wrap = layout->flow >= TINYSG_FLEX_ROW_WRAP;
  uint64_t used = 0;
  for (size_t i = first; i < count; i++) {
    uint32_t size = main_size(&items[i], column), cross = cross_size(&items[i], column);
    uint32_t gap = i == first ? 0 : layout->gap;
    if (wrap && i > first && used + gap + size > available) break;
    used += gap + size;
    line.basis += size;
    line.grow += items[i].grow;
    if (cross > line.cross) line.cross = cross;
    line.end = i + 1;
  }
  return line;
}

static uint32_t alignment_offset(tinysg_align align, uint32_t remaining) {
  if (align == TINYSG_ALIGN_CENTER) return remaining / 2;
  return align == TINYSG_ALIGN_END ? remaining : 0;
}

static void place_line(const tinysg_flex_layout *layout, tinysg_flex_item *items, tinysg_area inner, flex_line line,
                       uint32_t cross_offset, bool column) {
  uint32_t available = column ? inner.height : inner.width;
  uint32_t count = (uint32_t)(line.end - line.first);
  uint64_t gaps = (uint64_t)(count - 1) * layout->gap;
  uint32_t gap = gaps >= available && count > 1 ? available / (count - 1) : layout->gap;
  uint32_t room = available - (count - 1) * gap;
  uint32_t spare = room > line.basis ? room - line.basis : 0;
  uint32_t unused = line.grow ? 0 : spare;
  uint32_t cursor = alignment_offset(layout->justify, unused);
  uint64_t basis_before = 0, grow_before = 0;
  for (size_t i = line.first; i < line.end; i++) {
    uint32_t basis = main_size(&items[i], column);
    uint32_t size = basis;
    if (line.basis > room)
      size = (uint32_t)(((basis_before + basis) * room / line.basis) - basis_before * room / line.basis);
    if (line.basis <= room && line.grow) {
      size += (uint32_t)(((grow_before + items[i].grow) * spare / line.grow) - grow_before * spare / line.grow);
    }
    uint32_t cross = cross_size(&items[i], column);
    if (cross > line.cross || layout->align == TINYSG_ALIGN_STRETCH) cross = line.cross;
    uint32_t offset = alignment_offset(layout->align, line.cross - cross);
    uint32_t between = layout->justify == TINYSG_ALIGN_SPACE_BETWEEN && count > 1
                           ? (uint32_t)((uint64_t)(i - line.first) * unused / (count - 1))
                           : 0;
    int32_t main = (column ? inner.y : inner.x) + (int32_t)(cursor + between);
    int32_t side = (column ? inner.x : inner.y) + (int32_t)(cross_offset + offset);
    items[i].area = column ? (tinysg_area){side, main, (uint16_t)cross, (uint16_t)size}
                           : (tinysg_area){main, side, (uint16_t)size, (uint16_t)cross};
    cursor += size + gap;
    basis_before += basis;
    grow_before += items[i].grow;
  }
}

bool tinysg_layout_flex(const tinysg_flex_layout *layout, tinysg_flex_item *items, size_t count) {
  if (!layout || (!items && count) || count > UINT16_MAX || (unsigned)layout->flow > TINYSG_FLEX_COLUMN_WRAP ||
      (unsigned)layout->justify > TINYSG_ALIGN_SPACE_BETWEEN || (unsigned)layout->align > TINYSG_ALIGN_STRETCH)
    return false;
  tinysg_area inner;
  if (!inner_area(layout->area, layout->padding, &inner)) return false;
  bool column = layout->flow == TINYSG_FLEX_COLUMN || layout->flow == TINYSG_FLEX_COLUMN_WRAP;
  uint32_t available = column ? inner.height : inner.width;
  uint32_t cross_available = column ? inner.width : inner.height, cross_offset = 0;
  size_t first = 0;
  while (first < count) {
    flex_line line = find_line(layout, items, count, first, available, column);
    if (layout->flow < TINYSG_FLEX_ROW_WRAP) line.cross = cross_available;
    if (line.cross > cross_available - cross_offset) line.cross = cross_available - cross_offset;
    place_line(layout, items, inner, line, cross_offset, column);
    uint32_t step = line.cross + layout->gap;
    cross_offset = step > cross_available - cross_offset ? cross_available : cross_offset + step;
    first = line.end;
  }
  return true;
}
