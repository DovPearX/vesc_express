#include "tinysg_image.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
  uint16_t first[16], offset[16];
  uint8_t count[16], values[256];
  bool valid;
} jpeg_huffman;

typedef struct {
  uint8_t id, h, v, quant, dc, ac;
  int32_t predictor;
  uint8_t samples[1024];
} jpeg_component;

typedef struct {
  const uint8_t *data;
  size_t bytes, position;
  uint32_t bits;
  unsigned available, components, max_h, max_v, restart;
  uint16_t width, height, quant[4][64];
  bool quant_valid[4], failed;
  jpeg_huffman huffman[2][4];
  jpeg_component component[3];
} jpeg_decoder;

static const uint8_t zigzag[64] = {0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,
                                   12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6,  7,  14, 21, 28,
                                   35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
                                   58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};

static const int16_t cosine[8][8] = {
    {2896, 4017, 3784, 3406, 2896, 2276, 1567, 799},     {2896, 3406, 1567, -799, -2896, -4017, -3784, -2276},
    {2896, 2276, -1567, -4017, -2896, 799, 3784, 3406},  {2896, 799, -3784, -2276, 2896, 3406, -1567, -4017},
    {2896, -799, -3784, 2276, 2896, -3406, -1567, 4017}, {2896, -2276, -1567, 4017, -2896, -799, 3784, -3406},
    {2896, -3406, 1567, 799, -2896, 4017, -3784, 2276},  {2896, -4017, 3784, -3406, 2896, -2276, 1567, -799}};

static unsigned jpeg_u16(const uint8_t *data) {
  return (unsigned)data[0] << 8 | data[1];
}

static uint8_t jpeg_clamp(int64_t value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return (uint8_t)value;
}

static unsigned jpeg_bits(jpeg_decoder *decoder, unsigned count) {
  while (decoder->available < count) {
    if (decoder->position >= decoder->bytes) {
      decoder->failed = true;
      return 0;
    }
    unsigned byte = decoder->data[decoder->position++];
    if (byte == 255) {
      if (decoder->position >= decoder->bytes || decoder->data[decoder->position++] != 0) {
        decoder->failed = true;
        return 0;
      }
    }
    decoder->bits = decoder->bits << 8 | byte;
    decoder->available += 8;
  }
  decoder->available -= count;
  return (decoder->bits >> decoder->available) & ((1u << count) - 1);
}

static unsigned jpeg_symbol(jpeg_decoder *decoder, const jpeg_huffman *table) {
  unsigned code = 0;
  for (unsigned length = 0; length < 16; length++) {
    code = code * 2 + jpeg_bits(decoder, 1);
    unsigned index = code - table->first[length];
    if (index < table->count[length]) return table->values[table->offset[length] + index];
    if (decoder->failed) return 0;
  }
  decoder->failed = true;
  return 0;
}

static int32_t jpeg_signed(jpeg_decoder *decoder, unsigned count) {
  if (!count) return 0;
  unsigned value = jpeg_bits(decoder, count);
  return value < (1u << (count - 1)) ? (int32_t)value - (int32_t)((1u << count) - 1) : (int32_t)value;
}

static bool jpeg_tables(jpeg_decoder *decoder, unsigned marker, const uint8_t *data, size_t bytes) {
  while (bytes) {
    unsigned header = *data++;
    bytes--;
    unsigned id = header & 15;
    unsigned kind = header >> 4;
    if (id > 3) return false;
    if (marker == 0xdb) {
      if (kind || bytes < 64) return false;
      for (unsigned i = 0; i < 64; i++) {
        if (!data[i]) return false;
        decoder->quant[id][zigzag[i]] = data[i];
      }
      decoder->quant_valid[id] = true;
      data += 64;
      bytes -= 64;
      continue;
    }
    if (kind > 1 || bytes < 16) return false;
    jpeg_huffman *table = &decoder->huffman[kind][id];
    unsigned code = 0, total = 0;
    for (unsigned i = 0; i < 16; i++) {
      table->first[i] = (uint16_t)code;
      table->offset[i] = (uint16_t)total;
      table->count[i] = data[i];
      total += data[i];
      code += data[i];
      if (code >= (1u << (i + 1))) return false;
      code *= 2;
    }
    if (!total || total > 256 || bytes - 16 < total) return false;
    memcpy(table->values, data + 16, total);
    table->valid = true;
    data += 16 + total;
    bytes -= 16 + total;
  }
  return true;
}

static bool jpeg_frame(jpeg_decoder *decoder, const uint8_t *data, size_t bytes) {
  if (decoder->components || bytes < 6 || data[0] != 8) return false;
  if (jpeg_u16(data + 1) != decoder->height || jpeg_u16(data + 3) != decoder->width) return false;
  unsigned count = data[5];
  if ((count != 1 && count != 3) || bytes != 6 + count * 3) return false;
  unsigned blocks = 0;
  for (unsigned i = 0; i < count; i++) {
    jpeg_component *component = &decoder->component[i];
    component->id = data[6 + i * 3];
    component->h = data[7 + i * 3] >> 4;
    component->v = data[7 + i * 3] & 15;
    component->quant = data[8 + i * 3];
    if (!component->h || component->h > 4 || !component->v || component->v > 4 || component->quant > 3) return false;
    for (unsigned j = 0; j < i; j++) {
      if (component->id == decoder->component[j].id) return false;
    }
    blocks += component->h * component->v;
    if (component->h > decoder->max_h) decoder->max_h = component->h;
    if (component->v > decoder->max_v) decoder->max_v = component->v;
  }
  if (blocks > 10 || (count == 1 && blocks != 1)) return false;
  decoder->components = count;
  return true;
}

static bool jpeg_block(jpeg_decoder *decoder, jpeg_component *component, uint8_t *output, unsigned stride) {
  int32_t coefficients[64] = {0};
  const uint16_t *quant = decoder->quant[component->quant];
  unsigned category = jpeg_symbol(decoder, &decoder->huffman[0][component->dc]);
  if (category > 11 || decoder->failed) return false;
  component->predictor += jpeg_signed(decoder, category);
  if (component->predictor < -32768 || component->predictor > 32767) return false;
  coefficients[0] = component->predictor * quant[0];
  bool dc_only = true;
  for (unsigned i = 1; i < 64;) {
    unsigned symbol = jpeg_symbol(decoder, &decoder->huffman[1][component->ac]);
    if (decoder->failed) return false;
    if (!symbol) break;
    unsigned run = symbol >> 4, size = symbol & 15;
    if (!size) {
      if (run != 15 || i + 16 > 64) return false;
      i += 16;
      continue;
    }
    i += run;
    if (size > 10 || i >= 64) return false;
    unsigned index = zigzag[i++];
    coefficients[index] = jpeg_signed(decoder, size) * quant[index];
    dc_only = false;
  }
  if (decoder->failed) return false;
  if (dc_only) {
    uint8_t value = jpeg_clamp((coefficients[0] + 4) / 8 + 128);
    for (unsigned y = 0; y < 8; y++) memset(output + y * stride, value, 8);
    return true;
  }
  int64_t rows[64];
  for (unsigned y = 0; y < 8; y++) {
    for (unsigned x = 0; x < 8; x++) {
      int64_t sum = 0;
      for (unsigned u = 0; u < 8; u++) sum += (int64_t)coefficients[y * 8 + u] * cosine[x][u];
      rows[y * 8 + x] = sum;
    }
  }
  for (unsigned y = 0; y < 8; y++) {
    for (unsigned x = 0; x < 8; x++) {
      int64_t sum = 0;
      for (unsigned v = 0; v < 8; v++) sum += rows[v * 8 + x] * cosine[y][v];
      output[y * stride + x] = jpeg_clamp(((sum + (1LL << 25)) >> 26) + 128);
    }
  }
  return true;
}

static unsigned jpeg_sample(const jpeg_decoder *decoder, unsigned component, unsigned x, unsigned y) {
  const jpeg_component *plane = &decoder->component[component];
  x = x * plane->h / decoder->max_h;
  y = y * plane->v / decoder->max_v;
  return plane->samples[y * plane->h * 8 + x];
}

static bool jpeg_scan(jpeg_decoder *decoder, const uint8_t *data, size_t bytes, uint8_t *pixels) {
  unsigned count = decoder->components;
  if (!count || bytes != 1 + count * 2 + 3 || data[0] != count) return false;
  if (data[1 + count * 2] || data[2 + count * 2] != 63 || data[3 + count * 2]) return false;
  for (unsigned i = 0; i < count; i++) {
    jpeg_component *component = &decoder->component[i];
    if (data[1 + i * 2] != component->id) return false;
    component->dc = data[2 + i * 2] >> 4;
    component->ac = data[2 + i * 2] & 15;
    if (component->dc > 3 || component->ac > 3 || !decoder->quant_valid[component->quant]) return false;
    if (!decoder->huffman[0][component->dc].valid || !decoder->huffman[1][component->ac].valid) return false;
  }
  unsigned mcu_width = decoder->max_h * 8, mcu_height = decoder->max_v * 8;
  unsigned index = 0, restart_id = 0;
  for (unsigned top = 0; top < decoder->height; top += mcu_height) {
    for (unsigned left = 0; left < decoder->width; left += mcu_width, index++) {
      if (index && decoder->restart && index % decoder->restart == 0) {
        decoder->available = 0;
        if (decoder->position >= decoder->bytes || decoder->data[decoder->position++] != 255) return false;
        while (decoder->position < decoder->bytes && decoder->data[decoder->position] == 255) decoder->position++;
        if (decoder->position >= decoder->bytes || decoder->data[decoder->position++] != 0xd0 + restart_id)
          return false;
        restart_id = (restart_id + 1) & 7;
        for (unsigned i = 0; i < count; i++) decoder->component[i].predictor = 0;
      }
      for (unsigned i = 0; i < count; i++) {
        jpeg_component *component = &decoder->component[i];
        for (unsigned y = 0; y < component->v; y++) {
          for (unsigned x = 0; x < component->h; x++) {
            if (!jpeg_block(decoder, component, component->samples + y * component->h * 64 + x * 8, component->h * 8))
              return false;
          }
        }
      }
      for (unsigned y = 0; y < mcu_height && top + y < decoder->height; y++) {
        for (unsigned x = 0; x < mcu_width && left + x < decoder->width; x++) {
          int luminance = (int)jpeg_sample(decoder, 0, x, y);
          int cb = count == 3 ? (int)jpeg_sample(decoder, 1, x, y) - 128 : 0;
          int cr = count == 3 ? (int)jpeg_sample(decoder, 2, x, y) - 128 : 0;
          uint16_t color = tinysg_rgb(jpeg_clamp(luminance + ((91881 * cr + 32768) >> 16)),
                                      jpeg_clamp(luminance - ((22554 * cb + 46802 * cr + 32768) >> 16)),
                                      jpeg_clamp(luminance + ((116130 * cb + 32768) >> 16)));
          size_t offset = ((size_t)(top + y) * decoder->width + left + x) * 2;
          pixels[offset] = (uint8_t)(color >> 8);
          pixels[offset + 1] = (uint8_t)color;
        }
      }
    }
  }
  size_t position = decoder->position;
  if (position >= decoder->bytes || decoder->data[position++] != 255) return false;
  while (position < decoder->bytes && decoder->data[position] == 255) position++;
  return position < decoder->bytes && decoder->data[position] == 0xd9;
}

bool tinysg_jpeg_decode(const uint8_t *data, size_t bytes, uint8_t *pixels, uint16_t width, uint16_t height) {
  if (!data || !pixels || bytes < 4 || !width || !height || data[0] != 255 || data[1] != 0xd8) return false;
  jpeg_decoder *decoder = calloc(1, sizeof(*decoder));
  if (!decoder) return false;
  decoder->data = data;
  decoder->bytes = bytes;
  decoder->position = 2;
  decoder->width = width;
  decoder->height = height;
  bool result = false;
  while (decoder->position + 4 <= bytes) {
    if (data[decoder->position++] != 255) break;
    while (decoder->position < bytes && data[decoder->position] == 255) decoder->position++;
    if (bytes - decoder->position < 3) break;
    unsigned marker = data[decoder->position++];
    unsigned length = jpeg_u16(data + decoder->position);
    if (length < 2 || length > bytes - decoder->position) break;
    const uint8_t *segment = data + decoder->position + 2;
    decoder->position += length;
    if (marker == 0xda) {
      result = jpeg_scan(decoder, segment, length - 2, pixels);
      break;
    }
    if (marker == 0xc0) {
      if (!jpeg_frame(decoder, segment, length - 2)) break;
      continue;
    }
    if (marker == 0xdb || marker == 0xc4) {
      if (!jpeg_tables(decoder, marker, segment, length - 2)) break;
      continue;
    }
    if (marker == 0xdd) {
      if (length != 4) break;
      decoder->restart = jpeg_u16(segment);
      continue;
    }
    if ((marker >= 0xe0 && marker <= 0xef) || marker == 0xfe) continue;
    break;
  }
  free(decoder);
  return result;
}
