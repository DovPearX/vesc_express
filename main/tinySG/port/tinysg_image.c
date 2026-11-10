#include "tinysg_image.h"

static uint16_t read_u16(const uint8_t *data) {
  return (uint16_t)(data[0] << 8 | data[1]);
}

bool tinysg_image_info(const uint8_t *data, size_t bytes, tinysg_asset *asset) {
  if (!data || !asset || bytes < 5) return false;
  *asset = (tinysg_asset){0};
  if (data[0] == 0xff && data[1] == 0xd8) {
    size_t offset = 2;
    while (offset + 3 < bytes) {
      if (data[offset++] != 0xff) return false;
      while (offset < bytes && data[offset] == 0xff) offset++;
      if (offset >= bytes) return false;
      uint8_t marker = data[offset++];
      if (marker == 0xda || marker == 0xd9 || offset + 2 > bytes) return false;
      size_t length = read_u16(data + offset);
      if (length < 2 || length > bytes - offset) return false;
      if (marker == 0xc0) {
        if (length < 8 || data[offset + 2] != 8) return false;
        asset->height = read_u16(data + offset + 3);
        asset->width = read_u16(data + offset + 5);
        asset->jpeg = true;
        return asset->width && asset->height && asset->width <= INT16_MAX && asset->height <= INT16_MAX;
      }
      offset += length;
    }
    return false;
  }
  asset->width = read_u16(data);
  asset->height = read_u16(data + 2);
  if (!asset->width || !asset->height || asset->width > INT16_MAX || asset->height > INT16_MAX) return false;
  switch (data[4]) {
  case 0x81:
    asset->format = TINYSG_A1;
    break;
  case 0:
    asset->format = TINYSG_A8;
    break;
  case 8:
    asset->format = TINYSG_RGB332;
    break;
  case 16:
    asset->format = TINYSG_RGB565;
    break;
  case 24:
    asset->format = TINYSG_RGB888;
    break;
  default:
    return false;
  }
  asset->bytes = tinysg_image_size((tinysg_pixel_format)asset->format, asset->width, asset->height);
  asset->pixels = data + 5;
  return asset->bytes <= bytes - 5;
}
