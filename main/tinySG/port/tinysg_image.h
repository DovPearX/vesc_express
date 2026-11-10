#ifndef TINYSG_IMAGE_H
#define TINYSG_IMAGE_H

#include "tinysg.h"

typedef struct {
  uint16_t width, height;
  uint8_t format;
  const uint8_t *pixels;
  size_t bytes;
  bool jpeg;
} tinysg_asset;

bool tinysg_image_info(const uint8_t *data, size_t bytes, tinysg_asset *asset);
bool tinysg_jpeg_decode(const uint8_t *data, size_t bytes, uint8_t *pixels, uint16_t width, uint16_t height);

#endif
