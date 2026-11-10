#include "tinysg_internal.h"

static const int16_t quarter_sine[91] = {
    0,     572,   1144,  1715,  2286,  2856,  3425,  3993,  4560,  5126,  5690,  6252,  6813,  7371,  7927,  8481,
    9032,  9580,  10126, 10668, 11207, 11743, 12275, 12803, 13328, 13848, 14364, 14876, 15383, 15886, 16383, 16876,
    17364, 17846, 18323, 18794, 19260, 19720, 20173, 20621, 21062, 21497, 21925, 22347, 22762, 23170, 23571, 23964,
    24351, 24730, 25101, 25465, 25821, 26169, 26509, 26841, 27165, 27481, 27788, 28087, 28377, 28659, 28932, 29196,
    29451, 29697, 29934, 30162, 30381, 30591, 30791, 30982, 31163, 31335, 31498, 31650, 31794, 31927, 32051, 32165,
    32269, 32364, 32448, 32523, 32587, 32642, 32687, 32722, 32747, 32762, 32767,
};

int16_t tinysg_sin_deg(int32_t angle) {
  int32_t a = angle % 360;
  if (a < 0) a += 360;
  if (a <= 90) return quarter_sine[a];
  if (a <= 180) return quarter_sine[180 - a];
  if (a <= 270) return (int16_t)-quarter_sine[a - 180];
  return (int16_t)-quarter_sine[360 - a];
}

int16_t tinysg_cos_deg(int32_t angle) {
  return tinysg_sin_deg(angle % 360 + 90);
}

static void sqrt_step(uint32_t *value, uint32_t *result, uint32_t bit) {
  uint32_t candidate = *result + bit;
  uint32_t next = *result >> 1;
  if (*value < candidate) {
    *result = next;
    return;
  }
  *value -= candidate;
  *result = next + bit;
}

uint32_t tinysg_integer_sqrt(uint32_t value) {
  uint32_t result = 0, bit = 1U << 30;
  while (bit > value) bit >>= 2;
  while (bit) {
    sqrt_step(&value, &result, bit);
    bit >>= 2;
  }
  return result;
}
