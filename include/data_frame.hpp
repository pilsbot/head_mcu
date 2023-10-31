#pragma once

#include <inttypes.h>

namespace head_mcu {

struct __attribute((__packed__)) Frame {
  uint16_t analog0;
  uint16_t analog1;
  union {
    uint8_t as_byte;
    struct __attribute((__packed__)) {
      uint8_t bit0:1;
      uint8_t bit1:1;
      uint8_t bit2:1;
      uint8_t bit3:1;
      uint8_t bit4:1;
      uint8_t bit5:1;
      uint8_t bit6:1;
      uint8_t bit7:1;
    } as_bit;
  } digital0_8;
};

typedef uint16_t UpdatePeriodMs;

struct __attribute((__packed__)) Command {
  static constexpr uint16_t MAGIC = 0xAFFE;
  uint16_t magic;
  enum Type : uint8_t
  {
    setUpdatePeriod = 0,
    setOutputFrame
  } type;
  union __attribute((__packed__)) {
    Frame frame;
    UpdatePeriodMs updatePeriod_ms;
  };
};

}

//char (*__kaboom)[sizeof( head_mcu::Frame )] = 1;

static_assert(sizeof(head_mcu::Frame) == 5, "yo");
static_assert(sizeof(head_mcu::Command) == sizeof(head_mcu::Frame) + 3, "oy");
