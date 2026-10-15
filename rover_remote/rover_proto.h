#pragma once
#include <stdint.h>

#define PKT_MAGIC 0xA7

enum PktType : uint8_t {
  PKT_PING = 1,
  PKT_ACK,
  PKT_DRIVE,
  PKT_STOP,
  PKT_MODE,
  PKT_ROTATE,
  PKT_HEADING_RESET
};

enum DriveMode : uint8_t {
  MODE_NORMAL = 0,
  MODE_CRAWL,
  MODE_CRUISE,
  MODE_SPORT,
  MODE_FIELD,
  MODE_HEADING_LOCK,
  MODE_DRIFT,
  MODE_ORBIT,
  MODE_TANK,
  MODE_COUNT
};

#define FLAG_TURBO 0x01
#define FLAG_CRAWL 0x02

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t type;
  int8_t x, y, z;
  uint8_t speed;
  uint8_t mode;
  uint8_t flags;
  int16_t arg;
  uint16_t seq;
} RoverPkt;
