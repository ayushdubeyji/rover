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
  MODE_AUTO,
  MODE_COUNT
};

#define FLAG_TURBO     0x01
#define FLAG_CRAWL     0x02
#define FLAG_AUTO      0x04
#define FLAG_CRASH_OFF 0x08
#define FLAG_IMPACT    0x10

enum ImpactDir : uint8_t {
  IMPACT_NONE = 0,
  IMPACT_FRONT = 1,
  IMPACT_REAR = 2,
  IMPACT_LEFT = 3,
  IMPACT_RIGHT = 4
};

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

#define PKT_TELEM 8
#define PKT_CALIB 9
enum CalibCmd : uint8_t { CAL_LEVEL = 1, CAL_GYRO = 2, CAL_MAG_RESET = 3, CAL_SAVE = 4, CAL_ZERO_YAW = 5 };

#define CAL_GYRO_OK  0x01
#define CAL_MAG_OK   0x02
#define CAL_LEVEL_OK 0x04
#define CAL_STILL    0x08

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t type;
  int16_t q[4];
  int16_t gyro[3];
  int16_t acc[3];
  uint16_t ultraMm;
  uint16_t tofMm[2];
  int16_t wheelMmS[4];
  int16_t posCm[2];
  uint16_t batMv;
  uint8_t calFlags;
  uint8_t magCoverage;
  uint16_t seq;
} TelemPkt;
