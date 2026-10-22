#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Frame Synchronization & Protocol Constants ---
#define ROVER_LINK_SYNC0          0xAA
#define ROVER_LINK_SYNC1          0x55
#define ROVER_LINK_MAX_PAYLOAD    64
#define ROVER_LINK_MAX_FRAME_SIZE (ROVER_LINK_MAX_PAYLOAD + 6)
#define PKT_MAGIC                 0xA7

// Default & fallback baud rates
#define ROVER_UART0_BAUD_DEFAULT  921600
#define ROVER_UART0_BAUD_FALLBACK 460800
#define ROVER_UART1_BAUD          115200

// Message Types
enum RoverMsgType : uint8_t {
  MSG_NONE        = 0x00,
  MSG_ROVER_CMD   = 0x01,  // Encapsulates RoverPkt
  MSG_TELEM       = 0x02,  // Encapsulates TelemPkt
  MSG_ENCODER     = 0x03,  // Encapsulates EncoderPkt
  MSG_CALIB       = 0x04,  // CalibCmd / CalibPkt
  MSG_PING        = 0x05,
  MSG_ACK         = 0x06,
  MSG_CONFIG      = 0x07,  // Motor config & tuning parameters
  MSG_RAW_PWM     = 0x08   // Direct per-motor speed (-100..100)
};

// Legacy packet types inside RoverPkt
enum PktType : uint8_t {
  PKT_PING          = 1,
  PKT_ACK           = 2,
  PKT_DRIVE         = 3,
  PKT_STOP          = 4,
  PKT_MODE          = 5,
  PKT_ROTATE        = 6,
  PKT_HEADING_RESET = 7,
  PKT_TELEM         = 8,
  PKT_CALIB         = 9
};

// Drive modes
enum DriveMode : uint8_t {
  MODE_NORMAL       = 0,
  MODE_CRAWL        = 1,
  MODE_CRUISE       = 2,
  MODE_SPORT        = 3,
  MODE_FIELD        = 4,
  MODE_HEADING_LOCK = 5,
  MODE_DRIFT        = 6,
  MODE_ORBIT        = 7,
  MODE_TANK         = 8,
  MODE_AUTO         = 9,
  MODE_COUNT        = 10
};

// Autonomous Navigation States (12 States)
enum AutoNavState : uint8_t {
  AUTO_CRUISE_CLEAR      = 0,
  AUTO_THINKING_SCAN     = 1,
  AUTO_PLANNING_PATH     = 2,
  AUTO_VEER_LEFT         = 3,
  AUTO_VEER_RIGHT        = 4,
  AUTO_CAUTIOUS_CREEP    = 5,
  AUTO_STUCK_DETECTED    = 6,
  AUTO_REVERSE_BACKTRACK = 7,
  AUTO_SCURVE_ESCAPE     = 8,
  AUTO_COMPASS_PIVOT     = 9,
  AUTO_WALL_FOLLOW       = 10,
  AUTO_IMPACT_REFLEX     = 11
};

// Mode / Telemetry Flags
#define FLAG_TURBO      0x01
#define FLAG_CRAWL      0x02
#define FLAG_AUTO       0x04
#define FLAG_CRASH_OFF  0x08
#define FLAG_IMPACT     0x10

// Impact Directions
enum ImpactDir : uint8_t {
  IMPACT_NONE  = 0,
  IMPACT_FRONT = 1,
  IMPACT_REAR  = 2,
  IMPACT_LEFT  = 3,
  IMPACT_RIGHT = 4
};

// Calibration Commands
enum CalibCmd : uint8_t {
  CAL_LEVEL     = 1,
  CAL_GYRO      = 2,
  CAL_MAG_RESET = 3,
  CAL_SAVE      = 4,
  CAL_ZERO_YAW  = 5
};

#define CAL_GYRO_OK   0x01
#define CAL_MAG_OK    0x02
#define CAL_LEVEL_OK  0x04
#define CAL_STILL     0x08

// Primary Drive Command (12 bytes)
typedef struct __attribute__((packed)) {
  uint8_t magic;    // 0xA7
  uint8_t type;     // PktType (e.g. PKT_DRIVE, PKT_STOP)
  int8_t x;         // Strafe: -100 (left) .. +100 (right)
  int8_t y;         // Fwd/Rev: -100 (back) .. +100 (fwd)
  int8_t z;         // Rotation: -100 (CCW) .. +100 (CW)
  uint8_t speed;    // Overall speed scaler: 0 .. 100%
  uint8_t mode;     // DriveMode
  uint8_t flags;    // FLAG_*
  int16_t arg;      // Optional argument (e.g. rotate target angle)
  uint16_t seq;     // Rolling sequence number
} RoverPkt;

// Primary Telemetry Packet (46 bytes)
typedef struct __attribute__((packed)) {
  uint8_t magic;         // 0xA7
  uint8_t type;          // PKT_TELEM (8)
  int16_t q[4];          // Quaternion * 16384 (q0, q1, q2, q3)
  int16_t gyro[3];       // Gyro deg/s * 16 (gx, gy, gz)
  int16_t acc[3];        // Accel mg (ax, ay, az, 1000 = 1g)
  uint16_t ultraMm;      // Legacy ultrasonic (0 = not installed)
  uint16_t tofMm[2];     // [0] = front (0 deg @ 11cm), [1] = rear (180 deg @ 10cm)
  int16_t wheelMmS[4];   // FL, FR, RL, RR estimated or measured speed
  int16_t posCm[2];      // Dead reckoning X, Y in cm
  uint16_t batMv;        // Battery mV or state indicator
  uint8_t calFlags;      // CAL_GYRO_OK, CAL_MAG_OK, etc.
  uint8_t magCoverage;   // 0..100%
  uint16_t seq;          // Rolling sequence number
} TelemPkt;

// Optional Encoder Packet (17 bytes)
typedef struct __attribute__((packed)) {
  uint8_t valid_mask;    // Bit 0: FL, Bit 1: FR, Bit 2: RL, Bit 3: RR, Bit 4: GP28 single-encoder
  int32_t enc_count[4];  // Accumulated tick counts for FL, FR, RL, RR
} EncoderPkt;

// Motor Configuration Packet (16 bytes)
typedef struct __attribute__((packed)) {
  int8_t invert[4];      // Invert flag for each wheel (-1 or 1)
  int8_t strafe_sign;    // -1 or 1
  int8_t rot_sign;       // -1 or 1
  uint16_t min_duty;     // 0 .. 65535 (default 38000)
  uint16_t accel;        // Acceleration step * 10 (e.g. 250 for 25.0)
  uint16_t spin_ms;      // 360 spin duration in ms (default 1400)
} ConfigPkt;

// --- CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection, no final XOR) ---
static inline uint16_t rover_crc16(const uint8_t *data, size_t len, uint16_t init) {
  uint16_t crc = init;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

// Pack a message into a framed buffer: [SYNC0, SYNC1, MSG_TYPE, LEN, PAYLOAD..., CRC16_H, CRC16_L]
// Returns total frame length, or 0 on error.
static inline size_t rover_pack_frame(uint8_t msg_type, const void *payload, uint8_t len,
                                      uint8_t *out_buf, size_t out_max) {
  if (!out_buf || len > ROVER_LINK_MAX_PAYLOAD || out_max < (size_t)(len + 6)) {
    return 0;
  }
  out_buf[0] = ROVER_LINK_SYNC0;
  out_buf[1] = ROVER_LINK_SYNC1;
  out_buf[2] = msg_type;
  out_buf[3] = len;
  if (len > 0 && payload) {
    memcpy(&out_buf[4], payload, len);
  }
  // CRC covers MSG_TYPE, LEN, and PAYLOAD
  uint16_t crc = rover_crc16(&out_buf[2], (size_t)(len + 2), 0xFFFF);
  out_buf[4 + len] = (uint8_t)((crc >> 8) & 0xFF);
  out_buf[5 + len] = (uint8_t)(crc & 0xFF);
  return (size_t)(len + 6);
}

// --- Non-blocking Streaming Frame Parser ---
typedef enum {
  PARSER_SEEK_SYNC0,
  PARSER_SEEK_SYNC1,
  PARSER_GET_TYPE,
  PARSER_GET_LEN,
  PARSER_GET_PAYLOAD,
  PARSER_GET_CRC_H,
  PARSER_GET_CRC_L
} ParserState;

typedef struct {
  ParserState state;
  uint8_t msg_type;
  uint8_t len;
  uint8_t payload_idx;
  uint8_t payload[ROVER_LINK_MAX_PAYLOAD];
  uint8_t crc_h;
  uint32_t rx_frames;
  uint32_t crc_errors;
} RoverFrameParser;

static inline void rover_parser_init(RoverFrameParser *parser) {
  if (!parser) return;
  memset(parser, 0, sizeof(RoverFrameParser));
  parser->state = PARSER_SEEK_SYNC0;
}

// Feeds one byte into parser.
// Returns true when a complete valid frame has been decoded.
// The caller can inspect parser->msg_type, parser->len, and parser->payload.
static inline bool rover_parser_feed(RoverFrameParser *parser, uint8_t b) {
  if (!parser) return false;

  switch (parser->state) {
    case PARSER_SEEK_SYNC0:
      if (b == ROVER_LINK_SYNC0) {
        parser->state = PARSER_SEEK_SYNC1;
      }
      break;

    case PARSER_SEEK_SYNC1:
      if (b == ROVER_LINK_SYNC1) {
        parser->state = PARSER_GET_TYPE;
      } else if (b == ROVER_LINK_SYNC0) {
        // Handle repeated 0xAA preamble
        parser->state = PARSER_SEEK_SYNC1;
      } else {
        parser->state = PARSER_SEEK_SYNC0;
      }
      break;

    case PARSER_GET_TYPE:
      parser->msg_type = b;
      parser->state = PARSER_GET_LEN;
      break;

    case PARSER_GET_LEN:
      if (b > ROVER_LINK_MAX_PAYLOAD) {
        // Frame too large - invalid
        parser->state = PARSER_SEEK_SYNC0;
      } else {
        parser->len = b;
        parser->payload_idx = 0;
        if (parser->len == 0) {
          parser->state = PARSER_GET_CRC_H;
        } else {
          parser->state = PARSER_GET_PAYLOAD;
        }
      }
      break;

    case PARSER_GET_PAYLOAD:
      parser->payload[parser->payload_idx++] = b;
      if (parser->payload_idx >= parser->len) {
        parser->state = PARSER_GET_CRC_H;
      }
      break;

    case PARSER_GET_CRC_H:
      parser->crc_h = b;
      parser->state = PARSER_GET_CRC_L;
      break;

    case PARSER_GET_CRC_L: {
      uint16_t expected_crc = ((uint16_t)parser->crc_h << 8) | b;
      // Calculate CRC over MSG_TYPE, LEN, and PAYLOAD
      uint8_t hdr[2] = {parser->msg_type, parser->len};
      uint16_t calc_crc = rover_crc16(hdr, 2, 0xFFFF);
      if (parser->len > 0) {
        calc_crc = rover_crc16(parser->payload, parser->len, calc_crc);
      }
      parser->state = PARSER_SEEK_SYNC0;
      if (calc_crc == expected_crc) {
        parser->rx_frames++;
        return true;
      } else {
        parser->crc_errors++;
        return false;
      }
    }
  }
  return false;
}

#ifdef __cplusplus
}
#endif
