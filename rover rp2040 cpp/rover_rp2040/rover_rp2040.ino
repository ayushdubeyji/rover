#include <Arduino.h>
#include <Wire.h>
#include <pico/mutex.h>
#include "pins.h"
#include "rover_link.h"
#include "motor_ctrl.h"
#include "ahrs.h"
#include "tof_sensors.h"
#include "autonomy.h"

// ==============================================================================
// Multicore Synchronization & Shared State
// ==============================================================================
auto_init_mutex(i2c_mutex);
auto_init_mutex(data_mutex);

volatile bool core0_ready = false;

// Shared command from UART to Core 0 control loop
struct SharedCommand {
  bool has_cmd = false;
  float x = 0, y = 0, z = 0, speed = 0;
  DriveMode mode = MODE_NORMAL;
  uint8_t flags = 0;
  uint32_t last_cmd_time = 0;
  bool is_stop = false;
  bool is_single_wheel = false;
  uint8_t wheel_idx = 0;
  float wheel_spd = 0;
  bool is_macro = false;
  uint8_t macro_id = 0;
  int8_t macro_dir = 0;
  bool is_heading_reset = false;
  uint16_t seq = 0;

  // Queued motor configuration to avoid cross-core racing
  bool has_cfg = false;
  int8_t cfg_invert[4] = { 1, 1, 1, 1 };
  int8_t cfg_strafe_sign = 1;
  int8_t cfg_rot_sign = 1;
  uint32_t cfg_min_duty = 38000;
  float cfg_accel = 25.0f;
  uint32_t cfg_spin_ms = 1400;
};

SharedCommand shared_cmd;

// Shared telemetry from Core 0 to Core 1 UART transmitter
TelemPkt shared_telem = {};
uint16_t shared_tof[2] = { 0, 0 };
float shared_heading = 0.0f;
float shared_dynJerk = 0.0f;

// Optional single-encoder counter on GP28
volatile int32_t enc_pulse_count = 0;
void isr_encoder_pulse() {
  enc_pulse_count++;
}

// Module instances
RoverMotors   motors;
RoverAHRS     ahrs;
DualToFDriver tof;
RoverAutonomy autonomy;

// Stream parsers for each serial channel
RoverFrameParser parser_uart0; // ESP32-C3
RoverFrameParser parser_uart1; // Raspberry Pi 3B+
RoverFrameParser parser_cdc;   // USB CDC

// ==============================================================================
// ASCII Command Handler (Fallback & Bench Testing)
// ==============================================================================
void handle_ascii_line(const char *line, size_t len) {
  if (len < 1) return;
  char c = line[0];

  mutex_enter_blocking(&data_mutex);
  shared_cmd.last_cmd_time = millis();
  if (c == 'D' && len >= 3) {
    float x = 0, y = 0, z = 0, s = 50;
    // format: D,x,y,z,s
    if (sscanf(line, "D,%f,%f,%f,%f", &x, &y, &z, &s) >= 4) {
      shared_cmd.has_cmd = true;
      shared_cmd.is_stop = false;
      shared_cmd.is_single_wheel = false;
      shared_cmd.is_macro = false;
      shared_cmd.x = x; shared_cmd.y = y; shared_cmd.z = z; shared_cmd.speed = s;
    }
  } else if (c == 'S') {
    shared_cmd.has_cmd = true;
    shared_cmd.is_stop = true;
  } else if (c == 'W' && len >= 3) {
    int w = 0; float spd = 0;
    if (sscanf(line, "W,%d,%f", &w, &spd) >= 2) {
      shared_cmd.has_cmd = true;
      shared_cmd.is_stop = false;
      shared_cmd.is_single_wheel = true;
      shared_cmd.wheel_idx = (uint8_t)w;
      shared_cmd.wheel_spd = spd;
    }
  } else if (c == 'M' && len >= 3) {
    int mid = 0, dir = 1;
    if (sscanf(line, "M,%d,%d", &mid, &dir) >= 2) {
      shared_cmd.has_cmd = true;
      shared_cmd.is_stop = false;
      shared_cmd.is_macro = true;
      shared_cmd.macro_id = (uint8_t)mid;
      shared_cmd.macro_dir = (int8_t)dir;
    }
  } else if (c == 'K' && len >= 10) {
    int inv[4], st, rt, md, sp;
    float ac;
    if (sscanf(line, "K,%d,%d,%d,%d,%d,%d,%d,%f,%d",
               &inv[0], &inv[1], &inv[2], &inv[3], &st, &rt, &md, &ac, &sp) >= 9) {
      shared_cmd.has_cfg = true;
      for (int i = 0; i < 4; i++) shared_cmd.cfg_invert[i] = (inv[i] == 1) ? -1 : 1;
      shared_cmd.cfg_strafe_sign = (st == 1) ? -1 : 1;
      shared_cmd.cfg_rot_sign = (rt == 1) ? -1 : 1;
      shared_cmd.cfg_min_duty = (md < 0) ? 0 : (uint32_t)md;
      shared_cmd.cfg_accel = ac;
      shared_cmd.cfg_spin_ms = (sp < 0) ? 200 : (uint32_t)sp;
    }
  }
  mutex_exit(&data_mutex);
}

// ==============================================================================
// Binary Frame Dispatcher
// ==============================================================================
void handle_binary_frame(uint8_t msg_type, const uint8_t *payload, uint8_t len) {
  if (msg_type == MSG_ROVER_CMD && len >= sizeof(RoverPkt)) {
    RoverPkt pkt;
    memcpy(&pkt, payload, sizeof(RoverPkt));

    mutex_enter_blocking(&data_mutex);
    shared_cmd.last_cmd_time = millis();
    shared_cmd.mode = (DriveMode)(pkt.mode % MODE_COUNT);
    shared_cmd.flags = pkt.flags;
    if (pkt.type == PKT_STOP) {
      shared_cmd.has_cmd = true;
      shared_cmd.is_stop = true;
    } else if (pkt.type == PKT_DRIVE) {
      shared_cmd.has_cmd = true;
      shared_cmd.is_stop = false;
      shared_cmd.is_single_wheel = false;
      shared_cmd.is_macro = false;
      shared_cmd.x = (float)pkt.x;
      shared_cmd.y = (float)pkt.y;
      shared_cmd.z = (float)pkt.z;
      shared_cmd.speed = (float)pkt.speed;
      shared_cmd.seq = pkt.seq;
    } else if (pkt.type == PKT_MODE) {
      shared_cmd.has_cmd = true;
      shared_cmd.speed = (float)pkt.speed;
      shared_cmd.seq = pkt.seq;
    } else if (pkt.type == PKT_ROTATE) {
      // Rotate in place macro
      shared_cmd.has_cmd = true;
      shared_cmd.is_stop = false;
      shared_cmd.is_macro = true;
      shared_cmd.macro_id = 1; // 360 spin
      shared_cmd.macro_dir = (pkt.z >= 0) ? 1 : -1;
      shared_cmd.seq = pkt.seq;
    } else if (pkt.type == PKT_HEADING_RESET) {
      shared_cmd.has_cmd = true;
      shared_cmd.is_heading_reset = true;
      shared_cmd.seq = pkt.seq;
    }
    mutex_exit(&data_mutex);
  } else if (msg_type == MSG_CONFIG && len >= sizeof(ConfigPkt)) {
    ConfigPkt cfg;
    memcpy(&cfg, payload, sizeof(ConfigPkt));
    mutex_enter_blocking(&data_mutex);
    shared_cmd.has_cfg = true;
    for (int i = 0; i < 4; i++) shared_cmd.cfg_invert[i] = cfg.invert[i];
    shared_cmd.cfg_strafe_sign = cfg.strafe_sign;
    shared_cmd.cfg_rot_sign = cfg.rot_sign;
    shared_cmd.cfg_min_duty = cfg.min_duty;
    shared_cmd.cfg_accel = (float)cfg.accel / 10.0f;
    shared_cmd.cfg_spin_ms = cfg.spin_ms;
    mutex_exit(&data_mutex);
  }
}

// Streaming parser helper for serial channels
void process_serial_stream(Stream &serial, RoverFrameParser &parser, char *ascii_buf, uint8_t &ascii_idx) {
  while (serial.available() > 0) {
    int c = serial.read();
    if (c < 0) break;
    uint8_t b = (uint8_t)c;

    // Check binary framing
    if (rover_parser_feed(&parser, b)) {
      handle_binary_frame(parser.msg_type, parser.payload, parser.len);
      ascii_idx = 0; // Binary frame completion resets ASCII buffer
    }

    // Only process ASCII characters when NOT in middle of a binary frame
    if (parser.state == PARSER_SEEK_SYNC0) {
      if (b == '\n' || b == '\r') {
        if (ascii_idx > 0) {
          ascii_buf[ascii_idx] = '\0';
          handle_ascii_line(ascii_buf, ascii_idx);
          ascii_idx = 0;
        }
      } else if (b >= 32 && b <= 126) {
        if (ascii_idx < 63) {
          ascii_buf[ascii_idx++] = (char)b;
        } else {
          ascii_idx = 0; // Buffer overflow reset
        }
      }
    }
  }
}

// ==============================================================================
// CORE 0: Motor PWM + 100 Hz Control Loop + IMU AHRS + Dead Reckoning
// ==============================================================================
void setup() {
  // 1. Initialize Serial Links
  Serial.begin(115200); // USB CDC
  
  Serial1.setTX(PIN_UART0_TX);
  Serial1.setRX(PIN_UART0_RX);
  Serial1.begin(ROVER_UART0_BAUD);

  Serial2.setTX(PIN_UART1_TX);
  Serial2.setRX(PIN_UART1_RX);
  Serial2.begin(ROVER_UART1_BAUD);

  rover_parser_init(&parser_uart0);
  rover_parser_init(&parser_uart1);
  rover_parser_init(&parser_cdc);

  // 2. Analog & Discrete Pins
  analogReadResolution(12);
  pinMode(PIN_BATTERY_ADC, INPUT);
  pinMode(PIN_RADAR_OUT, INPUT);
  pinMode(PIN_ENCODER_OPTIONAL, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_OPTIONAL), isr_encoder_pulse, RISING);

  // 3. Motors
  motors.begin();

  // 4. Shared I2C1 Bus
  Wire1.setSDA(PIN_I2C1_SDA);
  Wire1.setSCL(PIN_I2C1_SCL);
  Wire1.begin();
  Wire1.setClock(ROVER_I2C_FREQ);

  // 5. Initialize AHRS (MPU6050 + Magnetometer) with I2C Mutex
  mutex_enter_blocking(&i2c_mutex);
  ahrs.begin(Wire1);
  mutex_exit(&i2c_mutex);

  core0_ready = true;
}

void loop() {
  static uint32_t last_tick_us = 0;
  uint32_t now_us = micros();

  // Strict 100 Hz (10 ms) Control Loop
  if (now_us - last_tick_us >= 10000) {
    last_tick_us = now_us;
    uint32_t now_ms = millis();

    // 1. Check for incoming commands from Core 1
    mutex_enter_blocking(&data_mutex);
    if (shared_cmd.has_cfg) {
      motors.setConfig(shared_cmd.cfg_invert, shared_cmd.cfg_strafe_sign, shared_cmd.cfg_rot_sign,
                       shared_cmd.cfg_min_duty, shared_cmd.cfg_accel, shared_cmd.cfg_spin_ms);
      shared_cmd.has_cfg = false;
    }

    DriveMode cur_mode = shared_cmd.mode;
    uint8_t cur_flags = shared_cmd.flags;
    bool crash_prot_on = !(cur_flags & FLAG_CRASH_OFF);
    uint32_t last_cmd_ms = shared_cmd.last_cmd_time;
    float stick_x = shared_cmd.x;
    float stick_y = shared_cmd.y;
    float stick_z = shared_cmd.z;
    float cmd_speed = shared_cmd.speed;

    if (shared_cmd.has_cmd) {
      if (shared_cmd.is_stop) {
        motors.stopAll();
      } else if (shared_cmd.is_heading_reset) {
        ahrs.zeroYaw();
        shared_cmd.is_heading_reset = false;
      } else if (shared_cmd.is_macro) {
        motors.startMacro(shared_cmd.macro_id, shared_cmd.macro_dir);
      } else if (shared_cmd.is_single_wheel) {
        motors.setSingleWheel(shared_cmd.wheel_idx, shared_cmd.wheel_spd);
      }
      shared_cmd.has_cmd = false;
    }
    uint16_t cur_tof[2] = { shared_tof[0], shared_tof[1] };
    mutex_exit(&data_mutex);

    // 2. Compute Forward & Strafe Velocities from Mecanum Kinematics
    // FL=0, FR=1, RL=2, RR=3
    // Pure Forward (y): w[0]=y, w[1]=y, w[2]=y, w[3]=y  => (w0 + w1 + w2 + w3) / 4 = y
    // Pure Strafe  (x): w[0]=x, w[1]=-x, w[2]=-x, w[3]=x => (w0 - w1 - w2 + w3) / 4 = x
    float fwd_speed = (motors.current[0] + motors.current[1] + motors.current[2] + motors.current[3]) / 4.0f;
    float strafe_speed = (motors.current[0] - motors.current[1] - motors.current[2] + motors.current[3]) / 4.0f;

    mutex_enter_blocking(&i2c_mutex);
    ahrs.step(Wire1, 0.01f, now_ms, motors.isMoving(), fwd_speed, strafe_speed);
    mutex_exit(&i2c_mutex);

    // 3. Step Autonomy & Crash Protection
    bool radar_moving = (digitalRead(PIN_RADAR_OUT) == HIGH);
    bool remote_alive = (last_cmd_ms > 0 && (now_ms - last_cmd_ms <= 1000));
    bool manual_override = (fabsf(stick_x) > 8.0f || fabsf(stick_y) > 8.0f || fabsf(stick_z) > 8.0f);

    float drive_x = 0.0f, drive_y = 0.0f, drive_z = 0.0f, drive_spd = cmd_speed;

    if (cur_mode == MODE_AUTO) {
      if (manual_override) {
        // Exclusive remote authority overrides autonomy
        drive_x = stick_x;
        drive_y = stick_y;
        drive_z = stick_z;
        drive_spd = cmd_speed;
      } else if (remote_alive) {
        // Autonomous navigation FSM
        drive_spd = (cmd_speed > 5.0f) ? cmd_speed : 50.0f;
        autonomy.runAutonomous(drive_x, drive_y, drive_z, drive_spd,
                               cur_tof[0], cur_tof[1], ahrs.heading,
                               ahrs.odomX, ahrs.odomY, ahrs.dynJerk,
                               ahrs.impactDir, ahrs.impactUntil,
                               radar_moving, now_ms);
        drive_x *= 100.0f;
        drive_y *= 100.0f;
        drive_z *= 100.0f;
      } else {
        // Remote lost: failsafe stop
        drive_x = 0; drive_y = 0; drive_z = 0;
      }
    } else {
      // Manual Teleop Mode
      drive_x = stick_x;
      drive_y = stick_y;
      drive_z = stick_z;
      drive_spd = cmd_speed;
    }

    // Apply Front (0 deg) and Rear (180 deg) Crash Protection
    if (crash_prot_on) {
      float norm_x = drive_x / 100.0f;
      float norm_y = drive_y / 100.0f;
      float norm_z = drive_z / 100.0f;
      autonomy.applyCrashProtection(norm_x, norm_y, norm_z, drive_spd,
                                    cur_tof[0], cur_tof[1],
                                    ahrs.impactDir, ahrs.impactUntil, now_ms);
      drive_x = norm_x * 100.0f;
      drive_y = norm_y * 100.0f;
      drive_z = norm_z * 100.0f;
    }

    // Set motor drive targets if not executing macro
    if (!motors.macro_active && !shared_cmd.is_single_wheel) {
      if (cur_mode == MODE_AUTO) {
        if (remote_alive || manual_override) {
          motors.setDrive(drive_x, drive_y, drive_z, drive_spd);
        } else {
          motors.stopAll();
        }
      } else {
        motors.setDrive(drive_x, drive_y, drive_z, drive_spd);
      }
    }

    // 4. Step Motor Controller (accel ramp, stiction kick, failsafe)
    motors.step(now_ms);

    // 5. Read Analog Battery & Radar
    uint16_t adc_raw = analogRead(PIN_BATTERY_ADC);
    float v_adc = ((float)adc_raw / 4095.0f) * 3.3f;
    uint16_t bat_mv = (uint16_t)(v_adc * BATTERY_DIVIDER_RATIO * 1000.0f);
    (void)bat_mv;

    // 6. Update Shared Telemetry
    static uint16_t telem_seq = 0;
    mutex_enter_blocking(&data_mutex);
    shared_telem.magic = 0xA7;
    shared_telem.type = PKT_TELEM;
    shared_telem.q[0] = (int16_t)(ahrs.fusion.q0 * 16384.0f);
    shared_telem.q[1] = (int16_t)(ahrs.fusion.q1 * 16384.0f);
    shared_telem.q[2] = (int16_t)(ahrs.fusion.q2 * 16384.0f);
    shared_telem.q[3] = (int16_t)(ahrs.fusion.q3 * 16384.0f);

    for (int i = 0; i < 3; i++) {
      shared_telem.gyro[i] = (int16_t)((ahrs.fusion.gyroRaw[i] - ahrs.fusion.gyroBias[i]) * RAD_TO_DEG * 16.0f);
      shared_telem.acc[i]  = (int16_t)(ahrs.lastAcc[i] * 1000.0f);
    }
    shared_telem.ultraMm = 0; // Ultrasonic removed
    shared_telem.tofMm[0] = cur_tof[0]; // Front ToF (0 deg)
    shared_telem.tofMm[1] = cur_tof[1]; // Rear ToF (180 deg)

    for (int i = 0; i < 4; i++) {
      shared_telem.wheelMmS[i] = (int16_t)(motors.current[i] * 10.0f);
    }
    shared_telem.posCm[0] = (int16_t)ahrs.odomX;
    shared_telem.posCm[1] = (int16_t)ahrs.odomY;
    shared_telem.batMv = (uint16_t)autonomy.autoState; // Encodes autoState for C3 bridge & S3 remote
    shared_telem.calFlags = (ahrs.imuReady ? CAL_GYRO_OK : 0) |
                            (ahrs.magReady ? CAL_MAG_OK : 0) |
                            (ahrs.fusion.stationary ? CAL_STILL : 0) |
                            (ahrs.impactDir != IMPACT_NONE ? FLAG_IMPACT : 0);
    shared_telem.magCoverage = (uint8_t)(ahrs.fusion.magCoverage * 100.0f);
    shared_telem.seq = ++telem_seq;

    shared_heading = ahrs.heading;
    shared_dynJerk = ahrs.dynJerk;
    mutex_exit(&data_mutex);
  }
}

// ==============================================================================
// CORE 1: Dual VL53L0X Non-blocking Driver + UART Dispatcher
// ==============================================================================
void setup1() {
  while (!core0_ready) {
    delay(5);
  }

  // Initialize Dual ToF sensors with fine-grained I2C Mutex
  tof.begin(Wire1, &i2c_mutex);
}

void loop1() {
  static uint32_t last_tof_ms = 0;
  static uint32_t last_telem_ms = 0;
  static uint32_t last_ascii_tel_ms = 0;

  static char ascii_buf_uart0[64]; static uint8_t ascii_idx_uart0 = 0;
  static char ascii_buf_uart1[64]; static uint8_t ascii_idx_uart1 = 0;
  static char ascii_buf_cdc[64];   static uint8_t ascii_idx_cdc = 0;

  uint32_t now = millis();

  // 1. Process Incoming Serial Streams
  process_serial_stream(Serial1, parser_uart0, ascii_buf_uart0, ascii_idx_uart0);
  process_serial_stream(Serial2, parser_uart1, ascii_buf_uart1, ascii_idx_uart1);
  process_serial_stream(Serial,  parser_cdc,   ascii_buf_cdc,   ascii_idx_cdc);

  // 2. Non-blocking Dual ToF Driver (~35 Hz / every 28 ms)
  if (now - last_tof_ms >= 28) {
    last_tof_ms = now;

    tof.update(Wire1, now, &i2c_mutex);

    mutex_enter_blocking(&data_mutex);
    shared_tof[0] = tof.front_mm;
    shared_tof[1] = tof.rear_mm;
    mutex_exit(&data_mutex);
  }

  // 3. 50 Hz Binary Telemetry Dispatch over UART0 (C3), UART1 (Pi 3B+), and USB CDC (Primary Pi Link)
  if (now - last_telem_ms >= 20) {
    last_telem_ms = now;

    TelemPkt t;
    mutex_enter_blocking(&data_mutex);
    t = shared_telem;
    mutex_exit(&data_mutex);

    uint8_t tx_frame[sizeof(TelemPkt) + 6];
    size_t frame_len = rover_pack_frame(MSG_TELEM, &t, sizeof(t), tx_frame, sizeof(tx_frame));
    if (frame_len > 0) {
      Serial1.write(tx_frame, frame_len);
      Serial2.write(tx_frame, frame_len);
      if (Serial) {
        Serial.write(tx_frame, frame_len);
      }
    }
  }

  // 4. Periodic 5 Hz ASCII Telemetry on USB CDC Serial Monitor
  if (now - last_ascii_tel_ms >= 200) {
    last_ascii_tel_ms = now;
    if (Serial) {
      mutex_enter_blocking(&data_mutex);
      Serial.printf("T,%d,%d,%d,%d,bat:%umV,tof:[%u,%u],pos:[%d,%d],hd:%.1f,jerk:%.2f\n",
                    shared_telem.wheelMmS[0] / 10, shared_telem.wheelMmS[1] / 10,
                    shared_telem.wheelMmS[2] / 10, shared_telem.wheelMmS[3] / 10,
                    shared_telem.batMv, shared_telem.tofMm[0], shared_telem.tofMm[1],
                    shared_telem.posCm[0], shared_telem.posCm[1],
                    shared_heading, shared_dynJerk);
      mutex_exit(&data_mutex);
    }
  }
}
