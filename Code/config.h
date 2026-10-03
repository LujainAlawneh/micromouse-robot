// =====================================================================
//  CONFIG: pins, hardware facts, auto-run settings
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// =====================================================================
//  CONFIG (fixed hardware facts)
// =====================================================================
// AUTO RUN (battery test without USB)
//  power on -> LED blinks AUTO_RUN_DELAY_MS -> runs AUTO_RUN_CMD by itself.
//  Again: BOOT button or hold a hand ~1 cm in front of the front sensor, then
//  take it away -> LED blinks RESTART_DELAY_MS -> runs again.
//  BOOT during the blinking = cancel. Any key on Serial during it = cancel.
//  The result of every 'w'/'W' run is saved in flash -> connect USB, press 'L'.
constexpr bool     AUTO_RUN          = true;
constexpr char     AUTO_RUN_CMD      = 'g';      // 'w' / 'W' / 'g' / 't' ...
constexpr uint32_t AUTO_RUN_DELAY_MS = 5000;     // time to put it down after power-on
constexpr uint32_t RESTART_DELAY_MS  = 3000;

// TB6612FNG (STBY tied to 3V3)
constexpr int PWMA = 25, AIN1 = 26, AIN2 = 27;   // channel A = LEFT
constexpr int PWMB = 4,  BIN1 = 16, BIN2 = 17;   // channel B = RIGHT (mirrored)

// Encoders (CHANGE interrupt on C1 only)
constexpr int LEFT_C1 = 32, LEFT_C2 = 33;
constexpr int RIGHT_C1 = 34, RIGHT_C2 = 35;      // input-only, no pull-up needed
// FIX: encoder direction. 'e' + push the robot forward one cell by hand ->
// BOTH must show about +770. If one is negative, flip only that sign.
constexpr int ENC_SIGN_L = -1;
constexpr int ENC_SIGN_R = -1;

// BNO055 on Wire1
constexpr int      IMU_SDA = 13, IMU_SCL = 14;
constexpr uint8_t  IMU_ADDR = 0x29;
constexpr uint32_t IMU_I2C_HZ = 100000;
// cw() MUST increase when the robot rotates CLOCKWISE (check with 'i').
// If 'r' spins forever instead of stopping at 90 deg -> flip this.
constexpr int      CW_SIGN = -1;
// Physical LEFT motor is on channel B ('r' was turning LEFT). If after this
// 'r' still turns left, set false and flip CW_SIGN instead.
constexpr bool     SWAP_MOTORS = true;
constexpr float    GYRO_DEADBAND = 0.15f;
constexpr int      GYRO_BIAS_SAMPLES = 300;
constexpr float    GYRO_BIAS_MAX = 0.5f;
constexpr bool     GYRO_LOW_LATENCY_DEFAULT = false;

// 3x VL6180X on Wire
constexpr int      TOF_SDA = 21, TOF_SCL = 22;
constexpr uint32_t TOF_I2C_HZ = 400000;
constexpr int      XSHUT_PIN[3] = {18, 19, 23};   // LEFT, FRONT, RIGHT
constexpr uint8_t  TOF_ADDR[3]  = {0x30, 0x31, 0x32};
constexpr bool     APPLY_TOF_OFFSET = true;
constexpr int8_t   TOF_OFFSET[3] = {120, 0, 90};      // updated (measured)
constexpr uint16_t REG_PTP_OFFSET = 0x0024;
constexpr uint16_t REG_XTALK_RATE = 0x001E;
constexpr uint16_t REG_MAX_CONV   = 0x001C;            // SYSRANGE__MAX_CONVERGENCE_TIME
// FIX: period must be > max convergence + readout averaging (~4.3 ms).
// Old: 20 ms period with the default 49 ms convergence -> sensor stalls when
// there is no wall -> "[ToF] ... re-init" spam.
constexpr uint16_t TOF_PERIOD_MS  = 30;
constexpr uint8_t  TOF_MAX_CONV_MS = 18;

// Odometry (measured)
// Odometry scale and cell length are TUNABLE now (P.MM_PER_CNT / P.CELL_LEN):
// press 'k' to calibrate the scale, and set CELL_LEN to your measured cell pitch.
// 'k' calibration distance -> P.CAL_MM (tunable: short mazes have no 500 mm run)
constexpr float WALL_MM      = 12.0f;

// Start position
constexpr bool  START_AT_BACK_WALL = true;
constexpr float AXLE_TO_TAIL_MM    = 35.0f;

// Misc
constexpr int      PIN_BUTTON = 0;
constexpr int      PIN_LED    = 2;
constexpr uint32_t CONTROL_PERIOD_US = 3000;
constexpr int      MAZE_N = 16;


