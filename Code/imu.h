// =====================================================================
//  IMU: BNO055 gyro / heading
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// =====================================================================
//  IMU (BNO055)
// =====================================================================
namespace Imu {
Adafruit_BNO055 bno(55, IMU_ADDR, &Wire1);
bool  okFlag = false, lowLat = false;
float gyroBias = 0, yawDeg = 0, gzN = 0;
bool  still = false;
double stillSum = 0; int stillN = 0;

void setLowLatency(bool on);
bool measureBias();

bool readGzRaw(float& out) {
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(0x18);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom((int)IMU_ADDR, 2) != 2) return false;
  int16_t v = (int16_t)(Wire1.read() | (Wire1.read() << 8));
  out = v / 16.0f;
  return true;
}

void write8(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(reg); Wire1.write(val);
  Wire1.endTransmission();
}

bool begin() {
  Wire1.begin(IMU_SDA, IMU_SCL, IMU_I2C_HZ);
  okFlag = bno.begin(OPERATION_MODE_IMUPLUS);
  if (!okFlag) { Serial.println("[IMU] BNO055 NOT found on Wire1 @0x29"); return false; }
  delay(50);
  bno.setExtCrystalUse(true);
  delay(50);
  setLowLatency(GYRO_LOW_LATENCY_DEFAULT);
  for (int i = 0; i < 3; i++) { if (measureBias()) return true; delay(500); }
  return false;
}

void setLowLatency(bool on) {
  if (!okFlag) return;
  bno.setMode(OPERATION_MODE_CONFIG);
  if (on) {
    write8(0x07, 1);
    // FIX: range MUST be 2000 dps (bits 0). With 1000 dps the raw value is 2x
    // larger than 1/16 dps -> gyro reads double -> robot turned only ~45 deg.
    write8(0x0A, (2 << 3) | 0);      // GYR_CONFIG_0: 116 Hz bandwidth, +-2000 dps
    write8(0x0B, 0);
    write8(0x07, 0);
    bno.setMode(OPERATION_MODE_GYRONLY);
  } else {
    bno.setMode(OPERATION_MODE_IMUPLUS);
  }
  delay(50);
  lowLat = on;
  Serial.printf("[IMU] mode %s\n", on ? "GYRONLY 116Hz (low latency)" : "IMUPLUS (fusion)");
}

bool lowLatency() { return lowLat; }

bool measureBias() {
  if (!okFlag) return false;
  Serial.println("[IMU] measuring gyro bias - keep the robot STILL...");
  delay(300);
  double sum = 0; int n = 0;
  for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
    float g;
    if (readGzRaw(g)) { sum += g; n++; }
    delay(7);
  }
  if (n < GYRO_BIAS_SAMPLES / 2) { Serial.println("[IMU] bias FAILED (I2C)"); return false; }
  gyroBias = sum / n;
  yawDeg = 0;
  Serial.printf("[IMU] bias = %.3f dps\n", gyroBias);
  if (fabsf(gyroBias) > GYRO_BIAS_MAX) {
    Serial.println("[IMU] |bias| too big (robot moved?) - retrying, keep it STILL");
    return false;
  }
  return true;
}

void update(float dt) {
  float raw;
  if (!okFlag || !readGzRaw(raw)) return;
  if (still) { stillSum += raw; stillN++; }
  float gz = raw - gyroBias;
  if (fabsf(gz) < GYRO_DEADBAND) gz = 0;
  gzN = gz * P.GYRO_SCALE;
  yawDeg += gzN * dt;
}

void stillBegin() { still = true; stillSum = 0; stillN = 0; }

void stillEnd() {
  still = false;
  if (stillN >= 25) {
    float m = stillSum / stillN;
    if (fabsf(m - gyroBias) < 0.3f) gyroBias += 0.3f * (m - gyroBias);
  }
}

float yaw()    { return yawDeg; }
float gzNow()  { return gzN; }
float cw()     { return CW_SIGN * yawDeg; }              // EDIT 2: Use CW_SIGN
float rateCW() { return CW_SIGN * gzN; }                 // EDIT 2: Use CW_SIGN
float bias()   { return gyroBias; }
void  resetYaw() { yawDeg = 0; }
bool  ok()     { return okFlag; }
}

