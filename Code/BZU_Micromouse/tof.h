// =====================================================================
//  TOF: 3x VL6180X distance sensors
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// =====================================================================
//  TOF (3x VL6180X)
// =====================================================================
enum TofId : uint8_t { TOF_LEFT = 0, TOF_FRONT = 1, TOF_RIGHT = 2 };

struct TofReading {
  float    mm = 255;
  bool     valid = false;
  uint32_t seq = 0;
  uint32_t tMs = 0;
  uint32_t errors = 0;
};

struct WallRefs { float left = 60, front = 50, right = 60; };

namespace Tof {
constexpr uint8_t  DEFAULT_ADDR = 0x29;
constexpr uint16_t MODEL_ID = 0x000, FRESH_RESET = 0x016, SYSRANGE_START = 0x018,
                   INTERMEAS = 0x01B, INT_CONFIG = 0x014, INT_CLEAR = 0x015,
                   RANGE_STATUS = 0x04D, INT_STATUS = 0x04F, RANGE_VAL = 0x062,
                   SLAVE_ADDR = 0x212;

bool       okFlag[3] = {false, false, false};
TofReading rd[3];
WallRefs   wallRefs;
int        rr = 0;
uint32_t   lastRecoverTry[3] = {0, 0, 0};
const char* NAME[3] = {"LEFT", "FRONT", "RIGHT"};

void loadRefs();

bool w8(uint8_t a, uint16_t reg, uint8_t v) {
  Wire.beginTransmission(a);
  Wire.write(reg >> 8); Wire.write(reg & 0xFF); Wire.write(v);
  return Wire.endTransmission() == 0;
}
bool w16(uint8_t a, uint16_t reg, uint16_t v) {
  Wire.beginTransmission(a);
  Wire.write(reg >> 8); Wire.write(reg & 0xFF); Wire.write(v >> 8); Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}
int r8(uint8_t a, uint16_t reg) {
  Wire.beginTransmission(a);
  Wire.write(reg >> 8); Wire.write(reg & 0xFF);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)a, 1) != 1) return -1;
  return Wire.read();
}

void loadSettings(uint8_t a) {
  static const uint16_t S[][2] = {
    {0x0207, 0x01}, {0x0208, 0x01}, {0x0096, 0x00}, {0x0097, 0xfd}, {0x00e3, 0x00},
    {0x00e4, 0x04}, {0x00e5, 0x02}, {0x00e6, 0x01}, {0x00e7, 0x03}, {0x00f5, 0x02},
    {0x00d9, 0x05}, {0x00db, 0xce}, {0x00dc, 0x03}, {0x00dd, 0xf8}, {0x009f, 0x00},
    {0x00a3, 0x3c}, {0x00b7, 0x00}, {0x00bb, 0x3c}, {0x00b2, 0x09}, {0x00ca, 0x09},
    {0x0198, 0x01}, {0x01b0, 0x17}, {0x01ad, 0x00}, {0x00ff, 0x05}, {0x0100, 0x05},
    {0x0199, 0x05}, {0x01a6, 0x1b}, {0x01ac, 0x3e}, {0x01a7, 0x1f}, {0x0030, 0x00},
    {0x0011, 0x10}, {0x010a, 0x50}, {0x003f, 0x46}, {0x0031, 0xFF}, {0x0041, 0x63},
    {0x002e, 0x01}, {0x003e, 0x31}, {INT_CONFIG, 0x04},
  };
  for (auto& s : S) w8(a, s[0], (uint8_t)s[1]);
}

bool initOne(int i) {
  digitalWrite(XSHUT_PIN[i], LOW);  delay(2);
  digitalWrite(XSHUT_PIN[i], HIGH); delay(5);
  uint8_t a = DEFAULT_ADDR;
  if (r8(a, MODEL_ID) != 0xB4) return false;
  if (r8(a, FRESH_RESET) & 0x01) { loadSettings(a); w8(a, FRESH_RESET, 0x00); }
  if (!w8(a, SLAVE_ADDR, TOF_ADDR[i])) return false;
  a = TOF_ADDR[i];
  if (r8(a, MODEL_ID) != 0xB4) return false;
  if (APPLY_TOF_OFFSET) {
    w16(a, REG_XTALK_RATE, 0);
    w8(a, REG_PTP_OFFSET, (uint8_t)TOF_OFFSET[i]);
  }
  w8(a, REG_MAX_CONV, TOF_MAX_CONV_MS);           // FIX: fits inside the period
  uint8_t per = TOF_PERIOD_MS >= 20 ? TOF_PERIOD_MS / 10 - 1 : 0;
  w8(a, INTERMEAS, per);
  w8(a, INT_CLEAR, 0x07);
  w8(a, SYSRANGE_START, 0x03);
  rd[i].tMs = millis();
  return true;
}

bool begin() {
  Wire.begin(TOF_SDA, TOF_SCL, TOF_I2C_HZ);
  Wire.setTimeOut(10);
  for (int i = 0; i < 3; i++) { pinMode(XSHUT_PIN[i], OUTPUT); digitalWrite(XSHUT_PIN[i], LOW); }
  delay(10);
  bool all = true;
  for (int i = 0; i < 3; i++) {
    okFlag[i] = false;
    for (int t = 0; t < 3 && !okFlag[i]; t++) okFlag[i] = initOne(i);
    Serial.printf("[ToF] %-5s %s @0x%02X offset %d\n", NAME[i], okFlag[i] ? "ok " : "FAIL",
                  TOF_ADDR[i], APPLY_TOF_OFFSET ? TOF_OFFSET[i] : 0);
    if (!okFlag[i]) { digitalWrite(XSHUT_PIN[i], LOW); all = false; }
  }
  loadRefs();
  return all;
}

void poll(bool allowRecover = false) {
  int i = rr; rr = (rr + 1) % 3;
  uint32_t now = millis();
  TofReading& r = rd[i];

  if (!okFlag[i]) {
    r.valid = false;
    if (allowRecover && now - lastRecoverTry[i] > 1000) {
      lastRecoverTry[i] = now;
      okFlag[i] = initOne(i);
      if (!okFlag[i]) digitalWrite(XSHUT_PIN[i], LOW);
      Serial.printf("[ToF] %s re-init %s\n", NAME[i], okFlag[i] ? "ok" : "failed");
    }
    return;
  }
  uint8_t a = TOF_ADDR[i];
  int st = r8(a, INT_STATUS);
  if (st < 0) { r.errors++; }
  else if ((st & 0x07) == 0x04) {
    int status = r8(a, RANGE_STATUS);
    int val    = r8(a, RANGE_VAL);
    w8(a, INT_CLEAR, 0x07);
    if (status < 0 || val < 0) { r.errors++; r.valid = false; }
    else {
      r.mm    = val;
      r.valid = ((status >> 4) == 0) && val < 250;
      if (!r.valid) r.errors++;
      r.seq++;
      r.tMs = now;
    }
  }
  if (now - r.tMs > 150) r.valid = false;
  if (now - r.tMs > 1000) okFlag[i] = false;
}

const TofReading& get(TofId id) { return rd[id]; }
bool ok(TofId id) { return okFlag[id]; }
WallRefs& refs() { return wallRefs; }

void loadRefs() {
  Preferences p;
  p.begin("tofref", true);
  if (p.isKey("L")) {
    wallRefs.left = p.getFloat("L"); wallRefs.front = p.getFloat("F"); wallRefs.right = p.getFloat("R");
    Serial.printf("[ToF] refs L %.1f F %.1f R %.1f (calibrated)\n", wallRefs.left, wallRefs.front, wallRefs.right);
  } else {
    Serial.println("[ToF] refs = defaults -> run 'c' calibration!");
  }
  p.end();
}

void saveRefs() {
  Preferences p;
  p.begin("tofref", false);
  p.putFloat("L", wallRefs.left); p.putFloat("F", wallRefs.front); p.putFloat("R", wallRefs.right);
  p.end();
}

bool wallLeft()  { return rd[0].valid && rd[0].mm < wallRefs.left  + P.SIDE_MARGIN; }
bool wallFront() { return rd[1].valid && rd[1].mm < wallRefs.front + P.FRONT_MARGIN; }
bool wallRight() { return rd[2].valid && rd[2].mm < wallRefs.right + P.SIDE_MARGIN; }
}

