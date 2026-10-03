// =====================================================================
//  MOTION: driving, turning, wall scans, recorder
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// =====================================================================
//  MOTION (turn, drive, etc.)
// =====================================================================
enum Dir : int { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };



// true only for a REAL typed key. Garbage bytes (0x00 / 0xFF) that an
// unpowered USB chip can put on RX when running on battery are ignored,
// otherwise they would stop every move.
static bool serialKeyHit() {
  bool hit = false;
  while (Serial.available()) {
    int c = Serial.read();
    if ((c >= 0x20 && c < 0x7F) || c == '\n' || c == '\r') hit = true;
  }
  return hit;
}

namespace Motion {
long     quarterTurns = 0;
float    dirTrim[4] = {0, 0, 0, 0};    // learned heading offset per direction (deg)
bool     abortFlag = false;
uint32_t lastUs = 0;
float    dtS = CONTROL_PERIOD_US * 1e-6f;
float    turnErr = 0, driveErr = 0;
uint32_t lastBrakeMs = 0, lastStopMs = 0;   // stop time between two drives
bool     scanCacheOk = false;               // last wall scan still valid (robot not moved)
// what happened in the last move (for the flash run log):
// 0 normal  U user stop  F front-wall emergency stop  T drive timeout
// H heading lost  E encoders not counting  t turn timeout  X turn wrong direction
char     event = 0;

inline float sgn(float x) { return x >= 0 ? 1.0f : -1.0f; }
void restartClock();
bool frontAlign();

// ---------------------------------------------------------------------
//  RECORDER: stores one sample every 2 control ticks (6 ms) in RAM while a
//  move runs (no Serial output during the move -> timing unchanged).
//  Printed afterwards as CSV. Phase letters:
//   A = turn accelerate  R = turn brake  B = turn creep  S = settle (brake/coast)
//   D = straight drive   K = front-align move   W = stopped wall scan
// ---------------------------------------------------------------------
struct RecS { uint16_t t; char ph; float cw, rate, dist, spd, front, lat; int16_t pl, pr; };
constexpr int REC_N = 600;                  // 3.6 s
RecS  rec[REC_N];
int   recN = 0;
bool  recOn = false;
uint32_t recT0 = 0;
uint8_t  recDiv = 0;
char  recPh = '-';
float recLat = NAN;
void recStart() { recN = 0; recDiv = 0; recT0 = millis(); recLat = NAN; recOn = true; }
void recStop()  { recOn = false; }
int  recCount() { return recN; }
const RecS& recGet(int i) { return rec[i]; }
static void recPush() {
  if (!recOn || recN >= REC_N) return;
  if (++recDiv < 2) return;
  recDiv = 0;
  const TofReading& F = Tof::get(TOF_FRONT);
  rec[recN++] = { (uint16_t)(millis() - recT0), recPh, Imu::cw(), Imu::rateCW(), Enc::distMM(),
                  Enc::speed(), F.valid ? F.mm : -1.0f, recLat, (int16_t)lastPwmL, (int16_t)lastPwmR };
}

void sensorsStep(bool idle) {
  while ((uint32_t)(micros() - lastUs) < CONTROL_PERIOD_US) { }
  uint32_t now = micros();
  dtS = (now - lastUs) * 1e-6f;
  if (dtS > 0.02f) dtS = CONTROL_PERIOD_US * 1e-6f;
  lastUs = now;
  Imu::update(dtS);
  Enc::update();
  Tof::poll(idle);
  recPush();
}

void settle(float rateTol, uint32_t holdMs, uint32_t maxMs) {
  recPh = 'S';
  brakeMotors();
  uint32_t t0 = millis(), okSince = 0;
  while (millis() - t0 < maxMs) {
    sensorsStep(false);
    bool still = fabsf(Imu::rateCW()) < rateTol && fabsf(Enc::speed()) < 8.0f;
    if (still) { if (!okSince) okSince = millis(); if (millis() - okSince >= holdMs) break; }
    else okSince = 0;
  }
}

// lateral error from the live side readings. 'src' returns which walls were used
// (bit0 = left, bit1 = right): when that changes, y jumps and the derivative must
// be restarted, otherwise the jump looks like a huge lateral speed.
float liveLateral(uint8_t* src = nullptr) {
  const WallRefs& ref = Tof::refs();
  const TofReading &L = Tof::get(TOF_LEFT), &R = Tof::get(TOF_RIGHT);
  float yl = L.mm - ref.left, yr = ref.right - R.mm;
  bool ul = L.valid && fabsf(yl) < P.SIDE_WINDOW, ur = R.valid && fabsf(yr) < P.SIDE_WINDOW;
  if (src) *src = (ul ? 1 : 0) | (ur ? 2 : 0);
  if (ul && ur) return 0.5f * (yl + yr);
  if (ul) return yl;
  if (ur) return yr;
  return NAN;
}

void headingDrive(float base, float cwTarget, float& integ, bool useLift, uint32_t tMs,
                  float corrGain = 1.0f) {
  (void)tMs;
  // written directly in clockwise terms (no sign juggling):
  float e = Imu::cw() - cwTarget;                          // + = rotated clockwise too far
  integ = constrain(integ + e * dtS, -50.0f, 50.0f);
  float corr = -(P.KP * e + P.KD * Imu::rateCW() + P.KI * integ);   // - = steer left
  corr = constrain(corr * corrGain, -P.MAX_CORR, P.MAX_CORR);
  float pl = base + corr, pr = base + P.RIGHT_TRIM - corr;
  if (useLift) {
    // FIX: both wheels always above the motor deadband -> no "one wheel starts first"
    float minP = P.END_PWM;
    float lift = max(0.0f, minP - min(pl, pr)); pl += lift; pr += lift;
    float over = max(0.0f, max(pl, pr) - 255.0f); pl -= over; pr -= over;
  }
  setLeft((int)lroundf(pl));
  setRight((int)lroundf(pr));
}

void begin() { pinMode(PIN_BUTTON, INPUT_PULLUP); restartClock(); }
void restartClock() { lastUs = micros(); }
void idleTick() { sensorsStep(true); }

bool tick() {
  sensorsStep(false);
  if (digitalRead(PIN_BUTTON) == LOW) abortFlag = true;
  if (Serial.available() && serialKeyHit()) abortFlag = true;
  if (abortFlag) { brakeMotors(); if (!event) event = 'U'; }
  return !abortFlag;
}

bool  aborted()        { return abortFlag; }
char  lastEvent()      { return event; }
void  clearAbort()     { abortFlag = false; }
int   dir()            { return (int)(((quarterTurns % 4) + 4) % 4); }
float headingTarget()  { return quarterTurns * 90.0f; }
float headingRef()     { return headingTarget() + dirTrim[dir()]; }   // + learned trim
void  clearTrim()      { for (int i = 0; i < 4; i++) dirTrim[i] = 0; }
float trim(int d)      { return dirTrim[d & 3]; }
float lastTurnError()  { return turnErr; }
float lastDriveError() { return driveErr; }
uint32_t lastStopTime() { return lastStopMs; }   // ms standing still before the last drive

void resetHeading() { Imu::resetYaw(); quarterTurns = 0; clearTrim(); scanCacheOk = false; lastBrakeMs = 0; }
void snapHeading()  { quarterTurns = lroundf(Imu::cw() / 90.0f); clearTrim(); scanCacheOk = false; }

// ---- wall scan helpers ----
static void scanCollect(float* sum, int* got, int* valid, uint32_t* last, int samples) {
  for (int i = 0; i < 3; i++) {
    const TofReading& r = Tof::get((TofId)i);
    if (r.seq == last[i] || got[i] >= samples) continue;
    last[i] = r.seq; got[i]++;
    if (r.valid) { sum[i] += r.mm; valid[i]++; }
  }
}
static WallScan scanBuild(const float* sum, const int* got, const int* valid) {
  WallScan w;
  const WallRefs& ref = Tof::refs();
  float mean[3];
  for (int i = 0; i < 3; i++) mean[i] = valid[i] ? sum[i] / valid[i] : NAN;
  w.l = mean[0]; w.f = mean[1]; w.r = mean[2];
  // majority vote: a wall needs more than half of the samples valid AND close
  w.left  = got[0] && valid[0] * 2 > got[0] && w.l < ref.left  + P.SIDE_MARGIN;
  w.front = got[1] && valid[1] * 2 > got[1] && w.f < ref.front + P.FRONT_MARGIN;
  w.right = got[2] && valid[2] * 2 > got[2] && w.r < ref.right + P.SIDE_MARGIN;
  float yl = w.left ? w.l - ref.left : NAN, yr = w.right ? ref.right - w.r : NAN;
  bool ul = w.left && fabsf(yl) < P.SIDE_WINDOW, ur = w.right && fabsf(yr) < P.SIDE_WINDOW;
  w.lateral = (ul && ur) ? 0.5f * (yl + yr) : ul ? yl : ur ? yr : NAN;
  return w;
}
// the last scan is reused as long as the robot has not moved or turned since
WallScan scanCache;
float scanCacheDist = 0, scanCacheCw = 0;
static void scanStore(const WallScan& w) {
  scanCache = w; scanCacheOk = true; scanCacheDist = Enc::distMM(); scanCacheCw = Imu::cw();
}

WallScan scan(int samples) {
  recPh = 'W';
  float sum[3] = {0, 0, 0};
  int got[3] = {0, 0, 0}, valid[3] = {0, 0, 0};
  uint32_t last[3];
  for (int i = 0; i < 3; i++) last[i] = Tof::get((TofId)i).seq;
  Imu::stillBegin();
  restartClock();
  uint32_t t0 = millis(), tMax = samples * TOF_PERIOD_MS * 3 + 150;
  while (millis() - t0 < tMax) {
    sensorsStep(true);
    scanCollect(sum, got, valid, last, samples);
    if (got[0] >= samples && got[1] >= samples && got[2] >= samples) break;
  }
  Imu::stillEnd();
  WallScan w = scanBuild(sum, got, valid);
  scanStore(w);
  return w;
}

// lateral of the last scan if the robot has not moved since (NAN otherwise) - never waits
float freshLateral() {
  if (scanCacheOk && fabsf(Enc::distMM() - scanCacheDist) < 1.5f && fabsf(Imu::cw() - scanCacheCw) < 1.5f)
    return scanCache.lateral;
  return NAN;
}

// reuse the last scan if the robot is still at the same place (saves a whole stop)
WallScan scanCached(int samples) {
  if (scanCacheOk && fabsf(Enc::distMM() - scanCacheDist) < 1.5f && fabsf(Imu::cw() - scanCacheCw) < 1.5f)
    return scanCache;
  return scan(samples);
}

// Brake + wait until still + read the walls AT THE SAME TIME (the ToF keeps measuring
// while the robot settles): one short stop instead of "settle, then scan".
static void settleScan(int samples, float rateTol, uint32_t holdMs, uint32_t maxMs) {
  recPh = 'S';
  brakeMotors();
  float sum[3] = {0, 0, 0};
  int got[3] = {0, 0, 0}, valid[3] = {0, 0, 0};
  uint32_t last[3];
  for (int i = 0; i < 3; i++) last[i] = Tof::get((TofId)i).seq;
  uint32_t t0 = millis(), okSince = 0;
  bool stillOn = false;
  while (millis() - t0 < maxMs) {
    sensorsStep(true);
    // samples only once nearly stopped (at 40 mm/s the robot moves ~1 mm per sample)
    if (fabsf(Enc::speed()) < 40.0f && fabsf(Imu::rateCW()) < 30.0f) scanCollect(sum, got, valid, last, samples);
    bool still = fabsf(Imu::rateCW()) < rateTol && fabsf(Enc::speed()) < 12.0f;
    if (still) { if (!okSince) { okSince = millis(); Imu::stillBegin(); stillOn = true; } }
    else       { okSince = 0; if (stillOn) { Imu::stillEnd(); stillOn = false; } }
    bool enough = got[0] >= samples && got[1] >= samples && got[2] >= samples;
    if (okSince && millis() - okSince >= holdMs && enough) break;
  }
  if (stillOn) Imu::stillEnd();
  if (got[0] >= samples && got[1] >= samples && got[2] >= samples) scanStore(scanBuild(sum, got, valid));
  else scanCacheOk = false;                  // not enough samples -> the next user scans again
}

bool driveMM(float mm, float cruise, float lateral = NAN, bool useFront = true) {
  const WallRefs& ref = Tof::refs();
  lastStopMs = (lastBrakeMs && millis() - lastBrakeMs < 10000) ? millis() - lastBrakeMs : 0;
  event = 0;
  restartClock();
  const float s0 = Enc::distMM();
  const uint32_t t0 = millis();
  const uint32_t timeout = (uint32_t)(mm * 8) + 1500;     // ~3 s for one cell
  float target = mm, integ = 0, vBrake = 0, sBrake = 0;
  // never drive into a wall that is already close in front
  bool wallAhead = false;                                  // was a wall visible at the start?
  if (useFront) {
    const TofReading& F0 = Tof::get(TOF_FRONT);
    wallAhead = F0.valid && F0.mm < ref.front + P.CELL_LEN;
    if (F0.valid && F0.mm < ref.front + 40.0f) target = min(target, max(0.0f, F0.mm - (ref.front + P.ALIGN_OFFSET)));
    if (target < 5.0f) { driveErr = 0; return frontAlign(); }   // already at the wall
  }
  // live centering state (filtered: raw ToF noise would shake the robot)
  float yF = NAN, yOld = NAN, dyF = 0, corrSum = 0; int corrN = 0;
  uint32_t tOld = 0; uint8_t srcPrev = 0; int closeN = 0;

  float aim = 0;
  if (!isnan(lateral) && P.CENTER_GAIN > 0 && mm > 60)
    aim = constrain(-atan2f(lateral * P.CENTER_GAIN, 0.5f * mm) * RAD_TO_DEG, -12.0f, 12.0f);

  bool ok = true;
  recLat = NAN;
  while (true) {
    recPh = 'D';
    if (!tick()) { ok = false; break; }
    uint32_t t = millis() - t0;
    float s = Enc::distMM() - s0;
    float rem = target - s;
    float v = Enc::speed();

    // safety: encoders really counting forward?
    if (t > 800 && s < 10.0f) {
      Serial.printf("[drive] encoders not counting forward (s=%.0f) -> check ENC_SIGN with 'e'\n", s);
      event = 'E'; ok = false; break;
    }
    const TofReading& F = Tof::get(TOF_FRONT);
    if (useFront) {
      // emergency stop: 2 consecutive close samples, and only after we really moved
      // emergency stop = about to TOUCH the wall. Capped at 30 mm so a wrong
      // reference can never stop the robot in the middle of a cell.
      const float stopMM = constrain(ref.front + P.ALIGN_OFFSET - 12.0f, 10.0f, 30.0f);
      // If the front was OPEN at the start, a wall cannot appear 30 mm ahead in the
      // middle of the cell: such readings are a post/side wall seen at an angle
      // (your #16-#18). Then only trust it near the end of the move.
      bool plausible = wallAhead || rem < 80.0f;
      if (F.valid && F.mm < stopMM && s > 25.0f && plausible) {
        if (++closeN >= 3) { Serial.println("[drive] front wall stop"); event = 'F'; vBrake = v; sBrake = s; break; }
      } else closeN = 0;
      if (F.valid && F.mm < ref.front + 100.0f && rem < 0.75f * P.CELL_LEN) {
        float remF = F.mm - (ref.front + P.ALIGN_OFFSET);
        // move the target SMOOTHLY toward what the front wall says (+-6 mm noise)
        if (fabsf(remF - rem) < 60.0f) { target += 0.5f * ((s + remF) - target); rem = target - s; }
      }
    }
    if (rem <= max(v, 0.0f) * P.DRIVE_LEAD) { vBrake = v; sBrake = s; break; }
    if (t > timeout) { Serial.println("[drive] timeout"); event = 'T'; vBrake = v; sBrake = s; break; }

    // START: both wheels get START_PWM at once (same value -> they break free
    // together), then ramp to cruise and fade the heading correction in.
    float base = P.START_PWM + (cruise - P.START_PWM) * min(1.0f, t / max(P.RAMP_MS, 1.0f));
    if (rem < P.SLOW_MM) base = min(base, P.END_PWM + (cruise - P.END_PWM) * rem / P.SLOW_MM);
    float corrGain = min(1.0f, t / max(P.CORR_MS, 1.0f));

    float fade = max(0.0f, 1.0f - s / mm);
    float cwT = headingRef() + aim * fade;                 // + learned corridor trim
    if (P.LIVE_CENTER > 0) {
      uint8_t src = 0;
      float y = liveLateral(&src);
      if (!isnan(y) && src) {
        uint32_t now = millis();
        if (src != srcPrev || isnan(yF) || fabsf(y - yF) > 25.0f) {
          yF = y; dyF = 0; yOld = y; tOld = now;            // restart: wall set changed
        } else {
          yF += P.LIVE_FILT * (y - yF);                     // low-pass the position
        }
        srcPrev = src;
        if (now - tOld >= 150) {                            // lateral SPEED over >=150 ms
          float dy = (yF - yOld) * 1000.0f / (float)(now - tOld);
          dyF = 0.6f * dyF + 0.4f * dy;
          yOld = yF; tOld = now;
        }
        float corr = constrain(-P.LIVE_CENTER * yF, -12.0f, 12.0f)
                   + constrain(-P.LIVE_DAMP * dyF, -4.0f, 4.0f);   // D term kept small
        cwT += corr;
        corrSum += corr; corrN++;                          // for the trim learning
        recLat = yF;
      }
    }
    // blocked (a wheel against a wall): the heading runs away although the
    // controller is already steering at its limit -> stop instead of pushing 3 s
    if (fabsf(Imu::cw() - cwT) > 30.0f) { Serial.println("[drive] heading lost (blocked?) - stop"); event = 'H'; ok = false; break; }
    headingDrive(base, cwT, integ, true, t, corrGain);
  }
  uint32_t brakeAt = millis();
  settleScan(3, 5.0f, 20, 350);             // stop + wall reading in one
  lastBrakeMs = brakeAt;

  float sEnd = Enc::distMM() - s0;
  driveErr = sEnd - target;

  // Move the STEADY part of the centering correction into this direction's trim:
  // next time the robot drives parallel to the corridor by itself, so the lateral
  // error converges to zero instead of settling at an offset.
  // Only from drives that STARTED centred: steering back from an offset (e.g. after
  // a turn) is not a heading error, learning it made all 4 trims -1.2..-3.5 deg.
  bool startCentred = !isnan(lateral) && fabsf(lateral) < 8.0f;
  if (ok && startCentred && P.TRIM_LEARN > 0 && corrN > 120 && sEnd > 100.0f) {
    int d = dir();
    dirTrim[d] = constrain(dirTrim[d] + P.TRIM_LEARN * (corrSum / corrN), -6.0f, 6.0f);
  }
  if (ok && vBrake > 60.0f && P.LEAD_LEARN > 0) {
    float meas = (sEnd - sBrake) / vBrake;
    P.DRIVE_LEAD = constrain(P.DRIVE_LEAD + P.LEAD_LEARN * (meas - P.DRIVE_LEAD), 0.0f, 0.15f);
  }
  return ok;
}

bool driveCells(int n, float cruise, float extraMM = 0, float lateral = NAN) {
  if (n <= 0) return true;
  if (!driveMM(n * P.CELL_LEN + extraMM, cruise, lateral)) return false;
  return frontAlign();
}

// small forward/backward move (front alignment). A KICK first breaks static
// friction (this robot needs ~80+ PWM to start), then ALIGN_PWM while moving.
static bool moveSmall(float mm) {
  restartClock();
  const float s0 = Enc::distMM();
  const uint32_t t0 = millis();
  float integ = 0, dirS = sgn(mm);
  bool ok = true, moving = false;
  while (true) {
    recPh = 'K';
    if (!tick()) { ok = false; break; }
    float done = (Enc::distMM() - s0) * dirS;
    float rem  = fabsf(mm) - done;
    if (rem <= fabsf(Enc::speed()) * P.DRIVE_LEAD + 0.5f) break;
    if (millis() - t0 > 1200) { Serial.println("[align] could not move - raise ALIGN_KICK"); break; }
    if (done > 2.0f) moving = true;                        // it broke free
    float pwm = moving ? P.ALIGN_PWM : P.ALIGN_KICK;
    headingDrive(dirS * pwm, headingRef(), integ, false, 0);
  }
  settleScan(3, 5.0f, 20, 300);
  return ok;
}

bool frontAlign() {
  // no front wall in the live reading -> skip the whole stopped scan (saves ~150 ms)
  const TofReading& F = Tof::get(TOF_FRONT);
  if (!F.valid || F.mm > Tof::refs().front + P.FRONT_MARGIN + 20.0f) return true;
  for (int k = 0; k < 2; k++) {
    WallScan w = scanCached(3);                // usually the one taken while stopping
    if (isnan(w.f) || !w.front) return true;
    float e = w.f - (Tof::refs().front + P.ALIGN_OFFSET);
    if (fabsf(e) <= P.ALIGN_TOL) return true;
    if (!moveSmall(e)) return false;
  }
  return true;
}

// The robot is placed with its BODY in the cell centre; it turns about the axle,
// which is -ALIGN_OFFSET mm behind -> move forward so the AXLE is in the centre.
bool startAtCentre() { return P.ALIGN_OFFSET < -1.0f ? moveSmall(-P.ALIGN_OFFSET) : true; }

// One smooth rotation to an absolute heading: RATE control on a decelerating
// profile that ends in a CONSTANT CRAWL, then a single brake. The PWM never
// reverses, so the robot cannot shake.
static bool rotateTo(float target, bool learn, bool isRetry = false) {
  recLat = NAN;
  const float start = Imu::cw();
  const float dirS  = sgn(target - start);
  const float D     = fabsf(target - start);
  if (D < 0.05f) return true;
  if (!isRetry) event = 0;
  float& tRef  = (dirS > 0) ? P.STOP_T_R : P.STOP_T_L;
  float& cpRef = (dirS > 0) ? P.CREEP_PWM_R : P.CREEP_PWM_L;
  const float Wc = P.TURN_CREEP_DPS;
  const float s0 = Enc::distMM();              // wheel average must stay here (turn in place)
  float remAtBrake = NAN;

  restartClock();
  const uint32_t t0 = millis();
  float wRamp = 0, wF = 0, kick = 0, kickMax = 0, creepSum = 0;
  int   creepN = 0;
  uint32_t creepT0 = 0, slowSince = 0;
  float wBrake = 0, thBrake = 0;
  char  ph = 'A';                              // A accelerate, R brake, B creep
  bool  ok = true, braked = false;

  while (true) {
    if (!tick()) { ok = false; break; }
    float w  = Imu::rateCW() * dirS;           // + = toward the target
    wF += 0.40f * (w - wF);
    float th  = (Imu::cw() - start) * dirS;
    float rem = D - th - wF * P.GYRO_LAG;      // gyro delay compensated (brake decision)
    float remG = D - th;                       // raw gyro (stop rule: STOP_T is learned on it)

    if (th < -20.0f) {
      Serial.println("[turn] WRONG DIRECTION -> check CW_SIGN / SWAP_MOTORS ('i' test)");
      event = 'X'; ok = false; break;
    }
    if (millis() - t0 > 2500) { Serial.println("[turn] timeout"); event = 't'; break; }

    // ---- phase changes ----
    // brake ONCE, early enough that the robot is at creep speed when the creep zone starts
    if (ph == 'A' && rem <= P.TURN_CRAWL_ANG + max(0.0f, wF * wF - Wc * Wc) / (2.0f * P.TURN_BRAKE_A))
      ph = 'R';
    if (ph == 'R' && wF <= 1.2f * Wc) { ph = 'B'; creepT0 = millis(); }
    // STOP: it still rotates STOP_T * speed after the brake -> brake exactly then
    if ((ph == 'B' && remG <= tRef * max(wF, 0.0f)) || remG <= 0.3f) {
      wBrake = wF; thBrake = th; braked = true; remAtBrake = remG; break;
    }

    float pwm;
    if (ph == 'A') {                           // accelerate / cruise (no brake pulses)
      wRamp = min(P.TURN_MAX_DPS, wRamp + P.TURN_ACC * dtS);
      pwm = P.TURN_MIN_PWM + P.TURN_FF * wRamp + P.TURN_KP * (wRamp - w);
    } else if (ph == 'R') {                    // one short brake
      pwm = 0;
    } else {                                   // creep: fixed PWM (no feedback hunting)
      if (wF < 0.5f * Wc) { if (!slowSince) slowSince = millis(); } else slowSince = 0;
      if (slowSince && millis() - slowSince > 60) kick = min(kick + 300.0f * dtS, 40.0f);  // stuck
      else if (wF > 0.8f * Wc)                    kick = max(0.0f, kick - 300.0f * dtS);
      kickMax = max(kickMax, kick);
      pwm = cpRef + kick;
      if (wF > 1.6f * Wc) pwm = 0;             // still far too fast -> brake a little more
      if (millis() - creepT0 > 40) { creepSum += wF; creepN++; }
    }
    recPh = ph;
    pwm = constrain(pwm, 0.0f, P.TURN_MAX_PWM);

    // TURN IN PLACE: push the wheel average back to where the turn started.
    // Also during the brake: your recording showed the left-turn slide happens THERE
    // (+1.6 mm). A small common PWM on a braked motor is still ~90% short-brake.
    float cm = constrain(-(P.CM_KP * (Enc::distMM() - s0) + P.CM_KD * Enc::speed()), -40.0f, 40.0f);
    if (pwm <= 0) cm = constrain(cm, -25.0f, 25.0f);
    // stuck creep: the kick must turn BOTH wheels. A big CM term took the forward
    // wheel under its friction (your timeout: pwm L -112 / R +51, stuck at -76 deg)
    if (kick > 0) cm = constrain(cm, -10.0f, 10.0f);
    setLeft((int)lroundf(dirS * pwm + cm));    // + dirS = clockwise
    setRight((int)lroundf(-dirS * pwm + cm));
  }
  // stop + read the side walls of the NEW corridor at the same time: the next drive
  // starts already steering toward the centre (your log: after right turns the robot
  // was 12-18 mm off-centre at the end of the next cell)
  settleScan(2, 5.0f, 25, 300);

  // rotation after the brake, and its time constant (coast / speed)
  float coast = (Imu::cw() - start) * dirS - thBrake;
  float tObs  = wBrake > 1.0f ? coast / wBrake : NAN;
  bool  normal = learn && ok && braked && ph == 'B' && D > 30.0f;
  // not from KICK turns: a stuck-then-kicked creep coasts much less (your log: 0.020 s)
  // and pulled the right STOP_T from 0.056 to 0.040 -> next turns overshot 2.6 deg
  if (normal && kickMax <= 5.0f && wBrake > 30.0f && P.TURN_LEARN > 0)
    tRef = constrain(tRef + constrain(P.TURN_LEARN * (tObs - tRef), -0.01f, 0.01f), 0.02f, 0.12f);
  // creep PWM -> creep at TURN_CREEP_DPS. If it got stuck (kick used) it was too
  // weak: raise it, and do NOT learn from the stop speed (the kick inflated it -
  // that is how the old learning walked down to 41 PWM and stalled).
  float creepAvg = creepN ? creepSum / creepN : NAN;
  if (normal && P.CREEP_LEARN > 0) {
    if (kickMax > 5.0f)          cpRef += 2.0f;
    else if (remAtBrake > 0.3f)  cpRef += P.CREEP_LEARN * (Wc - wBrake);
    cpRef = constrain(cpRef, P.CREEP_MIN_PWM, 120.0f);
  }
  const float slide = Enc::distMM() - s0;
  if (P.TRACE > 0)
    Serial.printf("   [%s %s] D %.1f  stop@ w %.0f rem %.2f  coast %.2f = %.3f s (learned %.3f)  creep %.0f dps @%.0f%s  slide %+.1f mm  -> err %+.2f\n",
                  dirS > 0 ? "R" : "L", isRetry ? "retry" : "main ", D, wBrake, remAtBrake, coast,
                  tObs, tRef, creepAvg, cpRef, kickMax > 5.0f ? " KICK" : "", slide, target - Imu::cw());
  return ok;
}

bool turn(int q) {
  quarterTurns += q;
  const float target = headingTarget();
  bool ok = rotateTo(target, true);
  // a wheel rubbing a wall can spoil one turn: repeat it once, with the SAME
  // smooth controller (no bursts). Costs ~150 ms and only when really needed.
  if (ok && fabsf(target - Imu::cw()) > P.TURN_RETRY) ok = rotateTo(target, false, true);
  turnErr = target - Imu::cw();
  return ok;
}

bool turnToDir(int d) {
  int k = ((d - dir()) % 4 + 4) % 4;
  if (k == 0) return true;
  if (k == 1) return turn(+1);
  if (k == 3) return turn(-1);
  return turn(Tof::wallRight() ? -2 : +2);
}
}

