// =====================================================================
//  DIAGNOSTICS: turn tests, calibration, streams, recorder
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

static void turnSeries(const int* seq, int n) {
  Motion::resetHeading();
  idleFor(300);
  float sq = 0; int k = 0;
  uint32_t tt = 0;
  for (int i = 0; i < n; i++) {
    uint32_t t0 = millis();
    if (!Motion::turn(seq[i])) break;
    tt += millis() - t0;
    float e = Motion::lastTurnError();
    sq += e * e; k++;
    Serial.printf("%+4d -> err %+.2f  (%lu ms)\n", seq[i] * 90, e, (unsigned long)(millis() - t0));
    idleFor(250);
  }
  if (k) Serial.printf("RMS %.2f deg over %d turns, %lu ms/turn\n", sqrtf(sq / k), k, (unsigned long)(tt / k));
}

static void calibrate() {
  Serial.println("Robot EXACTLY centered, walls LEFT/FRONT/RIGHT, facing front wall. 2 s...");
  idleFor(2000);
  float s[3] = {0, 0, 0}; int n[3] = {0, 0, 0};
  for (int rep = 0; rep < 8; rep++) {
    WallScan w = Motion::scan(5);
    if (!isnan(w.l)) { s[0] += w.l; n[0]++; }
    if (!isnan(w.f)) { s[1] += w.f; n[1]++; }
    if (!isnan(w.r)) { s[2] += w.r; n[2]++; }
  }
  // A centred robot is ~half a cell from each wall. Anything bigger means that
  // wall was NOT there (or the robot was not centred) -> refuse to save it,
  // otherwise every front/side behaviour aims at the wrong distance.
  const float maxRef = 0.55f * P.CELL_LEN;              // 110 mm for a 200 mm cell
  float v[3] = { n[0] >= 4 ? s[0] / n[0] : NAN,
                 n[1] >= 4 ? s[1] / n[1] : NAN,
                 n[2] >= 4 ? s[2] / n[2] : NAN };
  const char* nm[3] = {"LEFT", "FRONT", "RIGHT"};
  bool bad = false;
  for (int i = 0; i < 3; i++)
    if (isnan(v[i]) || v[i] < 10.0f || v[i] > maxRef) {
      Serial.printf("  %s = %.1f mm -> NOT plausible (must be 10..%.0f). Wall missing / not centred?\n",
                    nm[i], v[i], maxRef);
      bad = true;
    }
  if (fabsf(v[0] - v[2]) > 25.0f) {
    Serial.printf("  LEFT %.1f vs RIGHT %.1f differ by > 25 mm -> robot not centred?\n", v[0], v[2]);
    bad = true;
  }
  if (bad) { Serial.println("refs NOT saved - fix the placement and run 'c' again"); return; }
  WallRefs& r = Tof::refs();
  r.left = v[0]; r.front = v[1]; r.right = v[2];
  Tof::saveRefs();
  Serial.printf("refs saved: L %.1f  F %.1f  R %.1f\n", r.left, r.front, r.right);
}

static void streamTof() {
  while (!key()) {
    idleFor(100);
    const TofReading &L = Tof::get(TOF_LEFT), &F = Tof::get(TOF_FRONT), &R = Tof::get(TOF_RIGHT);
    Serial.printf("L %3.0f%s%c  F %3.0f%s%c  R %3.0f%s%c   err %lu/%lu/%lu\n",
      L.mm, L.valid ? " " : "x", Tof::wallLeft() ? '*' : ' ',
      F.mm, F.valid ? " " : "x", Tof::wallFront() ? '*' : ' ',
      R.mm, R.valid ? " " : "x", Tof::wallRight() ? '*' : ' ',
      (unsigned long)L.errors, (unsigned long)F.errors, (unsigned long)R.errors);
  }
}

static void streamImu() {
  Serial.println("rotate CLOCKWISE by hand -> cw must increase");
  while (!key()) { idleFor(100); Serial.printf("cw %8.2f  rate %7.1f  bias %.3f\n", Imu::cw(), Imu::rateCW(), Imu::bias()); }
}

static void streamEnc() {
  long l0 = Enc::countL(), r0 = Enc::countR();
  Serial.println("push forward -> BOTH must be POSITIVE and roughly equal ('k' = scale)");
  while (!key()) { idleFor(100); Serial.printf("L %ld  R %ld\n", Enc::countL() - l0, Enc::countR() - r0); }
}

// read a number typed on the Serial monitor (Enter ends it); NAN = nothing typed
static float readNumber(uint32_t timeoutMs = 30000) {
  String s = "";
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    Motion::idleTick();
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') { if (s.length()) return s.toFloat(); }
      else if ((c >= '0' && c <= '9') || c == '.') s += c;
      t0 = millis();
    }
  }
  return NAN;
}

// Distance calibration: the robot DRIVES (no hand pushing - pushing by hand slips
// and gives a wrong scale), you measure the real distance and type it in.
static void distScaleTest() {
  Serial.printf("The robot will drive %.0f mm of FREE SPACE (not inside the maze). 2 s...\n", P.CAL_MM);
  idleFor(2000);
  drain();                                     // the Enter of the command would abort the drive
  Motion::resetHeading();
  Motion::clearAbort();
  if (!Motion::driveMM(P.CAL_MM, P.CRUISE_PWM, NAN, false)) { Serial.println("aborted"); return; }
  float believed = P.CAL_MM + Motion::lastDriveError();
  Serial.printf("robot thinks it drove %.1f mm\n", believed);
  Serial.print("measure the REAL distance, type it in mm and press Enter: ");
  float meas = readNumber();
  if (isnan(meas) || meas < 50) { Serial.println("\ncancelled"); return; }
  float k = meas / believed;
  if (k < 0.5f || k > 2.0f) { Serial.printf("\nratio %.2f looks wrong - not applied\n", k); return; }
  P.MM_PER_CNT *= k;
  Serial.printf("\nmeasured %.1f mm -> MM_PER_CNT = %.4f  (x%.3f).  press 'S' to save\n",
                meas, P.MM_PER_CNT, k);
  Serial.printf("also set CELL_LEN (now %.0f mm) to your measured cell pitch\n", P.CELL_LEN);
}

static void gyroScaleTest() {
  // Accuracy matters: 1 % scale error = ~1 deg per 90-deg turn, always the same way
  // -> the heading slowly rotates and the robot drifts to one side in EVERY direction.
  Serial.println("GYRO SCALE: put a mark on the floor under the robot's nose.");
  Serial.println("Rotate it by hand CLOCKWISE exactly N full turns (3 is good), back to the mark,");
  Serial.println("then type N and press Enter.");
  drain();
  float a0 = Imu::cw();
  float n = readNumber(120000);
  if (isnan(n) || n < 1 || n > 20) { Serial.println("cancelled"); return; }
  float m = Imu::cw() - a0;
  float truth = 360.0f * n;
  if (fabsf(m) < 0.5f * truth) { Serial.printf("measured only %.1f deg - rotate CLOCKWISE, try again\n", m); return; }
  float sug = P.GYRO_SCALE * truth / fabsf(m);
  Serial.printf("measured %.1f deg for %.0f deg -> error %+.2f %%\n", m, truth, 100.0f * (fabsf(m) - truth) / truth);
  Serial.printf("GYRO_SCALE %.4f -> %.4f  (applied, press 'S' to save)\n", P.GYRO_SCALE, sug);
  P.GYRO_SCALE = sug;
}

// print the recorded samples as CSV (paste them to Claude)
static void recDump(const char* title) {
  int n = Motion::recCount();
  Serial.printf("=== REC %s | %d samples, 6 ms apart | GYRO_LAG %.3f CREEP_R %.0f CREEP_L %.0f STOP_T_R %.4f STOP_T_L %.4f ===\n",
                title, n, P.GYRO_LAG, P.CREEP_PWM_R, P.CREEP_PWM_L, P.STOP_T_R, P.STOP_T_L);
  Serial.println("t,ph,cw,rate,dist,spd,pwmL,pwmR,front,lat");
  for (int i = 0; i < n; i++) {
    const Motion::RecS& r = Motion::recGet(i);
    Serial.printf("%u,%c,%.2f,%.0f,%.1f,%.0f,%d,%d,%.0f,%.1f\n",
                  r.t, r.ph, r.cw, r.rate, r.dist, r.spd, r.pl, r.pr, r.front, r.lat);
  }
  Serial.println("=== END ===");
}

// recorded single moves for diagnosis
static void recTurn(int q) {
  Motion::resetHeading(); idleFor(400);
  Motion::recStart();
  Motion::turn(q);
  idleFor(150);                                 // keep recording a bit after the stop
  Motion::recStop();
  char t[40]; snprintf(t, sizeof(t), "turn %+d  err %+.2f", q * 90, Motion::lastTurnError());
  recDump(t);
}
static void recDrive() {
  Motion::resetHeading(); idleFor(400);
  float lat = Motion::scan(4).lateral;
  Motion::recStart();
  Motion::driveCells(1, P.CRUISE_PWM, 0, lat);
  Motion::recStop();
  char t[48]; snprintf(t, sizeof(t), "drive 1 cell  err %+.1f mm  lat0 %.1f", Motion::lastDriveError(), lat);
  recDump(t);
}

