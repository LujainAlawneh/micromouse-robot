// =====================================================================
//  WALL FOLLOW test ('w' / 'W')
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// ---------------------------------------------------------------------
//  'w' / 'W' wall-follow test (NO maze algorithm): sensing + straight +
//  turns + centering. 'w' = right-hand rule (right > straight > left > back)
//  'W' = straight first (straight > right > left > back)
// ---------------------------------------------------------------------

static void wallFollow(bool straightFirst, int maxCells = 40) {
  Serial.println("WALL FOLLOW: robot in a cell CENTER. any key / BOOT = stop");
  idleFor(1000);
  drain();
  Imu::measureBias();
  Motion::resetHeading();
  Motion::clearAbort();
  float sumLat = 0, sqTurn = 0; int nLat = 0, nTurn = 0, cells = 0;
  uint32_t stopSum = 0, tRun0 = millis(); int stopN = 0;
  memset(&runLog, 0, sizeof(runLog));
  runLog.magic = RUNLOG_MAGIC;
  runLog.mode  = straightFirst ? 'W' : 'w';
  runLog.end   = 'D';
  led(1);
  Motion::startAtCentre();                         // body-centred -> axle-centred
  while (cells < maxCells) {
    WallScan w = Motion::scanCached(3);
    int q; const char* act;
    if (straightFirst && !w.front) { q = 0;  act = "STRAIGHT"; }
    else if (!w.right)             { q = +1; act = "RIGHT"; }
    else if (!w.front)             { q = 0;  act = "STRAIGHT"; }
    else if (!w.left)              { q = -1; act = "LEFT"; }
    else                           { q = +2; act = "BACK"; }
    Serial.printf("#%2d  L%d F%d R%d  (%.0f / %.0f / %.0f mm)  lat %+.1f  -> %s\n",
                  cells, w.left, w.front, w.right, w.l, w.f, w.r, w.lateral, act);
    if (!isnan(w.lateral)) { sumLat += fabsf(w.lateral); nLat++; }

    CellLog* c = cells < RUNLOG_MAX ? &runLog.c[cells] : nullptr;
    if (c) {
      c->l = mm8(w.l); c->f = mm8(w.f); c->r = mm8(w.r);
      c->walls = (w.left ? 1 : 0) | (w.front ? 2 : 0) | (w.right ? 4 : 0);
      c->act = act[0]; c->lat = s8(w.lateral); c->latT = -128;
      runLog.n = cells + 1;
    }

    float lat = w.lateral;
    if (q != 0) {
      bool okT = Motion::turn(q);
      float e = Motion::lastTurnError();
      if (c) { c->turnErr = s8(e * 10.0f); c->ev = Motion::lastEvent(); }
      if (!okT) { runLog.end = Motion::lastEvent() ? Motion::lastEvent() : 'U'; break; }
      sqTurn += e * e; nTurn++;
      lat = Motion::freshLateral();                // read while the turn settled (no extra stop)
      if (c) c->latT = s8(lat);
      Serial.printf("     turn err %+.2f deg   lat after turn %+.1f mm\n", e, lat);
    }
    bool okD = Motion::driveCells(1, P.CRUISE_PWM, 0, lat);
    uint32_t st = Motion::lastStopTime();
    if (c) {
      c->driveErr = s8(Motion::lastDriveError());
      c->stopMs = (uint16_t)min(st, (uint32_t)65535);
      if (Motion::lastEvent()) c->ev = Motion::lastEvent();
    }
    if (!okD) { runLog.end = Motion::lastEvent() ? Motion::lastEvent() : 'U'; break; }
    if (cells > 0 && st) { stopSum += st; stopN++; }
    Serial.printf("     drive err %+.1f mm   (stopped %lu ms before it)\n", Motion::lastDriveError(), (unsigned long)st);
    cells++;
  }
  brakeMotors(); led(0);
  float tRun = (millis() - tRun0) / 1000.0f;
  runLog.meanLat = nLat ? sumLat / nLat : 0.0f;
  runLog.turnRms = nTurn ? sqrtf(sqTurn / nTurn) : 0.0f;
  runLog.timeS   = tRun;
  runLog.done    = (uint8_t)min(cells, 255);
  runLog.avgStop = (uint16_t)(stopN ? stopSum / stopN : 0);
  for (int i = 0; i < 4; i++) runLog.trims[i] = Motion::trim(i);
  Serial.printf("== %d cells | mean |lat| %.1f mm | turn RMS %.2f deg | %s ==\n",
                cells, runLog.meanLat, runLog.turnRms, endText(runLog.end));
  Serial.printf("   time %.1f s = %.2f s per cell | average stop between moves %lu ms\n",
                tRun, cells ? tRun / cells : 0.0f, (unsigned long)runLog.avgStop);
  Serial.printf("   heading trims N %+.2f E %+.2f S %+.2f W %+.2f deg  (learned)\n",
                Motion::trim(0), Motion::trim(1), Motion::trim(2), Motion::trim(3));
  runLogSave();                                    // robot is standing: flash write is safe now
  Serial.println("   (saved in flash -> 'L' prints it again, also after a power cycle)");
}

