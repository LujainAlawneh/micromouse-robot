// =====================================================================
//  AUTO RUN: battery start, countdown, hand wave
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// ---------------------------------------------------------------------
//  AUTO RUN helpers (battery, no USB)
// ---------------------------------------------------------------------
// LED blinks (faster in the last second). false = cancelled by a key or BOOT.
static bool countdown(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    uint32_t left = ms - (millis() - t0);
    led((millis() / (left < 1000 ? 80 : 250)) % 2);
    Motion::idleTick();
    if (key() || digitalRead(PIN_BUTTON) == LOW) {
      while (digitalRead(PIN_BUTTON) == LOW) delay(5);
      led(0);
      Serial.println("cancelled");
      return false;
    }
  }
  led(0);
  return true;
}

// start without touching the robot: hold a hand ~1 cm in front of the FRONT
// sensor (< 25 mm) for 0.3..4 s, then take it away (> 40 mm or nothing seen).
static bool handWave() {
  static uint32_t inSince = 0, lastIn = 0, outSince = 0;
  static bool armed = false;
  const TofReading& F = Tof::get(TOF_FRONT);
  const uint32_t now = millis();
  if (F.valid && F.mm < 25) {
    if (!inSince) inSince = now;
    lastIn = now; outSince = 0;
    armed = now - inSince > 300 && now - inSince < 4000;   // longer = a wall, not a hand
    return false;
  }
  inSince = 0;
  if (!armed) return false;
  if (now - lastIn > 1500) { armed = false; return false; }
  if (!F.valid || F.mm > 40) {
    if (!outSince) outSince = now;
    if (now - outSince > 200) { armed = false; outSince = 0; return true; }
  } else outSince = 0;
  return false;
}

static void autoRun() {
  const bool wf = AUTO_RUN_CMD == 'w' || AUTO_RUN_CMD == 'W';
  // on the desk (USB plugged in to read the log) no wall is seen -> don't start
  // and don't overwrite the saved log of the last maze run
  if (wf && !Tof::wallLeft() && !Tof::wallFront() && !Tof::wallRight()) {
    Serial.println("no wall around the robot -> not in the maze, auto run skipped");
    blink(3, 150);
    return;
  }
  Serial.printf("\n>>> AUTO RUN '%c'\n", AUTO_RUN_CMD);
  if (AUTO_RUN_CMD == 'g') solveRun(false);
  else if (AUTO_RUN_CMD == 'n') fullRun(true, false);
  else exec(AUTO_RUN_CMD);
  while (digitalRead(PIN_BUTTON) == LOW) delay(5);   // the BOOT press that stopped it is not a new start
  delay(50);
  if (wf) {
    if (runLog.end == 'D') blink(2, 400);              // 2 slow = finished all cells
    else                   blink(8, 70);               // fast  = stopped early ('L' says why)
  }
  Serial.println("ready: BOOT or hand wave = run again");
}

