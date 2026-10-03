// =====================================================================
//  BZU Micromouse v2 — Consolidated .ino (as-built hardware)
//  Single-file consolidation from modular structure with edits applied:
//  - CW_SIGN (gyro direction) + SWAP_MOTORS (left/right channels)
//  - Turns: trapezoid angle profile + PD tracking (smooth, no brake/nudge)
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_BNO055.h>
// Code is split into headers. The ORDER of the includes below matters
// (each file uses things defined in the files above it).
#include "config.h"
#include "params.h"
#include "motors.h"
#include "encoders.h"
#include "imu.h"
#include "tof.h"
#include "maze.h"
#include "tune.h"
#include "motion.h"
#include "helpers.h"
#include "runs.h"
#include "diagnostics.h"
#include "runlog.h"
#include "wallfollow.h"
#include "commands.h"
#include "autorun.h"

void setup() {
  Serial.setTxBufferSize(4096);             // prints no longer block the robot
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_LED, OUTPUT);
  motorsBegin();
  Enc::begin();
  Tune::load();
  bool tofOk = Tof::begin();
  bool imuOk = Imu::begin();
  Motion::begin();
  Maze::reset();
  if (!tofOk || !imuOk) { Serial.println("!!! INIT PROBLEM - see above"); blink(10, 50); }
  help();
  if (runLogLoad()) { runLogSummary(); Serial.println("   'L' = cell-by-cell log of that run"); }

  if (AUTO_RUN) {
    Serial.printf("AUTO_RUN '%c' in %lu ms  (any key / BOOT = cancel)\n",
                  AUTO_RUN_CMD, (unsigned long)AUTO_RUN_DELAY_MS);
    if (countdown(AUTO_RUN_DELAY_MS)) autoRun();
  }
}

void loop() {
  Motion::idleTick();
  bool btn = digitalRead(PIN_BUTTON) == LOW;
  if (btn || (AUTO_RUN && handWave())) {
    while (digitalRead(PIN_BUTTON) == LOW) delay(5);
    Motion::clearAbort();
    if (AUTO_RUN) {
      Serial.printf("start '%c' in %lu ms  (any key / BOOT = cancel)\n",
                    AUTO_RUN_CMD, (unsigned long)RESTART_DELAY_MS);
      if (countdown(RESTART_DELAY_MS)) autoRun();
    } else {
      solveRun(false);
    }
    return;
  }
  if (AUTO_RUN) led(millis() % 2000 < 40);          // short blip = waiting for BOOT / hand
  if (Serial.available()) {
    int c = Serial.read();
    if (c >= 0x20 && c < 0x7F) { drain(); exec((char)c); }   // ignore garbage bytes on battery
  }
}
