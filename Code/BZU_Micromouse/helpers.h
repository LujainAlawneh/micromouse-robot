// =====================================================================
//  HELPERS: LED, blink, key check, idle
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

static void led(bool on) { digitalWrite(PIN_LED, on); }
static void blink(int n, int ms = 120) { for (int i = 0; i < n; i++) { led(1); delay(ms); led(0); delay(ms); } }
static void drain() { delay(80); while (Serial.available()) Serial.read(); }   // eat the Enter/newline
static bool key() { if (Serial.available() && serialKeyHit()) { drain(); return true; } return false; }
static void idleFor(uint32_t ms) { Motion::restartClock(); uint32_t t = millis(); while (millis() - t < ms) Motion::idleTick(); }

