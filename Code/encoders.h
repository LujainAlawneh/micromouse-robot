// =====================================================================
//  ENCODERS: wheel counts and speed
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// =====================================================================
//  ENCODERS
// =====================================================================
volatile long leftCount = 0, rightCount = 0;

void IRAM_ATTR leftEncoder() {
  if (digitalRead(LEFT_C1) == digitalRead(LEFT_C2)) leftCount++; else leftCount--;
}
void IRAM_ATTR rightEncoder() {
  if (digitalRead(RIGHT_C1) == digitalRead(RIGHT_C2)) rightCount--; else rightCount++;
}

namespace Enc {
constexpr int WIN = 8;
float    hist[WIN];
uint32_t histT[WIN];
int      idx = 0;
float    v = 0;

void begin() {
  pinMode(LEFT_C1, INPUT_PULLUP);  pinMode(LEFT_C2, INPUT_PULLUP);
  pinMode(RIGHT_C1, INPUT);        pinMode(RIGHT_C2, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_C1), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_C1), rightEncoder, CHANGE);
  for (int i = 0; i < WIN; i++) { hist[i] = 0; histT[i] = micros(); }
}

long countL() { return ENC_SIGN_L * leftCount; }
long countR() { return ENC_SIGN_R * rightCount; }
float countsAvg() { return 0.5f * (float)(countL() + countR()); }
float distMM() { return countsAvg() * P.MM_PER_CNT; }

void update() {
  uint32_t now = micros();
  float d = distMM();
  int old = idx;
  float dt = (now - histT[old]) * 1e-6f;
  if (dt > 0.001f) v = (d - hist[old]) / dt;
  hist[idx] = d; histT[idx] = now;
  idx = (idx + 1) % WIN;
}

float speed() { return v; }
}

