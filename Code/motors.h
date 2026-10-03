// =====================================================================
//  MOTORS: TB6612FNG driver
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// =====================================================================
//  MOTORS
// =====================================================================
void motorsBegin() {
  for (int p : {AIN1, AIN2, BIN1, BIN2, PWMA, PWMB}) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  digitalWrite(AIN1, HIGH);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 0);      analogWrite(PWMB, 0);
}

static void driveA(int pwm) {        // + = forward
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else          { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); pwm = -pwm; }
  analogWrite(PWMA, pwm);
}

static void driveB(int pwm) {        // + = forward (mirrored motor)
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  else          { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); pwm = -pwm; }
  analogWrite(PWMB, pwm);
}

int lastPwmL = 0, lastPwmR = 0;      // last commanded values (recorder)
void setLeft(int pwm)  { lastPwmL = constrain(pwm, -255, 255); if (SWAP_MOTORS) driveB(pwm); else driveA(pwm); }
void setRight(int pwm) { lastPwmR = constrain(pwm, -255, 255); if (SWAP_MOTORS) driveA(pwm); else driveB(pwm); }

void brakeMotors() {
  lastPwmL = lastPwmR = 0;
  analogWrite(PWMA, 0);      analogWrite(PWMB, 0);
  digitalWrite(AIN1, HIGH);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);  digitalWrite(BIN2, HIGH);
}

