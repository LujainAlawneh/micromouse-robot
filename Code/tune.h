// =====================================================================
//  TUNE: live parameter editing + flash save
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// =====================================================================
//  PARAMETERS (TUNE namespace)
// =====================================================================
namespace Tune {
struct Entry { const char* name; float* v; float step; };
Entry table[] = {
  {"KP", &P.KP, 0.5f}, {"KD", &P.KD, 0.05f}, {"KI", &P.KI, 0.1f},
  {"MM_PER_CNT", &P.MM_PER_CNT, 0.002f}, {"CELL_LEN", &P.CELL_LEN, 1},
  {"MAX_CORR", &P.MAX_CORR, 5}, {"CRUISE_PWM", &P.CRUISE_PWM, 5},
  {"RUN_PWM", &P.RUN_PWM, 5}, {"END_PWM", &P.END_PWM, 2}, {"RAMP_MS", &P.RAMP_MS, 25},
  {"RIGHT_TRIM", &P.RIGHT_TRIM, 1}, {"SLOW_MM", &P.SLOW_MM, 5},
  {"START_PWM", &P.START_PWM, 5}, {"CORR_MS", &P.CORR_MS, 25},
  {"DRIVE_LEAD", &P.DRIVE_LEAD, 0.002f},
  {"CENTER_GAIN", &P.CENTER_GAIN, 0.1f}, {"LIVE_CENTER", &P.LIVE_CENTER, 0.02f},
  {"LIVE_FILT", &P.LIVE_FILT, 0.01f}, {"LIVE_DAMP", &P.LIVE_DAMP, 0.02f},
  {"TRIM_LEARN", &P.TRIM_LEARN, 0.1f},
  {"MAZE_SIZE", &P.MAZE_SIZE, 1}, {"START_RIGHT", &P.START_RIGHT, 1}, {"CAL_MM", &P.CAL_MM, 50},
  {"TURN_MAX_DPS", &P.TURN_MAX_DPS, 20}, {"TURN_ACC", &P.TURN_ACC, 100},
  {"TURN_BRAKE_A", &P.TURN_BRAKE_A, 100}, {"TURN_CREEP_DPS", &P.TURN_CREEP_DPS, 5},
  {"TURN_CRAWL_ANG", &P.TURN_CRAWL_ANG, 1},
  {"TURN_MIN_PWM", &P.TURN_MIN_PWM, 2}, {"TURN_MAX_PWM", &P.TURN_MAX_PWM, 10},
  {"TURN_RETRY", &P.TURN_RETRY, 0.5f},
  {"TURN_LEARN", &P.TURN_LEARN, 0.05f}, {"TRACE", &P.TRACE, 1},
  {"TURN_FF", &P.TURN_FF, 0.02f}, {"TURN_KP", &P.TURN_KP, 0.05f},
  {"CREEP_PWM_R", &P.CREEP_PWM_R, 2}, {"CREEP_PWM_L", &P.CREEP_PWM_L, 2},
  {"CREEP_LEARN", &P.CREEP_LEARN, 0.02f}, {"CM_KP", &P.CM_KP, 0.5f}, {"CM_KD", &P.CM_KD, 0.02f},
  {"STOP_T_R", &P.STOP_T_R, 0.002f}, {"CREEP_MIN_PWM", &P.CREEP_MIN_PWM, 2},
  {"STOP_T_L", &P.STOP_T_L, 0.002f}, {"GYRO_LAG", &P.GYRO_LAG, 0.002f},
  {"LEAD_LEARN", &P.LEAD_LEARN, 0.05f},
  {"ALIGN_PWM", &P.ALIGN_PWM, 2}, {"ALIGN_KICK", &P.ALIGN_KICK, 5},
  {"ALIGN_TOL", &P.ALIGN_TOL, 0.5f}, {"ALIGN_OFFSET", &P.ALIGN_OFFSET, 1},
  {"GYRO_SCALE", &P.GYRO_SCALE, 0.002f},
  {"SIDE_MARGIN", &P.SIDE_MARGIN, 5}, {"FRONT_MARGIN", &P.FRONT_MARGIN, 5},
  {"SIDE_WINDOW", &P.SIDE_WINDOW, 5},
};
constexpr int COUNT = sizeof(table) / sizeof(table[0]);
int sel = 0;
constexpr uint32_t VERSION = 20;   // bumped: old saved params are ignored

void show(int i) { Serial.printf("[%2d] %-13s = %.4g   (step %.3g)\n", i, table[i].name, *table[i].v, table[i].step); }

void load() {
  Preferences p;
  p.begin("params", true);
  if (p.getUInt("ver", 0) == VERSION && p.getBytesLength("P") == sizeof(Params)) {
    p.getBytes("P", &P, sizeof(Params));
    Serial.println("[params] loaded from flash");
  } else {
    Serial.println("[params] defaults");
  }
  p.end();
}

void save() {
  Preferences p;
  p.begin("params", false);
  p.putUInt("ver", VERSION);
  p.putBytes("P", &P, sizeof(Params));
  p.end();
  Serial.println("[params] saved");
}

void defaults() { P = Params(); Serial.println("[params] factory defaults (not saved)"); }

void list() {
  for (int i = 0; i < COUNT; i++) show(i);
  Serial.printf("wall refs (from 'c'): L %.1f  F %.1f  R %.1f mm\n",
                Tof::refs().left, Tof::refs().front, Tof::refs().right);
  Serial.print("selected: "); show(sel);
}

void select(int step) { sel = (sel + step + COUNT) % COUNT; show(sel); }

void adjust(int sign) { *table[sel].v += sign * table[sel].step; show(sel); }
}

