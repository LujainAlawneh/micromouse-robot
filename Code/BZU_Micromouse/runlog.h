// =====================================================================
//  RUN LOG: result of each run saved in flash (read with 'L')
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// ---- run log in FLASH: a battery run (no USB) can be read later with 'L' ----
struct CellLog {
  uint8_t  l, f, r;          // wall distances mm (255 = nothing seen)
  uint8_t  walls;            // bit0 L, bit1 F, bit2 R
  char     act;              // S R L B
  char     ev;               // Motion event of this cell (0 = normal)
  int8_t   lat;              // lateral mm at the scan (-128 = unknown)
  int8_t   turnErr;          // deg x10 (0 = no turn)
  int8_t   driveErr;         // mm
  int8_t   latT;             // lateral mm right after the turn (-128 = unknown)
  uint16_t stopMs;           // standing still before this drive
};
constexpr int      RUNLOG_MAX   = 40;
constexpr uint32_t RUNLOG_MAGIC = 0x57464C32;          // "WFL2"
struct RunLog {
  uint32_t magic;
  uint8_t  n;                // cells logged
  char     mode;             // 'w' / 'W'
  char     end;              // D done, U user stop, or the event that stopped it
  uint8_t  done;             // cells completed
  float    meanLat, turnRms, timeS, trims[4];
  uint16_t avgStop, pad2;
  CellLog  c[RUNLOG_MAX];
};
static RunLog runLog;

static uint8_t mm8(float v) { return isnan(v) ? 255 : (uint8_t)constrain(lroundf(v), 0L, 254L); }
static int8_t  s8(float v)  { return isnan(v) ? -128 : (int8_t)constrain(lroundf(v), -127L, 127L); }

static void runLogSave() {
  Preferences p;
  p.begin("runlog", false);
  p.putBytes("L", &runLog, sizeof(runLog));
  p.end();
}

static bool runLogLoad() {
  Preferences p;
  p.begin("runlog", true);
  bool ok = p.getBytesLength("L") == sizeof(runLog) && p.getBytes("L", &runLog, sizeof(runLog)) == sizeof(runLog);
  p.end();
  return ok && runLog.magic == RUNLOG_MAGIC;
}

static const char* endText(char e) {
  switch (e) {
    case 'D': return "finished all cells";
    case 'U': return "stopped by BOOT / key";
    case 'F': return "front-wall emergency stop";
    case 'T': return "drive timeout";
    case 'H': return "heading lost (blocked?)";
    case 'E': return "encoders not counting";
    case 't': return "turn timeout";
    case 'X': return "turn wrong direction";
    default:  return "stopped";
  }
}

static void runLogSummary() {
  Serial.printf("== LAST '%c' RUN (flash): %d cells | mean |lat| %.1f mm | turn RMS %.2f deg | %s ==\n",
                runLog.mode, runLog.done, runLog.meanLat, runLog.turnRms, endText(runLog.end));
  Serial.printf("   time %.1f s = %.2f s per cell | average stop between moves %u ms\n",
                runLog.timeS, runLog.done ? runLog.timeS / runLog.done : 0.0f, runLog.avgStop);
}

static void runLogPrint() {
  if (!runLogLoad()) { Serial.println("no run saved yet"); return; }
  static const char* ACT[4] = {"STRAIGHT", "RIGHT", "LEFT", "BACK"};
  for (int i = 0; i < runLog.n && i < RUNLOG_MAX; i++) {
    const CellLog& c = runLog.c[i];
    int a = c.act == 'R' ? 1 : c.act == 'L' ? 2 : c.act == 'B' ? 3 : 0;
    char lat[10];
    if (c.lat == -128) snprintf(lat, sizeof(lat), "  -"); else snprintf(lat, sizeof(lat), "%+d", c.lat);
    Serial.printf("#%2d  L%d F%d R%d  (%d / %d / %d mm)  lat %s  -> %s", i,
                  c.walls & 1, (c.walls >> 1) & 1, (c.walls >> 2) & 1, c.l, c.f, c.r, lat, ACT[a]);
    if (a) {
      Serial.printf("  turn %+.1f deg", c.turnErr / 10.0f);
      if (c.latT != -128) Serial.printf(" lat after %+d", c.latT);
    }
    Serial.printf("  drive %+d mm  stop %u ms", c.driveErr, c.stopMs);
    if (c.ev) Serial.printf("  [%s]", endText(c.ev));
    Serial.println();
  }
  runLogSummary();
  Serial.printf("   heading trims N %+.2f E %+.2f S %+.2f W %+.2f deg\n",
                runLog.trims[0], runLog.trims[1], runLog.trims[2], runLog.trims[3]);
}

