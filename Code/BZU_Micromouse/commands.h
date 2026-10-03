// =====================================================================
//  COMMANDS: help text + serial command dispatch
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

static void help() {
  Serial.println(F(
    "\n=== BZU MICROMOUSE v2 ===\n"
    " g SOLVE: standard flood fill start -> centre, then stop (+ result report)\n"
    " n full competition run (search, return, speed run)   G speed run (maze from flash)\n"
    " f 1 cell   F 4 cells   r right   l left   u 180   q square   a front-align\n"
    " t 10x alternating 90 (RMS)        y 4x 180 (RMS)\n"
    " w wall follow (right-hand)        W wall follow (straight first)\n"
    " L print the last w/W run saved in flash (e.g. a battery run without USB)\n"
    " s ToF stream  i IMU stream  e encoders  o gyro scale (N turns by hand)\n"
    " k distance scale (drives 500 mm, you type the measured mm)\n"
    " RECORD: x right turn  X left turn  U 180  d one cell  (prints CSV for Claude)\n"
    " b re-measure gyro bias   m toggle gyro mode (IMUPLUS / low-latency)\n"
    " c calibrate wall refs (centered, 3 walls)\n"
    " [ ] select param   + - change   ? list   S save   R defaults\n"
    " p print maze   z clear maze   h help\n"
    " any key / BOOT = STOP during motion\n"
    " AUTO_RUN: BOOT, or hand ~1 cm in front of the robot then away = run again"));
}

static void exec(char c) {
  Motion::clearAbort();
  switch (c) {
    case 'g': solveRun(true); break;                 // standard: start -> centre, stop
    case 'n': fullRun(true, true); break;            // competition: search, return, speed runs
    case 'G': if (Maze::load()) fullRun(false, true); else Serial.println("no maze saved"); break;
    case 'f': Motion::resetHeading(); idleFor(300); Motion::driveCells(1, P.CRUISE_PWM, 0, Motion::scan(3).lateral);
              Serial.printf("dist err %+.1f mm  lead %.3f\n", Motion::lastDriveError(), P.DRIVE_LEAD); break;
    case 'F': Motion::resetHeading(); idleFor(300); Motion::driveCells(4, P.CRUISE_PWM, 0, Motion::scan(3).lateral);
              Serial.printf("dist err %+.1f mm\n", Motion::lastDriveError()); break;
    case 'r': Motion::resetHeading(); Motion::turn(+1); Serial.printf("err %+.2f\n", Motion::lastTurnError()); break;
    case 'l': Motion::resetHeading(); Motion::turn(-1); Serial.printf("err %+.2f\n", Motion::lastTurnError()); break;
    case 'u': Motion::resetHeading(); Motion::turn(+2); Serial.printf("err %+.2f\n", Motion::lastTurnError()); break;
    case 't': { static const int s[10] = {1, -1, 1, -1, 1, -1, 1, -1, 1, -1}; turnSeries(s, 10); } break;
    case 'y': { static const int s[4] = {2, -2, 2, -2}; turnSeries(s, 4); } break;
    case 'q': Motion::resetHeading(); idleFor(300);
              for (int i = 0; i < 4; i++) { if (!Motion::driveCells(1, P.CRUISE_PWM)) break; if (!Motion::turn(+1)) break; }
              Serial.printf("final heading err %+.2f\n", Motion::lastTurnError()); break;
    case 'a': Motion::resetHeading(); Motion::frontAlign(); break;
    case 'w': wallFollow(false); break;
    case 'W': wallFollow(true); break;
    case 'L': runLogPrint(); break;
    case 's': streamTof(); break;
    case 'i': streamImu(); break;
    case 'e': streamEnc(); break;
    case 'o': gyroScaleTest(); break;
    case 'k': distScaleTest(); break;
    case 'x': recTurn(+1); break;
    case 'X': recTurn(-1); break;
    case 'U': recTurn(+2); break;
    case 'd': recDrive(); break;
    case 'b': Imu::measureBias(); break;
    case 'm': Imu::setLowLatency(!Imu::lowLatency()); Imu::measureBias(); break;
    case 'c': calibrate(); break;
    case '[': Tune::select(-1); break;
    case ']': Tune::select(+1); break;
    case '+': case '=': Tune::adjust(+1); break;
    case '-': Tune::adjust(-1); break;
    case '?': Tune::list(); break;
    case 'S': Tune::save(); break;
    case 'R': Tune::defaults(); Motion::clearTrim(); break;
    case 'p': Maze::flood(true, false); Maze::print(); break;
    case 'z': Maze::reset(); Serial.println("maze cleared"); break;
    case '\n': case '\r': case ' ': break;
    default: help();
  }
  brakeMotors();
}

