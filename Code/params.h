// =====================================================================
//  PARAMETERS: tunable values + WallScan struct
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// =====================================================================
//  PARAMETERS (tunable, changeable live from Serial)
// =====================================================================
struct Params {
  // Straight driving: heading PID
  float KP = 6.0f, KD = 0.4f, KI = 0.0f;
  float MAX_CORR   = 60;
  float CRUISE_PWM = 100;
  float RUN_PWM    = 140;
  float END_PWM    = 55;
  float RAMP_MS    = 400;
  float RIGHT_TRIM = 0;
  float SLOW_MM    = 60;
  float DRIVE_LEAD = 0.030f;
  // Odometry: calibrate MM_PER_CNT with 'k', set CELL_LEN to your measured cell pitch
  float MM_PER_CNT   = 0.3500f;  // mm per encoder count  ('k' calibrates this)
  float CELL_LEN     = 200.0f;   // mm, YOUR cell pitch (competition maze = 182)
  float MAZE_SIZE    = 8;        // cells per side: 3 for your test maze, 16 for the real one
  // Start corner. Competition rule: facing the start cell's opening ("north"), the
  // outside walls are on the LEFT and BEHIND -> 0. Outside wall on the RIGHT -> 1.
  float START_RIGHT  = 0;
  float CAL_MM       = 400;      // distance the 'k' calibration drives (needs free space!)
  // Start of a move: both wheels get the same PWM first (break friction together),
  // then the heading correction fades in -> no wheel starts before the other.
  float START_PWM    = 80;
  float CORR_MS      = 150;      // ms to fade the heading correction in
  // Centering: P on the lateral error + D that ALIGNS the robot with the corridor.
  // The D term is what stops the slow sideways drift caused by a small gyro-scale
  // or turn error (the heading can be "perfect" and still not follow the corridor).
  float CENTER_GAIN  = 0.40f;   // stopped reading -> aim angle for the next move
  float LIVE_CENTER  = 0.25f;   // deg per mm of lateral error (0 = off)
  float LIVE_FILT    = 0.03f;   // low-pass on the side readings (~100 ms) - our ToF is +-6 mm
  float LIVE_DAMP    = 0.0f;    // deg per (mm/s): OFF. With +-6 mm noise the derivative is
                                // pure noise and the P term + trim are already well damped.
  // The steady part of the centering correction is moved into a per-direction
  // heading trim, so the robot ends up driving PARALLEL to the corridor and the
  // lateral error goes to zero instead of settling at an offset.
  float TRIM_LEARN   = 0.40f;   // 0 = off
  // Turns: RATE profile (accel - decel - constant crawl) + learned crawl coast.
  // No position bursts at the end -> the robot never shakes; the crawl is slow and
  // constant, so the coast angle is repeatable and is learned per direction.
  float TURN_MAX_DPS  = 250;    // cruise rate
  float TURN_ACC      = 1500;   // dps/s ramp up
  float TURN_BRAKE_A  = 1800;   // dps/s: short-brake deceleration (from your recordings)
  float TURN_CREEP_DPS = 70;    // dps: speed of the final creep
  float TURN_CRAWL_ANG = 14;    // deg: last part of the turn done creeping
  float TURN_MIN_PWM  = 45;     // deadband compensation (PWM where wheels just move)
  float TURN_MAX_PWM  = 160;
  float TURN_RETRY    = 99;     // deg: retry threshold. OFF: from standstill a retry is a
                                // stick-slip jump (your log: it made 5 turns worse). The next
                                // straight's heading PID removes a small residual anyway.
  float TURN_LEARN    = 0.20f;  // stop-time learning rate for turns
  float TRACE         = 1;      // 1 = print one diagnostic line per turn
  float TURN_FF       = 0.20f;  // pwm per dps (rate feed-forward)
  float TURN_KP       = 0.50f;  // pwm per dps of RATE error
  float CREEP_PWM_R   = 56;     // PWM of the creep, right turns (learned -> TURN_CREEP_DPS)
  float CREEP_PWM_L   = 50;     // PWM of the creep, left turns (learned)
  float CREEP_MIN_PWM = 45;     // learning never goes below this (below = the creep stalls)
  float CREEP_LEARN   = 0.08f;  // PWM per dps of creep-speed error, per turn
  // TURN IN PLACE: your motors are stronger forward, so every turn crept ~4 mm
  // forward (-> the front-right drift after 't'). This holds the wheel average.
  float CM_KP         = 6.0f;   // PWM per mm of wheel-average drift
  float CM_KD         = 0.15f;  // PWM per mm/s
  // Stop rule from YOUR data (32 turns): the angle the robot still rotates after
  // the brake is PROPORTIONAL to its speed at the brake: coast = STOP_T * speed
  // (right 0.054 s, left 0.058 s, +-0.007). A fixed coast angle was wrong whenever
  // the creep speed changed (47..114 dps) and it had a built-in ~1 deg undershoot.
  float STOP_T_R      = 0.054f; // s, right turns (learned)
  float STOP_T_L      = 0.058f; // s, left turns (learned)
  float GYRO_LAG      = 0.010f; // s, gyro filter delay (IMUPLUS ~10 ms)
  float LEAD_LEARN    = 0.30f;  // coast-model learning for turns and drives (0 = off)
  // Front alignment
  float ALIGN_PWM    = 65;      // front alignment speed once moving
  float ALIGN_KICK   = 110;     // short kick to break static friction (90 failed once)
  float ALIGN_TOL    = 8.0f;    // mm - no reverse/forward move for smaller errors (sensor is +-6)
  // The robot TURNS about its wheel axle, but ref.front was taken with the BODY in the
  // cell centre. Your 'w' log (23 turns): the axle was ~24 mm behind the centre, so
  // every turn threw the robot 15-25 mm sideways (right turns -> right, left -> left,
  // 180 -> nothing). Stopping 24 mm closer to the front wall puts the axle in the centre.
  // Tune: 'lat after turn' positive after RIGHT turns -> make it more negative.
  float ALIGN_OFFSET = -24.0f;  // mm added to the front target (ref.front)
  // Gyro
  float GYRO_SCALE   = 0.9684f;   // measured with 'o' (3 turns by hand)
  // Walls
  float SIDE_MARGIN  = 60;
  float FRONT_MARGIN = 90;
  float SIDE_WINDOW  = 60;      // centering only if |reading - ref| < window
};

Params P;

struct WallScan {
  bool  left = false, front = false, right = false;
  float l = NAN, f = NAN, r = NAN;
  float lateral = NAN;
};

