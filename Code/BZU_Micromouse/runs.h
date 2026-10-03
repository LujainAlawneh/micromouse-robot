// =====================================================================
//  RUNS: explore, speed run, solve, full competition run
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

static int  posX = 0, posY = 0;
static bool firstMove = false;

static void mapWalls(const WallScan& w) {
  int d = Motion::dir();
  Maze::setWall(posX, posY, (d + 3) % 4, w.left);
  Maze::setWall(posX, posY, d, w.front);
  Maze::setWall(posX, posY, (d + 1) % 4, w.right);
  Maze::markVisited(posX, posY);
  Serial.printf("(%d,%d) d%d  L%d F%d R%d  (%.0f / %.0f / %.0f mm)  lat %.1f\n",
                posX, posY, d, w.left, w.front, w.right, w.l, w.f, w.r, w.lateral);
}

static bool isTarget(bool toGoal, int x, int y) { return toGoal ? Maze::isGoal(x, y) : (x == Maze::startX() && y == 0); }

static int statMoves = 0, statTurns = 0;          // for the solve report

static bool explore(bool toGoal) {
  const float startExtra = START_AT_BACK_WALL ? (P.CELL_LEN - WALL_MM) * 0.5f - AXLE_TO_TAIL_MM : 0.0f;
  const int maxMoves = 4 * Maze::N() * Maze::N();   // safety: never wander forever
  int moves0 = statMoves;
  while (true) {
    WallScan w = Motion::scanCached(3);
    mapWalls(w);
    if (isTarget(toGoal, posX, posY)) return true;
    if (statMoves - moves0 > maxMoves) { Serial.println("MOVE LIMIT - lost?"); return false; }

    Maze::flood(toGoal, false);
    if (Maze::dist(posX, posY) == Maze::UNREACHABLE) { Serial.println("NO PATH"); return false; }
    int nd = Maze::nextDir(posX, posY, Motion::dir(), false);
    if (nd < 0) return false;

    int cells = 1;
    int cx = posX + Maze::dx(nd), cy = posY + Maze::dy(nd);
    while (Maze::visited(cx, cy) && !isTarget(toGoal, cx, cy) &&
           Maze::nextDir(cx, cy, nd, false) == nd) {
      cells++; cx += Maze::dx(nd); cy += Maze::dy(nd);
    }

    float lat = w.lateral;
    if (nd != Motion::dir()) {
      statTurns += (((nd - Motion::dir()) % 4 + 4) % 4 == 2) ? 2 : 1;
      if (!Motion::turnToDir(nd)) return false;
      lat = Motion::freshLateral();                  // read while the turn settled (no extra stop)
    }
    float extra = (firstMove && nd == NORTH) ? startExtra : 0.0f;
    firstMove = false;
    if (!Motion::driveCells(cells, P.CRUISE_PWM, extra, lat)) return false;
    statMoves += cells;
    posX += cells * Maze::dx(nd);
    posY += cells * Maze::dy(nd);
  }
}

static bool speedRun() {
  static int path[Maze::NMAX * Maze::NMAX];
  int n = Maze::buildPath(path, Maze::NMAX * Maze::NMAX);
  if (n <= 0) { Serial.println("no known path"); return false; }
  for (int i = 0; i < n;) {
    int d = path[i], k = 1;
    while (i + k < n && path[i + k] == d) k++;
    if (!Motion::turnToDir(d)) return false;
    float lat = Motion::freshLateral();              // from the last stop, if it has one
    if (!Motion::driveCells(k, P.RUN_PWM, 0, lat)) return false;
    posX += k * Maze::dx(d); posY += k * Maze::dy(d);
    i += k;
  }
  return true;
}

static void waitStart() {
  Serial.println("Start: BOOT button / hand in front / any key");
  while (true) {
    Motion::idleTick();
    if (digitalRead(PIN_BUTTON) == LOW) { while (digitalRead(PIN_BUTTON) == LOW) delay(5); break; }
    const TofReading& F = Tof::get(TOF_FRONT);
    if (F.valid && F.mm < 25) { while (Tof::get(TOF_FRONT).valid && Tof::get(TOF_FRONT).mm < 60) Motion::idleTick(); break; }
    if (key()) break;
  }
}

// STANDARD flood fill (like the simulator code): start -> centre, then STOP.
// Unknown walls are assumed open; after every cell: read walls, re-flood,
// step to the neighbour with the smallest distance.
static void solveRun(bool wait) {
  if (wait) waitStart();
  blink(3);
  idleFor(800);
  Imu::measureBias();
  Motion::resetHeading();
  Motion::clearAbort();
  led(1);
  posX = Maze::startX(); posY = 0;
  firstMove = START_AT_BACK_WALL;
  statMoves = statTurns = 0;
  Maze::reset();
  Serial.printf("== SOLVE: standard flood fill, %dx%d, start (%d,0) -> centre ==\n",
                Maze::N(), Maze::N(), Maze::startX());
  uint32_t t0 = millis();
  bool ok = explore(true);
  float tS = (millis() - t0) / 1000.0f;
  brakeMotors();
  int unique = 0;
  for (int x = 0; x < Maze::N(); x++)
    for (int y = 0; y < Maze::N(); y++) if (Maze::visited(x, y)) unique++;
  if (ok) Maze::save();                            // 'G' can now speed-run the known path
  Serial.println("\nSTANDARD FLOODFILL TEST RESULT");
  Serial.printf("Maze Size: %d x %d\n", Maze::N(), Maze::N());
  Serial.printf("Result: %s\n", ok ? "SUCCESS" : "FAILED");
  Serial.printf("%s: %.2f s\n", ok ? "Time to Center" : "Elapsed Time", tS);
  Serial.printf("Final Position: (%d, %d)\n", posX, posY);
  Serial.printf("Unique Cells Visited: %d\n", unique);
  Serial.printf("Total Moves: %d\n", statMoves);
  Serial.printf("Total Turns: %d\n", statTurns);
  Maze::flood(true, false);
  Maze::print();
  if (ok) blink(4, 80);
  led(0);
}

static void fullRun(bool search, bool wait) {
  if (wait) waitStart();
  blink(3);
  idleFor(800);
  Imu::measureBias();
  Motion::resetHeading();
  Motion::clearAbort();
  led(1);
  posX = Maze::startX(); posY = 0;
  firstMove = START_AT_BACK_WALL;
  if (search) {
    Maze::reset();
    Serial.println("== SEARCH ==");
    if (!explore(true)) goto fail;
    Maze::save(); blink(4, 80); led(1);
    Serial.println("== RETURN ==");
    if (!explore(false)) goto fail;
    Maze::save();
    Motion::turnToDir(NORTH);
    firstMove = false;
    idleFor(1000);
  } else {
    firstMove = false;
  }
  Serial.println("== SPEED RUN ==");
  if (!speedRun()) goto fail;
  blink(4, 80); led(1);
  explore(false);
  Motion::turnToDir(NORTH);
  Serial.println("DONE");
  led(0); brakeMotors();
  return;
fail:
  Serial.println("STOPPED");
  led(0); brakeMotors();
}

