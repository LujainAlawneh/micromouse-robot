// =====================================================================
//  MAZE: wall map + flood fill
//  Part of BZU Micromouse v2 (included from BZU_Micromouse.ino)
// =====================================================================
#pragma once

// =====================================================================
//  MAZE (wall map + flood fill)
// =====================================================================
namespace Maze {
constexpr int NMAX = MAZE_N;                 // array size (16)
constexpr uint8_t UNREACHABLE = 255;
// effective size comes from P.MAZE_SIZE -> test on a 3x3 maze without recompiling
inline int N() { return (int)constrain(P.MAZE_SIZE, 2.0f, (float)NMAX); }
// start cell column: 0 = bottom-left corner, N-1 = bottom-right corner (facing north)
inline int startX() { return P.START_RIGHT > 0.5f ? N() - 1 : 0; }

uint8_t wallBits[NMAX][NMAX];
uint8_t knownBits[NMAX][NMAX];
uint8_t distMap[NMAX][NMAX];
bool    seen[NMAX][NMAX];

bool inside(int x, int y) { return x >= 0 && y >= 0 && x < N() && y < N(); }
void setWall(int x, int y, int d, bool present);

inline int dx(int d) { return d == 1 ? 1 : d == 3 ? -1 : 0; }
inline int dy(int d) { return d == 0 ? 1 : d == 2 ? -1 : 0; }

void reset() {
  memset(wallBits, 0, sizeof(wallBits));
  memset(knownBits, 0, sizeof(knownBits));
  memset(seen, 0, sizeof(seen));
  const int n = N();
  for (int i = 0; i < n; i++) {
    setWall(i, n - 1, 0, true);
    setWall(i, 0, 2, true);
    setWall(n - 1, i, 1, true);
    setWall(0, i, 3, true);
  }
}

void setWall(int x, int y, int d, bool present) {
  if (!inside(x, y)) return;
  // an OUTER wall can never be open. If the sensors say so, the robot's idea of
  // where it is / which way it faces is wrong (e.g. started in the wrong corner).
  if (!present && !inside(x + dx(d), y + dy(d))) {
    Serial.printf("!! (%d,%d) sees NO wall on the maze border (side %d) -> wrong start corner/direction?\n"
                  "   start cell: face its opening; outer wall on the robot's LEFT -> START_RIGHT 0,\n"
                  "   on its RIGHT -> START_RIGHT 1 ('[' ']' to select it, '+' '-', 'S' save)\n", x, y, d);
    return;
  }
  uint8_t b = 1 << d;
  knownBits[x][y] |= b;
  if (present) wallBits[x][y] |= b; else wallBits[x][y] &= ~b;
  int nx = x + dx(d), ny = y + dy(d);
  if (inside(nx, ny)) {
    uint8_t ob = 1 << ((d + 2) % 4);
    knownBits[nx][ny] |= ob;
    if (present) wallBits[nx][ny] |= ob; else wallBits[nx][ny] &= ~ob;
  }
}

bool hasWall(int x, int y, int d) { return wallBits[x][y] & (1 << d); }
bool isKnown(int x, int y, int d) { return knownBits[x][y] & (1 << d); }
void markVisited(int x, int y)    { if (inside(x, y)) seen[x][y] = true; }
bool visited(int x, int y)        { return seen[x][y]; }

bool isGoal(int x, int y) {
  const int n = N();
  if (n % 2 == 0) return (x == n / 2 - 1 || x == n / 2) && (y == n / 2 - 1 || y == n / 2);
  return x == n / 2 && y == n / 2;                    // odd size: the single centre cell
}

void flood(bool toGoal, bool unknownIsWall) {
  static uint8_t qx[NMAX * NMAX], qy[NMAX * NMAX];
  int head = 0, tail = 0;
  memset(distMap, UNREACHABLE, sizeof(distMap));
  for (int x = 0; x < N(); x++)
    for (int y = 0; y < N(); y++) {
      bool target = toGoal ? isGoal(x, y) : (x == startX() && y == 0);
      if (target) { distMap[x][y] = 0; qx[tail] = x; qy[tail] = y; tail++; }
    }
  while (head < tail) {
    int x = qx[head], y = qy[head]; head++;
    for (int d = 0; d < 4; d++) {
      if (hasWall(x, y, d)) continue;
      if (unknownIsWall && !isKnown(x, y, d)) continue;
      int nx = x + dx(d), ny = y + dy(d);
      if (!inside(nx, ny) || distMap[nx][ny] != UNREACHABLE) continue;
      distMap[nx][ny] = distMap[x][y] + 1;
      qx[tail] = nx; qy[tail] = ny; tail++;
    }
  }
}

uint8_t dist(int x, int y) { return distMap[x][y]; }

int nextDir(int x, int y, int dir, bool unknownIsWall) {
  int best = -1, bestScore = 1 << 30;
  for (int k = 0; k < 4; k++) {
    static const int order[4] = {0, 1, 3, 2};
    int d = (dir + order[k]) % 4;
    if (hasWall(x, y, d)) continue;
    if (unknownIsWall && !isKnown(x, y, d)) continue;
    int nx = x + dx(d), ny = y + dy(d);
    if (!inside(nx, ny) || distMap[nx][ny] == UNREACHABLE) continue;
    int score = distMap[nx][ny] * 16
              + (visited(nx, ny) ? 2 : 0)
              + (order[k] == 0 ? 0 : order[k] == 2 ? 4 : 1);
    if (score < bestScore) { bestScore = score; best = d; }
  }
  return best;
}

int buildPath(int* dirs, int maxLen) {
  flood(true, true);
  if (distMap[startX()][0] == UNREACHABLE) return -1;
  int x = startX(), y = 0, d = 0, n = 0;
  while (!isGoal(x, y) && n < maxLen) {
    int nd = nextDir(x, y, d, true);
    if (nd < 0) return -1;
    dirs[n++] = nd;
    x += dx(nd); y += dy(nd); d = nd;
  }
  return n;
}

void save() {
  Preferences p;
  p.begin("maze", false);
  p.putBytes("w", wallBits, sizeof(wallBits));
  p.putBytes("k", knownBits, sizeof(knownBits));
  p.putBytes("v", seen, sizeof(seen));
  p.end();
}

bool load() {
  Preferences p;
  p.begin("maze", true);
  bool ok = p.getBytesLength("w") == sizeof(wallBits);
  if (ok) {
    p.getBytes("w", wallBits, sizeof(wallBits));
    p.getBytes("k", knownBits, sizeof(knownBits));
    p.getBytes("v", seen, sizeof(seen));
  }
  p.end();
  return ok;
}

void print() {
  const int n = N();
  for (int y = n - 1; y >= 0; y--) {
    String top = "+", mid = "";
    for (int x = 0; x < n; x++) {
      top += hasWall(x, y, 0) ? "---+" : (isKnown(x, y, 0) ? "   +" : " . +");
      mid += hasWall(x, y, 3) ? "|" : (isKnown(x, y, 3) ? " " : ".");
      uint8_t v = distMap[x][y];
      char buf[5];
      if (v == UNREACHABLE) snprintf(buf, sizeof(buf), " ##");
      else snprintf(buf, sizeof(buf), "%3d", v);
      mid += buf;
    }
    mid += "|";
    Serial.println(top);
    Serial.println(mid);
  }
  String bottom = "+";
  for (int x = 0; x < n; x++) bottom += "---+";
  Serial.println(bottom);
}
}

