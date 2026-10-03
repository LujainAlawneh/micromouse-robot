 # BZU Micromouse – Firmware

Arduino firmware (ESP32) for an autonomous micromouse robot. The robot reads
three distance sensors and a gyroscope, builds a map of the maze, and solves it
with a flood-fill algorithm.

## Features

- **Maze solving:** flood-fill search, return to start, and speed run
- **Straight driving:** heading PID using the gyroscope, with wall centering from the side sensors
- **Turns:** rate profile (accelerate, brake, slow creep) with learned correction values
- **Odometry:** wheel encoders with a calibratable distance scale
- **Live tuning:** change parameters from the Serial Monitor and save them in flash
- **Auto run:** starts by itself on battery power (no USB needed)
- **Run log:** the result of every wall-follow run is saved in flash and can be read later with `L`

## Hardware

| Part | Used for |
|---|---|
| ESP32 development board | Main controller |
| TB6612FNG motor driver | Drives the two DC motors (STBY tied to 3V3) |
| 2x DC gear motors with encoders | Movement and odometry |
| BNO055 IMU | Heading and turn angle |
| 3x VL6180X ToF sensors | Left, front and right wall distance |

## Pin map

These values come from `config.h`.

| Function | Pin(s) |
|---|---|
| Motor channel A (PWM / IN1 / IN2) | 25 / 26 / 27 |
| Motor channel B (PWM / IN1 / IN2) | 4 / 16 / 17 |
| Left encoder (C1 / C2) | 32 / 33 |
| Right encoder (C1 / C2) | 34 / 35 |
| BNO055 I2C (SDA / SCL), on `Wire1` | 13 / 14 |
| ToF I2C (SDA / SCL), on `Wire` | 21 / 22 |
| ToF XSHUT (left / front / right) | 18 / 19 / 23 |
| BOOT button | 0 |
| Status LED | 2 |

ToF sensors are given the addresses `0x30`, `0x31`, `0x32` at start-up. The
BNO055 uses address `0x29` on its own I2C bus.

## Code structure

The sketch folder is `BZU_Micromouse/`. The `.ino` file only contains `setup()`
and `loop()`; everything else is in headers.

| File | Content |
|---|---|
| `BZU_Micromouse.ino` | `setup()`, `loop()` and the include list |
| `config.h` | Pins and fixed hardware settings |
| `params.h` | Tunable parameters (PID, speeds, turn settings) |
| `motors.h` | Motor driver control |
| `encoders.h` | Encoder counting and speed |
| `imu.h` | BNO055 gyro and heading |
| `tof.h` | VL6180X distance sensors |
| `maze.h` | Wall map and flood fill |
| `tune.h` | Live parameter editing and saving |
| `motion.h` | Driving, turning, wall scans |
| `helpers.h` | LED, key check, idle helpers |
| `runs.h` | Explore, speed run, solve, full run |
| `diagnostics.h` | Turn tests, calibration, sensor streams |
| `runlog.h` | Run log saved in flash |
| `wallfollow.h` | Wall-follow test |
| `commands.h` | Help text and Serial commands |
| `autorun.h` | Battery start, countdown, hand-wave start |

> The order of the `#include` lines in `BZU_Micromouse.ino` matters: each file
> uses things defined in the files above it.

## Requirements

- Arduino IDE
- **ESP32 board support** (Espressif), board: *ESP32 Dev Module*
- **Adafruit BNO055** library (also install its dependencies, *Adafruit BusIO* and *Adafruit Unified Sensor*)

`Wire` and `Preferences` come with the ESP32 board package. The VL6180X sensors
are controlled directly through their registers, so no extra library is needed.

## Upload

1. Open `BZU_Micromouse/BZU_Micromouse.ino` in the Arduino IDE.
2. Select the board and the COM port.
3. Click **Upload**.
4. Open the Serial Monitor at **115200 baud** to use the commands below.

## Auto run (battery mode)

By default `AUTO_RUN` is `true` in `config.h`. After power-on the LED blinks for
5 seconds and the robot then starts the command set in `AUTO_RUN_CMD` (`g` by
default). **Keep the robot in the maze, or set `AUTO_RUN` to `false`, before
powering it on.**

- To run again: press the BOOT button, or hold a hand about 1 cm in front of the
  front sensor and take it away.
- To cancel during the countdown: press BOOT or send any key on Serial.

## Serial commands

Type a letter in the Serial Monitor (the robot must be placed in a cell centre).

| Key | Action |
|---|---|
| `g` | Solve: flood fill from start to the centre, then stop |
| `n` | Full competition run: search, return, speed run |
| `G` | Speed run using the maze saved in flash |
| `f` / `F` | Drive 1 cell / 4 cells |
| `r` / `l` / `u` | Turn right / left / 180 degrees |
| `t` / `y` | Turn series test (10x 90 degrees / 4x 180 degrees) |
| `q` | Square test |
| `a` | Front alignment |
| `w` / `W` | Wall follow (right-hand rule / straight first) |
| `L` | Print the last wall-follow run saved in flash |
| `s` / `i` / `e` | Stream ToF / IMU / encoders |
| `o` | Gyro scale test |
| `k` | Distance scale calibration |
| `b` | Re-measure gyro bias |
| `m` | Toggle gyro mode |
| `c` | Calibrate wall references (robot centred, 3 walls) |
| `x` / `X` / `U` / `d` | Record a right turn / left turn / 180 / one cell (prints CSV) |
| `[` `]` | Select a parameter |
| `+` `-` | Change the selected parameter |
| `?` | List parameters |
| `S` | Save parameters to flash |
| `R` | Restore default parameters |
| `p` | Print the maze |
| `z` | Clear the maze |
| `h` | Help |

Any key or the BOOT button stops the robot during a move.

## Calibration

Before the first run, check these in order:

1. `e`, then push the robot forward one cell by hand. Both encoders should show a similar positive value. If one is negative, flip its `ENC_SIGN` in `config.h`.
2. `i`, then rotate the robot clockwise. The angle must increase. If not, flip `CW_SIGN` in `config.h`.
3. `k` to calibrate the distance scale, and set `CELL_LEN` and `MAZE_SIZE` to match your maze (in `params.h`, or live with `+` `-` and `S`).
4. `c` to calibrate the wall references.

## Notes

- Left and right motor channels are swapped in software (`SWAP_MOTORS`), and the gyro direction is set with `CW_SIGN`. Both are in `config.h` and depend on how your robot is wired.
- Default `CELL_LEN` is 200 mm and `MAZE_SIZE` is 8; change them for your maze.
