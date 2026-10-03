# Micromouse Robot

An autonomous micromouse robot that explores a maze, builds a map of it, and
solves it using a flood-fill algorithm. Designed and built from scratch: custom
3D-printed chassis, ESP32 firmware, and sensor-based control.

https://github.com/user-attachments/assets/da5e71bc-a102-4974-b1de-06f63951b2f3

## Team

**Supervisor:** Dr. Wasel Ghanem

**Team members:** Lujain Alawneh, Safia Salameh, Anees Hammoudeh, Dana Obaid, Mohammad Abutteen

## About the project

A micromouse is a small robot that has to find its way through a maze on its
own. The goal of this project is to design, build, and program a micromouse
from scratch, covering mechanical design, electronics, and software.

The robot reads its surroundings with three distance sensors (left, front,
right) and a gyroscope, tracks its movement with wheel encoders, and uses a
flood-fill algorithm to find a path through the maze.

## Hardware

| Part | Used for |
|---|---|
| ESP32 development board (30-pin) | Main controller |
| TB6612FNG motor driver | Drives the two motors |
| 2x DC gear motors with encoders | Movement and odometry |
| BNO055 IMU | Heading and turn angle |
| 3x VL6180X ToF sensors | Left, front and right wall distance |

<!-- Add battery and wheels as new table rows here if you want -->

The full pin map is in [`Code/README.md`](Code/README.md).

## Schematic

The approved circuit schematic, drawn in KiCad:

<!-- IMAGE: drag the schematic image here -->

[View full-resolution schematic (PDF)](Hardware/Micromouse_First.pdf)

## Design

The chassis was designed in CAD and 3D printed. We started with a preliminary
design and improved it into the final version.

**Preliminary design**

<!-- IMAGE: drag the preliminary CAD image here -->

**Final design**

<!-- IMAGE: drag the final CAD image here -->

## Build process

**1. Initial wiring and planning**

<!-- IMAGE: drag the first wiring photo here -->

**2. The robot taking shape**

<!-- IMAGE: drag the "robot taking shape" photo here -->

**3. Final robot**

<!-- IMAGE: drag the final robot photo here -->

## Software

The firmware is written for the ESP32 using the Arduino framework. The main
parts are:

- **Flood-fill** maze solving (search run, return, and speed run)
- **Heading PID** for straight driving, with centering between the walls
- **Controlled turns** using the gyroscope
- **Encoder odometry** with calibration
- **Live tuning** of parameters over the Serial Monitor

How to upload the code, the pin map, and all Serial commands are in
[`Code/README.md`](Code/README.md).

## Challenges

*(Draft based on the code. Edit it so it matches what you really experienced.)*

- **Three identical distance sensors:** all three VL6180X sensors start with the same I2C address, so each one is switched on separately and given its own address at start-up.
- **Sensor noise:** the distance readings vary by a few millimetres, so the readings are filtered before the robot uses them for centering.
- **Accurate turns:** turning exactly 90 degrees needed a controlled speed profile and tuning with the gyroscope.
- **Motors not identical:** the robot drifted forward when turning in place, so the controller corrects for it.

## Future work

<!-- Write your real next steps here, for example a custom PCB to remove the wires, faster speed runs -->

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
