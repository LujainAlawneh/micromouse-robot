# Micromouse Robot

<div align="center">
  <img width="450" alt="Micromouse Robot" src="YOUR-IMAGE-LINK" />
</div>

## Project Overview

An autonomous Micromouse robot that explores a maze, builds a map of it, and
solves it using a Flood Fill algorithm. The robot was designed and built from
scratch, including a custom 3D-printed chassis, ESP32-based firmware, sensor
integration, and closed-loop motion control.

https://github.com/user-attachments/assets/da5e71bc-a102-4974-b1de-06f63951b2f3

> Final Micromouse prototype during testing.

---

## Team

**Supervisor:** Dr. Wasel Ghanem

**Team Members:**
- Lujain Alawneh
- Safia Salameh
- Anees Hammoudeh
- Dana Obaid
- Mohammad Abutteen

---

## About the Project

A Micromouse is a small autonomous robot that has to find its way through a
maze without human control.

The goal of this project was to design, build, and program a complete Micromouse
robot from scratch, covering:

- Mechanical design
- 3D printing
- Electronics
- Sensor integration
- Motor control
- Embedded programming
- Maze mapping
- Autonomous navigation

The robot reads its surroundings using three VL6180X Time-of-Flight distance
sensors positioned on the left, front, and right sides.

It tracks its orientation using a BNO055 IMU and measures wheel movement using
motor encoders.

The ESP32 processes this information, updates an internal representation of the
maze, and uses the Flood Fill algorithm to determine the next direction of
movement.

---

## Hardware

| Part | Used For |
|---|---|
| ESP32 Development Board (30-pin) | Main controller |
| TB6612FNG Motor Driver | Drives the two motors |
| 2x N20 DC Gear Motors with Encoders | Movement and odometry |
| BNO055 IMU | Heading and turn-angle measurement |
| 3x VL6180X ToF Sensors | Left, front, and right wall-distance measurement |
| 9V Rechargeable Battery | Main power source |
| DC-DC Voltage Regulator | Steps the battery voltage down for the electronics |
| Wheels | Robot movement |
| Custom 3D-Printed Chassis | Mechanical structure and component mounting |

The full pin map and software setup are available in:

[`Code/README.md`](Code/README.md)

---

## Schematic

The approved circuit schematic was created using KiCad. It includes the
connections between:

- ESP32
- VL6180X sensors
- BNO055 IMU
- TB6612FNG motor driver
- Motors and encoders
- Battery
- Voltage regulator

<img width="701" height="503" alt="Micromouse circuit schematic" src="https://github.com/user-attachments/assets/cef75da9-4727-4dc8-9262-0d14f23c5f77" />

*Circuit schematic of the Micromouse.*

---

## Design

The chassis was designed using CAD software and manufactured using 3D printing.
The design went through several iterations before reaching the final version.

### Preliminary Design

The first design was used to test the basic component placement, dimensions,
and mechanical structure.

<img width="562" height="424" alt="Preliminary CAD design" src="https://github.com/user-attachments/assets/d92b3999-1192-4491-96db-2e20482407b6" />

*Preliminary chassis design.*

### Final Design

After testing and identifying mechanical issues, the chassis dimensions and
component placement were improved.

The final chassis was designed to hold:

- ESP32
- Motor driver
- Distance sensors
- IMU
- Battery
- Motors
- Wheels
- Wiring

<img width="900" alt="Final CAD design" src="https://github.com/user-attachments/assets/886dd40c-d063-41f5-a978-a26f2b7abbe1" />

*Final chassis design.*

---

## Build Process

The robot was built gradually, starting with individual component testing and
ending with full hardware and software integration.

### 1. Initial Wiring and Planning

The electronics were first connected outside the chassis to verify the basic
connections between the ESP32, sensors, motor driver, motors, encoders, and IMU.

<!-- Drag the first wiring photo here -->

### 2. Mechanical Assembly

The motors, wheels, sensors, battery, and control electronics were installed on
the 3D-printed chassis.

<!-- Drag the robot-taking-shape photo here -->

### 3. Final Robot

After solving the mechanical and electrical problems, all components were
integrated into the final robot.

<!-- Drag the final robot photo here -->

---

## Software

The firmware was developed for the ESP32 using the Arduino framework.

The main software features include:

- **Flood Fill maze solving**
- **Maze mapping**
- **Search run**
- **Return navigation**
- **Speed-run logic**
- **Heading PID control**
- **Wall centering**
- **Controlled turning**
- **BNO055-based heading feedback**
- **Encoder odometry**
- **Motor speed control**
- **Sensor filtering**
- **Live parameter tuning through the Serial Monitor**

Instructions for uploading the firmware, the pin map, and available Serial
commands are available in:

[`Code/README.md`](Code/README.md)

---

## Software Overview

The software is divided into two main stages: maze exploration and path
execution.

During the **Search / Exploration Mode**, the robot senses the maze, detects
walls, updates its internal map, runs the Flood Fill algorithm, and selects the
next direction.

The motion-control system then uses the BNO055 and wheel encoders to execute
the selected movement accurately.

After the maze has been explored, the stored path can be used for the
**Return / Speed Run Mode**.

<img width="900" alt="Software overview diagram" src="https://github.com/user-attachments/assets/2ebddd12-7941-4d0f-8cb6-31c24bd748b5" />

*Software overview.*

---

## How It Works

The robot starts without knowing the complete maze.

As it moves through the maze, it continuously senses its surroundings, updates
the internal maze map, calculates the best direction, and executes the movement.

The navigation cycle works as follows:

1. The three VL6180X sensors measure the distances to the left, front, and right.
2. The ESP32 converts these measurements into wall or no-wall information.
3. Newly detected walls are stored in the internal maze representation.
4. The Flood Fill algorithm calculates or updates the values of the maze cells.
5. The robot selects the accessible neighboring cell with the lowest Flood Fill value.
6. The ESP32 converts the selected direction into motor commands.
7. The TB6612FNG motor driver controls the two motors.
8. Encoder feedback is used to monitor traveled distance and wheel rotation.
9. The BNO055 IMU provides heading information for accurate turns.
10. The process repeats as the robot explores the maze.

This allows the robot to gradually build a map of an initially unknown maze.

---

## Flood Fill Algorithm

Flood Fill is the main path-planning algorithm used by the robot.

The target cell is assigned the lowest value, usually `0`.

Other cells are assigned increasing values based on their distance from the
target while respecting the known maze walls.

The robot then attempts to move toward an accessible neighboring cell with a
lower value.

Example:

```text
6 5 4 3
5 4 3 2
4 3 2 1
3 2 1 0
```

<!-- Drag a Flood Fill image here if you have one -->

---

## Future Work

- **Custom PCB:** move all connections onto a printed circuit board to remove the loose wires and make the robot smaller and lighter.
- **Faster speed runs:** increase the speed of the final run by further tuning the PID and turn parameters.
- **Smarter paths:** add diagonal movement and smoother corners to shorten the run time.
- **Larger mazes:** test and tune the robot on a full-size 16x16 maze.
- **Better sensing:** reduce sensor noise to improve wall centering.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
