# Micromouse Robot

An autonomous Micromouse robot designed to explore an unknown maze, build an
internal map, and determine an efficient path using the Flood Fill algorithm.

The robot was designed and built from scratch, including the mechanical chassis,
electronics, sensing system, embedded software, and autonomous navigation logic.

## Project Overview

The Micromouse uses:

- ESP32 as the main controller
- Three VL6180X ToF sensors for left, front, and right wall detection
- BNO055 IMU for heading and turn-angle feedback
- Two N20 DC motors with encoders for movement and odometry
- TB6612FNG motor driver for motor control
- Flood Fill algorithm for maze navigation
- Custom 3D-printed chassis

During exploration, the robot detects walls, updates its internal maze map,
recalculates Flood Fill values, and selects the next direction to move.

## Team

**Supervisor:** Dr. Wasel Ghanem

**Team Members:**
- Lujain Alawneh
- Safia Salameh
- Anees Hammoudeh
- Dana Obaid
- Mohammad Abutteen

## Presentation

The full project presentation is available here:

[View Project Presentation](Presentation.pdf)

## Main Features

- Autonomous maze exploration
- Flood Fill path planning
- Real-time wall detection
- Maze mapping
- Encoder-based odometry
- BNO055 heading feedback
- Controlled 90-degree turns
- Motor speed and direction control
- Custom CAD and 3D-printed chassis

## Project Goal

The goal of this project was to integrate mechanical design, electronics,
embedded programming, sensing, and autonomous navigation into one complete
Micromouse robot.

---

Developed as an autonomous robotics project.
