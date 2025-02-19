# Maze Solving Robot with MPU6050-Based Navigation

## Overview
This project implements a maze-solving robot using an Arduino-based system with MPU6050 for precise movement control and IR sensors for wall detection. The robot uses a Depth-First Search (DFS) algorithm to navigate through the maze while maintaining accurate positioning and orientation using gyroscope and accelerometer data.

## Features
- Depth-First Search (DFS) maze-solving algorithm
- Precise movement control using MPU6050 (gyroscope and accelerometer)
- Wall detection using IR sensors
- Accurate 90-degree turns and straight-line movement
- Position tracking and cell-based navigation
- EEPROM storage for visited cells

## Hardware Requirements
- Arduino Board (e.g., Arduino Uno)
- MPU6050 Gyroscope/Accelerometer Module
- 3x IR Sensors (left, front, right)
- 2x DC Motors with wheels
- L298N Motor Driver or similar
- Power Supply
- Chassis and mounting hardware

## Pin Configuration
### Motor Pins
- Left Motor Speed: Pin 9
- Right Motor Speed: Pin 5
- Left Motor Direction 1: Pin 3
- Left Motor Direction 2: Pin 2
- Right Motor Direction 1: Pin 8
- Right Motor Direction 2: Pin 4

### IR Sensor Pins
- Left IR Sensor: Pin 10
- Front IR Sensor: Pin 11
- Right IR Sensor: Pin 12

### MPU6050
- SDA: Arduino SDA Pin
- SCL: Arduino SCL Pin
- VCC: 5V
- GND: GND

## Dependencies
- Wire.h (Arduino Standard Library)
- EEPROM.h (Arduino Standard Library)

## Installation
1. Connect the hardware components according to the pin configuration
2. Install the Arduino IDE
3. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/maze-solving-robot.git

Open the .ino file in Arduino IDE
Upload the code to your Arduino board

Configuration
Adjust these parameters in the code according to your setup:
cppCopyconst float CELL_SIZE = 0.3; // Size of maze cell in meters
const int maxSpeed = 255; // Maximum motor speed
const int minSpeed = 160; // Minimum motor speed
int equilibriumSpeed = 248; // Initial speed for straight movement
Calibration
The MPU6050 needs to be calibrated before use:

Place the robot on a flat surface
Power on the robot
Wait for the calibration process to complete (indicated by serial output)
The robot is ready to use after calibration

Usage

Place the robot at the start position of the maze
Power on the system
The robot will automatically:

Detect walls using IR sensors
Navigate through the maze using DFS
Track its position using MPU6050
Store visited cells in EEPROM
Stop when reaching the goal position



How It Works

Movement Control

Uses MPU6050 data for precise orientation control
Implements PID-like control for straight-line movement
Maintains accurate 90-degree turns


Maze Solving

Implements DFS algorithm for maze exploration
Tracks visited cells
Maintains a stack for backtracking
Updates position based on movement and orientation


Sensor Integration

Uses IR sensors for wall detection
Uses MPU6050 for orientation and movement tracking
Combines sensor data for accurate navigation



Troubleshooting

Inaccurate Turns: Adjust the PID control parameters in controlSpeed()
Drift in Straight Lines: Check MPU6050 calibration and equilibriumSpeed
Wall Detection Issues: Adjust IR sensor positioning and sensitivity
Movement Issues: Verify motor connections and speed settings

Contributing

Fork the repository
Create your feature branch (git checkout -b feature/AmazingFeature)
Commit your changes (git commit -m 'Add some AmazingFeature')
Push to the branch (git push origin feature/AmazingFeature)
Open a Pull Request

ARYAN TOMAR
Acknowledgments

Thanks to the Arduino community for libraries and support
MPU6050 implementation inspired by various open-source projects
