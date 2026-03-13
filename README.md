Vishwa
Overview

Vishwa is a competetion-focused development repository that contains code, experiments, and system components developed under the Vishwa VJTI.
Embedded systems

Remote communication

Control algorithms

Hardware integration

It acts as a development base for robotics experiments, rover systems, and automation tools.

Objectives

The main goals of this repository are:

Robotics Development for Our Rover Participating in IRC 25'

Implement control systems for robotic platforms.

Embedded Systems Integration

Interface microcontrollers such as ESP32 with hardware modules.

Remote Communication

Enable remote control or monitoring using protocols like SSH or web interfaces.

Experimental Robotics Platform

Provide a sandbox for testing robotics concepts.

Repository Structure

Typical structure of the project:

vishwa/
│
├── firmware/
│   ├── esp32/
│   └── microcontroller_code/
│
├── communication/
│   ├── ssh_control
│   └── networking
│
├── control_system/
│   ├── motor_control
│   └── navigation
│
├── hardware/
│   ├── schematics
│   └── board_notes
│
├── docs/
│   ├── system_design.md
│   └── hardware_setup.md
│
└── README.md
System Architecture

The robotics system typically consists of the following layers.

1. Hardware Layer

Includes:

Microcontroller (ESP32 / similar)

Motor drivers

Sensors

Communication modules

2. Firmware Layer

Embedded code running on the microcontroller responsible for:

Motor control

Sensor data collection

Communication protocols

3. Communication Layer

Provides remote access and control.

Possible methods include:

SSH

Web interface

Serial communication

4. Control Layer

Higher-level logic for:

Movement control

System coordination

Command interpretation

Hardware Requirements

Typical components used in this project include:

ESP32 / ESP32-S3 microcontroller

Motor drivers

DC motors

Power supply / battery

Communication interface (WiFi / serial)

Optional components:

Sensors

PWM expanders

Embedded Linux controller

Software Requirements

Development environment may include:

Arduino IDE / PlatformIO

Python

ROS2 (for robotics integration)

Git

Operating systems supported:

Linux

macOS

Windows

Installation

Clone the repository:

git clone https://github.com/Anant-on-git/vishwa.git

Navigate into the directory:

cd vishwa

Install dependencies if required:

pip install -r requirements.txt

For microcontroller firmware, open the project using:

Arduino IDE

PlatformIO

Usage
Running Embedded Firmware

Upload firmware to the microcontroller:

Select board: ESP32
Upload firmware
Running Control System

If a Python control interface exists:

python control.py
Remote Communication

SSH access may be enabled for remote device control.

Example:

ssh user@device_ip
Example Applications

Possible applications include:

Rover control systems

Remote robotics experiments

IoT based robot monitoring

Autonomous robotics research

Future Improvements

Planned improvements for the project:

ROS2 integration

Autonomous navigation algorithms

Multi-robot coordination

Mesh communication systems

AI based robotics control

Contribution

Contributions are welcome.

Steps:

Fork the repository

Create a new branch

Implement your feature

Submit a pull request

Author:

Kishor Sumb

Robotics developer focused on:

Embedded systems

AI + robotics

autonomous systems

experimental engineering
