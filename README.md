# Vishwa

## Overview
**Vishwa** is a competition-focused robotics development repository created for the **Vishwa VJTI team**.  
It contains code, experiments, and system components used in the development of our rover for **IRC 2025**.

The repository focuses on building robust robotics systems combining:

- Embedded systems
- Remote communication
- Control algorithms
- Hardware integration

It serves as a development platform for robotics experiments, rover systems, and automation tools.

---

## Objectives

### 1. Robotics Development (IRC 2025 Rover)
Develop and implement control systems for our rover participating in **International Rover Challenge 2025 (IRC 25')**.

### 2. Embedded Systems Integration
Interface microcontrollers such as **ESP32 / ESP32-S3** with motors, sensors, and external hardware modules.

### 3. Remote Communication
Enable remote control and monitoring of robotic systems using protocols such as:

- SSH
- Web interfaces
- Serial communication

### 4. Experimental Robotics Platform
Provide a sandbox environment for testing robotics concepts and rapid prototyping.

---

## Repository Structure

```
vishwa/
│
├── firmware/
│   ├── esp32/
│   └── microcontroller_code/
│
├── communication/
│   ├── ssh_control/
│   └── networking/
│
├── control_system/
│   ├── motor_control/
│   └── navigation/
│
├── hardware/
│   ├── schematics/
│   └── board_notes/
│
├── docs/
│   ├── system_design.md
│   └── hardware_setup.md
│
└── README.md
```

---

## System Architecture

The robotics platform is structured into four major layers.

### 1. Hardware Layer
Physical components used in the rover system.

Includes:
- Microcontroller (ESP32 / ESP32-S3)
- Motor drivers
- Sensors
- Communication modules

---

### 2. Firmware Layer
Embedded software running on the microcontroller responsible for:

- Motor control
- Sensor data acquisition
- Communication protocol handling

---

### 3. Communication Layer
Handles remote connectivity and command transmission.

Supported interfaces may include:

- SSH
- Web-based control interface
- Serial communication

---

### 4. Control Layer
Higher-level logic responsible for:

- Movement control
- System coordination
- Command interpretation
- Navigation logic

---

## Hardware Requirements

Typical hardware used in the system:

- ESP32 / ESP32-S3 microcontroller
- Motor drivers
- DC motors
- Battery / power management system
- WiFi or serial communication modules

Optional components:

- Sensors (IMU, encoders, cameras)
- PWM expanders
- Embedded Linux controller

---

## Software Requirements

Development environment may include:

- **Arduino IDE / PlatformIO**
- **Python**
- **ROS2** (for robotics integration)
- **Git**

Supported operating systems:

- Linux
- macOS
- Windows

---

## Installation

Clone the repository:

```bash
git clone https://github.com/Anant-on-git/vishwa.git
```

Navigate into the project directory:

```bash
cd vishwa
```

Install Python dependencies (if required):

```bash
pip install -r requirements.txt
```

For firmware development, open the firmware directory using:

- Arduino IDE
- PlatformIO

---

## Usage

### Running Embedded Firmware

Upload the firmware to the microcontroller:

1. Select board: **ESP32 / ESP32-S3**
2. Compile the firmware
3. Upload to the device

---

### Running Control System

If a Python control interface is available:

```bash
python control.py
```

---

### Remote Communication

Access the device remotely using SSH:

```bash
ssh user@device_ip
```

---

## Example Applications

This repository can support development of:

- Rover control systems
- Remote robotics experiments
- IoT-based robot monitoring
- Autonomous robotics research

---

## Future Improvements

Planned enhancements include:

- ROS2 integration
- Autonomous navigation algorithms
- Multi-robot coordination
- Mesh communication systems
- AI-driven robotics control

---

## Contribution

Contributions are welcome.

1. Fork the repository  
2. Create a feature branch  

```bash
git checkout -b feature-name
```

3. Commit your changes  
4. Submit a pull request  

---

## Author

**Kishor Sumb**

Robotics developer focused on:

- Embedded systems
- AI + Robotics
- Autonomous systems
- Experimental engineering
