# Line-Follower-Robot

This repository contains a complete implementation of a **line-following robot**, designed for both **educational** and **competitive** use. It includes:

- Code for **individual IR sensors** and **sensor arrays**
- Implementations in **Arduino** and **MicroPython**

Supports both **black** and **white line following**, using **PID control** and  now working on advanced AI algorithms like **neural networks**, **fuzzy logic**, and **genetic algorithms** for optimized behavior.

---

## Features

- **Individual IR Sensor Control**  
  PID-based line following using **5 IR sensors**.

- **Sensor Array Control**  
  Supports **5 or 7 sensor arrays** with optimized PID tuning for competition-level performance.

- **MicroPython Implementation**  
  Neural network-based control logic on **Raspberry Pi Pico**.


- **Modular Codebase**  
  Cleanly organized for easy customization and future extension.

---

## Hardware Requirements

### Microcontroller
- **Arduino Uno** or compatible (for Arduino-based implementation)
- **Raspberry Pi Pico** (for MicroPython-based implementation)

### Sensors
- **5x IR sensors** for individual sensor control
- **5 or 7 IR sensor array** for array-based PID control

### Motor Driver
- **TB6612FNG dual motor driver**
- **L298N dual motor driver**

### Motors
- **2x DC motors** (6V, 100–200 RPM recommended)

### Control Inputs
- **2x push buttons** (for calibration and start)
- **1x LED** (for status indication)

### Power Supply
- **7.4V–11.1V LiPo battery** or equivalent regulated source

### Chassis
- Compatible **robot chassis with wheels**

### Miscellaneous
- Breadboard
- Jumper wires
- **220Ω resistor** for LED

---

## 🔗 Circuit Connections

### Arduino Pinout

| Component         | Pin Assignment        |
|------------------|------------------------|
| IR Sensors        | A0–A4 (individual) / A0–A6 (array) |
| Motor Driver      | (AIN1 (4), AIN2 (3), PWMA (9), BIN1 (6), BIN2 (7), PWMB (10), STBY (5) for tb6612fng) / (E1 (5), I1 (6), I2 (7), I3 (8), I4 (9), E2(10) for individual_sensors/main_code.ino  |
| Buttons           | Calibrate: 11, Start: 12 |
| LED               | Pin 13 (with 220Ω resistor) |

### Raspberry Pi Pico Pinout

| Component         | Pin Assignment        |
|------------------|------------------------|
| IR Sensors        | GP26–GP28 (ADC0–ADC2) |
| Motor Driver      | AIN1 (GP2), AIN2 (GP3), PWMA (GP6), BIN1 (GP4), BIN2 (GP5), PWMB (GP7), STBY (GP8) |
| Buttons           | Calibrate: GP9, Start: GP10, Mode: GP11 |
| LED               | GP25 (with 220Ω resistor) |

---
