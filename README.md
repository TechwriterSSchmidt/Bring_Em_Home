# Bring Em Home 🏠

A dedicated GPS navigation device designed to help user safely find their way back to their starting location on every hike using GPS-breadcrumbs. 

## Overview

This device uses GPS and compass sensors to find its way back to a stored “starting position.” It is ideal for hikers who want to be sure they can always find their way back to their starting point, e.g., in poor visibility, in pitch darkness, or if they get lost in bad weather.

## Table of Contents
- [Quick Start Guide](Docs/QUICK_REFERENCE.md)
- [Release Notes](RELEASE_NOTES.md)
- [Hardware Requirements](#hardware-requirements)
- [Hardware Connections (SuperMini nRF52840)](#hardware-connections-supermini-nrf52840)
- [Power Management & Optimization](#power-management--optimization)
- [Features](#features)
- [License](#license)

## Hardware Requirements

- **SuperMini nRF52840** (Nice!Nano form factor compat.)
  - Tiny, powerful nRF52840 MCU.
- **Quescan M10FD (u-blox M10) GNSS Module**
  - High-performance, ultra-low power GPS/Galileo/GLONASS/BeiDou receiver.
- **1.5" OLED Display**
  - SH1107 Driver, 128x128 resolution, I2C interface.
- **Bosch BNO085 IMU**
  - High-precision VR-grade absolute orientation sensor.
- **Lithium Polymer Battery**

## Hardware Connections (SuperMini nRF52840)

See [Docs/SCHEMATIC.txt](Docs/SCHEMATIC.txt) for detailed wiring diagrams.

### I2C Bus (OLED + Compass)
- **SDA**: P0.22
- **SCL**: P0.20

### GPS Module
- **RX**: P0.08
- **TX**: P0.06

### User Interface
- **Button**: P0.15 (Main User Input)
- **Vibration Motor**: P0.13
- **RGB LED**: P0.17 (Internal)

## Power Management & Optimization

The device features an advanced power management system centered around the **nRF52840's Deep Sleep** capabilities.

### Power Saving Strategy

| Feature | Configuration | Optimization Benefit |
| :--- | :--- | :--- |
| **GPS Throttling** | **1-2 Hz** | Powers down the RF stage between fixes. |
| **Sensor Fusion** | **10 Hz** | Reduces I2C traffic to minimum needed for smooth UI. |
| **Smart Display** | **Auto-Off** | Automatically creates deep-sleep windows for the OLED when not actively navigating. |
| **File System** | **LittleFS** | Efficiently stores breadcrumbs to Flash memory (persists through reboot). |

### Estimated Runtimes

Calculated based on average power consumption of ~15mA (Screen Off, GPS Active) and ~35mA (Screen On, Active Navigation).

| Battery Capacity | Screen Off (Tracking Only) | Screen On (Navigating) | Deep Sleep (Standby) |
| :--- | :--- | :--- | :--- |
| **1000 mAh** | ~66 Hours | ~28 Hours | > 1 Year |
| **1200 mAh** | ~80 Hours | ~34 Hours | > 1 Year |
| **1500 mAh** | ~100 Hours | ~42 Hours | > 1 Year
## Features

- **Compass Navigation**: Tilt-compensated heading to home/waypoint.
- **Breadcrumb Trail**: Automatically drops digital breadcrumbs to trace your path back.
- **Persistent Storage**: Never lose your home point or path if power fails (saved to Internal Flash).
- **Return Logic**: Guidance back to the nearest previous point, then the next, retracing your steps.
- **Simple UI**: Designed for ease of use under stress (Panic Button, High Contrast).

## License

This project is licensed under the **PolyForm Noncommercial License 1.0.0**.

- **Noncommercial Use**: You may use this software for personal, educational, or evaluation purposes.
- **Commercial Use Restricted**: You may NOT use this software for commercial purposes (selling, paid services, business use) without prior written consent.

<details>
<summary>View Full License Text</summary>

### PolyForm Noncommercial License 1.0.0

#### 1. Purpose
This license allows you to use the software for noncommercial purposes.

#### 2. Agreement
In order to receive this license, you must agree to its rules. The rules of this license are both obligations (like a contract) and conditions to your license. You must not do anything with this software that triggers a rule that you cannot or will not follow.

#### 3. License Grant
The licensor grants you a copyright license for the software to do everything you might do with the software that would otherwise infringe the licensor's copyright in it for any permitted purpose. However, you may only do so to the extent that such use does not violate the rules.

#### 4. Permitted Purpose
A purpose is a permitted purpose if it consists of:
1. Personal use
2. Evaluation of the software
3. Development of software using the software as a dependency or evaluation tool
4. Educational use

**Commercial use is strictly prohibited without prior written consent from the author.**

#### 5. Rules

##### 5.1. Noncommercial Use
You must not use the software for any commercial purpose. A commercial purpose includes, but is not limited to:
1. Using the software to provide a service to third parties for a fee.
2. Selling the software or a derivative work.
3. Using the software in a commercial environment or business workflow.

##### 5.2. Notices
You must ensure that anyone who gets a copy of any part of the software from you also gets a copy of these terms or the URL for them above, as well as copies of any copyright notice or other rights notice in the software.

#### 6. Disclaimer
**THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.**

</details>
