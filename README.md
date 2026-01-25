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

PolyForm Noncommercial License 1.0.0

Copyright (c) 2026 Bring Em Home Contributors

1. Rights Granted

The Licensor grants to each Recipient all rights required to use the
Software and other Content, on a royalty-free basis, for any purpose
other than a Commercial Use.

2. Limitations on Grant

The Licensor does not grant any rights for Commercial Use.

3. Conditions

The Licensor grants Rights only on the condition that:

    a. The Recipient must ensure that anyone who receives the Software
       or other Content from the Recipient also receives a copy of this
       Agreement or the URL <https://polyformproject.org/licenses/noncommercial/1.0.0>,
       and

    b. If the Recipient makes any changes to the Software, they must
       license those changes under the terms of this Agreement.

4. Definitions

"Commercial Use" means any use of the Software or other Content by a
business, or for a commercial purpose, including use by any other person
on behalf of a business or for a commercial purpose.

"Content" means copyrightable material, including software and
documentation.

"Licensor" means the individual or entity offering the Content under
this Agreement.

"Recipient" means anyone who receives the Content under this Agreement.

"Software" means the computer software offered under this Agreement.

5. Disclaimer

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
