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
- **Button 1**: P0.15 (Main)
- **Button 2**: P0.24 (Aux)
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

## Features

- **Compass Navigation**: Tilt-compensated heading to home/waypoint.
- **Breadcrumb Trail**: Automatically drops digital breadcrumbs to trace your path back.
- **Persistent Storage**: Never lose your home point or path if power fails (saved to Internal Flash).
- **Return Logic**: Guidance back to the nearest previous point, then the next, retracing your steps.
- **Simple UI**: Designed for ease of use under stress (Panic Button, High Contrast).

## License

PolyForm Noncommercial License 1.0.0.
