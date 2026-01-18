# Bring Em Home 🏠

A dedicated GPS navigation device designed to help Emilie safely find her way back to her starting location on every hike using GPS-breadcrumbs. It also serves as a LoRa-based buddy finder (already implemented) and SOS-flasher (future implementation planned) in case she needs help.

## Overview

This device uses GPS and compass sensors to find its way back to a stored “starting position.” It is ideal for hikers who want to be sure they can always find their way back to their starting point, e.g., in poor visibility, in pitch darkness, or if they get lost in bad weather.

## Table of Contents
- [Quick Start Guide](Docs/QUICK_REFERENCE.md)
- [Release Notes](RELEASE_NOTES.md)
- [Hardware Requirements](#hardware-requirements)
- [Hardware Connections (Heltec T114)](#hardware-connections-heltec-t114)
- [Power Management & Optimization](#power-management--optimization)
- [Performance & Stability](#performance--stability)
- [Features](#features)
- [Configuration](#configuration)
- [User Interface](#user-interface)
- [Power & Battery Life](#power--battery-life)
- [Release Notes](RELEASE_NOTES.md)
- [License](#license)

## Hardware Requirements

- **Heltec Mesh Node T114 (nRF52840 (V2))**
  - Ultra-low power nRF52840 MCU, SX1262 LoRa.
- **Quescan M10FD (u-blox M10) GNSS Module**
  - High-performance, ultra-low power GPS/Galileo/GLONASS/BeiDou receiver.
  - 25mm Patch Antenna for superior reception in forests.
- **1.5" OLED Display**
  - SH1107 Driver, 128x128 resolution, I2C interface.
- **Bosch BNO085 IMU**
  - High-precision VR-grade absolute orientation sensor (Recommended).
  - *Note: The M10FD often includes a QMC5883L compass, but the BNO085 is vastly superior for navigation due to sensor fusion.*
- **Peripherals (Optional)**
  - External Button
  - WS2812 RGB LED (Status Heartbeat)
  - Vibration Motor
  - *Note: Flashlight and SOS-LED are disabled and removed from this version.*

## Hardware Connections (Heltec T114)

### I2C Bus 1 (Internal: OLED Display)
- **SDA**: Pin 37 (P1.05) - *Use GPS Connector*
- **SCL**: Pin 39 (P1.07) - *Use GPS Connector*

### I2C Bus 2 (External: BNO085 Compass)
- **SDA**: Pin 28 (P0.28)
- **SCL**: Pin 30 (P0.30)
- **VCC**: 3.3V (Connect to VEXT or 3.3V)
- **GND**: GND

### GPS Module (Header P2)
- **RX**: Pin 8 (P0.08)
- **TX**: Pin 7 (P0.07)
- **RST**: Disconnected
- **WAKE**: Disconnected
- **PPS**: Disconnected

### Peripherals
- **Button**: Pin 31 (P0.31) - Active Low
- **Vibration Motor**: Pin 13 (P0.13) - Active High
- **Flashlight LED**: *Removed (Conflict/Power)*
- **WS2812 LED**: Pin 29 (P0.29) - External Status LED

## Power Management & Optimization

The device features an advanced power management system centered around the **nRF52840's Deep Sleep** capabilities and hardware-specific optimizations.

### Power Saving Strategy

To achieve **~75h+ runtime** (Eco Mode), we utilize specific hardware optimizations:

| Feature | Configuration | Optimization Benefit |
| :--- | :--- | :--- |
| **GPS Throttling** | **2 Hz** (500ms) | Powers down the M10FD RF stage between fixes (Cyclic Tracking). Savings: **~15mA**. |
| **Sensor Fusion** | **10 Hz** (100ms) | Reduces I2C interrupts on the nRF52 by 50%, allowing longer MCU sleep cycles. |
| **Buddy Sync** | **GPS Time-Slotted** | Synchronizes LoRa Rx windows with GPS time. The radio sleeps ~90% of the time and only wakes when a partner packet is expected. |
| **Smart Display** | **Auto-Off (5m)** | Automatically creates deep-sleep windows for the OLED (saving ~20mA) when not actively navigating. |
| **MCU Sleep** | **Idle Mode** | nRF52840 enters ultra-low power state (<50uA) whenever no interrupts are pending. |

### Estimated Battery Life (1500mAh LiPo)

Estimates with optimized refresh rates (GPS 2Hz, IMU 10Hz):

| Mode | Current Draw | Estimated Runtime |
|------|--------------|-------------------|
| **Hiking (Display Auto-Off)** | ~18 mA | **~83 Hours** |
| **Hiking (Display Always On)** | ~35 mA | **~42 Hours** |
| **Buddy Mode (Smart Sync)** | ~16 mA | **~90 Hours** |
| **Deep Sleep** | < 0.05 mA | > 1 Year |

*Note: The new u-blox M10 GPS (Configured to 2Hz) and BNO085 (Configured to 10Hz) significantly reduce power consumption.*

## Performance & Stability

- **Non-Blocking Logic**: All critical systems (LEDs, LoRa) use non-blocking state machines to ensure the main loop never freezes.
- **10Hz Display Limit**: The screen refresh rate is capped at 10Hz (100ms) to prioritize GPS data processing and prevent buffer overflows.
- **Sensor Startup**: Optimized boot sequence ensures sensors (BNO055, GPS) are fully powered before initialization.

## Features

- **Intuitive UI**: Intuitive graphical interface with GPS signal bars and battery status.
- **Smart Auto-Home-Position**: Automatically sets home position on power-up when GPS fix is found.
- **Crash Recovery**: Restores home position and breadcrumbs if device restarts due to software error.
- **Auto-Calibration Storage**: Automatically saves compass calibration data to Flash memory once fully calibrated, eliminating the need to recalibrate on every startup.
- Save home position manually with button press (Long Press > 10s)
- **Smart Breadcrumbs**: Records path points distance-based AND on significant direction changes (to capture turns).
- **Smart GPS Filter**: Ignores GPS drift when standing still or moving too fast (>12km/h).
- **Safety First**: Battery runtime estimation and Panic Button.
- **Panic Button**: Long press (3s) immediately activates Return Mode to guide you home.
- **Manual Home Confirmation**: Prevents accidental home resets.
- **Calibration Warning**: Displays "CAL!" if the compass needs calibration.
- **Charging Detection**: Automatically detects wall charger and shows charging progress.
- **Return Mode**: Backtracking mode to retrace steps.
- **Buddy Tracking**: Two-way location sharing with a partner device.
- Calculate distance to home/waypoint
- Digital compass (North Arrow)
- Visual navigation arrow (Target Arrow)
- Persistent storage of home position
- Low power consumption (Auto-off)
- **Visual Feedback**: RGB LED flashes for confirmations and warnings (replacing vibration).

## Future Roadmap / Planned Features
- **SOS Beacon / Flashlight**: Currently disabled and removed. Planned to be reintroduced with a miniature DC-SSR (Solid State Relay) for better efficiency later.
- **LoRaWAN Integration**: Infrastructure present but currently disabled.

## Configuration

You can customize the device settings in `include/config.h`. This includes hardware pins, timeouts, and **Active Hardware Modules**.

### Default Settings Table

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **Navigation** | | |
| `BREADCRUMB_DIST` | `100.0` m | Distance between regular breadcrumbs |
| `MAX_BREADCRUMBS` | `8500` | Max number of stored waypoints (~850km range) |
| `MIN_SPEED_KPH` | `1.0` km/h | Min speed to record (prevents GPS drift) |
| `MAX_SPEED_KPH` | `12.0` km/h | Max speed (prevents glitches) |
| `BREADCRUMB_TURN_THRESHOLD` | `45.0` ° | Angle change to trigger smart breadcrumb |
| `BREADCRUMB_MIN_DIST_TURN` | `20.0` m | Min distance to check for turns |
| **Power & Timing** | | |
| `DISPLAY_TIMEOUT` | `120000` ms | Display auto-off time (2 minutes) |
| `LORA_TX_INTERVAL` | `60000` ms | SOS Beacon interval (1 minute) |
| `BAT_CHECK_INTERVAL_ACTIVE` | `60000` ms | Battery check interval (Display ON) |
| `BAT_CHECK_INTERVAL_IDLE` | `300000` ms | Battery check interval (Display OFF) |
| `CHARGE_CHECK_INTERVAL` | `3000` ms | Charger detection polling rate |
| `CHARGE_CHECK_DURATION` | `300000` ms | Time to poll for charger after boot (5 mins) |
| **Buddy Tracking** | | |
| `BUDDY_DEVICE_ID` | `0` or `1` | Unique ID for time-slot synchronization |
| `ENABLE_SMART_POWER` | `1` | Enable GPS-synced power saving |

**User Data Options:**
- `USER_BLOOD_TYPE`: Your blood type (e.g., "A+").
- `USER_GENDER`: Your gender (e.g., "female"). *Optional: Comment out to disable.*
- `USER_BIRTH_YEAR`: Your birth year (e.g., 1992). The device calculates age automatically from GPS time. *Optional: Comment out to disable.*
- `USER_MED_ALLERGIES`: Medication allergies (e.g., "Penicillin"). *Optional: Comment out to disable.*

These details are included in the LoRa SOS beacon to assist rescue teams.

## User Interface

The device features a high-contrast OLED display designed for readability in sunlight.

### 1. Explorer Mode (Default)
Shows the direction to Home (or next waypoint), current compass heading, and distance.
- **Header**: Vertical Battery Icon, Compass Status (C:Ok), Satellite Signal.
- **Center**: Navigation Arrow (Points to destination) with 8 Cardinal Directions (N, NE, E, SE, S, SW, W, NW). Intermediate directions are shown as dots.
- **Footer**: "HOME" label (Left) and Distance in km (Right).

![Navigation Screen](Screens/mockup_nav.png)

### 2. Buddy Tracking (Overlay)
When a partner device is detected via LoRa P2P:
- **Buddy Arrow**: A secondary, hollow arrow appears on the compass ring, pointing towards your partner.
- **Distance**: The distance to your partner is displayed in the **Top Left** corner (e.g., "B: 150m").
- **Stale Data**: If the signal is older than 60 seconds, a "?" is appended (e.g., "B: 150m?").
- This overlay is active in **Explorer Mode**.

### 3. Breadcrumb Mode (Backtracking)
Activated by a **Double Click**. Guides you back to your starting point by following your recorded path in reverse.
- **Header**: Same as Explorer Mode.
- **Center**: Direction Arrow pointing to the **Next Waypoint**. (No cardinal directions shown).
- **Footer**: "WAYPOINT" label (Left) and Distance to it (Right).

![Breadcrumb Screen](Screens/mockup_breadcrumb.png)

### 4. Searching for GPS
Displayed when no GPS fix is available. Shows "Searching SATs" with an animated progress bar.

![Searching Screen](Screens/mockup_searching.png)

### 5. SOS Mode
*Feature currently disabled / removed.*

### 6. Charging Mode
Displayed when connected to a charger (Voltage > 4.4V). 
- Shows "Loading battery..." and an animated battery icon.
- Displays **Estimated Time to Full** (e.g., "Est: 1.5h").

![Charging Screen](Screens/mockup_charging.png)

### 7. OTA Update Mode
Activated by holding the button for **5 seconds** while powering on.
- Creates a WiFi Access Point: `Bring_Em_Home` (No Password).
- Connect and visit `192.168.4.1` to upload new firmware (`firmware.bin`) or filesystem (`littlefs.bin`).
- **Indicator**: The Status LED pulses **White**.

![OTA Screen](Screens/mockup_ota.png)

## Power & Battery Life

The device is optimized for long hikes using the ultra-low power **nRF52840** MCU and the efficient **u-blox M10** GPS.

**Settings:**
- **Display Timeout:** 120 seconds (configurable in `src/config.h`)
- **Status LED:** 40% Brightness
- **GPS Mode:** Continuous Tracking (High Precision)
- **LoRa SOS:** Transmits every 60 seconds (in SOS mode).

### Low Battery Warning
When the battery drops below 10%:
- **LED Signal**: The Status LED flashes yellow every 10 seconds.
- **Display**: The battery icon in the top-left corner appears empty.

## Software Setup

### Prerequisites
- [PlatformIO](https://platformio.org/) (recommended)
- USB drivers for nRF52840 (usually automatic on Win10/11)

## Usage

### Navigation

- The screen displays:
  - **Distance** to home (in meters or km). *Note: This is the total path distance along your breadcrumbs, not just "as the crow flies".*
  - **Direction Arrow** pointing towards home relative to your current heading
  - **Breadcrumb Count**: Number of auto-saved waypoints (saved every 250m)
- Follow the arrow to return to your starting point.

### Navigation Modes

The device has two modes, toggled by a **Double Click** on the button:

1.  **Explorer Mode (Default)**
    *   **Compass Arrow**: Points to **North**.
    *   **Display**: Shows distance to Home.
    *   **Action**: Automatically saves breadcrumbs every 250m.

2.  **Bring me home! Mode**
    *   **Target Arrow**: Points to the **Next Waypoint** or **Home**.
    *   **Display**: Shows distance to the target.
    *   **Action**: Guides you back along your path.

### Power Saving

- The display automatically turns off after **5 minutes** of inactivity.
- **Single Click** the button to toggle the display ON or OFF.
- Turning the display ON resets the 5-minute timer.

*> Note: Checkout the [Quick Reference](Docs/QUICK_REFERENCE.md) for more information.*


## Technical Details

### GPS Specifications
- Update rate: 1-10 Hz
- Accuracy: ~2.5m CEP
- Cold start time: ~30s
- Warm start time: ~2s

### Compass Specifications
- Resolution: 0.73 mGauss
- Range: ±8 Gauss
- Accuracy: 1-2° heading accuracy

## Future Enhancements (Planned)

- LoRaWAN SOS Beacon (Hybrid Mode with TTN integration)
- Add target coordinates via captive portal
- Add altitude display
- Speed and time estimates

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
