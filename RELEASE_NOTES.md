# Release Notes - Bring Em Home

## [1.0.2] - 2026-01-08
### Changed
- **Hardware Configuration**: Split I2C bus to facilitate assembly.
    - **Bus 1 (Internal)**: OLED Display (Default Pins 16/13).
    - **Bus 2 (External)**: Compass Sensor (BNO085) moved to Header P2 (Pins 28/8).
- **GPS Pin Mapping**: Corrected GPS pin definitions based on T114 V2 schematic analysis (RX=37, TX=39, RST=38, WAKE=34).
- **Peripherals**: Temporarily disabled Vibration Motor and Flashlight due to pin conflicts with GPS.

## [1.0.1] - 2026-01-08
### Fixed
- **LoRa Init Failure**: Corrected `PIN_LORA_RST` from 25 to 18 to match Heltec Mesh Node T114 V2 hardware/firmware definitions.
- **Pin Conflict**: Removed `PIN_LORA_RST` from BNO085 constructor.
- **Watchdog**: Added WDT (10s) via nRF52 register access.

## Version 1.0.0 (Safety Update)
**Date:** January 2, 2026
**Status:** Stable / Safety Features Implemented

### Summary
This major update focuses on "Safety-First" features designed to prevent user error and ensure reliability in critical situations. It introduces a manual home confirmation step, a dedicated panic button, and persistent storage for the home location.

### New Features

#### Safety-First Home Logic
- **Problem Solved:** Prevents the device from accidentally overwriting a saved home location (e.g., the car) if the device is turned on mid-hike.
- **Mechanism:** Upon acquiring a GPS fix, the device enters `CONFIRM_HOME` mode.
    - **1 Click:** Set NEW Home (Current Location).
    - **2 Clicks:** Load SAVED Home (From Flash Memory).
- **Persistence:** The home location is now saved to the internal file system and persists across reboots.

#### Panic Button
- **Action:** Holding the button for **3-6 seconds** immediately activates **Return Mode**.
- **Behavior:** Bypasses all menus and guides the user back to the nearest breadcrumb or home.
- **Feedback:** Distinct vibration pattern and "RETURN MODE" message.

#### Compass Calibration Warning
- **Indicator:** Displays "CAL!" on the screen if the BNO085 reports low calibration status.
- **Benefit:** Ensures the user knows when the compass might be inaccurate.

#### Internal File System
- Integrated `Adafruit_LittleFS` to store configuration and state (Home Location) reliably.

### Changes
- **Power Off:** Hold time increased to **6 seconds** to accommodate the Panic Button function.
- **UI:** Added "SET HOME?" and "LOAD HOME?" prompts.

---

## Version 0.9.0 (Pre-Release)
**Date:** January 2, 2026
**Status:** Hardware Migration Complete / Feature Freeze for Testing

### Summary
This version concludes the migration from the ESP32 architecture to the nRF52840 (Heltec T114) platform. The codebase has been refactored to support new hardware sensors, improved power management, and the introduction of the "Buddy Tracking" feature.

### Hardware Support
- **MCU:** Heltec Mesh Node T114 (nRF52840)
- **GNSS:** Matek SAM-M10Q (u-blox M10) @ 9600 Baud
- **IMU:** Bosch BNO055 (I2C @ 400kHz) or BNO085 (High Precision)
- **Display:** SH1107 OLED (128x128)

### New Features

#### Buddy Tracking (Two-Way)
- Devices exchange position data via LoRa P2P in "Explore Mode".
- **Smart Power Sync:** Uses GPS time to synchronize transmit/receive windows, reducing power consumption by ~80% compared to continuous listening.
- A secondary, hollow arrow on the compass display indicates the direction to the partner device.
- Distance to the partner device is displayed in meters.
- **Configuration Required:** Each device must have a unique `BUDDY_DEVICE_ID` (0 or 1) set in `config.h`.

#### BNO085 Sensor Support
- Added driver support for the BNO085 IMU.
- Enabled via `USE_BNO085` in `config.h`.
- Utilizes "Stabilized Rotation Vector" for improved heading accuracy compared to the BNO055.

#### Hybrid SOS Connectivity
- **LoRa P2P (Default):** Configured for maximum range (SF12, 22dBm, CR 4/8).
- **LoRaWAN (Prepared):** Infrastructure for TTN integration added but currently disabled by default.
- **Fallback Logic:** System automatically reverts to P2P communication if LoRaWAN connection fails.

#### Power Management
- Implementation of Deep Sleep mode (< 50µA standby current).
- Voltage-based battery percentage estimation.
- Charging detection logic.

#### Navigation
- Smart Breadcrumb recording based on distance and directional changes.
- "Return to Home" directional arrow.
- Storage of compass calibration data in non-volatile memory.

### Resource Usage (nRF52840)
- **RAM:** ~5.1% (12,596 / 248,832 bytes)
- **Flash:** ~15.9% (129,928 / 815,104 bytes)

### Changes from Previous Versions
- Removal of legacy ESP32 and L76K GPS code.
- `platformio.ini` updated to target the `heltec_t114` environment.
- LoRa output power increased to 22dBm.

### Known Issues
- **LoRaWAN Keys:** `include/config.h` contains placeholder keys. These must be updated before enabling Hybrid Mode.
- **Field Testing:** Range testing of the SF12 LoRa configuration is pending.
- **Receiver:** Companion "Base Station" software is not yet available.
