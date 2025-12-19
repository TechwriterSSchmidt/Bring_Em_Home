# Bring Em Home 🧭

A dedicated GPS navigation device designed to guide Emilie safely back to her starting location on every hike using GPS-breadcrumbs. It also serves as an emergency LoRa-beacon and SOS-flasher in case she needs help.

## Overview

This device uses GPS and compass sensors to help navigate back to a saved "home" position. Perfect for hikers who want peace of mind knowing they can always find their way back to their starting point.

## Hardware Requirements

- **Heltec Wireless Tracker (V1.1)**
  - Integrated ESP32-S3, SX1262 LoRa, and UC6580 GNSS.
- **1.5" OLED Display**
  - SH1107 Driver, 128x128 resolution, I2C interface.
- **Bosch BNO055 9-Axis Absolute Orientation Sensor**
  - For precise compass heading.
- **Peripherals (Optional)**
  - Vibration Motor
  - High Power LED (Flashlight)
  - External Button

## Hardware Connections

The Heltec Wireless Tracker integrates most components, simplifying the wiring significantly.

### I2C Bus (Shared: BNO055 Compass & SH1107 OLED)
- **SDA**: GPIO 41
- **SCL**: GPIO 42
- **VCC**: 3.3V
- **GND**: GND

### Peripherals
- **Button**: GPIO 0 (Built-in "PRG" button or external)
- **Vibration Motor**: GPIO 7
- **Flashlight LED**: GPIO 5
- **Status LED (NeoPixel)**: GPIO 18 (Internal WS2812)

## Features

- ✅ Real-time GPS coordinate display
- ✅ **Smart Auto-Home**: Automatically sets home on power-up when GPS fix is found.
- ✅ **Crash Recovery**: Restores home position if device restarts due to software error.
- ✅ Save home position manually with button press (Long Press)
- ✅ Breadcrumb trail (auto-save every 250m)
- ✅ Backtracking mode to retrace steps
- ✅ Calculate distance to home/waypoint
- ✅ Digital compass (North Arrow)
- ✅ Visual navigation arrow (Target Arrow)
- ✅ Persistent storage of home position
- ✅ Satellite count display
- ✅ Low power consumption (Auto-off)

## 🔋 Power & Battery Life

The device is optimized for long hikes. It uses dynamic CPU frequency scaling (240MHz active / 80MHz eco) and auto-display timeout.

**Settings:**
- **Display Timeout:** 120 seconds (configurable in `src/config.h`)
- **Status LED:** 40% Brightness
- **CPU:** Auto-throttling when display is off.
- **LoRa SOS:** Transmits every 60 seconds (in SOS mode).

| Battery Capacity | Hiking Mode (Avg. Consumption ~90mA) | SOS Mode (Avg. Consumption ~220mA) |
| :--- | :--- | :--- |
| **1500 mAh** | **~14 Hours** | **~5.5 Hours** |
| **2000 mAh** | **~19 Hours** | **~7.5 Hours** |
| **3000 mAh** | **~28 Hours** | **~11 Hours** |

*> Note: Estimates include a 15% safety margin for converter losses and battery aging.*

## Getting Started

1. **Assemble Hardware**: Connect the OLED and BNO055 to the I2C pins (41/42).
2. **Flash Firmware**: Use PlatformIO to upload the code.
3. **Go Outside**: The device needs a clear view of the sky to get a GPS lock.
4. **Set Home**: Long-press the button to save your current location as "Home".
5. **Navigate**: Follow the arrow on the display to return.

## Software Setup

### Prerequisites
- [PlatformIO](https://platformio.org/) (recommended) or Arduino IDE
- USB drivers for ESP32-S3

### Building with PlatformIO

1. Clone this repository:
   ```bash
   git clone https://github.com/TechwriterSSchmidt/Bring_Em_Home.git
   cd Bring_Em_Home
   ```

2. Build the project:
   ```bash
   pio run
   ```

3. Upload to the ESP32-S3:
   ```bash
   pio run --target upload
   ```

4. Monitor serial output:
   ```bash
   pio device monitor
   ```

### Building with Arduino IDE

1. Install the following libraries via Library Manager:
   - TinyGPSPlus
   - GFX Library for Arduino (MoonOnOurNation)
   - Adafruit BNO055
   - Adafruit Unified Sensor

2. Select Board: "ESP32S3 Dev Module"
3. Set partition scheme to default
4. Open `src/main.cpp` and upload

## Usage

### First Time Setup

1. **Power On**: Switch the device ON using the physical switch.
2. **Wait for GPS**: The device will automatically search for satellites.
3. **Auto-Save**: As soon as a valid GPS fix is obtained, the current location is **automatically saved as Home**.
   - The screen will show **"HOME SET!"**.
   - No button press is required.

### Navigation

- The screen displays:
  - **Distance** to home (in meters or km)
  - **Direction Arrow** pointing towards home relative to your current heading
  - **Breadcrumb Count**: Number of auto-saved waypoints (saved every 2.5 minutes)
- Follow the arrow to return to your starting point.

### Navigation Modes

The device has two modes, toggled by a **Double Click** on the button:

1.  **Recording Mode (Default)**
    *   **Compass Arrow**: Points to **North**.
    *   **Display**: Shows distance to Home.
    *   **Action**: Automatically saves breadcrumbs every 250m.

2.  **Backtracking Mode (Return)**
    *   **Target Arrow**: Points to the **Next Waypoint** or **Home**.
    *   **Display**: Shows distance to the target.
    *   **Action**: Guides you back along your path.

### Power Saving

- The display automatically turns off after **5 minutes** of inactivity.
- **Single Click** the button to toggle the display ON or OFF.
- Turning the display ON resets the 5-minute timer.

### Navigation Indicators

The display shows:
- **GPS Status**: Satellite count
- **Coordinates**: Real-time Lat/Lon at the bottom
- **Compass Heading**: Current direction (0-360°)
- **Distance**: To Home or Next Waypoint
- **Visual Arrow**: 
    - **Recording Mode**: North Indicator
    - **Backtracking Mode**: Direction to Target

### Finding Your Way Home

1. **Double Click** the button to enter **Backtracking Mode** (Screen shows "RETURN").
2. Follow the **Arrow**.
3. The device will guide you from waypoint to waypoint until you reach Home.

## Display Layout

- **Top Bar**: Status Icons (SOS, Light), Satellite Count, Calibration
- **Center**: Main Navigation Info (Arrow, Distance)
- **Bottom**: Coordinates / Status Messages

## Troubleshooting

### GPS Not Locking
- Ensure you're outdoors with clear view of sky
- GPS may take 1-5 minutes for initial lock
- Avoid urban canyons, dense forests, or indoor use
- Check GPS module connections

### Compass Reading Incorrect
- Keep device away from magnetic materials (metal, speakers, motors)
- Calibrate by moving device in figure-8 pattern
- Check I2C connections to HMC5883L

### Display Not Working
- Check power supply (USB should provide 5V)
- Verify backlight is enabled
- Check SPI connections

### Home Position Not Saving
- Ensure GPS has valid lock before saving
- Check that flash memory is not full
- Try power cycling the device

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

## Future Enhancements

- [ ] Add waypoint system (multiple saved locations)
- [ ] Track and display hiking path
- [ ] Battery percentage display
- [ ] Power saving sleep mode
- [ ] Touch screen interface for settings
- [ ] Log coordinates to SD card
- [ ] Add altitude display
- [ ] Speed and time estimates
- [ ] Breadcrumb trail feature

## Contributing

Feel free to open issues or submit pull requests for improvements!

## License

This project is open source. Feel free to use and modify as needed.

## Credits

Created to help Emilie find her way home on hikes! 🏔️➡️🏠
