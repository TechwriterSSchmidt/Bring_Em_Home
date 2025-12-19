# Bring Em Home 🧭

A robust GPS navigation and safety device designed to help hikers find their way back to a starting location. Built on the **Heltec Wireless Tracker** platform for maximum reliability.

## Overview

This device uses GPS and a high-precision compass to guide the user back to a saved "home" position. It features a high-contrast OLED display for visibility in all lighting conditions and includes safety features like LoRa tracking and an SOS beacon.

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

## Features
- **GPS Navigation**: Breadcrumb tracking and "Return Home" arrow.
- **High-Contrast Display**: 128x128 OLED for excellent readability.
- **LoRa Connectivity**: Ready for long-range tracking (SX1262).
- **BLE Beacon**: Broadcasts "Emilie_Beacon" for close-range finding.
- **Flashlight**: High power LED mode.
- **SOS Signal**: Automatic SOS Morse code flashing.
- **Vibration Feedback**: Haptic feedback for interactions.
- **Fail-Safe**: Auto-restart on hardware failure.

## Getting Started

1. **Assemble Hardware**: Connect the OLED and BNO055 to the I2C pins (41/42).
2. **Flash Firmware**: Use PlatformIO to upload the code.
3. **Go Outside**: The device needs a clear view of the sky to get a GPS lock.
4. **Set Home**: Long-press the button to save your current location as "Home".
5. **Navigate**: Follow the arrow on the display to return.

## License
MIT License

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
   - The screen will flash **GREEN** and show "HOME SET!".
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
    *   **Green Arrow**: Points to **North** (Compass).
    *   **Display**: Shows distance to Home.
    *   **Action**: Automatically saves breadcrumbs every 250m.

2.  **Backtracking Mode (Return)**
    *   **Red Arrow**: Points to the **Next Waypoint** or **Home**.
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
    - **Green (Recording)**: North Indicator
    - **Red (Backtracking)**: Direction to Target

### Finding Your Way Home

1. **Double Click** the button to enter **Backtracking Mode** (Screen turns Blue briefly).
2. Follow the **Red Arrow**.
3. The device will guide you from waypoint to waypoint until you reach Home.

## Features

- ✅ Real-time GPS coordinate display
- ✅ **Smart Auto-Home**: Automatically sets home on power-up when GPS fix is found.
- ✅ **Crash Recovery**: Restores home position if device restarts due to software error.
- ✅ Save home position manually with button press (Long Press)
- ✅ Breadcrumb trail (auto-save every 250m)
- ✅ Backtracking mode to retrace steps
- ✅ Calculate distance to home/waypoint
- ✅ Digital compass (Green North Arrow)
- ✅ Visual navigation arrow (Red Target Arrow)
- ✅ Persistent storage of home position
- ✅ Satellite count display
- ✅ Low power consumption (Auto-off)

## Display Color Coding

- 🟡 **Yellow**: Title and home status
- 🟢 **Green**: Active GPS lock and navigation info
- 🔴 **Red**: GPS searching/no lock
- 🔵 **Cyan**: Coordinates
- 🟣 **Magenta**: Compass heading
- 🟠 **Orange**: Warnings/no home set

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

### Power Consumption
- Active (GPS + Display): ~150-200mA @ 5V
- Sleep mode: Not yet implemented

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
