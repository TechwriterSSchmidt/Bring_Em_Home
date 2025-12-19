# Bring Em Home 🧭

A GPS navigation device to guide Emilie back to her starting location when lost on hikes.

## Overview

This device uses GPS and compass sensors to help navigate back to a saved "home" position. Perfect for hikers who want peace of mind knowing they can always find their way back to their starting point.

## Hardware Requirements

- **ESP32-S3 Development Board** with [Waveshare ESP32-S3 1.47" Touch LCD Display](https://www.waveshare.com/esp32-s3-touch-lcd-1.47.htm)
- **HGLRC M100-5883 M10 GPS Module** with integrated HMC5883L compass
- Connecting wires
- USB cable for programming and power

## Hardware Connections

### GPS Module (UART Connection)
- GPS TX → ESP32-S3 GPIO 18 (RX)
- GPS RX → ESP32-S3 GPIO 17 (TX)
- GPS VCC → 5V
- GPS GND → GND

### Compass/Magnetometer (I2C Connection)
The HMC5883L compass is integrated in the GPS module:
- SDA → ESP32-S3 GPIO 8
- SCL → ESP32-S3 GPIO 9
- VCC → 3.3V
- GND → GND

### Display
The Waveshare ESP32-S3 Touch LCD display is built into the board:
- Pre-wired to the following pins:
  - CS: GPIO 10
  - DC: GPIO 13
  - RST: GPIO 14
  - MOSI: GPIO 11
  - SCLK: GPIO 12
  - Backlight: GPIO 38

### Buttons
- **BOOT Button (GPIO 0)**: Save current position as home
- Built-in on ESP32-S3 board

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
   - Adafruit GFX Library
   - Adafruit ST7735 and ST7789 Library
   - Adafruit BusIO
   - Adafruit Unified Sensor
   - Adafruit HMC5883 Unified

2. Select Board: "ESP32S3 Dev Module"
3. Set partition scheme to default
4. Open `src/main.cpp` and upload

## Usage

### First Time Setup

1. Power on the device
2. Wait for GPS to acquire satellite lock (may take 1-5 minutes outdoors with clear sky view)
3. Once GPS shows "LOCKED" on screen, press the **BOOT button** to save your current position as "home"
4. The screen will flash green and show "HOME SAVED!"

### Navigation

The display shows:
- **GPS Status**: LOCKED or SEARCHING
- **Current Coordinates**: Your current latitude/longitude
- **Compass Heading**: Current direction you're facing (0-360°)
- **Home Position**: Saved home coordinates
- **Distance to Home**: How far you are from home (meters/kilometers)
- **Bearing to Home**: Direction to travel to reach home (0-360°)
- **Visual Arrow**: Points towards home position (relative to your current heading)

### Finding Your Way Home

1. Look at the green arrow on the right side of the display
2. Turn your body until the arrow points straight up (north on compass)
3. Walk in that direction
4. The distance and arrow will update as you move
5. When distance reaches 0, you're home!

## Features

- ✅ Real-time GPS coordinate display
- ✅ Save home position with button press
- ✅ Calculate distance to home (Haversine formula)
- ✅ Calculate bearing to home
- ✅ Digital compass with heading display
- ✅ Visual arrow pointing towards home
- ✅ Persistent storage of home position (survives power cycles)
- ✅ Satellite count display
- ✅ Color-coded display for easy reading
- ✅ Low power consumption

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
