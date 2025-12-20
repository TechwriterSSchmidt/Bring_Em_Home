# Hardware Setup Guide

## Reference Documentation
- **Official Schematic**: [Heltec Wireless Tracker V1.1 Schematic](https://resource.heltec.cn/download/Wireless_Tracker/Wireless_Tacker1.1/HTIT-Tracker_V0.5.pdf)

## Components List

### Main Components
1. **Heltec Wireless Tracker (V1.1)**
   - **Core**: ESP32-S3 + SX1262 LoRa + UC6580 GNSS
   - **Display**: Onboard 0.96" TFT (Optional use)
   - **Power**: Built-in LiPo Charger

2. **1.5" OLED Display**
   - **Driver**: SH1107
   - **Resolution**: 128x128 pixels
   - **Interface**: I2C (4-pin)

3. **Bosch BNO055 9-Axis IMU**
   - Absolute Orientation Sensor
   - I2C interface

### Peripherals
- **Button**: Waterproof momentary push button
- **Vibration Motor**: 3V coin vibration motor
- **LED**: High Power LED (with appropriate driver/transistor)

## Pinout Diagram

### Internal Components (No Wiring Needed)
- **GPS**: Connected internally (RX=34, TX=33)
- **LoRa**: Connected internally (SPI)

### I2C Bus (Shared: BNO055 Compass & SH1107 OLED)

```
Heltec Board          Devices (Parallel Connection)
============          =============================
GPIO 41 (SDA) <-----> BNO055 SDA & OLED SDA
GPIO 42 (SCL) <-----> BNO055 SCL & OLED SCL
3V3           ------> BNO055 VCC & OLED VCC
GND           ------> BNO055 GND & OLED GND
```

### Peripherals

```
Component      Heltec GPIO
=========      ===========
Button         GPIO 0 (PRG Button or External)
Vibration      GPIO 7
Flashlight     GPIO 5
Status LED     GPIO 18 (Internal NeoPixel)
```

## Assembly Instructions

### Step 1: Compass/IMU & OLED Connection

Connect the BNO055 and OLED via I2C to the Heltec Header:

1. Locate the I2C pins on the Heltec Board (SDA=41, SCL=42).
2. Connect both devices in parallel:
   ```
   Heltec SDA (41) → BNO055 SDA & OLED SDA
   Heltec SCL (42) → BNO055 SCL & OLED SCL
   Heltec 3V3      → BNO055 VCC & OLED VCC
   Heltec GND      → BNO055 GND & OLED GND
   ```
   ```

### Step 3: Power Connection

- Connect USB-C cable to ESP32-S3 board
- This provides both programming interface and power
- For portable use, consider a USB power bank

### Step 4: Physical Assembly

1. **Temporary Setup** (for testing):
   - Use breadboard and jumper wires
   - Keep GPS module elevated for better satellite reception

2. **Permanent Setup**:
   - Consider using a small project box or 3D printed enclosure
   - GPS antenna should be positioned on top for best reception
   - Keep compass away from magnetic interference
   - Mount display facing outward
   - Add strain relief for wires

## Testing Hardware Connections

### Initial Power-On Test

1. Connect USB cable
2. Display should light up and show boot screen
3. Check serial monitor at 115200 baud

### GPS Test

Expected serial output:
```
Bring Em Home - Starting...
GPS initialized
GPS: SEARCHING...
```

After 1-5 minutes outdoors:
```
GPS: LOCKED
GPS: 48.123456, 11.123456 | Heading: 45.0° | No home set
```

### Compass Test

Expected serial output during initialization:
```
HMC5883L detected!
```

If you see:
```
No HMC5883L detected
```

Check I2C connections and I2C address (default is 0x1E).

### Display Test

Display should show:
- Title "Bring Em Home" in yellow
- GPS status (red if searching, green if locked)
- Current coordinates
- Compass heading
- Instructions

## Troubleshooting Hardware

### GPS Not Working

**Symptoms**: "GPS: SEARCHING..." never changes

**Solutions**:
1. Verify GPS module has power (LED may blink)
2. Check UART wiring (TX/RX not swapped)
3. Move outdoors with clear sky view
4. Check baud rate (should be 9600)
5. Try swapping TX/RX if still not working

### Compass Not Working

**Symptoms**: "Compass: FAIL" on boot screen

**Solutions**:
1. Check I2C wiring
2. Verify I2C address with scanner sketch
3. Ensure 3.3V power (not 5V which may damage sensor)
4. Check for I2C pull-up resistors (usually built-in)

### Display Issues

**Symptoms**: Black screen or garbled display

**Solutions**:
1. Check backlight pin (GPIO 38)
2. Verify SPI connections
3. Check power supply (needs stable 5V)
4. Adjust display rotation in code if needed

### Magnetic Interference

**Symptoms**: Compass readings jump around or incorrect

**Solutions**:
1. Keep away from metal, motors, speakers
2. Don't mount near large batteries
3. Calibrate by moving in figure-8 pattern
4. Ensure wires are not creating magnetic loops

## Power Consumption

Typical current draw:
- Display on: ~120mA
- GPS active: ~40mA
- ESP32-S3: ~30mA
- Total: ~190mA @ 5V (~1W)

For portable use:
- 10,000mAh power bank: ~50 hours runtime
- 5,000mAh power bank: ~25 hours runtime

## Enclosure Design Considerations

When designing or selecting an enclosure:

1. **GPS Antenna**: Must have clear view of sky
   - Mount on top of enclosure
   - Use plastic/acrylic window above antenna
   - Avoid metal enclosures

2. **Display**: Accessible and visible
   - Front-facing mount
   - Consider anti-glare protection
   - Touchscreen requires exposed glass

3. **Buttons**: Easy access
   - Boot button should be accessible
   - Consider external buttons via GPIO

4. **Ventilation**: ESP32-S3 generates heat
   - Small ventilation holes
   - Don't seal completely

5. **Mounting**: For hiking use
   - Belt clip or carabiner attachment
   - Wrist strap consideration
   - Shock absorption padding

## Safety Considerations

⚠️ **Important Safety Notes**:

1. **GPS is NOT a replacement for map and compass skills**
2. Always carry backup navigation tools
3. Device requires clear sky view - won't work indoors/underground
4. Battery can die - have backup power or route plan
5. Accuracy is ~2-5 meters, not precise enough for technical navigation
6. Compass can be affected by magnetic interference
7. This is a navigational aid, not a safety device

## Advanced Modifications

### Adding External GPS Antenna

Some M100 modules support external antenna:
1. Connect to u.FL connector on module
2. Mount antenna on top of enclosure
3. Improves signal in difficult conditions

### Adding Battery Monitoring

Add voltage divider to monitor battery:
```
Battery+ → Resistor 10kΩ → GPIO 4 → Resistor 10kΩ → GND
```

Then read ADC to estimate battery level.

### Weatherproofing

For outdoor durability:
1. Use IP67 rated enclosure
2. Seal all wire entry points
3. Add silica gel desiccant inside
4. Use waterproof connectors

## Status LED Indicators (Internal WS2812)

The Heltec Wireless Tracker features a built-in WS2812 RGB LED (NeoPixel) on GPIO 18.

- **🔴 Pulsing Red:** Searching for GPS satellites.
- **🟡 Blinking Yellow (every 10s):** Low Battery warning.
- **⚫ Off:** Normal operation (GPS fixed, Battery OK) to save power.

## Bill of Materials (BOM)

| Item | Quantity | Est. Price |
|------|----------|------------|
| Waveshare ESP32-S3 Touch LCD 1.47" | 1 | $15-20 |
| HGLRC M100-5883 M10 GPS Module | 1 | $15-25 |
| Jumper Wires (F-F) | 8+ | $2-5 |
| USB-C Cable | 1 | $3-5 |
| Optional: Project Box | 1 | $5-10 |
| Optional: USB Power Bank | 1 | $10-20 |
| **Total** | | **~$50-85** |

## Recommended Tools

- Soldering iron (if soldering connections)
- Wire strippers
- Multimeter (for testing connections)
- Small screwdrivers
- Hot glue gun (for strain relief)

## Next Steps

After assembly:
1. Flash the firmware (see README.md)
2. Test indoors first (GPS won't lock, but system should boot)
3. Take outdoors for GPS lock test
4. Calibrate compass by moving in figure-8 pattern
5. Save home position and test navigation

For software setup and usage, see [README.md](README.md).
