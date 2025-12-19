# Troubleshooting Guide

This guide helps you diagnose and fix common issues with the Bring Em Home GPS navigation device.

## Table of Contents
- [Build & Upload Issues](#build--upload-issues)
- [Hardware Issues](#hardware-issues)
- [GPS Issues](#gps-issues)
- [Compass Issues](#compass-issues)
- [Display Issues](#display-issues)
- [Navigation Issues](#navigation-issues)
- [Performance Issues](#performance-issues)

---

## Build & Upload Issues

### PlatformIO Build Fails

**Symptom**: `pio run` command fails with errors

**Common Causes & Solutions**:

1. **Missing platform**: 
   ```bash
   pio platform install espressif32
   ```

2. **Library dependency issues**:
   ```bash
   pio lib install
   ```

3. **Cache issues**:
   ```bash
   pio run --target clean
   rm -rf .pio
   pio run
   ```

### Upload Fails

**Symptom**: Cannot upload to ESP32-S3

**Solutions**:
1. Hold BOOT button while clicking upload
2. Check USB cable (data cable, not just power)
3. Install/update USB drivers:
   - Windows: Install CH340 or CP2102 drivers
   - Linux: Add user to dialout group: `sudo usermod -a -G dialout $USER`
   - Mac: Install USB serial drivers

4. Check port selection:
   ```bash
   pio device list
   ```

5. Try different USB port
6. Reset ESP32-S3 after upload starts

### Compilation Errors

**Error**: `Arduino.h: No such file or directory`
- **Solution**: Ensure espressif32 platform is installed

**Error**: Library not found (e.g., TinyGPSPlus)
- **Solution**: Check `platformio.ini` lib_deps section
- Run: `pio lib install`

**Error**: Board not found
- **Solution**: Verify board name in `platformio.ini`: `esp32-s3-devkitc-1`

---

## Hardware Issues

### Device Won't Power On

**Symptoms**: No display, no lights, completely dead

**Check**:
- [ ] USB cable connected firmly
- [ ] USB cable is data cable (not just power)
- [ ] USB port provides sufficient power (500mA+)
- [ ] Try different USB cable
- [ ] Try different USB port or power source
- [ ] Check for physical damage

### Device Resets/Reboots Randomly

**Causes**:
1. **Insufficient power**: Use quality USB cable/power source
2. **Loose connections**: Check all wire connections
3. **Short circuit**: Inspect wiring for shorts
4. **Overheating**: Ensure ventilation, check for excessive current draw

**Solutions**:
- Use USB power bank with 2A+ output
- Secure all connections with solder or crimps
- Add ferrite bead on USB cable to reduce noise
- Monitor serial output for crash logs

### Getting Very Hot

**Normal**: ESP32-S3 warm to touch during operation
**Problem**: Too hot to touch comfortably

**Solutions**:
- Ensure adequate ventilation
- Check for short circuits
- Verify power supply voltage (should be 5V, not higher)
- Add small heatsink to ESP32-S3 chip

---

## GPS Issues

### GPS Never Locks ("SEARCHING..." Forever)

**Requirements for GPS Lock**:
- Clear view of sky (outdoors)
- No obstructions overhead
- 4+ satellites visible
- Can take 1-5 minutes for first lock

**Troubleshooting Steps**:

1. **Verify GPS is receiving power**:
   - Check 5V connection
   - LED on GPS module should blink

2. **Check wiring**:
   ```
   GPS TX → ESP32 GPIO 18 (RX1)
   GPS RX → ESP32 GPIO 17 (TX1)
   ```
   - Try swapping TX/RX if not working

3. **Verify GPS is sending data**:
   - Connect to serial monitor (115200 baud)
   - Should see NMEA sentences if GPS is transmitting

4. **Test GPS module separately**:
   ```cpp
   void loop() {
     while (gpsSerial.available()) {
       Serial.write(gpsSerial.read());
     }
   }
   ```
   - Should see output like: `$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47`

5. **Environmental issues**:
   - Move to open field
   - Get away from buildings
   - Wait 5 minutes without moving
   - Check weather (heavy clouds affect GPS)

6. **Module issues**:
   - Try external antenna if available
   - Check GPS module LED (should blink indicating satellites)
   - Module may need firmware update (rare)

### GPS Lock Then Loses Signal

**Causes**:
- Moved under obstruction (trees, buildings, tunnels)
- Wiring issues causing intermittent connection
- Power supply problems
- GPS module overheating

**Solutions**:
- Keep GPS antenna facing upward
- Secure wire connections
- Use stable power source
- Add small heatsink to GPS module

### GPS Accuracy Very Poor (50m+ error)

**Normal Accuracy**: ±2-5 meters with good signal
**Poor Accuracy**: ±10-50+ meters

**Causes**:
- Multipath (signal bouncing off buildings)
- Poor satellite geometry (all satellites in one direction)
- Atmospheric interference
- Moving too fast
- Module in poor location

**Solutions**:
- Wait for more satellites (6+ preferred)
- Move to open area
- Stay still for 30 seconds to let GPS stabilize
- Check satellite count on display
- Consider external antenna

### GPS Coordinates Frozen

**Symptom**: Display shows coordinates but they don't update

**Check**:
1. Serial monitor - are new NMEA sentences coming in?
2. GPS Valid flag - is GPS reporting valid fix?
3. Code logic - is location being updated?

**Debug Code**:
```cpp
// Add to loop()
if (gps.location.isUpdated()) {
  Serial.println("GPS updated!");
} else {
  Serial.println("GPS not updating");
}
```

---

## Compass Issues

### Compass Not Detected ("Compass: FAIL")

**Symptom**: Boot screen shows "Compass: FAIL"

**Troubleshooting**:

1. **Check I2C wiring**:
   ```
   HMC5883L SDA → ESP32 GPIO 8
   HMC5883L SCL → ESP32 GPIO 9
   VCC → 3.3V (NOT 5V!)
   GND → GND
   ```

2. **Run I2C scanner**:
   ```cpp
   void setup() {
     Wire.begin(8, 9);
     Serial.begin(115200);
     
     Serial.println("I2C Scanner");
     for(byte i = 0; i < 127; i++) {
       Wire.beginTransmission(i);
       if(Wire.endTransmission() == 0) {
         Serial.printf("Found device at 0x%02X\n", i);
       }
     }
   }
   ```
   - Should find HMC5883L at address `0x1E`

3. **Check power voltage**:
   - HMC5883L requires 3.3V (5V may damage it!)
   - Measure voltage at VCC pin

4. **Verify I2C pull-ups**:
   - Most modules have built-in pull-ups
   - If not, add 4.7kΩ resistors from SDA/SCL to 3.3V

### Compass Reading Incorrect

**Symptom**: Heading doesn't match actual direction

**Causes**:
- Not calibrated
- Magnetic interference
- Mounting angle incorrect
- Hard iron or soft iron interference

**Calibration Process**:
1. Hold device level
2. Rotate slowly 360° horizontally
3. Tilt and rotate in figure-8 pattern
4. Keep away from metal during calibration

**Interference Sources**:
- Metal objects (keys, belt buckle, tools)
- Speakers and magnets
- Electric motors
- Power cables with high current
- Steel-frame buildings
- Cars and metal structures

**Solutions**:
- Keep compass 5cm+ from other electronics
- Mount compass on extension if needed
- Add magnetic shielding (mu-metal)
- Perform calibration in same environment as use

### Compass Reading Jumpy/Erratic

**Symptom**: Heading value jumps around wildly

**Causes**:
1. **Magnetic interference**: See above
2. **Poor I2C connection**: Loose wires
3. **Electrical noise**: From display, GPS, or power supply
4. **Module damaged**: Physical damage to sensor

**Solutions**:
- Check/resolder I2C connections
- Add 0.1µF capacitor across VCC and GND at sensor
- Move compass farther from noise sources
- Add ferrite beads on I2C lines
- Use shielded cable for I2C

### Compass Stuck at One Value

**Symptom**: Heading never changes even when rotating

**Debug**:
```cpp
void loop() {
  sensors_event_t event;
  mag.getEvent(&event);
  Serial.printf("X: %.2f  Y: %.2f  Z: %.2f\n", 
    event.magnetic.x, event.magnetic.y, event.magnetic.z);
  delay(100);
}
```

**If values don't change**:
- Sensor is damaged or in wrong mode
- I2C communication issue
- Need to re-initialize sensor

**If values are all zero**:
- Sensor not communicating
- Check wiring and I2C address

---

## Display Issues

### Display Stays Black

**Checklist**:
- [ ] Power connected
- [ ] Backlight enabled (GPIO 38)
- [ ] Display initialization code runs
- [ ] No error in serial monitor
- [ ] Try adjusting contrast/brightness

**Test backlight**:
```cpp
pinMode(38, OUTPUT);
digitalWrite(38, HIGH); // Should make screen light up (white)
```

**Test display**:
```cpp
tft.fillScreen(ST77XX_RED);
delay(1000);
tft.fillScreen(ST77XX_GREEN);
```

### Display Shows Garbage/Random Pixels

**Causes**:
- Wrong display driver
- Incorrect SPI speed
- Loose connections
- Wrong pin configuration

**Solutions**:
1. Verify pin definitions match Waveshare board
2. Try lower SPI speed in initialization
3. Check all SPI connections
4. Ensure correct rotation setting

### Display Very Dim

**Solutions**:
- Check backlight is on (GPIO 38 HIGH)
- Verify power supply voltage (should be 5V)
- Screen may have adjustable brightness in code

### Text Cut Off or Misaligned

**Issues**:
- Wrong rotation setting
- Wrong screen resolution
- Coordinates calculated for wrong size

**Fix**:
```cpp
tft.setRotation(1); // Try values 0-3
// Verify dimensions
#define SCREEN_WIDTH  172
#define SCREEN_HEIGHT 320
```

### Display Update Slow/Laggy

**Causes**:
- Drawing too much too fast
- No update limiting
- Complex graphics

**Current code has 500ms update interval**:
```cpp
const unsigned long DISPLAY_UPDATE_INTERVAL = 500;
```

Increase this value for slower updates, decrease for faster (but more CPU usage).

---

## Navigation Issues

### Arrow Points Wrong Direction

**After recent fixes**: Arrow should now normalize angles correctly

**If still wrong**:
1. Check compass calibration
2. Verify GPS has valid fix
3. Ensure bearing calculation is correct
4. Check that heading and bearing are both in 0-360° range

**Debug Output**:
```cpp
Serial.printf("Heading: %.1f  Bearing: %.1f  Relative: %.1f\n", 
  heading, bearingToHome, relativeAngle);
```

### Distance Never Decreases

**Symptoms**: Walking toward home but distance stays same or increases

**Causes**:
1. GPS not updating (frozen)
2. Walking wrong direction
3. GPS accuracy poor (large error)
4. Obstacle requires detour

**Check**:
- Is GPS showing "LOCKED"?
- Are coordinates on screen changing?
- Is satellite count 4+?
- Stop for 30 seconds and check again

### Can't Save Home Position

**Symptom**: Press BOOT button but nothing happens

**Checklist**:
- [ ] GPS shows "LOCKED" status?
- [ ] Satellite count 4+?
- [ ] Button actually pressed (feel click)?
- [ ] Serial monitor shows "Home saved" message?

**Debug**:
```cpp
// In loop(), add:
Serial.printf("Button state: %d\n", digitalRead(BTN_SAVE));
// Should show 1 when released, 0 when pressed
```

### Home Position Lost After Power Cycle

**Should NOT happen**: Home is saved to flash memory

**If it does**:
1. Check serial monitor for save confirmation
2. Preferences storage may be corrupted
3. Flash memory may be failing

**Workaround**: Manually set home in code:
```cpp
void setup() {
  // ... other setup code ...
  
  // Manual home position override
  homeLat = 48.123456;  // Your coordinates
  homeLon = 11.123456;
  homeSet = true;
}
```

---

## Performance Issues

### Device Laggy/Unresponsive

**Causes**:
- Too many display updates
- Blocking delays in code (should be fixed now)
- GPS processing too slow
- Memory issues

**Check**:
- Recent code fixes removed blocking delays
- Monitor serial output for crash/reset messages
- Check memory usage

**Optimization**:
```cpp
// Reduce display update frequency
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000; // 1 second
```

### Serial Monitor Shows Garbage

**Cause**: Wrong baud rate

**Solution**: Set to 115200 baud in serial monitor

### Memory Errors/Crashes

**Symptoms**: Random resets, garbage on screen, freezing

**ESP32-S3 has plenty of RAM**, but check:
```cpp
Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
```

Should have 100KB+ free. If low:
- Reduce string buffers
- Limit number of GPS sentences processed
- Reduce display complexity

---

## Advanced Debugging

### Enable Debug Output

Add to code:
```cpp
#define DEBUG 1

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif
```

Then use:
```cpp
DEBUG_PRINTLN("GPS updated");
DEBUG_PRINT("Heading: ");
DEBUG_PRINTLN(heading);
```

### Monitor Resource Usage

```cpp
void loop() {
  // ... normal code ...
  
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 5000) {
    Serial.printf("Free heap: %d, Loop time: %dms\n", 
      ESP.getFreeHeap(), millis() - lastDebugTime);
    lastDebugTime = millis();
  }
}
```

### Test Individual Components

Create separate test sketches:
- GPS only
- Compass only  
- Display only

This isolates problems to specific hardware.

---

## Getting More Help

If none of these solutions work:

1. **Check serial monitor output** - Often shows the exact error
2. **Take photos** of your wiring and setup
3. **Document the issue**:
   - What you expected
   - What actually happened
   - Steps to reproduce
4. **Open GitHub issue** with:
   - Hardware used
   - Complete error messages
   - Photos of setup
   - What you've tried

## Common Error Messages

| Error | Meaning | Solution |
|-------|---------|----------|
| `Guru Meditation Error` | ESP32 crashed | Check serial output for stack trace |
| `Brownout detector was triggered` | Voltage dropped too low | Better power supply |
| `Flash read err, 1000` | Flash memory error | Re-flash firmware |
| `rst:0x10 (RTCWDT_RTC_RESET)` | Watchdog timeout | Code is blocking too long |
| `assertion failed` | Internal error | Check recent code changes |

---

**Still stuck? Open an issue on GitHub with detailed information!**
