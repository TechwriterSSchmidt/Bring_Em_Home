# Usage Guide - Bring Em Home

This guide will walk you through using your GPS navigation device for the first time and on your hikes.

## Quick Start

### First Power-On

1. **Connect Power**: Plug in the USB-C cable to the ESP32-S3 board
2. **Wait for Boot**: The display will show "Bring Em Home" and initialization status
3. **GPS Acquisition**: Wait for GPS to lock (outdoor required, 1-5 minutes)
4. **Compass Check**: Verify compass is working (heading should change as you rotate)

### Setting Your Home Position

**Automatic Mode:**
1. Turn the device **ON** at your starting point (Trailhead).
2. Wait for GPS lock.
3. The device will automatically save the first valid position as **HOME**.
   - Screen flashes **GREEN** and shows "HOME SET!".

**Note on Power Cycling:**
- Turning the device **OFF** via the physical switch will **RESET** the home position logic.
- The next time you turn it ON, it will try to set a **NEW** home position.
- If the device restarts due to a software error (Crash/Watchdog), it will **RESTORE** the previous home position.

### Compass Calibration
The BNO055 sensor calibrates itself automatically during movement.
- **Status Indicator**: A small "CAL: M:x" (0-3) is shown on the top bar.
- **To Calibrate**: Move the device in a figure-8 motion until the status shows 3 (Green).

## Controls

The device is operated by a single button (GPIO 14). The software includes debouncing to prevent accidental triggers.

| Action | Gesture | Function |
|--------|---------|----------|
| **Toggle Display** | 1 Click | Turns screen ON/OFF to save power. |
| **Switch Mode** | 2 Clicks | Toggles between **Explorer Mode** (Recording) and **Bring Me Home Mode** (Backtracking). |
| **Flashlight** | 3 Clicks | Toggles the High Power LED flashlight. |
| **SOS Signal** | 5 Clicks | Starts 5s Countdown, then activates SOS Morse code & LoRa Beacon. |
| **Reset Home** | Hold >10s | **(Explorer Mode only)** Overwrites the current Home position with your current location. Useful if you started tracking too early (e.g. at the bus stop). |

## Understanding the Display

### Screen Layout

The display uses a graphical interface to show all important information at a glance.

**Top Bar (Status):**
- **Left**: Status Icons
  - `L`: Flashlight is ON
  - `SOS`: SOS Mode Active (shows countdown to next LoRa beacon)
    - *Note: The LoRa message includes your coordinates, battery level, and blood type.*
- **Center**: Mode Title
  - `EXPLORER`: Recording path, waiting to set Home.
  - `BRING ME HOME!`: Navigating back to start.
- **Right**: 
  - **Battery**: Current charge in %
  - **GPS Signal**: 5-bar signal strength indicator (like a phone)
  - **M:x**: Compass Calibration Status (0=Uncalibrated, 3=Fully Calibrated)

**Center Area (Navigation):**
- **Satellite Dish Animation**: Shown when searching for GPS signal.
- **Navigation Arrow**: 
  - Points the way to your destination (Home).
  - **Up**: Walk straight.
  - **Left/Right**: Turn.
  - **Down**: Turn around.
  - In `EXPLORER` mode, the arrow acts as a compass (North indicator).
  - **Compass Dots**: Small dots indicate intermediate directions (NE, SE, SW, NW).

**Charging Screen:**
- Appears automatically when a charger is connected (>4.4V).
- Shows "Loading battery..." and a progress bar.
- LED pulses green.

**SOS Screen:**
- **Countdown**: Shows "SOS IN 5..." allowing cancellation.
- **Active**: Shows "SOS ACTIVE" and "SENDING..." when transmitting LoRa.
- **Runtime**: Displays estimated battery runtime (e.g., "Est: 4.5h").

**Bottom Area (Info):**
- **Distance**: Distance to Home (in meters or km).
- **Label**: "HOME" (Destination) or "WAITING GPS".

### Status Indicators

**GPS Signal Bars**:
- **0 Bars**: No GPS fix.
- **1-2 Bars**: Weak signal (low accuracy).
- **3-5 Bars**: Strong signal (high accuracy).

**Home Status**:
- In `EXPLORER` mode, the device automatically sets the Home position once GPS is locked.
- When `BRING ME HOME!` is active, the arrow guides you back to that position.

## Navigation Instructions

### Basic Navigation

1. **Check GPS**: Ensure you have signal bars (at least 2-3).
2. **Check Mode**: Ensure you are in `BRING ME HOME!` mode.
3. **Follow Arrow**: Turn your body until the arrow points **UP**.
4. **Walk**: Walk in that direction. The distance should decrease.

### Step-by-Step Navigation

**Method 1: Using the Arrow (Easiest)**

1. Look at the green arrow on screen
2. Turn your body until the arrow points straight up
3. Walk in that direction
4. Keep checking - arrow will update as you move
5. When distance shows < 10m, you've arrived!

**Method 2: Using Compass & Bearing**

1. Note the **Bearing** (e.g., 270° = West)
2. Turn until **Heading** matches **Bearing**
3. Walk straight, maintaining that heading
4. Re-check every few minutes as you get closer

**Example**:
```
Distance: 850 m
Bearing: 315°
Heading: 045°

→ Need to turn left/counterclockwise
→ Target heading is 315° (Northwest)
→ Walk until heading shows ~315°
```

### Tips for Accurate Navigation

✅ **DO**:
- Keep device level for accurate compass readings
- Wait for GPS lock before starting navigation
- Check your direction every 30-60 seconds
- Stop and let GPS update if you're uncertain
- Walk at a steady pace for better GPS tracking

❌ **DON'T**:
- Run (GPS updates take time)
- Hold device near metal objects
- Navigate in thick forest canopy if possible
- Trust navigation in urban areas (buildings cause GPS errors)
- Navigate through caves or tunnels (no GPS signal)

## Real-World Navigation Scenarios

### Scenario 1: Lost on a Trail

**Situation**: You've wandered off trail and aren't sure which way to go.

**Steps**:
1. Stop moving and find a clearing
2. Wait for GPS to stabilize (30 seconds)
3. Check distance to home
4. Note the bearing to home
5. Look at surrounding landmarks
6. Follow the arrow, checking every minute
7. If distance increases, stop and re-check

### Scenario 2: Taking a Detour

**Situation**: The direct route has an obstacle (river, cliff, dense brush).

**Steps**:
1. Note current distance and bearing
2. Navigate around obstacle
3. Keep checking that distance is generally decreasing
4. Don't worry if bearing changes - just keep distance going down
5. Once past obstacle, resume direct navigation

### Scenario 3: Multiple Stops

**Situation**: You want to explore multiple areas but always return to camp.

**Steps**:
1. Save home at your camp (BOOT button)
2. Explore freely
3. To return: follow arrow back to camp
4. To set a new home: save new position at new location
5. Previous home will be overwritten

## Battery Management

### Checking Battery Life

The device draws ~190mA at 5V:
- 10,000mAh power bank: ~50 hours
- 5,000mAh power bank: ~25 hours
- Typical day hike: 6-8 hours → plenty of battery

### Extending Battery Life

Currently the device is always on. Future updates may add:
- Screen dimming
- GPS power-save mode
- Sleep mode between checks

For now, bring an adequate power bank for your hike duration.

## Troubleshooting While Hiking

### GPS Shows "SEARCHING" Forever

**Causes**:
- Dense tree cover
- Canyon walls blocking sky view
- Bad weather (heavy clouds)
- Device in pocket/backpack

**Solutions**:
- Move to clearing
- Get to higher ground
- Hold device up toward sky
- Wait 2-3 minutes without moving

### Compass Acting Erratic

**Causes**:
- Near metal objects (belt buckle, keys, phone)
- Magnetic interference from electronics
- Not calibrated

**Solutions**:
- Move device away from metal
- Turn off other electronics nearby
- Wave device in figure-8 pattern several times
- Use bearing number instead of arrow

### Arrow Spinning or Jumping

**Causes**:
- Poor GPS signal
- Moving too fast
- GPS multipath (urban areas)

**Solutions**:
- Stop moving for 30 seconds
- Move to open area
- Use bearing number for navigation
- Trust distance more than arrow

### Distance Not Decreasing

**Causes**:
- Walking wrong direction
- GPS not updating (no signal)
- Obstacle in direct path

**Solutions**:
- Stop and verify arrow direction
- Check if GPS shows LOCKED
- Consider if terrain requires detour
- Re-check after 1-2 minutes

### Home Position Lost

**Causes**:
- Power cycled without saving
- Memory corruption (rare)

**Solutions**:
- Home is saved to flash memory, should survive power cycle
- If lost, you'll need to rely on memory or map
- Always carry backup navigation tools!

## Safety Guidelines

### Before You Hike

✅ **Pre-Trip Checklist**:
- [ ] Device fully charged
- [ ] Power bank charged (if multi-day)
- [ ] Tested GPS lock at home
- [ ] Saved home position at trailhead
- [ ] Verified compass works
- [ ] Backup map and compass packed
- [ ] Someone knows your plans

### During Your Hike

⚠️ **Remember**:
- This device is an AID, not a replacement for skills
- Always carry paper map and compass
- GPS can fail: battery, clouds, terrain
- Accuracy is ±2-5 meters, not inch-perfect
- Signal can be lost in valleys, caves, forests
- Device is not waterproof (unless you added enclosure)

### Emergency Situations

If device fails or you're truly lost:

1. **STOP**: Don't wander aimlessly
2. **THINK**: Where did you come from?
3. **OBSERVE**: Look for landmarks, trails, water
4. **PLAN**: Decide to wait or self-rescue
5. **Use device** if still working, but don't rely solely on it

Emergency Contact: Always carry emergency whistle and signal mirror!

## Advanced Usage

### Calibrating the Compass

If compass seems off:

1. Hold device level
2. Move in figure-8 pattern for 30 seconds
3. Rotate slowly in complete circle
4. Test by pointing device at known direction (e.g., toward sun at noon = South)

### Understanding GPS Accuracy

GPS accuracy depends on:
- **Excellent** (±2m): Open field, 8+ satellites, clear sky
- **Good** (±5m): Light tree cover, 6+ satellites
- **Fair** (±10-15m): Moderate forest, 4-5 satellites
- **Poor** (±50m+): Heavy forest, canyon, bad weather

### Reading Coordinates

Format: Decimal Degrees (DD)
- **Latitude**: North/South position (-90 to +90)
  - Positive = North of equator
  - Negative = South of equator
  - Example: 48.123456 = ~48.12° North

- **Longitude**: East/West position (-180 to +180)
  - Positive = East of Prime Meridian
  - Negative = West of Prime Meridian
  - Example: -122.123456 = ~122.12° West

You can enter these coordinates into any mapping app for reference.

## Maintenance

### Regular Care

- Wipe screen gently with soft cloth
- Keep connections clean and dry
- Store in dry place when not in use
- Avoid extreme temperatures

### After Each Use

- Clean any dirt or moisture
- Check wire connections
- Recharge batteries
- Verify device still boots correctly

### Long-Term Storage

- Store at room temperature
- Keep battery at 50-70% charge
- Check every few months
- Update software as available

## Frequently Asked Questions

**Q: How accurate is the distance measurement?**
A: Within 2-10 meters typically, depending on GPS signal quality.

**Q: Does it work worldwide?**
A: Yes! GPS works globally. Compass works everywhere too.

**Q: Can I save multiple home positions?**
A: Currently only one home position. Future updates may add waypoints.

**Q: What if I accidentally overwrite home?**
A: Write down your home coordinates! Then you can manually enter them (requires code modification currently).

**Q: Does it work indoors?**
A: No. GPS requires clear view of sky. Won't work in buildings, caves, or dense forest canopy.

**Q: How long does GPS take to lock?**
A: 30 seconds to 5 minutes, depending on conditions. First use may take longer.

**Q: Can I use it for geocaching?**
A: Yes! Save the geocache location as home, navigate to it.

**Q: Is it waterproof?**
A: No, unless you've added a waterproof enclosure. Avoid rain.

**Q: Can I track my path?**
A: Not currently. Future enhancement possibility.

## Getting Help

If you encounter issues:

1. Check troubleshooting section above
2. Verify hardware connections (see HARDWARE.md)
3. Check serial monitor output for debugging
4. Review README.md for setup issues
5. Open issue on GitHub repository

## Legal Disclaimer

This device is provided as-is for educational and recreational use. 

**NOT FOR**:
- Professional navigation
- Search and rescue operations  
- Life-safety applications
- Aviation or maritime navigation

Always carry proper maps, compass, and know how to use them. Device failure is possible and should be planned for. User assumes all risk.

---

**Have a safe hike! May you always find your way home! 🏔️➡️🏠**
