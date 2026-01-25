# Quick Reference Guide - Bring Em Home

## 🚀 Quick Start (First Use)

1. **Power On**: Hold Button 1 for **3 Seconds** (Vibration feedback).
2. **Wait for GPS**: Go outdoors and wait for satellite lock (1-5 min).
   - *Screen says: "Searching SATs"*
   - *Status LED*: Red Pulse.
3. **Confirm Home**: When GPS is found, the screen asks "SET HOME?".
   - **Click Button 1**: Set **NEW** Home (Current Location).
   - **Click Button 2**: Load **SAVED** Home (e.g., Car).

## 🎯 Navigation Modes

### 1. Explore Mode (Default)
- **Display**: Shows Compass, Speed, and Distance from Home.
- **Function**: Records "Breadcrumbs" (Waypoints) automatically as you walk.
- **Feedback**: Green LED Flash (every 4s) indicates recording.

### 2. Return Mode (Backtrack)
- **Activate**: Menu -> Select 'SWITCH MODE'.
- **OR**: Panic Button (Hold Button 1 for 3s) for emergency return.
- **Display**: Shows Arrow pointing to the next Breadcrumb (or Home).
- **Function**: Guides you back along your exact path.
- **Feedback**: Vibration when reaching a waypoint.

## 📱 Display Guide (OLED)

| Icon/Text | Meaning |
|-----------|---------|
| **House Icon** | Your starting point (Home). |
| **Solid Arrow** | Direction to Home/Waypoint. Keep it pointing UP. |
| **Bars (Top R)** | GPS Signal Strength (1-4 bars). |
| **Bat %** | Battery Level. |
| **C:Good** | Compass Calibration Status. |

## 🔘 Button Functions

**Button 1 (Main Action)**
| Action | Gesture | Function |
|--------|---------|----------|
| **Wake Display** | 1 Click | Turn Screen ON |
| **Set Waypoint** | 1 Click (When ON) | Manually mark a point of interest |
| **Panic Mode** | Hold 3s | **IMMEDIATE** Return Mode (Safety Override) |
| **Confirm Home** | 1 Click | (Startup only) Set New Home |

**Button 2 (Auxiliary)**
| Action | Gesture | Function |
|--------|---------|----------|
| **Cycle Screens** | 1 Click | Switch between Compass/Stats views |
| **Open Menu** | Hold | Open Options Menu |
| **Load Saved** | 1 Click | (Startup only) Load Previous Home |

## ⚡ Charging & Battery

- **Charging**: Connect USB-C.
- **Low Battery**: LED flashes Yellow (< 10%).

## 💡 Status LED (Screen Off)

Even when the display is visible to save power, the RGB LED gives feedback:
- **Green Flash (every 4s)**: Explorer Mode is active, GPS is fixed, Breadcrumbs are recording.
- **Red Pulse**: Searching for GPS satellites.
- **Yellow Flash**: Low Battery Warning.

## 📊 Technical Info

- **MCU**: SuperMini nRF52840
- **GPS**: Quescan M10FD (u-blox M10)
- **Compass**: BNO085 (I2C)
- **Display**: SH1107 (I2C)
- **Battery**: LiPo with charging circuit
