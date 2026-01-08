# Quick Reference Guide - Bring Em Home

## 🚀 Quick Start (First Use)

1. **Power On**: Hold Button for **3 Seconds** (Vibration: Short-Short-Long).
2. **Wait for GPS**: Go outdoors and wait for satellite lock (1-5 min).
   - *Screen says: "Searching SATs"*
3. **Confirm Home**: When GPS is found, the screen asks "SET HOME?".
   - **1 Click**: Set **NEW** Home (Current Location).
   - **2 Clicks**: Load **SAVED** Home (e.g., Car).

## 🎯 Navigation Modes

### 1. Explore Mode (Default)
- **Display**: Shows Compass, Speed, and Distance from Home.
- **Buddy Tracking**: Shows a hollow arrow pointing to your partner if they are nearby.
- **Function**: Records "Breadcrumbs" (Waypoints) automatically as you walk.
- **Action**: Enjoy your hike!

### 2. Return Mode (Backtrack)
- **Activate**: Menu (1 Click) -> Select 'SWITCH MODE' (Hold).
- **OR**: Panic Button (Hold 3s) for emergency return.
- **Display**: Shows Arrow pointing to the next Breadcrumb (or Home).
- **Function**: Guides you back along your exact path.
- **Feedback**: Vibration when reaching a waypoint.

## 📱 Display Guide (OLED)

| Icon/Text | Meaning |
|-----------|---------|
| **House Icon** | Your starting point (Home). |
| **Solid Arrow** | Direction to Home/Waypoint. Keep it pointing UP. |
| **Hollow Arrow** | Direction to your Buddy (Partner Device). |
| **B: 120m** | Distance to Buddy (Top Left). "?" means old data. |
| **Bars (Top R)** | GPS Signal Strength (1-4 bars). |
| **Bat %** | Battery Level. |
| **C:Good** | Compass Calibration Status. |

## 🔘 Button Functions (Single Button Menu)

| Action | Gesture | Function |
|--------|---------|----------|
| **Wake Display** | 1 Click | Turn Screen ON |
| **Open Menu** | 1 Click (When ON) | Open Options Menu (Bottom of Screen) |
| **Navigate Menu** | 1 Click | Cycle options: Mode -> Light -> Power Off -> SOS |
| **Select Option** | Hold Button | Execute selected menu option |
| **Panic Mode** | Hold 3s (No Menu) | **IMMEDIATE** Return Mode (Safety Override) |
| **SOS** | Menu -> SOS -> Hold 10s | Start SOS Beacon (Requires explicit hold) |
| **Confirm Home** | 1x / 2x Clicks | Only at Startup: 1x=New Home, 2x=Load Saved |

## ⚡ Charging & Battery

- **Charging**: Connect USB-C. External LED (WS2812) pulses Green.
- **Full**: LED solid Green.
- **Low Battery**: LED flashes Yellow (< 10%).

## 📊 Technical Info

- **MCU**: nRF52840 (Heltec T114)
- **LoRa**: SX1262 (868 MHz)
- **GPS**: Heltec L76K GNSS Module
- **Compass**: BNO055 (I2C 0x28)
- **Display**: SH1107 (I2C 0x3C)
