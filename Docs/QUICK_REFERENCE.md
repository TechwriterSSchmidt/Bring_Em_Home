# Quick Reference Guide - Bring Em Home

## 🚀 Quick Start (First Use)

1. **Power On**: Hold Button for **3 Seconds** (Vibration: Short-Short-Long).
2. **Wait for GPS**: Go outdoors and wait for satellite lock (1-5 min).
   - *Screen says: "Searching SATs"*
3. **Auto-Home**: The first valid GPS fix automatically sets your "Home" position.
   - *Feedback: "HOME SET!"*

## 🎯 Navigation Modes

### 1. Explore Mode (Default)
- **Display**: Shows Compass, Speed, and Distance from Home.
- **Function**: Records "Breadcrumbs" (Waypoints) automatically as you walk.
- **Action**: Enjoy your hike!

### 2. Return Mode (Backtrack)
- **Activate**: Double-Click Button.
- **Display**: Shows Arrow pointing to the next Breadcrumb (or Home).
- **Function**: Guides you back along your exact path.
- **Feedback**: Vibration when reaching a waypoint.

## 📱 Display Guide (OLED)

| Icon/Text | Meaning |
|-----------|---------|
| **House Icon** | Your starting point (Home). |
| **Arrow** | Direction to walk. Keep it pointing UP. |
| **Bars (Top R)** | GPS Signal Strength (1-4 bars). |
| **Bat %** | Battery Level. |
| **C:Good** | Compass Calibration Status. |

## 🔘 Button Functions

| Action | Gesture | Function |
|--------|---------|----------|
| **Toggle Display** | 1 Click | Screen ON/OFF (Saves Power) |
| **Switch Mode** | 2 Clicks | Explore ↔ Return Mode |
| **Flashlight** | 3 Clicks | Toggle LED Light |
| **SOS Mode** | 5 Clicks | Start SOS Countdown (LoRa Beacon) |
| **Power OFF** | Hold 3s | Deep Sleep (Vibration: Long-Short-Short) |
| **Reset Home** | Hold >10s | Set NEW Home Position (Overwrite old) |

## ⚡ Charging & Battery

- **Charging**: Connect USB-C. External LED (WS2812) pulses Green.
- **Full**: LED solid Green.
- **Low Battery**: LED flashes Yellow (< 10%).

## 📊 Technical Info

- **MCU**: nRF52840 (Heltec T114)
- **LoRa**: SX1262 (868 MHz)
- **GPS**: L76K GNSS
- **Compass**: BNO055 (I2C 0x28)
- **Display**: SH1107 (I2C 0x3C)
