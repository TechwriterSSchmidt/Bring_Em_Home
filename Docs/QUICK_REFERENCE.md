# Quick Reference Guide - Bring Em Home

## 🚀 Quick Start (First Use)

1. ⚡ Plug in USB-C cable (or ensure battery is charged)
2. ⏰ Wait for GPS lock (1-5 min outdoors)
   - *Screen says: "Waiting for GPS..."*
3. 📍 Long-Press Button (2s) to save home
4. ✅ Screen shows "Home Saved!"

## 🎯 Navigation

### Follow the Arrow Method
1. 👀 Look at the big arrow on the OLED
2. 🔄 Turn until arrow points **UP** ⬆️
3. 🚶 Walk straight ahead
4. 🏠 Distance counts down as you get closer

## 📱 Display Guide (OLED)

| Screen | Meaning |
|--------|---------|
| **Waiting...** | Searching for satellites. Go outside! |
| **Recording** | Tracking your hike. Shows Speed & Dist. |
| **Backtracking** | Guiding you home. Shows Arrow & Dist. |

## 🔘 Button Functions (GPIO 6)

| Action | Gesture | Function |
|--------|---------|----------|
| **Toggle Display** | 1 Click | Screen ON/OFF |
| **Switch Mode** | 2 Clicks | Explore ↔ Bring Me Home |
| **Flashlight** | 3 Clicks | Toggle LED Light |
| **SOS Mode** | 5 Clicks | Start SOS Countdown |
| **Reset Home** | Hold >10s | Set NEW Home (Explore Mode) |
| **OTA Update** | Hold 5s (Boot) | WiFi Update (White LED Pulse) |

## 📊 Technical Info

- **GPS**: Internal UC6580
- **LoRa**: Internal SX1262 (868 MHz)
- **Compass**: External BNO055 (I2C Addr 0x28)
- **Display**: External SH1107 (I2C Addr 0x3C)
│ Heading: 045°              │ ← Direction facing
│ HOME POSITION SET          │
│ Distance: 345 m            │ ← How far to home
│ Bearing: 270°              │ ← Direction to home
│              ↑             │ ← Arrow to home
│            Home            │
└────────────────────────────┘
```

## 🆘 Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| GPS won't lock | Go outdoors, clear sky view, wait 3-5 min |
| Arrow jumping | Stop for 30 sec, let GPS stabilize |
| Compass spinning | Move away from metal, recalibrate |
| Distance increasing | Turn around! Wrong direction |
| No home set | Press BOOT when GPS shows LOCKED |

## ⚠️ Important Tips

### ✅ DO:
- Wait for GPS LOCKED before saving home
- Check direction every 30-60 seconds
- Stop to let GPS update if uncertain
- Carry backup map & compass
- Keep device level for compass accuracy

### ❌ DON'T:
- Run (GPS can't keep up)
- Navigate indoors (no GPS signal)
- Hold near metal objects
- Trust 100% without backup navigation
- Ignore terrain obstacles

## 🔋 Battery Life

| Power Source | Runtime |
|--------------|---------|
| 10,000 mAh | ~50 hours |
| 5,000 mAh | ~25 hours |
| Typical day hike | 6-8 hours |

## 🧭 Compass Directions

```
        N (0°)
         ↑
         |
W (270°)─┼─ E (90°)
         |
         ↓
        S (180°)
```

## 📍 GPS Accuracy

| Conditions | Accuracy |
|------------|----------|
| Open field, 8+ sats | ±2m |
| Light trees, 6+ sats | ±5m |
| Forest, 4-5 sats | ±10-15m |
| Heavy forest/canyon | ±50m+ |

## 🔧 Calibrate Compass

1. Hold device level
2. Move in figure-8 pattern (30 sec)
3. Rotate in complete circle
4. Test against known direction

## 🌐 Understanding Coordinates

**Latitude**: North ↑ / South ↓ (-90 to +90)
- Positive = North of equator
- Negative = South of equator

**Longitude**: East → / West ← (-180 to +180)
- Positive = East of Prime Meridian
- Negative = West of Prime Meridian

## 🆘 If Lost and Device Fails

1. **STOP** - Don't wander
2. **THINK** - Where did you come from?
3. **OBSERVE** - Look for landmarks, trails
4. **PLAN** - Stay or self-rescue?
5. **SIGNAL** - Whistle, mirror, bright clothes

## 📐 Distance Units

- Shows in **kilometers** (km)

Example:
- `Distance: 0.85 km` = 850 meters
- `Distance: 2.45 km` = 2,450 meters

## 💡 Pro Tips

1. **Write down home coordinates** - backup if memory lost!
2. **Take photo of home location** - helps memory
3. **Mark waypoints on map** - cross-reference with GPS
4. **Check device before leaving** - ensure GPS locks at trailhead
5. **Bring power bank** - for multi-day hikes
6. **Keep device accessible** - check frequently

## 🏔️ Before Every Hike

- [ ] Device charged
- [ ] GPS locks at trailhead
- [ ] Home position saved
- [ ] Compass working
- [ ] Backup map & compass packed
- [ ] Someone knows your plans
- [ ] Emergency whistle & signal gear

## 📞 Emergency Numbers

Before hiking, know local emergency numbers:
- USA/Canada: 911
- Europe (international emergency number that also works in Switzerland): 112
- UK: 999
- Australia: 000

Add your local emergency number: ___________

## ⚖️ Remember

**This is a navigation AID, not a replacement for skills!**

- Always carry paper map & compass
- Know how to use traditional navigation
- GPS can fail (battery, weather, terrain)
- Device accuracy: ±2-5m (not inch-perfect)
- No substitute for experience and planning

---

## 🆘 Quick Emergency Reference

**If truly lost:**
1. STOP moving
2. Stay calm
3. Assess situation
4. Use device if working
5. Signal for help if needed
6. Stay put if uncertain

**Universal signals:**
- 3 whistle blasts = Help needed
- 3 fires in triangle = Distress
- Bright clothing = Visibility
- Mirror flashes = Attention

---

**Print this card and laminate it for field use!**

**Have a safe hike! 🏔️➡️🏠**
