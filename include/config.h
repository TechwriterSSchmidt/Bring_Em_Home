#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- System Configuration ---
#define SERIAL_BAUD         115200  // Serial Monitor Baud Rate

// --- Hardware Module Configuration ---
// Set to 1 to enable, 0 to disable connected hardware
#define HAS_COMPASS_MODULE  1       // 1 = Mounted, 0 = None
#define USE_BNO085          1       // 1 = BNO085 (Precision), 0 = BNO055 (Standard)
#define HAS_VIB_MOTOR       1       // 1 = Vibration Motor Enabled 
#define HAS_FLASHLIGHT      0       // 0 = Disabled
#define HAS_SOS_LED         0       // 0 = Disabled
#define HAS_RGB_LED         1       // 1 = NeoPixel/WS2812 connected

// --- Hardware Pins (Seeed Xiao nRF52840) ---

// I2C (OLED + BNO085)
#define PIN_I2C_SDA      D4  // P0.04 (SDA)
#define PIN_I2C_SCL      D5  // P0.05 (SCL)

// External I2C (Same pins for now if sharing bus)
#define PIN_EXT_SDA      PIN_I2C_SDA
#define PIN_EXT_SCL      PIN_I2C_SCL

// GPS Serial (Xiao UART)
#define PIN_GPS_RX       D7  // P1.12 (RX) -> Connect to GPS TX
#define PIN_GPS_TX       D6  // P1.11 (TX) -> Connect to GPS RX

// User Interface
#define PIN_BUTTON_1     D0  // P0.02
#define PIN_BUTTON       PIN_BUTTON_1 

// Feedback (Outputs)
#define PIN_VIB_MOTOR    D1  // P0.03
#undef PIN_NEOPIXEL          // Remove default definition
#define PIN_NEOPIXEL     D2  // P0.28

// Power Management
// Xiao nRF52840 reads battery via P0.31 (AIN7) using internal divider active-high
#define PIN_BAT_READ     P0_31 // AIN7
#define PIN_BAT_CHARGE   P0_17 // Status Pin (Low = Charging, High = Done/No Power)

// Mapping for compatibility
#define PIN_BAT_ADC      PIN_BAT_READ
#define PIN_BAT_READ_CTRL P0_14 // Active High to enable battery read divider

// Enable/Disable Features
// #define HAS_RGB_LED      1 // Defined in code via PIN_NEOPIXEL check, but can be explicit here

#define PIN_FLASHLIGHT      -1      // Disabled

// --- Power Management ---
#define BATTERY_CAPACITY_MAH 1000   // Battery Capacity in mAh (Adjust to your battery)
#define SOS_CURRENT_MA      220     // Estimated current consumption in SOS mode (mA) -> Kept for calculation logic usage if any, otherwise unused
#define CHARGE_CURRENT_MA   500     // Estimated charging current (mA) - usually 500mA for USB
#define DISPLAY_TIMEOUT     120000  // Display Auto-Off (ms) -> 2 Minutes
#define BAT_CHECK_INTERVAL_ACTIVE 60000  // Battery check interval when Display ON (ms)
#define BAT_CHECK_INTERVAL_IDLE   300000 // Battery check interval when Display OFF (ms)
#define CHARGE_CHECK_INTERVAL     3000   // Charge detection interval (ms)
#define CHARGE_CHECK_DURATION     300000 // Duration to check for charger after boot (ms) -> 5 Minutes

// --- Feature Settings ---
// User Data - All Personal Data / LoRa Features are REMOVED (Confidential/Unused)

// Navigation
#define DEFAULT_BREADCRUMB_DIST 25.0 // Default distance between breadcrumbs (m)
#define MAX_BREADCRUMBS     5000    // Max breadcrumbs (~80KB RAM usage, covers 125km @ 25m)
#define MIN_SPEED_KPH       0.5     // Minimum speed to record breadcrumb (prevents GPS drift)
#define MAX_SPEED_KPH       12.0    // Maximum realistic hiking speed (prevents GPS glitches)
#define BREADCRUMB_TURN_THRESHOLD 45.0 // Degrees change to trigger breadcrumb
#define BREADCRUMB_MIN_DIST_TURN  20.0 // Min distance to check for turn (meters)
#define ACCEL_MOTION_THRESHOLD 0.3  // Linear Acceleration threshold (m/s^2) to confirm movement

// Display Smart Wake
#define TILT_WAKE_ANGLE     35.0    // Angle (Pitch) to detect "Looking at screen" (< 35 deg from flat)
#define TILT_SLEEP_DELAY    2000    // Delay to turn off screen when arm dropped (ms) -> fast off
#define TILT_HYSTERESIS     10.0    // Hysteresis for tilt detection

// User Interface
#define CLICK_DELAY         500     // Max delay between clicks (ms)
#define LED_BRIGHTNESS      100     // NeoPixel Brightness (0-255) -> ~40%
#define DISPLAY_REFRESH_RATE 100    // Display refresh interval (ms)
#define LED_UPDATE_INTERVAL 20      // LED update interval (ms)
#define GPS_SEARCH_ANIM_INTERVAL 50 // GPS search animation speed (ms)
#define CHARGE_ANIM_INTERVAL 500    // Charging animation speed (ms)

// Button Timings
#define LONG_PRESS_DURATION 3000    // Long press to Power Off (ms)
#define VERY_LONG_PRESS_DURATION 10000 // Very long press to Reset Home (ms)
#define SOS_COUNTDOWN_DURATION 5000 // SOS Countdown duration (ms)

// Vibration Patterns
#define VIB_DURATION_SHORT  200     // Short vibration (ms)
#define VIB_DURATION_LONG   500     // Long vibration (ms)

// Feedback Durations
#define FEEDBACK_DURATION_SHORT 1000 // Short feedback message duration (ms)
#define FEEDBACK_DURATION_LONG  2000 // Long feedback message duration (ms)

// Hardware Constants (I2C Addresses & IDs)
// #define USE_BNO085          1       // Moved to Hardware Module Config above
#define BNO055_ID           55      // BNO055 Sensor ID
#define BNO055_ADDRESS      0x28    // BNO055 I2C Address
#define BNO085_ADDRESS      0x4A    // BNO085 I2C Address (Default 0x4A, Alt 0x4B)
#define I2C_SPEED_FAST      400000  // I2C Speed (400kHz)
#define GPS_BAUD            9600    // GPS Serial Baud Rate (Matek M10Q default)
#define NEOPIXEL_INDEX      0       // Index of the single NeoPixel

// LoRa config removed (Not used in this version)

// UI Layout
#define STATUS_BAR_LINE_Y   15      // Y-coordinate of the status bar separator
#define BATTERY_BAR_HEIGHT  8       // Max height of battery bar
#define BREADCRUMB_REACH_DIST 20.0  // Distance to consider breadcrumb reached (m)

// Display Power Management
#define DISPLAY_POWER_SAVE_ON  1    // Display OFF
#define DISPLAY_POWER_SAVE_OFF 0    // Display ON

#endif
