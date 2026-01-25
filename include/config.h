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

// --- Hardware Pins (SuperMini nRF52840) ---

// I2C (OLED + BNO085)
#define PIN_I2C_SDA      22  // P0.22
#define PIN_I2C_SCL      20  // P0.20

// GPS Serial
#define PIN_GPS_RX       8   // P0.08 (Conn to GPS TX)
#define PIN_GPS_TX       6   // P0.06 (Conn to GPS RX)

// User Interface
#define PIN_BUTTON_1     15  // P0.15 (Long Press: Home, Short: Waypoint)
#define PIN_BUTTON       PIN_BUTTON_1 // Mapping for legacy code convenience

// Feedback (Outputs)
#define PIN_VIB_MOTOR    13  // P0.13 (Vibration Motor)
#define PIN_NEOPIXEL     29  // P0.29 (External WS2812 Data)

// Power Management
#define PIN_BAT_VOLT     4   // P0.04 (AIN2)
#define PIN_BAT_CHARGE   5   // P0.05 (Charging Status)

// Enable/Disable Features
// #define HAS_RGB_LED      1 // Defined in code via PIN_NEOPIXEL check, but can be explicit here

#define PIN_FLASHLIGHT      -1      // Disabled

// --- Power Management ---
#define BATTERY_CAPACITY_MAH 1500   // Battery Capacity in mAh (Adjust to your battery)
#define SOS_CURRENT_MA      220     // Estimated current consumption in SOS mode (mA) -> Kept for calculation logic usage if any, otherwise unused
#define CHARGE_CURRENT_MA   500     // Estimated charging current (mA) - usually 500mA for USB
#define DISPLAY_TIMEOUT     120000  // Display Auto-Off (ms) -> 2 Minutes
#define BAT_CHECK_INTERVAL_ACTIVE 60000  // Battery check interval when Display ON (ms)
#define BAT_CHECK_INTERVAL_IDLE   300000 // Battery check interval when Display OFF (ms)
#define CHARGE_CHECK_INTERVAL     3000   // Charge detection interval (ms)
#define CHARGE_CHECK_DURATION     300000 // Duration to check for charger after boot (ms) -> 5 Minutes

// --- Feature Settings ---
// User Data - Comment out to disable specific fields
#define USER_BLOOD_TYPE     "A+"         // Optional: Blood Type
#define USER_GENDER         "female"     // Optional: Gender
#define USER_BIRTH_YEAR     1984         // Optional: Birth Year (Age calculated via GPS)
#define USER_MED_ALLERGIES  "None"       // Optional: Medication Allergies

// LoRa SOS - REMOVED

// Buddy Tracking Configuration - REMOVED

// Navigation
#define BREADCRUMB_DIST     50.0    // Distance between breadcrumbs (meters) - Reduced for better precision
#define MAX_BREADCRUMBS     2000    // Max breadcrumbs (Limited for Filesystem storage)
#define MIN_SPEED_KPH       0.5     // Minimum speed to record breadcrumb (prevents GPS drift)
#define MAX_SPEED_KPH       12.0    // Maximum realistic hiking speed (prevents GPS glitches)
#define BREADCRUMB_TURN_THRESHOLD 45.0 // Degrees change to trigger breadcrumb
#define BREADCRUMB_MIN_DIST_TURN  20.0 // Min distance to check for turn (meters)

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

// LoRa Configuration
#define LORA_BW             125.0   // Bandwidth (kHz) - Keep 125 for stability
#define LORA_SF             12      // Spreading Factor (12 = Max Range, Slowest)
#define LORA_CR             8       // Coding Rate (4/8 = Max Error Correction)
#define LORA_SYNC_WORD      0x12    // Sync Word (Private Network)
#define LORA_POWER          22      // Output Power (dBm) - Max for SX1262
#define LORA_PREAMBLE       8       // Preamble Length
#define LORA_TCXO_VOLTAGE   1.6     // TCXO Voltage (1.6V for Heltec T114)

// --- LoRaWAN Configuration (Hybrid Mode) ---
// Set to 1 to enable LoRaWAN (TTN) + P2P Fallback
#define ENABLE_LORAWAN      0       

// TTN Keys (MSB Format) - Get these from TTN Console
#define LORAWAN_JOIN_EUI    0x0000000000000000  // AppEUI
#define LORAWAN_DEV_EUI     0x0000000000000000  // DeviceEUI
// AppKey: 16 Bytes, e.g. {0x00, 0x01, ...}
#define LORAWAN_APP_KEY     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

// UI Layout
#define STATUS_BAR_LINE_Y   15      // Y-coordinate of the status bar separator
#define BATTERY_BAR_HEIGHT  8       // Max height of battery bar
#define BREADCRUMB_REACH_DIST 20.0  // Distance to consider breadcrumb reached (m)

// Display Power Management
#define DISPLAY_POWER_SAVE_ON  1    // Display OFF
#define DISPLAY_POWER_SAVE_OFF 0    // Display ON

#endif
