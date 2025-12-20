#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- System Configuration ---
#define WDT_TIMEOUT         10      // Watchdog Timeout in Seconds
#define SERIAL_BAUD         115200  // Serial Monitor Baud Rate

// --- Hardware Pins (Heltec Wireless Tracker V1.1) ---
// Internal Devices
#define PIN_GPS_RX          34
#define PIN_GPS_TX          33
#define PIN_LORA_NSS        8
#define PIN_LORA_DIO1       14
#define PIN_LORA_RST        12
#define PIN_LORA_BUSY       13
#define PIN_LORA_SCK        9
#define PIN_LORA_MISO       11
#define PIN_LORA_MOSI       10
#define PIN_NEOPIXEL        18      // Internal WS2812 RGB LED
#define PIN_VEXT            3       // Power Control for Sensors

// External Peripherals
#define PIN_I2C_SDA         41      // OLED & BNO055 SDA
#define PIN_I2C_SCL         42      // OLED & BNO055 SCL
#define PIN_BUTTON          0       // Built-in PRG Button
#define PIN_VIB_MOTOR       7       // Vibration Motor
#define PIN_FLASHLIGHT      5       // High Power LED

// --- Power Management ---
#define BATTERY_CAPACITY_MAH 2000   // Battery Capacity in mAh (Adjust to your battery)
#define SOS_CURRENT_MA      220     // Estimated current consumption in SOS mode (mA)
#define DISPLAY_TIMEOUT     120000  // Display Auto-Off (ms) -> 2 Minutes
#define CPU_FREQ_HIGH       240     // MHz (Active)
#define CPU_FREQ_LOW        80      // MHz (Standby)

// --- Feature Settings ---
// User Data - Comment out to disable specific fields
#define USER_BLOOD_TYPE     "A+"         // Optional: Blood Type
#define USER_GENDER         "female"     // Optional: Gender
#define USER_BIRTH_YEAR     1984         // Optional: Birth Year (Age calculated via GPS)
#define USER_MED_ALLERGIES  "None"       // Optional: Medication Allergies

// LoRa SOS
#define LORA_TX_INTERVAL    60000   // SOS Beacon Interval (ms) -> 1 Minute
#define LORA_FREQ           868.0   // Frequency (EU868)
#define SOS_MSG_TEXT        "SOS!"  // Custom SOS Message Text

// Navigation
#define BREADCRUMB_DIST     250.0   // Distance between breadcrumbs (meters)
#define MAX_BREADCRUMBS     1000    // Max breadcrumbs (Increased for smart tracking)
#define MIN_SPEED_KPH       1.0     // Minimum speed to record breadcrumb (prevents GPS drift)
#define MAX_SPEED_KPH       12.0    // Maximum realistic hiking speed (prevents GPS glitches)
#define BREADCRUMB_TURN_THRESHOLD 45.0 // Degrees change to trigger breadcrumb
#define BREADCRUMB_MIN_DIST_TURN  20.0 // Min distance to check for turn (meters)

// User Interface
#define CLICK_DELAY         500     // Max delay between clicks (ms)
#define LED_BRIGHTNESS      100     // NeoPixel Brightness (0-255) -> ~40%

#endif
