#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- System Configuration ---
#define SERIAL_BAUD         115200  // Serial Monitor Baud Rate

// --- Hardware Pins ---

// Heltec Mesh Node T114 (nRF52840)
#define PIN_GPS_RX          39      // P1.07
#define PIN_GPS_TX          37      // P1.05
#define PIN_LORA_NSS        24      // P0.24
#define PIN_LORA_DIO1       20      // P0.20
#define PIN_LORA_RST        25      // P0.25
#define PIN_LORA_BUSY       17      // P0.17
#define PIN_LORA_SCK        19      // P0.19
#define PIN_LORA_MISO       23      // P0.23
#define PIN_LORA_MOSI       22      // P0.22
#define PIN_VEXT            21      // P0.21 (Power Control)
#define PIN_I2C_SDA         16      // P0.16
#define PIN_I2C_SCL         13      // P0.13
#define PIN_BUTTON          42      // P1.10 (User Button)
#define PIN_BAT_ADC         4       // P0.04 (AIN2 - Battery Voltage)
#define PIN_BAT_READ_CTRL   6       // P0.06 (Battery Divider Control)
#define PIN_NEOPIXEL        40      // P1.08 (External WS2812 Data)

// Outputs
#define PIN_VIB_MOTOR       36      // P1.04 (Vibration Motor)
#define PIN_FLASHLIGHT      38      // P1.06 (Flashlight LED)

// --- Power Management ---
#define BATTERY_CAPACITY_MAH 2000   // Battery Capacity in mAh (Adjust to your battery)
#define SOS_CURRENT_MA      220     // Estimated current consumption in SOS mode (mA)
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

// LoRa SOS
#define LORA_TX_INTERVAL    60000   // SOS Beacon Interval (ms) -> 1 Minute
#define LORA_FREQ           868.0   // Frequency (EU868)
#define SOS_MSG_TEXT        "SOS!"  // Custom SOS Message Text

// Navigation
#define BREADCRUMB_DIST     100.0   // Distance between breadcrumbs (meters)
#define MAX_BREADCRUMBS     8500    // Max breadcrumbs (Increased for smart tracking)
#define MIN_SPEED_KPH       1.0     // Minimum speed to record breadcrumb (prevents GPS drift)
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

// Hardware Constants
#define BNO055_ID           55      // BNO055 Sensor ID
#define BNO055_ADDRESS      0x28    // BNO055 I2C Address
#define I2C_SPEED_FAST      400000  // I2C Speed (400kHz)
#define GPS_BAUD            9600    // GPS Serial Baud Rate
#define NEOPIXEL_INDEX      0       // Index of the single NeoPixel

// LoRa Configuration
#define LORA_BW             125.0   // Bandwidth (kHz)
#define LORA_SF             9       // Spreading Factor
#define LORA_CR             7       // Coding Rate (4/7)
#define LORA_SYNC_WORD      0x12    // Sync Word (Private Network)
#define LORA_POWER          10      // Output Power (dBm)
#define LORA_PREAMBLE       8       // Preamble Length
#define LORA_GAIN           0       // Gain (0 = Auto)

// UI Layout
#define STATUS_BAR_LINE_Y   15      // Y-coordinate of the status bar separator
#define BATTERY_BAR_HEIGHT  8       // Max height of battery bar
#define BREADCRUMB_REACH_DIST 20.0  // Distance to consider breadcrumb reached (m)

// Display Power Management
#define DISPLAY_POWER_SAVE_ON  1    // Display OFF
#define DISPLAY_POWER_SAVE_OFF 0    // Display ON

#endif
