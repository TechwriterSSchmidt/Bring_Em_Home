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
#define DISPLAY_TIMEOUT     120000  // Display Auto-Off (ms) -> 2 Minutes
#define CPU_FREQ_HIGH       240     // MHz (Active)
#define CPU_FREQ_LOW        80      // MHz (Standby)

// --- Feature Settings ---
// User Data
#define USER_BLOOD_TYPE     "A+"    // Blood Type for SOS Message

// LoRa SOS
#define LORA_TX_INTERVAL    60000   // SOS Beacon Interval (ms) -> 1 Minute
#define LORA_FREQ           868.0   // Frequency (EU868)

// Navigation
#define BREADCRUMB_DIST     250.0   // Distance between breadcrumbs (meters)
#define MAX_BREADCRUMBS     350     // Max breadcrumbs (approx 80-90km)
#define MIN_SPEED_KPH       1.0     // Minimum speed to record breadcrumb (prevents GPS drift)
#define MAX_SPEED_KPH       12.0    // Maximum realistic hiking speed (prevents GPS glitches)

// User Interface
#define CLICK_DELAY         500     // Max delay between clicks (ms)
#define LED_BRIGHTNESS      100     // NeoPixel Brightness (0-255) -> ~40%

#endif
