// ESP32-S3 - Bring Em Home Project
// Using U8g2 library for SH1107 128x128 OLED (I2C)
// Hardware: ESP32-S3 + SH1107 OLED + BNO055 + ATGM336H

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#ifndef HELTEC_T114
#include <Preferences.h>
#include <WiFi.h> // For power management
#include <esp_task_wdt.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Update.h>
#endif

#ifdef HELTEC_T114
#include <nrf_gpio.h>
#include <nrf_power.h>
#endif

#include <vector>
#include <RadioLib.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"

// Display dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128

// Power Management
unsigned long lastInteractionTime = 0;
bool isDisplayOn = true;
bool lastButtonState = HIGH;
unsigned long buttonPressStartTime = 0;
bool isLongPressHandled = false;

// Vibration Control
unsigned long vibrationStartTime = 0;
bool isVibrating = false;

// Feature State
bool isFlashlightOn = false;
bool isSOSActive = false;
bool isSOSCountdown = false;
unsigned long sosCountdownStartTime = 0;
bool isCharging = false;
unsigned long lastSOSUpdate = 0;
int sosStep = 0;
int lastValidBatteryPct = 0; // Stores battery % before charging started
float estimatedChargeTimeHours = 0; // Estimated time to full charge
unsigned long chargeStartTime = 0;

// Objects
TinyGPSPlus gps;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#ifndef HELTEC_T114
Preferences preferences;
#endif

#ifdef HELTEC_T114
// Use default SPI instance for nRF52
SPIClass& loraSPI = SPI;
#else
// Use HSPI for ESP32
SPIClass loraSPI(HSPI);
#endif

SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);

#if defined(PIN_NEOPIXEL) && (PIN_NEOPIXEL >= 0)
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#else
// Dummy class or just don't use it. 
// For simplicity, we'll create a dummy object if needed or just use -1 which is handled by library usually.
Adafruit_NeoPixel pixels(1, -1, NEO_GRB + NEO_KHZ800);
#endif

// LoRa State
unsigned long lastLoRaTx = 0;

// U8g2 Display Object (SH1107 128x128 I2C)
// Full framebuffer (F) for smooth animation
U8G2_SH1107_128X128_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// App Modes
enum AppMode {
    MODE_EXPLORE,
    MODE_BRING_HOME
};
AppMode currentMode = MODE_EXPLORE;

// Navigation Data
double homeLat = 0.0;
double homeLon = 0.0;
bool hasHome = false;
int currentHeading = 0;
unsigned long startTime = 0;

// Breadcrumbs
struct Breadcrumb {
    double lat;
    double lon;
};
std::vector<Breadcrumb> breadcrumbs;
const char* BREADCRUMB_FILE = "/breadcrumbs.dat";
int targetBreadcrumbIndex = -1; // For backtracking

// Button Logic (Multi-click)
int clickCount = 0;
unsigned long lastClickTime = 0;

// Compass Calibration State
bool isCalibrated = false;
bool calibSaved = false;

// --- Non-Blocking Globals ---
volatile bool transmissionFlag = false;
bool isLoRaTransmitting = false;

unsigned long feedbackEndTime = 0;
String feedbackTitle = "";
String feedbackSub = "";

void setFlag(void) {
  transmissionFlag = true;
}

void showFeedback(String title, String sub, int duration) {
    feedbackTitle = title;
    feedbackSub = sub;
    feedbackEndTime = millis() + duration;
    if (!isDisplayOn) {
        u8g2.setPowerSave(0);
        isDisplayOn = true;
        #ifndef HELTEC_T114
        setCpuFrequencyMhz(240);
        #endif
    }
}

void triggerVibration() {
    digitalWrite(PIN_VIB_MOTOR, HIGH);
    vibrationStartTime = millis();
    isVibrating = true;
}

void toggleFlashlight() {
    isFlashlightOn = !isFlashlightOn;
    isSOSActive = false; // Disable SOS if manual light is toggled
    digitalWrite(PIN_FLASHLIGHT, isFlashlightOn ? HIGH : LOW);
}

void toggleSOS() {
    isSOSActive = !isSOSActive;
    isFlashlightOn = false; // Disable manual light
    if (!isSOSActive) {
        digitalWrite(PIN_FLASHLIGHT, LOW);
        radio.sleep(); // Put LoRa to sleep
        Serial.println("SOS Deactivated. LoRa Sleeping.");
    } else {
        Serial.println("SOS Activated! LoRa Waking up.");
        // Wake up LoRa (re-init if needed or just start sending)
        // We will handle TX in the loop
    }
}

// --- Helper Functions ---

void powerOff() {
    Serial.println("Entering Deep Sleep...");
    
    // 1. Feedback
    #ifdef HELTEC_T114
    digitalWrite(PIN_VIB_MOTOR, HIGH);
    delay(200);
    digitalWrite(PIN_VIB_MOTOR, LOW);
    delay(200);
    digitalWrite(PIN_VIB_MOTOR, HIGH);
    delay(500);
    digitalWrite(PIN_VIB_MOTOR, LOW);
    #endif

    // 2. Display OFF
    u8g2.setPowerSave(1);
    
    // 3. Sensors OFF
    #ifdef HELTEC_T114
    digitalWrite(PIN_VEXT, LOW);
    digitalWrite(PIN_FLASHLIGHT, LOW);
    #endif

    // 4. LoRa Sleep
    radio.sleep();

    // 5. Wait for button release (Important!)
    while(digitalRead(PIN_BUTTON) == LOW) {
        delay(10);
    }

    // 6. Configure Wakeup
    #ifdef HELTEC_T114
    // Configure Button pin for Wakeup (Sense LOW)
    nrf_gpio_cfg_sense_input(PIN_BUTTON, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    
    // Enter System OFF
    NRF_POWER->SYSTEMOFF = 1;
    while(1); // Wait for shutdown
    #else
    // ESP32 Deep Sleep
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BUTTON, 0);
    esp_deep_sleep_start();
    #endif
}

int getBatteryPercent() {
    #ifdef HELTEC_T114
    // Heltec T114 Battery Reading (Simple Approximation)
    // ADC Resolution default is 10-bit (0-1023) in Arduino nRF52
    // Reference is usually 3.6V internal
    // Divider is 1/2 (100k+100k)
    int raw = analogRead(PIN_BAT_ADC);
    float voltage = (raw / 1023.0) * 3.6 * 2.0; 
    
    int pct = (int)((voltage - 3.3) * 100 / (4.2 - 3.3));
    if (pct > 100) pct = 100;
    if (pct < 0) pct = 0;
    return pct;
    #else
    // Heltec Wireless Tracker V1.1: GPIO 1, Divider ~2
    // 4.2V = 100%, 3.3V = 0%
    // Using analogReadMilliVolts for factory calibrated reading
    uint32_t voltage_mv = analogReadMilliVolts(1) * 2; // Multiply by 2 for voltage divider
    
    // Simple linear mapping
    // 4200mV = 100%, 3300mV = 0%
    int pct = (int)((voltage_mv - 3300) * 100 / (4200 - 3300));
    if (pct > 100) pct = 100;
    if (pct < 0) pct = 0;
    return pct;
    #endif
}

void updateSOS() {
    if (!isSOSActive) return;
    
    unsigned long now = millis();

    // --- LoRa Transmission (Every 1 Minute) ---
    if (now - lastLoRaTx > LORA_TX_INTERVAL) {
        lastLoRaTx = now;
        int batPct = getBatteryPercent();
        
        // Calculate estimated runtime
        // Capacity remaining = Total Capacity * (Percent / 100)
        // Runtime (h) = Capacity remaining (mAh) / Current Draw (mA)
        float remainingCap = (float)BATTERY_CAPACITY_MAH * ((float)batPct / 100.0);
        float runtimeHours = remainingCap / (float)SOS_CURRENT_MA;

        String msg = String(SOS_MSG_TEXT) + " Lat:" + String(gps.location.lat(), 6) + 
                     " Lon:" + String(gps.location.lng(), 6) + 
                     " Bat:" + String(batPct) + "%" +
                     " Est:" + String(runtimeHours, 1) + "h";

        #ifdef USER_BLOOD_TYPE
        msg += " Type:" + String(USER_BLOOD_TYPE);
        #endif

        #ifdef USER_GENDER
        msg += " Gen:" + String(USER_GENDER);
        #endif

        #ifdef USER_BIRTH_YEAR
        if (gps.date.isValid() && gps.date.year() > 2020) {
            int age = gps.date.year() - USER_BIRTH_YEAR;
            msg += " Age:" + String(age);
        } else {
            msg += " Born:" + String(USER_BIRTH_YEAR);
        }
        #endif

        #ifdef USER_MED_ALLERGIES
        msg += " Alg:" + String(USER_MED_ALLERGIES);
        #endif

        Serial.print("Sending LoRa SOS: "); Serial.println(msg);
        
        // Wake up and send
        radio.standby();
        
        // ASYNC TRANSMISSION
        transmissionFlag = false;
        int state = radio.startTransmit(msg);
        
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("LoRa TX Started...");
            isLoRaTransmitting = true;
        } else {
            Serial.print("LoRa TX Failed, code "); Serial.println(state);
            radio.sleep();
        }
    }

    // --- LED Pattern ---
    // SOS Pattern: ... --- ... (Dot=200ms, Dash=600ms, Gap=200ms, LetterGap=600ms, WordGap=1400ms)
    // Simplified: 3 short, 3 long, 3 short
    
    const int DOT = 200;
    const int DASH = 600;
    const int GAP = 200;
    
    // Timing array for one cycle (On times and Off times interleaved)
    // S (Dot, Gap, Dot, Gap, Dot, Gap) -> O (Dash, Gap, Dash, Gap, Dash, Gap) -> S (Dot, Gap, Dot, Gap, Dot, Gap) -> Pause
    static const int pattern[] = {
        DOT, GAP, DOT, GAP, DOT, GAP*3, // S
        DASH, GAP, DASH, GAP, DASH, GAP*3, // O
        DOT, GAP, DOT, GAP, DOT, GAP*7 // S + Pause
    };
    const int steps = sizeof(pattern) / sizeof(pattern[0]);
    
    if (now - lastSOSUpdate > pattern[sosStep]) {
        lastSOSUpdate = now;
        sosStep++;
        if (sosStep >= steps) sosStep = 0;
        
        // Even steps are ON, Odd steps are OFF
        digitalWrite(PIN_FLASHLIGHT, (sosStep % 2 == 0) ? HIGH : LOW);
    }
}

// Helper: Draw Rotated Arrow (Fancy Needle)
void drawArrow(int cx, int cy, int r, float angleDeg, bool showCardinals = false) {
    float angleRad = (angleDeg - 90) * PI / 180.0; // -90 to point up at 0 deg
    
    // Tip
    int x1 = cx + r * cos(angleRad);
    int y1 = cy + r * sin(angleRad);
    
    // Tail (indented)
    int x2 = cx - (r * 0.3) * cos(angleRad);
    int y2 = cy - (r * 0.3) * sin(angleRad);
    
    // Side 1 (Right)
    int x3 = cx + (r * 0.35) * cos(angleRad + PI/2);
    int y3 = cy + (r * 0.35) * sin(angleRad + PI/2);

    // Side 2 (Left)
    int x4 = cx + (r * 0.35) * cos(angleRad - PI/2);
    int y4 = cy + (r * 0.35) * sin(angleRad - PI/2);
    
    // Draw two triangles for the needle
    u8g2.drawTriangle(x1, y1, x3, y3, x2, y2);
    u8g2.drawTriangle(x1, y1, x4, y4, x2, y2);
    
    // Center Dot
    u8g2.drawDisc(cx, cy, 3);

    // Draw Cardinals if requested
    if (showCardinals) {
        u8g2.setFont(u8g2_font_5x7_tr); // Smaller font for all directions to fit better
        int r_text = r + 14; // Slightly larger radius
        
        struct Cardinal { const char* label; float offset; };
        Cardinal dirs[] = { 
            {"N", 0}, {".", 45}, {"E", 90}, {".", 135}, 
            {"S", 180}, {".", 225}, {"W", 270}, {".", 315} 
        };
        
        for (auto& d : dirs) {
            float a = (angleDeg + d.offset - 90) * PI / 180.0;
            int tx = cx + r_text * cos(a);
            int ty = cy + r_text * sin(a);
            
            if (strcmp(d.label, ".") == 0) {
                // Draw 2x2 dot (4 pixels)
                u8g2.drawBox(tx - 1, ty - 1, 2, 2);
            } else {
                // Center text
                int w = u8g2.getStrWidth(d.label);
                int h = 6; // approx height
                u8g2.setCursor(tx - w/2, ty + h/2); // Adjust y for baseline
                u8g2.print(d.label);
            }
        }
    }
}

// --- Helper Functions ---

void saveBreadcrumbs() {
    #ifndef HELTEC_T114
    File file = LittleFS.open(BREADCRUMB_FILE, "w");
    if (file) {
        for (const auto& b : breadcrumbs) {
            file.write((uint8_t*)&b, sizeof(Breadcrumb));
        }
        file.close();
    }
    #endif
}

void loadBreadcrumbs() {
    #ifndef HELTEC_T114
    if (LittleFS.exists(BREADCRUMB_FILE)) {
        File file = LittleFS.open(BREADCRUMB_FILE, "r");
        if (file) {
            breadcrumbs.clear();
            Breadcrumb b;
            while (file.read((uint8_t*)&b, sizeof(Breadcrumb)) == sizeof(Breadcrumb)) {
                breadcrumbs.push_back(b);
            }
            file.close();
            Serial.printf("Loaded %d breadcrumbs from Flash.\n", breadcrumbs.size());
        }
    }
    #endif
}

// Hilfsfunktion für kritische Fehler
void fatalError(String message) {
    Serial.println("\n!!! KRITISCHER FEHLER !!!");
    Serial.println("Grund: " + message);
    Serial.println("System wird in 5 Sekunden neu gestartet...");

    // Falls eine Onboard-LED definiert ist, blinken wir hektisch
    #ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
    for(int i = 0; i < 20; i++) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
    #else
    delay(2000);
    #endif

    Serial.println("Restarting...");
    #ifdef HELTEC_T114
    NVIC_SystemReset();
    #else
    ESP.restart(); // Software-Reset durchführen
    #endif
}

// Helper: Update RGB Status LED
void updateStatusLED() {
    static unsigned long lastBlink = 0;
    static int pulseBrightness = 0;
    static int pulseDir = 5;
    static unsigned long lastPulseTime = 0;

    // New static vars for Low Battery
    static bool hasWarnedLowBattery = false;
    static unsigned long lastLowBatFlash = 0;

    // 1. Battery Warning (< 10%)
    int pct = getBatteryPercent();
    bool lowBattery = (pct < 10); 

    if (lowBattery) {
        // Trigger Vibration ONCE when entering low battery state
        if (!hasWarnedLowBattery) {
            // 3x Vibration (Blocking is acceptable here for safety warning)
            for(int i=0; i<3; i++) {
                digitalWrite(PIN_VIB_MOTOR, HIGH);
                delay(200);
                digitalWrite(PIN_VIB_MOTOR, LOW);
                delay(200);
            }
            hasWarnedLowBattery = true;
        }

        // Yellow Flash every 10s
        if (millis() - lastLowBatFlash > 10000) {
            pixels.setPixelColor(0, pixels.Color(255, 200, 0)); // Yellow
            pixels.show();
            delay(100); // Short flash
            pixels.clear();
            pixels.show();
            lastLowBatFlash = millis();
        }
        
        // Ensure LED is off between flashes (since we return early)
        // The delay above handles the off state, but if we just entered this block
        // and didn't flash, we want to make sure we aren't stuck with a red color from previous state.
        // However, pixels.clear() is called at the end of the flash.
        // If we transition from Red Pulse to Low Battery, we might need to clear once.
        // But since we clear after flash, and we return, it should be fine.
        // To be safe, we can ensure it's off if not flashing.
        if (millis() - lastLowBatFlash > 150) {
             pixels.clear();
             pixels.show();
        }

        return; // Override other LED patterns
    } else {
        hasWarnedLowBattery = false; // Reset if charged
    }

    // 2. GPS Search (Red Pulse)
    if (!gps.location.isValid()) {
        if (millis() - lastPulseTime > 20) { // Smooth fading
            pixels.setPixelColor(0, pixels.Color(pulseBrightness, 0, 0));
            pixels.show();
            pulseBrightness += pulseDir;
            if (pulseBrightness >= LED_BRIGHTNESS || pulseBrightness <= 0) { // Max brightness
                pulseDir = -pulseDir;
            }
            lastPulseTime = millis();
        }
    } else {
        // GPS Fix acquired -> LED off to save power
        pixels.clear();
        pixels.show();
    }
}

#ifndef HELTEC_T114
void enterOTAMode() {
    Serial.println("Entering OTA Mode...");
    
    // Init Display for OTA Message
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    
    const char* lines[] = {
        "OTA UPDATE MODE",
        "Connect WiFi:",
        "Bring_Em_Home",
        "Go to:",
        "192.168.4.1"
    };
    int y_positions[] = {10, 30, 45, 65, 80};

    for(int i=0; i<5; i++) {
        int w = u8g2.getStrWidth(lines[i]);
        u8g2.drawStr((SCREEN_WIDTH - w) / 2, y_positions[i], lines[i]);
    }
    u8g2.sendBuffer();

    // Start WiFi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Bring_Em_Home"); // No password
    
    // Start DNS Server (Captive Portal)
    DNSServer dnsServer;
    dnsServer.start(53, "*", WiFi.softAPIP());

    // Start WebServer
    WebServer server(80);
    
    // Root Page
    server.on("/", HTTP_GET, [&server]() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", 
            "<!DOCTYPE html><html><head><title>Bring Em Home OTA</title>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>"
            "<style>body{font-family:sans-serif;text-align:center;padding:20px;}"
            "input{margin:10px;padding:10px;}</style></head>"
            "<body><h1>Firmware Update</h1>"
            "<p>Select firmware.bin or littlefs.bin</p>"
            "<form method='POST' action='/update' enctype='multipart/form-data'>"
            "<input type='file' name='update'><br>"
            "<input type='submit' value='Update Device'>"
            "</form></body></html>");
    });

    // Update Handler
    server.on("/update", HTTP_POST, [&server]() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
        delay(1000);
        ESP.restart();
    }, [&server]() {
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
            Serial.printf("Update: %s\n", upload.filename.c_str());
            int cmd = U_FLASH;
            if (upload.filename.indexOf("littlefs") > -1 || upload.filename.indexOf("spiffs") > -1) {
                cmd = U_SPIFFS;
                Serial.println("Target: Filesystem");
            } else {
                Serial.println("Target: Firmware");
            }
            
            if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) {
                Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            } else {
                Update.printError(Serial);
            }
        }
    });

    server.begin();
    Serial.println("OTA Server started.");

    unsigned long otaStart = millis();
    unsigned long lastLedUpdate = 0;
    int ledBright = 0;
    int ledDir = 5;

    while (millis() - otaStart < 300000) { // 5 Minutes
        dnsServer.processNextRequest();
        server.handleClient();
        
        // White Pulsing LED
        if (millis() - lastLedUpdate > 20) {
            pixels.setPixelColor(0, pixels.Color(ledBright, ledBright, ledBright)); // White
            pixels.show();
            ledBright += ledDir;
            if (ledBright >= 150 || ledBright <= 0) ledDir = -ledDir;
            lastLedUpdate = millis();
        }
        
        delay(10);
    }

    Serial.println("OTA Timeout. Restarting...");
    ESP.restart();
}
#endif

// Helper: Calculate Total Path Distance to Home
double calculateTotalDistanceToHome() {
    if (!gps.location.isValid() || !hasHome) return 0.0;
    
    double totalDist = 0.0;
    
    if (currentMode == MODE_BRING_HOME) {
        if (targetBreadcrumbIndex >= 0 && !breadcrumbs.empty()) {
            // 1. Current Pos -> Target Breadcrumb
            totalDist += gps.distanceBetween(gps.location.lat(), gps.location.lng(), 
                                           breadcrumbs[targetBreadcrumbIndex].lat, 
                                           breadcrumbs[targetBreadcrumbIndex].lon);
            
            // 2. Target Breadcrumb -> ... -> First Breadcrumb
            for (int i = targetBreadcrumbIndex; i > 0; i--) {
                totalDist += gps.distanceBetween(breadcrumbs[i].lat, breadcrumbs[i].lon,
                                               breadcrumbs[i-1].lat, breadcrumbs[i-1].lon);
            }
            
            // 3. First Breadcrumb -> Home
            totalDist += gps.distanceBetween(breadcrumbs[0].lat, breadcrumbs[0].lon, homeLat, homeLon);
        } else {
            // Direct to Home
            totalDist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon);
        }
    } else {
        // MODE_EXPLORE
        // Calculate full trail distance back
        if (!breadcrumbs.empty()) {
            // 1. Current -> Last Breadcrumb
            totalDist += gps.distanceBetween(gps.location.lat(), gps.location.lng(), 
                                           breadcrumbs.back().lat, breadcrumbs.back().lon);
            
            // 2. Sum of all segments
            for (int i = breadcrumbs.size() - 1; i > 0; i--) {
                totalDist += gps.distanceBetween(breadcrumbs[i].lat, breadcrumbs[i].lon,
                                               breadcrumbs[i-1].lat, breadcrumbs[i-1].lon);
            }
            
            // 3. First -> Home
            totalDist += gps.distanceBetween(breadcrumbs[0].lat, breadcrumbs[0].lon, homeLat, homeLon);
        } else {
            // Direct
            totalDist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon);
        }
    }
    
    return totalDist;
}

void loadCompassCalibration() {
    #ifndef HELTEC_T114
    Preferences calibPrefs;
    calibPrefs.begin("bno055", true); // Read-only mode
    adafruit_bno055_offsets_t calibrationData;
    
    if (calibPrefs.getBytesLength("calib") == sizeof(adafruit_bno055_offsets_t)) {
        calibPrefs.getBytes("calib", &calibrationData, sizeof(adafruit_bno055_offsets_t));
        bno.setSensorOffsets(calibrationData);
        Serial.println("Compass: Calibration loaded from Flash.");
        isCalibrated = true; 
        calibSaved = true;   
    } else {
        Serial.println("Compass: No calibration found.");
    }
    calibPrefs.end();
    #endif
}

void saveCompassCalibration() {
    #ifndef HELTEC_T114
    adafruit_bno055_offsets_t newCalib;
    if (bno.getSensorOffsets(newCalib)) {
        Preferences calibPrefs;
        calibPrefs.begin("bno055", false); // Read-write mode
        calibPrefs.putBytes("calib", &newCalib, sizeof(adafruit_bno055_offsets_t));
        showFeedback("Compass Calib", "SAVED!", 1000);
    }
    #endif
}

void setup() {
    #ifdef HELTEC_T114
    // --- Wake-up Check (Soft-Off Logic) ---
    // If we woke up from System OFF via Button, we need to check if it's a real "Turn On" request (3s hold).
    // If not, we go back to sleep immediately.
    
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    
    // Check if button is pressed at boot
    if (digitalRead(PIN_BUTTON) == LOW) {
        // Wait and check for 3s hold
        unsigned long start = millis();
        bool turnOn = false;
        
        // Feedback: Short blip to indicate "I'm listening"
        pinMode(PIN_VIB_MOTOR, OUTPUT);
        digitalWrite(PIN_VIB_MOTOR, HIGH);
        delay(50);
        digitalWrite(PIN_VIB_MOTOR, LOW);

        while (digitalRead(PIN_BUTTON) == LOW) {
            if (millis() - start > 3000) {
                turnOn = true;
                break;
            }
            delay(10);
        }
        
        if (!turnOn) {
            // Button released too early -> Go back to sleep
            powerOff(); 
        } else {
            // Success! Feedback
            digitalWrite(PIN_VIB_MOTOR, HIGH);
            delay(200);
            digitalWrite(PIN_VIB_MOTOR, LOW);
        }
    } else {
        // Powered on by battery insertion or reset -> Go to sleep?
        // Usually we want to start if battery is inserted. 
        // But if we want strict Soft-Off, we might want to sleep.
        // For now, let's allow boot on battery insert.
    }

    // Power on VExt for sensors (GPS, LoRa, OLED)
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, HIGH);
    delay(100); // Wait for power to stabilize
    #endif

    // ADC Setup for Battery
    #ifndef HELTEC_T114
    analogSetAttenuation(ADC_11db);
    #endif

    Serial.begin(SERIAL_BAUD);

    // Check for OTA Mode (Button held for 5s at boot)
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    if (digitalRead(PIN_BUTTON) == LOW) {
        Serial.println("Button held... Waiting 5s for OTA...");
        delay(100); // Debounce
        unsigned long pressStart = millis();
        bool enterOTA = false;
        while (digitalRead(PIN_BUTTON) == LOW) {
            if (millis() - pressStart > 5000) {
                enterOTA = true;
                break;
            }
            delay(10);
        }
        
        if (enterOTA) {
            #ifndef HELTEC_T114
            enterOTAMode(); // Does not return (restarts ESP)
            #endif
        }
    }

    // Init NeoPixel
    pixels.begin();
    pixels.setBrightness(LED_BRIGHTNESS); // 40% Brightness (approx 100/255)
    pixels.clear();
    pixels.show();

    #ifndef HELTEC_T114
    // Initialize Watchdog Timer
    esp_task_wdt_init(WDT_TIMEOUT, true); // Enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); // Add current thread to WDT watch

    // Power Optimization: Turn off Radios
    WiFi.mode(WIFI_OFF);
    btStop();
    setCpuFrequencyMhz(CPU_FREQ_HIGH); // Start with high performance for UI

    // Wakeup Safety Check (Prevent accidental pocket activation)
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        // Woke up by button. Verify hold duration.
        unsigned long wakeStart = millis();
        bool wakeConfirmed = false;
        
        // Wait for user to hold button for 2 seconds
        while (digitalRead(PIN_BUTTON) == LOW) {
            if (millis() - wakeStart > 2000) { 
                wakeConfirmed = true;
                break;
            }
            delay(10);
        }
        
        if (!wakeConfirmed) {
            // Button released too early -> Go back to sleep immediately
            esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BUTTON, 0);
            esp_deep_sleep_start();
        }
    }
    #endif

    delay(2000);
    Serial.println("\n\n=== Bring Em Home ===");
    
    // Button Setup
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    lastInteractionTime = millis();

    // Vibration Setup
    pinMode(PIN_VIB_MOTOR, OUTPUT);
    digitalWrite(PIN_VIB_MOTOR, LOW);

    // Flashlight Setup
    pinMode(PIN_FLASHLIGHT, OUTPUT);
    digitalWrite(PIN_FLASHLIGHT, LOW);

    // VEXT Power Control (Enable GPS & Sensors)
    // Heltec Wireless Tracker V1.1: GPIO 3 must be HIGH to enable GNSS
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, HIGH);
    delay(500); // Give sensors time to wake up

    // Initialize Sensors
    #ifdef HELTEC_T114
    Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.begin();
    #else
    if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL)) {
        fatalError("I2C Bus Fehler!");
    }
    #endif
    
    // Initialize BNO055
    if (!bno.begin()) {
        Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
        fatalError("BNO055 Error!");
    }
    bno.setExtCrystalUse(true);
    
    // Load Calibration
    loadCompassCalibration();

    // Initialize GPS (ATGM336H typically 9600 baud)
    #ifdef HELTEC_T114
    Serial1.setPins(PIN_GPS_RX, PIN_GPS_TX);
    Serial1.begin(9600);
    #else
    Serial1.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    #endif
    // GPS defaults to ON when powered

    // Initialize LoRa (SX1262)
    #ifdef HELTEC_T114
    loraSPI.setPins(PIN_LORA_MISO, PIN_LORA_SCK, PIN_LORA_MOSI);
    loraSPI.begin();
    #else
    loraSPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
    #endif
    int state = radio.begin(LORA_FREQ, 125.0, 9, 7, 0x12, 10, 8, 0, false); // EU868
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("LoRa Init Success!");
        radio.setDio1Action(setFlag);
        radio.sleep(); // Sleep immediately
    } else {
        Serial.print("LoRa Init Failed, code "); Serial.println(state);
        // Non-fatal, we continue
    }
    
    #ifndef HELTEC_T114
    // Initialize Preferences
    if (!preferences.begin("bringemhome", false)) {
        fatalError("Preferences Init Failed!");
    }

    // Init Filesystem
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
    }
    
    // Smart Home Loading: Only load if NOT a fresh power-on
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_POWERON) {
        Serial.println("Power On Reset: Clearing Session (Home & Breadcrumbs).");
        hasHome = false; 
        // Breadcrumbs löschen
        LittleFS.remove(BREADCRUMB_FILE);
        breadcrumbs.clear();
        breadcrumbs.reserve(MAX_BREADCRUMBS);
    } else {
        Serial.printf("Reset Reason: %d. Attempting to restore session.\n", reason);
        homeLat = preferences.getDouble("lat", 0.0);
        homeLon = preferences.getDouble("lon", 0.0);
        hasHome = (homeLat != 0.0 && homeLon != 0.0);
        // Breadcrumbs laden
        breadcrumbs.reserve(MAX_BREADCRUMBS);
        loadBreadcrumbs();
    }
    Serial.printf("Home: %f, %f (HasHome: %d)\n", homeLat, homeLon, hasHome);
    #endif
    
    startTime = millis(); // Start timer for auto-home

    // Initialize display
    u8g2.begin();
    u8g2.setDrawColor(1); // White
    u8g2.setFontPosTop(); // Easier coordinate handling
    
    // Memory Debugging
    #ifndef HELTEC_T114
    Serial.printf("Total Heap: %d, Free Heap: %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
    Serial.printf("Total PSRAM: %d, Free PSRAM: %d\n", ESP.getPsramSize(), ESP.getFreePsram());
    #endif

    u8g2.clearBuffer();
    
    // Boot Logo (3 Seconds)
    u8g2.clearBuffer();
    
    // Draw House Icon
    int cx = 64;
    int cy = 50;
    
    // Roof
    u8g2.drawLine(cx-24, cy, cx, cy-24);
    u8g2.drawLine(cx, cy-24, cx+24, cy);
    u8g2.drawLine(cx-24, cy, cx+24, cy);
    
    // Body
    u8g2.drawFrame(cx-18, cy, 36, 28);
    
    // Door
    u8g2.drawBox(cx-6, cy+12, 12, 16);
    
    // Chimney
    u8g2.drawBox(cx+12, cy-20, 6, 12);
    
    // Smoke
    u8g2.drawDisc(cx+22, cy-24, 2);
    u8g2.drawDisc(cx+26, cy-30, 3);

    // Text
    u8g2.setFont(u8g2_font_ncenB10_tr);
    const char* title = "Bring Em Home";
    int w = u8g2.getStrWidth(title);
    u8g2.setCursor((SCREEN_WIDTH - w) / 2, 105);
    u8g2.print(title);
    
    u8g2.setFont(u8g2_font_5x7_tr);
    const char* sub = "v1.0 - Ready";
    w = u8g2.getStrWidth(sub);
    u8g2.setCursor((SCREEN_WIDTH - w) / 2, 120);
    u8g2.print(sub);
    
    u8g2.sendBuffer();
    delay(3000);
}

void loop() {
    // Reset Watchdog Timer
    #ifndef HELTEC_T114
    esp_task_wdt_reset();
    #endif

    // --- LoRa Async Handler ---
    if (transmissionFlag) {
        transmissionFlag = false;
        if (isLoRaTransmitting) {
            radio.finishTransmit();
            radio.sleep();
            isLoRaTransmitting = false;
            Serial.println("LoRa TX Finished.");
        }
    }

    // --- Compass Calibration Check ---
    static unsigned long lastCalibCheck = 0;
    if (millis() - lastCalibCheck > 1000) {
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        if (!calibSaved && system == 3 && gyro == 3 && accel == 3 && mag == 3) {
            saveCompassCalibration();
            calibSaved = true;
        }
        lastCalibCheck = millis();
    }

    // --- Charging Detection ---
    // User Request: Detect charging via voltage jump/level.
    // "Sobald man ein Ladegerät anschliesst, steigt die Spannung sprunghaft an und bleibt auf einen hohen Niveau (so etwa 5v)"
    // We check if voltage is significantly above normal battery level (4.2V).
    
    static unsigned long lastChargeCheck = 0;
    if (millis() - lastChargeCheck > 1000) {
        #ifdef HELTEC_T114
        // Placeholder for T114 charging detection
        #else
        uint32_t voltage_mv = analogReadMilliVolts(1) * 2;
        
        // Threshold: 4400mV (4.4V)
        if (voltage_mv > 4400) { 
             if (!isCharging) {
                 // Just started charging
                 isCharging = true;
                 chargeStartTime = millis();
                 
                 // Calculate estimated time to full
                 // Capacity needed = Total * (1.0 - current_pct)
                 // Time = Capacity / Current
                 float needed_mAh = (float)BATTERY_CAPACITY_MAH * (1.0 - ((float)lastValidBatteryPct / 100.0));
                 estimatedChargeTimeHours = needed_mAh / (float)CHARGE_CURRENT_MA;
                 if (estimatedChargeTimeHours < 0) estimatedChargeTimeHours = 0;
             }
        } else {
             isCharging = false;
             // Update last valid battery percent while NOT charging
             // This ensures we have a good reading before the voltage spikes
             lastValidBatteryPct = getBatteryPercent();
        }
        #endif
        lastChargeCheck = millis();
    }

    // Update Status LED (50Hz)
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate > 20) {
        if (isCharging) {
            // Green Pulsing
            static int chargeBright = 0;
            static int chargeDir = 5;
            pixels.setPixelColor(0, pixels.Color(0, chargeBright, 0));
            pixels.show();
            chargeBright += chargeDir;
            if (chargeBright >= 150 || chargeBright <= 0) chargeDir = -chargeDir;
        } else {
            updateStatusLED();
        }
        lastLedUpdate = millis();
    }

    // 1. Process GPS
    while (Serial1.available()) {
        gps.encode(Serial1.read());
    }

    // 2. Process Compass
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    currentHeading = (int)orientationData.orientation.x;

    // Get Calibration Status (0=Uncalibrated, 3=Fully Calibrated)
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    // 3. Button Logic
    bool currentButtonState = digitalRead(PIN_BUTTON);
    
    // Button Pressed
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        buttonPressStartTime = millis();
        isLongPressHandled = false;
    }
    
    // Button Held (Long Press)
    if (currentButtonState == LOW) {
        unsigned long pressDuration = millis() - buttonPressStartTime;
        
        // 3s: Power Off (Soft-Off)
        if (!isLongPressHandled && pressDuration > 3000) {
             if (currentMode == MODE_EXPLORE || currentMode == MODE_BRING_HOME) {
                 // Only allow power off if not in special modes (like SOS countdown, though that's handled elsewhere)
                 isLongPressHandled = true;
                 powerOff();
             }
        }

        // 10s: Reset Home (Extra Long Press)
        if (!isLongPressHandled && pressDuration > 10000) {
            if (currentMode == MODE_EXPLORE && gps.location.isValid()) {
                // Action
                homeLat = gps.location.lat();
                homeLon = gps.location.lng();
                #ifndef HELTEC_T114
                preferences.putDouble("lat", homeLat);
                preferences.putDouble("lon", homeLon);
                #endif
                hasHome = true;
                
                // LED Blitzgewitter (Grün)
                for(int i=0; i<5; i++) {
                    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); pixels.show();
                    digitalWrite(PIN_VIB_MOTOR, HIGH);
                    delay(100);
                    pixels.clear(); pixels.show();
                    digitalWrite(PIN_VIB_MOTOR, LOW);
                    delay(100);
                }
                
                // Display Feedback
                showFeedback("HOME RESET!", "", 2000);
                lastInteractionTime = millis();
            }
        }

        // 3s: Toggle SOS (Long Press) - REMOVED
        /*
        if (!isLongPressHandled && pressDuration > 3000) {
            isSOSActive = !isSOSActive;
            triggerVibration();
            isLongPressHandled = true;
            
            // Visual Feedback
            if (!isDisplayOn) { u8g2.setPowerSave(0); isDisplayOn = true; }
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB14_tr);
            u8g2.drawStr(10, 50, isSOSActive ? "SOS" : "SOS");
            u8g2.drawStr(30, 80, isSOSActive ? "ON" : "OFF");
            u8g2.sendBuffer();
            delay(1000);
            lastInteractionTime = millis();
        }
        */
    }
    
    // Button Released
    if (currentButtonState == HIGH && lastButtonState == LOW) {
        if (!isLongPressHandled && (millis() - buttonPressStartTime < 2000)) {
            // Register Click (only if not a long press)
            clickCount++;
            lastClickTime = millis();
        }
    }
    lastButtonState = currentButtonState;

    // Process Clicks (Delayed to detect double click)
    if (clickCount > 0 && (millis() - lastClickTime > CLICK_DELAY)) {
        
        // SOS Countdown Cancellation (Any click cancels)
        if (isSOSCountdown) {
            isSOSCountdown = false;
            showFeedback("SOS CANCELLED", "", 2000);
            clickCount = 0;
            lastInteractionTime = millis();
            // Continue loop to avoid triggering other actions
        } else if (clickCount == 1) {
            // Single Click: Toggle Display
            if (isDisplayOn) {
                u8g2.setPowerSave(1); // Display OFF
                isDisplayOn = false;
                #ifndef HELTEC_T114
                setCpuFrequencyMhz(80); // Low power mode
                #endif
            } else {
                u8g2.setPowerSave(0); // Display ON
                isDisplayOn = true;
                #ifndef HELTEC_T114
                setCpuFrequencyMhz(240); // High performance mode
                #endif
                lastInteractionTime = millis();
            }
        } else if (clickCount == 2) {
            // Double Click: Toggle Mode
            if (currentMode == MODE_EXPLORE) {
                currentMode = MODE_BRING_HOME;
                // Find closest breadcrumb to start with
                if (!breadcrumbs.empty() && gps.location.isValid()) {
                    double minDst = 99999999;
                    int minIdx = -1;
                    for(int i=0; i<breadcrumbs.size(); i++) {
                        double d = gps.distanceBetween(gps.location.lat(), gps.location.lng(), breadcrumbs[i].lat, breadcrumbs[i].lon);
                        if (d < minDst) {
                            minDst = d;
                            minIdx = i;
                        }
                    }
                    targetBreadcrumbIndex = minIdx;
                } else {
                    targetBreadcrumbIndex = -1;
                }
                
                // Feedback
                showFeedback("BRING ME HOME!", "", 1000);
            } else {
                currentMode = MODE_EXPLORE;
                // Feedback
                showFeedback("EXPLORER", "", 1000);
            }
            lastInteractionTime = millis();
        } else if (clickCount == 3) {
            // Triple Click: Flashlight
            toggleFlashlight();
            triggerVibration();
        } else if (clickCount >= 5) {
            // 5x Click: Start SOS Countdown
            isSOSCountdown = true;
            sosCountdownStartTime = millis();
            if (!isDisplayOn) { u8g2.setPowerSave(0); isDisplayOn = true; }
            triggerVibration(); // Feedback that sequence started
        }
        clickCount = 0;
    }

    // SOS Countdown Timer Check
    if (isSOSCountdown) {
        if (millis() - sosCountdownStartTime > 5000) {
            isSOSCountdown = false;
            toggleSOS();
            triggerVibration();
            delay(100);
            triggerVibration();
            delay(100);
            triggerVibration();
        }
    }

    // 4. Auto-Home Logic (Immediate on Fix if not set)
    if (!hasHome && gps.location.isValid()) {
        // Removed 5 min timer check - Set immediately on first fix
        homeLat = gps.location.lat();
        homeLon = gps.location.lng();
        hasHome = true;
        #ifndef HELTEC_T114
        preferences.putDouble("lat", homeLat);
        preferences.putDouble("lon", homeLon);
        #endif
        Serial.println("Auto-Home Set (First Fix)!");
        
        // Visual Feedback
        String latStr = "Lat: " + String(homeLat, 4);
        String lonStr = "Lon: " + String(homeLon, 4);
        showFeedback("HOME SET!", latStr + "\n" + lonStr, 2000);
        lastInteractionTime = millis();
    }

    // 5. Breadcrumb Logic (Recording with Filter)
    if (currentMode == MODE_EXPLORE && gps.location.isValid()) {
        bool shouldSave = false;
        
        // Filter: Only save if moving (> 1 km/h) AND not too fast (< 15 km/h) to prevent GPS drift/glitches
        double speed = gps.speed.kmph();
        if (speed > MIN_SPEED_KPH && speed < MAX_SPEED_KPH) {
            if (breadcrumbs.empty()) {
                shouldSave = true;
            } else {
                Breadcrumb last = breadcrumbs.back();
                double dist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), last.lat, last.lon);
                
                // Condition 1: Distance Threshold (Regular interval)
                if (dist > BREADCRUMB_DIST) {
                    shouldSave = true;
                }
                // Condition 2: Smart Turn Detection
                // If we moved enough to detect a turn (e.g. 20m) AND the direction changed significantly
                else if (dist > BREADCRUMB_MIN_DIST_TURN) {
                    // Calculate bearing from last breadcrumb to current position
                    double currentSegmentBearing = gps.courseTo(last.lat, last.lon, gps.location.lat(), gps.location.lng());
                    
                    // We need the bearing of the PREVIOUS segment to compare.
                    // If we only have 1 breadcrumb, we can't compare segments.
                    if (breadcrumbs.size() >= 2) {
                        Breadcrumb prev = breadcrumbs[breadcrumbs.size()-2];
                        double prevSegmentBearing = gps.courseTo(prev.lat, prev.lon, last.lat, last.lon);
                        
                        double diff = abs(currentSegmentBearing - prevSegmentBearing);
                        if (diff > 180) diff = 360 - diff; // Normalize to 0-180
                        
                        if (diff > BREADCRUMB_TURN_THRESHOLD) {
                            shouldSave = true;
                            Serial.printf("Smart Breadcrumb: Turn detected (%.1f deg)\n", diff);
                        }
                    }
                }
            }
        }
        
        if (shouldSave) {
            // Ringbuffer Limit Check
            if (breadcrumbs.size() >= MAX_BREADCRUMBS) {
                breadcrumbs.erase(breadcrumbs.begin()); // Delete oldest
            }

            Breadcrumb b;
            b.lat = gps.location.lat();
            b.lon = gps.location.lng();
            breadcrumbs.push_back(b);
            
            // Save to Flash immediately
            saveBreadcrumbs();
            
            Serial.printf("Breadcrumb saved: %f, %f (Total: %d)\n", b.lat, b.lon, breadcrumbs.size());
        }
    }

    // 6. Backtrack Logic (Target Update)
    if (currentMode == MODE_BRING_HOME && gps.location.isValid() && !breadcrumbs.empty()) {
        if (targetBreadcrumbIndex >= 0) {
            double distToTarget = gps.distanceBetween(gps.location.lat(), gps.location.lng(), 
                                                    breadcrumbs[targetBreadcrumbIndex].lat, 
                                                    breadcrumbs[targetBreadcrumbIndex].lon);
            // If we reached the target (within 20m), move to previous
            if (distToTarget < 20.0) {
                targetBreadcrumbIndex--;
                triggerVibration();
                // If index < 0, we are past the last breadcrumb, target becomes Home
            }
        }
    }

    // 7. Vibration Logic
    if (isVibrating && (millis() - vibrationStartTime > 500)) {
        digitalWrite(PIN_VIB_MOTOR, LOW);
        isVibrating = false;
    }

    // 8. Display Timeout
    if (isDisplayOn && (millis() - lastInteractionTime > DISPLAY_TIMEOUT)) {
        u8g2.setPowerSave(1); // Display OFF
        isDisplayOn = false;
        #ifndef HELTEC_T114
        setCpuFrequencyMhz(80); // Low power mode
        #endif
    }

    // 9. Update Display
    if (isDisplayOn) {
        // Ensure high performance when display is ON
        #ifndef HELTEC_T114
        if (getCpuFrequencyMhz() < 240) setCpuFrequencyMhz(240);
        #endif

        static unsigned long lastUpdate = 0;
        if (millis() - lastUpdate > 100) { // 10Hz update for smooth compass
            lastUpdate = millis();
            
            u8g2.clearBuffer();
            int w = 0;

            // --- Charging Mode ---
            if (isCharging) {
                u8g2.setFont(u8g2_font_ncenB10_tr);
                const char* title = "Loading battery...";
                w = u8g2.getStrWidth(title);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 30);
                u8g2.print(title);
                
                // Battery Icon Large
                u8g2.drawFrame(44, 50, 40, 20); // Body
                u8g2.drawBox(84, 56, 4, 8);     // Terminal
                
                // Animated Fill
                static int chargeAnim = 0;
                static unsigned long lastChargeAnim = 0;
                if (millis() - lastChargeAnim > 500) {
                    chargeAnim = (chargeAnim + 1) % 5;
                    lastChargeAnim = millis();
                }
                // 4 bars inside
                for(int i=0; i<chargeAnim; i++) {
                    u8g2.drawBox(46 + (i*9), 52, 7, 16);
                }

                // FEEDBACK OVERLAY ---
            if (millis() < feedbackEndTime) {
                u8g2.setFont(u8g2_font_ncenB10_tr);
                w = u8g2.getStrWidth(feedbackTitle.c_str());
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 50);
                u8g2.print(feedbackTitle);
                
                if (feedbackSub.length() > 0) {
                    u8g2.setFont(u8g2_font_6x10_tr);
                    // Handle simple newline for 2 lines max
                    int nl = feedbackSub.indexOf('\n');
                    if (nl > 0) {
                        String l1 = feedbackSub.substring(0, nl);
                        String l2 = feedbackSub.substring(nl+1);
                        w = u8g2.getStrWidth(l1.c_str());
                        u8g2.setCursor((SCREEN_WIDTH - w) / 2, 75);
                        u8g2.print(l1);
                        w = u8g2.getStrWidth(l2.c_str());
                        u8g2.setCursor((SCREEN_WIDTH - w) / 2, 90);
                        u8g2.print(l2);
                    } else {
                        w = u8g2.getStrWidth(feedbackSub.c_str());
                        u8g2.setCursor((SCREEN_WIDTH - w) / 2, 80);
                        u8g2.print(feedbackSub);
                    }
                }
                u8g2.sendBuffer();
                return;
            }

            // --- Estimated Time
                float hoursLeft = estimatedChargeTimeHours - ((float)(millis() - chargeStartTime) / 3600000.0);
                if (hoursLeft < 0) hoursLeft = 0;
                
                String timeStr = "Est: " + String(hoursLeft, 1) + "h";
                u8g2.setFont(u8g2_font_ncenB12_tr);
                w = u8g2.getStrWidth(timeStr.c_str());
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 90);
                u8g2.print(timeStr);

                u8g2.sendBuffer();
                return;
            }

            // --- SOS Countdown ---
            if (isSOSCountdown) {
                u8g2.setFont(u8g2_font_ncenB12_tr);
                const char* title = "SOS MODE IN";
                w = u8g2.getStrWidth(title);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 20);
                u8g2.print(title);

                // Countdown Number
                int remaining = 5 - (millis() - sosCountdownStartTime) / 1000;
                if (remaining < 0) remaining = 0;
                
                u8g2.setFont(u8g2_font_logisoso42_tn);
                String cntStr = String(remaining);
                w = u8g2.getStrWidth(cntStr.c_str());
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 45); // Centered vertically (adjusted)
                u8g2.print(cntStr);
                
                u8g2.setFont(u8g2_font_6x10_tr);
                const char* sub = "Press button to Cancel";
                w = u8g2.getStrWidth(sub);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 110);
                u8g2.print(sub);

                u8g2.sendBuffer();
                return;
            }

            // --- SOS MODE ---
            if (isSOSActive) {
                // Huge SOS Display
                u8g2.setFont(u8g2_font_ncenB14_tr);
                const char* title = "SOS ACTIVE";
                w = u8g2.getStrWidth(title);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 20);
                u8g2.print(title);

                // Countdown
                int secToNext = (LORA_TX_INTERVAL - (millis() - lastLoRaTx)) / 1000;
                if (secToNext < 0) secToNext = 0;
                
                u8g2.setFont(u8g2_font_logisoso42_tn);
                String cntStr = String(secToNext);
                w = u8g2.getStrWidth(cntStr.c_str());
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 45); // Centered vertically (adjusted)
                u8g2.print(cntStr);

                // Footer info
                u8g2.setFont(u8g2_font_6x10_tr);
                const char* sub = "Sending Location...";
                w = u8g2.getStrWidth(sub);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 110);
                u8g2.print(sub);

                u8g2.sendBuffer();
                return; // Skip normal display
            }
            
            // --- Header ---
            // Top Line
            u8g2.drawHLine(0, 15, 128);
            u8g2.setFont(u8g2_font_5x7_tr);

            // 1. Battery (Left)
            // Draw Vertical Battery Icon
            u8g2.drawFrame(0, 4, 6, 10); // Body
            u8g2.drawBox(2, 2, 2, 2);    // Terminal (Knob)
            
            // Fill Level
            int pct = getBatteryPercent();
            int h = (pct * 8) / 100; // Max height 8px inside
            if (h > 0) u8g2.drawBox(1, 4 + (10-1-h), 4, h);
            
            // Text
            u8g2.setCursor(8, 12); // Right of icon
            u8g2.print(pct);
            u8g2.print("%");

            // 2. Compass (Center)
            String compStr = "Bad";
            if (mag == 3) compStr = "Good";
            else if (mag == 2) compStr = "Ok";
            else if (mag == 1) compStr = "Low";
            
            String compDisp = "C:" + compStr;
            w = u8g2.getStrWidth(compDisp.c_str());
            u8g2.setCursor((128 - w) / 2, 10);
            u8g2.print(compDisp);

            // 3. SAT (Right) - 4 Bars
            int sats = gps.satellites.value();
            int bars = 0;
            if (gps.location.isValid()) {
                if (sats >= 6) bars = 4;
                else if (sats >= 5) bars = 3;
                else if (sats >= 4) bars = 2;
                else bars = 1;
            }
            
            // Draw 4 Bars (Right aligned)
            int startX = 128 - (4 * 4); 
            for (int i=0; i<4; i++) {
                int h = (i+1) * 2 + 2; // 4, 6, 8, 10
                if (i < bars) {
                    u8g2.drawBox(startX + (i*4), 14-h, 3, h);
                } else {
                    u8g2.drawFrame(startX + (i*4), 14-h, 3, h);
                }
            }

            // --- Main Content ---
            
            // If NO GPS, show Text Animation
            if (!gps.location.isValid()) {
                u8g2.setFont(u8g2_font_ncenB10_tr);
                const char* baseTxt = "Searching SATs";
                int w = u8g2.getStrWidth(baseTxt);
                
                // Center the text
                int startX = (SCREEN_WIDTH - w) / 2;
                int y = 60;

                u8g2.setCursor(startX, y);
                u8g2.print(baseTxt);

                // Progress Bar Animation
                int barWidth = 64; // Half screen width
                int barHeight = 12;
                int barX = (SCREEN_WIDTH - barWidth) / 2;
                int barY = y + 15;

                u8g2.drawFrame(barX, barY, barWidth, barHeight);

                static int animPos = 0;
                static unsigned long lastAnim = 0;
                if (millis() - lastAnim > 50) {
                    animPos = (animPos + 2) % (barWidth - 3);
                    lastAnim = millis();
                }
                
                // Draw filling bar
                if (animPos > 0) u8g2.drawBox(barX + 2, barY + 2, animPos, barHeight - 4);

                u8g2.sendBuffer();
                return; // Skip rest of drawing
            }

            // Target Info
            double targetLat = homeLat;
            double targetLon = homeLon;
            bool targetIsHome = true;
            double distToTarget = 0;
            
            if (currentMode == MODE_BRING_HOME && targetBreadcrumbIndex >= 0 && !breadcrumbs.empty()) {
                targetLat = breadcrumbs[targetBreadcrumbIndex].lat;
                targetLon = breadcrumbs[targetBreadcrumbIndex].lon;
                targetIsHome = false;
                if (gps.location.isValid()) {
                    distToTarget = gps.distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLon);
                }
            }

            // Distance & Arrow Logic
            double dist = 0;
            double bearing = 0;
            bool showArrow = false;

            if (currentMode == MODE_EXPLORE) {
                 // Compass Mode - Always Active
                 if (gps.location.isValid() && hasHome) {
                    dist = calculateTotalDistanceToHome();
                 }
                 bearing = 0; // North
                 showArrow = true;
            } else {
                // Backtrack Mode - Needs GPS
                if (gps.location.isValid()) {
                    dist = calculateTotalDistanceToHome();
                    bearing = gps.courseTo(gps.location.lat(), gps.location.lng(), targetLat, targetLon);
                    showArrow = true;
                }
            }

            // Draw Arrow (Center)
            if (showArrow) {
                int relBearing = (int)bearing - currentHeading;
                if (relBearing < 0) relBearing += 360;
                
                // Draw fancy needle
                int arrowCy = 64; // Center of screen
                // Show cardinals only in Explore Mode (where arrow points North)
                bool showCardinals = (currentMode == MODE_EXPLORE);
                drawArrow(SCREEN_WIDTH/2, arrowCy, 30, relBearing, showCardinals);
            } else {
                // No GPS (Backtracking) or No Home (Recording - but arrow is always shown now)
                if (currentMode == MODE_BRING_HOME && !gps.location.isValid()) {
                    u8g2.setFont(u8g2_font_ncenB10_tr);
                    u8g2.drawStr(35, 60, "NO GPS");
                }
            }

            // Distance & Label (Bottom Footer)
            if (gps.location.isValid() && (hasHome || currentMode == MODE_BRING_HOME)) {
                if (!targetIsHome) {
                    // Backtracking Mode: Show Next WP AND Home
                    
                    // Line 1: Next WP (Small)
                    u8g2.setFont(u8g2_font_6x10_tr);
                    u8g2.setCursor(0, 110);
                    u8g2.print("NEXT:");
                    
                    String nextStr;
                    if (distToTarget < 1000) nextStr = String((int)distToTarget) + "m";
                    else nextStr = String(distToTarget / 1000.0, 2) + "km";
                    
                    int w = u8g2.getStrWidth(nextStr.c_str());
                    u8g2.setCursor(SCREEN_WIDTH - w, 110);
                    u8g2.print(nextStr);

                    // Line 2: Home (Total)
                    u8g2.setFont(u8g2_font_ncenB10_tr);
                    u8g2.setCursor(0, 125);
                    u8g2.print("HOME");

                    String distStr = String(dist / 1000.0, 2) + " km";
                    u8g2.setFont(u8g2_font_ncenB12_tr);
                    w = u8g2.getStrWidth(distStr.c_str());
                    u8g2.setCursor(SCREEN_WIDTH - w, 125);
                    u8g2.print(distStr);
                } else {
                    // Direct Mode or Final Leg
                    String label = "HOME";
                    u8g2.setFont(u8g2_font_ncenB10_tr);
                    u8g2.setCursor(0, 125);
                    u8g2.print(label);

                    // Distance (Bottom Right) - Always in km
                    String distStr = String(dist / 1000.0, 2) + " km";
                    
                    u8g2.setFont(u8g2_font_ncenB12_tr);
                    w = u8g2.getStrWidth(distStr.c_str());
                    u8g2.setCursor(SCREEN_WIDTH - w, 125);
                    u8g2.print(distStr);
                }
            } else if (currentMode == MODE_EXPLORE && !hasHome) {
                 if (gps.location.isValid()) {
                    u8g2.setFont(u8g2_font_ncenB10_tr);
                    u8g2.drawStr(25, 110, "SET HOME");
                 } else {
                    u8g2.setFont(u8g2_font_6x10_tr);
                    u8g2.drawStr(25, 110, "WAITING GPS");
                 }
            }

            u8g2.sendBuffer();
        }
    }
}