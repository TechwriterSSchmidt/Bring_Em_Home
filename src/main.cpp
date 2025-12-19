// ESP32-S3 - Bring Em Home Project
// Using U8g2 library for SH1107 128x128 OLED (I2C)
// Hardware: ESP32-S3 + SH1107 OLED + BNO055 + ATGM336H

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Preferences.h>
#include <vector>
#include <WiFi.h> // For power management
#include <esp_task_wdt.h>
#include <RadioLib.h>
#include <Adafruit_NeoPixel.h>
#include <LittleFS.h>
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
unsigned long lastSOSUpdate = 0;
int sosStep = 0;

// Objects
TinyGPSPlus gps;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Preferences preferences;
SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);
SPIClass loraSPI(HSPI);
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

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

int getBatteryPercent() {
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
}

void updateSOS() {
    if (!isSOSActive) return;
    
    unsigned long now = millis();

    // --- LoRa Transmission (Every 1 Minute) ---
    if (now - lastLoRaTx > LORA_TX_INTERVAL) {
        lastLoRaTx = now;
        String msg = "SOS! Lat:" + String(gps.location.lat(), 6) + 
                     " Lon:" + String(gps.location.lng(), 6) + 
                     " Bat:" + String(getBatteryPercent()) + "%" +
                     " Type:" + USER_BLOOD_TYPE;
        Serial.print("Sending LoRa SOS: "); Serial.println(msg);
        
        // Wake up and send
        radio.standby();
        int state = radio.transmit(msg);
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("LoRa TX Success!");
        } else {
            Serial.print("LoRa TX Failed, code "); Serial.println(state);
        }
        // Go back to sleep to save power until next TX
        radio.sleep();
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
        u8g2.setFont(u8g2_font_6x10_tr);
        int r_text = r + 12; // Radius for text
        
        struct Cardinal { const char* label; float offset; };
        Cardinal dirs[] = { {"N", 0}, {"E", 90}, {"S", 180}, {"W", 270} };
        
        for (auto& d : dirs) {
            float a = (angleDeg + d.offset - 90) * PI / 180.0;
            int tx = cx + r_text * cos(a);
            int ty = cy + r_text * sin(a);
            
            // Center text
            int w = u8g2.getStrWidth(d.label);
            int h = 8; // approx height
            u8g2.setCursor(tx - w/2, ty - h/2);
            u8g2.print(d.label);
        }
    }
}

// --- Helper Functions ---

void saveBreadcrumbs() {
    File file = LittleFS.open(BREADCRUMB_FILE, "w");
    if (file) {
        for (const auto& b : breadcrumbs) {
            file.write((uint8_t*)&b, sizeof(Breadcrumb));
        }
        file.close();
    }
}

void loadBreadcrumbs() {
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
    ESP.restart(); // Software-Reset durchführen
}

// Helper: Update RGB Status LED
void updateStatusLED() {
    static unsigned long lastBlink = 0;
    static int pulseBrightness = 0;
    static int pulseDir = 5;
    static unsigned long lastPulseTime = 0;

    // 1. Battery Warning (< 10%)
    // Heltec Wireless Tracker has voltage divider on GPIO 1.
    // ADC value 0-4095. Reference 3.3V. Divider factor ~2 (needs calibration).
    // Assuming 3.3V logic, 3.7V LiPo.
    // For now, we use a placeholder or simple check if we had the calibration data.
    // Since we don't have calibration, we skip the actual reading to avoid false alarms.
    // TODO: Implement ADC reading on GPIO 1.
    bool lowBattery = (getBatteryPercent() < 10); 
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

void setup() {
    // Power on VExt for sensors (GPS, LoRa, OLED)
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, HIGH);
    delay(100); // Wait for power to stabilize

    // ADC Setup for Battery
    analogSetAttenuation(ADC_11db);

    Serial.begin(SERIAL_BAUD);

    // Init NeoPixel
    pixels.begin();
    pixels.setBrightness(LED_BRIGHTNESS); // 40% Brightness (approx 100/255)
    pixels.clear();
    pixels.show();

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

    delay(2000);
    Serial.println("\n\n=== ESP32-S3 Bring Em Home ===");
    
    // Button Setup
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    lastInteractionTime = millis();

    // Vibration Setup
    pinMode(PIN_VIB_MOTOR, OUTPUT);
    digitalWrite(PIN_VIB_MOTOR, LOW);

    // Flashlight Setup
    pinMode(PIN_FLASHLIGHT, OUTPUT);
    digitalWrite(PIN_FLASHLIGHT, LOW);

    // Initialize Sensors
    if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL)) {
        fatalError("I2C Bus Fehler!");
    }
    
    // Initialize BNO055
    if (!bno.begin()) {
        Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
        fatalError("BNO055 Error!");
    }
    bno.setExtCrystalUse(true);

    // Initialize GPS (ATGM336H typically 9600 baud)
    Serial1.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    // GPS defaults to ON when powered

    // Initialize LoRa (SX1262)
    loraSPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
    int state = radio.begin(LORA_FREQ, 125.0, 9, 7, 0x12, 10, 8, 0, false); // EU868
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("LoRa Init Success!");
        radio.sleep(); // Sleep immediately
    } else {
        Serial.print("LoRa Init Failed, code "); Serial.println(state);
        // Non-fatal, we continue
    }
    
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
    } else {
        Serial.printf("Reset Reason: %d. Attempting to restore session.\n", reason);
        homeLat = preferences.getDouble("lat", 0.0);
        homeLon = preferences.getDouble("lon", 0.0);
        hasHome = (homeLat != 0.0 && homeLon != 0.0);
        // Breadcrumbs laden
        loadBreadcrumbs();
    }
    Serial.printf("Home: %f, %f (HasHome: %d)\n", homeLat, homeLon, hasHome);
    
    startTime = millis(); // Start timer for auto-home

    // Initialize display
    u8g2.begin();
    u8g2.setDrawColor(1); // White
    u8g2.setFontPosTop(); // Easier coordinate handling
    
    // Memory Debugging
    Serial.printf("Total Heap: %d, Free Heap: %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
    Serial.printf("Total PSRAM: %d, Free PSRAM: %d\n", ESP.getPsramSize(), ESP.getFreePsram());

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
    esp_task_wdt_reset();

    // Update Status LED (50Hz)
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate > 20) {
        updateStatusLED();
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
        
        // 10s: Reset Home (Extra Long Press)
        if (!isLongPressHandled && pressDuration > 10000) {
            if (currentMode == MODE_EXPLORE && gps.location.isValid()) {
                // Action
                homeLat = gps.location.lat();
                homeLon = gps.location.lng();
                preferences.putDouble("lat", homeLat);
                preferences.putDouble("lon", homeLon);
                hasHome = true;
                
                // Feedback (LED & Vibration)
                isLongPressHandled = true;
                
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
                if (!isDisplayOn) { u8g2.setPowerSave(0); isDisplayOn = true; }
                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_ncenB12_tr);
                u8g2.drawStr(10, 60, "HOME RESET!");
                u8g2.sendBuffer();
                delay(2000);
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
        if (clickCount == 1) {
            // Single Click: Toggle Display
            if (isDisplayOn) {
                u8g2.setPowerSave(1); // Display OFF
                isDisplayOn = false;
                setCpuFrequencyMhz(80); // Low power mode
            } else {
                u8g2.setPowerSave(0); // Display ON
                isDisplayOn = true;
                setCpuFrequencyMhz(240); // High performance mode
                lastInteractionTime = millis();
            }
        } else if (clickCount >= 2) {
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
                if (!isDisplayOn) { 
                    u8g2.setPowerSave(0); 
                    isDisplayOn = true; 
                    // setCpuFrequencyMhz(240); // Keep at 80MHz
                }
                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_ncenB10_tr); // Smaller font to fit
                u8g2.drawStr(5, 60, "BRING ME HOME!");
                u8g2.sendBuffer();
                delay(1000);
            } else {
                currentMode = MODE_EXPLORE;
                // Feedback
                if (!isDisplayOn) { 
                    u8g2.setPowerSave(0); 
                    isDisplayOn = true; 
                    // setCpuFrequencyMhz(240); // Keep at 80MHz
                }
                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_ncenB12_tr);
                u8g2.drawStr(20, 60, "EXPLORER");
                u8g2.sendBuffer();
                delay(1000);
            }
            lastInteractionTime = millis();
        } else if (clickCount == 3) {
            // Triple Click: Flashlight
            toggleFlashlight();
            triggerVibration();
        } else if (clickCount >= 5) {
            // 5x Click: SOS (LoRa + LED)
            toggleSOS();
            triggerVibration();
            delay(100);
            triggerVibration();
            delay(100);
            triggerVibration();
        }
        clickCount = 0;
    }

    // 4. Auto-Home Logic (Immediate on Fix if not set)
    if (!hasHome && gps.location.isValid()) {
        // Removed 5 min timer check - Set immediately on first fix
        homeLat = gps.location.lat();
        homeLon = gps.location.lng();
        hasHome = true;
        preferences.putDouble("lat", homeLat);
        preferences.putDouble("lon", homeLon);
        Serial.println("Auto-Home Set (First Fix)!");
        
        // Visual Feedback
        if (!isDisplayOn) { u8g2.setPowerSave(0); isDisplayOn = true; }
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB12_tr);
        u8g2.drawStr(10, 50, "HOME SET!");
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(10, 80);
        u8g2.print("Lat: "); u8g2.print(homeLat, 4);
        u8g2.setCursor(10, 95);
        u8g2.print("Lon: "); u8g2.print(homeLon, 4);
        u8g2.sendBuffer();
        delay(2000);
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
                if (dist > BREADCRUMB_DIST) {
                    shouldSave = true;
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
        setCpuFrequencyMhz(80); // Low power mode
    }

    // 9. Update Display
    if (isDisplayOn) {
        // Ensure high performance when display is ON
        if (getCpuFrequencyMhz() < 240) setCpuFrequencyMhz(240);

        static unsigned long lastUpdate = 0;
        if (millis() - lastUpdate > 100) { // 10Hz update for smooth compass
            lastUpdate = millis();
            
            u8g2.clearBuffer();
            int w = 0;

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
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 80);
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
                // Animation Timer
                static int animPhase = 0;
                static unsigned long lastAnim = 0;
                if (millis() - lastAnim > 500) {
                    animPhase = (animPhase + 1) % 4;
                    lastAnim = millis();
                }

                u8g2.setFont(u8g2_font_ncenB10_tr);
                const char* baseTxt = "Searching SATs";
                int w = u8g2.getStrWidth(baseTxt);
                
                // Center the text block roughly (accounting for dots width approx 15px)
                int totalW = w + 15; 
                int startX = (SCREEN_WIDTH - totalW) / 2;
                int y = 64;

                u8g2.setCursor(startX, y);
                u8g2.print(baseTxt);

                // Animated Dots
                if (animPhase >= 1) u8g2.print(" .");
                if (animPhase >= 2) u8g2.print(" .");
                if (animPhase >= 3) u8g2.print(" .");

                u8g2.sendBuffer();
                return; // Skip rest of drawing
            }

            // Target Info
            double targetLat = homeLat;
            double targetLon = homeLon;
            bool targetIsHome = true;
            
            if (currentMode == MODE_BRING_HOME && targetBreadcrumbIndex >= 0 && !breadcrumbs.empty()) {
                targetLat = breadcrumbs[targetBreadcrumbIndex].lat;
                targetLon = breadcrumbs[targetBreadcrumbIndex].lon;
                targetIsHome = false;
            }

            // Distance & Arrow Logic
            double dist = 0;
            double bearing = 0;
            bool showArrow = false;

            if (currentMode == MODE_EXPLORE) {
                 // Compass Mode - Always Active
                 if (gps.location.isValid() && hasHome) {
                    dist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon);
                 }
                 bearing = 0; // North
                 showArrow = true;
            } else {
                // Backtrack Mode - Needs GPS
                if (gps.location.isValid()) {
                    dist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLon);
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
                
                // N indicator for Recording mode (REMOVED - now part of drawArrow)
                /*
                if (currentMode == MODE_EXPLORE) {
                    float angleRad = (relBearing - 90) * PI / 180.0;
                    int nx = (SCREEN_WIDTH/2) + 45 * cos(angleRad);
                    int ny = arrowCy + 45 * sin(angleRad);
                    u8g2.setFont(u8g2_font_ncenB10_tr);
                    u8g2.setCursor(nx-5, ny-5);
                    u8g2.print("N");
                }
                */
            } else {
                // No GPS (Backtracking) or No Home (Recording - but arrow is always shown now)
                if (currentMode == MODE_BRING_HOME && !gps.location.isValid()) {
                    u8g2.setFont(u8g2_font_ncenB10_tr);
                    u8g2.drawStr(35, 60, "NO GPS");
                }
            }

            // Distance & Label (Bottom Footer)
            if (gps.location.isValid() && (hasHome || currentMode == MODE_BRING_HOME)) {
                // Label (Bottom Left)
                String label = targetIsHome ? "HOME" : "WAYPOINT";
                u8g2.setFont(u8g2_font_ncenB10_tr);
                u8g2.setCursor(0, 125);
                u8g2.print(label);

                // Distance (Bottom Right) - Always in km
                String distStr = String(dist / 1000.0, 2) + " km";
                
                u8g2.setFont(u8g2_font_ncenB12_tr);
                w = u8g2.getStrWidth(distStr.c_str());
                u8g2.setCursor(SCREEN_WIDTH - w, 125);
                u8g2.print(distStr);
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
    
    delay(10);
}