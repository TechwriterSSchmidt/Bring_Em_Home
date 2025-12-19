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
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEBeacon.h>
#include <esp_task_wdt.h>
#include <RadioLib.h>
#include <Adafruit_NeoPixel.h>

#define WDT_TIMEOUT 10 // 10 Seconds Watchdog Timeout

// ESP32-S3 Pin Configuration (Heltec Wireless Tracker)
#define BUTTON_PIN 0  // Built-in Boot Button (PRG) or External on GPIO 0
#define VIB_PIN 7     // Vibration Motor (Moved from 13)
#define FLASHLIGHT_PIN 5 // High Power LED (Moved from 21)
#define VEXT_PIN 3    // Power Control for Sensors
#define PIN_NEOPIXEL 18 // Internal WS2812 LED

// LoRa Pins (SX1262)
#define LORA_NSS 8
#define LORA_DIO1 14
#define LORA_RST 12
#define LORA_BUSY 13
#define LORA_SCK 9
#define LORA_MISO 11
#define LORA_MOSI 10

// Sensor Pins
#define GPS_RX 34     // Internal GPS RX
#define GPS_TX 33     // Internal GPS TX
#define I2C_SDA 41    // External I2C SDA
#define I2C_SCL 42    // External I2C SCL

// Display dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128

// Power Management
#define DISPLAY_TIMEOUT 300000 // 5 minutes in milliseconds
unsigned long lastInteractionTime = 0;
bool isDisplayOn = true;
bool lastButtonState = HIGH;
unsigned long buttonPressStartTime = 0;
bool isLongPressHandled = false;

// Vibration Control
unsigned long vibrationStartTime = 0;
bool isVibrating = false;

// Feature State
bool isBeaconActive = false;
bool isFlashlightOn = false;
bool isSOSActive = false;
unsigned long lastSOSUpdate = 0;
int sosStep = 0;
#define BEACON_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
BLEAdvertising *pAdvertising;

// Objects
TinyGPSPlus gps;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Preferences preferences;
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
SPIClass loraSPI(HSPI);
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// LoRa State
unsigned long lastLoRaTx = 0;
const unsigned long LORA_INTERVAL = 120000; // 2 Minutes

// U8g2 Display Object (SH1107 128x128 I2C)
// Full framebuffer (F) for smooth animation
U8G2_SH1107_128X128_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// App Modes
enum AppMode {
    MODE_RECORDING,
    MODE_BACKTRACKING
};
AppMode currentMode = MODE_RECORDING;

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
const double BREADCRUMB_DISTANCE = 250.0; // Meters
int targetBreadcrumbIndex = -1; // For backtracking

// Button Logic (Multi-click)
int clickCount = 0;
unsigned long lastClickTime = 0;
const int CLICK_DELAY = 500; // ms to wait for multi-click (Increased for 5-click SOS)

void triggerVibration() {
    digitalWrite(VIB_PIN, HIGH);
    vibrationStartTime = millis();
    isVibrating = true;
}

void initBLE() {
    BLEDevice::init("Emilie_Beacon");
    BLEServer *pServer = BLEDevice::createServer();
    pAdvertising = BLEDevice::getAdvertising();
    BLEService *pService = pServer->createService(BEACON_UUID);
    pAdvertising->addServiceUUID(BEACON_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  
    pAdvertising->setMinPreferred(0x12);
}

void setBeacon(bool enable) {
    if (enable) {
        BLEDevice::startAdvertising();
        isBeaconActive = true;
        Serial.println("BLE Beacon STARTED");
    } else {
        BLEDevice::stopAdvertising();
        isBeaconActive = false;
        Serial.println("BLE Beacon STOPPED");
    }
}

void toggleFlashlight() {
    isFlashlightOn = !isFlashlightOn;
    isSOSActive = false; // Disable SOS if manual light is toggled
    digitalWrite(FLASHLIGHT_PIN, isFlashlightOn ? HIGH : LOW);
}

void toggleSOS() {
    isSOSActive = !isSOSActive;
    isFlashlightOn = false; // Disable manual light
    if (!isSOSActive) {
        digitalWrite(FLASHLIGHT_PIN, LOW);
        radio.sleep(); // Put LoRa to sleep
        Serial.println("SOS Deactivated. LoRa Sleeping.");
    } else {
        Serial.println("SOS Activated! LoRa Waking up.");
        // Wake up LoRa (re-init if needed or just start sending)
        // We will handle TX in the loop
    }
}

void updateSOS() {
    if (!isSOSActive) return;
    
    unsigned long now = millis();

    // --- LoRa Transmission (Every 2 Minutes) ---
    if (now - lastLoRaTx > LORA_INTERVAL) {
        lastLoRaTx = now;
        String msg = "SOS! Lat:" + String(gps.location.lat(), 6) + " Lon:" + String(gps.location.lng(), 6);
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
        digitalWrite(FLASHLIGHT_PIN, (sosStep % 2 == 0) ? HIGH : LOW);
    }
}

// Helper: Draw Rotated Arrow (Fancy Needle)
void drawArrow(int cx, int cy, int r, float angleDeg) {
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
    bool lowBattery = false; 

    if (lowBattery) {
        if (millis() - lastBlink > 10000) { // Every 10 seconds
            pixels.setPixelColor(0, pixels.Color(255, 180, 0)); // Yellow
            pixels.show();
            delay(100); // Short flash
            pixels.clear();
            pixels.show();
            lastBlink = millis();
        }
        return; // Priority over GPS
    }

    // 2. GPS Search (Red Pulsing)
    if (!gps.location.isValid()) {
        if (millis() - lastPulseTime > 20) { // Smooth fading
            pixels.setPixelColor(0, pixels.Color(pulseBrightness, 0, 0));
            pixels.show();
            pulseBrightness += pulseDir;
            if (pulseBrightness >= 100 || pulseBrightness <= 0) { // Max brightness 100
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
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, HIGH);
    delay(100); // Wait for power to stabilize

    Serial.begin(115200);

    // Init NeoPixel
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.show();

    // Initialize Watchdog Timer
    esp_task_wdt_init(WDT_TIMEOUT, true); // Enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); // Add current thread to WDT watch

    // Power Optimization: Turn off Radios
    WiFi.mode(WIFI_OFF);
    btStop();
    setCpuFrequencyMhz(240); // Start with high performance for UI

    // Wakeup Safety Check (Prevent accidental pocket activation)
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        // Woke up by button. Verify hold duration.
        unsigned long wakeStart = millis();
        bool wakeConfirmed = false;
        
        // Wait for user to hold button for 2 seconds
        while (digitalRead(BUTTON_PIN) == LOW) {
            if (millis() - wakeStart > 2000) { 
                wakeConfirmed = true;
                break;
            }
            delay(10);
        }
        
        if (!wakeConfirmed) {
            // Button released too early -> Go back to sleep immediately
            esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
            esp_deep_sleep_start();
        }
    }

    delay(2000);
    Serial.println("\n\n=== ESP32-S3 Bring Em Home ===");
    
    // Button Setup
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    lastInteractionTime = millis();

    // Vibration Setup
    pinMode(VIB_PIN, OUTPUT);
    digitalWrite(VIB_PIN, LOW);

    // Flashlight Setup
    pinMode(FLASHLIGHT_PIN, OUTPUT);
    digitalWrite(FLASHLIGHT_PIN, LOW);

    // Initialize Sensors
    if (!Wire.begin(I2C_SDA, I2C_SCL)) {
        fatalError("I2C Bus Fehler!");
    }
    
    // Initialize BNO055
    if (!bno.begin()) {
        Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
        fatalError("BNO055 Error!");
    }
    bno.setExtCrystalUse(true);

    // Initialize GPS (ATGM336H typically 9600 baud)
    Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    // GPS defaults to ON when powered

    // Initialize LoRa (SX1262)
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    int state = radio.begin(868.0, 125.0, 9, 7, 0x12, 10, 8, 0, false); // EU868
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
    
    // Smart Home Loading: Only load if NOT a fresh power-on
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_POWERON) {
        Serial.println("Power On Reset: Clearing Home for new hike.");
        hasHome = false; 
        // We don't delete from prefs yet, we just ignore it.
        // It will be overwritten when we get a fix.
    } else {
        Serial.printf("Reset Reason: %d. Attempting to restore session.\n", reason);
        homeLat = preferences.getDouble("lat", 0.0);
        homeLon = preferences.getDouble("lon", 0.0);
        hasHome = (homeLat != 0.0 && homeLon != 0.0);
    }
    Serial.printf("Home: %f, %f (HasHome: %d)\n", homeLat, homeLon, hasHome);
    
    startTime = millis(); // Start timer for auto-home

    // Initialize display
    u8g2.begin();
    u8g2.setDrawColor(1); // White
    u8g2.setFontPosTop(); // Easier coordinate handling
    
    initBLE(); // Prepare BLE
    
    // Memory Debugging
    Serial.printf("Total Heap: %d, Free Heap: %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
    Serial.printf("Total PSRAM: %d, Free PSRAM: %d\n", ESP.getPsramSize(), ESP.getFreePsram());

    u8g2.clearBuffer();
    
    // Splash Screen
    u8g2.drawHLine(10, 40, 108);
    
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(15, 60, "Bring Em");
    u8g2.drawStr(35, 80, "Home");
    
    u8g2.drawHLine(10, 90, 108);
    
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(20, 110, "Waiting for GPS...");
    u8g2.sendBuffer();
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
    bool currentButtonState = digitalRead(BUTTON_PIN);
    
    // Button Pressed
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        buttonPressStartTime = millis();
        isLongPressHandled = false;
    }
    
    // Button Held (Long Press)
    if (currentButtonState == LOW) {
        unsigned long pressDuration = millis() - buttonPressStartTime;
        
        // 3s: Toggle Beacon (One shot)
        if (!isLongPressHandled && pressDuration > 3000) {
            setBeacon(!isBeaconActive);
            triggerVibration();
            isLongPressHandled = true;
            
            // Visual Feedback
            if (!isDisplayOn) { u8g2.setPowerSave(0); isDisplayOn = true; }
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB14_tr);
            u8g2.drawStr(10, 50, isBeaconActive ? "BEACON" : "BEACON");
            u8g2.drawStr(30, 80, isBeaconActive ? "ON" : "OFF");
            u8g2.sendBuffer();
            delay(1000);
            lastInteractionTime = millis();
        }
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
            if (currentMode == MODE_RECORDING) {
                currentMode = MODE_BACKTRACKING;
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
                    setCpuFrequencyMhz(240);
                }
                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_ncenB12_tr);
                u8g2.drawStr(20, 60, "RETURN");
                u8g2.sendBuffer();
                delay(1000);
            } else {
                currentMode = MODE_RECORDING;
                // Feedback
                if (!isDisplayOn) { 
                    u8g2.setPowerSave(0); 
                    isDisplayOn = true; 
                    setCpuFrequencyMhz(240);
                }
                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_ncenB12_tr);
                u8g2.drawStr(20, 60, "EXPLORE");
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

    // 5. Breadcrumb Logic (Recording)
    if (currentMode == MODE_RECORDING && gps.location.isValid()) {
        bool shouldSave = false;
        if (breadcrumbs.empty()) {
            shouldSave = true;
        } else {
            Breadcrumb last = breadcrumbs.back();
            double dist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), last.lat, last.lon);
            if (dist > BREADCRUMB_DISTANCE) {
                shouldSave = true;
            }
        }
        
        if (shouldSave) {
            Breadcrumb b;
            b.lat = gps.location.lat();
            b.lon = gps.location.lng();
            breadcrumbs.push_back(b);
            Serial.printf("Breadcrumb saved: %f, %f\n", b.lat, b.lon);
        }
    }

    // 6. Backtrack Logic (Target Update)
    if (currentMode == MODE_BACKTRACKING && gps.location.isValid() && !breadcrumbs.empty()) {
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
        digitalWrite(VIB_PIN, LOW);
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
        static unsigned long lastUpdate = 0;
        if (millis() - lastUpdate > 100) { // 10Hz update for smooth compass
            lastUpdate = millis();
            
            u8g2.clearBuffer();
            
            // --- Header ---
            // Top Line
            u8g2.drawHLine(0, 15, 128);
            
            // Status Icons (Top Left)
            u8g2.setFont(u8g2_font_5x7_tr);
            u8g2.setCursor(0, 0);
            if (isBeaconActive) u8g2.print("B ");
            if (isFlashlightOn) u8g2.print("L ");
            if (isSOSActive) u8g2.print("SOS ");

            // Mode Title (Centered)
            u8g2.setFont(u8g2_font_6x12_tr);
            String title = (currentMode == MODE_RECORDING) ? "EXPLORE" : "RETURN";
            int w = u8g2.getStrWidth(title.c_str());
            u8g2.setCursor((SCREEN_WIDTH - w) / 2, 2);
            u8g2.print(title);

            // Satellites (Top Right)
            u8g2.setCursor(110, 0);
            if (gps.location.isValid()) {
                u8g2.print(gps.satellites.value());
            } else {
                u8g2.print("X");
            }

            // Calibration (Small, Left of Sats)
            u8g2.setCursor(90, 0);
            u8g2.printf("M:%d", mag);

            // --- Main Content ---
            
            // If NO GPS, show Satellite Dish Animation
            if (!gps.location.isValid()) {
                int cx = SCREEN_WIDTH / 2;
                int cy = 64;
                
                // Dish Base
                u8g2.drawLine(cx-10, cy+10, cx+10, cy+10);
                u8g2.drawLine(cx, cy+10, cx, cy+20);
                u8g2.drawLine(cx-5, cy+20, cx+5, cy+20);
                
                // Dish Curve (Semi-circle pointing up-right)
                // Simple approximation with lines
                u8g2.drawLine(cx-10, cy+10, cx-15, cy-5);
                u8g2.drawLine(cx-15, cy-5, cx-5, cy-15);
                u8g2.drawLine(cx-5, cy-15, cx+10, cy+10); // Chord
                
                // Antenna
                u8g2.drawLine(cx-5, cy+5, cx+5, cy-5);
                u8g2.drawDisc(cx+5, cy-5, 2); // LNB
                
                // Animated Waves
                int frame = (millis() / 300) % 4;
                if (frame >= 1) u8g2.drawCircle(cx+5, cy-5, 8, U8G2_DRAW_UPPER_RIGHT);
                if (frame >= 2) u8g2.drawCircle(cx+5, cy-5, 14, U8G2_DRAW_UPPER_RIGHT);
                if (frame >= 3) u8g2.drawCircle(cx+5, cy-5, 20, U8G2_DRAW_UPPER_RIGHT);
                
                u8g2.setFont(u8g2_font_6x10_tr);
                const char* txt = "Searching...";
                w = u8g2.getStrWidth(txt);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 100);
                u8g2.print(txt);
                
                u8g2.sendBuffer();
                return; // Skip rest of drawing
            }

            // Target Info
            double targetLat = homeLat;
            double targetLon = homeLon;
            bool targetIsHome = true;
            
            if (currentMode == MODE_BACKTRACKING && targetBreadcrumbIndex >= 0 && !breadcrumbs.empty()) {
                targetLat = breadcrumbs[targetBreadcrumbIndex].lat;
                targetLon = breadcrumbs[targetBreadcrumbIndex].lon;
                targetIsHome = false;
            }

            // Distance & Arrow Logic
            double dist = 0;
            double bearing = 0;
            bool showArrow = false;

            if (currentMode == MODE_RECORDING) {
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
                drawArrow(SCREEN_WIDTH/2, arrowCy, 35, relBearing);
                
                // N indicator for Recording mode
                if (currentMode == MODE_RECORDING) {
                    float angleRad = (relBearing - 90) * PI / 180.0;
                    int nx = (SCREEN_WIDTH/2) + 45 * cos(angleRad);
                    int ny = arrowCy + 45 * sin(angleRad);
                    u8g2.setFont(u8g2_font_ncenB10_tr);
                    u8g2.setCursor(nx-5, ny-5);
                    u8g2.print("N");
                }
            } else {
                // No GPS (Backtracking) or No Home (Recording - but arrow is always shown now)
                if (currentMode == MODE_BACKTRACKING && !gps.location.isValid()) {
                    u8g2.setFont(u8g2_font_ncenB10_tr);
                    u8g2.drawStr(35, 60, "NO GPS");
                }
            }

            // Distance Text (Bottom)
            if (gps.location.isValid() && (hasHome || currentMode == MODE_BACKTRACKING)) {
                String distStr;
                if (dist < 1000) distStr = String(dist, 0) + " m";
                else distStr = String(dist / 1000.0, 2) + " km";
                
                u8g2.setFont(u8g2_font_ncenB14_tr);
                w = u8g2.getStrWidth(distStr.c_str());
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 100);
                u8g2.print(distStr);
                
                // Label
                String label = targetIsHome ? "HOME" : "WAYPOINT";
                u8g2.setFont(u8g2_font_6x10_tr);
                w = u8g2.getStrWidth(label.c_str());
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 120); // Bottom
                u8g2.print(label);
            } else if (currentMode == MODE_RECORDING && !hasHome) {
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