// Heltec Mesh Node T114 (nRF52840) - Bring Em Home Project
// Using U8g2 library for SH1107 128x128 OLED (I2C)
// Hardware: nRF52840 + SH1107 OLED + BNO055 + L76K GNSS

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <nrf_gpio.h>
#include <nrf_power.h>
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
int lastValidBatteryPct = 50; // Default to 50% to avoid 0% division errors on boot
float estimatedChargeTimeHours = 0; 
unsigned long chargeStartTime = 0;

// Objects
TinyGPSPlus gps;
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_ID, BNO055_ADDRESS);

// Use default SPI instance for nRF52
SPIClass& loraSPI = SPI;

SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);

// Dummy NeoPixel for now as it's disabled in config
#if defined(PIN_NEOPIXEL) && (PIN_NEOPIXEL >= 0)
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#else
Adafruit_NeoPixel pixels(1, -1, NEO_GRB + NEO_KHZ800);
#endif

// LoRa State
unsigned long lastLoRaTx = 0;

// U8g2 Display Object (SH1107 128x128 I2C)
U8G2_SH1107_128X128_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// App Modes
enum AppMode {
    MODE_EXPLORE,
    MODE_RETURN
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
int targetBreadcrumbIndex = -1; 

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

// Forward Declarations
float readBatteryVoltage();
int getBatteryPercent();

void setFlag(void) {
  transmissionFlag = true;
}

void showFeedback(String title, String sub, int duration) {
    feedbackTitle = title;
    feedbackSub = sub;
    feedbackEndTime = millis() + duration;
    if (!isDisplayOn) {
        u8g2.setPowerSave(DISPLAY_POWER_SAVE_OFF);
        isDisplayOn = true;
    }
}

void triggerVibration() {
    digitalWrite(PIN_VIB_MOTOR, HIGH);
    vibrationStartTime = millis();
    isVibrating = true;
}

void toggleFlashlight() {
    isFlashlightOn = !isFlashlightOn;
    isSOSActive = false; 
    digitalWrite(PIN_FLASHLIGHT, isFlashlightOn ? HIGH : LOW);
}

void toggleSOS() {
    isSOSActive = !isSOSActive;
    isFlashlightOn = false; 
    if (!isSOSActive) {
        digitalWrite(PIN_FLASHLIGHT, LOW);
        radio.sleep(); 
        Serial.println("SOS Deactivated. LoRa Sleeping.");
    } else {
        Serial.println("SOS Activated! LoRa Waking up.");
    }
}

// --- Helper Functions ---

void powerOff() {
    Serial.println("Entering Deep Sleep...");
    
    // 1. Feedback (Falling Pattern: Long - Short - Short)
    digitalWrite(PIN_VIB_MOTOR, HIGH); delay(500);
    digitalWrite(PIN_VIB_MOTOR, LOW);  delay(200);
    digitalWrite(PIN_VIB_MOTOR, HIGH); delay(200);
    digitalWrite(PIN_VIB_MOTOR, LOW);  delay(200);
    digitalWrite(PIN_VIB_MOTOR, HIGH); delay(200);
    digitalWrite(PIN_VIB_MOTOR, LOW);

    // 2. Display OFF
    u8g2.setPowerSave(DISPLAY_POWER_SAVE_ON);
    
    // 3. Sensors OFF
    digitalWrite(PIN_VEXT, LOW);
    digitalWrite(PIN_FLASHLIGHT, LOW);

    // 4. LoRa Sleep
    radio.sleep();

    // 5. Wait for button release (Important!)
    while(digitalRead(PIN_BUTTON) == LOW) {
        delay(10);
    }

    // 6. Configure Wakeup
    nrf_gpio_cfg_sense_input(PIN_BUTTON, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    
    // Enter System OFF
    NRF_POWER->SYSTEMOFF = 1;
    while(1); // Wait for shutdown
}

float readBatteryVoltage() {
    // 1. Enable Voltage Divider (P0.06 HIGH)
    pinMode(PIN_BAT_READ_CTRL, OUTPUT);
    digitalWrite(PIN_BAT_READ_CTRL, HIGH);
    delay(10); // Wait for stabilization

    // 2. Read ADC
    int raw = analogRead(PIN_BAT_ADC);
    
    // 3. Disable Voltage Divider (Save Power)
    digitalWrite(PIN_BAT_READ_CTRL, LOW);

    // ADC Resolution default is 10-bit (0-1023) in Arduino nRF52
    // Reference is usually 3.6V internal (scaled)
    // Divider is 1/2 (100k+100k) -> Multiplier 2.0
    return (raw / 1023.0) * 3.6 * 2.0;
}

int getBatteryPercent() {
    static unsigned long lastBatCheck = 0;
    static int cachedPct = 50; // Init with safe value

    // Dynamic Interval: 60s if Display ON, 300s (5min) if Display OFF
    unsigned long interval = isDisplayOn ? BAT_CHECK_INTERVAL_ACTIVE : BAT_CHECK_INTERVAL_IDLE;

    // Update if interval passed or first run
    if (millis() - lastBatCheck > interval || lastBatCheck == 0) {
        float voltage = readBatteryVoltage();
        
        // Simple linear mapping for LiPo (3.3V to 4.2V)
        int pct = (int)((voltage - 3.3) * 100 / (4.2 - 3.3));
        if (pct > 100) pct = 100;
        if (pct < 0) pct = 0;
        
        cachedPct = pct;
        lastBatCheck = millis();
    }
    return cachedPct;
}

// Helper: Update RGB Status LED (Non-Blocking)
void updateStatusLED() {
    static unsigned long lastBlink = 0;
    static int pulseBrightness = 0;
    static int pulseDir = 5;
    static unsigned long lastPulseTime = 0;

    // New static vars for Low Battery
    static bool hasWarnedLowBattery = false;
    static unsigned long lastLowBatFlash = 0;
    static int lowBatVibState = 0;
    static unsigned long lastLowBatVibTime = 0;

    // 1. Battery Warning (< 10%)
    int pct = getBatteryPercent();
    bool lowBattery = (pct < 10); 

    if (lowBattery) {
        // Trigger Vibration ONCE when entering low battery state (Non-blocking state machine)
        if (!hasWarnedLowBattery) {
            unsigned long now = millis();
            if (lowBatVibState == 0) { // Start
                digitalWrite(PIN_VIB_MOTOR, HIGH);
                lastLowBatVibTime = now;
                lowBatVibState = 1;
            } else if (lowBatVibState == 1 && now - lastLowBatVibTime > 200) { // Off 1
                digitalWrite(PIN_VIB_MOTOR, LOW);
                lastLowBatVibTime = now;
                lowBatVibState = 2;
            } else if (lowBatVibState == 2 && now - lastLowBatVibTime > 200) { // On 2
                digitalWrite(PIN_VIB_MOTOR, HIGH);
                lastLowBatVibTime = now;
                lowBatVibState = 3;
            } else if (lowBatVibState == 3 && now - lastLowBatVibTime > 200) { // Off 2
                digitalWrite(PIN_VIB_MOTOR, LOW);
                lastLowBatVibTime = now;
                lowBatVibState = 4;
            } else if (lowBatVibState == 4 && now - lastLowBatVibTime > 200) { // On 3
                digitalWrite(PIN_VIB_MOTOR, HIGH);
                lastLowBatVibTime = now;
                lowBatVibState = 5;
            } else if (lowBatVibState == 5 && now - lastLowBatVibTime > 200) { // Done
                digitalWrite(PIN_VIB_MOTOR, LOW);
                hasWarnedLowBattery = true; // Done
                lowBatVibState = 0;
            }
        }

        // Yellow Flash every 10s
        if (millis() - lastLowBatFlash > 10000) {
            pixels.setPixelColor(0, pixels.Color(255, 200, 0)); // Yellow
            pixels.show();
            // We need a tiny delay for the flash visibility, or use state machine. 
            // 20ms is acceptable, 100ms is borderline. Let's use a quick check in main loop to turn off.
            lastLowBatFlash = millis();
        }
        
        // Turn off LED after 100ms
        if (millis() - lastLowBatFlash > 100 && millis() - lastLowBatFlash < 200) {
             pixels.clear();
             pixels.show();
        }

        return; // Override other LED patterns
    } else {
        hasWarnedLowBattery = false; // Reset if charged
        lowBatVibState = 0;
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

void updateSOS() {
    if (!isSOSActive) return;
    
    unsigned long now = millis();

    // --- LoRa Transmission (Every 1 Minute) ---
    if (now - lastLoRaTx > LORA_TX_INTERVAL) {
        lastLoRaTx = now;
        int batPct = getBatteryPercent();
        
        // Calculate estimated runtime
        float remainingCap = (float)BATTERY_CAPACITY_MAH * ((float)batPct / 100.0);
        float runtimeHours = remainingCap / (float)SOS_CURRENT_MA;

        String msg = String(SOS_MSG_TEXT) + " Lat:" + String(gps.location.lat(), 6) + 
                     " Lon:" + String(gps.location.lng(), 6) + 
                     " Bat:" + String(batPct) + "%" +
                     " Est:" + String(runtimeHours, 1) + "h";

        Serial.print("Sending LoRa SOS: "); Serial.println(msg);
        
        radio.standby();
        transmissionFlag = false;
        int state = radio.startTransmit(msg);
        
        if (state == RADIOLIB_ERR_NONE) {
            isLoRaTransmitting = true;
        } else {
            radio.sleep();
        }
    }

    // --- LED Pattern (SOS) ---
    const int DOT = 200;
    const int GAP = 200;
    static const int pattern[] = {
        DOT, GAP, DOT, GAP, DOT, GAP*3, // S
        DOT*3, GAP, DOT*3, GAP, DOT*3, GAP*3, // O (Dash=3xDot)
        DOT, GAP, DOT, GAP, DOT, GAP*7 // S + Pause
    };
    const int steps = sizeof(pattern) / sizeof(pattern[0]);
    
    if (now - lastSOSUpdate > pattern[sosStep]) {
        lastSOSUpdate = now;
        sosStep++;
        if (sosStep >= steps) sosStep = 0;
        digitalWrite(PIN_FLASHLIGHT, (sosStep % 2 == 0) ? HIGH : LOW);
    }
}

// Helper: Draw Rotated Arrow
void drawArrow(int cx, int cy, int r, float angleDeg, bool showCardinals = false) {
    float angleRad = (angleDeg - 90) * PI / 180.0;
    int x1 = cx + r * cos(angleRad);
    int y1 = cy + r * sin(angleRad);
    int x2 = cx - (r * 0.3) * cos(angleRad);
    int y2 = cy - (r * 0.3) * sin(angleRad);
    int x3 = cx + (r * 0.35) * cos(angleRad + PI/2);
    int y3 = cy + (r * 0.35) * sin(angleRad + PI/2);
    int x4 = cx + (r * 0.35) * cos(angleRad - PI/2);
    int y4 = cy + (r * 0.35) * sin(angleRad - PI/2);
    
    u8g2.drawTriangle(x1, y1, x3, y3, x2, y2);
    u8g2.drawTriangle(x1, y1, x4, y4, x2, y2);
    u8g2.drawDisc(cx, cy, 3);

    if (showCardinals) {
        u8g2.setFont(u8g2_font_5x7_tr);
        int r_text = r + 14;
        struct Cardinal { const char* label; float offset; };
        Cardinal dirs[] = { {"N", 0}, {".", 45}, {"E", 90}, {".", 135}, {"S", 180}, {".", 225}, {"W", 270}, {".", 315} };
        for (auto& d : dirs) {
            float a = (angleDeg + d.offset - 90) * PI / 180.0;
            int tx = cx + r_text * cos(a);
            int ty = cy + r_text * sin(a);
            if (strcmp(d.label, ".") == 0) u8g2.drawBox(tx - 1, ty - 1, 2, 2);
            else {
                int w = u8g2.getStrWidth(d.label);
                u8g2.setCursor(tx - w/2, ty + 3);
                u8g2.print(d.label);
            }
        }
    }
}

double calculateTotalDistanceToHome() {
    if (!gps.location.isValid() || !hasHome) return 0.0;
    double totalDist = 0.0;
    
    if (currentMode == MODE_RETURN) {
        if (targetBreadcrumbIndex >= 0 && !breadcrumbs.empty()) {
            totalDist += gps.distanceBetween(gps.location.lat(), gps.location.lng(), breadcrumbs[targetBreadcrumbIndex].lat, breadcrumbs[targetBreadcrumbIndex].lon);
            for (int i = targetBreadcrumbIndex; i > 0; i--) {
                totalDist += gps.distanceBetween(breadcrumbs[i].lat, breadcrumbs[i].lon, breadcrumbs[i-1].lat, breadcrumbs[i-1].lon);
            }
            totalDist += gps.distanceBetween(breadcrumbs[0].lat, breadcrumbs[0].lon, homeLat, homeLon);
        } else {
            totalDist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon);
        }
    } else {
        if (!breadcrumbs.empty()) {
            totalDist += gps.distanceBetween(gps.location.lat(), gps.location.lng(), breadcrumbs.back().lat, breadcrumbs.back().lon);
            for (int i = breadcrumbs.size() - 1; i > 0; i--) {
                totalDist += gps.distanceBetween(breadcrumbs[i].lat, breadcrumbs[i].lon, breadcrumbs[i-1].lat, breadcrumbs[i-1].lon);
            }
            totalDist += gps.distanceBetween(breadcrumbs[0].lat, breadcrumbs[0].lon, homeLat, homeLon);
        } else {
            totalDist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon);
        }
    }
    return totalDist;
}

void fatalError(String message) {
    Serial.println("\n!!! KRITISCHER FEHLER !!!");
    Serial.println("Grund: " + message);
    Serial.println("System wird in 5 Sekunden neu gestartet...");
    delay(2000);
    NVIC_SystemReset();
}

void setup() {
    // --- Wake-up Check (Soft-Off Logic) ---
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    
    if (digitalRead(PIN_BUTTON) == LOW) {
        unsigned long start = millis();
        bool turnOn = false;
        
        pinMode(PIN_VIB_MOTOR, OUTPUT);
        digitalWrite(PIN_VIB_MOTOR, HIGH); delay(50); digitalWrite(PIN_VIB_MOTOR, LOW);

        while (digitalRead(PIN_BUTTON) == LOW) {
            if (millis() - start > 3000) {
                turnOn = true;
                break;
            }
            delay(10);
        }
        
        if (!turnOn) {
            powerOff(); 
        } else {
            // Rising Pattern
            digitalWrite(PIN_VIB_MOTOR, HIGH); delay(100); digitalWrite(PIN_VIB_MOTOR, LOW); delay(100);
            digitalWrite(PIN_VIB_MOTOR, HIGH); delay(100); digitalWrite(PIN_VIB_MOTOR, LOW); delay(100);
            digitalWrite(PIN_VIB_MOTOR, HIGH); delay(500); digitalWrite(PIN_VIB_MOTOR, LOW);
        }
    }

    // Power on VExt for sensors (GPS, LoRa, OLED)
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, HIGH);
    
    // Init Battery Read Control
    pinMode(PIN_BAT_READ_CTRL, OUTPUT);
    digitalWrite(PIN_BAT_READ_CTRL, LOW);

    // CRITICAL FIX: Sensors need time to boot after VEXT HIGH
    delay(500); 

    Serial.begin(SERIAL_BAUD);

    // Init NeoPixel
    pixels.begin();
    pixels.setBrightness(LED_BRIGHTNESS);
    pixels.clear();
    pixels.show();

    // Initial Battery Check (to avoid 0% issues)
    lastValidBatteryPct = getBatteryPercent();
    if (lastValidBatteryPct == 0 && readBatteryVoltage() > 4.3) {
        // If we boot on charger, assume 0 for calculation purposes or handle gracefully
        lastValidBatteryPct = 1; 
    }

    delay(1000);
    Serial.println("\n\n=== Bring Em Home (T114) ===");
    
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    lastInteractionTime = millis();

    pinMode(PIN_VIB_MOTOR, OUTPUT);
    digitalWrite(PIN_VIB_MOTOR, LOW);

    pinMode(PIN_FLASHLIGHT, OUTPUT);
    digitalWrite(PIN_FLASHLIGHT, LOW);

    Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(I2C_SPEED_FAST); // Increase I2C speed to 400kHz to prevent display blocking GPS
    
    // Initialize BNO055 with Retry Logic
    bool bnoFound = false;
    for(int i=0; i<3; i++) {
        if(bno.begin()) {
            bnoFound = true;
            break;
        }
        Serial.println("BNO055 Retry...");
        delay(200);
    }
    
    if (!bnoFound) {
        Serial.println("No BNO055 detected!");
        // fatalError("BNO055 Error!"); // Optional: Don't crash if compass fails, just run without
    } else {
        bno.setExtCrystalUse(true);
    }
    
    Serial1.setPins(PIN_GPS_RX, PIN_GPS_TX);
    Serial1.begin(GPS_BAUD);

    loraSPI.setPins(PIN_LORA_MISO, PIN_LORA_SCK, PIN_LORA_MOSI);
    loraSPI.begin();

    int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC_WORD, LORA_POWER, LORA_PREAMBLE, LORA_GAIN, false);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("LoRa Init Success!");
        radio.setDio1Action(setFlag);
        radio.sleep();
    } else {
        Serial.print("LoRa Init Failed, code "); Serial.println(state);
    }
    
    startTime = millis();

    // Reserve memory for breadcrumbs to prevent fragmentation
    breadcrumbs.reserve(MAX_BREADCRUMBS);

    u8g2.begin();
    u8g2.setDrawColor(1);
    u8g2.setFontPosTop();
    
    // Boot Logo
    u8g2.clearBuffer();
    int cx = 64, cy = 50;
    u8g2.drawLine(cx-24, cy, cx, cy-24);
    u8g2.drawLine(cx, cy-24, cx+24, cy);
    u8g2.drawLine(cx-24, cy, cx+24, cy);
    u8g2.drawFrame(cx-18, cy, 36, 28);
    u8g2.drawBox(cx-6, cy+12, 12, 16);
    u8g2.drawBox(cx+12, cy-20, 6, 12);
    u8g2.setFont(u8g2_font_ncenB10_tr);
    const char* title = "Bring Em Home";
    int w = u8g2.getStrWidth(title);
    u8g2.setCursor((SCREEN_WIDTH - w) / 2, 105);
    u8g2.print(title);
    u8g2.sendBuffer();
    delay(2000);
}

void loop() {
    // 0. Process GPS OFTEN (Before heavy tasks)
    while (Serial1.available()) {
        gps.encode(Serial1.read());
    }

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
            calibSaved = true;
        }
        lastCalibCheck = millis();
    }

    // --- Charging Detection ---
    static unsigned long lastChargeCheck = 0;
    bool shouldCheck = (millis() < CHARGE_CHECK_DURATION) || isCharging;

    if (shouldCheck && (millis() - lastChargeCheck > CHARGE_CHECK_INTERVAL)) {
        float voltage = readBatteryVoltage();
        
        if (voltage > 4.3) { 
             if (!isCharging) {
                 isCharging = true;
                 chargeStartTime = millis();
                 
                 // Avoid division by zero if battery was dead
                 float currentPct = (lastValidBatteryPct < 1) ? 1.0 : (float)lastValidBatteryPct;
                 
                 float needed_mAh = (float)BATTERY_CAPACITY_MAH * (1.0 - (currentPct / 100.0));
                 estimatedChargeTimeHours = needed_mAh / (float)CHARGE_CURRENT_MA;
                 if (estimatedChargeTimeHours < 0) estimatedChargeTimeHours = 0;
             }
        } else {
             isCharging = false;
             lastValidBatteryPct = getBatteryPercent();
        }
        lastChargeCheck = millis();
    }

    // Update Status LED (50Hz)
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate > LED_UPDATE_INTERVAL) {
        if (isCharging) {
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

    // 1. Process GPS AGAIN (To keep buffer empty)
    while (Serial1.available()) {
        gps.encode(Serial1.read());
    }

    // 2. Process Compass
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    currentHeading = (int)orientationData.orientation.x;

    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    // 3. Button Logic
    bool currentButtonState = digitalRead(PIN_BUTTON);
    
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        buttonPressStartTime = millis();
        isLongPressHandled = false;
    }
    
    if (currentButtonState == LOW) {
        unsigned long pressDuration = millis() - buttonPressStartTime;
        
        if (!isLongPressHandled && pressDuration > LONG_PRESS_DURATION) {
             if (currentMode == MODE_EXPLORE || currentMode == MODE_RETURN) {
                 isLongPressHandled = true;
                 powerOff();
             }
        }

        if (!isLongPressHandled && pressDuration > VERY_LONG_PRESS_DURATION) {
            if (currentMode == MODE_EXPLORE && gps.location.isValid()) {
                homeLat = gps.location.lat();
                homeLon = gps.location.lng();
                hasHome = true;
                
                for(int i=0; i<5; i++) {
                    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); pixels.show();
                    digitalWrite(PIN_VIB_MOTOR, HIGH); delay(100);
                    pixels.clear(); pixels.show();
                    digitalWrite(PIN_VIB_MOTOR, LOW); delay(100);
                }
                showFeedback("HOME RESET!", "", FEEDBACK_DURATION_LONG);
                lastInteractionTime = millis();
            }
        }
    }
    
    if (currentButtonState == HIGH && lastButtonState == LOW) {
        if (!isLongPressHandled && (millis() - buttonPressStartTime < 2000)) {
            clickCount++;
            lastClickTime = millis();
        }
    }
    lastButtonState = currentButtonState;

    if (clickCount > 0 && (millis() - lastClickTime > CLICK_DELAY)) {
        if (isSOSCountdown) {
            isSOSCountdown = false;
            showFeedback("SOS CANCELLED", "", FEEDBACK_DURATION_LONG);
            clickCount = 0;
            lastInteractionTime = millis();
        } else if (clickCount == 1) {
            if (isDisplayOn) {
                u8g2.setPowerSave(DISPLAY_POWER_SAVE_ON);
                isDisplayOn = false;
            } else {
                u8g2.setPowerSave(DISPLAY_POWER_SAVE_OFF);
                isDisplayOn = true;
                lastInteractionTime = millis();
            }
        } else if (clickCount == 2) {
            if (currentMode == MODE_EXPLORE) {
                currentMode = MODE_RETURN;
                if (!breadcrumbs.empty() && gps.location.isValid()) {
                    double minDst = 99999999;
                    int minIdx = -1;
                    for(int i=0; i<breadcrumbs.size(); i++) {
                        double d = gps.distanceBetween(gps.location.lat(), gps.location.lng(), breadcrumbs[i].lat, breadcrumbs[i].lon);
                        if (d < minDst) { minDst = d; minIdx = i; }
                    }
                    targetBreadcrumbIndex = minIdx;
                } else {
                    targetBreadcrumbIndex = -1;
                }
                showFeedback("RETURN MODE", "", FEEDBACK_DURATION_SHORT);
            } else {
                currentMode = MODE_EXPLORE;
                showFeedback("EXPLORE MODE", "", FEEDBACK_DURATION_SHORT);
            }
            lastInteractionTime = millis();
        } else if (clickCount == 3) {
            toggleFlashlight();
            triggerVibration();
        } else if (clickCount >= 5) {
            isSOSCountdown = true;
            sosCountdownStartTime = millis();
            if (!isDisplayOn) { u8g2.setPowerSave(DISPLAY_POWER_SAVE_OFF); isDisplayOn = true; }
            triggerVibration();
        }
        clickCount = 0;
    }

    if (isSOSCountdown) {
        if (millis() - sosCountdownStartTime > SOS_COUNTDOWN_DURATION) {
            isSOSCountdown = false;
            toggleSOS();
            triggerVibration(); delay(100); triggerVibration(); delay(100); triggerVibration();
        }
    }

    // 4. Auto-Home Logic
    if (!hasHome && gps.location.isValid()) {
        homeLat = gps.location.lat();
        homeLon = gps.location.lng();
        hasHome = true;
        Serial.println("Auto-Home Set (First Fix)!");
        String latStr = "Lat: " + String(homeLat, 4);
        String lonStr = "Lon: " + String(homeLon, 4);
        showFeedback("HOME SET!", latStr + "\n" + lonStr, FEEDBACK_DURATION_LONG);
        lastInteractionTime = millis();
    }

    // 5. Breadcrumb Logic
    if (currentMode == MODE_EXPLORE && gps.location.isValid()) {
        bool shouldSave = false;
        double speed = gps.speed.kmph();
        if (speed > MIN_SPEED_KPH && speed < MAX_SPEED_KPH) {
            if (breadcrumbs.empty()) {
                shouldSave = true;
            } else {
                Breadcrumb last = breadcrumbs.back();
                double dist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), last.lat, last.lon);
                if (dist > BREADCRUMB_DIST) {
                    shouldSave = true;
                } else if (dist > BREADCRUMB_MIN_DIST_TURN) {
                    double currentSegmentBearing = gps.courseTo(last.lat, last.lon, gps.location.lat(), gps.location.lng());
                    if (breadcrumbs.size() >= 2) {
                        Breadcrumb prev = breadcrumbs[breadcrumbs.size()-2];
                        double prevSegmentBearing = gps.courseTo(prev.lat, prev.lon, last.lat, last.lon);
                        double diff = abs(currentSegmentBearing - prevSegmentBearing);
                        if (diff > 180) diff = 360 - diff;
                        if (diff > BREADCRUMB_TURN_THRESHOLD) shouldSave = true;
                    }
                }
            }
        }
        
        if (shouldSave) {
            if (breadcrumbs.size() >= MAX_BREADCRUMBS) breadcrumbs.erase(breadcrumbs.begin());
            Breadcrumb b; b.lat = gps.location.lat(); b.lon = gps.location.lng();
            breadcrumbs.push_back(b);
            Serial.printf("Breadcrumb saved: %f, %f (Total: %d)\n", b.lat, b.lon, breadcrumbs.size());
        }
    }

    // 6. Backtrack Logic
    if (currentMode == MODE_RETURN && gps.location.isValid() && !breadcrumbs.empty()) {
        if (targetBreadcrumbIndex >= 0) {
            double distToTarget = gps.distanceBetween(gps.location.lat(), gps.location.lng(), breadcrumbs[targetBreadcrumbIndex].lat, breadcrumbs[targetBreadcrumbIndex].lon);
            if (distToTarget < BREADCRUMB_REACH_DIST) {
                targetBreadcrumbIndex--;
                triggerVibration();
            }
        }
    }

    // 7. Vibration Logic
    if (isVibrating && (millis() - vibrationStartTime > VIB_DURATION_LONG)) {
        digitalWrite(PIN_VIB_MOTOR, LOW);
        isVibrating = false;
    }

    // 8. Display Timeout
    if (isDisplayOn && (millis() - lastInteractionTime > DISPLAY_TIMEOUT)) {
        u8g2.setPowerSave(DISPLAY_POWER_SAVE_ON);
        isDisplayOn = false;
    }

    // 9. Update Display
    if (isDisplayOn) {
        static unsigned long lastUpdate = 0;
        if (millis() - lastUpdate > DISPLAY_REFRESH_RATE) {
            lastUpdate = millis();
            u8g2.clearBuffer();
            int w = 0;

            if (isCharging) {
                u8g2.setFont(u8g2_font_ncenB10_tr);
                const char* title = "Loading battery...";
                w = u8g2.getStrWidth(title);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 30);
                u8g2.print(title);
                
                u8g2.drawFrame(44, 50, 40, 20);
                u8g2.drawBox(84, 56, 4, 8);
                
                static int chargeAnim = 0;
                static unsigned long lastChargeAnim = 0;
                if (millis() - lastChargeAnim > CHARGE_ANIM_INTERVAL) {
                    chargeAnim = (chargeAnim + 1) % 5;
                    lastChargeAnim = millis();
                }
                for(int i=0; i<chargeAnim; i++) u8g2.drawBox(46 + (i*9), 52, 7, 16);

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

            if (millis() < feedbackEndTime) {
                u8g2.setFont(u8g2_font_ncenB10_tr);
                w = u8g2.getStrWidth(feedbackTitle.c_str());
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 50);
                u8g2.print(feedbackTitle);
                if (feedbackSub.length() > 0) {
                    u8g2.setFont(u8g2_font_6x10_tr);
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

            if (isSOSCountdown) {
                u8g2.setFont(u8g2_font_ncenB12_tr);
                const char* title = "SOS MODE IN";
                w = u8g2.getStrWidth(title);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 20);
                u8g2.print(title);
                int remaining = 5 - (millis() - sosCountdownStartTime) / 1000;
                if (remaining < 0) remaining = 0;
                u8g2.setFont(u8g2_font_logisoso42_tn);
                String cntStr = String(remaining);
                w = u8g2.getStrWidth(cntStr.c_str());
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 45);
                u8g2.print(cntStr);
                u8g2.setFont(u8g2_font_6x10_tr);
                const char* sub = "Press button to Cancel";
                w = u8g2.getStrWidth(sub);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 110);
                u8g2.print(sub);
                u8g2.sendBuffer();
                return;
            }

            if (isSOSActive) {
                u8g2.setFont(u8g2_font_ncenB14_tr);
                const char* title = "SOS ACTIVE";
                w = u8g2.getStrWidth(title);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 20);
                u8g2.print(title);
                int secToNext = (LORA_TX_INTERVAL - (millis() - lastLoRaTx)) / 1000;
                if (secToNext < 0) secToNext = 0;
                u8g2.setFont(u8g2_font_logisoso42_tn);
                String cntStr = String(secToNext);
                w = u8g2.getStrWidth(cntStr.c_str());
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 45);
                u8g2.print(cntStr);
                u8g2.setFont(u8g2_font_6x10_tr);
                const char* sub = "Sending Location...";
                w = u8g2.getStrWidth(sub);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 110);
                u8g2.print(sub);
                u8g2.sendBuffer();
                return;
            }
            
            u8g2.drawHLine(0, STATUS_BAR_LINE_Y, SCREEN_WIDTH);
            u8g2.setFont(u8g2_font_5x7_tr);

            u8g2.drawFrame(0, 4, 6, 10);
            u8g2.drawBox(2, 2, 2, 2);
            int pct = getBatteryPercent();
            int h = (pct * BATTERY_BAR_HEIGHT) / 100;
            if (h > 0) u8g2.drawBox(1, 4 + (10-1-h), 4, h);
            u8g2.setCursor(8, 12);
            u8g2.print(pct); u8g2.print("%");

            String compStr = "Bad";
            if (mag == 3) compStr = "Good"; else if (mag == 2) compStr = "Ok"; else if (mag == 1) compStr = "Low";
            String compDisp = "C:" + compStr;
            w = u8g2.getStrWidth(compDisp.c_str());
            u8g2.setCursor((128 - w) / 2, 10);
            u8g2.print(compDisp);

            int sats = gps.satellites.value();
            int bars = 0;
            if (gps.location.isValid()) {
                if (sats >= 6) bars = 4; else if (sats >= 5) bars = 3; else if (sats >= 4) bars = 2; else bars = 1;
            }
            int startX = 128 - (4 * 4); 
            for (int i=0; i<4; i++) {
                int h = (i+1) * 2 + 2;
                if (i < bars) u8g2.drawBox(startX + (i*4), 14-h, 3, h); else u8g2.drawFrame(startX + (i*4), 14-h, 3, h);
            }

            if (!gps.location.isValid()) {
                u8g2.setFont(u8g2_font_ncenB10_tr);
                const char* baseTxt = "Searching SATs";
                int w = u8g2.getStrWidth(baseTxt);
                int startX = (SCREEN_WIDTH - w) / 2;
                int y = 60;
                u8g2.setCursor(startX, y);
                u8g2.print(baseTxt);
                int barWidth = 64; int barHeight = 12;
                int barX = (SCREEN_WIDTH - barWidth) / 2; int barY = y + 15;
                u8g2.drawFrame(barX, barY, barWidth, barHeight);
                static int animPos = 0; static unsigned long lastAnim = 0;
                if (millis() - lastAnim > GPS_SEARCH_ANIM_INTERVAL) { animPos = (animPos + 2) % (barWidth - 3); lastAnim = millis(); }
                if (animPos > 0) u8g2.drawBox(barX + 2, barY + 2, animPos, barHeight - 4);
                u8g2.sendBuffer();
                return;
            }

            double targetLat = homeLat;
            double targetLon = homeLon;
            bool targetIsHome = true;
            double distToTarget = 0;
            
            if (currentMode == MODE_RETURN && targetBreadcrumbIndex >= 0 && !breadcrumbs.empty()) {
                targetLat = breadcrumbs[targetBreadcrumbIndex].lat;
                targetLon = breadcrumbs[targetBreadcrumbIndex].lon;
                targetIsHome = false;
                if (gps.location.isValid()) distToTarget = gps.distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLon);
            }

            double dist = 0;
            double bearing = 0;
            bool showArrow = false;

            if (currentMode == MODE_EXPLORE) {
                 if (gps.location.isValid() && hasHome) dist = calculateTotalDistanceToHome();
                 bearing = 0;
                 showArrow = true;
            } else {
                if (gps.location.isValid()) {
                    dist = calculateTotalDistanceToHome();
                    bearing = gps.courseTo(gps.location.lat(), gps.location.lng(), targetLat, targetLon);
                    showArrow = true;
                }
            }

            if (showArrow) {
                int relBearing = (int)bearing - currentHeading;
                if (relBearing < 0) relBearing += 360;
                int arrowCy = 64;
                bool showCardinals = (currentMode == MODE_EXPLORE);
                drawArrow(SCREEN_WIDTH/2, arrowCy, 30, relBearing, showCardinals);
            } else {
                if (currentMode == MODE_RETURN && !gps.location.isValid()) {
                    u8g2.setFont(u8g2_font_ncenB10_tr);
                    u8g2.drawStr(35, 60, "NO GPS");
                }
            }

            if (gps.location.isValid() && (hasHome || currentMode == MODE_RETURN)) {
                if (!targetIsHome) {
                    u8g2.setFont(u8g2_font_6x10_tr);
                    u8g2.setCursor(0, 110);
                    u8g2.print("NEXT:");
                    String nextStr;
                    if (distToTarget < 1000) nextStr = String((int)distToTarget) + "m"; else nextStr = String(distToTarget / 1000.0, 2) + "km";
                    int w = u8g2.getStrWidth(nextStr.c_str());
                    u8g2.setCursor(SCREEN_WIDTH - w, 110);
                    u8g2.print(nextStr);
                    u8g2.setFont(u8g2_font_ncenB10_tr);
                    u8g2.setCursor(0, 125);
                    u8g2.print("HOME");
                    String distStr = String(dist / 1000.0, 2) + " km";
                    u8g2.setFont(u8g2_font_ncenB12_tr);
                    w = u8g2.getStrWidth(distStr.c_str());
                    u8g2.setCursor(SCREEN_WIDTH - w, 125);
                    u8g2.print(distStr);
                } else {
                    String label = "HOME";
                    u8g2.setFont(u8g2_font_ncenB10_tr);
                    u8g2.setCursor(0, 125);
                    u8g2.print(label);
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