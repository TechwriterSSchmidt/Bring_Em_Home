// ESP32-S3-LCD-1.3 - Bring Em Home Project
// Using Arduino_GFX library with ST7789 driver
// Hardware: Waveshare ESP32-S3-LCD-1.3 (240x240)
// Pinout based on nishad2m8/WS-1.3

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <MechaQMC5883.h>
#include <Preferences.h>
#include <vector>
#include <WiFi.h> // For power management

// RGB565 Color definitions
#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define YELLOW  0xFFE0
#define CYAN    0x07FF
#define MAGENTA 0xF81F

// Elegant Palette
#define C_GOLD       0xFEA0 
#define C_EMERALD    0x05E6
#define C_RUBY       0xC804
#define C_SILVER     0xC618
#define C_DARK       0x10A2

// ESP32-S3-LCD-1.3 Pin Configuration
// Based on nishad2m8/WS-1.3 User_Setup.h
#define TFT_MOSI  41
#define TFT_SCK   40
#define TFT_CS    39
#define TFT_DC    38
#define TFT_RST   42
#define TFT_BL    45 
#define BUTTON_PIN 14 // External waterproof button
#define VIB_PIN 13    // Vibration Motor

// Sensor Pins
#define GPS_RX 17
#define GPS_TX 18
#define I2C_SDA 8
#define I2C_SCL 9

// Display dimensions
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 240

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

// GPS Power Management (UBX Commands for u-blox M10/M8)
void setGPSPower(bool on) {
    if (on) {
        // Wake up (send dummy bytes)
        Serial1.write(0xFF);
        delay(100);
    } else {
        // Send UBX-RXM-PMREQ (Backup Mode)
        // Header: 0xB5 0x62, Class: 0x02, ID: 0x41, Len: 0x08
        // Payload: Duration(0=Inf), Flags(2=Backup)
        uint8_t packet[] = {
            0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 
            0x00, 0x00, 0x00, 0x00, 
            0x02, 0x00, 0x00, 0x00,
            0x4D, 0x3B // Checksum (Calculated)
        };
        Serial1.write(packet, sizeof(packet));
    }
}

void triggerVibration() {
    digitalWrite(VIB_PIN, HIGH);
    vibrationStartTime = millis();
    isVibrating = true;
}

// Objects
TinyGPSPlus gps;
MechaQMC5883 qmc;
Preferences preferences;

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
const int CLICK_DELAY = 400; // ms to wait for double click

// Create display bus and display object
Arduino_DataBus *bus = new Arduino_ESP32SPI(
    TFT_DC,   // DC
    TFT_CS,   // CS
    TFT_SCK,  // SCK
    TFT_MOSI, // MOSI
    -1        // MISO (not used)
);

// ST7789 240x240
Arduino_GFX *gfx = new Arduino_ST7789(
    bus,
    TFT_RST,      // RST
    0,            // rotation (0 = portrait)
    true,         // IPS panel
    SCREEN_WIDTH, // width = 240
    SCREEN_HEIGHT,// height = 240
    0,            // col_offset1
    0,            // row_offset1
    0,            // col_offset2
    0             // row_offset2
);

Arduino_Canvas *canvas = nullptr;
Arduino_GFX *drawTarget = nullptr;

// Helper: Draw Rotated Arrow (Fancy Needle)
void drawArrow(Arduino_GFX *dst, int cx, int cy, int r, float angleDeg, uint16_t color) {
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
    dst->fillTriangle(x1, y1, x3, y3, x2, y2, color);
    dst->fillTriangle(x1, y1, x4, y4, x2, y2, color);
    
    // Center Dot
    dst->fillCircle(cx, cy, 3, C_GOLD);
}

void setup() {
    Serial.begin(115200);
    
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
    
    // Backlight Control
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH); // ON
    
    // Button Setup
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    lastInteractionTime = millis();

    // Vibration Setup
    pinMode(VIB_PIN, OUTPUT);
    digitalWrite(VIB_PIN, LOW);

    // Initialize Sensors
    Wire.begin(I2C_SDA, I2C_SCL);
    qmc.init();
    Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    setGPSPower(true); // Ensure GPS is awake
    
    // Initialize Preferences
    preferences.begin("bringemhome", false);
    homeLat = preferences.getDouble("lat", 0.0);
    homeLon = preferences.getDouble("lon", 0.0);
    hasHome = (homeLat != 0.0 && homeLon != 0.0);
    Serial.printf("Home: %f, %f\n", homeLat, homeLon);
    
    startTime = millis(); // Start timer for auto-home

    // Initialize display
    if (!gfx->begin()) {
        Serial.println("ERROR: gfx->begin() failed!");
        while(1) { delay(1000); }
    }
    
    // Memory Debugging
    Serial.printf("Total Heap: %d, Free Heap: %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
    Serial.printf("Total PSRAM: %d, Free PSRAM: %d\n", ESP.getPsramSize(), ESP.getFreePsram());

    // Canvas removed for stability (caused crash due to low RAM)
    // We will draw directly to gfx
    drawTarget = gfx;

    gfx->fillScreen(BLACK);
    
    // Elegant Splash Screen
    gfx->drawFastHLine(20, 80, 200, C_SILVER);
    
    gfx->setCursor(10, 100); // Adjusted X for larger text
    gfx->setTextColor(C_EMERALD);
    gfx->setTextSize(3); // Larger Title
    gfx->println("Bring Em Home");
    
    gfx->drawFastHLine(20, 140, 200, C_SILVER);
    
    gfx->setCursor(25, 180); // Adjusted X
    gfx->setTextColor(C_GOLD);
    gfx->setTextSize(2); // Larger Subtitle
    gfx->println("Waiting for GPS...");
}

void loop() {
    // 1. Process GPS
    while (Serial1.available()) {
        gps.encode(Serial1.read());
    }

    // 2. Process Compass
    uint16_t x, y, z;
    float heading;
    qmc.read(&x, &y, &z, &heading);
    currentHeading = (int)heading;

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
        
        // 2s: Save Home (Only in Recording Mode, One shot)
        if (!isLongPressHandled && pressDuration > 2000 && pressDuration < 5000) {
            if (currentMode == MODE_RECORDING) {
                Serial.println("Long Press: Saving Home");
                if (gps.location.isValid()) {
                    homeLat = gps.location.lat();
                    homeLon = gps.location.lng();
                    hasHome = true;
                    preferences.putDouble("lat", homeLat);
                    preferences.putDouble("lon", homeLon);
                    
                    // Visual Feedback
                    if (!isDisplayOn) { 
                        digitalWrite(TFT_BL, HIGH); 
                        isDisplayOn = true; 
                        setCpuFrequencyMhz(240);
                    }
                    gfx->fillScreen(C_EMERALD);
                    gfx->setCursor(40, 110);
                    gfx->setTextColor(BLACK);
                    gfx->setTextSize(2);
                    gfx->println("HOME SAVED!");
                    delay(1000);
                    gfx->fillScreen(BLACK);
                } else {
                    // Error Feedback
                    if (!isDisplayOn) { 
                        digitalWrite(TFT_BL, HIGH); 
                        isDisplayOn = true; 
                        setCpuFrequencyMhz(240);
                    }
                    gfx->fillScreen(C_RUBY);
                    gfx->setCursor(40, 110);
                    gfx->setTextColor(WHITE);
                    gfx->setTextSize(2);
                    gfx->println("NO GPS FIX!");
                    delay(1000);
                    gfx->fillScreen(BLACK);
                }
            }
            isLongPressHandled = true; // Prevent repeat
            lastInteractionTime = millis();
        }
        
        // 5s: Power Off (Deep Sleep)
        if (pressDuration > 5000) {
            // Feedback
            if (!isDisplayOn) { digitalWrite(TFT_BL, HIGH); }
            setCpuFrequencyMhz(240);
            gfx->fillScreen(BLACK);
            gfx->setCursor(40, 100);
            gfx->setTextColor(C_SILVER);
            gfx->setTextSize(2);
            gfx->println("GOODBYE");
            delay(1000);
            digitalWrite(TFT_BL, LOW);
            
            // Turn off GPS
            setGPSPower(false);
            
            // Configure Wakeup
            esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0); // Wake on LOW
            
            // Go to Sleep
            Serial.println("Entering Deep Sleep...");
            esp_deep_sleep_start();
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
                digitalWrite(TFT_BL, LOW);
                isDisplayOn = false;
                setCpuFrequencyMhz(80); // Low power mode
            } else {
                digitalWrite(TFT_BL, HIGH);
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
                    digitalWrite(TFT_BL, HIGH); 
                    isDisplayOn = true; 
                    setCpuFrequencyMhz(240);
                }
                gfx->fillScreen(C_RUBY);
                gfx->setCursor(40, 110);
                gfx->setTextColor(WHITE);
                gfx->setTextSize(2);
                gfx->println("RETURN MODE");
                delay(1000);
                gfx->fillScreen(BLACK);
            } else {
                currentMode = MODE_RECORDING;
                // Feedback
                if (!isDisplayOn) { 
                    digitalWrite(TFT_BL, HIGH); 
                    isDisplayOn = true; 
                    setCpuFrequencyMhz(240);
                }
                gfx->fillScreen(C_EMERALD);
                gfx->setCursor(40, 110);
                gfx->setTextColor(BLACK);
                gfx->setTextSize(2);
                gfx->println("EXPLORE MODE");
                delay(1000);
                gfx->fillScreen(BLACK);
            }
            lastInteractionTime = millis();
        }
        clickCount = 0;
    }

    // 4. Auto-Home Logic (5 minutes)
    if (!hasHome && gps.location.isValid()) {
        if (millis() - startTime > 300000) { // 5 mins
            homeLat = gps.location.lat();
            homeLon = gps.location.lng();
            hasHome = true;
            preferences.putDouble("lat", homeLat);
            preferences.putDouble("lon", homeLon);
            Serial.println("Auto-Home Set!");
        }
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
        digitalWrite(TFT_BL, LOW);
        isDisplayOn = false;
        setCpuFrequencyMhz(80); // Low power mode
    }

    // 9. Update Display
    if (isDisplayOn) {
        static unsigned long lastUpdate = 0;
        if (millis() - lastUpdate > 100) { // 10Hz update for smooth compass
            lastUpdate = millis();
            
            // Use drawTarget (either canvas or gfx)
            drawTarget->fillScreen(BLACK);
            
            // --- Header ---
            // Top Line
            drawTarget->drawFastHLine(20, 40, 200, C_SILVER); // Moved down slightly
            
            // Mode Title (Centered)
            drawTarget->setTextSize(3); // Larger Title
            String title = (currentMode == MODE_RECORDING) ? "EXPLORE" : "RETURN";
            uint16_t titleColor = (currentMode == MODE_RECORDING) ? C_EMERALD : C_RUBY;
            
            int16_t x1, y1;
            uint16_t w, h;
            drawTarget->getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
            drawTarget->setCursor((SCREEN_WIDTH - w) / 2, 10);
            drawTarget->setTextColor(titleColor);
            drawTarget->print(title);

            // Satellites (Small, Top Right)
            drawTarget->setTextSize(2); // Larger Sat count
            drawTarget->setTextColor(C_SILVER);
            drawTarget->setCursor(200, 5);
            if (gps.location.isValid()) {
                drawTarget->printf("%d", gps.satellites.value());
            } else {
                drawTarget->print("X");
            }

            // --- Main Content ---
            
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
            uint16_t arrowColor = C_SILVER;

            if (currentMode == MODE_RECORDING) {
                 // Compass Mode - Always Active
                 if (gps.location.isValid() && hasHome) {
                    dist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon);
                 }
                 bearing = 0; // North
                 showArrow = true;
                 arrowColor = C_EMERALD;
            } else {
                // Backtrack Mode - Needs GPS
                if (gps.location.isValid()) {
                    dist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLon);
                    bearing = gps.courseTo(gps.location.lat(), gps.location.lng(), targetLat, targetLon);
                    showArrow = true;
                    arrowColor = C_RUBY;
                }
            }

            // Draw Arrow (Center)
            if (showArrow) {
                int relBearing = (int)bearing - currentHeading;
                if (relBearing < 0) relBearing += 360;
                
                // Draw fancy needle
                int arrowCy = 115; // Centered vertically in upper section
                drawArrow(drawTarget, SCREEN_WIDTH/2, arrowCy, 60, relBearing, arrowColor);
                
                // N indicator for Recording mode
                if (currentMode == MODE_RECORDING) {
                    float angleRad = (relBearing - 90) * PI / 180.0;
                    int nx = (SCREEN_WIDTH/2) + 75 * cos(angleRad);
                    int ny = arrowCy + 75 * sin(angleRad);
                    drawTarget->setCursor(nx-9, ny-9); // Adjusted for larger font
                    drawTarget->setTextColor(C_EMERALD);
                    drawTarget->setTextSize(3); // Larger N
                    drawTarget->print("N");
                }
            } else {
                // No GPS (Backtracking) or No Home (Recording - but arrow is always shown now)
                if (currentMode == MODE_BACKTRACKING && !gps.location.isValid()) {
                    drawTarget->setCursor(50, 140); // Adjusted X
                    drawTarget->setTextColor(C_RUBY);
                    drawTarget->setTextSize(3); // Larger Warning
                    drawTarget->print("NO GPS");
                }
            }

            // Distance Text (Below Arrow)
            if (gps.location.isValid() && (hasHome || currentMode == MODE_BACKTRACKING)) {
                String distStr;
                if (dist < 1000) distStr = String(dist, 0) + " m";
                else distStr = String(dist / 1000.0, 2) + " km";
                
                drawTarget->setTextSize(4); // Huge Distance
                drawTarget->getTextBounds(distStr, 0, 0, &x1, &y1, &w, &h);
                drawTarget->setCursor((SCREEN_WIDTH - w) / 2, 185); // Moved up slightly
                drawTarget->setTextColor(WHITE);
                drawTarget->print(distStr);
                
                // Label
                String label = targetIsHome ? "HOME" : "WAYPOINT";
                drawTarget->setTextSize(2); // Larger Label
                drawTarget->getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
                drawTarget->setCursor((SCREEN_WIDTH - w) / 2, 220);
                drawTarget->setTextColor(C_GOLD);
                drawTarget->print(label);
            } else if (currentMode == MODE_RECORDING && !hasHome) {
                 // Show "SET HOME" if we have GPS but no home, or just hint
                 if (gps.location.isValid()) {
                    drawTarget->setCursor(40, 190);
                    drawTarget->setTextColor(C_GOLD);
                    drawTarget->setTextSize(3); // Larger
                    drawTarget->print("SET HOME");
                 } else {
                    drawTarget->setCursor(40, 190);
                    drawTarget->setTextColor(C_SILVER);
                    drawTarget->setTextSize(2); // Larger
                    drawTarget->print("WAITING GPS");
                 }
            }

            // Flush Canvas to Display (only if using canvas)
            if (canvas != nullptr) {
                canvas->flush();
            }
        }
    }
    
    delay(10);
}