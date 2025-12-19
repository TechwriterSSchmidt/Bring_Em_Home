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

// Helper: Draw Rotated Arrow (Fancy Needle)
void drawArrow(int cx, int cy, int r, float angleDeg, uint16_t color) {
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
    gfx->fillTriangle(x1, y1, x3, y3, x2, y2, color);
    gfx->fillTriangle(x1, y1, x4, y4, x2, y2, color);
    
    // Center Dot
    gfx->fillCircle(cx, cy, 3, C_GOLD);
}

// Initialization sequence from Waveshare Demo
void lcd_reg_init(void) {

  static const uint8_t init_operations[] = {
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x11,  // 2: Out of sleep mode, no args, w/delay
    END_WRITE,
    DELAY, 120,

    BEGIN_WRITE,
    WRITE_C8_D16, 0xDF, 0x98, 0x53,
    WRITE_C8_D8, 0xB2, 0x23, 

    WRITE_COMMAND_8, 0xB7,
    WRITE_BYTES, 4,
    0x00, 0x47, 0x00, 0x6F,

    WRITE_COMMAND_8, 0xBB,
    WRITE_BYTES, 6,
    0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0,

    WRITE_C8_D16, 0xC0, 0x44, 0xA4,
    WRITE_C8_D8, 0xC1, 0x16, 

    WRITE_COMMAND_8, 0xC3,
    WRITE_BYTES, 8,
    0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77,

    WRITE_COMMAND_8, 0xC4,
    WRITE_BYTES, 12,
    0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82,

    WRITE_COMMAND_8, 0xC8,
    WRITE_BYTES, 32,
    0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00, 0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,

    WRITE_COMMAND_8, 0xD0,
    WRITE_BYTES, 5,
    0x04, 0x06, 0x6B, 0x0F, 0x00,

    WRITE_C8_D16, 0xD7, 0x00, 0x30,
    WRITE_C8_D8, 0xE6, 0x14, 
    WRITE_C8_D8, 0xDE, 0x01, 

    WRITE_COMMAND_8, 0xB7,
    WRITE_BYTES, 5,
    0x03, 0x13, 0xEF, 0x35, 0x35,

    WRITE_COMMAND_8, 0xC1,
    WRITE_BYTES, 3,
    0x14, 0x15, 0xC0,

    WRITE_C8_D16, 0xC2, 0x06, 0x3A,
    WRITE_C8_D16, 0xC4, 0x72, 0x12,
    WRITE_C8_D8, 0xBE, 0x00, 
    WRITE_C8_D8, 0xDE, 0x02, 

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3,
    0x00, 0x02, 0x00,

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3,
    0x01, 0x02, 0x00,

    WRITE_C8_D8, 0xDE, 0x00, 
    WRITE_C8_D8, 0x35, 0x00, 
    WRITE_C8_D8, 0x3A, 0x05, 

    WRITE_COMMAND_8, 0x2A,
    WRITE_BYTES, 4,
    0x00, 0x22, 0x00, 0xCD,

    WRITE_COMMAND_8, 0x2B,
    WRITE_BYTES, 4,
    0x00, 0x00, 0x01, 0x3F,

    WRITE_C8_D8, 0xDE, 0x02, 

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3,
    0x00, 0x02, 0x00,
    
    WRITE_C8_D8, 0xDE, 0x00, 
    WRITE_C8_D8, 0x36, 0x00,
    WRITE_COMMAND_8, 0x21,
    END_WRITE,
    
    DELAY, 10,

    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x29,  // 5: Main screen turn on, no args, w/delay
    END_WRITE
  };
  bus->batchOperation(init_operations, sizeof(init_operations));
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
    
    // Create Canvas for flicker-free drawing
    canvas = new Arduino_Canvas(SCREEN_WIDTH, SCREEN_HEIGHT, gfx);

    gfx->fillScreen(BLACK);
    
    // Elegant Splash Screen
    gfx->drawFastHLine(20, 50, 200, C_SILVER);
    
    gfx->setCursor(30, 70);
    gfx->setTextColor(C_EMERALD);
    gfx->setTextSize(2);
    gfx->println("Bring Em Home");
    
    gfx->drawFastHLine(20, 100, 200, C_SILVER);
    
    gfx->setCursor(40, 130);
    gfx->setTextColor(C_GOLD);
    gfx->setTextSize(1);
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
            
            canvas->fillScreen(BLACK);
            
            // --- Header ---
            // Top Line
            canvas->drawFastHLine(20, 35, 200, C_SILVER);
            
            // Mode Title (Centered)
            canvas->setTextSize(2);
            String title = (currentMode == MODE_RECORDING) ? "EXPLORE" : "RETURN";
            uint16_t titleColor = (currentMode == MODE_RECORDING) ? C_EMERALD : C_RUBY;
            
            int16_t x1, y1;
            uint16_t w, h;
            canvas->getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
            canvas->setCursor((SCREEN_WIDTH - w) / 2, 10);
            canvas->setTextColor(titleColor);
            canvas->print(title);

            // Satellites (Small, Top Right)
            canvas->setTextSize(1);
            canvas->setTextColor(C_SILVER);
            canvas->setCursor(190, 5);
            if (gps.location.isValid()) {
                canvas->printf("%d", gps.satellites.value());
            } else {
                canvas->print("X");
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
                
                // Draw fancy needle on Canvas
                // We need to adapt drawArrow to use canvas or pass it
                // For now, let's inline the drawing or update drawArrow to take GFX*
                // But drawArrow uses global 'gfx'. We should update it or duplicate logic.
                // Let's duplicate logic here for canvas to avoid signature change issues
                
                int cx = SCREEN_WIDTH/2;
                int cy = 110;
                int r = 55;
                float angleRad = (relBearing - 90) * PI / 180.0;
                
                int x1 = cx + r * cos(angleRad);
                int y1 = cy + r * sin(angleRad);
                int x2 = cx - (r * 0.3) * cos(angleRad);
                int y2 = cy - (r * 0.3) * sin(angleRad);
                int x3 = cx + (r * 0.35) * cos(angleRad + PI/2);
                int y3 = cy + (r * 0.35) * sin(angleRad + PI/2);
                int x4 = cx + (r * 0.35) * cos(angleRad - PI/2);
                int y4 = cy + (r * 0.35) * sin(angleRad - PI/2);
                
                canvas->fillTriangle(x1, y1, x3, y3, x2, y2, arrowColor);
                canvas->fillTriangle(x1, y1, x4, y4, x2, y2, arrowColor);
                canvas->fillCircle(cx, cy, 3, C_GOLD);
                
                // N indicator for Recording mode
                if (currentMode == MODE_RECORDING) {
                    int nx = (SCREEN_WIDTH/2) + 70 * cos(angleRad);
                    int ny = 110 + 70 * sin(angleRad);
                    canvas->setCursor(nx-6, ny-6);
                    canvas->setTextColor(C_EMERALD);
                    canvas->setTextSize(2);
                    canvas->print("N");
                }
            } else {
                // No GPS (Backtracking) or No Home (Recording - but arrow is always shown now)
                if (currentMode == MODE_BACKTRACKING && !gps.location.isValid()) {
                    canvas->setCursor(60, 100);
                    canvas->setTextColor(C_RUBY);
                    canvas->setTextSize(2);
                    canvas->print("NO GPS");
                }
            }

            // Distance Text (Below Arrow)
            if (gps.location.isValid() && (hasHome || currentMode == MODE_BACKTRACKING)) {
                String distStr;
                if (dist < 1000) distStr = String(dist, 0) + " m";
                else distStr = String(dist / 1000.0, 2) + " km";
                
                canvas->setTextSize(3);
                canvas->getTextBounds(distStr, 0, 0, &x1, &y1, &w, &h);
                canvas->setCursor((SCREEN_WIDTH - w) / 2, 180);
                canvas->setTextColor(WHITE);
                canvas->print(distStr);
                
                // Label
                String label = targetIsHome ? "HOME" : "WAYPOINT";
                canvas->setTextSize(1);
                canvas->getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
                canvas->setCursor((SCREEN_WIDTH - w) / 2, 215);
                canvas->setTextColor(C_GOLD);
                canvas->print(label);
            } else if (currentMode == MODE_RECORDING && !hasHome) {
                 // Show "SET HOME" if we have GPS but no home, or just hint
                 if (gps.location.isValid()) {
                    canvas->setCursor(50, 180);
                    canvas->setTextColor(C_GOLD);
                    canvas->setTextSize(2);
                    canvas->print("SET HOME");
                 } else {
                    canvas->setCursor(50, 180);
                    canvas->setTextColor(C_SILVER);
                    canvas->setTextSize(1);
                    canvas->print("WAITING GPS");
                 }
            }

            // Flush Canvas to Display
            canvas->flush();
        }
    }
    
    delay(10);
}