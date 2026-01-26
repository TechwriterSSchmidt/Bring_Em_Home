// Heltec Mesh Node T114 (nRF52840) - Bring Em Home Project
// Using U8g2 library for SH1107 128x128 OLED (I2C)
// Hardware: nRF52840 + SH1107 OLED + BNO055 + L76K GNSS

#include <Arduino.h>
#include "config.h" // Config must be included first to define USE_BNO085
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <nrf_wdt.h> // Watchdog Timer
#include <cmath> // For copysign, abs

using namespace Adafruit_LittleFS_Namespace;

#if USE_BNO085
#include <Adafruit_BNO08x.h>
#else
#include <Adafruit_BNO055.h>
#endif
#include <nrf_gpio.h>
#include <nrf_power.h>
#include <vector>
#include <Adafruit_NeoPixel.h>

// --- Data Structures ---
struct Breadcrumb {
    double lat;
    double lon;
};

std::vector<Breadcrumb> breadcrumbs;

enum AppMode {
    MODE_EXPLORE,
    MODE_RETURN,
    MODE_CONFIRM_HOME,
    MODE_SET_ID  // New Mode for ID Selection
};

// --- Globals for State ---
AppMode currentMode = MODE_CONFIRM_HOME; 
int targetBreadcrumbIndex = -1;

// Define second I2C bus for External Compass
TwoWire Wire1(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, PIN_EXT_SDA, PIN_EXT_SCL);

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
bool hasCompass = false; // Flag to indicate if compass is present
uint8_t compassAccuracy = 0; // 0=Unreliable, 3=High Accuracy

// Objects
TinyGPSPlus gps;

#if USE_BNO085
// Pins: -1 = Reset Pin not used (or not connected to MCU)
Adafruit_BNO08x bno08x(-1); 
sh2_SensorValue_t sensorValue;
#else
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_ID, BNO055_ADDRESS, &Wire1);
#endif

// Dummy NeoPixel for now as it's disabled in config
#if defined(PIN_NEOPIXEL) && (PIN_NEOPIXEL >= 0)
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#else
Adafruit_NeoPixel pixels(1, -1, NEO_GRB + NEO_KHZ800);
#endif

// U8g2 Display Object (SH1107 128x128 I2C)
U8G2_SH1107_128X128_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// App Modes
// Moved to Top

// --- MENU DEFINITONS ---
enum MenuState {
  MENU_NONE,
  MENU_MODE_SWITCH, // Explore <-> Return
  MENU_DIST_SWITCH, // 25m / 50m / 75m
  MENU_POWER_OFF    // Ausschalten
};

MenuState currentMenuSelection = MENU_NONE;
unsigned long lastMenuInteraction = 0;
const unsigned long MENU_TIMEOUT = 4000; // Menü schließt automatisch nach 4s
const unsigned long LONG_PRESS_THRESHOLD = 800; // ms für Bestätigung
const unsigned long SOS_CONFIRM_TIME = 10000; // 10 Sekunden halten für SOS aus Menü
// AppMode currentMode = MODE_CONFIRM_HOME; // Moved to top

// Config Data
int deviceID = -1; // -1 = Not Set

// Navigation Data
float breadcrumbDistance = DEFAULT_BREADCRUMB_DIST;
double homeLat = 0.0;
double homeLon = 0.0;
double savedHomeLat = 0.0;
double savedHomeLon = 0.0;
bool hasSavedHome = false;
bool hasHome = false;
double currentHeading = 0.0;
double currentPitch = 0.0; // Added Pitch for Smart Wake
double displayedHeading = 0.0;
unsigned long startTime = 0;

// Breadcrumbs Struct Moved to Top

// Button Logic (Multi-click)
int clickCount = 0;
unsigned long lastClickTime = 0;

// Compass Calibration State
bool isCalibrated = false;
bool calibSaved = false;

// --- FileSystem Helpers ---
// EXPERIMENTAL: Atomic Write Strategy to prevent corruption
void saveFileSafe(const char* filename, String content) {
    InternalFS.begin();
    String tmpName = String(filename) + ".tmp";
    
    // 1. Write to temp file
    if (InternalFS.exists(tmpName.c_str())) InternalFS.remove(tmpName.c_str());
    
    File file = InternalFS.open(tmpName.c_str(), FILE_O_WRITE);
    if (file) {
        file.print(content);
        file.flush();
        file.close();
        
        // 2. Remove old file
        if (InternalFS.exists(filename)) InternalFS.remove(filename);
        
        // 3. Rename temp to actual
        InternalFS.rename(tmpName.c_str(), filename);
        // Serial.printf("Atomic Write Success: %s\n", filename);
    } else {
        Serial.printf("Atomic Write Failed: %s\n", filename);
    }
}

void saveHomePosition(double lat, double lon) {
    String data = String(lat, 8) + "," + String(lon, 8);
    saveFileSafe("/home.txt", data);
    Serial.println("Home Saved to Flash (Safe Mode)!");
}

bool loadHomePosition(double &lat, double &lon) {
    InternalFS.begin();
    File file = InternalFS.open("/home.txt", FILE_O_READ);
    if (file) {
        String content = file.readString();
        int comma = content.indexOf(',');
        if (comma > 0) {
            lat = atof(content.substring(0, comma).c_str());
            lon = atof(content.substring(comma + 1).c_str());
            file.close();
            Serial.println("Home Loaded from Flash: " + String(lat, 6) + ", " + String(lon, 6));
            return true;
        }
        file.close();
    }
    return false;
}

void saveState(int mode, int targetIdx) {
    // Format: MODE,TARGET_INDEX,DIST
    String data = String(mode) + "," + String(targetIdx) + "," + String((int)breadcrumbDistance);
    saveFileSafe("/state.txt", data);
}

void loadState(AppMode &mode, int &targetIdx) {
    InternalFS.begin();
    if (InternalFS.exists("/state.txt")) {
        File file = InternalFS.open("/state.txt", FILE_O_READ);
        if (file) {
            String content = file.readString();
            int c1 = content.indexOf(',');
            if (c1 > 0) {
                mode = (AppMode)content.substring(0, c1).toInt();
                int c2 = content.indexOf(',', c1 + 1);
                if (c2 > 0) {
                     targetIdx = content.substring(c1 + 1, c2).toInt();
                     breadcrumbDistance = content.substring(c2 + 1).toFloat();
                } else {
                     targetIdx = content.substring(c1 + 1).toInt();
                     breadcrumbDistance = DEFAULT_BREADCRUMB_DIST;
                }
                
                // Sanity Check
                if (breadcrumbDistance != 25.0 && breadcrumbDistance != 50.0 && breadcrumbDistance != 75.0) {
                    breadcrumbDistance = DEFAULT_BREADCRUMB_DIST;
                }
                
                Serial.printf("State Restored: Mode %d, Target %d, Dist %.0f\n", mode, targetIdx, breadcrumbDistance);
            }
            file.close();
        }
    }
}

void clearBreadcrumbs() {
    InternalFS.begin();
    InternalFS.remove("/crumbs.dat");
    InternalFS.remove("/state.txt"); // Clear state too
    Serial.println("Breadcrumbs cleared from Flash");
}

void saveBreadcrumb(double lat, double lon) {
    InternalFS.begin();
    File file = InternalFS.open("/crumbs.dat", FILE_O_WRITE);
    if (file) {
        file.seek(file.size()); // Simulate Append
        Breadcrumb b;
        b.lat = lat;
        b.lon = lon;
        size_t written = file.write((uint8_t*)&b, sizeof(Breadcrumb));
        file.flush(); // Ensure data hits flash
        file.close();
        
        if (written != sizeof(Breadcrumb)) {
             Serial.println("CRITICAL: Flash Write Failed (Full?)");
             // Visual Error signal
             #if HAS_RGB_LED
             pixels.setPixelColor(0, pixels.Color(255, 0, 0)); pixels.show(); delay(100);
             pixels.setPixelColor(0, pixels.Color(0, 0, 0)); pixels.show(); delay(100);
             pixels.setPixelColor(0, pixels.Color(255, 0, 0)); pixels.show();
             #endif
        }
    } else {
        Serial.println("CRITICAL: FS Open Failed!");
    }
}

void loadBreadcrumbs() {
    InternalFS.begin();
    breadcrumbs.clear();
    
    if (!InternalFS.exists("/crumbs.dat")) return;
    
    File file = InternalFS.open("/crumbs.dat", FILE_O_READ);
    if (file) {
        Breadcrumb b;
        while (file.read((uint8_t*)&b, sizeof(Breadcrumb)) == sizeof(Breadcrumb)) {
            breadcrumbs.push_back(b);
        }
        file.close();
        Serial.println("Loaded " + String(breadcrumbs.size()) + " breadcrumbs.");
    }
}
// Globals moved to top

// --- Non-Blocking Globals ---
volatile bool transmissionFlag = false;
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
  // Flag removed;
    }
}

void triggerVibration() {
    #if HAS_VIB_MOTOR
    digitalWrite(PIN_VIB_MOTOR, HIGH);
    vibrationStartTime = millis();
    isVibrating = true;
    #else
    // Visual Feedback via RGB LED (White Flash) if Motor Missing
    pixels.setPixelColor(0, pixels.Color(100, 100, 100)); // White
    pixels.show();
    vibrationStartTime = millis();
    isVibrating = true; // Recycle logic to turn LED off after delay
    #endif
}

void toggleFlashlight() {
    isFlashlightOn = !isFlashlightOn;
    isSOSActive = false; 
    // digitalWrite(PIN_FLASHLIGHT, isFlashlightOn ? HIGH : LOW);
    // TODO: Future upgrade: Miniature DC-SSR for power saving. LED removed for now.
}

void toggleSOS() {
    isSOSActive = !isSOSActive;
    isFlashlightOn = false; 
    if (!isSOSActive) {
        // digitalWrite(PIN_FLASHLIGHT, LOW);
        Serial.println("SOS Deactivated.");
    } else {
        Serial.println("SOS Activated!");
    }
}

void powerOff() {
    Serial.println("Entering Deep Sleep...");
    
    #if HAS_VIB_MOTOR
    digitalWrite(PIN_VIB_MOTOR, HIGH); delay(500);
    digitalWrite(PIN_VIB_MOTOR, LOW);  delay(200);
    digitalWrite(PIN_VIB_MOTOR, HIGH); delay(200);
    digitalWrite(PIN_VIB_MOTOR, LOW);  delay(200);
    digitalWrite(PIN_VIB_MOTOR, HIGH); delay(200);
    digitalWrite(PIN_VIB_MOTOR, LOW);
    #else
    // LED Fade Out
    for(int i=150; i>=0; i-=5) {
        pixels.setPixelColor(0, pixels.Color(0, 0, i)); // Blue Fade
        pixels.show();
        delay(10);
    }
    pixels.clear(); pixels.show();
    #endif

    // 2. Display OFF
    u8g2.setPowerSave(DISPLAY_POWER_SAVE_ON);
    
    // 3. Sensors OFF 
    // digitalWrite(PIN_VEXT, LOW); // No VEXT on SuperMini

    delay(10);

    // 6. Configure Wakeup
    nrf_gpio_cfg_sense_input(PIN_BUTTON, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    
    // Enter System OFF
    NRF_POWER->SYSTEMOFF = 1;
    while(1); // Wait for shutdown
}

float readBatteryVoltage() {
    // Seeed Xiao nRF52840 Specific Logic
    // P0.14 (PIN_BAT_READ_CTRL) must be LOW to ENABLE reading (Active Low)
    pinMode(PIN_BAT_READ_CTRL, OUTPUT);
    digitalWrite(PIN_BAT_READ_CTRL, LOW); 
    delay(10); // Stabilization

    // Read 12-bit ADC (0-4095) default for nRF52 Arduino
    analogReadResolution(12);
    int raw = analogRead(PIN_BAT_ADC);
    
    // Disable (High Impedance / High to turn off FET?)
    // Xiao Schematic: P0.14 drives Gate of PMOS. Low -> Conducts. High -> Off.
    digitalWrite(PIN_BAT_READ_CTRL, HIGH); 

    // Xiao Divider: 1M + 510K. Ratio = (1000+510)/510 = 2.9607
    // Reference 3.3V (VDD) usually? Or Internal 3.0/3.6?
    // Seeed Wiki says: Vcc = 3.3V. Resolution 1024 or 4096.
    // Formula from Wiki: Voltage = (raw / 4095.0) * 3.3 * 2.96 * 1.03 (Calibration)
    
    // Let's use 3.3V ref and 12-bit (4095)
    return (raw / 4095.0) * 3.3 * 2.96078;
}

int getBatteryPercent() {
    static unsigned long lastBatCheck = 0;
    static int cachedPct = 50; // Init with safe value

    // Dynamic Interval: 60s if Display ON, 300s (5min) if Display OFF
    unsigned long interval = isDisplayOn ? BAT_CHECK_INTERVAL_ACTIVE : BAT_CHECK_INTERVAL_IDLE;

    // Update if interval passed or first run
    if (millis() - lastBatCheck > interval || lastBatCheck == 0) {
        float voltage = readBatteryVoltage();
        
        // Conservative LiPo Mapping (Piecewise Linear)
        // Helps avoid "sudden death" at low voltages
        int pct = 0;
        if (voltage >= 4.05) {
            pct = 100;
        } else if (voltage >= 3.85) { // 3.85 - 4.05 -> 70% - 100%
            pct = 70 + (int)((voltage - 3.85) / (4.05 - 3.85) * 30);
        } else if (voltage >= 3.70) { // 3.70 - 3.85 -> 20% - 70%
            pct = 20 + (int)((voltage - 3.70) / (3.85 - 3.70) * 50);
        } else if (voltage >= 3.50) { // 3.50 - 3.70 -> 5% - 20%
            pct = 5 + (int)((voltage - 3.50) / (3.70 - 3.50) * 15);
        } else { // 3.30 - 3.50 -> 0% - 5%
            pct = (int)((voltage - 3.30) / (3.50 - 3.30) * 5);
        }

        if (pct > 100) pct = 100;
        if (pct < 0) pct = 0;
        
        cachedPct = pct;
        lastBatCheck = millis();
    }
    return cachedPct;
}

// Helper: Update RGB Status LED (Non-Blocking)
void updateStatusLED() {
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
        // GPS Fix acquired
        
        // Explorer Mode Heartbeat (Green Blink every 4s) to indicate recording without display
        if (currentMode == MODE_EXPLORE) {
            static unsigned long lastHeartbeat = 0;
            unsigned long now = millis();
            
            // Blink ON
            if (now - lastHeartbeat > 4000) {
                lastHeartbeat = now;
                pixels.setPixelColor(0, pixels.Color(0, LED_BRIGHTNESS, 0)); // Green
                pixels.show();
            } 
            // Blink OFF after 50ms (Short blip for power saving)
            else if (now - lastHeartbeat > 50) { 
                pixels.clear();
                pixels.show();
            }
        } else {
            // Other modes (Return/Confirm) -> LED off to save power
            pixels.clear();
            pixels.show();
        }
    }
}

// --- Hilfsfunktion für Menü-Icons ---
void drawMenuOverlay() {
  if (currentMenuSelection == MENU_NONE) return;

  // Schwarze Box am unteren Rand zeichnen
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, SCREEN_HEIGHT - 30, SCREEN_WIDTH, 30);
  u8g2.setDrawColor(1);
  u8g2.drawFrame(0, SCREEN_HEIGHT - 30, SCREEN_WIDTH, 30);

  u8g2.setFont(u8g2_font_helvB08_tf);
  u8g2.setCursor(5, SCREEN_HEIGHT - 10);

  switch (currentMenuSelection) {
    case MENU_MODE_SWITCH:
      u8g2.print("OPTION: SWITCH MODE");
      u8g2.setCursor(90, SCREEN_HEIGHT - 10);
      u8g2.print("[Hold]");
      break;
    case MENU_DIST_SWITCH:
      u8g2.print("DIST: " + String((int)breadcrumbDistance) + "m");
      u8g2.setCursor(90, SCREEN_HEIGHT - 10);
      u8g2.print("[Hold]");
      break;
    case MENU_POWER_OFF:
      u8g2.print("OPTION: POWER OFF");
      u8g2.setCursor(90, SCREEN_HEIGHT - 10);
      u8g2.print("[Hold]");
      break;
    default:
      break;
  }
}

void updateSOS() {
    if (!isSOSActive) return;
    
    unsigned long now = millis();

    // --- LED Pattern (SOS) ---
    // S (...) O (---) S (...)
    const int DOT = 200;
    const int GAP = 200;
    static const int pattern[] = {
        DOT, GAP, DOT, GAP, DOT, GAP*3,         // S
        DOT*3, GAP, DOT*3, GAP, DOT*3, GAP*3,   // O
        DOT, GAP, DOT, GAP, DOT, GAP*7          // S + Pause
    };
    const int steps = sizeof(pattern) / sizeof(pattern[0]);
    
    if (now - lastSOSUpdate > (unsigned long)pattern[sosStep]) {
        lastSOSUpdate = now;
        sosStep++;
        if (sosStep >= steps) sosStep = 0;
        
        // Blink NeoPixel RED
        if (sosStep % 2 == 0) {
            pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // ON (Red)
        } else {
            pixels.setPixelColor(0, pixels.Color(0, 0, 0));   // OFF
        }
        pixels.show();
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
    
    Serial.begin(115200);
    // SAFETY CATCH: Wait for USB and do nothing else
    // This allows us to verify if the core firmware is running.
    // Comment out the "while(1)" loop to proceed with normal boot.
    /*
    while (!Serial) delay(10);
    Serial.println("USB ALIVE - Safe Mode");
    while(1) { Serial.println("."); delay(1000); NRF_WDT->RR[0] = 0x6E524635; } 
    */

    /* DISABLED for Debugging USB
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
    */
    delay(500); // Startup stabilization

    // Power on VExt for sensors (GPS, LoRa, OLED)
    // pinMode(PIN_VEXT, OUTPUT);
    // digitalWrite(PIN_VEXT, HIGH);
    
    // GPS Power Management (V2 Board)
    #if defined(PIN_GPS_RST) && (PIN_GPS_RST >= 0)
    pinMode(PIN_GPS_RST, OUTPUT);
    digitalWrite(PIN_GPS_RST, HIGH); // Release Reset
    #endif
    #if defined(PIN_GPS_WAKE) && (PIN_GPS_WAKE >= 0)
    pinMode(PIN_GPS_WAKE, OUTPUT);
    digitalWrite(PIN_GPS_WAKE, HIGH); // Wake Up
    #endif

    // Init Battery Read Control
    pinMode(PIN_BAT_READ_CTRL, OUTPUT);
    digitalWrite(PIN_BAT_READ_CTRL, LOW);

    // CRITICAL: Sensors need time to boot after VEXT HIGH
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

    // Wait for Serial Monitor to connect (max 5 seconds)
    unsigned long serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart < 5000)) {
        delay(10);
    }

    delay(1000);
    Serial.println("\n\n=== Bring Em Home (T114) ===");

    // --- Watchdog Timer (WDT) Setup ---
    // Direct Register Access for nRF52840
    // Timeout: 10 seconds (approx)
    NRF_WDT->CONFIG = 0x01; // Keep running in Sleep mode
    NRF_WDT->CRV = 32768 * 10; // 32768Hz * 10s
    NRF_WDT->RREN = 0x01;   // Enable Reload Register 0
    NRF_WDT->TASKS_START = 1; // Start Watchdog
    Serial.println("WDT Initialized (10s)");
    
    // Load Saved Home
    hasSavedHome = loadHomePosition(savedHomeLat, savedHomeLon);

    // Load Breadcrumbs
    loadBreadcrumbs();

    // Resume State (Crash Recovery)
    loadState(currentMode, targetBreadcrumbIndex);
    if (currentMode == MODE_RETURN) {
        Serial.println("Resuming RETURN MODE after reset");
        // Ensure valid target index
        if (targetBreadcrumbIndex < 0 && !breadcrumbs.empty()) targetBreadcrumbIndex = breadcrumbs.size() - 1;
    }

    pinMode(PIN_BUTTON, INPUT_PULLUP);
    lastInteractionTime = millis();

    pinMode(PIN_VIB_MOTOR, OUTPUT);
    digitalWrite(PIN_VIB_MOTOR, LOW);

    // pinMode(PIN_FLASHLIGHT, OUTPUT);
    // digitalWrite(PIN_FLASHLIGHT, LOW);
    // TODO: Future upgrade: Miniature DC-SSR for power saving. LED removed for now.

    // Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
    // Wire.begin();
    // Wire.setClock(I2C_SPEED_FAST); // Increase I2C speed to 400kHz
    
    // Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
    // Wire.begin();
    // Wire.setClock(I2C_SPEED_FAST); 

    // Init External I2C for Sensor (P2 Header)
    // Wire1 uses same pins as Wire, avoiding double-init
    // Wire1.setPins(PIN_EXT_SDA, PIN_EXT_SCL);
    // Wire1.begin();
    // Wire1.setClock(I2C_SPEED_FAST);
    
    // Initialize IMU (BNO085 or BNO055)
    #if USE_BNO085
    /* 
    if (!bno08x.begin_I2C(BNO085_ADDRESS, &Wire)) {
        Serial.println("No BNO085 detected on External I2C!");
        hasCompass = false;
    } else {
        Serial.println("BNO085 Found!");
        hasCompass = true;
        for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
            // Optional: Print Part/Version info if needed
            // Serial.print("Part "); ...
        }
        // Enable Rotation Vector (Heading)
        if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 100000)) { // 100ms interval (10Hz)
             Serial.println("Could not enable rotation vector");
        }
        // Enable Linear Acceleration (Motion Detection)
        if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 200000)) { // 200ms
             Serial.println("Could not enable linear accel");
        }
    }
    */
    #else
    // Initialize BNO055 with Retry Logic
    bool bnoFound = false;
    /*
    for(int i=0; i<3; i++) {
        if(bno.begin()) {
            bnoFound = true;
            break;
        }
        delay(200);
    }
    */
    
    if (!bnoFound) {
        Serial.println("No BNO055 detected!");
        hasCompass = false;
    } else {
        hasCompass = true;
        bno.setExtCrystalUse(true);
    }
    #endif
    
    Serial1.setPins(PIN_GPS_RX, PIN_GPS_TX);
    Serial1.begin(GPS_BAUD);

    // --- GPS CONFIGURATION (U-Blox M10) ---
    // Set Update Rate to 2Hz (500ms) for better responsiveness while hiking/biking
    // Payload: 500ms (0x01F4), 1 cycle, GPS Time
    uint8_t ubxRate2Hz[] = { 
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 
        0xF4, 0x01, 0x01, 0x00, 0x01, 0x00, 
        0x0B, 0x77 
    };
    delay(100); // Allow GPS to boot UART
    Serial1.write(ubxRate2Hz, sizeof(ubxRate2Hz));
    Serial.println("GPS configured to 2Hz");

    // Optional: Send UBX config commands here if M10FD needs setup (e.g. rate, constellations)
    // Most M10 modules default to Auto-Baud or 9600/38400.
    
    startTime = millis();

    // Reserve memory for breadcrumbs to prevent fragmentation
    breadcrumbs.reserve(MAX_BREADCRUMBS);

    // u8g2.begin();
    // u8g2.setDrawColor(1);
    // u8g2.setFontPosTop();
    
    // Boot Logo
    // u8g2.clearBuffer();
    /*
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
    */
    delay(2000);
}

void loop() {
    // Feed the Watchdog (Keep system alive)
    NRF_WDT->RR[0] = 0x6E524635; // NRF_WDT_RR_VALUE

    // 0. Process GPS OFTEN (Before heavy tasks)
    while (Serial1.available()) {
        gps.encode(Serial1.read());
    }

    // --- GPS Debug Output (Every 2s) ---
    static unsigned long lastGpsDebug = 0;
    if (millis() - lastGpsDebug > 2000) {
        lastGpsDebug = millis();
        Serial.print("[GPS] Sats: "); Serial.print(gps.satellites.value());
        Serial.print(" | HDOP: "); Serial.print(gps.hdop.hdop()); // < 2.0 is good
        
        if (gps.location.isValid()) {
            Serial.print(" | Fix: "); 
            Serial.print(gps.location.lat(), 6);
            Serial.print(", ");
            Serial.print(gps.location.lng(), 6);
        } else {
            Serial.print(" | No Fix");
        }
        
        // Check if we are receiving ANY data
        Serial.print(" | Chars: "); Serial.println(gps.charsProcessed());
    }

    // --- Compass Calibration Check ---
    static unsigned long lastCalibCheck = 0;
    if (hasCompass && (millis() - lastCalibCheck > 1000)) {
        #if !USE_BNO085
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        if (!calibSaved && system == 3 && gyro == 3 && accel == 3 && mag == 3) {
            calibSaved = true;
        }
        #endif
        lastCalibCheck = millis();
    }

    // --- Charging Detection & Critical Battery Protect ---
    static unsigned long lastChargeCheck = 0;
    bool shouldCheck = (millis() < CHARGE_CHECK_DURATION) || isCharging;

    if (shouldCheck && (millis() - lastChargeCheck > CHARGE_CHECK_INTERVAL)) {
        float voltage = readBatteryVoltage();
        
        // Safety Cutoff (Protect LiPo)
        if (!isCharging && voltage < 3.0) {
             Serial.println("CRITICAL: Battery Low (<3.0V). Shutdown.");
             u8g2.setPowerSave(DISPLAY_POWER_SAVE_OFF);
             u8g2.clearBuffer();
             u8g2.setFont(u8g2_font_ncenB14_tr);
             u8g2.drawStr(10, 64, "EMPTY!");
             u8g2.sendBuffer();
             delay(3000);
             powerOff(); 
        }
        
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
        if (isSOSActive) {
            updateSOS();
        } else if (isCharging) {
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

    // 2. Process Compass & IMU Motion Analysis
    static bool isMoving = true; // Default to true if no IMU
    static unsigned long lastVerticalTime = 0; // Timer for drop detection
    
    if (hasCompass) {
        #if USE_BNO085
        if (bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
                // Convert Quaternion to Euler (Heading & Pitch)
                float r = sensorValue.un.arvrStabilizedRV.real;
                float i = sensorValue.un.arvrStabilizedRV.i;
                float j = sensorValue.un.arvrStabilizedRV.j;
                float k = sensorValue.un.arvrStabilizedRV.k;
                
                // Yaw (Heading)
                float siny_cosp = 2 * (r * k + i * j);
                float cosy_cosp = 1 - 2 * (j * j + k * k);
                currentHeading = atan2(siny_cosp, cosy_cosp) * 180.0 / PI;
                if (currentHeading < 0) currentHeading += 360.0;
                
                // Pitch (Tilt) - Rotation around X-axis (assuming device is mounted flat)
                // Note: Depending on mounting, this might need to swap with Roll.
                // Assuming standard: Flat = 0.
                float sinp = 2 * (r * j - k * i);
                if (fabs(sinp) >= 1) currentPitch = (sinp > 0) ? (PI / 2) : -(PI / 2); // Use 90 degrees if out of range
                else currentPitch = asin(sinp);
                currentPitch = currentPitch * 180.0 / PI;

                compassAccuracy = sensorValue.status;
            } else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
                 float x = sensorValue.un.linearAcceleration.x;
                 float y = sensorValue.un.linearAcceleration.y;
                 float z = sensorValue.un.linearAcceleration.z;
                 float mag = sqrt(x*x + y*y + z*z);
                 if (mag > ACCEL_MOTION_THRESHOLD) isMoving = true; else isMoving = false;
            }
        }
        #else
        sensors_event_t orientationData, linearAccelData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        currentHeading = orientationData.orientation.x;
        currentPitch = orientationData.orientation.y; // BNO055 Y is typically Pitch (check mounting)
        
        bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
        float x = linearAccelData.acceleration.x;
        float y = linearAccelData.acceleration.y;
        float z = linearAccelData.acceleration.z;
        float mag = sqrt(x*x + y*y + z*z);
        if (mag > ACCEL_MOTION_THRESHOLD) isMoving = true; else isMoving = false;
        #endif
    } else {
        isMoving = true; // Fallback: Trust GPS if no IMU
    }

    // --- SMART WAKE / SLEEP (IMU Based) ---
    // Only in MODE_RETURN (Power saving critical) or EXPLORE? Let's do both for consistency.
    // If device is held FLAT (+/- Threshold) -> Keep Awake / Wake Up
    // If device is DROPPED (Vertical) -> Fast Sleep
    
    if (hasCompass) {
        float absPitch = fabs(currentPitch);
        
        // Hysteresis Logic for State Stability (Schmitt Trigger)
        // If Display is ON, we are generous (keep it on up to 45 deg)
        // If Display is OFF, we are strict (must be flat < 35 deg to wake)
        float threshold = isDisplayOn ? (TILT_WAKE_ANGLE + TILT_HYSTERESIS) : TILT_WAKE_ANGLE;
        bool isFlat = (absPitch < threshold);
        
        if (isFlat) {
            // User is looking at screen: Keep awake
            // Immediate Wake (No lag when lifting)
            if (!isDisplayOn) {
                 u8g2.setPowerSave(DISPLAY_POWER_SAVE_OFF);
                 isDisplayOn = true;
            }
            lastInteractionTime = millis(); // Refresh standard timeout
            lastVerticalTime = millis();    // Reset "Time since flat"
        } else {
            // Screen is tilted away (Hanging)
            if (isDisplayOn) {
                 bool menuActive = (currentMenuSelection != MENU_NONE) || (millis() - lastMenuInteraction < 2000);
                 
                 // Turn OFF only if:
                 // 1. Not in menu
                 // 2. Tilted for more than TILT_SLEEP_DELAY (2s time-based Hysteresis)
                 if (!menuActive && (millis() - lastVerticalTime > TILT_SLEEP_DELAY)) {
                      u8g2.setPowerSave(DISPLAY_POWER_SAVE_ON);
                      isDisplayOn = false;
                 }
            } else {
                // If display is already off, keep tracking lastVerticalTime to prevent instant sleep-lock scenarios
                lastVerticalTime = millis(); 
            }
        }
    }
    
    // 3. Button Logic (Debounced)
    bool rawReading = digitalRead(PIN_BUTTON);
    static bool lastRawReading = HIGH;
    static unsigned long lastDebounceTime = 0;
    static bool debouncedButtonState = HIGH;

    if (rawReading != lastRawReading) {
        lastDebounceTime = millis();
    }
    lastRawReading = rawReading;

    if ((millis() - lastDebounceTime) > 50) {
        debouncedButtonState = rawReading;
    }

    bool currentButtonState = debouncedButtonState;
    unsigned long now = millis();

    // Button Pressed (Falling Edge)
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        buttonPressStartTime = now;
        isLongPressHandled = false;
    }

    // While Button Held
    if (currentButtonState == LOW) {
        unsigned long duration = now - buttonPressStartTime;
        
        // TOGGLE: EXPLORE <-> RETURN MODE (3s Hold)
        if (!isLongPressHandled && duration > 3000 && duration < 10000) {
             isLongPressHandled = true;

             if (currentMode == MODE_RETURN) {
                 // Deactivate -> Go to Explore
                 currentMode = MODE_EXPLORE;
                 saveState(currentMode, targetBreadcrumbIndex);
                 triggerVibration();
                 showFeedback("EXPLORE MODE", "Tracking...", FEEDBACK_DURATION_LONG);
             } else {
                 // Activate -> Go to Return
                 currentMode = MODE_RETURN;
                 // Haptic Feedback
                 triggerVibration(); delay(100); triggerVibration(); delay(100); triggerVibration();
                 showFeedback("GET EM HOME!", "Returning...", FEEDBACK_DURATION_LONG);
                 
                 // Set target to last breadcrumb
                 if (!breadcrumbs.empty()) {
                      targetBreadcrumbIndex = breadcrumbs.size() - 1;
                 }
                 saveState(currentMode, targetBreadcrumbIndex);
             }

             if (!isDisplayOn) { u8g2.setPowerSave(DISPLAY_POWER_SAVE_OFF); isDisplayOn = true; }
             lastInteractionTime = now;
        }

        // EXTRA LONG PRESS (15s) -> SETUP MODE (ID Selection) [Keep logic but fix structure if broken]
        if (!isLongPressHandled && duration > 15000) {
             isLongPressHandled = true;
             // Legacy setup mode logic... disabled/simplified
             showFeedback("SETUP LOCKED", "", 2000);
        }
    }

    // Button Released (Rising Edge)
    if (currentButtonState == HIGH && lastButtonState == LOW) {
        unsigned long duration = now - buttonPressStartTime;
        lastInteractionTime = now; // Reset display timeout
        
        if (!isDisplayOn) {
            // Wake Up only
            u8g2.setPowerSave(DISPLAY_POWER_SAVE_OFF);
            isDisplayOn = true;
            isLongPressHandled = true; // Don't process click
        } else {
             // Display is ON
             if (!isLongPressHandled) {
                 if (duration < LONG_PRESS_THRESHOLD) {
                     // --- SHORT CLICK ---
                     if (currentMode == MODE_SET_ID) {
                         deviceID++;
                         if (deviceID > 5) deviceID = 1;
                     } else {
                         // Cycle Menu
                         switch(currentMenuSelection) {
                             case MENU_NONE: currentMenuSelection = MENU_MODE_SWITCH; break;
                             case MENU_MODE_SWITCH: currentMenuSelection = MENU_DIST_SWITCH; break;
                             case MENU_DIST_SWITCH: currentMenuSelection = MENU_POWER_OFF; break;
                             case MENU_POWER_OFF: currentMenuSelection = MENU_NONE; break;
                             default: currentMenuSelection = MENU_NONE; break;
                         }
                         if (currentMenuSelection != MENU_NONE) lastMenuInteraction = now;
                     }
                 } else {
                     // --- LONG CLICK (Action) ---
                     if (currentMode == MODE_SET_ID) {
                         showFeedback("ID SAVED", "ID: " + String(deviceID), 3000);
                         currentMode = MODE_EXPLORE;
                     } else {
                         if (currentMenuSelection != MENU_NONE) {
                             switch(currentMenuSelection) {
                                 case MENU_MODE_SWITCH:
                                    if (currentMode == MODE_EXPLORE) {
                                        currentMode = MODE_RETURN;
                                        showFeedback("GET EM HOME!", "Returning...", 2000);
                                        if (!breadcrumbs.empty() && gps.location.isValid()) {
                                            targetBreadcrumbIndex = breadcrumbs.size() - 1; 
                                        }
                                    } else {
                                        currentMode = MODE_EXPLORE;
                                        showFeedback("EXPLORE MODE", "", 2000);
                                    }
                                    saveState(currentMode, targetBreadcrumbIndex);
                                    triggerVibration();
                                    currentMenuSelection = MENU_NONE;
                                    break;
                                 case MENU_DIST_SWITCH:
                                    breadcrumbDistance += 25.0;
                                    if (breadcrumbDistance > 75.0) breadcrumbDistance = 25.0;
                                    saveState(currentMode, targetBreadcrumbIndex);
                                    triggerVibration();
                                    lastMenuInteraction = now; // Keep menu open
                                    break;
                                 case MENU_POWER_OFF:
                                    powerOff();
                                    break;
                                 default: break;
                             }
                         }
                     }
                 }
             }
        }
    }
    lastButtonState = currentButtonState;

    // Confirm Home Logic (Legacy 1x/2x click)
    if (currentMode == MODE_CONFIRM_HOME && clickCount > 0 && (millis() - lastClickTime > 1000)) {
            if (clickCount == 1) {
                // YES: Set Home Here
                homeLat = gps.location.lat();
                homeLon = gps.location.lng();
                hasHome = true;
                saveHomePosition(homeLat, homeLon);
                currentMode = MODE_EXPLORE;
                breadcrumbs.clear();
                clearBreadcrumbs();
                showFeedback("HOME SET!", "Train Cleared", FEEDBACK_DURATION_LONG);
            } else if (clickCount >= 2) {
                // NO: Use Saved Home
                if (hasSavedHome) {
                    homeLat = savedHomeLat;
                    homeLon = savedHomeLon;
                    hasHome = true;
                    currentMode = MODE_EXPLORE;
                    showFeedback("OLD HOME", "Loaded from Flash", FEEDBACK_DURATION_LONG);
                } else {
                    showFeedback("NO SAVED HOME", "Click 1x to Set", FEEDBACK_DURATION_LONG);
                }
            }
            clickCount = 0;
    }
    
    // Menu Timeout
    if (currentMenuSelection != MENU_NONE && (millis() - lastMenuInteraction > MENU_TIMEOUT)) {
        currentMenuSelection = MENU_NONE;
    }

    if (isSOSCountdown) {
        if (millis() - sosCountdownStartTime > SOS_COUNTDOWN_DURATION) {
            isSOSCountdown = false;
            toggleSOS();
            triggerVibration(); delay(100); triggerVibration(); delay(100); triggerVibration();
        }
    }

    // 4. Auto-Home Logic (Replaced by Manual Confirmation)
    if (!hasHome && gps.location.isValid() && currentMode != MODE_CONFIRM_HOME) {
        // Instead of auto-setting, we enter Confirmation Mode
        currentMode = MODE_CONFIRM_HOME;
        Serial.println("GPS Fix! Asking for Home Confirmation...");
        triggerVibration();
        if (!isDisplayOn) { u8g2.setPowerSave(DISPLAY_POWER_SAVE_OFF); isDisplayOn = true; }
    }

    // 5. Breadcrumb Logic
    bool isGoodSignal = gps.hdop.hdop() < 2.5;
    
    // IMU Filter: Only record if we are physically moving (or no IMU present)
    // This effectively kills "Stationary Drift" where GPS wanders while sitting still.
    
    if (currentMode == MODE_EXPLORE && gps.location.isValid() && isGoodSignal && isMoving) {
        bool shouldSave = false;
        double speed = gps.speed.kmph();
        if (speed > MIN_SPEED_KPH && speed < MAX_SPEED_KPH) {
            if (breadcrumbs.empty()) {
                shouldSave = true;
            } else {
                Breadcrumb last = breadcrumbs.back();
                double dist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), last.lat, last.lon);
                if (dist > breadcrumbDistance) {
                    shouldSave = true;
                } else if (dist > BREADCRUMB_MIN_DIST_TURN) {
                    double currentSegmentBearing = gps.courseTo(last.lat, last.lon, gps.location.lat(), gps.location.lng());
                    if (breadcrumbs.size() >= 2) {
                        Breadcrumb prev = breadcrumbs[breadcrumbs.size()-2];
                        double prevSegmentBearing = gps.courseTo(prev.lat, prev.lon, last.lat, last.lon);
                        double diff = fabs(currentSegmentBearing - prevSegmentBearing);
                        if (diff > 180) diff = 360 - diff;
                        if (diff > BREADCRUMB_TURN_THRESHOLD) shouldSave = true;
                    }
                }
            }
        }
        
        if (shouldSave) {
            // RAM Buffer Logic (Circular buffer removed for simplicity with append-only file)
            if (breadcrumbs.size() >= MAX_BREADCRUMBS) {
                 // Option A: Stop recording
                 // Option B: Remove oldest? (Hard with append-only file)
                 // We will just stop adding to RAM to prevent crash.
                 Serial.println("Breadcrumb Limit Reached!");
            } else {
                Breadcrumb b; b.lat = gps.location.lat(); b.lon = gps.location.lng();
                breadcrumbs.push_back(b);
                saveBreadcrumb(b.lat, b.lon); // Persist to Flash
                Serial.printf("Breadcrumb saved: %f, %f (Total: %d)\n", b.lat, b.lon, breadcrumbs.size());
            }
        }
    }

    // 6. Backtrack Logic
    if (currentMode == MODE_RETURN && gps.location.isValid() && !breadcrumbs.empty()) {
        if (targetBreadcrumbIndex >= 0) {
            double distToTarget = gps.distanceBetween(gps.location.lat(), gps.location.lng(), breadcrumbs[targetBreadcrumbIndex].lat, breadcrumbs[targetBreadcrumbIndex].lon);
            if (distToTarget < BREADCRUMB_REACH_DIST) {
                targetBreadcrumbIndex--;
                triggerVibration();
                saveState(currentMode, targetBreadcrumbIndex); 
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
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 40); // Moved up
                u8g2.print(feedbackTitle);

                // Special Graphic for "Return Mode"
                if (feedbackTitle == "GET EM HOME!") {
                    // Coordinates
                    int base_y = 95;
                    
                    // 1. Flower "Emilie" (Left) - Outline
                    int fx = 25; int fr = 6;
                    u8g2.drawCircle(fx, base_y - fr, fr); // Top
                    u8g2.drawCircle(fx, base_y + fr, fr); // Bottom
                    u8g2.drawCircle(fx - fr, base_y, fr); // Left
                    u8g2.drawCircle(fx + fr, base_y, fr); // Right
                    u8g2.drawCircle(fx, base_y, 3);       // Center

                    // 2. House (Right) - Outline
                    int hx = 95; // Moved slightly right
                    int hy = base_y;
                    u8g2.drawFrame(hx, hy - 8, 16, 16); // Body
                    // Roof (Triangle)
                    u8g2.drawLine(hx - 2, hy - 8, hx + 8, hy - 18);
                    u8g2.drawLine(hx + 8, hy - 18, hx + 18, hy - 8);
                    u8g2.drawLine(hx - 2, hy - 8, hx + 18, hy - 8); // Roof base
                    // Door (Filled)
                    u8g2.drawBox(hx + 6, hy, 4, 8);

                    // 3. Thick Outline Arrow (Center)
                    int ax_s = 50; int ax_e = 85; 
                    int ay = base_y;
                    int aw = 8; // Arrow Width (Tail)
                    int hw = 16; // Head Width
                    int hl = 12; // Head Length

                    // Drawn as loop of lines
                    // Tail Top (50, 91) -> Head Base Top (73, 91)
                    u8g2.drawLine(ax_s, ay - aw/2, ax_e - hl, ay - aw/2);
                    // Head Base Top Up (73, 91) -> Wing Top (73, 87)
                    u8g2.drawLine(ax_e - hl, ay - aw/2, ax_e - hl, ay - hw/2);
                    // Wing Top -> Tip (85, 95)
                    u8g2.drawLine(ax_e - hl, ay - hw/2, ax_e, ay);
                    // Tip -> Wing Bottom (73, 103)
                    u8g2.drawLine(ax_e, ay, ax_e - hl, ay + hw/2);
                    // Wing Bottom -> Head Base Bottom (73, 99)
                    u8g2.drawLine(ax_e - hl, ay + hw/2, ax_e - hl, ay + aw/2);
                    // Head Base Bottom -> Tail Bottom (50, 99)
                    u8g2.drawLine(ax_e - hl, ay + aw/2, ax_s, ay + aw/2);
                    // Tail Back (50, 99) -> Tail Top (50, 91)
                    u8g2.drawLine(ax_s, ay + aw/2, ax_s, ay - aw/2);

                } else if (feedbackSub.length() > 0) {
                    u8g2.setFont(u8g2_font_6x10_tr);
                    // ... old logic for generic messages ...
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
                        u8g2.setCursor((SCREEN_WIDTH - w) / 2, 70);
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
                
                u8g2.setFont(u8g2_font_6x10_tr);
                const char* sub = "Beacon Active";
                w = u8g2.getStrWidth(sub);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 60);
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

            {
                #if !USE_BNO085
                if (mag == 3) compStr = "Good"; else if (mag == 2) compStr = "Ok"; else if (mag == 1) compStr = "Low";
                String compDisp = "C:" + compStr;
                w = u8g2.getStrWidth(compDisp.c_str());
                u8g2.setCursor((128 - w) / 2, 10);
                u8g2.print(compDisp);
                #else
                // BNO085 Calibration Check (Accuracy in bits 0-1 of status)
                uint8_t accuracy = sensorValue.status & 0x03;
                if (accuracy < 2) {
                    u8g2.setFont(u8g2_font_5x7_tr);
                    u8g2.setCursor(50, 12);
                    u8g2.print("CAL!");
                }
                #endif
            }

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

            if (currentMode == MODE_CONFIRM_HOME) {
                u8g2.setFont(u8g2_font_ncenB10_tr);
                const char* title = "SET HOME HERE?";
                w = u8g2.getStrWidth(title);
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 30);
                u8g2.print(title);
                
                u8g2.setFont(u8g2_font_6x10_tr);
                String latStr = "Lat: " + String(gps.location.lat(), 5);
                String lonStr = "Lon: " + String(gps.location.lng(), 5);
                w = u8g2.getStrWidth(latStr.c_str());
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 50);
                u8g2.print(latStr);
                w = u8g2.getStrWidth(lonStr.c_str());
                u8g2.setCursor((SCREEN_WIDTH - w) / 2, 62);
                u8g2.print(lonStr);
                
                u8g2.drawHLine(0, 75, SCREEN_WIDTH);
                
                u8g2.setFont(u8g2_font_ncenB10_tr);
                u8g2.setCursor(10, 95);
                u8g2.print("1x: YES");
                u8g2.setCursor(10, 115);
                u8g2.print("2x: NO (Load)");
                
                u8g2.sendBuffer();
                return;
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

            // Smooth Compass
            double diff = currentHeading - displayedHeading;
            if (diff > 180) diff -= 360;
            if (diff < -180) diff += 360;
            displayedHeading += diff * 0.15;
            if (displayedHeading >= 360) displayedHeading -= 360;
            if (displayedHeading < 0) displayedHeading += 360;

            if (showArrow) {
                int relBearing = (int)bearing - (int)displayedHeading;
                if (relBearing < 0) relBearing += 360;
                int arrowCy = 64;
                bool showCardinals = (currentMode == MODE_EXPLORE);
                drawArrow(SCREEN_WIDTH/2, arrowCy, 30, relBearing, showCardinals);

                // Compass Low Accuracy Warning
                if (hasCompass && compassAccuracy < 2) {
                    u8g2.setFont(u8g2_font_5x7_tr);
                    u8g2.drawStr(110, 8, "CAL"); // Top Right
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
            drawMenuOverlay();
            u8g2.sendBuffer();
        }
    }
}