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

using namespace Adafruit_LittleFS_Namespace;

#if USE_BNO085
#include <Adafruit_BNO08x.h>
#else
#include <Adafruit_BNO055.h>
#endif
#include <nrf_gpio.h>
#include <nrf_power.h>
#include <vector>
#include <RadioLib.h>
#include <Adafruit_NeoPixel.h>

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

// --- FileSystem Helpers ---
void saveHomePosition(double lat, double lon) {
    InternalFS.begin();
    File file = InternalFS.open("/home.txt", FILE_O_WRITE);
    if (file) {
        file.seek(0);
        file.print(String(lat, 8));
        file.print(",");
        file.print(String(lon, 8));
        file.close();
        Serial.println("Home Saved to Flash!");
    }
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
#if USE_BNO085
Adafruit_BNO08x bno08x(PIN_LORA_RST); // Reset pin not strictly needed if tied to MCU reset, but good practice
sh2_SensorValue_t sensorValue;
#else
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_ID, BNO055_ADDRESS);
#endif

// Use default SPI instance for nRF52
SPIClass& loraSPI = SPI;

SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);

// LoRaWAN Globals (Hybrid Mode)
LoRaWANNode* node = nullptr;
bool loraWanJoined = false;
uint64_t joinEUI = LORAWAN_JOIN_EUI;
uint64_t devEUI = LORAWAN_DEV_EUI;
uint8_t appKey[] = LORAWAN_APP_KEY;
uint8_t nwkKey[] = LORAWAN_APP_KEY; // For LoRaWAN 1.0.x, NwkKey is usually same as AppKey

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
    MODE_RETURN,
    MODE_CONFIRM_HOME
};

// --- MENU DEFINITONS ---
enum MenuState {
  MENU_NONE,
  MENU_MODE_SWITCH, // Explore <-> Return
  MENU_LIGHT,       // Taschenlampe
  MENU_SOS,         // SOS Starten
  MENU_POWER_OFF    // Ausschalten
};

MenuState currentMenuSelection = MENU_NONE;
unsigned long lastMenuInteraction = 0;
const unsigned long MENU_TIMEOUT = 4000; // Menü schließt automatisch nach 4s
const unsigned long LONG_PRESS_THRESHOLD = 800; // ms für Bestätigung
const unsigned long SOS_CONFIRM_TIME = 10000; // 10 Sekunden halten für SOS aus Menü
AppMode currentMode = MODE_CONFIRM_HOME; // Start in Confirm Mode (Wait for GPS)

// Navigation Data
double homeLat = 0.0;
double homeLon = 0.0;
double savedHomeLat = 0.0;
double savedHomeLon = 0.0;
bool hasSavedHome = false;
bool hasHome = false;
double currentHeading = 0.0;
double displayedHeading = 0.0;
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

// Buddy Tracking
double buddyLat = 0.0;
double buddyLon = 0.0;
unsigned long lastBuddyPacketTime = 0;
bool hasBuddy = false;
unsigned long lastBuddyTx = 0; // For sending periodic updates

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
    case MENU_LIGHT:
      u8g2.print("OPTION: LIGHT");
      u8g2.print(isFlashlightOn ? " OFF" : " ON");
      u8g2.setCursor(90, SCREEN_HEIGHT - 10);
      u8g2.print("[Hold]");
      break;
    case MENU_SOS:
      u8g2.print("OPTION: SEND SOS");
      u8g2.setCursor(80, SCREEN_HEIGHT - 10);
      if (digitalRead(PIN_BUTTON) == LOW && currentMenuSelection == MENU_SOS) {
           unsigned long pressedFor = millis() - buttonPressStartTime;
           int remain = 10 - (pressedFor / 1000);
           if (remain < 0) remain = 0;
           u8g2.print("HOLD: " + String(remain));
      } else {
           u8g2.print("[Hold 10s]");
      }
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

    // --- LoRa Transmission (Every 1 Minute) ---
    if (now - lastLoRaTx > LORA_TX_INTERVAL) {
        lastLoRaTx = now;
        int batPct = getBatteryPercent();
        
        // Calculate estimated runtime
        float remainingCap = (float)BATTERY_CAPACITY_MAH * ((float)batPct / 100.0);
        float runtimeHours = remainingCap / (float)SOS_CURRENT_MA;

        // --- Hybrid Mode Logic ---
        bool sentViaLoRaWAN = false;

        #if ENABLE_LORAWAN
        if (!node) {
             node = new LoRaWANNode(&radio, &EU868);
        }

        // Try to join if not joined
        if (!loraWanJoined) {
            Serial.println("LoRaWAN: Attempting Join...");
            // Note: beginOTAA is usually called once to set keys, then activateOTAA does the join
            node->beginOTAA(joinEUI, devEUI, nwkKey, appKey);
            int state = node->activateOTAA();
            if (state == RADIOLIB_ERR_NONE) {
                Serial.println("LoRaWAN: Joined!");
                loraWanJoined = true;
            } else {
                Serial.print("LoRaWAN: Join Failed, code "); Serial.println(state);
            }
        }

        if (loraWanJoined) {
             // Prepare Payload (CayenneLPP style or simple bytes)
             // 4 bytes Lat, 4 bytes Lon, 1 byte Bat
             uint8_t payload[9];
             int32_t lat = gps.location.lat() * 1000000;
             int32_t lon = gps.location.lng() * 1000000;
             
             payload[0] = (lat >> 24) & 0xFF;
             payload[1] = (lat >> 16) & 0xFF;
             payload[2] = (lat >> 8) & 0xFF;
             payload[3] = lat & 0xFF;
             payload[4] = (lon >> 24) & 0xFF;
             payload[5] = (lon >> 16) & 0xFF;
             payload[6] = (lon >> 8) & 0xFF;
             payload[7] = lon & 0xFF;
             payload[8] = (uint8_t)batPct;

             Serial.println("LoRaWAN: Sending Uplink...");
             int state = node->sendReceive(payload, 9);
             if (state == RADIOLIB_ERR_NONE) {
                 Serial.println("LoRaWAN: Sent!");
                 sentViaLoRaWAN = true;
             } else {
                 Serial.print("LoRaWAN: Send Failed, code "); Serial.println(state);
                 // If send fails, maybe we lost coverage. 
                 // We don't reset loraWanJoined immediately to avoid re-joining loop, 
                 // but for SOS reliability, we fall back to P2P.
             }
        }
        #endif

        if (!sentViaLoRaWAN) {
            // Fallback to P2P
            Serial.println("LoRa P2P: Sending SOS...");
            
            // Re-Initialize for P2P (overwrites LoRaWAN config)
            radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC_WORD, LORA_POWER, LORA_PREAMBLE, LORA_GAIN, false);
            
            String msg = String(SOS_MSG_TEXT) + " Lat:" + String(gps.location.lat(), 6) + 
                         " Lon:" + String(gps.location.lng(), 6) + 
                         " Bat:" + String(batPct) + "%" +
                         " Est:" + String(runtimeHours, 1) + "h";

            radio.standby();
            transmissionFlag = false;
            int state = radio.startTransmit(msg);
            
            if (state == RADIOLIB_ERR_NONE) {
                isLoRaTransmitting = true;
            } else {
                radio.sleep();
            }
            
            // Note: If we switched to P2P, the LoRaWAN session in the radio chip is lost.
            // Next time we try LoRaWAN, we might need to restore it or re-join.
            // For simplicity in this Hybrid implementation, we will force a re-join next time if ENABLE_LORAWAN is on.
            #if ENABLE_LORAWAN
            loraWanJoined = false; 
            #endif
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
    
    if (now - lastSOSUpdate > (unsigned long)pattern[sosStep]) {
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

// --- Buddy Protocol Manager ---
void sendBuddyPacket() {
    #if !ENABLE_LORAWAN
    if (gps.location.isValid()) {
        String msg = "BUDDY Lat:" + String(gps.location.lat(), 6) + " Lon:" + String(gps.location.lng(), 6);
        Serial.println("Sending Buddy Update...");
        radio.standby();
        transmissionFlag = false;
        int state = radio.startTransmit(msg);
        if (state == RADIOLIB_ERR_NONE) {
            isLoRaTransmitting = true;
        } else {
            Serial.print("TX Failed: "); Serial.println(state);
            radio.startReceive(); // Go back to RX if failed
        }
    }
    #endif
}

void manageBuddyProtocol() {
    if (isSOSActive) return;
    #if ENABLE_LORAWAN
    return;
    #endif

    // Fallback if no GPS time: Use simple timer and continuous RX
    if (!gps.time.isValid()) {
        if (millis() - lastBuddyTx > BUDDY_TX_INTERVAL) {
            lastBuddyTx = millis();
            sendBuddyPacket();
        }
        return;
    }

    #if ENABLE_SMART_POWER
    int currentSec = gps.time.second();
    
    // Define Slots
    bool txSlot = false;
    bool rxSlot = false;
    
    // Device 0: TX @ :00, :30. RX @ :15, :45 (+/- 2s)
    // Device 1: TX @ :15, :45. RX @ :00, :30 (+/- 2s)
    
    if (BUDDY_DEVICE_ID == 0) {
        if (currentSec == 0 || currentSec == 30) txSlot = true;
        if ((currentSec >= 13 && currentSec <= 17) || (currentSec >= 43 && currentSec <= 47)) rxSlot = true;
    } else {
        if (currentSec == 15 || currentSec == 45) txSlot = true;
        if ((currentSec >= 58 || currentSec <= 2) || (currentSec >= 28 && currentSec <= 32)) rxSlot = true;
    }
    
    // TX Logic
    static int lastTxSec = -1;
    if (txSlot && currentSec != lastTxSec) {
        sendBuddyPacket();
        lastTxSec = currentSec;
    }
    
    // RX/Sleep Logic
    static bool isListening = false;
    if (!isLoRaTransmitting) {
        if (rxSlot) {
            if (!isListening) {
                radio.startReceive();
                isListening = true;
            }
        } else {
            if (isListening) {
                radio.standby();
                isListening = false;
            }
        }
    }
    #else
    // Standard Logic (Continuous RX)
    if (millis() - lastBuddyTx > BUDDY_TX_INTERVAL) {
        lastBuddyTx = millis();
        sendBuddyPacket();
    }
    #endif
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
    
    // Load Saved Home
    hasSavedHome = loadHomePosition(savedHomeLat, savedHomeLon);

    pinMode(PIN_BUTTON, INPUT_PULLUP);
    lastInteractionTime = millis();

    pinMode(PIN_VIB_MOTOR, OUTPUT);
    digitalWrite(PIN_VIB_MOTOR, LOW);

    pinMode(PIN_FLASHLIGHT, OUTPUT);
    digitalWrite(PIN_FLASHLIGHT, LOW);

    Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(I2C_SPEED_FAST); // Increase I2C speed to 400kHz to prevent display blocking GPS
    
    Wire.setClock(I2C_SPEED_FAST); // Increase I2C speed to 400kHz to prevent display blocking GPS
    
    // Initialize IMU (BNO085 or BNO055)
    #if USE_BNO085
    if (!bno08x.begin_I2C(BNO085_ADDRESS)) {
        Serial.println("No BNO085 detected!");
    } else {
        Serial.println("BNO085 Found!");
        for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
            Serial.print("Part "); Serial.print(bno08x.prodIds.entry[n].swPartNumber);
            Serial.print(": Version "); Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
            Serial.print("."); Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
            Serial.print("."); Serial.println(bno08x.prodIds.entry[n].swVersionPatch);
        }
        // Enable Rotation Vector (Heading)
        if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 50000)) { // 50ms interval
             Serial.println("Could not enable rotation vector");
        }
    }
    #else
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
    #endif
    
    Serial1.setPins(PIN_GPS_RX, PIN_GPS_TX);
    Serial1.begin(GPS_BAUD);

    loraSPI.setPins(PIN_LORA_MISO, PIN_LORA_SCK, PIN_LORA_MOSI);
    loraSPI.begin();

    int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC_WORD, LORA_POWER, LORA_PREAMBLE, LORA_GAIN, false);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("LoRa Init Success!");
        radio.setDio1Action(setFlag);
        // Start Listening immediately (P2P)
        #if !ENABLE_LORAWAN
        radio.startReceive();
        #endif
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
            isLoRaTransmitting = false;
            Serial.println("LoRa TX Finished.");
            
            // Resume RX if needed (Smart Power handles this, but fallback needs immediate RX)
            #if !ENABLE_LORAWAN
            #if ENABLE_SMART_POWER
            if (!gps.time.isValid()) radio.startReceive();
            #else
            radio.startReceive();
            #endif
            #else
            if (!loraWanJoined) radio.startReceive();
            #endif
        } else {
            // RX Done
            String str;
            int state = radio.readData(str);
            if (state == RADIOLIB_ERR_NONE) {
                Serial.print("RX: "); Serial.println(str);
                // Parse Buddy Packet: "BUDDY Lat:48.123 Lon:11.123"
                if (str.startsWith("BUDDY")) {
                    int latIdx = str.indexOf("Lat:");
                    int lonIdx = str.indexOf("Lon:");
                    if (latIdx > 0 && lonIdx > 0) {
                        String latStr = str.substring(latIdx + 4, str.indexOf(" ", latIdx));
                        String lonStr = str.substring(lonIdx + 4);
                        buddyLat = latStr.toFloat();
                        buddyLon = lonStr.toFloat();
                        lastBuddyPacketTime = millis();
                        hasBuddy = true;
                        showFeedback("BUDDY FOUND!", "", 2000);
                        triggerVibration();
                    }
                }
            }
            // Resume Listening (Smart Power handles this, but fallback needs immediate RX)
            #if !ENABLE_LORAWAN
            #if ENABLE_SMART_POWER
            if (!gps.time.isValid()) radio.startReceive();
            #else
            radio.startReceive();
            #endif
            #else
            if (!loraWanJoined) radio.startReceive();
            #endif
        }
    }

    // --- Compass Calibration Check ---
    static unsigned long lastCalibCheck = 0;
    if (millis() - lastCalibCheck > 1000) {
        #if !USE_BNO085
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        if (!calibSaved && system == 3 && gyro == 3 && accel == 3 && mag == 3) {
            calibSaved = true;
        }
        #endif
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
    #if USE_BNO085
    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
            // Convert Quaternion to Euler (Heading)
            float qw = sensorValue.un.arvrStabilizedRV.real;
            float qx = sensorValue.un.arvrStabilizedRV.i;
            float qy = sensorValue.un.arvrStabilizedRV.j;
            float qz = sensorValue.un.arvrStabilizedRV.k;
            
            float siny_cosp = 2 * (qw * qz + qx * qy);
            float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
            float yaw = atan2(siny_cosp, cosy_cosp);
            
            currentHeading = (yaw * 180.0 / PI);
            if (currentHeading < 0) currentHeading += 360.0;
        }
    }
    // uint8_t mag = 3; // BNO085 handles calibration internally, assume good
    #else
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    currentHeading = orientationData.orientation.x;

    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    #endif

    // --- Buddy Protocol (Smart Power / Periodic) ---
    if (currentMode == MODE_EXPLORE) {
        manageBuddyProtocol();
    }

    // 3. Button Logic
    bool currentButtonState = digitalRead(PIN_BUTTON);
    unsigned long now = millis();

    // Button Pressed (Falling Edge)
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        buttonPressStartTime = now;
        isLongPressHandled = false;
    }

    // While Button Held
    if (currentButtonState == LOW) {
        unsigned long duration = now - buttonPressStartTime;

        // PANIC SHORTCUT (Hold 3s, no menu active)
        if (!isLongPressHandled && currentMenuSelection == MENU_NONE && duration > 3000) {
             isLongPressHandled = true;
             // Force Panic / Return Mode
             if (currentMode == MODE_EXPLORE) {
                 currentMode = MODE_RETURN;
                 // Haptic Feedback
                 triggerVibration(); delay(100); triggerVibration(); delay(100); triggerVibration();
                 showFeedback("RETURN MODE", "PANIC ACTIVATED", FEEDBACK_DURATION_LONG);
                 if (!isDisplayOn) { u8g2.setPowerSave(DISPLAY_POWER_SAVE_OFF); isDisplayOn = true; }
                 lastInteractionTime = now;
             }
        }
        
        // SOS COUNTDOWN (Hold 10s if Menu=SOS)
        if (currentMenuSelection == MENU_SOS && duration > SOS_CONFIRM_TIME) {
             if (!isSOSActive && !isLongPressHandled) { // Trigger once
                 isLongPressHandled = true;
                 if (!isSOSActive) toggleSOS();
                 currentMenuSelection = MENU_NONE;
                 triggerVibration(); delay(500); triggerVibration();
             }
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
                     // SHORT CLICK
                     if (currentMode == MODE_CONFIRM_HOME) {
                         clickCount++;
                         lastClickTime = now;
                     } else {
                         // NORMAL OPERATION -> SINGLE BUTTON MENU navigation
                         if (currentMenuSelection == MENU_NONE) {
                             currentMenuSelection = MENU_MODE_SWITCH;
                         } else {
                             // Cycle (MODE -> LIGHT -> OFF -> SOS)
                             switch(currentMenuSelection) {
                                 case MENU_MODE_SWITCH: currentMenuSelection = MENU_LIGHT; break;
                                 case MENU_LIGHT: currentMenuSelection = MENU_POWER_OFF; break;
                                 case MENU_POWER_OFF: currentMenuSelection = MENU_SOS; break;
                                 case MENU_SOS: currentMenuSelection = MENU_NONE; break;
                                 default: currentMenuSelection = MENU_NONE;
                             }
                         }
                         lastMenuInteraction = now;
                     }
                 } else {
                     // LONG CLICK (Action confirmation)
                     if (currentMenuSelection != MENU_NONE) {
                         switch(currentMenuSelection) {
                             case MENU_MODE_SWITCH:
                                if (currentMode == MODE_EXPLORE) {
                                    currentMode = MODE_RETURN;
                                    showFeedback("RETURN MODE", "", 2000);
                                    if (!breadcrumbs.empty() && gps.location.isValid()) {
                                        targetBreadcrumbIndex = breadcrumbs.size() - 1; 
                                    }
                                } else {
                                    currentMode = MODE_EXPLORE;
                                    showFeedback("EXPLORE MODE", "", 2000);
                                }
                                triggerVibration();
                                currentMenuSelection = MENU_NONE;
                                break;
                             case MENU_LIGHT:
                                toggleFlashlight();
                                triggerVibration();
                                currentMenuSelection = MENU_NONE;
                                break;
                             case MENU_SOS:
                                // Warn user
                                showFeedback("HOLD TO ACTIVATE", "10 SECONDS", 2000);
                                triggerVibration();
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
    lastButtonState = currentButtonState;

    // Legacy "Confirm Home" logic
    if (currentMode == MODE_CONFIRM_HOME && clickCount > 0 && (millis() - lastClickTime > 1000)) {
            if (clickCount == 1) {
                // YES: Set Home Here
                homeLat = gps.location.lat();
                homeLon = gps.location.lng();
                hasHome = true;
                saveHomePosition(homeLat, homeLon);
                currentMode = MODE_EXPLORE;
                showFeedback("HOME SET!", "Saved to Flash", FEEDBACK_DURATION_LONG);
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

            // Buddy Indicator (Top Right)
            if (hasBuddy && (millis() - lastBuddyPacketTime < 300000)) {
                u8g2.setCursor(100, 12);
                u8g2.print("BUDDY");
            } else {
                #if !USE_BNO085
                String compStr = "Bad";
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

                // Draw Buddy Arrow (Secondary)
                if (hasBuddy && (millis() - lastBuddyPacketTime < 300000)) {
                    double buddyBearing = gps.courseTo(gps.location.lat(), gps.location.lng(), buddyLat, buddyLon);
                    double buddyDist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), buddyLat, buddyLon);
                    
                    int relBuddy = (int)buddyBearing - (int)displayedHeading;
                    if (relBuddy < 0) relBuddy += 360;
                    
                    // Draw small hollow triangle for Buddy
                    float angleRad = (relBuddy - 90) * PI / 180.0;
                    int r = 25; // Slightly smaller radius
                    int cx = SCREEN_WIDTH/2;
                    int cy = arrowCy;
                    int x1 = cx + r * cos(angleRad);
                    int y1 = cy + r * sin(angleRad);
                    int x2 = cx + (r-8) * cos(angleRad + 0.5);
                    int y2 = cy + (r-8) * sin(angleRad + 0.5);
                    int x3 = cx + (r-8) * cos(angleRad - 0.5);
                    int y3 = cy + (r-8) * sin(angleRad - 0.5);
                    
                    u8g2.drawLine(x1, y1, x2, y2);
                    u8g2.drawLine(x2, y2, x3, y3);
                    u8g2.drawLine(x3, y3, x1, y1);
                    
                    // Show Buddy Distance (Top Left)
                    u8g2.setFont(u8g2_font_5x7_tr);
                    String bDistStr = "B:" + String((int)buddyDist) + "m";
                    if (millis() - lastBuddyPacketTime > 60000) bDistStr += "?";
                    u8g2.setCursor(0, 25);
                    u8g2.print(bDistStr);
                }
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
            drawMenuOverlay();
            u8g2.sendBuffer();
        }
    }
}