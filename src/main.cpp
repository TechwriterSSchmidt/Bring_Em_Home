/**
 * Bring Em Home - GPS Navigation Device
 * 
 * A device to guide Emilie back to her starting location when lost on hikes.
 * 
 * Hardware:
 * - ESP32-S3 with Waveshare 1.47" Touch LCD Display (JD9853 / AXS5106L)
 * - HGLRC M100-5883 M10 GPS Module with Compass (HMC5883L)
 * 
 * Features:
 * - Save home position with button press
 * - Display current GPS coordinates
 * - Calculate and display distance/bearing to home
 * - Visual compass display
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Arduino_GFX_Library.h>
#include <Adafruit_HMC5883_U.h>
#include <Preferences.h>

// Pin definitions for Waveshare ESP32-S3 1.47" Touch LCD
#define TFT_CS     10
#define TFT_RST    14
#define TFT_DC     13
#define TFT_MOSI   11
#define TFT_SCLK   12
#define TFT_BLK    38

// GPS Serial (UART1)
#define GPS_RX     18
#define GPS_TX     17

// I2C for Compass (HMC5883L)
#define I2C_SDA    8
#define I2C_SCL    9

// Button pins
#define BTN_SAVE   0   // Boot button to save home position
#define BTN_MODE   1   // Mode button

// Display settings
#define SCREEN_WIDTH  172
#define SCREEN_HEIGHT 320

// Compass arrow display position
#define COMPASS_CENTER_X  280
#define COMPASS_CENTER_Y  100
#define COMPASS_RADIUS    30

// Create objects
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI);
// Waveshare 1.47" uses ST7789 driver with specific offsets and resolution
// Note: If using the Waveshare ESP32-S3-Touch-LCD-1.47 board, check if pins match: DC=45, CS=21, SCK=38, MOSI=39, RST=47
Arduino_GFX *tft = new Arduino_ST7789(bus, TFT_RST, 0 /* rotation */, false /* IPS */, SCREEN_WIDTH, SCREEN_HEIGHT, 34 /* col_offset1 */, 0 /* row_offset1 */, 34 /* col_offset2 */, 0 /* row_offset2 */);

TinyGPSPlus gps;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Preferences preferences;

// GPS Serial
HardwareSerial gpsSerial(1);

// Home position
double homeLat = 0.0;
double homeLon = 0.0;
bool homeSet = false;

// Current position
double currentLat = 0.0;
double currentLon = 0.0;
bool gpsValid = false;

// Compass
float heading = 0.0;
float bearingToHome = 0.0;
float distanceToHome = 0.0;

// Display update timing
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 500; // Update every 500ms

// Button state
bool lastBtnSaveState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// Home saved feedback state
bool showHomeSavedFeedback = false;
unsigned long homeSavedFeedbackTime = 0;
const unsigned long HOME_SAVED_FEEDBACK_DURATION = 1000; // 1 second

void setup() {
  Serial.begin(115200);
  Serial.println("Bring Em Home - Starting...");

  // Initialize buttons
  pinMode(BTN_SAVE, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);
  
  // Initialize backlight
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH); // Turn on backlight

  // Initialize display
  tft->begin();
  tft->setRotation(1); // Landscape
  tft->fillScreen(BLACK);
  tft->setTextColor(WHITE);
  tft->setTextSize(1);
  
  tft->setCursor(10, 10);
  tft->println("Bring Em Home");
  tft->println("Initializing...");

  // Initialize I2C for compass
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Initialize compass
  if (mag.begin()) {
    Serial.println("HMC5883L detected!");
    tft->println("Compass: OK");
  } else {
    Serial.println("No HMC5883L detected");
    tft->println("Compass: FAIL");
  }

  // Initialize GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  tft->println("GPS: Starting...");
  Serial.println("GPS initialized");

  // Load saved home position
  preferences.begin("bring-em-home", false);
  homeLat = preferences.getDouble("homeLat", 0.0);
  homeLon = preferences.getDouble("homeLon", 0.0);
  
  if (homeLat != 0.0 || homeLon != 0.0) {
    homeSet = true;
    Serial.printf("Loaded home: %.6f, %.6f\n", homeLat, homeLon);
    tft->println("Home pos loaded");
  }
  preferences.end();

  delay(2000);
  tft->fillScreen(BLACK);
}

void saveHomePosition() {
  if (gpsValid) {
    homeLat = currentLat;
    homeLon = currentLon;
    homeSet = true;
    
    preferences.begin("bring-em-home", false);
    preferences.putDouble("homeLat", homeLat);
    preferences.putDouble("homeLon", homeLon);
    preferences.end();
    
    Serial.printf("Home saved: %.6f, %.6f\n", homeLat, homeLon);
    
    // Trigger non-blocking visual feedback
    showHomeSavedFeedback = true;
    homeSavedFeedbackTime = millis();
  }
}

void readGPS() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
    
    if (gps.location.isUpdated()) {
      currentLat = gps.location.lat();
      currentLon = gps.location.lng();
      gpsValid = true;
    }
  }
}

void readCompass() {
  sensors_event_t event;
  mag.getEvent(&event);
  
  // Calculate heading
  heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Convert to degrees
  heading = heading * 180.0 / PI;
  
  // Normalize to 0-360
  if (heading < 0) {
    heading += 360.0;
  }
}

void calculateNavigation() {
  if (!gpsValid || !homeSet) {
    return;
  }
  
  // Calculate distance using Haversine formula
  double dLat = (homeLat - currentLat) * PI / 180.0;
  double dLon = (homeLon - currentLon) * PI / 180.0;
  
  double a = sin(dLat/2) * sin(dLat/2) +
             cos(currentLat * PI / 180.0) * cos(homeLat * PI / 180.0) *
             sin(dLon/2) * sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  distanceToHome = 6371000 * c; // Earth radius in meters
  
  // Calculate bearing
  double y = sin(dLon) * cos(homeLat * PI / 180.0);
  double x = cos(currentLat * PI / 180.0) * sin(homeLat * PI / 180.0) -
             sin(currentLat * PI / 180.0) * cos(homeLat * PI / 180.0) * cos(dLon);
  bearingToHome = atan2(y, x) * 180.0 / PI;
  
  // Normalize to 0-360
  if (bearingToHome < 0) {
    bearingToHome += 360.0;
  }
}

void drawCompassArrow(int centerX, int centerY, int radius, float angle, uint16_t color) {
  float rad = (angle - 90) * PI / 180.0; // Adjust so 0° is up
  
  // Arrow tip
  int x1 = centerX + radius * cos(rad);
  int y1 = centerY + radius * sin(rad);
  
  // Arrow base left
  float rad2 = (angle - 90 - 140) * PI / 180.0;
  int x2 = centerX + (radius * 0.5) * cos(rad2);
  int y2 = centerY + (radius * 0.5) * sin(rad2);
  
  // Arrow base right
  float rad3 = (angle - 90 + 140) * PI / 180.0;
  int x3 = centerX + (radius * 0.5) * cos(rad3);
  int y3 = centerY + (radius * 0.5) * sin(rad3);
  
  tft->fillTriangle(x1, y1, x2, y2, x3, y3, color);
  tft->drawCircle(centerX, centerY, radius + 2, WHITE);
}

void updateDisplay() {
  unsigned long currentTime = millis();
  
  // Handle home saved feedback (non-blocking)
  if (showHomeSavedFeedback) {
    if (currentTime - homeSavedFeedbackTime < HOME_SAVED_FEEDBACK_DURATION) {
      // Show green feedback screen
      tft->fillScreen(GREEN);
      tft->setTextColor(BLACK);
      tft->setCursor(40, SCREEN_HEIGHT/2 - 10);
      tft->setTextSize(2);
      tft->println("HOME SAVED!");
      return; // Skip normal display update
    } else {
      // Feedback period ended
      showHomeSavedFeedback = false;
      tft->fillScreen(BLACK);
    }
  }
  
  if (currentTime - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL) {
    return;
  }
  lastDisplayUpdate = currentTime;
  
  tft->fillScreen(BLACK);
  tft->setTextSize(1);
  tft->setTextColor(WHITE);
  
  // Title
  tft->setCursor(5, 5);
  tft->setTextColor(YELLOW);
  tft->setTextSize(2);
  tft->println("Bring Em Home");
  
  // GPS Status
  tft->setCursor(5, 30);
  tft->setTextSize(1);
  tft->setTextColor(WHITE);
  tft->print("GPS: ");
  if (gpsValid) {
    tft->setTextColor(GREEN);
    tft->print("LOCKED");
    tft->setTextColor(WHITE);
    tft->print(" Sats: ");
    tft->print(gps.satellites.value());
  } else {
    tft->setTextColor(RED);
    tft->println("SEARCHING...");
  }
  
  // Current position
  tft->setCursor(5, 45);
  tft->setTextColor(CYAN);
  tft->print("Lat: ");
  tft->println(currentLat, 6);
  tft->setCursor(5, 55);
  tft->print("Lon: ");
  tft->println(currentLon, 6);
  
  // Compass heading
  tft->setCursor(5, 70);
  tft->setTextColor(MAGENTA);
  tft->print("Heading: ");
  tft->print((int)heading);
  tft->println("°");
  
  // Home position and navigation
  if (homeSet) {
    tft->setCursor(5, 90);
    tft->setTextColor(YELLOW);
    tft->println("HOME POSITION SET");
    
    tft->setCursor(5, 105);
    tft->setTextColor(CYAN);
    tft->print("Home: ");
    tft->print(homeLat, 6);
    tft->setCursor(5, 115);
    tft->print("      ");
    tft->print(homeLon, 6);
    
    if (gpsValid) {
      // Distance
      tft->setCursor(5, 130);
      tft->setTextColor(GREEN);
      tft->print("Distance: ");
      if (distanceToHome < 1000) {
        tft->print((int)distanceToHome);
        tft->println(" m");
      } else {
        tft->print(distanceToHome / 1000.0, 2);
        tft->println(" km");
      }
      
      // Bearing
      tft->setCursor(5, 145);
      tft->print("Bearing: ");
      tft->print((int)bearingToHome);
      tft->println("°");
      
      // Compass arrow showing direction to home
      float relativeAngle = bearingToHome - heading;
      // Normalize angle to -180 to +180 range
      while (relativeAngle > 180.0) relativeAngle -= 360.0;
      while (relativeAngle < -180.0) relativeAngle += 360.0;
      drawCompassArrow(COMPASS_CENTER_X, COMPASS_CENTER_Y, COMPASS_RADIUS, relativeAngle, GREEN);
      
      tft->setCursor(240, 140);
      tft->setTextSize(1);
      tft->println("Home");
    }
  } else {
    tft->setCursor(5, 90);
    tft->setTextColor(ORANGE);
    tft->println("NO HOME SET");
    tft->setCursor(5, 105);
    tft->setTextSize(1);
    tft->println("Press BOOT button");
    tft->setCursor(5, 115);
    tft->println("to save home pos");
  }
  
  // Instructions at bottom
  tft->setCursor(5, 160);
  tft->setTextColor(WHITE);
  tft->setTextSize(1);
  tft->println("BOOT: Save home position");
}

void loop() {
  // Read GPS data
  readGPS();
  
  // Read compass
  readCompass();
  
  // Calculate navigation
  calculateNavigation();
  
  // Handle button press for saving home position
  bool btnSaveReading = digitalRead(BTN_SAVE);
  
  if (btnSaveReading != lastBtnSaveState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (btnSaveReading == LOW && lastBtnSaveState == HIGH) {
      saveHomePosition();
    }
  }
  
  lastBtnSaveState = btnSaveReading;
  
  // Update display
  updateDisplay();
  
  // Debug output
  if (gpsValid) {
    Serial.printf("GPS: %.6f, %.6f | Heading: %.1f° | ", 
                  currentLat, currentLon, heading);
    if (homeSet) {
      Serial.printf("Distance: %.1fm | Bearing: %.1f°\n", 
                    distanceToHome, bearingToHome);
    } else {
      Serial.println("No home set");
    }
  }
  
  delay(100);
}
