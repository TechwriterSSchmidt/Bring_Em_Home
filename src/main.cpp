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
#include "esp_lcd_touch_axs5106l.h" // Include Touch Driver

// Color Definitions
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define ORANGE  0xFD20

// Pin definitions for Waveshare ESP32-S3 1.47" Touch LCD
#define TFT_DC     45
#define TFT_CS     21
#define TFT_SCLK   38
#define TFT_MOSI   39
#define TFT_BLK    46

// Confusing Reset Pin Logic from Demo 01_gfx_helloworld
#define REAL_LCD_RST 40  // Used for manual toggle
#define DRIVER_RST   47  // Passed to constructor

// Touch Pins
#define TOUCH_RST  47
#define TOUCH_INT  48
#define TOUCH_SDA  42
#define TOUCH_SCL  41

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
#define ROTATION 1 // Landscape mode

// Compass arrow display position
#define COMPASS_CENTER_X  280
#define COMPASS_CENTER_Y  100
#define COMPASS_RADIUS    30

// Create objects
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI);

// NOTE: Passing DRIVER_RST (47) to constructor, but we will toggle REAL_LCD_RST (40) manually!
Arduino_GFX *tft = new Arduino_ST7789(bus, DRIVER_RST, ROTATION /* rotation */, false /* IPS */, SCREEN_WIDTH, SCREEN_HEIGHT, 34 /* col_offset1 */, 0 /* row_offset1 */, 34 /* col_offset2 */, 0 /* row_offset2 */);

// Touch Data
touch_data_t touch_data;
unsigned long lastTouchTime = 0;
bool touchFound = false;


// Initialization sequence for JD9853 / ST7789 on Waveshare 1.47"
void lcd_reg_init(void) {
  Serial.println("Sending LCD init sequence...");
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
  Serial.println("LCD init sequence sent.");
}

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
bool compassFound = false;

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

void loadHomePosition(); // Forward declaration
void saveHomePosition(); // Forward declaration

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
  // Manual reset as per Waveshare example 01_gfx_helloworld
  pinMode(REAL_LCD_RST, OUTPUT);
  digitalWrite(REAL_LCD_RST, LOW);
  delay(10);
  digitalWrite(REAL_LCD_RST, HIGH);
  delay(10);

  if (!tft->begin()) {
    Serial.println("tft->begin() failed!");
  }
  
  lcd_reg_init(); // Custom init for JD9853/ST7789 on Waveshare 1.47"
  
  tft->setRotation(ROTATION); // Landscape
  tft->fillScreen(RED); // RED for visibility
  tft->setTextColor(WHITE);
  tft->setTextSize(1);
  
  tft->setCursor(10, 10);
  tft->println("Bring Em Home");
  tft->println("Initializing...");

  // I2C Scanner
  Serial.println("Scanning I2C (Wire)...");
  Wire.end(); // Ensure clean state
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
  Wire.setClock(100000); // Force 100kHz standard speed
  
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
    }
  }

  // Temporarily disabled Touch Init to isolate Display
  /*
  Serial.println("Scanning I2C (Wire1)...");
  Wire1.begin(TOUCH_SDA, TOUCH_SCL);
  for (byte i = 1; i < 127; i++) {
    Wire1.beginTransmission(i);
    if (Wire1.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
      if (i == AXS5106L_ADDR) {
        touchFound = true;
        Serial.println("Touch controller found!");
      }
    }
  }
  
  // Initialize Touch
  if (touchFound) {
    bsp_touch_init(&Wire1, TOUCH_RST, TOUCH_INT, ROTATION, SCREEN_WIDTH, SCREEN_HEIGHT);
    tft->println("Touch: OK");
  } else {
    Serial.println("Touch controller NOT found");
    tft->println("Touch: FAIL");
  }
  */
  
  // Initialize compass
  // Wire.begin(I2C_SDA, I2C_SCL); // Already called above
  if (mag.begin()) {
    Wire.setClock(100000); // Ensure clock is still 100kHz after init
    Serial.println("HMC5883L detected!");
    tft->println("Compass: OK");
    compassFound = true;
  } else {
    Serial.println("No HMC5883L detected");
    tft->println("Compass: FAIL");
    compassFound = false;
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
  if (!compassFound) return;

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
  // Read Touch
  if (touchFound) {
    bsp_touch_read();
    if (bsp_touch_get_coordinates(&touch_data)) {
        Serial.printf("Touch: x=%d, y=%d\n", touch_data.coords[0].x, touch_data.coords[0].y);
        
        // Simple Touch Action: Save Home if touched in center
        // Coordinates are relative to screen rotation
        if (touch_data.coords[0].x > 50 && touch_data.coords[0].x < 270 && 
            touch_data.coords[0].y > 50 && touch_data.coords[0].y < 120) {
            if (millis() - lastTouchTime > 1000) {
              saveHomePosition();
              lastTouchTime = millis();
            }
        }
    }
  }

  // Read GPS
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
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 1000) {
    lastDebugTime = millis();
    Serial.print("Heartbeat... ");
    if (gpsValid) {
      Serial.printf("GPS: %.6f, %.6f | Heading: %.1f° | ", 
                    currentLat, currentLon, heading);
      if (homeSet) {
        Serial.printf("Distance: %.1fm | Bearing: %.1f°\n", 
                      distanceToHome, bearingToHome);
      } else {
        Serial.println("No home set");
      }
    } else {
      Serial.println("Waiting for GPS fix...");
    }
  }
  
  delay(10); // Reduced delay for smoother touch response
}
