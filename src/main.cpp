// ESP32-C6-Touch-LCD-1.47 - Bring Em Home Project
// Using Arduino_GFX library with ST7789 driver

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>

// RGB565 Color definitions
#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define YELLOW  0xFFE0
#define CYAN    0x07FF
#define MAGENTA 0xF81F

// ESP32-C6-Touch-LCD-1.47 Pin Configuration
// Source: https://community.home-assistant.io/t/help-needed-to-get-this-tiny-screen-working-esp32-c6-1-47in-display/800102/26
#define TFT_SCK   7
#define TFT_MOSI  6
#define TFT_DC    15
#define TFT_CS    14
#define TFT_RST   21
#define TFT_BL    22

// Display dimensions
#define SCREEN_WIDTH  172
#define SCREEN_HEIGHT 320

// Create display bus and display object
Arduino_DataBus *bus = new Arduino_ESP32SPI(
    TFT_DC,   // DC
    TFT_CS,   // CS
    TFT_SCK,  // SCK
    TFT_MOSI, // MOSI
    -1        // MISO (not used)
);

// ST7789 with IPS=true (working config!)
Arduino_GFX *gfx = new Arduino_ST7789(
    bus,
    TFT_RST,      // RST
    0,            // rotation (0 = portrait)
    true,         // IPS panel = true (THIS WORKED!)
    SCREEN_WIDTH, // width = 172
    SCREEN_HEIGHT,// height = 320
    34,           // col_offset = 34 (Centering for 172x320 in 240x320 driver)
    0             // row_offset = 0
);

void setup() {
    Serial.begin(115200);
    // Wait for Serial to be ready (optional, but good for debugging)
    // while(!Serial) delay(100); 
    delay(2000);
    Serial.println("\n\n=== ESP32-C6 Bring Em Home ===");

    // I2C Scan to check for Touch Controller
    Wire.begin(1, 2); // SDA=1, SCL=2
    Serial.println("Scanning I2C bus...");
    int nDevices = 0;
    for(byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");
            nDevices++;
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
    
    // Backlight Control Test
    pinMode(TFT_BL, OUTPUT);
    Serial.println("Blinking Backlight...");
    for(int i=0; i<3; i++) {
        digitalWrite(TFT_BL, HIGH); // ON
        delay(200);
        digitalWrite(TFT_BL, LOW);  // OFF
        delay(200);
    }
    digitalWrite(TFT_BL, HIGH); // Keep ON
    Serial.println("Backlight ON");
    
    // Initialize display
    Serial.println("Initializing display...");
    if (!gfx->begin()) {
        Serial.println("ERROR: gfx->begin() failed!");
        while(1) { delay(1000); }
    }
    Serial.println("Display initialized OK");
    
    // Clear screen to black
    gfx->fillScreen(BLACK);
    Serial.println("Screen cleared to BLACK");
    
    // Turn on backlight
    digitalWrite(TFT_BL, HIGH);
    Serial.println("Backlight ON");
    
    delay(500);
    
    // Draw test pattern
    Serial.println("Drawing test pattern...");
    
    // Red rectangle top
    gfx->fillRect(0, 0, SCREEN_WIDTH, 80, RED);
    
    // Green rectangle middle  
    gfx->fillRect(0, 80, SCREEN_WIDTH, 80, GREEN);
    
    // Blue rectangle bottom
    gfx->fillRect(0, 160, SCREEN_WIDTH, 80, BLUE);
    
    // White rectangle at very bottom
    gfx->fillRect(0, 240, SCREEN_WIDTH, 80, WHITE);
    
    // Draw some text
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    gfx->setCursor(10, 30);
    gfx->print("ESP32-C6");
    
    gfx->setTextColor(BLACK);
    gfx->setCursor(10, 110);
    gfx->print("Bring Em");
    
    gfx->setTextColor(WHITE);
    gfx->setCursor(10, 190);
    gfx->print("Home!");
    
    gfx->setTextColor(BLACK);
    gfx->setCursor(10, 270);
    gfx->print("Working!");
    
    Serial.println("=== Setup complete! ===");
}

void loop() {
    // Color cycling demo
    static uint32_t lastChange = 0;
    static int colorIndex = 0;
    
    if (millis() - lastChange > 3000) {
        lastChange = millis();
        colorIndex = (colorIndex + 1) % 4;
        
        uint16_t color;
        const char* colorName;
        
        switch(colorIndex) {
            case 0: color = RED;    colorName = "RED";    break;
            case 1: color = GREEN;  colorName = "GREEN";  break;
            case 2: color = BLUE;   colorName = "BLUE";   break;
            default: color = WHITE; colorName = "WHITE";  break;
        }
        
        gfx->fillScreen(color);
        Serial.printf("Screen color: %s\n", colorName);
        
        // Draw color name
        gfx->setTextSize(3);
        gfx->setTextColor(color == WHITE ? BLACK : WHITE);
        gfx->setCursor(20, 140);
        gfx->print(colorName);
    }
}