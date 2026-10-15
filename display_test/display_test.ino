#include <Arduino.h>
#include <Arduino_GFX_Library.h>

// =============================================================================
// ST7789 1.3-inch 240x240 Display Diagnostic (Arduino_GFX - Waveshare Engine)
// =============================================================================
#define LCD_DC_PIN      13   // P2 Pin 8  (DC)
#define LCD_CS_PIN      GFX_NOT_DEFINED // No CS pin on 7-pin module
#define LCD_SCLK_PIN    12   // P2 Pin 10 (SCL)
#define LCD_MOSI_PIN    11   // P2 Pin 9  (SDA)
#define LCD_RST_PIN     14   // P2 Pin 11 (RES)
#define LCD_BL_PIN      9    // P2 Pin 12 (BLK)

#define RED     RGB565_RED
#define GREEN   RGB565_GREEN
#define BLUE    RGB565_BLUE
#define BLACK   RGB565_BLACK
#define WHITE   RGB565_WHITE
#define CYAN    RGB565_CYAN
#define YELLOW  RGB565_YELLOW
#define MAGENTA RGB565_MAGENTA

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC_PIN, LCD_CS_PIN, LCD_SCLK_PIN, LCD_MOSI_PIN);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST_PIN, 0 /* rotation */, true /* IPS */, 240, 240);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- ST7789 ARDUINO_GFX TEST ---");

  // 1. Backlight ON
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);

  // 2. Hardware Reset pulse
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(50);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(100);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(150);

  // 3. Initialize GFX engine
  Serial.println("[*] Starting GFX display engine...");
  if (!gfx->begin()) {
    Serial.println("[!] ERROR: gfx->begin() failed!");
  } else {
    Serial.println("[+] GFX initialized successfully! Starting color cycle...");
  }
}

void loop() {
  Serial.println("-> Screen: RED");
  gfx->fillScreen(RED);
  delay(1000);

  Serial.println("-> Screen: GREEN");
  gfx->fillScreen(GREEN);
  delay(1000);

  Serial.println("-> Screen: BLUE");
  gfx->fillScreen(BLUE);
  delay(1000);

  Serial.println("-> Screen: WHITE + TEXT TEST");
  gfx->fillScreen(BLACK);
  gfx->drawRect(5, 5, 230, 230, GREEN);
  gfx->drawCircle(120, 120, 50, CYAN);
  gfx->setTextSize(2);
  gfx->setTextColor(WHITE);
  gfx->setCursor(30, 80);
  gfx->println("ST7789 240x240");
  gfx->setTextColor(YELLOW);
  gfx->setCursor(45, 110);
  gfx->println("DISPLAY OK!");
  gfx->setTextSize(1);
  gfx->setTextColor(MAGENTA);
  gfx->setCursor(35, 150);
  gfx->println("Waveshare ESP32-S3");
  delay(3000);
}
