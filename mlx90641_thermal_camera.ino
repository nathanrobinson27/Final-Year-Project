#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <Adafruit_GFX.h>
#include <Adafruit_TFTLCD.h>

// ========== TFT Setup (8-bit mode) ==========
#define LCD_CS   A7
#define LCD_CD   A6
#define LCD_WR   A5
#define LCD_RD   A4
#define LCD_RESET -1

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

// ========== AMG8833 Setup ==========
Adafruit_AMG88xx amg;

// ========== Display Constants ==========
#define GRID_SIZE 8
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240
#define BLOCK_W (SCREEN_WIDTH / GRID_SIZE)
#define BLOCK_H (SCREEN_HEIGHT / GRID_SIZE)

float pixels[64]; // 8x8 thermal array

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize TFT
  tft.reset();
  delay(200);

  uint16_t ID = tft.readID();
  if (ID == 0x0101) ID = 0x9341; // common fallback
  tft.begin(ID);
  tft.setRotation(1); // landscape
  tft.fillScreen(0x0000);

  // Initialize AMG8833
  if (!amg.begin(0x68)) {
    Serial.println("AMG8833 not found. Check wiring!");
    while (1);
  }

  Serial.println("Thermal camera initialized.");
}

void loop() {
  amg.readPixels(pixels);

  float minTemp = pixels[0], maxTemp = pixels[0];

  // Find min/max temperature for scaling
  for (int i = 0; i < 64; i++) {
    if (pixels[i] < minTemp) minTemp = pixels[i];
    if (pixels[i] > maxTemp) maxTemp = pixels[i];
  }

  // Draw thermal grid
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      int i = y * GRID_SIZE + x;
      float temp = pixels[i];
      uint16_t color = tempToColor(temp, minTemp, maxTemp);

      int px = x * BLOCK_W;
      int py = y * BLOCK_H;

      tft.fillRect(px, py, BLOCK_W, BLOCK_H, color);
    }
  }

  delay(100); // ~10Hz refresh rate
}

// Convert temperature to color (simple gradient)
uint16_t tempToColor(float temp, float minT, float maxT) {
  float ratio = (temp - minT) / (maxT - minT);
  if (ratio < 0) ratio = 0;
  if (ratio > 1) ratio = 1;

  // Map from blue to red (HSV-like ramp)
  uint8_t r = ratio * 255;
  uint8_t g = 0;
  uint8_t b = (1 - ratio) * 255;

  return tft.color565(r, g, b);
}