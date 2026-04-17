#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
//#include <Adafruit_MLX90393.h>
#include <Adafruit_AMG88xx.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// Define TFT
#define TFT_CS   10
#define TFT_DC    9
#define TFT_RST   -1   // or -1 if tied to RESET/3.3V

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

Adafruit_LIS3MDL lis3mdl;
Adafruit_AMG88xx amg;
float pixels[64];

// ----------------------
// ANALOG SENSORS
// ----------------------
#define PIEZO_PIN A0
#define A1324_PIN A1

// ----------------------
// Fault Indicators
// ----------------------
#define LED1_PIN 6   // turns on when ANY fault occurs
#define LED2_PIN 7   // turns on when MORE THAN ONE fault occurs

// Thresholds (example values — adjust to your real limits)
#define PIEZO_RMS_THRESHOLD     1600.0
#define MAG_GAUSS_THRESHOLD     100.0
#define LIS_THRESHOLD           300.0
#define THERMAL_THRESHOLD       200   // byte value from 0–255

// ----------------------
// GLOBALS
// ----------------------
uint8_t thermalFrame[192];
int thermalIndex = 0;
bool receivingFrame = false;
float mlxFrame[192];


// ----------------------
// PIEZO PROCESSING
// ----------------------
/*
float readPiezo(float &rms, float &peak, float &crest) {
  unsigned long start = millis();
  float sumSq = 0;
  peak = 0;

  while (millis() - start < 1000) {
    int v = analogRead(PIEZO_PIN);
    float fv = (float)v;
    sumSq += fv * fv;
    if (fv > peak) peak = fv;
  }

  rms = sqrt(sumSq / 1000.0);
  crest = peak / rms;

}
*/

void readPiezo(float &rms, float &peak, float &crest) {
  unsigned long start = millis();
  float sumSq = 0;
  peak = 0;
  int n = 0;

  while (millis() - start < 1000) {
    int v = analogRead(PIEZO_PIN);
    float fv = (float)v;
    sumSq += fv * fv;
    if (fv > peak) peak = fv;
    n++;
  }

  if (n > 0) {
    rms = sqrt(sumSq / n);
    crest = peak / rms;
  } else {
    rms = 0;
    crest = 0;
  }
}

float readA1324() {
  int adc = analogRead(A1324_PIN);
  float voltage = (adc / 1023.0) * 5.0;   // Convert ADC to volts
  float gauss = (voltage - 2.5) / 0.005;  // Convert volts to gauss
  return gauss;
}

// ----------------------
// SETUP
// ----------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Wire.begin();
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  // LED setup
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);


  // --- LIS3MDL SETUP ---
  if (!lis3mdl.begin_I2C()) {
    Serial.println("Failed to find LIS3MDL!");
    while (1);
  }

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  
  /*
  // --- AMG8833 SETUP ---
  if (!amg.begin(0x68)) {
    Serial.println("Failed to find AMG8833!");
    while (1);
  }
  */
}

// ----------------------
// MAIN LOOP
// ----------------------
void loop() {

  // Checking serial
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    //Serial.println("0x");
    //Serial.print(b, HEX);
    //Serial.println(" ");
  }


  // PIEZO
  float piezoRMS, piezoPeak, piezoCrest;
  readPiezo(piezoRMS, piezoPeak, piezoCrest);

  // A1324
  float magAnalog = readA1324();

  // LIS3MDL
  float lisX, lisY, lisZ;
  readLIS3MDL(lisX, lisY, lisZ);

  // AMG8833
  //float tMin, tAvg, tMax;
  //readAMG8833Stats(tMin, tAvg, tMax);

  // Thermal information
  if (readThermalFrame()) {
    drawThermal();
  }


  // LED 
  int faults = checkFaultsAndUpdateLEDs(
                piezoRMS,
                magAnalog,
                lisX, lisY, lisZ,
                thermalFrame
             );



  // ----------------------
  // BUILD CSV PACKET
  // ----------------------
  String packet = "";
  packet += piezoRMS;  packet += ",";
  packet += piezoPeak; packet += ",";
  packet += piezoCrest; packet += ",";
  packet += magAnalog; packet += ",";
  packet += lisX; packet += ",";
  packet += lisY; packet += ",";
  packet += lisZ; 
  //packet += tMin; packet += ",";
  //packet += tAvg; packet += ",";
  //packet += tMax;

  Serial1.println(packet);
  Serial.println(packet);

  //delay(100);
}

void readLIS3MDL(float &x, float &y, float &z) {
  sensors_event_t event;
  lis3mdl.getEvent(&event);

  x = event.magnetic.x;  // microtesla
  y = event.magnetic.y;
  z = event.magnetic.z;
}

void readAMG8833Stats(float &tMin, float &tAvg, float &tMax) {
  amg.readPixels(pixels);

  tMin = 999;
  tMax = -999;
  float sum = 0;

  for (int i = 0; i < 64; i++) {
    float v = pixels[i];
    if (v < tMin) tMin = v;
    if (v > tMax) tMax = v;
    sum += v;
  }

  tAvg = sum / 64.0;
}


bool readThermalFrame() {
  static enum { WAIT_HEADER1, WAIT_HEADER2, READ_DATA, WAIT_FOOTER1, WAIT_FOOTER2 } state = WAIT_HEADER1;
  static int dataIndex = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    switch (state) {

      case WAIT_HEADER1:
        if (b == 0xAA) state = WAIT_HEADER2;
        break;

      case WAIT_HEADER2:
        if (b == 0x55) {
          state = READ_DATA;
          dataIndex = 0;
        } else {
          state = WAIT_HEADER1;
        }
        break;

      case READ_DATA:
        ((uint8_t*)mlxFrame)[dataIndex++] = b;
        if (dataIndex >= 192 * sizeof(float)) {
          state = WAIT_FOOTER1;
        }
        break;

      case WAIT_FOOTER1:
        if (b == 0x55) state = WAIT_FOOTER2;
        else state = WAIT_HEADER1;
        break;

      case WAIT_FOOTER2:
        if (b == 0xAA) {
          // FULL FRAME RECEIVED SUCCESSFULLY
          return true;
        }
        state = WAIT_HEADER1;
        break;
    }
  }

  return false;
}


void drawThermal() {
  int w = 16;
  int h = 12;
  int cellW = 20;
  int cellH = 20;

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      float t = mlxFrame[y * w + x];

      // Map temperature to 0–255
      float tMin = 20.0;
      float tMax = 40.0;
      uint8_t v = constrain(map(t, tMin, tMax, 0, 255), 0, 255);

      uint16_t color = tft.color565(v, 0, 255 - v);
      tft.fillRect(x * cellW, y * cellH, cellW, cellH, color);
    }
  }
}


int checkFaultsAndUpdateLEDs(float piezoRMS,
                             float magAnalog,
                             float lisX, float lisY, float lisZ,
                             uint8_t *thermalFrame) 
{
  int faultCount = 0;

  // --- PIEZO ---
  if (piezoRMS > PIEZO_RMS_THRESHOLD)
    faultCount++;

  // --- MAGNETIC ---
  if (abs(magAnalog) > MAG_GAUSS_THRESHOLD)
    faultCount++;

  // --- LIS3MDL ---
  if (abs(lisX) > LIS_THRESHOLD ||
      abs(lisY) > LIS_THRESHOLD ||
      abs(lisZ) > LIS_THRESHOLD)
    faultCount++;

  // --- THERMAL (max pixel) ---
  uint8_t maxThermal = 0;
  for (int i = 0; i < 192; i++) {
    if (thermalFrame[i] > maxThermal)
      maxThermal = thermalFrame[i];
  }
  if (maxThermal > THERMAL_THRESHOLD)
    faultCount++;

  // --- LED LOGIC (live, non‑latching) ---
  if (faultCount == 0) {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
  }
  else if (faultCount == 1) {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, LOW);
  }
  else { // 2 or more faults
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
  }

  return faultCount;
}

