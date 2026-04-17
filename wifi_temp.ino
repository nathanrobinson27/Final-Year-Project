#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>
#include <MLX90641.h>
#include <Wire.h>
#define EEPROM_WORDS 832

MLX90641 myIRcam;

const char* ssid     = "X";
const char* password = "X";

#define SD_CS   5
#define RX2_PIN 16
#define TX2_PIN 17
#define BAUD    115200

WebServer server(80);
File dataFile;

String serialLineBuffer = "";

// Simple HH:MM:SS from millis()
String getTimeString() {
  unsigned long ms = millis() / 1000;
  unsigned long s  = ms % 60;
  unsigned long m  = (ms / 60) % 60;
  unsigned long h  = (ms / 3600);

  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
  return String(buf);
}

void handleRoot() {
  String html;
  html += "<h1>ESP32 Data Logger</h1>";
  html += "<p>Logging to data.csv on SD card.</p>";
  html += "<a href=\"/download\">Download data.csv</a><br>";
  html += "<a href=\"/clear\">Clear data.csv</a>";
  server.send(200, "text/html", html);
}

void handleDownload() {
  File file = SD.open("/data.csv");
  if (!file) {
    server.send(500, "text/plain", "Failed to open data.csv");
    return;
  }
  server.streamFile(file, "text/csv");
  file.close();
}

void handleClear() {
  if (dataFile) {
    dataFile.close();
  }
  SD.remove("/data.csv");

  dataFile = SD.open("/data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println(
      "Time,"
      "Piezo_RMS,Piezo_Peak,Piezo_Crest,"
      "Magnetic_Gauss,"
      "LIS3MDL_X,LIS3MDL_Y,LIS3MDL_Z,"
      "Temp_Min,Temp_Avg,Temp_Max"
    );

    dataFile.flush();
    server.send(200, "text/plain", "data.csv cleared and header rewritten.");
  } else {
    server.send(500, "text/plain", "Failed to recreate data.csv");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Serial2 for Arduino Mega
  Serial2.begin(BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);

  // SD init
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card failed or not present");
    return;
  }
  Serial.println("SD card initialized.");

  // Reset file on every boot
  if (!SD.exists("/data.csv")) {
  dataFile = SD.open("/data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println(
      "Time,"
      "Piezo_RMS,Piezo_Peak,Piezo_Crest,"
      "Magnetic_Gauss,"
      "LIS3MDL_X,LIS3MDL_Y,LIS3MDL_Z,"
      "Temp_Min,Temp_Avg,Temp_Max"
    );

    dataFile.flush();
  }
} else {
  dataFile = SD.open("/data.csv", FILE_APPEND);
}

  Serial.println("Header written to data.csv");

  // WiFi
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin("X", "X");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Web server routes
  server.on("/", handleRoot);
  server.on("/download", handleDownload);
  server.on("/clear", handleClear);
  server.begin();

  // ---------- MLX90641 SETUP ----------
  Wire.begin(25, 26);
  Wire.setClock(400000);

  myIRcam.setRefreshRate(0x03);   // 4 Hz
  delay(600);                     // POR delay

  // Read EEPROM
  if (!myIRcam.readEEPROMBlock(0x2400, EEPROM_WORDS, myIRcam.eeData)) {
    Serial.println("EEPROM read failed!");
    while (1);
  }

  // Calibration reads
  myIRcam.Vdd = myIRcam.readVdd();
  myIRcam.Ta  = myIRcam.readTa();
  myIRcam.readPixelOffset();
  myIRcam.readAlpha();
  myIRcam.readKta();
  myIRcam.readKv();
  myIRcam.KsTa = myIRcam.readKsTa();
  myIRcam.readCT();
  myIRcam.readKsTo();
  myIRcam.readAlphaCorrRange();
  myIRcam.Emissivity = myIRcam.readEmissivity();
  myIRcam.alpha_CP = myIRcam.readAlpha_CP();
  myIRcam.pix_OS_ref_CP = myIRcam.readOff_CP();
  myIRcam.Kv_CP = myIRcam.readKv_CP();
  myIRcam.KTa_CP = myIRcam.readKTa_CP();
  myIRcam.TGC = myIRcam.readTGC();

  Serial.println("MLX90641 ready!");

}

void loop() {
  server.handleClient();

  float tMin, tMax, tAvg;
  readMLX90641(tMin, tMax, tAvg);
  sendMLXFrameToMega();


  // Read from Arduino (Serial2) line-by-line
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      serialLineBuffer.trim();
      if (serialLineBuffer.length() > 0) {
        String timeStr = getTimeString();
        String csvLine = timeStr + "," + serialLineBuffer + "," +
                 String(tMin) + "," + String(tAvg) + "," + String(tMax);
        if (dataFile) {
          dataFile.println(csvLine);
          dataFile.flush();  // 1 Hz is fine to flush each time
        }
        Serial.println(csvLine);  // Debug
      }
      serialLineBuffer = "";
    } else if (c != '\r') {
      serialLineBuffer += c;
    }
  }
  delay(1000);
}

void readMLX90641(float &tMin, float &tMax, float &tAvg) {
  // Wait for new data
  unsigned long start = millis();
  while (!myIRcam.isNewDataAvailable()) {
    if (millis() - start > 500) return;  // timeout
    delay(10);
  }

  myIRcam.clearNewDataBit();

  // Read all 192 temperatures
  myIRcam.readTempC();

  // Compute min, max, average
  tMin = 9999;
  tMax = -9999;
  tAvg = 0;

  for (int i = 0; i < 192; i++) {
    float t = myIRcam.T_o[i];
    if (t < tMin) tMin = t;
    if (t > tMax) tMax = t;
    tAvg += t;
  }

  tAvg /= 192.0;
}

void sendMLXFrameToMega() {
  const uint8_t START1 = 0xAA;
  const uint8_t START2 = 0x55;
  const uint8_t END1   = 0x55;
  const uint8_t END2   = 0xAA;

  // Send header
  Serial2.write(START1);
  Serial2.write(START2);

  // Send 192 floats (768 bytes)
  Serial2.write((uint8_t*)myIRcam.T_o, 192 * sizeof(float));

  // Send footer
  Serial2.write(END1);
  Serial2.write(END2);
}

