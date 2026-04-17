#include <Wire.h>
#include <DFRobot_ADS1115.h>

DFRobot_ADS1115 ads(&Wire);

// Sensitivity values from Alphasense datasheets
const float SENS_040 = 55.0;     // mV per ppm (PIDX-A-040)
const float SENS_10K = 0.55;     // mV per ppm (PIDX-A-10K)

// Offsets measured at startup (clean air)
float offset040 = 0;
float offset10K = 0;

void setup() {
  Serial.begin(9600);

  ads.setAddr_ADS1115(0x48);
  ads.setGain(eGAIN_TWOTHIRDS);
  ads.setMode(eMODE_SINGLE);
  ads.setRate(eRATE_128);
  ads.setOSMode(eOSMODE_SINGLE);
  ads.init();

  delay(2000);
  Serial.println("Measuring baseline offsets...");

  // Take samples to establish baseline
  long sum040 = 0;
  long sum10K = 0;

  for (int i = 0; i < 200; i++) {
    sum040 += ads.readVoltage(0);
    sum10K += ads.readVoltage(1);
    delay(50);
  }

  offset040 = sum040 / 200.0;
  offset10K = sum10K / 200.0;

  Serial.print("Offset PIDX-A-040: ");
  Serial.print(offset040);
  Serial.println(" mV");

  Serial.print("Offset PIDX-A-10K: ");
  Serial.print(offset10K);
  Serial.println(" mV");

  Serial.println("Ready.\n");
}

void loop() {
  if (ads.checkADS1115()) {

    float v040 = ads.readVoltage(0);
    float v10K = ads.readVoltage(1);

    // Convert to ppm
    float ppm040 = (v040 - offset040) / SENS_040;
    float ppm10K = (v10K - offset10K) / SENS_10K;

    // Prevent negative values
    if (ppm040 < 0) ppm040 = 0;
    if (ppm10K < 0) ppm10K = 0;

    Serial.print("PIDX-A-040 (0–40 ppm): ");
    Serial.print(ppm040, 2);
    Serial.print(" ppm   |   ");

    Serial.print("PIDX-A-10K (0–10,000 ppm): ");
    Serial.print(ppm10K, 2);
    Serial.println(" ppm");

  } else {
    Serial.println("ADS1115 Disconnected");
  }

  delay(100);
}
