#include <Wire.h>
#define AIRSPEED_SENSOR_ADDRESS 0x28
const float P_min = -7.4726673;
const float P_max = 7.4726673;
const int D_OUT_min = 1638;
const int D_OUT_max = 14745;
const float P_ambient = 101325;
const float R_air = 287.05;
const int numSamples = 10;

float airspeedSamples[numSamples] = {0};
int sampleIndex = 0;
float calculateAverage(float newSample) {
  airspeedSamples[sampleIndex] = newSample;
  sampleIndex = (sampleIndex + 1) % numSamples;
  float sum = 0.0;
  for (int i = 0; i < numSamples; i++) {
    sum += airspeedSamples[i];
  }
  return sum / numSamples;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);
}
void loop() {
  Wire.beginTransmission(AIRSPEED_SENSOR_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(AIRSPEED_SENSOR_ADDRESS, 4);
  if (Wire.available() == 4) {
    uint8_t byte1 = Wire.read();
    uint8_t byte2 = Wire.read();
    uint8_t byte3 = Wire.read();
    uint8_t byte4 = Wire.read();
    
    uint16_t rawPressure = ((byte1 & 0x3F) << 8) | byte2;
    uint16_t rawTemperature = ((byte3 << 8) | byte4) >> 5;

    float P_range = P_max - P_min;
    float D_OUT_mid = (D_OUT_min + D_OUT_max) / 2.0;
    float diffPressure = (float)(rawPressure - D_OUT_mid) / (D_OUT_max - D_OUT_min) * P_range;
    float temperatureC = (rawTemperature * 200.0 / 2047.0) - 50.0;
    float temperatureK = temperatureC + 273.15;
    float airDensity = P_ambient / (R_air * temperatureK);
    float airspeed = 0.0;
    if (diffPressure > 0.001) {
      airspeed = sqrt(2 * (diffPressure * 1000) / airDensity);
      if (airspeed > 50.0) {
        airspeed = 50.0;
      }
    }
    airspeed = calculateAverage(airspeed);
    Serial.print("Differential Pressure: ");
    Serial.print(diffPressure, 3);
    Serial.println(" kPa");
    Serial.print("Airspeed: ");
    Serial.print(airspeed, 3);
    Serial.println(" m/s");
    Serial.print("Airspeed: ");
    Serial.print(airspeed*2.23694, 3);
    Serial.println(" mph");
    Serial.print("Temperature: ");
    Serial.print(temperatureC, 2);
    Serial.println(" °C");
    Serial.print("Air Density: ");
    Serial.print(airDensity, 3);
    Serial.println(" kg/m³");
    Serial.println();
  } else {
    Serial.println("Error: No data received from sensor");
  }
  delay(100);
}