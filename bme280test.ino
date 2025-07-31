#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // wait for serial
  Serial.println(F("BME280 test"));

  unsigned status = bme.begin();  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    while (1) delay(10);
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;
  Serial.println();
}

void loop() {
  printValues();
  delay(delayTime);
}

void printValues() {
  float rawTemp = bme.readTemperature();  // read raw temperature
  float correctedTemp = rawTemp;

  // Correction logic for Sensor C
  if (rawTemp < -10) {
    correctedTemp -= 1.65;
  } else if (rawTemp >= -10 && rawTemp < 10) {
    correctedTemp -= 4.09;
  } else if (rawTemp >= 10) {
    correctedTemp -= 5.4;
  }
  // Correction logic for Sensor B
  "  if (rawTemp < -10) {
    correctedTemp -= 1.65;
  } else if (rawTemp >= -10 && rawTemp < 10) {
    correctedTemp -= 3.60;
  } else if (rawTemp >= 10) {
    correctedTemp -= 5.16;
  }"

    // Correction logic for Sensor A
  "  if (rawTemp < -10) {
    correctedTemp -= 1.64;
  } else if (rawTemp >= -10 && rawTemp < 10) {
    correctedTemp -= 3.54;
  } else if (rawTemp >= 10) {
    correctedTemp -= 5.57;
  }"


  Serial.print("Raw Temperature = ");
  Serial.print(rawTemp);
  Serial.println(" °C");

  Serial.print("Corrected Temperature (Sensor C) = ");
  Serial.print(correctedTemp);
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  Serial.println();
}
