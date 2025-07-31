#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

#define SD_CS_PIN 10
#define BUFFER_SIZE 100 
#define LINE_SIZE 100 

char** dataBuffer;
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);

  // Initialize BME280
  if (!bme.begin(0x76)) {
    Serial.println("Sensor not found!");
    while (1);
  }

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card failed!");
    while (1);
  }

  // Allocate outer buffer in PSRAM
  dataBuffer = (char**) ps_malloc(BUFFER_SIZE * sizeof(char*));
  if (!dataBuffer) {
    Serial.println("Failed to allocate dataBuffer in PSRAM!");
    while (1);
  }

  // Allocate each line in PSRAM
  for (int i = 0; i < BUFFER_SIZE; i++) {
    dataBuffer[i] = (char*) ps_malloc(LINE_SIZE);
    if (!dataBuffer[i]) {
      Serial.println("Failed to allocate line buffer in PSRAM!");
      while (1);
    }
  }

  Serial.println("Setup complete. Using PSRAM for buffering.");
}

void loop() {
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float pres = bme.readPressure() / 100.0F;

  snprintf(dataBuffer[bufferIndex], LINE_SIZE, "%lu,%.2f,%.2f,%.2f",
           millis(), temp, hum, pres);
  bufferIndex++;

  if (bufferIndex >= BUFFER_SIZE) {
    File file = SD.open("/weather.csv", FILE_APPEND);
    if (file) {
      for (int i = 0; i < BUFFER_SIZE; i++) {
        file.println(dataBuffer[i]);
      }
      file.close();
      Serial.println("Buffered data written to SD.");
    } else {
      Serial.println("Failed to write to SD.");
    }
    bufferIndex = 0;
  }

  delay(3000); 
}
