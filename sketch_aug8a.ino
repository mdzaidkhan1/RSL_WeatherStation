#include <Arduino.h>
#include <DFRobot_DHT20.h>
#include <Adafruit_BME280.h> //
#include <semphr.h>


#define BUFFER_SIZE 2        // Number of readings stored
#define READ_INTERVAL 3000    // in milliseconds (300 seconds = 5 minutes)
#define SEALEVELPRESSURE_HPA 1013.25

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

SemaphoreHandle_t mutex;

struct SensorData {
  float temperature;
  float humidity;
  float pressure;
  float altitude;
};

SensorData DataBuffer[BUFFER_SIZE];
int BufferIndex = 0;
bool bufferFullFlag = false; 


DFRobot_DHT20 dht20;
Adafruit_BME280 bme;


void Task1(void *pvParameters);
void Task2(void *pvParameters);
void Task3(void *pvParameters);

void setup() {
  Serial.begin(115200);

  // Init sensors
  dht20.begin();
  if (!bme.begin(0x77)) {
    Serial.println("Could not find BME280 sensor!");
  }
  }

  // Create Mutex
  mutex = xSemaphoreCreateMutex();
  if (mutex != NULL) {
    Serial.println("Mutex created");
  }

  // Create Tasks
  xTaskCreate(Task1, "Task1", 256, NULL, 1, NULL);
  xTaskCreate(Task2, "Task2", 128, NULL, 1, NULL);
  xTaskCreate(Task3, "Task3", 256, NULL, 1, NULL);
}

void Task1(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      if (BufferIndex < BUFFER_SIZE) {
        DataBuffer[BufferIndex].temperature = dht20.getTemperature();
        DataBuffer[BufferIndex].humidity = dht20.getHumidity() * 100;
        DataBuffer[BufferIndex].pressure = bme.readPressure() / 100.0F;
        DataBuffer[BufferIndex].altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

        BufferIndex++;
        Serial.print("Reading stored at index ");
        Serial.println(BufferIndex);
      }
      xSemaphoreGive(mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(READ_INTERVAL));
  }
}

void Task2(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      if (BufferIndex >= BUFFER_SIZE) {
        bufferFullFlag = true;
      }
      xSemaphoreGive(mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
    Serial.println("task2 done"); // Check every second
  }
}

void Task3(void *pvParameters) {
  for (;;) {
    if (bufferFullFlag) {
      if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        Serial.println("Buffer full! Printing data:");
        for (int i = 0; i < BUFFER_SIZE; i++) {
          Serial.print("Index ");
          Serial.print(i);
          Serial.print(": Temp = ");
          Serial.print(DataBuffer[i].temperature);
          Serial.print(" C, Humidity = ");
          Serial.print(DataBuffer[i].humidity);
          Serial.print(" %, Pressure = ");
          Serial.print(DataBuffer[i].pressure);
          Serial.print(" hPa, Altitude = ");
          Serial.print(DataBuffer[i].altitude);
          Serial.println(" m");
        }
        // Reset buffer
        BufferIndex = 0;
        bufferFullFlag = false;
        Serial.println("Buffer cleared.\n");
      }
      xSemaphoreGive(mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(500)); //delay
  }
}

void loop() {
}

///// hey zaid anytime i upload this code to the esp32 it uploads and verifies properly but i get nothing when its done like nothing from output in the serial monitor
//  i have been able to fix it but the Bme280 ia giving nan values for the pressure and altitiude i will try to get it to work today
