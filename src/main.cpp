#include <Wire.h>
#include <SparkFunLSM6DS3.h> // Example library
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// #include "ElevateSafe_TinyML.h" // Exported from Edge Impulse

// Task Handles
TaskHandle_t SensorTaskHandle;
TaskHandle_t InferenceTaskHandle;
TaskHandle_t DisplayTaskHandle;

// Data Queue
QueueHandle_t imuQueue;

// Global variable to store latest Z value for display
volatile float latestAccelZ = 0.0;
SemaphoreHandle_t displayMutex;

// Define your new I2C pins
const int IMU_SDA_PIN = 41; //verde
const int IMU_SCL_PIN = 42; //arancione

// Heltec WiFi LoRa 32 V3 onboard OLED pins
const int OLED_SDA_PIN = 17;
const int OLED_SCL_PIN = 18;
const int OLED_RST_PIN = 21;
const int OLED_VEXT_PIN = 36;

// Heltec WiFi LoRa 32 V3 onboard OLED (usually I2C address 0x3C)
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_RESET = -1;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

// Sensor Instances
LSM6DS3 myIMU(I2C_MODE, 0x6B);
const int HALL_DOOR_PIN = 1; 
const int HALL_FLOOR_PIN = 2;

// Struct for our sensor data
struct SensorData {
  float accelX, accelY, accelZ;
  int doorHall;
  int floorHall;
};

void scanI2C(TwoWire &bus, const char *busName) {
  Serial.printf("I2C scan on %s...\n", busName);
  int count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      Serial.printf("  - device at 0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) {
    Serial.println("  - no devices found");
  }
}

// --- TASK 1: Sensor Reading (100Hz) ---
void SensorTask(void *pvParameters) {
  SensorData currentData;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz
  uint8_t printDivider = 0;

  for(;;) {
    currentData.accelX = myIMU.readFloatAccelX();
    currentData.accelY = myIMU.readFloatAccelY();
    currentData.accelZ = myIMU.readFloatAccelZ();
    latestAccelZ = currentData.accelZ;  // Update global variable for display
    //currentData.doorHall = analogRead(HALL_DOOR_PIN);
    //currentData.floorHall = analogRead(HALL_FLOOR_PIN);

    // Send data to queue, do not block if full
    xQueueSend(imuQueue, &currentData, 0);

    // Print at 10Hz to avoid flooding serial output and timing jitter.
    if (++printDivider >= 10) {
      printDivider = 0;
      Serial.printf("[%10lu ms] IMU Accel [g] X: %.3f Y: %.3f Z: %.3f\n",
                    millis(),
                    currentData.accelX,
                    currentData.accelY,
                    currentData.accelZ);
    }

    // Ensure strict 100Hz timing
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// --- TASK 2: TinyML Inference ---
void InferenceTask(void *pvParameters) {
  SensorData receivedData;
  float mlBuffer[300]; // Example: 100 samples * 3 axes
  int bufferIndex = 0;

  for(;;) {
    // Wait for data from the sensor task
    if (xQueueReceive(imuQueue, &receivedData, portMAX_DELAY) == pdPASS) {
      
      /* Check simple thresholds first (Hall Sensors)
      if (receivedData.doorHall > 2000) {
        Serial.println("EVENT: Door Open Detected!");
      }

      // Fill buffer for ML
      mlBuffer[bufferIndex++] = receivedData.accelX;
      mlBuffer[bufferIndex++] = receivedData.accelY;
      mlBuffer[bufferIndex++] = receivedData.accelZ;

      // When buffer is full, run inference!
      if (bufferIndex >= 300) {
        bufferIndex = 0; // Reset
        
        // --- PSEUDOCODE FOR EDGE IMPULSE ---
        // signal_t signal;
        // int err = numpy::signal_from_buffer(mlBuffer, 300, &signal);
        // ei_impulse_result_t result = { 0 };
        // err = run_classifier(&signal, &result, false);
        // Serial.printf("Anomaly Score: %.3f\n", result.anomaly);
        
        // TODO: Pass results to a LoRa communication task*/

      
    }
  }
}

// --- TASK 3: Display Update (2Hz) ---
void DisplayTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500); // 2Hz to reduce flicker

  for(;;) {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      float zValue = latestAccelZ;
      xSemaphoreGive(displayMutex);

      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Z Accel:");
      display.setTextSize(1);
      display.setCursor(0, 25);
      display.print(zValue, 3);  // Print with 3 decimal places
      display.println(" g");
      display.display();
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(115200);
  // Give the monitor a short window to attach after reset.
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart) < 3000) {
    delay(10);
  }
  Serial.println("\n[BOOT] ElevateSafe start");
  delay(300);

  // Heltec V3 powers OLED through Vext (active LOW).
  pinMode(OLED_VEXT_PIN, OUTPUT);
  digitalWrite(OLED_VEXT_PIN, LOW);

  // Hardware reset line for onboard OLED.
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, LOW);
  delay(20);
  digitalWrite(OLED_RST_PIN, HIGH);
  delay(20);

  // Initialize I2C on dedicated OLED pins and scan for devices.
  Wire1.begin(OLED_SDA_PIN, OLED_SCL_PIN);  // 17, 18 for OLED
  Wire1.setClock(400000);
  scanI2C(Wire1, "Wire1(OLED)");

  // Keep static "Hello" text on-screen.
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false)) {
    Serial.println("OLED init failed at 0x3C. Check I2C scan output.");
  } else {
    Serial.println("OLED init OK");
  }
  
  // 1. Initialize the second I2C bus (Wire) with your custom pins
  Wire.begin();  // default pins
  scanI2C(Wire, "Wire(default pins)");
  Wire.~TwoWire();  // release it

  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);  // 41, 42 for IMU
  status_t imuStatus = myIMU.begin();
  Serial.printf("IMU begin status: %d (0=IMU_SUCCESS, 1=IMU_ERROR)\n", imuStatus);

  scanI2C(Wire, "Wire(IMU)");

  Serial.println("[BOOT] Tasks starting");

  // Create a queue to hold 100 samples
  imuQueue = xQueueCreate(100, sizeof(SensorData));

  // Create display mutex for thread-safe access
  displayMutex = xSemaphoreCreateMutex();

  // Create Tasks
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 3, &SensorTaskHandle, 1);
  xTaskCreatePinnedToCore(InferenceTask, "InferenceTask", 8192, NULL, 2, &InferenceTaskHandle, 0);
  xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 2048, NULL, 1, &DisplayTaskHandle, 0);
}

void loop() {
  // Empty. FreeRTOS handles everything in tasks.
}