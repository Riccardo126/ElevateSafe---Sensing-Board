#include <Wire.h>
#include <SparkFunLSM6DS3.h> // Example library
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_timer.h>
#include <freertos/stream_buffer.h>

// ========== DEBUG MODE CONFIGURATION ==========
// Set to true for debug messages, false for actual serial communication
bool DEBUG_MODE = false;

// Helper functions for debug/communication output
void debugPrint(const char* format, ...) {
  if (DEBUG_MODE) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.print(buffer);
  }
}

void debugPrintln(const char* message) {
  if (DEBUG_MODE) {
    Serial.println(message);
  }
}

// ==============================================

// Task Handles
TaskHandle_t SensorTaskHandle;
TaskHandle_t DisplayTaskHandle;
TaskHandle_t CommTaskHandle;

// StreamBuffer for efficient binary data transfer
StreamBufferHandle_t sensorStreamBuffer;

// Semaphore to trigger sampling at precise 1kHz
SemaphoreHandle_t samplingTrigger;

// Timer handle
esp_timer_handle_t samplingTimer;


#define SAMPLE_RATE_HZ 1000 //max 1600 for LSMDS3
#define SAMPLES_PER_BLOCK 50
#define TIMER_PERIOD_US (1000000 / SAMPLE_RATE_HZ)  // 1000 us = 1ms
#define STREAM_BUFFER_SIZE (SAMPLES_PER_BLOCK * sizeof(SensorData) * 2)

// Global variable to store latest Z value for display
volatile float latestAccelZ = 0.0;
// Ensures deterministic, jitter-free sampling at exactly 1000 Hz
SemaphoreHandle_t displayMutex;

// Define your new I2C pins
const int IMU_SDA_PIN = 6; //verde
const int IMU_SCL_PIN = 7; //arancione

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
  float accel[3];  // [0]=X, [1]=Y, [2]=Z
  int doorHall;
  int floorHall;
};

// Counter for block sequence
volatile uint8_t blockSequence = 0;

void scanI2C(TwoWire &bus, const char *busName) {
  debugPrint("I2C scan on %s...\n", busName);
  int count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      debugPrint("  - device at 0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) {
    debugPrintln("  - no devices found");
  }
}


// Hardware timer ISR callback - triggers SensorTask at 1kHz
void timerCallback(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(samplingTrigger, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// --- TASK 1: Sensor Reading (triggered by hardware timer) ---
void SensorTask(void *pvParameters) {
  SensorData currentData;

  for (;;) {
    // Wait for hardware timer to trigger (1kHz)
    if (xSemaphoreTake(samplingTrigger, portMAX_DELAY) == pdTRUE) {
      currentData.accel[0] = myIMU.readFloatAccelX();
      currentData.accel[1] = myIMU.readFloatAccelY();
      currentData.accel[2] = myIMU.readFloatAccelZ();
      currentData.doorHall = 0;  // Placeholder
      currentData.floorHall = 0;  // Placeholder

      // Update global variable for DisplayTask
      if (xSemaphoreTake(displayMutex, 0) == pdTRUE) {
        latestAccelZ = currentData.accel[2];
        xSemaphoreGive(displayMutex);
      }

      // Write to StreamBuffer (non-blocking)
      xStreamBufferSend(sensorStreamBuffer, &currentData, sizeof(SensorData), 0);
    }
  }
}


// --- TASK 2: Communication - sends data blocks ---
void vCommTask(void *pvParameters) {
  uint8_t blockBuffer[SAMPLES_PER_BLOCK * sizeof(SensorData)];
  size_t totalBytes = SAMPLES_PER_BLOCK * sizeof(SensorData);
  
  uint32_t blocksSent = 0;
  debugPrintln("[CommTask] Started");  // <-- ADD THIS
  
  for (;;) {
    // Block until we have a full block of 50 samples
    size_t receivedBytes = xStreamBufferReceive(
      sensorStreamBuffer, 
      blockBuffer, 
      totalBytes, 
      portMAX_DELAY  // Wait indefinitely for full block
    );
    
    debugPrint("[CommTask] Got %d bytes\n", receivedBytes);  // <-- ADD THIS
    
    if (receivedBytes == totalBytes) {
      if (DEBUG_MODE) {
        // Cast blockBuffer to SensorData array
        SensorData* dataBlock = (SensorData*)blockBuffer;
        
        // Calculate statistics for Z axis (accel[2])
        float minZ = dataBlock[0].accel[2];
        float maxZ = dataBlock[0].accel[2];
        float sumZ = 0.0;
        
        for (int i = 0; i < SAMPLES_PER_BLOCK; i++) {
          float z = dataBlock[i].accel[2];
          sumZ += z;
          if (z < minZ) minZ = z;
          if (z > maxZ) maxZ = z;
        }
        float avgZ = sumZ / SAMPLES_PER_BLOCK;
        
        // Display block statistics
        debugPrint("[CommTask] Block #%lu | Z: avg=%.3f min=%.3f max=%.3f g\n", 
                   millis(), avgZ, minZ, maxZ);
      } else {
        // Send block through serial with frame synchronization preamble (0xAA)
        Serial.write(0xAA); // Preamble byte
        Serial.write(blockBuffer, totalBytes);
        blocksSent++;
      }
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
  if(DEBUG_MODE) {
    Serial.begin(115200); 
  }
  else {
    Serial.begin(921600);
  }
  // Give the monitor a short window to attach after reset.
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart) < 3000) {
    delay(10);
  }
  debugPrintln("\n[BOOT] ElevateSafe start");
  delay(3000);

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
  Wire1.setClock(400000);  // Set for OLED
  scanI2C(Wire1, "Wire1(OLED)");

  // Keep static "Hello" text on-screen.
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false)) {
    debugPrintln("OLED init failed at 0x3C. Check I2C scan output.");
  } else {
    debugPrintln("OLED init OK");
  }
  
  // 1. Initialize the second I2C bus (Wire) with your custom pins

  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);  // Initialize directly with IMU pins
  Wire.setClock(400000);
  scanI2C(Wire, "Wire(IMU)");
  status_t imuStatus = myIMU.begin();
  debugPrint("IMU begin status: %d (0=IMU_SUCCESS, 1=IMU_ERROR)\n", imuStatus);
  if (imuStatus != 0) {
    debugPrintln("ERROR: IMU initialization failed!");
  }

  debugPrintln("[BOOT] Initializing FreeRTOS objects");

  // Create display mutex for thread-safe access to latestAccelZ
  displayMutex = xSemaphoreCreateMutex();
  if (displayMutex == NULL) {
    debugPrintln("ERROR: Display mutex creation failed!");
    while(1); // halt
  }

  // Create StreamBuffer for sensor data
  sensorStreamBuffer = xStreamBufferCreate(
    STREAM_BUFFER_SIZE,
    SAMPLES_PER_BLOCK * sizeof(SensorData)  // Trigger level: one full block
  );
  
  if (sensorStreamBuffer == NULL) {
    debugPrintln("ERROR: StreamBuffer creation failed!");
    while(1); // halt
  }

  // Create sampling trigger semaphore
  samplingTrigger = xSemaphoreCreateBinary();
  if (samplingTrigger == NULL) {
    debugPrintln("ERROR: Semaphore creation failed!");
    while(1); // halt
  }

  // Create and start hardware timer (1kHz sampling)
  esp_timer_create_args_t timerConfig = {
    .callback = timerCallback,
    .arg = NULL,
    .name = "SamplingTimer"
  };
  
  if (esp_timer_create(&timerConfig, &samplingTimer) != ESP_OK) {
    debugPrintln("ERROR: Timer creation failed!");
    while(1); // halt
  }
  
  if (esp_timer_start_periodic(samplingTimer, TIMER_PERIOD_US) != ESP_OK) {
    debugPrintln("ERROR: Timer start failed!");
    while(1); // halt
  }

  debugPrint("[BOOT] Hardware timer started: %lu Hz sampling\n", (long)SAMPLE_RATE_HZ);
  debugPrint("[BOOT] Sending blocks of %d samples (%.1f ms)\n", 
                SAMPLES_PER_BLOCK, 
                (float)SAMPLES_PER_BLOCK * 1000.0f / SAMPLE_RATE_HZ);

  debugPrintln("[BOOT] Creating FreeRTOS tasks...");

  // Create Tasks
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 3, &SensorTaskHandle, 1);
  xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 2048, NULL, 1, &DisplayTaskHandle, 0);
  xTaskCreate(vCommTask, "CommTask", 4096, NULL, 2, &CommTaskHandle);
}

void loop() {
  // Empty. FreeRTOS handles everything in tasks.
}