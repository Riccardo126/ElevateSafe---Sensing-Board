#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <freertos/stream_buffer.h>
#include "display_task.h"
#include "comm_task.h"
#include "sensor_task.h"
#include "filter_task.h"
#include "shared.h"
#include "config.h"

// #include "ElevateSafe_TinyML.h" // Exported from Edge Impulse

// ========== DEBUG MODE CONFIGURATION ==========
// Set to true for debug messages, false for actual serial communication
bool DEBUG_MODE = true;

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
TaskHandle_t FilterTaskHandle;
TaskHandle_t DisplayTaskHandle;
TaskHandle_t CommTaskHandle;

// StreamBuffer for efficient binary data transfer
StreamBufferHandle_t sensorStreamBuffer;
StreamBufferHandle_t filteredSensorStreamBuffer;

// Semaphore to trigger sampling at precise 1kHz
SemaphoreHandle_t samplingTrigger;

// Timer handle
esp_timer_handle_t samplingTimer;

// Global variable to store latest Z value for display
volatile float latestAccelZ = 0.0;
SemaphoreHandle_t displayMutex;

// Heltec WiFi LoRa 32 V3 onboard OLED pins
const int OLED_SDA_PIN = 17;
const int OLED_SCL_PIN = 18;
const int OLED_RST_PIN = 21;
const int OLED_VEXT_PIN = 36;

// Heltec WiFi LoRa 32 V3 onboard OLED (usually I2C address 0x3C)
// OLED — solo su Heltec
#ifdef HAS_OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  const int SCREEN_WIDTH = 128;
  const int SCREEN_HEIGHT = 64;
  const int OLED_RESET = -1;
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
#endif

// Sensor Instances
// IMU — libreria diversa per board diversa
#ifdef HAS_LSM6DS3
  #include <SparkFunLSM6DS3.h>
  LSM6DS3 myIMU(I2C_MODE, 0x6B);
#endif

#ifdef HAS_MPU6050
  #include <Adafruit_MPU6050.h>
  #include <Adafruit_Sensor.h>
  Adafruit_MPU6050 myIMU;
#endif

const int HALL_DOOR_PIN = 1;
const int HALL_FLOOR_PIN = 2;

// Calibration offsets for IMU (auto-calibrated on startup)
float CALIB_X = 0.024, CALIB_Y = 0.08, CALIB_Z = 9.8;

// Block header for synchronization - defined in shared.h

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



void setup() {
  if(DEBUG_MODE) {
    Serial.begin(115200); 
  }
  else {
    Serial.begin(2000000);
  }
  // Give the monitor a short window to attach after reset.
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart) < 3000) {
    delay(10);
  }
  debugPrintln("\n[BOOT] ElevateSafe start");
  delay(3000);

  // Heltec V3 powers OLED through Vext (active LOW).
  #ifdef HAS_OLED
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
    Wire1.setClock(100000);  // Set to 100kHz
    scanI2C(Wire1, "Wire1(OLED)");

    // Keep static "Hello" text on-screen.
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false)) {
      debugPrintln("OLED init failed at 0x3C. Check I2C scan output.");
    } else {
      debugPrintln("OLED init OK");
    }
  #endif
  // 1. Initialize the second I2C bus (Wire) with your custom pins

  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);  // Initialize directly with IMU pins
  pinMode(IMU_SDA_PIN, INPUT_PULLUP);  // Enable internal pullup
  pinMode(IMU_SCL_PIN, INPUT_PULLUP);  // Enable internal pullup
  Wire.setClock(100000);  // Set to 100kHz
  delay(100);  // Wait for sensor to power up
  scanI2C(Wire, "Wire(IMU)");
   

  #ifdef HAS_LSM6DS3
    debugPrintln("Initializing LSM6DS3...");
    int imuStatus = myIMU.begin();
  #endif
  #ifdef HAS_MPU6050
    debugPrintln("Initializing MPU6050...");
    bool imuInit = myIMU.begin();
    int imuStatus = imuInit ? 0 : 1;
    if (imuInit) myIMU.setAccelerometerRange(MPU6050_RANGE_2_G);
  #endif  

  debugPrint("IMU begin status: %d (0=IMU_SUCCESS, 1=IMU_ERROR)\n", imuStatus);
  if (imuStatus != 0) {
    debugPrintln("ERROR: IMU initialization failed!");
    while(1); // halt
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
  filteredSensorStreamBuffer = xStreamBufferCreate(
    STREAM_BUFFER_SIZE,
    SAMPLES_PER_BLOCK * sizeof(SensorData)  // Trigger level: one full block
  );
  
  if (sensorStreamBuffer == NULL || filteredSensorStreamBuffer == NULL) {
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
  xTaskCreatePinnedToCore(FilterTask, "FilterTask", 4096, NULL, 2, &FilterTaskHandle, 0);
  xTaskCreate(vCommTask, "CommTask", 4096, NULL, 2, &CommTaskHandle);
  xTaskCreate(DisplayTask, "DisplayTask", 2048, NULL, 1, &DisplayTaskHandle);
}

void loop() {
  // Empty. FreeRTOS handles everything in tasks.
}