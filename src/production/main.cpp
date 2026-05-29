/*#include <Arduino.h>

#include "comm_task.h"

constexpr uint32_t kDemoAlertPeriodMs = 20000;
constexpr uint32_t kDemoTaskStackSize = 3072;
constexpr UBaseType_t kDemoTaskPriority = 1;

void demoAlertTask(void *pvParameters) {
	(void)pvParameters;

	uint32_t lastDemoSendMs = millis() - kDemoAlertPeriodMs;
	bool sendWarning = true;

	for (;;) {
		if (millis() - lastDemoSendMs >= kDemoAlertPeriodMs) {
			lastDemoSendMs = millis();

			AlertData alertData{};
			alertData.elev_id = random(1, 11);
			alertData.alarm = sendWarning ? 1 : 2;
			sendWarning = !sendWarning;

			if (!sendAlertData(alertData)) {
				Serial.println("[BOOT] Failed to enqueue alert");
			} else {
				Serial.printf("[BOOT] Enqueued alert alarm=%d\n", alertData.alarm);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void setup() {
	Serial.begin(115200);
	delay(1000); // Allow time for serial monitor to connect
	while (!Serial && millis() < 2000) {
		delay(10);
	}

	if (!initLoRaCommTask()) {
		Serial.println("[BOOT] LoRa initialization failed");
		while (true) {
			delay(1000);
		}
	}

	BaseType_t created = xTaskCreatePinnedToCore(demoAlertTask,
	                                             "DemoAlertTask",
	                                             kDemoTaskStackSize,
	                                             nullptr,
	                                             kDemoTaskPriority,
	                                             nullptr,
	                                             1);
	if (created != pdPASS) {
		Serial.println("[BOOT] Failed to create demo task");
		while (true) {
			delay(1000);
		}
	}

	Serial.println("[BOOT] LoRa comm task ready");
}

void loop() {}
*/

#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include "WiFi.h"
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
TaskHandle_t IntegratorTaskHandle;

bool anomalyXY = false;
bool anomalyZmiss = false;
bool Up = false;
bool Down = false;

// StreamBuffer for efficient binary data transfer
StreamBufferHandle_t sensorStreamBuffer; // raw sensor data from SensorTask to FilterTask 
StreamBufferHandle_t filteredSensorStreamBuffer; // filtered data from FilterTask to IntegratorTask
QueueHandle_t commSensorQueue; // anomaly packets from multiple tasks to CommTask

// Semaphore to trigger sampling at precise 1kHz
SemaphoreHandle_t samplingTrigger;

// Timer handle
esp_timer_handle_t samplingTimer;
// Ensures deterministic, jitter-free sampling at exactly 1000 Hz
SemaphoreHandle_t displayMutex;

// Global variable to store latest Z value for display
volatile float latestAccelZ = 0.0;

// Global max z speed for comparison
volatile float maxZaccel = 0.0f;


// Heltec WiFi LoRa 32 V3 onboard OLED (usually I2C address 0x3C)
// OLED — solo su Heltec
#ifdef HAS_OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  // Heltec WiFi LoRa 32 V3 onboard OLED pins
  const int OLED_SDA_PIN = 17;
  const int OLED_SCL_PIN = 18;
  const int OLED_RST_PIN = 21;
  const int OLED_VEXT_PIN = 36;
  const int SCREEN_WIDTH = 128;
  const int SCREEN_HEIGHT = 64;
  const int OLED_RESET = -1;
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
#endif

// Sensor Instances
// IMU — libreria diversa per board diversa
// Calibration offsets for IMU (auto-calibrated on startup)
#ifdef HAS_LSM6DS3
  #include <SparkFunLSM6DS3.h>
  LSM6DS3 myIMU(I2C_MODE, 0x6B);
  float CALIB_X = 0.010, CALIB_Y = -0.002, CALIB_Z = 1.013;
#endif


const int HALL_FLOOR_PIN = 7;

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

ElevState currentState = FERMO; // Stato iniziale dell'ascensore

void setup() {
  Serial.begin(115200); 

  // Give the monitor a short window to attach after reset.
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart) < 3000) {
    delay(10);
  }
  debugPrintln("\n[BOOT] ElevateSafe start");
  delay(3000);

  // disable bluetooth and wifi
  WiFi.mode(WIFI_OFF); // Disattiva completamente il WiFi
  btStop(); // Disattiva completamente il Bluetooth

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

    
    // add to solve snow problem on oled:
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("HELLO FROM SENSING NODE");
    display.display();
  #endif

  // 1. Initialize the second I2C bus (Wire) with your custom pins
  pinMode(IMU_SDA_PIN, INPUT_PULLUP);  // Enable internal pullup
  pinMode(IMU_SCL_PIN, INPUT_PULLUP);  // Enable internal pullup
  bool wireInit = Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);  // Initialize directly with IMU pins
  if (!wireInit) {
    debugPrintln("ERROR: Wire.begin() failed! Check SDA/SCL pin definitions and connections.");
    while(1); // halt
  }

  Wire.setClock(100000);  // Set to 100kHz
  delay(100);  // Wait for sensor to power up
  scanI2C(Wire, "Wire(IMU)");

  bool imuStatus = false;

  #ifdef HAS_LSM6DS3
    debugPrintln("Initializing LSM6DS3...");
    status_t imuS = myIMU.begin();
    imuStatus = (imuS == IMU_SUCCESS);
  #endif
  #ifdef HAS_MPU6050
    debugPrintln("Initializing MPU6050...");
    imuStatus = !myIMU.begin(0x68, &Wire); // looks like 0=success, 1=failure, opposite of usual Arduino convention
    if (!imuStatus) myIMU.setAccelerometerRange(MPU6050_RANGE_2_G);
  #endif  

  debugPrint("IMU begin status: %d \n", imuStatus);
  if (!imuStatus) {
    debugPrintln("ERROR: IMU initialization failed!");
    while(1); // halt
  }
  // Hall setup
  debugPrintln("[BOOT] Initializing Hall sensor pin...");
  pinMode(HALL_FLOOR_PIN, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // ADC Attenuation: 11dB allows reading up to ~3.1V
  
  
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
    sizeof(SensorData)  // Trigger level: wake as soon as one sample is available
  );
  filteredSensorStreamBuffer = xStreamBufferCreate(
    STREAM_BUFFER_SIZE,
    sizeof(SensorData)  // Trigger level: wake as soon as one sample is available
  );

  commSensorQueue = xQueueCreate(20, sizeof(AlertData));

  if (sensorStreamBuffer == NULL || filteredSensorStreamBuffer == NULL || commSensorQueue == NULL) {
    debugPrintln("ERROR: StreamBuffer or queue creation failed!");
    while(1); // halt
  }

  if (commSensorQueue == nullptr) {
    Serial.println("[LoRa] Failed to create alert queue");
    while (true) {
      delay(1000);
    }
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
  xTaskCreatePinnedToCore(vSensorTask, "SensorTask", 4096, NULL, 3, &SensorTaskHandle, 1);
  xTaskCreatePinnedToCore(vFilterTask, "FilterTask", 4096, NULL, 2, &FilterTaskHandle, 0);
  //xTaskCreatePinnedToCore(vIntegratorTask, "IntegratorTask", 4096, NULL, 2, &IntegratorTaskHandle, 0);
  xTaskCreate(vCommTask, "CommTask", 6096, NULL, 2, &CommTaskHandle);
  xTaskCreate(vDisplayTask, "DisplayTask", 2048, NULL, 1, &DisplayTaskHandle);
}

void loop() {
  // Empty. FreeRTOS handles everything in tasks.
}