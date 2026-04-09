#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <freertos/stream_buffer.h>
#include <freertos/semphr.h>
#include "config.h"  // For SensorData and other configs

// ========== DEBUG MODE CONFIGURATION ==========
// Set to true for debug messages, false for actual serial communication
extern bool DEBUG_MODE;

// Helper functions for debug/communication output
void debugPrint(const char* format, ...);
void debugPrintln(const char* message);

// ==============================================

// Task Handles
extern TaskHandle_t SensorTaskHandle;
extern TaskHandle_t DisplayTaskHandle;
extern TaskHandle_t CommTaskHandle;

// StreamBuffer for efficient binary data transfer
extern StreamBufferHandle_t sensorStreamBuffer;

// Semaphore to trigger sampling at precise 1kHz
extern SemaphoreHandle_t samplingTrigger;

// Timer handle
extern esp_timer_handle_t samplingTimer;

// Global variable to store latest Z value for display
extern volatile float latestAccelZ;
extern SemaphoreHandle_t displayMutex;

// Define your new I2C pins
#ifdef BOARD_HELTEC
  const int IMU_SDA_PIN = 41; //verde
  const int IMU_SCL_PIN = 42; //arancione
#endif
#ifdef BOARD_NODEMCU
  const int IMU_SDA_PIN = 26; //verde
  const int IMU_SCL_PIN = 27; //arancione
#endif

// Heltec WiFi LoRa 32 V3 onboard OLED pins
extern const int OLED_SDA_PIN;
extern const int OLED_SCL_PIN;
extern const int OLED_RST_PIN;
extern const int OLED_VEXT_PIN;

// Heltec WiFi LoRa 32 V3 onboard OLED (usually I2C address 0x3C)
// OLED — solo su Heltec
#ifdef HAS_OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  extern constexpr int SCREEN_WIDTH;
  extern constexpr int SCREEN_HEIGHT;
  extern constexpr int OLED_RESET;
  extern Adafruit_SSD1306 display;
#endif

// Sensor Instances
// IMU — libreria diversa per board diversa
#ifdef HAS_LSM6DS3
  #include <SparkFunLSM6DS3.h>
  extern LSM6DS3 myIMU;
#endif

#ifdef HAS_MPU6050
  #include <Adafruit_MPU6050.h>
  #include <Adafruit_Sensor.h>
  extern Adafruit_MPU6050 myIMU;
#endif

extern const int HALL_DOOR_PIN;
extern const int HALL_FLOOR_PIN;

// Calibration offsets for IMU (auto-calibrated on startup)
extern float CALIB_X;
extern float CALIB_Y;
extern float CALIB_Z;

// Block header for synchronization
struct BlockHeader {
  uint8_t magicByte;    // 0xAA - fixed marker
  uint8_t blockSeq;     // Sequence number (0-255, wraps around)
};

// Counter for block sequence
extern volatile uint8_t blockSequence;

// I2C scan function
void scanI2C(TwoWire &bus, const char *busName);

// Hardware timer ISR callback - triggers SensorTask at 1kHz
void timerCallback(void *arg);