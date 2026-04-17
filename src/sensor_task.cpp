#include "sensor_task.h"
#include "shared.h"
#include "config.h"

// --- TASK 1: Sensor Reading (triggered by hardware timer) ---
void SensorTask(void *pvParameters) {
  SensorData currentData;

  for (;;) {
    // Wait for hardware timer to trigger (1kHz)
    if (xSemaphoreTake(samplingTrigger, portMAX_DELAY) == pdTRUE) {         
      #ifdef HAS_LSM6DS3
        currentData.accelXYZ[0] = myIMU.readFloatAccelX() - CALIB_X;
        currentData.accelXYZ[1] = myIMU.readFloatAccelY() - CALIB_Y;
        currentData.accelXYZ[2] = myIMU.readFloatAccelZ() - CALIB_Z;
      #endif
      #ifdef HAS_MPU6050
        sensors_event_t accel, gyro, temp;
        myIMU.getEvent(&accel, &gyro, &temp);
        currentData.accelXYZ[0] = (accel.acceleration.x ) - CALIB_X;
        currentData.accelXYZ[1] = (accel.acceleration.y ) - CALIB_Y;
        currentData.accelXYZ[2] = (accel.acceleration.z ) - CALIB_Z; // / 9.81f it measures differently
      #endif
      currentData.doorHall = 0;  // Placeholder
      currentData.floorHall = 0;


      // Update global variable for DisplayTask
      if (xSemaphoreTake(displayMutex, 0) == pdTRUE) {
        latestAccelZ = currentData.accelXYZ[2];
        xSemaphoreGive(displayMutex);
      }

      // Write to StreamBuffer with a small timeout so we can detect failures.
      size_t bytesSent = 0;
      #if defined(FILTER_TYPE) && (FILTER_TYPE == 0)
      // If no filtering, send raw data directly from SensorTask
      bytesSent = xStreamBufferSend(sensorStreamBuffer, &currentData, sizeof(SensorData), pdMS_TO_TICKS(10));
      #endif
      #if !defined(FILTER_TYPE) || (FILTER_TYPE != 0)
      // If filtering is enabled, send raw data to FilterTask for processing  
      bytesSent = xStreamBufferSend(filteredSensorStreamBuffer, &currentData, sizeof(SensorData), pdMS_TO_TICKS(10));
      #endif
      if (bytesSent != sizeof(SensorData)) {
        debugPrint("[SensorTask] StreamBuffer send failed (%u/%u)\n", (unsigned)bytesSent, (unsigned)sizeof(SensorData));
      }
    }
  }
}