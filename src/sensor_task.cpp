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
        currentData.accelXYZ[0] = (accel.acceleration.x / 9.81f) - CALIB_X;
        currentData.accelXYZ[1] = (accel.acceleration.y / 9.81f) - CALIB_Y;
        currentData.accelXYZ[2] = (accel.acceleration.z / 9.81f) - CALIB_Z; // / 9.81f it measures differently
      #endif
      currentData.doorHall = 0;  // Placeholder
      currentData.floorHall = 0;


      // Update global variable for DisplayTask
      if (xSemaphoreTake(displayMutex, 0) == pdTRUE) {
        latestAccelZ = currentData.accelXYZ[2];
        xSemaphoreGive(displayMutex);
      }

      // Write to StreamBuffer (non-blocking)
      xStreamBufferSend(sensorStreamBuffer, &currentData, sizeof(SensorData), 0);
    }
  }
}