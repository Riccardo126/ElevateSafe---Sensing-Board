#include "sensor_task.h"
#include "shared.h"
#include "config.h"

// --- TASK 1: Sensor Reading (triggered by hardware timer) ---
void vSensorTask(void *pvParameters) {
  // first time imu axis alignment and calibration
  #ifdef HAS_LSM6DS3
    myIMU.readFloatAccelX();
  #endif
  #ifdef HAS_MPU6050
    sensors_event_t accel, gyro, temp;
    myIMU.getEvent(&accel, &gyro, &temp);
    float angleXYZ[3] = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z};
  #endif
  SensorData currentData;

  for (;;) {
    // Wait for hardware timer to trigger (1kHz)
    if (xSemaphoreTake(samplingTrigger, portMAX_DELAY) == pdTRUE) {         
      #ifdef HAS_LSM6DS3
        if (DEADBANDING) {// deadband = 0.005
        float calibratedX = (myIMU.readFloatAccelX() - CALIB_X) * 9.81f; // convert to m/s^2
        float calibratedY = (myIMU.readFloatAccelY() - CALIB_Y) * 9.81f; // convert to m/s^2
        float calibratedZ = (myIMU.readFloatAccelZ() - CALIB_Z) * 9.81f; // convert to m/s^2
        currentData.accelXYZ[0] = fabsf(calibratedX) < DEADBAND_THR ? 0 : calibratedX;
        currentData.accelXYZ[1] = fabsf(calibratedY) < DEADBAND_THR ? 0 : calibratedY; 
        currentData.accelXYZ[2] = fabsf(calibratedZ) < DEADBAND_THR ? 0 : calibratedZ; 
      } else {
        currentData.accelXYZ[0] = (myIMU.readFloatAccelX() - CALIB_X) * 9.81f; // convert to m/s^2
        currentData.accelXYZ[1] = (myIMU.readFloatAccelY() - CALIB_Y) * 9.81f; // convert to m/s^2
        currentData.accelXYZ[2] = (myIMU.readFloatAccelZ() - CALIB_Z) * 9.81f; // convert to m/s^2
      }    
      #endif
      #ifdef HAS_MPU6050
        sensors_event_t accel, gyro, temp;
        myIMU.getEvent(&accel, &gyro, &temp);
        if (DEADBANDING) { // deadband = 0.05 
          // deadband = 0.05
          float calibratedX = (accel.acceleration.x ) - CALIB_X;
          float calibratedY = (accel.acceleration.y ) - CALIB_Y;
          float calibratedZ = (accel.acceleration.z ) - CALIB_Z; // / 9.81f it measures differently
          currentData.accelXYZ[0] = fabsf(calibratedX) < DEADBAND_THR ? 0 : calibratedX;
          currentData.accelXYZ[1] = fabsf(calibratedY) < DEADBAND_THR ? 0 : calibratedY;
          currentData.accelXYZ[2] = fabsf(calibratedZ) < DEADBAND_THR ? 0 : calibratedZ; // / 9.81f it measures differently
        } else {
          currentData.accelXYZ[0] = accel.acceleration.x - CALIB_X;
          currentData.accelXYZ[1] = accel.acceleration.y - CALIB_Y;
          currentData.accelXYZ[2] = accel.acceleration.z - CALIB_Z; // / 9.81f it measures differently
        }
        #endif
      currentData.anomaly = false;
      currentData.floorHall = analogRead(HALL_FLOOR_PIN) - 1900; // calib


      // Update global variable for DisplayTask
      if (xSemaphoreTake(displayMutex, 0) == pdTRUE) {
        latestAccelZ = currentData.accelXYZ[2];
        xSemaphoreGive(displayMutex);
      }

      // Write to StreamBuffer with a small timeout so we can detect failures.
      size_t bytesSent = 0; 
      bytesSent = xStreamBufferSend(sensorStreamBuffer, &currentData, sizeof(SensorData), pdMS_TO_TICKS(10));

      if (bytesSent != sizeof(SensorData)) {
        debugPrint("[SensorTask] StreamBuffer send failed (%u/%u)\n", (unsigned)bytesSent, (unsigned)sizeof(SensorData));
      }
    }
  }
}