#include "integrator_task.h"
#include "shared.h"
#include "config.h"
#include <math.h>

// --- TASK 3: Analysis - integrates acceleration to velocity and distance ---
void vIntegratorTask(void *pvParameters) {
  SensorData sample;
  uint32_t sampleCount = 0; // keep track from start

  const float dt = 1.0f / SAMPLE_RATE_HZ;
  float prevAccelZ = 0.0f;
  float prevVelocityZ = 0.0f;
  float cum_vZ = 0.0f;
  float delta_vZ = 0.0f;
  float positionZ = 0.0f;
  float maxAccelZ = 0.0f; // for now only local
  bool hasPrevAccel = false;

  uint16_t hallStaticCounter = 0; // counts how many consecutive samples have the same hall reading (for static detection) 
  uint16_t hallPrev = 0; // previous hall reading for static detection
  bool isStatic = false; // flag to indicate if the elevator is static
  uint16_t staticCounterThreshold = SAMPLE_RATE_HZ * 1; // number of consecutive samples to consider the elevator as static (e.g., 100 samples at 1kHz = 100ms)
  uint32_t lastTimeStoppedMs = 0; // timestamp of the last time the elevator was detected as stopped

  debugPrintln("[IntegratorTask] Started");

  for (;;) {
    size_t receivedBytes = xStreamBufferReceive(
      filteredSensorStreamBuffer,
      &sample,
      sizeof(SensorData),
      portMAX_DELAY
    );

    if (receivedBytes != sizeof(SensorData)) continue;

    float accelX = sample.accelXYZ[0];
    float accelY = sample.accelXYZ[1];
    float accelZ = sample.accelXYZ[2];
    int floorHall = sample.floorHall;
    if (accelZ > maxAccelZ) maxAccelZ = accelZ;

    if (!hasPrevAccel) { // First sample, just initialize
      prevAccelZ = accelZ;
      hasPrevAccel = true;
      hallPrev = floorHall;
      sampleCount++;
      continue;
    }

    // Trapezoidal integration for velocity, instant and cumulative, and for distance
    delta_vZ = 0.5f * (prevAccelZ + accelZ) * dt; // Area of trapezoid for this interval
    if (DEADBANDING) {
      delta_vZ = fabsf(delta_vZ) < DEADBAND_THR/50.0f ? 0 : delta_vZ; // Deadband for velocity
    }
    cum_vZ = prevVelocityZ + delta_vZ; // Update cumulative velocity
    
    positionZ += 0.5f * (prevVelocityZ + cum_vZ) * dt; // Trapezoidal integration for distance

    // Static detection based on hall sensor
    if (fabsf(floorHall) >= 250 && floorHall >= 0.7 *hallPrev && floorHall <= 1.3 * hallPrev) { // if hall reading is stable within 10% of previous
      hallStaticCounter++;
      if (hallStaticCounter >= staticCounterThreshold) {
        debugPrint("im stopped");
        if (!isStatic) lastTimeStoppedMs = millis();
        isStatic = true;
        // reset stuff
        cum_vZ = 0.0f;
        accelZ = 0.0f;
        prevVelocityZ = 0.0f;
        prevAccelZ = 0.0f;
        // positionZ = 0.0f;
        
      }
    } else {
      hallStaticCounter = 0;
      isStatic = false;
    }
    hallPrev = floorHall;

    if (velocityQueue != NULL) xQueueOverwrite(velocityQueue, &cum_vZ);

    prevAccelZ = accelZ;
    prevVelocityZ = cum_vZ;
    sampleCount++;

    // check elevator stuck between floors: no hall and speed 0
    if (fabsf(floorHall) <= 100 && fabsf(cum_vZ) <= 0.05) {
      uint32_t nowMs = millis();
      CommData commOut;
      if (nowMs - lastTimeStoppedMs >= 500) { // if we've been stopped for at least 500ms
        commOut.anomalyType = 2; // Placeholder for actual anomaly type
        commOut.elevatorID = 1; // Placeholder for actual elevator ID
        xQueueSendToBack(commSensorQueue, &commOut, 5);
      }
    }
    if (sampleCount % 5 == 0) {
      //Serial.printf("%.3f\t%.3f\n", cum_vZ, positionZ);
    }
  }
}