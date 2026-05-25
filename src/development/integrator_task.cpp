#include "integrator_task.h"
#include "shared.h"
#include "config.h"
#include <math.h>

// --- TASK 3: Analysis - integrates acceleration to velocity and distance ---
void vIntegratorTask(void *pvParameters) {
  SensorData sample;

  const float dt = 1.0f / SAMPLE_RATE_HZ;
  float prevAccelZ = 0.0f;
  float prevVelocityZ = 0.0f;
  float cum_vZ = 0.0f;
  float delta_vZ = 0.0f;
  float positionZ = 0.0f;
  float maxAccelZ = 0.0f; // for now only local
  uint32_t sampleCount = 0; // keep track from start
  bool hasPrevAccel = false;

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
    if (accelZ > maxAccelZ) maxAccelZ = accelZ;

    if (!hasPrevAccel) { // First sample, just initialize
      prevAccelZ = accelZ;
      hasPrevAccel = true;
      sampleCount++;
      continue;
    }

    // Trapezoidal integration for velocity, instant and cumulative, and for distance
    delta_vZ = 0.5f * (prevAccelZ + accelZ) * dt; // Area of trapezoid for this interval
    cum_vZ = prevVelocityZ + delta_vZ; // Update cumulative velocity
    positionZ += 0.5f * (prevVelocityZ + cum_vZ) * dt; // Trapezoidal integration for distance


    if (velocityQueue != NULL) {
      xQueueOverwrite(velocityQueue, &cum_vZ);
    }

    prevAccelZ = accelZ;
    prevVelocityZ = cum_vZ;
    sampleCount++;

    if (sampleCount % 10 == 0) {
      Serial.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", accelX, accelY, accelZ, cum_vZ, positionZ);
    }
  }
}