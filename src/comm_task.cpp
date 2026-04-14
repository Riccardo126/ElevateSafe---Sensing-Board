#include "comm_task.h"
#include "shared.h"
#include "config.h"

// --- TASK 2: Communication - sends data blocks ---
void vCommTask(void *pvParameters) {
  auto totalBytes = SAMPLES_PER_BLOCK * sizeof(SensorData); 
  uint8_t blockBuffer[totalBytes];
  
  uint32_t blocksSent = 0;
  debugPrintln("[CommTask] Started");  // <-- ADD THIS
  
  for (;;) {
    // Block until we have a full block of 50 samples
    size_t receivedBytes = xStreamBufferReceive(
      filteredSensorStreamBuffer, 
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
        float minZ = dataBlock[0].accelXYZ[2];
        float maxZ = dataBlock[0].accelXYZ[2];
        float sumZ = 0.0;
        
        for (int i = 0; i < SAMPLES_PER_BLOCK; i++) {
          float z = dataBlock[i].accelXYZ[2];
          sumZ += z;
          if (z < minZ) minZ = z;
          if (z > maxZ) maxZ = z;
        }
        float avgZ = sumZ / SAMPLES_PER_BLOCK;
        
        // Display block statistics
        debugPrint("[CommTask] Block #%lu | Z: avg=%.3f min=%.3f max=%.3f g\n", 
                   millis(), avgZ, minZ, maxZ);
      } else {
        // Send block through serial with frame synchronization preamble (0xAA 0xBB 0xCC 0xDD)
        // Format: [HEADER:4 bytes] + [BLOCK:1000 bytes] = 1004 bytes total
        uint8_t header[] = {0xAA, 0xBB, 0xCC, 0xDD};
        Serial.write(header, sizeof(header));
        Serial.write(blockBuffer, totalBytes);
        blocksSent++;
        debugPrint("[CommTask] Sent block #%lu with preamble AABBCCDD\n", blocksSent);
      };
    } else {
      debugPrint("[CommTask] Received incomplete block: %d bytes\n", receivedBytes);
    }
  }
}