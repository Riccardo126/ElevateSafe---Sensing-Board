#include "comm_task.h"
#include "shared.h"
#include "config.h"

// --- TASK 2: Communication - sends data blocks ---
void vCommTask(void *pvParameters) {
  size_t totalBytes = SAMPLES_PER_BLOCK * sizeof(SensorData);
  uint8_t blockBuffer[totalBytes];
  BlockHeader header;
  header.magicByte = 0xAA;  // Fixed sync marker
  
  // Send a sync marker every 100ms to help with synchronization
  uint32_t lastSyncMarker = millis();
  
  for (;;) {    
    // Block until we have a full block of 50 samples.
    size_t receivedBytes = xStreamBufferReceive(
      filteredSensorStreamBuffer,
      blockBuffer,
      totalBytes,
      portMAX_DELAY
    );

    if (receivedBytes != totalBytes) {
      debugPrint("[CommTask] Unexpected stream receive size: %u\n", (unsigned)receivedBytes);
    }
    
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
        // Send block header + data in communication mode
        header.blockSeq = blockSequence++;
        Serial.write((uint8_t*)&header, sizeof(BlockHeader));
        Serial.write(blockBuffer, totalBytes);
      }
    }
  }
}
