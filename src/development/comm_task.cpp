#include "comm_task.h"
#include "shared.h"
#include "config.h"

// --- TASK 2: Communication - sends data blocks ---
void vCommTask(void *pvParameters) {
  CommData packet;
  
  uint32_t blocksSent = 0;
  debugPrintln("[CommTask] Started");  // <-- ADD THIS
  
  for (;;) {
    // Block until we have an anomaly packet
    BaseType_t received = xQueueReceive(
      commSensorQueue,
      &packet,
      portMAX_DELAY
    );
    
    if (received != pdPASS) {
      debugPrintln("[CommTask] Received no packet");
      continue;
    }

    if (DEBUG_MODE) {
      //debugPrint("[CommTask] anomalyType=%u elevatorID=%u\n", packet.anomalyType, packet.elevatorID);
    } else {
      Serial.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
      blocksSent++;
      debugPrint("[CommTask] Sent packet #%lu\n", blocksSent);
    }
  }
}