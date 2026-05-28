#include "comm_task.h"
#include "shared.h"
#include "config.h"

// --- TASK 2: Communication - sends data blocks ---
void vCommTask(void *pvParameters) {
  CommData packet;
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
  }
}