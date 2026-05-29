#include "display_task.h"
#include "shared.h"
#include "config.h"

const char* stateToString(ElevState state) {
    switch(state) {
        case FERMO: return "STILL";
        case PARTENZA: return "STARTING";
        case VIAGGIO_COSTANTE: return "CONSTANT";
        case FRENATA: return "BRAKING";
        default: return "UNKNOWN";
    }
}

// --- TASK 3: Display Update (2Hz) ---
void vDisplayTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500); // 2Hz to reduce flicker

  for(;;) {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      float zValue = latestAccelZ;
      xSemaphoreGive(displayMutex);

      const char* stateStr = stateToString(currentState);

      #ifdef HAS_OLED
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("Z Accel:");
        display.print(zValue, 3);  // Print with 3 decimal places
        display.println(" g");
        display.setCursor(0, 20);
        display.setTextSize(1);
        display.print(stateStr);
        
        display.display();
      #endif
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}