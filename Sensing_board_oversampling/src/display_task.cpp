#include "display_task.h"
#include "shared.h"
#include "config.h"

// --- TASK 3: Display Update (2Hz) ---
void DisplayTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500); // 2Hz to reduce flicker

  for(;;) {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      float zValue = latestAccelZ;
      xSemaphoreGive(displayMutex);
      #ifdef HAS_OLED
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("Z Accel:");
        display.setTextSize(1);
        display.setCursor(0, 25);
        display.print(zValue, 3);  // Print with 3 decimal places
        display.println(" g");
        display.display();
      #endif
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}