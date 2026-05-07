#include <Arduino.h>

#include "comm_task.h"

constexpr uint32_t kDemoAlertPeriodMs = 10000;
constexpr uint32_t kDemoTaskStackSize = 3072;
constexpr UBaseType_t kDemoTaskPriority = 1;

void demoAlertTask(void *pvParameters) {
	(void)pvParameters;

	uint32_t lastDemoSendMs = 0;
	bool sendWarning = true;

	for (;;) {
		if (millis() - lastDemoSendMs >= kDemoAlertPeriodMs) {
			lastDemoSendMs = millis();

			AlertData alertData{};
			alertData.alarm = sendWarning ? 1 : 2;
			sendWarning = !sendWarning;

			if (!sendAlertData(alertData)) {
				Serial.println("[BOOT] Failed to enqueue alert");
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void setup() {
	Serial.begin(115200);
	while (!Serial && millis() < 2000) {
		delay(10);
	}

	if (!initLoRaCommTask()) {
		Serial.println("[BOOT] LoRa initialization failed");
		while (true) {
			delay(1000);
		}
	}

	BaseType_t created = xTaskCreatePinnedToCore(demoAlertTask,
	                                             "DemoAlertTask",
	                                             kDemoTaskStackSize,
	                                             nullptr,
	                                             kDemoTaskPriority,
	                                             nullptr,
	                                             1);
	if (created != pdPASS) {
		Serial.println("[BOOT] Failed to create demo task");
		while (true) {
			delay(1000);
		}
	}

	Serial.println("[BOOT] LoRa comm task ready");
}

void loop() {}
