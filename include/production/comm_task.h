#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "shared.h"


struct AlertData {
	uint8_t elev_id;
	uint8_t alarm;
	uint8_t reserved[2];
};

bool initLoRaCommTask();
bool sendAlertData(const AlertData &alertData);
void vCommTask(void *pvParameters);
