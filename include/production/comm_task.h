#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

typedef struct AlertData {
    uint8_t elev_id;
    uint8_t alarm;
    uint8_t reserved[2]; // Padding to make the struct 4 bytes total for efficient queue storage
} AlertData;

bool initLoRaCommTask();
bool sendAlertData(const AlertData &alertData);
void vCommTask(void *pvParameters);
