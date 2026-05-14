#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

typedef struct AlertData {
    uint8_t elev_id;
    uint8_t alarm;
    uint8_t reserved[2]; // Padding per allineamento a 4 byte
} AlertData;

bool initLoRaCommTask();
void cloudTask(void *pvParameters);

extern QueueHandle_t alertQueue;