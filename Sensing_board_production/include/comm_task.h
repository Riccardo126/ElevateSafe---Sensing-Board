#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

typedef struct AlertData {
    int alarm;
} AlertData;

bool initLoRaCommTask();
bool sendAlertData(const AlertData &alertData);
