#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

typedef struct AlertData {
    uint8_t elev_id;
    uint8_t alarm;
    uint8_t reserved[2]; // Padding to make the struct 4 bytes total for efficient queue storage
} AlertData;

const uint8_t LORA_AES_KEY[16] = {
    0x18, 0xE5, 0x5A, 0x4A, 0x6B, 0xDF, 0x78, 0x3F, 
    0xFB, 0x6C, 0xD0, 0x88, 0xD0, 0x16, 0xAF, 0x80
};

bool initLoRaCommTask();
bool sendAlertData(const AlertData &alertData);
