#include "comm_task.h"

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <string.h>

namespace {
constexpr float kLoRaFrequencyMHz = 868.0f;
constexpr float kLoRaBandwidthKHz = 125.0f;
constexpr uint8_t kLoRaSpreadingFactor = 7;
constexpr uint8_t kLoRaCodingRate = 5;
constexpr int8_t kLoRaOutputPower = 14;
constexpr uint16_t kLoRaPreambleLength = 8;
constexpr float kTcxoVoltage = 1.6f;
constexpr bool kUseLdoRegulator = false;

constexpr uint8_t kRadioCsPin = 14;
constexpr uint8_t kRadioIrqPin = 35;
constexpr uint8_t kRadioResetPin = 5;
constexpr uint8_t kRadioBusyPin = 34;
constexpr uint8_t kRadioSckPin = 12;
constexpr uint8_t kRadioMisoPin = 15;
constexpr uint8_t kRadioMosiPin = 13;

constexpr size_t kAlertQueueLength = 8;
constexpr size_t kLoRaTaskStackSize = 6144;
constexpr UBaseType_t kLoRaTaskPriority = 2;
constexpr BaseType_t kLoRaTaskCore = 0;

SPIClass loraSpi(FSPI);
Module loraModule(kRadioCsPin, kRadioIrqPin, kRadioResetPin, kRadioBusyPin, loraSpi);
SX1262 loraRadio(&loraModule);
QueueHandle_t alertQueue = nullptr;
TaskHandle_t loRaTaskHandle = nullptr;

void logRadioStatus(const char *prefix, int16_t state) {
    Serial.printf("%s %d\n", prefix, state);
}

void loraTask(void *pvParameters) {
    (void)pvParameters;

    AlertData alertData{};
    for (;;) {
        if (xQueueReceive(alertQueue, &alertData, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        const uint8_t *payload = reinterpret_cast<const uint8_t *>(&alertData);
        int16_t state = loraRadio.transmit(payload, sizeof(AlertData));
        if (state == RADIOLIB_ERR_NONE) {
            Serial.printf("[LoRa] Sent AlertData alarm=%d size=%u bytes\n",
                          alertData.alarm,
                          static_cast<unsigned>(sizeof(AlertData)));
        } else {
            logRadioStatus("[LoRa] TX failed", state);
        }
    }
}
} // namespace

static_assert(sizeof(AlertData) == 4, "AlertData must stay 4 bytes on ESP32");

bool initLoRaCommTask() {
    if (alertQueue != nullptr) {
        return true;
    }

    pinMode(kRadioCsPin, OUTPUT);
    pinMode(kRadioIrqPin, INPUT);
    pinMode(kRadioResetPin, OUTPUT);
    pinMode(kRadioBusyPin, INPUT);

    loraSpi.begin(kRadioSckPin, kRadioMisoPin, kRadioMosiPin, kRadioCsPin);

    int16_t state = loraRadio.begin(kLoRaFrequencyMHz,
                                    kLoRaBandwidthKHz,
                                    kLoRaSpreadingFactor,
                                    kLoRaCodingRate,
                                    RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                                    kLoRaOutputPower,
                                    kLoRaPreambleLength,
                                    kTcxoVoltage,
                                    kUseLdoRegulator);
    if (state != RADIOLIB_ERR_NONE) {
        logRadioStatus("[LoRa] init failed", state);
        return false;
    }

    alertQueue = xQueueCreate(kAlertQueueLength, sizeof(AlertData));
    if (alertQueue == nullptr) {
        Serial.println("[LoRa] Failed to create alert queue");
        return false;
    }

    BaseType_t created = xTaskCreatePinnedToCore(loraTask,
                                                 "LoRaCommTask",
                                                 kLoRaTaskStackSize,
                                                 nullptr,
                                                 kLoRaTaskPriority,
                                                 &loRaTaskHandle,
                                                 kLoRaTaskCore);
    if (created != pdPASS) {
        Serial.println("[LoRa] Failed to create FreeRTOS task");
        vQueueDelete(alertQueue);
        alertQueue = nullptr;
        return false;
    }

    Serial.println("[LoRa] task initialized");
    return true;
}

bool sendAlertData(const AlertData &alertData) {
    if (alertQueue == nullptr) {
        return false;
    }

    return xQueueSend(alertQueue, &alertData, 0) == pdTRUE;
}
