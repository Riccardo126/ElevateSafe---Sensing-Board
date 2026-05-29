#include "comm_task.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>
#include "shared.h"


constexpr float kLoRaFrequencyMHz = 868.0f;
constexpr float kLoRaBandwidthKHz = 125.0f;
constexpr uint8_t kLoRaSpreadingFactor = 7;
constexpr uint8_t kLoRaCodingRate = 5;
constexpr int8_t kLoRaOutputPower = 14;
constexpr uint16_t kLoRaPreambleLength = 8;
constexpr float kTcxoVoltage = 1.6f;
constexpr bool kUseLdoRegulator = false;

constexpr uint8_t kRadioCsPin = 8;
constexpr uint8_t kRadioIrqPin = 14;
constexpr uint8_t kRadioResetPin = 12;
constexpr uint8_t kRadioBusyPin = 13;
constexpr uint8_t kRadioSckPin = 9;
constexpr uint8_t kRadioMisoPin = 11;
constexpr uint8_t kRadioMosiPin = 10;

constexpr uint8_t kOledAddress = 0x3C;
constexpr uint8_t kOledSdaPin = 17;
constexpr uint8_t kOledSclPin = 18;
constexpr int16_t kOledResetPin = 21;
constexpr uint8_t kOledVextPin = 36;
constexpr int kOledWidth = 128;
constexpr int kOledHeight = 64;

SPIClass loraSpi(FSPI);
Module loraModule(kRadioCsPin, kRadioIrqPin, kRadioResetPin, kRadioBusyPin, loraSpi);
SX1262 loraRadio(&loraModule);
Adafruit_SSD1306 oled(kOledWidth, kOledHeight, &Wire, kOledResetPin);
bool oledReady = false;

void showOledStatus(const char *title, const char *line) {
    if (!oledReady) {
        return;
    }

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println(title);
    oled.println(line);
    oled.display();
}

void logRadioStatus(const char *prefix, int16_t state) {
    Serial.printf("%s %d\n", prefix, state);
}

void vCommTask(void *pvParameters) {
    (void)pvParameters;
    initLoRaCommTask();
    AlertData alertData{};

    for (;;) {
        if (xQueueReceive(commSensorQueue, &alertData, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        char oledLine[24];
        snprintf(oledLine, sizeof(oledLine), "alarm=%d", alertData.alarm);
        Serial.printf("[LoRa] TX start alarm=%d\n", alertData.alarm);
        //showOledStatus("Invio...", oledLine);

        // Invio diretto dei dati in plaintext
        int16_t state = loraRadio.transmit(reinterpret_cast<uint8_t*>(&alertData), sizeof(AlertData));
        
        if (state == RADIOLIB_ERR_NONE) {
            Serial.printf("[LoRa] Sent AlertData alarm=%d , elev_id=%d , size=%u bytes\n",
                          alertData.alarm,
                          alertData.elev_id,
                          static_cast<unsigned>(sizeof(AlertData)));
            //showOledStatus("Messaggio inviato", oledLine);
        } else {
            logRadioStatus("[LoRa] TX failed", state);
            char errorLine[24];
            snprintf(errorLine, sizeof(errorLine), "err=%d", state);
            //showOledStatus("Invio fallito", errorLine);
        }
    }
}
static_assert(sizeof(AlertData) == 4, "AlertData must stay 4 bytes on ESP32");

bool initLoRaCommTask() {

    // Heltec V3 powers the onboard OLED through Vext (active low).
    pinMode(kOledVextPin, OUTPUT);
    digitalWrite(kOledVextPin, LOW);
    delay(50);

    Wire.begin(kOledSdaPin, kOledSclPin);
    oledReady = oled.begin(SSD1306_SWITCHCAPVCC, kOledAddress);
    if (!oledReady) {
        Serial.println("[OLED] Init failed");
    } else {
        //showOledStatus("OLED pronto");
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

    Serial.println("[LoRa] task initialized");
    return true;
}

bool sendAlertData(const AlertData &alertData) {
    return xQueueSend(commSensorQueue, &alertData, 0) == pdTRUE;
}