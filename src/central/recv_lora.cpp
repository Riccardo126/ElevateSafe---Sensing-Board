#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include "lora.h"
#include "secrets.h"

// OLED Display
extern Adafruit_SSD1306 display;

namespace {
// LoRa Configuration Parameters
constexpr float kLoRaFrequencyMHz = 868.0f;
constexpr float kLoRaBandwidthKHz = 125.0f;
constexpr uint8_t kLoRaSpreadingFactor = 7;
constexpr uint8_t kLoRaCodingRate = 5;
constexpr int8_t kLoRaOutputPower = 14;
constexpr uint16_t kLoRaPreambleLength = 8;
constexpr float kTcxoVoltage = 1.6f;
constexpr bool kUseLdoRegulator = false;

// LoRa Pin Configuration
constexpr uint8_t kRadioCsPin = 8;
constexpr uint8_t kRadioIrqPin = 14;
constexpr uint8_t kRadioResetPin = 12;
constexpr uint8_t kRadioBusyPin = 13;
constexpr uint8_t kRadioSckPin = 9;
constexpr uint8_t kRadioMisoPin = 11;
constexpr uint8_t kRadioMosiPin = 10;

// Task Configuration
constexpr size_t kAlertQueueLength = 8;
constexpr size_t kLoRaTaskStackSize = 6144;
constexpr UBaseType_t kLoRaTaskPriority = 2;
constexpr BaseType_t kLoRaTaskCore = 0;

// LoRa Radio Module
SPIClass loraSpi(FSPI);
Module loraModule(kRadioCsPin, kRadioIrqPin, kRadioResetPin, kRadioBusyPin, loraSpi);
SX1262 loraRadio(&loraModule);

// Queue and Task Handle
TaskHandle_t loRaTaskHandle = nullptr;

// Statistics
volatile uint32_t packetsReceived = 0;
volatile uint32_t receptionErrors = 0;
volatile uint8_t lastAlarmValue = 0;
volatile uint8_t lastElevId = 0;
volatile uint32_t lastPacketTime = 0;
volatile uint32_t packetIntervalMs = 0;

void logRadioStatus(const char *prefix, int16_t state) {
    uint32_t ms = millis();
    Serial.printf("[%lu ms] %s %d\n", ms, prefix, state);
}

void updateDisplay(const char *status, uint8_t elev_id, uint8_t alarmValue = 0) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    
    display.println("=== LoRa RECEIVER ===");
    display.println("");
        
    display.print("Status: ");
    display.println(status);
    display.println("");
    display.print("Alarm: ");
    display.println(alarmValue);
    display.print("Elevator ID: ");
    display.println(elev_id);
    display.print("Total Packets: ");
    display.println(packetsReceived);
    
    display.display();
}

void loraReceiveTask(void *pvParameters) {
    (void)pvParameters;
    uint32_t ms = millis();
    Serial.printf("[%lu ms] [LoRa Task] Reception task started\n", ms);
    
    AlertData alertData{};
    uint32_t lastWaitingTime = 0;
    
    for (;;) {
        uint32_t currentTime = millis();
        
        // Print "Waiting" every 1 second
        if (currentTime - lastWaitingTime >= 1000) {
            Serial.printf("[%lu ms] [LoRa] Waiting for packet...\n", currentTime);
            updateDisplay("Waiting...", lastElevId, lastAlarmValue);
            lastWaitingTime = currentTime;
        }
        
        // Ricezione diretta dei dati in plaintext all'interno della struttura AlertData
        int16_t state = loraRadio.receive(reinterpret_cast<uint8_t*>(&alertData), sizeof(AlertData));
        
        if (state == RADIOLIB_ERR_NONE) {
            packetsReceived++;
            lastAlarmValue = alertData.alarm;
            lastElevId = alertData.elev_id;
            
            uint32_t rxTime = millis();
            Serial.printf("[%lu ms] [LoRa] Packet received! Alarm=%u, Elevator ID=%u, Packets=%u\n",
                          rxTime, alertData.alarm, alertData.elev_id, packetsReceived);
            
            // Update display with Receiving status and alarm value
            updateDisplay("Receiving!", alertData.elev_id, alertData.alarm);
            
            // Send to queue for further processing
            xQueueSend(alertQueue, &alertData, 0);
            
            // Keep display for 1.5 seconds
            delay(1500);
            lastWaitingTime = millis();
            
        } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
            // Error (not timeout)
            receptionErrors++;
            uint32_t errTime = millis();
            Serial.printf("[%lu ms] [LoRa] RX error: %d, Total errors: %u\n", errTime, state, receptionErrors);
            
            char errMsg[32];
            sprintf(errMsg, "Error: %d", state);
            updateDisplay(errMsg, lastElevId, lastAlarmValue);
            
            delay(1000);
            lastWaitingTime = millis();
        }
        
        // Mandatory FreeRTOS delay for Watchdog Timer
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

} // namespace

static_assert(sizeof(AlertData) == 4, "AlertData must stay 4 bytes on ESP32");

bool initLoRaCommTask() {
    if (alertQueue != nullptr) {
        Serial.println("[LoRa] Already initialized");
        return true;
    }

    Serial.println("[LoRa] Initializing LoRa receiver...");

    // Setup pins
    pinMode(kRadioCsPin, OUTPUT);
    pinMode(kRadioIrqPin, INPUT);
    pinMode(kRadioResetPin, OUTPUT);
    pinMode(kRadioBusyPin, INPUT);
    
    Serial.printf("[LoRa] Pin setup: CS=%d, IRQ=%d, RST=%d, BUSY=%d\n",
                  kRadioCsPin, kRadioIrqPin, kRadioResetPin, kRadioBusyPin);

    // SPI initialization
    loraSpi.begin(kRadioSckPin, kRadioMisoPin, kRadioMosiPin, kRadioCsPin);
    Serial.println("[LoRa] SPI initialized");

    // Radio initialization
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
        logRadioStatus("[LoRa] Init failed", state);
        updateDisplay("Init Failed!", 0, true);
        return false;
    }

    Serial.println("[LoRa] Radio initialized successfully");

    // Create alert queue
    alertQueue = xQueueCreate(kAlertQueueLength, sizeof(AlertData));
    if (alertQueue == nullptr) {
        Serial.println("[LoRa] Failed to create alert queue");
        updateDisplay("Queue Error!", 0, true);
        return false;
    }

    Serial.println("[LoRa] Alert queue created");

    // Create reception task
    BaseType_t created = xTaskCreatePinnedToCore(loraReceiveTask,
                                                 "LoRaRecvTask",
                                                 kLoRaTaskStackSize,
                                                 nullptr,
                                                 kLoRaTaskPriority,
                                                 &loRaTaskHandle,
                                                 kLoRaTaskCore);
    if (created != pdPASS) {
        Serial.println("[LoRa] Failed to create FreeRTOS task");
        vQueueDelete(alertQueue);
        alertQueue = nullptr;
        updateDisplay("Task Error!", 0, true);
        return false;
    }

    Serial.println("[LoRa] Reception task created successfully");
    return true;
}