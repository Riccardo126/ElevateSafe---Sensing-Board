#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "cloud.h"
#include "secrets.h"
#include "lora.h"

AlertData data;

QueueHandle_t alertQueue = NULL;

TaskHandle_t task_aws = NULL;

void GenerateAlertData() {
    data.elev_id = random(1, 10); // Simula ID dell'ascensore
    data.alarm = random(0, 2);     // Simula stato dell'allarme (0 o 1)
}

void sendAlertData(void *pvParameters) {
    (void)pvParameters;
    while (true) {
        GenerateAlertData();
        if (xQueueSend(alertQueue, &data, portMAX_DELAY) != pdTRUE) {
            Serial.println("ERRORE: Impossibile inviare alla coda!");
        } else {
            Serial.printf("Dati inviati alla coda: Elevator ID=%u, Alarm=%u\n", data.elev_id, data.alarm);
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    // --- SETUP CODA FREERTOS ---
    alertQueue = xQueueCreate(10, sizeof(AlertData));
    if (alertQueue == NULL) {
        Serial.println("ERRORE: Coda non creata!");
        while(1); // Blocco di sicurezza
    }

    xTaskCreate(    sendAlertData,
                    "task_send_alert",
                    4096,
                    NULL,
                    10,
                    NULL
                );

    xTaskCreate(    connectAWS,
                    "task_aws",
                    4096,
                    NULL,
                    10,
                    &task_aws
                );
}

void loop()
{
}  