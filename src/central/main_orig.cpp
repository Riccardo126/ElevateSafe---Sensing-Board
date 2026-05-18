/*
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "lora.h"
#include "cloud.h"
#include "secrets.h"

// OLED Display pins
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define VEXT_PIN 36

QueueHandle_t alertQueue = NULL;

TaskHandle_t task_wifi = NULL;

// Creazione globale del display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=== LoRa RECEIVER START ===\n");

    // Power up OLED display (VEXT trucco Heltec)
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(100);
    
    // Reset Hardware OLED
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(10);
    digitalWrite(OLED_RST, HIGH);
    delay(100);

    // Initialize I2C per lo schermo
    Wire.begin(OLED_SDA, OLED_SCL);
    Serial.println("[Setup] I2C initialized");

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("[Setup] OLED initialization failed!");
        while (1) delay(1000);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("LoRa RECEIVER");
    display.println("Initializing...");
    display.display();

    // Avvia il modulo LoRa
    if (!initLoRaCommTask()) {
        Serial.println("[Setup] LoRa initialization FAILED!");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("INIT FAILED!");
        display.display();
        while (1) delay(1000);
    }

    Serial.println("[Setup] LoRa receiver ready!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("LoRa RECEIVER");
    display.println("Ready!");
    display.println("");
    display.println("Waiting for packets...");
    display.display();
    delay(2000);

    // --- SETUP CODA FREERTOS ---
    alertQueue = xQueueCreate(10, sizeof(AlertData));
    if (alertQueue == NULL) {
        Serial.println("ERRORE: Coda non creata!");
        while(1); // Blocco di sicurezza
    }

    // --- SETUP MODULO LORA ---
    if (!initLoRaCommTask()) {
        Serial.println("ERRORE: Radio LoRa fallita!");
        display.println("LORA FAIL");
        display.display();
        while(1);
    }

    xTaskCreate(    connect_wifi,
                    "task_wifi",
                    4096,
                    NULL,
                    10,
                    &task_wifi);
    // --- FINE ---
    display.println("SISTEMA ONLINE!");
    display.display();
    Serial.println("Centro di Comando: Setup completato. Cedo il controllo a FreeRTOS.");

    // Delete Arduino loop task
    vTaskDelete(NULL);
}

void loop() {
    // Questo blocco non viene mai eseguito
}
    */

    