/*
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "lora.h"
#include "cloud.h"
#include "secrets.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define VEXT_PIN 36

QueueHandle_t alertQueue = NULL;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n\n=== CENTRAL NODE START ===\n");

    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(100);
    
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(10);
    digitalWrite(OLED_RST, HIGH);
    delay(100);

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
    display.println("CENTRAL NODE");
    display.println("Initializing...");
    display.display();

    if (!initLoRaCommTask()) {
        Serial.println("[Setup] LoRa initialization FAILED!");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("INIT FAILED!");
        display.display();
        while (1) delay(1000);
    }

    Serial.println("[Setup] central node ready!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("CENTRAL NODE");
    display.println("Ready!");
    display.println("");
    display.println("Waiting for packets...");
    display.display();
    delay(2000);

    alertQueue = xQueueCreate(10, sizeof(AlertData));
    if (alertQueue == NULL) {
        Serial.println("ERRORE: Coda non creata!");
        display.println("QUEUE FAIL!");
        display.display();
        while(1);
    }

    if (!initLoRaCommTask()) {
        Serial.println("ERRORE: Radio LoRa fallita!");
        display.println("LORA FAIL!");
        display.display();
        while(1);
    }

    xTaskCreatePinnedToCore(    connectAWS,
                                "task_aws",
                                12288,
                                NULL,
                                2,
                                NULL,
                                1
    );

    display.println("SISTEMA ONLINE!");
    display.display();
    Serial.println("Setup completato.");

    vTaskDelete(NULL);
}

void loop() {}
*/