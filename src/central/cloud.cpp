#include "cloud.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <time.h>
#include "secrets.h"
#include "lora.h"

#define MQTT_LOOP (50 / portTICK_PERIOD_MS)
#define RETRY_DELAY (2000 / portTICK_PERIOD_MS)

#define AWS_IOT_PUBLISH_TOPIC "elevateSafe/alerts"
#define AWS_IOT_SUBSCRIBE_TOPIC "elevateSafe/commands"

TaskHandle_t task_publish = NULL;

WiFiClientSecure wifiClient = WiFiClientSecure();
PubSubClient mqttClient(wifiClient);

// --- CALLBACK ---
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
        Serial.print("[MQTT] Failed to deserialize JSON: ");
        Serial.println(error.c_str());
        return;
    }
    
    const char* command = doc["message"];
    
    Serial.print("[MQTT] Received command: ");
    Serial.println(command ? command : "Unknown");
}

void syncTime() {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print("[WiFi Task] Sincronizzazione ora...");
    time_t now = time(nullptr);
    while (now < 8 * 3600 * 2) { 
        delay(500);
        Serial.print(".");
        now = time(nullptr);
    }
    Serial.println(" OK!");
}

void message_publish(void *pvParameters) {
    vTaskDelay(RETRY_DELAY);

    StaticJsonDocument<200> doc;
    AlertData data;

    Serial.println("\n\n[Publish Task] Started, waiting for data from queue...\n\n");

    while (true) {
        if (xQueueReceive(alertQueue, &data, portMAX_DELAY) == pdTRUE) {
            
            Serial.printf("[Publish Task] Ricevuto da LoRa: Elevator ID=%u, Alarm=%u\n", data.elev_id, data.alarm);
            
            doc.clear();
            doc["elevator_id"] = data.elev_id;
            doc["alarm"] = data.alarm;
            
            char out[256];
            memset(out, 0, sizeof(out)); // Pulisce la memoria
            serializeJson(doc, out);
            
            if (mqttClient.connected()) {
                if(mqttClient.publish(AWS_IOT_PUBLISH_TOPIC, out)) {
                    Serial.println("[MQTT] Inviato ad AWS con successo!");
                } else {
                    Serial.println("[MQTT] Errore di invio.");
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void connectAWS(void *pvParameters) {
    Serial.println("[WiFi Task] Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    syncTime();

    wifiClient.setCACert(AWS_CERT_CA);
    wifiClient.setCertificate(AWS_CERT_CRT);
    wifiClient.setPrivateKey(AWS_CERT_PRIVATE);

    mqttClient.setBufferSize(512);
    mqttClient.setServer(AWS_IOT_ENDPOINT, 8883);
    mqttClient.setCallback(mqtt_callback);

    Serial.println("\n[WiFi Task] Connecting to AWS IoT\n");

    String clientId = String(THINGNAME) + "-" + String(random(1000, 9999));

    while (!mqttClient.connect(THINGNAME)) {
        Serial.print(".");
        delay(2000);
    }
 
    if (!mqttClient.connected()) {
        Serial.println("AWS IoT Timeout!");
        vTaskDelete(NULL);
    }
 
    mqttClient.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    Serial.println("AWS IoT Connected!");

    xTaskCreate(message_publish, "task_publish", 4096, NULL, 4, &task_publish);

    while (true) {
        mqttClient.loop();
        vTaskDelay(MQTT_LOOP);
    }
}
