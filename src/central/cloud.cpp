#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <time.h>
#include <sys/time.h>
#include "secrets.h"
#include "lora.h"
#include "cloud.h"


#define MQTT_LOOP 1000 / portTICK_PERIOD_MS
#define RETRY_DELAY 2000 / portTICK_PERIOD_MS
#define WIFI_MAX_RETRIES 10

#define AWS_IOT_PUBLISH_TOPIC "elevateSafe/alerts"
#define AWS_IOT_SUBSCRIBE_TOPIC "elevateSafe/commands"

TaskHandle_t task_mqtt = NULL;
TaskHandle_t task_publish = NULL;

WiFiClientSecure wifiClient = WiFiClientSecure();
PubSubClient mqttClient(wifiClient);

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
        Serial.print("[MQTT] Failed to deserialize JSON: ");
        Serial.println(error.c_str());
        return;
    }
    const char* command = doc["command"];
    Serial.print("[MQTT] Received command: ");
    Serial.println(command);
}



void connectAWS(void *pvParameters) {
    Serial.println("[WiFi Task] Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    wifiClient.setCACert(AWS_CERT_CA);
    wifiClient.setCertificate(AWS_CERT_CRT);
    wifiClient.setPrivateKey(AWS_CERT_PRIVATE);

    mqttClient.setServer(AWS_IOT_ENDPOINT, 8883);
    mqttClient.setCallback(mqtt_callback);

    Serial.println("\n[WiFi Task] Connecting to AWS IoT\n");


    while (!mqttClient.connect(THINGNAME))
    {
        Serial.print(".");
        delay(100);
    }
 
    if (!mqttClient.connected())
    {
        Serial.println("AWS IoT Timeout!");
        return;
    }
 
    // Subscribe to a topic
    mqttClient.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
 
    Serial.println("AWS IoT Connected!");

    xTaskCreate(message_publish, "task_publish", 4096, NULL, 4, &task_publish);

}


void message_publish(void *pvParameters) {
    vTaskDelay(RETRY_DELAY);

    StaticJsonDocument<200> doc;
    AlertData data;

    Serial.println("\n\n[Publish Task] Started, waiting for data from queue...\n\n");

    while (true) {
        if (xQueueReceive(alertQueue, &data, portMAX_DELAY) == pdTRUE) {
            
            Serial.printf("[Publish Task] Received data from queue: Elevator ID=%u, Alarm=%u\n", data.elev_id, data.alarm);
            doc.clear();
            doc["elevator_id"] = data.elev_id;
            doc["alarm"] = data.alarm;
            char out[256];
            size_t len = serializeJson(doc, out);
            
            if (mqttClient.connected()) {
                mqttClient.publish(AWS_IOT_PUBLISH_TOPIC, out, len);
            }
        }
        vTaskDelay(RETRY_DELAY);
    }
}
