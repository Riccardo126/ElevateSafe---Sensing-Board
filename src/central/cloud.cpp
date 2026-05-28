#include "cloud.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <time.h>
#include "secrets.h"
#include "lora.h"

#define MQTT_LOOP (50 / portTICK_PERIOD_MS)
#define AWS_IOT_PUBLISH_TOPIC "elevateSafe/alerts"
#define AWS_IOT_SUBSCRIBE_TOPIC "elevateSafe/commands"

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

    while (!mqttClient.connect(clientId.c_str())) {
        Serial.print(".");
        delay(1000);
    }
 
    if (!mqttClient.connected()) {
        Serial.println("AWS IoT Timeout!");
        vTaskDelete(NULL);
    }
 
    mqttClient.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    Serial.println("AWS IoT Connected!");
    Serial.println("[WiFi Task] Pronti a ricevere dati da LoRa e pubblicare...");

    // VARIABILE PER LA CODA
    AlertData data;
    StaticJsonDocument<200> doc;
    char out[256];

    while (true) {
        // Mantieni viva la connessione MQTT in modo sicuro
        mqttClient.loop();

        // Leggi dalla coda SENZA BLOCCARE il task (0 delay)
        if (xQueueReceive(alertQueue, &data, 0) == pdTRUE) {
            Serial.printf("[WiFi Task] Ricevuto da LoRa: Elevator ID=%u, Alarm=%u\n", data.elev_id, data.alarm);
            
            doc.clear();
            doc["elevator_id"] = data.elev_id;
            doc["alarm"] = data.alarm;
            
            memset(out, 0, sizeof(out));
            serializeJson(doc, out);
            
            if (mqttClient.connected()) {
                if(mqttClient.publish(AWS_IOT_PUBLISH_TOPIC, out)) {
                    Serial.println("[MQTT] Inviato ad AWS con successo!");
                } else {
                    Serial.println("[MQTT] Errore di invio.");
                }
            }
        }
        
        vTaskDelay(MQTT_LOOP);
    }
}