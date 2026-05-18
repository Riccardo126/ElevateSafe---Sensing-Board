#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "secrets.h"

void connectAWS(void *pvParameters);
void message_publish(void *pvParameters);

extern QueueHandle_t alertQueue;