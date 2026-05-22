#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void connectAWS(void *pvParameters);

extern QueueHandle_t alertQueue;