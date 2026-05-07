```cpp
/* Define the item type */
typedef struct {
    uint32_t timestamp_ms;
    float value;
    uint8_t sensor_id;
} SensorSample_t; /* fixed, known size */

/* Create the queue for this type */
QueueHandle_t q = xQueueCreate(8, sizeof(SensorSample_t));

/* Send: a *copy* of 'sample' is written into the queue buffer */
SensorSample_t sample = {
    .timestamp_ms = xTaskGetTickCount(),
    .value = sensor_read(),
    .sensor_id = 1
};
xQueueSend(q, &sample, portMAX_DELAY); /* safe: stack var OK */

/* Receive: copy written into 'rx' -- no shared memory */
SensorSample_t rx;
xQueueReceive(q, &rx, portMAX_DELAY);



// Blocking Receive Consumer blocks until data is available -- no busy-wait
float v;
xQueueReceive(q, &v, portMAX_DELAY);
/* resumes here once an itemis enqueued by the producer */

// Blocking Send (back-pressure) Producer blocks when queueis full -- natural throttle
xQueueSend(q, &sample, portMAX_DELAY);
/* resumes once the frees a slot */



ISR context
/* Must use FromISR variants */
BaseType_t woken = pdFALSE;
xQueueSendFromISR(q, &v, &woken);
/* Yield for higher-priority tasks */
portYIELD_FROM_ISR(woken);



/* Queue handle shared between tasks */
static QueueHandle_t sensorQueue;
/* Sensor task: samples every PERIOD_MS milliseconds */
void vSensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(500); /* 500 ms */
    for (;;) {
        /* 1. Read raw value from the sensor driver */
        float sample = sensor_read();
        /* 2. Push to queue { wait up to 10 ms if full */
        if (xQueueSend(sensorQueue, &sample,
                pdMS_TO_TICKS(10)) != pdPASS) {
            overflowCount++; /* log dropped sample */
        }
        /* 3. Sleep until next period (drift-corrected) */
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
//Note: vTaskDelayUntil keeps the sampling period stable even when sensor read() has variable execution time.



void vAggregationTask(void *pvParameters) {
    const uint8_t WINDOW = 10; /* tumbling window of 10 samples */
    float buf[WINDOW];
    uint8_t idx = 0;
    float sum = 0.0f, vmin = FLT_MAX, vmax = -FLT_MAX;
    for (;;) {
        float sample;
        /* Block until a new sample arrives */
        xQueueReceive(sensorQueue, &sample, portMAX_DELAY);
        /* Accumulate */
        buf[idx++] = sample;
        sum += sample;
        if (sample < vmin) vmin = sample;
        if (sample > vmax) vmax = sample;
        /* Emit aggregate when window is complete */
        if (idx == WINDOW) {
            WindowResult_t res = {
                .avg = sum / WINDOW,
                .min = vmin,
                .max = vmax,
                .ts = xTaskGetTickCount()
            };
            publish_result(&res); /* MQTT / UART / display */
            /* Reset for next tumbling window */
            idx = 0; sum = 0.0f;
            vmin = FLT_MAX; vmax = -FLT_MAX;
        }
    }
}



void AverageTask(void *pvParameters) {
    float sum = 0;
    int value;
    float distanceReadings[WINDOW_SIZE] = {0}; // Array to store last 5 readings
    int pos = 0; // Index for circular buffer
    int count = 0; // Counter to track the number of stored values
    while (1) {
        if (xQueueReceive(xQueue, &(value), (TickType_t)5)) {
            distanceReadings[pos] = value;
            pos = (pos + 1) % WINDOW_SIZE; // Circular buffer index
            if (count < WINDOW_SIZE) count++; // Ensure we don't exceed the array size
            // Compute the moving average
            float sum = 0;
            for (int i = 0; i < count; i++) {
                sum += distanceReadings[i];
            }
            float averageDistance = sum / count;
            printf("Average Distance: %.2f cm\n", averageDistance);
        }
    }
}


// MOVING AVERAGE SLIDING WINDOW
#define WINDOW_SIZE 5 // Moving average window size
#define ANOMALY_THRESHOLD 15 // Threshold for detecting anomalies (adjustable)
void SMA(void *pvParameters) {
    float values[WINDOW_SIZE] = {0}; // Circular buffer for the moving window
    float sum = 0;
    int count = 0;
    int index = 0;
    while (1) {
        float newValue;
        // Wait for a new value from the queue
        if (xQueueReceive(xQueue, &newValue, portMAX_DELAY)) {
            sum -= values[index]; // Remove the oldest value from the sum
            values[index] = newValue; // Add the new value
            sum += newValue;
            index = (index + 1) % WINDOW_SIZE; // Update the circular index
            if (count < WINDOW_SIZE) count++;
            float movingAvg = sum / count; // Compute the moving average
            if (abs(newValue - movingAvg) > ANOMALY_THRESHOLD) {
                Serial.print("Anomaly Detected! Value: ");
                Serial.print(newValue);
                Serial.print(" cm | Moving Average: ");
                Serial.print(movingAvg);
                Serial.println(" cm");
            } else {
                Serial.print("Normal Value: ");
                Serial.print(newValue);
                Serial.println(" cm");
            }
        }
    }
}

//MOVING AVERAGE TUMBLING WINDOW
#define WINDOW_SIZE 5 // Moving average window size
#define ANOMALY_THRESHOLD 15 // Threshold for detecting anomalies (adjustable)
void SMA(void *pvParameters) {
    float values[WINDOW_SIZE];
    float sum;
    int count;
    while (1) {
        sum = 0;
        count = 0;
        for (int i = 0; i < WINDOW_SIZE; i++) { // Retrieve 5 new values (tumbling window)
            if (xQueueReceive(xQueue, &values[i], portMAX_DELAY)) {
                sum += values[i];
                count++;
            }
        }
        float tumblingAvg = sum / count;
        for (int i = 0; i < WINDOW_SIZE; i++) { // Check each value for anomalies based on the batch average
            if (abs(values[i] - tumblingAvg) > ANOMALY_THRESHOLD) {
                Serial.print("Anomaly Detected! Value: ");
                Serial.print(values[i]);
                Serial.print(" cm | Batch Average: ");
                Serial.print(tumblingAvg);
                Serial.println(" cm");
            } else {
                Serial.print("Normal Value: ");
                Serial.print(values[i]);
                Serial.println(" cm");
            }
        }
        Serial.println("----- End of Tumbling Window -----");
    }
}


// exponential average 
#define WINDOW_SIZE 5 // Moving average window size
#define ANOMALY_THRESHOLD 15 // Threshold for detecting anomalies (adjustable)
void SMA(void *pvParameters) {
    float values[WINDOW_SIZE] = {0};
    float movingBaseline = BASELINE_DISTANCE; // Adaptive baseline
    int count = 0;
    while (1) {
        if (uxQueueMessagesWaiting(xQueue) >= WINDOW_SIZE) {
            float sum = 0, latestValue = 0;
            for (int i = 0; i < WINDOW_SIZE; i++) { // Read the last 5 values and compute moving average
                xQueueReceive(xQueue, &values[i], 0);
                sum += values[i];
                if (i == WINDOW_SIZE - 1) {
                    latestValue = values[i]; // Get the most recent value
                }
            }

            float movingAvg = sum / WINDOW_SIZE;
            movingBaseline = (movingBaseline * 0.9) + (movingAvg * 0.1); // Exponential Smoothing
            // Detect anomalies (sharp deviations from moving baseline)
            if (abs(latestValue - movingBaseline) > ANOMALY_THRESHOLD) {
                Serial.print("Anomaly Detected! Value: ");
                Serial.print(latestValue);
                Serial.print(" cm | Expected ~");
                Serial.print(movingBaseline);
                Serial.println(" cm");
            } else {
                Serial.print("Normal Value: ");
                Serial.print(latestValue);
                Serial.println(" cm");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust processing rate



#define HAMP_THRESHOLD 3.0 // Threshold for MAD-based anomaly detection
float median(float arr[], int size) {
    float temp[size];
    memcpy(temp, arr, size * sizeof(float));
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (temp[i] > temp[j]) {
                float swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }
    return (size % 2 == 0) ? (temp[size/2 - 1] + temp[size/2]) / 2.0 : temp[size/2];
}

void HampelFilterTask(void *pvParameters) {
    float values[WINDOW_SIZE] = {0}; // Circular buffer for the moving window
    int count = 0;
    int index = 0;
    while (1) {
        float newValue;
        if (xQueueReceive(xQueue, &newValue, portMAX_DELAY)) { // Wait for a new value from the queue
            values[index] = newValue; // Store new value in the circular buffer
            index = (index + 1) % WINDOW_SIZE;
            if (count < WINDOW_SIZE) count++;
            if (count < WINDOW_SIZE) continue; // Ensure full window before processing
            float medianVal = median(values, WINDOW_SIZE); // Compute median of the window
            float deviations[WINDOW_SIZE]; // Compute Median Absolute Deviation (MAD)
            for (int i = 0; i < WINDOW_SIZE; i++) {
                deviations[i] = fabs(values[i] - medianVal);
            }
            float mad = median(deviations, WINDOW_SIZE);
            float threshold = HAMP_THRESHOLD * mad; // Hampel Filter: Flag anomalies
            if (fabs(newValue - medianVal) > threshold) {
                Serial.print("Hampel Filter: Anomaly Detected! Value: ");
                Serial.print(newValue);
                Serial.print(" cm | Median: ");
                Serial.print(medianVal);
                Serial.print(" | MAD: ");
                Serial.print(mad);
                Serial.println(" cm");
            } else {
                Serial.print("Normal Value: ");
                Serial.print(newValue);
                Serial.println(" cm");
            }
        }
    }
}
```
