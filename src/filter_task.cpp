#include "filter_task.h"
#include "shared.h"
#include "config.h"

// --- TASK 1.5 : Data Filtering
#define WINDOW_SIZE SAMPLES_PER_BLOCK / 2  // Moving average window size 
#define SLIDING_STEP WINDOW_SIZE / 3 // Number of samples to slide the window each time
#define ALPHA 0.2f  // peso exponential moving average (0=lento, 1=veloce)
#define ANOMALY_THRESHOLD_X 4 // soglia di deviazione per segnalare anomalie (in g?) 
#define ANOMALY_THRESHOLD_Y 4 // soglia di deviazione per segnalare anomalie (in g?) 
#define ANOMALY_THRESHOLD_Z 20 // soglia di deviazione per segnalare anomalie (in g?) 

void FilterTask(void *pvParameters) {
    // buffer circolare
    float windowX[WINDOW_SIZE] = {0};
    float windowY[WINDOW_SIZE] = {0};
    float windowZ[WINDOW_SIZE] = {0};


    int head = 0;          // indice del prossimo elemento da sovrascrivere
    int filled = 0;        // quanti elementi validi ci sono (fino a WINDOW_SIZE)
    
    float windowSumX = 0;   // somma corrente della finestra
    float windowSumY = 0;   // somma corrente della finestra
    float windowSumZ = 0;   // somma corrente della finestra
    
    // exponential moving average — tiene memoria oltre la finestra
    float emaX = 0;
    float emaY = 0;
    float emaZ = 0;
    bool ema_initialized = false;

    // contatore per il passo sliding
    int stepCount = 0;

    SensorData block;  // ricevi la struct intera

    while (1) {
        // ricevi un singolo SensorData dallo stream buffer
        size_t received = xStreamBufferReceive(
            sensorStreamBuffer,
            &block,
            sizeof(SensorData),
            portMAX_DELAY
        );

        if (received != sizeof(SensorData)) continue;

        float newValueX = block.accelXYZ[0];  // asse X
        float newValueY = block.accelXYZ[1];  // asse Y
        float newValueZ = block.accelXYZ[2];  // asse Z

        // --- aggiorna la finestra circolare ---
        // rimuovi il valore che stai sovrascrivendo dalla somma
        windowSumX -= windowX[head];
        windowSumY -= windowY[head];
        windowSumZ -= windowZ[head];
        
        // inserisci il nuovo valore
        windowX[head] = newValueX;
        windowY[head] = newValueY;
        windowZ[head] = newValueZ;
        
        windowSumX += newValueX;
        windowSumY += newValueY;
        windowSumZ += newValueZ;

        // avanza la testa
        head = (head + 1) % WINDOW_SIZE;
        if (filled < WINDOW_SIZE) filled++;

        // --- exponential moving average ---
        if (!ema_initialized && filled == WINDOW_SIZE) {
            // inizializza EMA sulla prima finestra completa
            // così parte già da un valore stabile e coerente con la window
            emaX = windowSumX / WINDOW_SIZE;
            emaY = windowSumY / WINDOW_SIZE;
            emaZ = windowSumZ / WINDOW_SIZE;
            ema_initialized = true;
        } else if (ema_initialized) {
            emaX = ALPHA * newValueX + (1.0f - ALPHA) * emaX;
            emaY = ALPHA * newValueY + (1.0f - ALPHA) * emaY;
            emaZ = ALPHA * newValueZ + (1.0f - ALPHA) * emaZ;
        }

        // --- sliding step: processa solo ogni SLIDING_STEP elementi ---
        stepCount++;
        if (stepCount < SLIDING_STEP) continue;  // aspetta il prossimo step
        stepCount = 0;
        if (filled < WINDOW_SIZE || !ema_initialized) continue;  // aspetta di avere una finestra piena e EMA inizializzata


        // --- calcola statistiche sulla finestra corrente ---
        float windowAvgX = (filled > 0) ? windowSumX / filled : 0;
        float windowAvgY = (filled > 0) ? windowSumY / filled : 0;
        float windowAvgZ = (filled > 0) ? windowSumZ / filled : 0;

        // anomaly: confronta EMA (trend lento) con media della finestra (trend recente)
        float deviationX = fabsf(windowAvgX - emaX);
        float deviationY = fabsf(windowAvgY - emaY);
        float deviationZ = fabsf(windowAvgZ - emaZ);

        if (deviationX > ANOMALY_THRESHOLD_X || deviationY > ANOMALY_THRESHOLD_Y || deviationZ > ANOMALY_THRESHOLD_Z) {
            debugPrint("[ANOMALY] dev=%.3f %.3f %.3f avg=%.3f %.3f %.3f ema=%.3f %.3f %.3f\n",
                       deviationX, deviationY, deviationZ,
                       windowAvgX, windowAvgY, windowAvgZ,
                       emaX, emaY, emaZ);
            // qui segnali l'anomalia — semaforo, queue, flag globale
            
        } else {
            // valore normale — puoi mandarlo al comm task
            //debugPrint("[FILTER] OK avg=%.3f %.3f %.3f ema=%.3f %.3f %.3f\n", windowAvgX, windowAvgY, windowAvgZ, emaX, emaY, emaZ);
            // ricrea una struct SensorData con i valori filtrati (es. media della finestra) e inviala al comm task
            SensorData filteredData;
            //filteredData.timestamp = block.timestamp;  // copia il timestamp
            filteredData.accelXYZ[0] = windowAvgX;
            filteredData.accelXYZ[1] = windowAvgY;
            filteredData.accelXYZ[2] = windowAvgZ;
            // invia i dati filtrati al task di comunicazione
            xStreamBufferSend(filteredSensorStreamBuffer, &filteredData, sizeof(SensorData), 0);
        }
    }
}