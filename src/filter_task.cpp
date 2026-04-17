#include "filter_task.h"
#include "shared.h"
#include "config.h"

// --- TASK 1.5 : Data Filtering
#define WINDOW_SIZE SAMPLES_PER_BLOCK / 4  // Moving average window size 
#define SLIDING_STEP WINDOW_SIZE / 4 // Number of samples to slide the window each time
#define ALPHA 0.35f  // peso exponential moving average (0=lento, 1=veloce)
#define ANOMALY_THRESHOLD_X 1 // soglia anomalie (solo rumore) 
#define ANOMALY_THRESHOLD_Y 1 // soglia anomalie (solo rumore) 
#define ANOMALY_THRESHOLD_Z 3 // soglia anomalie (in m/s^2, es. di scossa improvvisa) 
#define HAMP_THRESHOLD 2.0 // Threshold for MAD-based anomaly detection

/*tipo di filtraggio: 
    0 = nessun filtro (raw), 
    1 = hampel, 
    2 = filtri tutto EMA, 
    3 = solo anomalie EMA
    */
void FilterTask(void *pvParameters) {
     
    int filtertype = (int)pvParameters;
    // buffer circolare
    float windowX[WINDOW_SIZE] = {0};
    float windowY[WINDOW_SIZE] = {0};
    float windowZ[WINDOW_SIZE] = {0};


    int head = 0;          // indice del prossimo elemento da sovrascrivere
    int filled = 0;        // quanti elementi validi ci sono (fino a WINDOW_SIZE)
    
    float windowSumX = 0, windowSumY = 0, windowSumZ = 0;   // somma corrente della finestra

    // exponential moving average — tiene memoria oltre la finestra
    float emaXfast = 0, emaYfast = 0, emaZfast = 0;
    float emaXslow = 0, emaYslow = 0, emaZslow = 0;
    bool ema_initialized = false;

    // contatore per il passo sliding
    int stepCount = 0;

    SensorData block;  // ricevi la struct intera

    for (;;) {
        // ricevi un singolo SensorData dallo stream buffer
        size_t received = xStreamBufferReceive(
            sensorStreamBuffer,
            &block,
            sizeof(SensorData),
            10 / portTICK_PERIOD_MS  // aspetta 10ms
        );

        if (received != sizeof(SensorData)) continue;

        float newValueX = block.accelXYZ[0];  // asse X
        float newValueY = block.accelXYZ[1];  // asse Y
        float newValueZ = block.accelXYZ[2];  // asse Z
        float oldValueZ;

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
            // inizializza EMA fast e slow uguali sulla prima finestra completa
            // così parte già da un valore stabile e coerente con la window
            emaXfast = emaXslow = windowSumX / WINDOW_SIZE;
            emaYfast = emaYslow = windowSumY / WINDOW_SIZE;
            emaZfast = emaZslow = windowSumZ / WINDOW_SIZE;
            oldValueZ = emaZfast; // inizializza oldValueZ per il confronto di anomalie
            ema_initialized = true;
        } else if (ema_initialized) {
            emaXfast = ALPHA * newValueX + (1.0f - ALPHA) * emaXfast;
            emaYfast = ALPHA * newValueY + (1.0f - ALPHA) * emaYfast;
            emaZfast = ALPHA * newValueZ + (1.0f - ALPHA) * emaZfast;

            emaXslow = (0.08f) * newValueX + (1.0f - 0.08f) * emaXslow; // EMA più lenta
            emaYslow = (0.08f) * newValueY + (1.0f - 0.08f) * emaYslow; // EMA più lenta
            emaZslow = (0.08f) * newValueZ + (1.0f - 0.08f) * emaZslow; // EMA più lenta
        }

        float jerkZ = fabsf(oldValueZ - newValueZ); // calcola il "jerk" sull'asse Z come differenza assoluta tra il nuovo valore e il vecchio
        oldValueZ = newValueZ; // aggiorna oldValueZ per il prossimo confronto
        
        // --- sliding step: processa solo ogni SLIDING_STEP elementi ---
        stepCount++;
        if (stepCount < SLIDING_STEP) continue;  // aspetta il prossimo step
        stepCount = 0;
        if (filled < WINDOW_SIZE || !ema_initialized) continue;  // aspetta di avere una finestra piena e EMA inizializzata


        // --- calcola statistiche sulla finestra corrente ---
        float windowAvgX = (filled > 0) ? windowSumX / filled : 0;
        float windowAvgY = (filled > 0) ? windowSumY / filled : 0;
        float windowAvgZ = (filled > 0) ? windowSumZ / filled : 0;

        // anomaly: confronta EMAfast con EMAslow o con la media della finestra, se la deviazione è troppo grande segnala un'anomalia
        float deviationX = fabsf(emaXslow - emaXfast);
        float deviationY = fabsf(emaYslow - emaYfast);
        float deviationZ = fabsf(emaZslow - emaZfast);

        
        if (deviationX > ANOMALY_THRESHOLD_X) {
            debugPrint("[ANOMALY on X] dev=%.3f avg=%.3f ema=%.3f\n",
                       deviationX, windowAvgX, emaXfast);
            // sostituisci con qualcosa, ma cosa? 
        }
        if (deviationY > ANOMALY_THRESHOLD_Y) {
            debugPrint("[ANOMALY on Y] dev=%.3f avg=%.3f ema=%.3f\n",
                       deviationY, windowAvgY, emaYfast);
            // sostituisci con qualcosa, ma cosa?
        }
        if (deviationZ > ANOMALY_THRESHOLD_Z || jerkZ > 3) { // considera anche un'anomalia se c'è un jerk improvviso superiore a 3g
            debugPrint("[ANOMALY on Z] dev=%.3f avg=%.3f ema=%.3f\n",
                       deviationZ, windowAvgZ, emaZfast);
            // sostituisci con qualcosa, ma cosa?
        }
        if (true) { // per adesso mandiamo sempre
            SensorData filteredData;
            filteredData.accelXYZ[0] = emaXfast;
            filteredData.accelXYZ[1] = emaYfast;
            filteredData.accelXYZ[2] = emaZfast;

            // invia i dati filtrati al task di comunicazione
            xStreamBufferSend(filteredSensorStreamBuffer, &filteredData, sizeof(SensorData), 0);
        }
    }
}

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