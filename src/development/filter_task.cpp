#include "filter_task.h"
#include "shared.h"
#include "config.h"

// --- TASK 1.5 : Data Filtering
#define WINDOW_SIZE     (SAMPLES_PER_BLOCK / 4)
#define SLIDING_STEP    (WINDOW_SIZE / 4)
#define ALPHA_FAST      0.40f
#define ALPHA_SLOW      0.05f
#define ANOMALY_THR_X   1.0f
#define ANOMALY_THR_Y   1.0f
#define ANOMALY_THR_Z   2.4f
#define JERK_THR_Z      3.0f
#define ANOMALY_COOLDOWN_MS 1000

/*tipo di filtraggio: 
    0 = nessun filtro (raw), 
    1 = hampel, 
    2 = filtri tutto EMA, 
    3 = solo anomalie EMA
    */
/*void vFilterTask(void *pvParameters) {
    // finestra circolare solo per Z (X e Y non servono per la media)
    float windowZ[WINDOW_SIZE] = {0};
    int   head    = 0;
    int   filled  = 0;
    float sumZ    = 0;

    // due EMA — fast per il valore corrente, slow per il trend
    float emaXfast = 0, emaYfast = 0, emaZfast = 0;
    float emaXslow = 0, emaYslow = 0, emaZslow = 0;
    float emaHall  = 0; // per il sensore di hall, se vogliamo filtrarlo

    float prevZ    = 0;       // per il jerk
    int   step     = 0;
    SensorData block;
    uint32_t printCount = 0;
    uint32_t lastAnomalyPacketMs = 0;

    for (;;) {
        size_t received = xStreamBufferReceive(
            sensorStreamBuffer, &block,
            sizeof(SensorData), pdMS_TO_TICKS(10)
        );
        if (received != sizeof(SensorData)) continue;

        if (FILTER_TYPE == 0) {
            // --- no filter, just pass through ---
            xStreamBufferSend(filteredSensorStreamBuffer, &block, sizeof(SensorData), 0);
            continue;
        }

        float x = block.accelXYZ[0];
        float y = block.accelXYZ[1];
        float z = block.accelXYZ[2];
        float hall = block.floorHall;

        // --- aggiorna EMA (i primi campioni sono imprecisi, non importa) ---
        emaXfast = ALPHA_FAST * x + (1.0f - ALPHA_FAST) * emaXfast;
        emaYfast = ALPHA_FAST * y + (1.0f - ALPHA_FAST) * emaYfast;
        emaZfast = ALPHA_FAST * z + (1.0f - ALPHA_FAST) * emaZfast;

        emaXslow = ALPHA_SLOW * x + (1.0f - ALPHA_SLOW) * emaXslow;
        emaYslow = ALPHA_SLOW * y + (1.0f - ALPHA_SLOW) * emaYslow;
        emaZslow = ALPHA_SLOW * z + (1.0f - ALPHA_SLOW) * emaZslow;

        emaHall = ALPHA_SLOW * hall + (1.0f - ALPHA_SLOW) * emaHall;

        if (FILTER_TYPE == 1) {
            // --- finestra circolare solo per Z ---
            sumZ -= windowZ[head];
            windowZ[head] = z;
            sumZ += z;
            head = (head + 1) % WINDOW_SIZE;
            if (filled < WINDOW_SIZE) filled++;
        }
        // --- jerk: variazione brusca tra campione e campione ---
        float jerkZ = fabsf(z - prevZ);
        prevZ = z;

        // --- sliding step ---
        if (++step < SLIDING_STEP) continue;
        step = 0;

        // --- anomaly detection ---
        bool anomalyX = false;
        bool anomalyY = false;
        bool anomalyZ = false;

        if (fabsf(emaXslow - emaXfast) > ANOMALY_THR_X) {
            debugPrint("[ANOMALY X] dev=%.3f fast=%.3f slow=%.3f\n",
                fabsf(emaXslow - emaXfast), emaXfast, emaXslow);
            anomalyX = true;
        }
        if (fabsf(emaYslow - emaYfast) > ANOMALY_THR_Y) {
            debugPrint("[ANOMALY Y] dev=%.3f fast=%.3f slow=%.3f\n",
                fabsf(emaYslow - emaYfast), emaYfast, emaYslow);
            anomalyY = true;
        }
        if (fabsf(emaZslow - emaZfast) > ANOMALY_THR_Z || jerkZ > JERK_THR_Z) {
            debugPrint("[ANOMALY Z] dev=%.3f jerk=%.3f fast=%.3f slow=%.3f\n",
                fabsf(emaZslow - emaZfast), jerkZ, emaZfast, emaZslow);
            anomalyZ = true;
        }

        // --- manda sempre i dati filtrati, anomaly è solo un flag ---
        SensorData out;
        CommData commOut;
        
        if (FILTER_TYPE == 1) {
            // --- HAMPel: NON COMPLETATO
            out.accelXYZ[0] = anomalyX ? emaXslow : emaXfast;
            out.accelXYZ[1] = anomalyY ? emaYslow : emaYfast;
            out.accelXYZ[2] = anomalyZ ? (sumZ / filled) : emaZfast; // media semplice come fallback
        } else if (FILTER_TYPE == 2) {
            // --- EMA tutto ---
            out.accelXYZ[0] = emaXfast;
            out.accelXYZ[1] = emaYfast;
            out.accelXYZ[2] = emaZfast;  
        } else if (FILTER_TYPE == 3) {
            // --- solo anomalie EMA: se non anomalia, passa i dati raw ---
            out.accelXYZ[0] = anomalyX ? emaXslow : x;
            out.accelXYZ[1] = anomalyY ? emaYslow : y;
            out.accelXYZ[2] = anomalyZ ? emaZslow : z;            
        }
        out.anomaly     = anomalyX || anomalyY || anomalyZ ? true : false;
        out.floorHall = emaHall;
        
        xStreamBufferSend(filteredSensorStreamBuffer, &out, sizeof(SensorData), 5);

        if (anomalyX || anomalyY || anomalyZ) {
            uint32_t nowMs = millis();
            if (nowMs - lastAnomalyPacketMs >= ANOMALY_COOLDOWN_MS) {
                commOut.anomalyType = 1; // Placeholder for actual anomaly type
                commOut.elevatorID = 1; // Placeholder for actual elevator ID
                xQueueSendToBack(commSensorQueue, &commOut, 5);
                lastAnomalyPacketMs = nowMs;
            }
        }
        

        printCount++;
        if (printCount % 5 == 0) {
            Serial.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", x, y, z, emaXslow, emaYslow, emaZslow, emaXfast, emaYfast, emaZfast, emaHall);
            printCount = 0;
        }

    }
    
}*/

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


#define ALPHA_VIBRATION 0.2f // for the sliding window of vibration detection

// Soglie per la Macchina a Stati 
#define THR_UP_START    0.6f    // threshold per rilevare la partenza verso l'alto
#define THR_UP_QUIET    0.15f   // threshold diventa costante
#define THR_DOWN_START  -0.35f   // threshold per rilevare la partenza verso il basso
#define THR_DOWN_QUIET  -0.12f  //threshold diventa costante
#define THR_BRAKE_UP    -0.6f    // threshold per rilevare l'inizio della frenata verso l'alto
#define THR_BRAKE_DOWN  0.25f     // threshold per rilevare l'inizio della frenata verso il basso
#define HALL_AT_FLOOR   250.0f

#define DEBOUNCE_DELAY 100 // ms, per evitare allarmi multipli di vibrazione in rapida successione

enum ElevState { 
    FERMO, 
    PICCO_PARTENZA, 
    VIAGGIO_COSTANTE, 
    PICCO_FRENATA 
};

void vFilterTask(void *pvParameters) {
    // finestra circolare solo per Z (X e Y non servono per la media)
    float windowZ[WINDOW_SIZE] = {0};
    int   head    = 0;
    int   filled  = 0;
    float sumZ    = 0;

    // due EMA — fast per il valore corrente, slow per il trend
    float emaXfast = 0, emaYfast = 0, emaZfast = 0;
    float emaXslow = 0, emaYslow = 0, emaZslow = 0;
    float emaHall  = 0; // per il sensore di hall, se vogliamo filtrarlo

    float prevZ    = 0;       // per il jerk
    int   step     = 0;
    SensorData block;
    uint32_t printCount = 0;
    uint32_t lastAnomalyPacketMs = 0;

    const float dt = 1.0f / SAMPLE_RATE_HZ;
    float prevAccelZ = 0.0f;
    float prevVelocityZ = 0.0f;
    float cum_vZ = 0.0f;
    float delta_vZ = 0.0f;
    float positionZ = 0.0f;
    float maxAccelZ = 0.0f; // for now only local
    bool hasPrevAccel = false;

    // Variabili della Macchina a Stati
    ElevState currentState = FERMO;
    bool travelingUp = false; 
    float currentThrStart = THR_UP_START;
    float currentThrQuiet = THR_UP_QUIET;
    uint32_t stopTimer = 0;
    bool misalignmentChecked = false;



    for (;;) {
        size_t received = xStreamBufferReceive(
            sensorStreamBuffer, &block,
            sizeof(SensorData), pdMS_TO_TICKS(10)
        );
        if (received != sizeof(SensorData)) continue;

        if (FILTER_TYPE == 0) {
            // --- no filter, just pass through ---
            xStreamBufferSend(filteredSensorStreamBuffer, &block, sizeof(SensorData), 0);
            continue;
        }

        float x = block.accelXYZ[0];
        float y = block.accelXYZ[1];
        float z = block.accelXYZ[2];
        float hall = block.floorHall;

        // --- aggiorna EMA (i primi campioni sono imprecisi, non importa) ---
        emaXfast = ALPHA_FAST * x + (1.0f - ALPHA_FAST) * emaXfast;
        emaYfast = ALPHA_FAST * y + (1.0f - ALPHA_FAST) * emaYfast;
        emaZfast = ALPHA_FAST * z + (1.0f - ALPHA_FAST) * emaZfast;

        emaXslow = ALPHA_SLOW * x + (1.0f - ALPHA_SLOW) * emaXslow;
        emaYslow = ALPHA_SLOW * y + (1.0f - ALPHA_SLOW) * emaYslow;
        emaZslow = ALPHA_SLOW * z + (1.0f - ALPHA_SLOW) * emaZslow;

        emaHall = ALPHA_SLOW * hall + (1.0f - ALPHA_SLOW) * emaHall;

        if (FILTER_TYPE == 1) {
            // --- finestra circolare solo per Z ---
            sumZ -= windowZ[head];
            windowZ[head] = z;
            sumZ += z;
            head = (head + 1) % WINDOW_SIZE;
            if (filled < WINDOW_SIZE) filled++;
        }
        // --- jerk: variazione brusca tra campione e campione ---
        float jerkZ = fabsf(z - prevZ);
        prevZ = z;

        // --- sliding step ---
        if (++step < SLIDING_STEP) continue;
        step = 0;


        // CANCELLO PER SPIKES
        bool isAtFloor = (fabsf(emaHall) > HALL_AT_FLOOR);
        float z_fsm = isAtFloor ? 0.0f : emaZfast;

        // --- anomaly detection ---
        bool anomalyX = false;
        bool anomalyY = false;
        bool anomalyZ = false;

        if (fabsf(emaXslow - emaXfast) > ANOMALY_THR_X) {
            debugPrint("[ANOMALY X] dev=%.3f fast=%.3f slow=%.3f\n",
                fabsf(emaXslow - emaXfast), emaXfast, emaXslow);
            anomalyX = true;
        }
        if (fabsf(emaYslow - emaYfast) > ANOMALY_THR_Y) {
            debugPrint("[ANOMALY Y] dev=%.3f fast=%.3f slow=%.3f\n",
                fabsf(emaYslow - emaYfast), emaYfast, emaYslow);
            anomalyY = true;
        }
        if (fabsf(emaZslow - emaZfast) > ANOMALY_THR_Z || jerkZ > JERK_THR_Z) {
            debugPrint("[ANOMALY Z] dev=%.3f jerk=%.3f fast=%.3f slow=%.3f\n",
                fabsf(emaZslow - emaZfast), jerkZ, emaZfast, emaZslow);
            anomalyZ = true;
        }

        // --- manda sempre i dati filtrati, anomaly è solo un flag ---
        SensorData out;
        CommData commOut;
        
        if (FILTER_TYPE == 1) {
            // --- HAMPel: NON COMPLETATO
            out.accelXYZ[0] = anomalyX ? emaXslow : emaXfast;
            out.accelXYZ[1] = anomalyY ? emaYslow : emaYfast;
            out.accelXYZ[2] = anomalyZ ? (sumZ / filled) : emaZfast; // media semplice come fallback
        } else if (FILTER_TYPE == 2) {
            // --- EMA tutto ---
            out.accelXYZ[0] = emaXfast;
            out.accelXYZ[1] = emaYfast;
            out.accelXYZ[2] = emaZfast;  
        } else if (FILTER_TYPE == 3) {
            // --- solo anomalie EMA: se non anomalia, passa i dati raw ---
            out.accelXYZ[0] = anomalyX ? emaXslow : x;
            out.accelXYZ[1] = anomalyY ? emaYslow : y;
            out.accelXYZ[2] = anomalyZ ? emaZslow : z;            
        }
        out.anomaly     = anomalyX || anomalyY || anomalyZ ? true : false;
        out.floorHall = emaHall;
        
        xStreamBufferSend(filteredSensorStreamBuffer, &out, sizeof(SensorData), 5);

        if (anomalyX || anomalyY || anomalyZ) {
            uint32_t nowMs = millis();
            if (nowMs - lastAnomalyPacketMs >= ANOMALY_COOLDOWN_MS) {
                commOut.anomalyType = 1; // Placeholder for actual anomaly type
                commOut.elevatorID = 1; // Placeholder for actual elevator ID
                xQueueSendToBack(commSensorQueue, &commOut, 5);
                lastAnomalyPacketMs = nowMs;
            }
        }
        if (!hasPrevAccel) { // First sample, just initialize
            prevAccelZ = emaZfast;
            hasPrevAccel = true;
            continue;
        }
        
        // Trapezoidal integration for velocity, instant and cumulative, and for distance
        delta_vZ = 0.5f * (prevAccelZ + emaZfast) * dt; // Area of trapezoid for this interval
        cum_vZ = prevVelocityZ + delta_vZ; // Update cumulative velocity

        printCount++;
        if (printCount % 1 == 0) {
            Serial.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", z, emaZslow, emaZfast, emaHall, delta_vZ, cum_vZ);
            printCount = 0;
        }

    }
    
}

/*

void vFilterTask(void *pvParameters) {
    
    

    // Contatore per non intasare la Seriale
    uint32_t printCounter = 0; 

    for (;;) {
        

        


        // ==========================================
        // 1. RILEVAMENTO ANOMALIE VIBRAZIONE
        // ==========================================

        bool anomalyXY = (vibX > ANOMALY_THR_X || vibY > ANOMALY_THR_Y);
        bool anomalyZ  = (vibZ > ANOMALY_THR_Z || fabsf(block.accelXYZ[2] - prevZ) > JERK_THR_Z);
        prevZ = block.accelXYZ[2];

        if (fabsf(emaXslow - emaXfast) > ANOMALY_THR_X || fabsf(emaYslow - emaYfast) > ANOMALY_THR_Y || fabsf(emaZslow - emaZfast) > ANOMALY_THR_Z) {
            uint32_t nowMs = millis();
            if (nowMs - lastAnomalyMs >= 2000) { 
                CommData commOut;
                commOut.anomalyType = anomalyZ ? 2 : 1;
                commOut.elevatorID = 1;
                xStreamBufferSend(commSensorStreamBuffer, &commOut, sizeof(CommData), 0);
                debugPrintln("ALLARME VIBRAZIONE!");
                lastAnomalyMs = nowMs;
            }
        }

        // ==========================================
        // 2. MACCHINA A STATI (L'Ascensore)
        // ==========================================
        switch (currentState) {
            
            case FERMO:
                if (z_fsm > THR_UP_START) {
                    currentState = PICCO_PARTENZA;
                    travelingUp = true;
                    currentThrStart = THR_UP_START;
                    currentThrQuiet = THR_UP_QUIET;
                    debugPrintln("Partenza verso l'ALTO");
                    CommData commOut = {EVENT_USAGE_UP, 1};
                    xStreamBufferSend(commSensorStreamBuffer, &commOut, sizeof(CommData), 0);
                    
                } else if (z_fsm < THR_DOWN_START) {
                    currentState = PICCO_PARTENZA;
                    travelingUp = false;
                    currentThrStart = THR_DOWN_START;
                    currentThrQuiet = THR_DOWN_QUIET;
                    debugPrintln("Partenza verso il BASSO");
                    CommData commOut = {EVENT_USAGE_DOWN, 1};
                    xStreamBufferSend(commSensorStreamBuffer, &commOut, sizeof(CommData), 0);
                }
                
                if (millis() - stopTimer > 1500 && !misalignmentChecked && !isAtFloor) {
                    if (!isAtFloor) {
                        debugPrintln("ALLARME: FERMO MA NON AL PIANO (Disallineato)!");
                        CommData commOut = {EVENT_MISALIGNMENT, 1};
                        xStreamBufferSend(commSensorStreamBuffer, &commOut, sizeof(CommData), 0);
                    } else {
                        debugPrintln("Cabina ferma e correttamente allineata.");
                    }
                    misalignmentChecked = true; 
                }
                break;

            case PICCO_PARTENZA:
                if (travelingUp){
                    // In salita, attendiamo che l'accelerazione positiva scenda verso lo zero (quiete)
                    if (z_fsm < currentThrQuiet) {
                        currentState = VIAGGIO_COSTANTE;
                        debugPrintln("... In viaggio a velocità costante ");
                    }
                } else {
                    // In discesa, attendiamo che l'accelerazione negativa salga verso lo zero (quiete)
                    if (z_fsm > currentThrQuiet) {
                        currentState = VIAGGIO_COSTANTE;
                        debugPrintln("... In viaggio a velocità costante (0g)");
                    }
                }
                break;

            case VIAGGIO_COSTANTE:
                // --- Logica esplicita per l'inizio della frenata ---
                if (travelingUp) {
                    // In salita, freniamo quando l'accelerazione diventa negativa (decelerazione)
                    if (z_fsm < THR_BRAKE_UP) {
                        currentState = PICCO_FRENATA;
                        debugPrintln("Inizio frenata in salita...");
                    }
                } else {
                    // In discesa, freniamo quando l'accelerazione diventa positiva (decelerazione)
                    if (z_fsm > THR_BRAKE_DOWN) {
                        currentState = PICCO_FRENATA;
                        debugPrintln("Inizio frenata in discesa...");
                    }
                }
                break;

            case PICCO_FRENATA:
                // --- Logica esplicita per il raggiungimento della quiete ---
                // Attendiamo che l'accelerazione torni vicina allo zero (quiete)
                if (fabsf(z_fsm) < fabsf(currentThrQuiet)) {
                    if(isAtFloor) {
                        currentState = FERMO;
                        stopTimer = millis();        // Avvia timer per controllo disallineamento
                        misalignmentChecked = false; // Reset per il prossimo controllo
                        debugPrintln("Ascensore fermo al piano.");
                    } else {
                        debugPrintln("Ascensore fermo, ma non al piano! (Possibile disallineamento)");
                    }
                }
                break;
        }
    }
}*/