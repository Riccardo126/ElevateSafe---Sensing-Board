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

void oledPrint(const char* line1, const char* line2, const char* line3, float value) {
      #ifdef HAS_OLED
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println(line1);
        display.println(line2);
        display.println(line3);
        display.setTextSize(2);
        display.setCursor(0, 25);
        display.print(value, 3);
        display.println(" m/s");
        display.display();
      #endif
}
#define ALPHA_VIBRATION 0.2f // for the sliding window of vibration detection

// Soglie per la Macchina a Stati 
#define THR_UP_START    0.4f    // threshold per rilevare la partenza verso l'alto
#define THR_UP_QUIET    0.15f   // threshold diventa costante
#define THR_DOWN_START  -0.2f   // threshold per rilevare la partenza verso il basso
#define THR_DOWN_QUIET  -0.12f  //threshold diventa costante
#define THR_BRAKE_UP    -0.2f    // threshold per rilevare l'inizio della frenata verso l'alto
#define THR_BRAKE_DOWN  0.15f     // threshold per rilevare l'inizio della frenata verso il basso
#define HALL_AT_FLOOR   250.0f

#define DEBOUNCE_DELAY 200 // ms, per evitare allarmi multipli di vibrazione in rapida succession

enum EventType {
    EVENT_NONE = 0,             // Nessun evento
    EVENT_VIBRATION_XY = 1,     // Vibrazione anomala (possibile urto contro le guide)
    EVENT_MISALIGNMENT = 2,     // Ascensore fermo, ma sensore di Hall non rileva il piano
    EVENT_USAGE_UP = 3,         // Partenza verso l'alto
    EVENT_USAGE_DOWN = 4        // Partenza verso il basso
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
    uint32_t oledCount = 0;
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
    //ElevState currentState = FERMO;
    bool travelingUp = false; 
    float currentThrStart = THR_UP_START;
    float currentThrQuiet = THR_UP_QUIET;
    uint32_t stopTimer = 0;
    bool misalignmentChecked = false;


    //  DEBOUNCING
    uint32_t stateTimer =0;

    for (;;) {
        size_t received = xStreamBufferReceive(
            sensorStreamBuffer, &block,
            sizeof(SensorData), pdMS_TO_TICKS(10)
        );
        if (received != sizeof(SensorData)) continue;

        uint32_t start = micros();

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


        latestAccelZ = emaZfast; // per il display

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

        float commBlock[5] = { z, emaZslow, emaZfast, emaHall, cum_vZ };
        uint8_t header[] = {0xAA, 0xBB, 0xCC, 0xDD};
        //Serial.write(header, sizeof(header));
        //Serial.write(reinterpret_cast<const uint8_t*>(commBlock), sizeof(commBlock));

        prevAccelZ = emaZfast;
        prevVelocityZ = cum_vZ;

        // ==========================================
        // 2. MACCHINA A STATI (L'Ascensore)
        // ==========================================
        switch (currentState) {
            case FERMO:
                if(!misalignmentChecked) {
                    if(stopTimer==0) stopTimer = millis();
                    else if (millis() - stopTimer >= 1500) {
                        if(!isAtFloor) {
                            debugPrintln("fermo fuori dal piano");

                            CommData commOut;
                            commOut.anomalyType = 2; // Misalignment
                            commOut.elevatorID = 1; // Placeholder for actual elevator ID
                            
                            xQueueSendToBack(commSensorQueue, &commOut, 5);
                        }
                        else {
                            debugPrintln("fermo a piano");
                        }
                        misalignmentChecked = true;
                    }
                }
                if (z_fsm > THR_UP_START) {
                    if (stateTimer == 0) stateTimer = millis();
                    else if (millis() - stateTimer >= 20) { // Deve durare > 50ms, non è uno spike
                        currentState = PARTENZA;
                        travelingUp = true;
                        stateTimer = 0;
                        stopTimer = 0;
                        misalignmentChecked = false; // Reset per la prossima fermata
                    }
                }
                else if (z_fsm < THR_DOWN_START) {
                    if (stateTimer == 0) stateTimer = millis();
                    else if (millis() - stateTimer >= 20) {
                        currentState = PARTENZA;
                        travelingUp = false;
                        stateTimer = 0;
                        stopTimer = 0;
                        misalignmentChecked = false;
                    }
                }
                else {
                    stateTimer = 0; // Il segnale è tornato normale, lo spike è ignorato
                }
                break;

            case PARTENZA:
                if(fabsf(z_fsm) < THR_UP_QUIET) {
                    if(stateTimer == 0) {
                        stateTimer = millis(); 
                    } 
                    else if (millis() - stateTimer >= 500) {
                        currentState = VIAGGIO_COSTANTE;
                        stateTimer = 0;
                    }
                } 
                else {
                    stateTimer = 0; 
                }
                break;

            case VIAGGIO_COSTANTE:
                bool isBraking;
                if (travelingUp) {
                    isBraking = (z_fsm < THR_BRAKE_UP);
                } else {
                    isBraking = (z_fsm > THR_BRAKE_DOWN);
                }

                if(isBraking) {
                    if(stateTimer == 0) stateTimer = millis();
                    else if (millis() - stateTimer >= 50) {
                        currentState = FRENATA;
                        stateTimer = 0;
                        stopTimer = 0; // reset stop timer when we enter braking state
                    }
                } else stateTimer = 0;
                break;

            case FRENATA:
                bool stillBraking;
                if (travelingUp) {
                    stillBraking = (z_fsm < THR_BRAKE_UP);
                } else {
                    stillBraking = (z_fsm > THR_BRAKE_DOWN);
                }

                if (stillBraking) stopTimer = 0;
                else if (fabsf(z_fsm) < THR_UP_QUIET) {
                    if(stopTimer == 0) stopTimer = millis();
                    else if (millis() - stopTimer >= DEBOUNCE_DELAY) {

                        currentState = FERMO;
                        stopTimer = millis();        // Avvia timer per controllo disallineamento
                        misalignmentChecked = false; // Reset per il prossimo controllo
                        debugPrintln("Ascensore fermo al piano.");
                    } else {
                        currentState = FERMO; // Rimaniamo fermi, ma non siamo al piano!
                        debugPrintln("Ascensore fermo, ma non al piano! (Possibile disallineamento)");
                    }
                }
                else stopTimer = 0;
                break;
        }


        
        printCount++;
        oledCount++;
        if (oledCount % 50 == 0) {
            /*oledPrint(
                currentState == FERMO ? "FERMO" : (currentState == PICCO_PARTENZA ? "PARTENZA" : (currentState == VIAGGIO_COSTANTE ? "VIAGGIO" : "FRENATA")),
                travelingUp ? "Verso l'ALTO" : "Verso il BASSO",
                isAtFloor ? "AL PIANO" : "IN MOVIMENTO",
                cum_vZ
            );*/
            oledCount = 0;

        }
        debugPrint("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", x, y, emaXslow, emaYslow, emaXfast, emaYfast);
  
        uint32_t stop = micros();
    }
    
}

