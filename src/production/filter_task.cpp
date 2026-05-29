#include "filter_task.h"
#include "comm_task.h"
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

void vFilterTask(void *pvParameters) {
    /* ========================================================================================================>    DA TOGLIERE
    // finestra circolare solo per Z (X e Y non servono per la media)
    float windowZ[WINDOW_SIZE] = {0};
    int   head    = 0;
    int   filled  = 0;
    float sumZ    = 0;
    */

    // Filtered data — fast for the actual value, slow for the trend
    float emaXfast = 0, emaYfast = 0, emaZfast = 0;
    float emaXslow = 0, emaYslow = 0, emaZslow = 0;
    float emaHall  = 0;

    float prevZ    = 0;       // for the jerk
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

    // STATE MACHINE VARIABLES
    bool travelingUp = false; 
    uint32_t stopTimer = 0;
    bool misalignmentChecked = false;
    uint32_t stateTimer =0;

    for (;;) {
        size_t received = xStreamBufferReceive(
            sensorStreamBuffer, &block,
            sizeof(SensorData), pdMS_TO_TICKS(10)
        );
        if (received != sizeof(SensorData)) continue;

        /* ========================================================================================================>    DA TOGLIERE
        if (FILTER_TYPE == 0) {
            // --- no filter, just pass through ---
            xStreamBufferSend(filteredSensorStreamBuffer, &block, sizeof(SensorData), 0);
            continue;
        }
        */

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


        latestAccelZ = emaZfast; // FOR DISPLAY

        /*
        if (FILTER_TYPE == 1) {
            // --- finestra circolare solo per Z ---
            sumZ -= windowZ[head];
            windowZ[head] = z;
            sumZ += z;
            head = (head + 1) % WINDOW_SIZE;
            if (filled < WINDOW_SIZE) filled++;
        }
        */

        // --- jerk: variazione brusca tra campione e campione ---
        float jerkZ = fabsf(z - prevZ);
        prevZ = z;

        /* ========================================================================================================>    DA TOGLIERE
        // --- sliding step ---
        if (++step < SLIDING_STEP) continue;
        step = 0;
        */


        // if hall is triggered = isAtFloor, we take accz as 0.00 instead of emaZfast
        bool isAtFloor = (fabsf(emaHall) > HALL_AT_FLOOR);
        float z_fsm = isAtFloor ? 0.0f : emaZfast;

        // --- XYZ anomaly detection ---
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
            anomalyY =  true;
        }

        if (fabsf(emaZslow - emaZfast) > ANOMALY_THR_Z || jerkZ > JERK_THR_Z) {
            debugPrint("[ANOMALY Z] dev=%.3f jerk=%.3f fast=%.3f slow=%.3f\n",
                fabsf(emaZslow - emaZfast), jerkZ, emaZfast, emaZslow);
            anomalyZ = true;
        }

        // --- manda sempre i dati filtrati, anomaly è solo un flag ---
        AlertData commOut{};
        
        /*
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

        */

       // ===================================================================================================   RICONTROLLA
        if (anomalyX || anomalyY || anomalyZ) { //|| anomalyZ
            uint32_t nowMs = millis();
            if (nowMs - lastAnomalyPacketMs >= ANOMALY_COOLDOWN_MS) {
                commOut.alarm = 1;
                commOut.elev_id = 1; // Placeholder for actual elevator ID
                sendAlertData(commOut);
                lastAnomalyPacketMs = nowMs;

                anomalyXY = true;
            }
        }
        // =================================================================================================    RICONTROLLA
        if (!hasPrevAccel) { // First sample, just initialize
            prevAccelZ = emaZfast;
            hasPrevAccel = true;
            continue;
        }
        
        /* =================================================================================================    RICONTROLLA
        // Trapezoidal integration for velocity, instant and cumulative, and for distance
        delta_vZ = 0.5f * (prevAccelZ + emaZfast) * dt; // Area of trapezoid for this interval
        cum_vZ = prevVelocityZ + delta_vZ; // Update cumulative velocity
        
        float commBlock[5] = { z, emaZslow, emaZfast, emaHall, cum_vZ };
        uint8_t header[] = {0xAA, 0xBB, 0xCC, 0xDD};
        Serial.write(header, sizeof(header));
        Serial.write(reinterpret_cast<const uint8_t*>(commBlock), sizeof(commBlock));
        
        prevVelocityZ = cum_vZ;
        */

        prevAccelZ = emaZfast;

        // ==========================================
        // 2.STATE MACHINE
        // ==========================================
        switch (currentState) {
            case FERMO:
                Up = false;
                Down = false;
                if (!misalignmentChecked) {
                    if (stopTimer == 0) {
                        stopTimer = millis();
                    } else if (millis() - stopTimer >= 1500) {
                        if (!isAtFloor) {
                            printf("STOPPED out of the floor\n");
                            commOut.alarm = 2;
                            commOut.elev_id = 1;
                            anomalyZmiss = true;
                            sendAlertData(commOut);
                        } else {
                            printf("STOPPED at the floor\n");
                            anomalyZmiss = false;
                            anomalyXY = false;
                        }
                        misalignmentChecked = true;
                    }
                }

                if (z_fsm > THR_UP_START) {
                    if (stateTimer == 0) {
                        stateTimer = millis();
                    } else if (millis() - stateTimer >= 20) {
                        currentState = PARTENZA;
                            commOut.alarm = 3;
                            commOut.elev_id = 1;
                        sendAlertData(commOut);
                        travelingUp = true;
                        stateTimer = 0;
                        stopTimer = 0;
                        Up = true;
                        misalignmentChecked = false;
                    }
                } else if (z_fsm < THR_DOWN_START) {
                    if (stateTimer == 0) {
                        stateTimer = millis();
                    } else if (millis() - stateTimer >= 20) {
                        currentState = PARTENZA;
                            commOut.alarm = 4;
                            commOut.elev_id = 1;
                        sendAlertData(commOut);
                        travelingUp = false;
                        stateTimer = 0;
                        stopTimer = 0;
                        Down = true;
                        misalignmentChecked = false;
                    }
                } else {
                    stateTimer = 0;
                }
                break;

            case PARTENZA:
                if (fabsf(z_fsm) < THR_UP_QUIET) {
                    if (stateTimer == 0) {
                        stateTimer = millis();
                    } else if (millis() - stateTimer >= 500) {
                        currentState = VIAGGIO_COSTANTE;
                        stateTimer = 0;
                    }
                } else {
                    stateTimer = 0;
                }
                break;

            case VIAGGIO_COSTANTE: {
                bool isBraking;
                if (travelingUp) {
                    isBraking = (z_fsm < THR_BRAKE_UP);
                } else {
                    isBraking = (z_fsm > THR_BRAKE_DOWN);
                }

                if (isBraking) {
                    if (stateTimer == 0) {
                        stateTimer = millis();
                    } else if (millis() - stateTimer >= 50) {
                        currentState = FRENATA;
                        stateTimer = 0;
                        stopTimer = 0;
                    }
                } else {
                    stateTimer = 0;
                }
                break;
            }

            case FRENATA: {
                bool stillBraking;
                if (travelingUp) {
                    stillBraking = (z_fsm < THR_BRAKE_UP);
                } else {
                    stillBraking = (z_fsm > THR_BRAKE_DOWN);
                }

                if (stillBraking) {
                    stopTimer = 0;
                } else if (fabsf(z_fsm) < THR_UP_QUIET) {
                    if (stopTimer == 0) {
                        stopTimer = millis();
                    } else if (millis() - stopTimer >= DEBOUNCE_DELAY) {
                        currentState = FERMO;
                        stopTimer = 0;
                        misalignmentChecked = false;
                    }
                } else {
                    stopTimer = 0;
                }
                break;
            }
        }

        if (printCount % 10 == 0) {
            debugPrint("%.3f\t%.3f\t%.3f\n", z, emaZslow, emaZfast);
            printCount = 0;

        }

    }
    
}

