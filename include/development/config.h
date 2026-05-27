#pragma once
#ifndef CONFIG_H
#define CONFIG_H

// Struct for our sensor data (needed for STREAM_BUFFER_SIZE)
struct SensorData {
  float accelXYZ[3];  // [0]=X, [1]=Y, [2]=Z
  int floorHall;
  bool anomaly;
};

// Struct for anomaly data
struct CommData {
  uint8_t anomalyType; // 0=normal data, 1=emergency malfunction, 2=warning vibration, 3=info
  uint8_t elevatorID;
};

#define SAMPLE_RATE_HZ 1000 //max 1600 for LSMDS3
#define SAMPLES_PER_BLOCK 50
#define TIMER_PERIOD_US (1000000 / SAMPLE_RATE_HZ)  // 1000 us = 1ms
#define STREAM_BUFFER_SIZE (SAMPLES_PER_BLOCK * sizeof(SensorData) * 2)
#define FILTER_TYPE 2 // tipo di filtraggio:  = nessun filtro (raw), 1 = hampel, 2 = filtri tutto EMA, 3 = solo anomalie EMA 
#define DEADBANDING true // se true, setta a zero i valori di accelerazione inferiori a 0.05g (per ridurre rumore e drift)
#define DEADBAND_THR 0.05f
#endif