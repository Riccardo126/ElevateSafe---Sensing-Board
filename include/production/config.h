#pragma once
#ifndef CONFIG_H
#define CONFIG_H

// Struct for our sensor data (needed for STREAM_BUFFER_SIZE)
struct SensorData {
  float accelXYZ[3];  // [0]=X, [1]=Y, [2]=Z
  int floorHall;
  bool anomaly;
};


#define SAMPLE_RATE_HZ 10 //max 1600 for LSMDS3
#define SAMPLES_PER_BLOCK 50
#define TIMER_PERIOD_US (1000000 / SAMPLE_RATE_HZ)  // 1000 us = 1ms
#define STREAM_BUFFER_SIZE (SAMPLES_PER_BLOCK * sizeof(SensorData) * 2)
#define FILTER_TYPE 2 // tipo di filtraggio:  = nessun filtro (raw), 1 = hampel, 2 = filtri tutto EMA, 3 = solo anomalie EMA 
#define DEADBANDING true // se true, setta a zero i valori di accelerazione inferiori a 0.05g (per ridurre rumore e drift)
#define DEADBAND_THR 0.05f
#endif