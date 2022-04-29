//
// Created by jonas on 4/29/2022.
//
#include "sim.h"
#include <stdlib.h>
#include "config/globals.h"


/* DataPoints Acceleration */
timestamp_t acceleration_time_array[5] = {20000, 21000, 9000000, 9000000, 9000000};
float32_t acceleration_array[5] = {1.0f, 10.0f, 0.0f, 0.0f, 0.0f};

/* DataPoints Pressure */
timestamp_t pressure_time_array[10] = {0, 20000, 23000, 26000, 28000, 48000, 70000, 9000000, 9000000, 9000000};
float32_t pressure_array[10] = {98000.0f, 98000.0f, 96000.0f, 94600.0f, 94000.0f, 96500.0f, 98000.0f, 98000.0f, 98000.0f, 98000.0f};


int32_t linear_interpol(float32_t time, float32_t LB_time, float32_t UB_time, float32_t LB_val, float32_t UB_val){
  return (int32_t)(((time - LB_time)/(UB_time - LB_time))*(UB_val - LB_val) + LB_val);
}


int32_t rand_bounds(int32_t lower_b, int32_t upper_b){
  return rand() % (upper_b-lower_b) - lower_b;
}

void start_simulation(){
  simulation_started = true;
}

