//
// Created by jonas on 4/29/2022.
//
#include "sim.h"
#include <stdlib.h>
#include "config/globals.h"

/* DataPoints Acceleration Rocket*/
timestamp_t acc_time_array[5] = {0, 9000000, 9000000, 9000000, 9000000};
float32_t acc_array[5] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/* DataPoints Pressure Rocket*/
timestamp_t pressure_time_array[10] = {0, 9000000, 9000000, 9000000, 9000000, 9000000, 9000000, 9000000, 9000000, 9000000};
float32_t pressure_array[10] = {98000.0f, 98000.0f, 98000.0f, 98000.0f, 98000.0f, 98000.0f, 98000.0f, 98000.0f, 98000.0f, 98000.0f};

/* DataPoints Acceleration Rocket*/
timestamp_t acc_rocket_time_array[3] = {20000, 21000, 9000000};
float32_t acc_rocket_array[3] = {1.0f, 10.0f, 0.0f};

/* DataPoints Pressure Rocket*/
timestamp_t pressure_rocket_time_array[8] = {0, 20000, 23000, 26000, 28000, 48000, 70000, 9000000};
float32_t pressure_rocket_array[8] = {98000.0f, 98000.0f, 96000.0f, 94600.0f, 94000.0f, 96500.0f, 98000.0f, 98000.0f};

/* DataPoints Acceleration Hop*/
timestamp_t acc_hop_time_array[3] = {15000, 15500, 9000000};
float32_t acc_hop_array[3] = {1.0f, 4.0f, 0.0f};

/* DataPoints Pressure */
timestamp_t pressure_hop_time_array[4] = {0, 15500, 17000, 9000000};
float32_t pressure_hop_array[4] = {98000.0f, 98000.0f, 96500.0f, 98000.0f};

void init_simulation_data(){
    if(global_cats_sim_choice == SIM_HOP){
      memcpy(&acc_time_array[0], &acc_hop_time_array[0], 3 * sizeof(timestamp_t));
      memcpy(&acc_array[0], &acc_hop_array[0], 3 * sizeof(float32_t));
      memcpy(&pressure_time_array[0], &pressure_hop_time_array[0], 4 * sizeof(timestamp_t));
      memcpy(&pressure_array[0], &pressure_hop_array[0], 4 * sizeof(float32_t));
    }
    else if(global_cats_sim_choice == SIM_300M){
      memcpy(&acc_time_array[0], &acc_rocket_time_array[0], 3 * sizeof(timestamp_t));
      memcpy(&acc_array[0], &acc_rocket_array[0], 3 * sizeof(float32_t));
      memcpy(&pressure_time_array[0], &pressure_rocket_time_array[0], 8 * sizeof(timestamp_t));
      memcpy(&pressure_array[0], &pressure_rocket_array[0], 8 * sizeof(float32_t));
    }
}

int32_t linear_interpol(float32_t time, float32_t LB_time, float32_t UB_time, float32_t LB_val, float32_t UB_val){
  return (int32_t)(((time - LB_time)/(UB_time - LB_time))*(UB_val - LB_val) + LB_val);
}


int32_t rand_bounds(int32_t lower_b, int32_t upper_b){
  return rand() % (upper_b-lower_b) - lower_b;
}

void start_simulation(char *args){
  if(strcmp(args, "hop") == 0){
    global_cats_sim_choice = SIM_HOP;
      simulation_started = true;
  }
  else if(strcmp(args, "300m") == 0){
    global_cats_sim_choice = SIM_300M;
      simulation_started = true;
  }
  else{
    global_cats_sim_choice = SIM_INVALID;
  }
}

