//
// Created by jonas on 4/29/2022.
//

#pragma once

#include "util/types.h"

int32_t linear_interpol(float32_t time, float32_t LB_time, float32_t UB_time, float32_t LB_val, float32_t UB_val);

int32_t rand_bounds(int32_t lower_b, int32_t upper_b);

void start_simulation();

extern timestamp_t acceleration_time_array[5];
extern float32_t acceleration_array[5];

extern timestamp_t pressure_time_array[10];
extern float32_t pressure_array[10];

