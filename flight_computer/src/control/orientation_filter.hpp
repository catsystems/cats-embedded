/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include "arm_math.h"
#include "util/types.hpp"

struct orientation_filter_t {
  float32_t gyro_data[4];
  float32_t estimate_data[4];
  float32_t t_sampl;
  arm_matrix_instance_f32 gyro;
  arm_matrix_instance_f32 estimate;
};

/* Filter Functions */
void init_orientation_filter(orientation_filter_t* filter);
void reset_orientation_filter(orientation_filter_t* filter);
void quaternion_kinematics(orientation_filter_t* filter, vf32_t angular_vel);
