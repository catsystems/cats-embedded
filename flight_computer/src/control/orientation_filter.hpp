/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

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
