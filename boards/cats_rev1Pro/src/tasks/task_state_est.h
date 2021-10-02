/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
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

_Noreturn void task_state_est(void *argument);

/* Offset Settings */
#define OFFSET_BARO
#define OFFSET_IMU
#define OFFSET_SENSOR_CHOICE 1
#define OFFSET_P             1500 /* Pa */
#define OFFSET_ACC           5    /* m/s^2 */

/* Spike Settings */
#define SPIKE_BARO
#define SPIKE_IMU
#define SPIKE_SENSOR_CHOICE 1
/* rng between 0 and 1 and it it is smaller than the threshold we inject a spike */
#define SPIKE_THRESHOLD 0.01f

/* Noise Settings */
#define ACC_NOISE_MAX_AMPL      0.2f  /* In m/s^2 */
#define PRESSURE_NOISE_MAX_AMPL 10.0f /* In Pa */
