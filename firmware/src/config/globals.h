/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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

#include "drivers/buzzer.h"
#include "drivers/servo.h"
#include "drivers/spi.h"
#include "flash/recorder.h"
#include "sensors/h3lis100dl.h"
#if IMU_TYPE == ICM20601_TYPE
#include "sensors/icm20601.h"
#elif IMU_TYPE == LSM6DSR_TYPE
#include "sensors/lsm6dsr.h"
#endif
#include "sensors/mmc5983ma.h"
#include "sensors/ms5607.h"
#include "target.h"
#include "util/types.h"

/** Sampling Frequencies **/
#define CONTROL_SAMPLING_FREQ 100  // in Hz

#define TELEMETRY_SAMPLING_FREQ 10  // in Hz

/** Device Handles **/

extern SPI_BUS SPI2_FLASH;

#if IMU_TYPE == ICM20601_TYPE
extern const ICM20601 IMU_DEV[NUM_IMU];
#elif IMU_TYPE == LSM6DSR_TYPE
extern LSM6DSR IMU_DEV[NUM_IMU];
#endif
extern MS5607 BARO_DEV[NUM_BARO];

extern MMC5983MA MAG;

extern const H3LIS100DL ACCEL;

extern BUZ BUZZER;

extern SERVO SERVO1;
extern SERVO SERVO2;

/** State Estimation **/

extern baro_data_t global_baro_sim[NUM_BARO];
extern imu_data_t global_imu_sim[NUM_BARO];

extern osEventFlagsId_t fsm_flag_id;

extern bool global_arming_bool;

/** Timers **/
extern cats_timer_t ev_timers[NUM_TIMERS];

/** Recorder Queue **/
extern osMessageQueueId_t rec_queue;
extern osMessageQueueId_t rec_cmd_queue;
extern osMessageQueueId_t event_queue;

extern volatile bool global_usb_detection;
extern volatile bool usb_device_initialized;
extern volatile bool usb_communication_complete;
extern volatile bool simulation_started;

/* recorder status is controlled by output functions, do not set manually! */
extern volatile recorder_status_e global_recorder_status;

extern event_action_map_elem_t* event_action_map;

extern const char* code_version;
