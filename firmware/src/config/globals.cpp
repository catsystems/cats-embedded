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

#include "config/globals.hpp"

#include "comm/fifo.hpp"
#include "drivers/spi.hpp"
#include "target.h"

/** Protocol Handles **/

/** Device Handles **/

// We need these for now, but will eventually get rid of them by replacing the actions with a class
driver::Servo* global_servo1 = nullptr;
driver::Servo* global_servo2 = nullptr;

/** State Estimation **/

baro_data_t global_baro_sim[NUM_BARO] = {};
imu_data_t global_imu_sim[NUM_BARO] = {};

osEventFlagsId_t fsm_flag_id;

/** Timers **/
cats_timer_t ev_timers[NUM_TIMERS] = {};

/** Recorder Queue **/
osMessageQueueId_t rec_queue;
osMessageQueueId_t rec_cmd_queue;
osMessageQueueId_t event_queue;

volatile bool global_usb_detection = false;
volatile bool usb_device_initialized = false;
volatile bool usb_communication_complete = false;
volatile bool simulation_started = false;
volatile bool imu_initialized[NUM_IMU] = {false};

volatile recorder_status_e global_recorder_status = REC_OFF;

event_action_map_elem_t* event_action_map = nullptr;

const char* code_version = "2.3.5";
char telemetry_code_version[20] = {};
const char* board_name = "CATS Vega";
