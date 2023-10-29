/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "config/globals.hpp"

#include "comm/fifo.hpp"
#include "drivers/spi.hpp"
#include "target.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
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

char telemetry_code_version[20] = {};
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
