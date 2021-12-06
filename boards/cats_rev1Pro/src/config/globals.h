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

#include "util/types.h"
#include "util/recorder.h"
#include "util/fifo.h"
#include "sensors/icm20601.h"
#include "sensors/ms5607.h"
#include "sensors/mmc5983ma.h"
#include "sensors/h3lis100dl.h"
#include "drivers/spi.h"
#include "drivers/sbus.h"
#include "drivers/buzzer.h"
#include "drivers/servo.h"
#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "sensor_config.h"

/** Sampling Frequencies **/
#define CONTROL_SAMPLING_FREQ 100

#define RECEIVER_SAMPLING_FREQ 50

#define USB_OUTPUT_BUFFER_SIZE 256
#define USB_INPUT_BUFFER_SIZE  256

/** Device Handles **/

extern SPI_BUS SPI2_FLASH;

extern const ICM20601 ICM1;
extern const ICM20601 ICM2;
extern const ICM20601 ICM3;

extern MS5607 MS1;
extern MS5607 MS2;
extern MS5607 MS3;

extern MMC5983MA MAG;

extern const H3LIS100DL ACCEL;

extern BUZ BUZZER;

extern SERVO SERVO1;
extern SERVO SERVO2;

/** Data streams **/

extern fifo_t usb_input_fifo;
extern uint8_t usb_fifo_in_buffer[USB_INPUT_BUFFER_SIZE];
extern fifo_t usb_output_fifo;
extern uint8_t usb_fifo_out_buffer[USB_OUTPUT_BUFFER_SIZE];

/** State Estimation **/
/* Index of high G Accel */
extern baro_data_t global_baro[NUM_BARO];
extern imu_data_t global_imu[NUM_IMU];
extern accel_data_t global_accel[NUM_ACCELEROMETER];
extern magneto_data_t global_magneto[NUM_MAGNETO];

extern state_estimation_input_t global_estimation_input;
extern SI_data_t global_SI_data;
extern flight_fsm_t global_flight_state;
extern drop_test_fsm_t global_drop_test_state;
extern estimation_output_t global_estimation_data;
extern dt_telemetry_trigger_t dt_telemetry_trigger;

/** Timers **/
extern cats_timer_t ev_timers[NUM_TIMERS];
extern cats_timer_t mach_timer;

/** Recorder Queue **/
extern osMessageQueueId_t rec_queue;
extern osMessageQueueId_t rec_cmd_queue;
extern osMessageQueueId_t event_queue;

/** Tracing Channels **/

#if (configUSE_TRACE_FACILITY == 1)
#include "trcRecorder.h"
extern traceString baro_channel;
extern traceString flash_channel;
#endif

extern volatile bool global_usb_detection;
extern volatile bool usb_communication_complete;

/* recorder status is controlled by output functions, do not set manually! */
extern volatile recorder_status_e global_recorder_status;

extern event_action_map_elem_t* event_action_map;

extern const char* code_version;
