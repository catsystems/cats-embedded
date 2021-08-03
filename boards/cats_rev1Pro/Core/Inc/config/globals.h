//
// Created by stoja on 28.12.20.
//

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
extern uint8_t usb_fifo_in_buffer[64];
extern fifo_t usb_output_fifo;
extern uint8_t usb_fifo_out_buffer[256];

/** State Estimation **/
/* Index of high G Accel */
extern baro_data_t global_baro[NUM_BARO];
extern imu_data_t global_imu[NUM_IMU];
extern accel_data_t global_accel;
extern magneto_data_t global_magneto;
extern flight_fsm_t global_flight_state;
extern drop_test_fsm_t global_drop_test_state;
extern sensor_elimination_t global_elimination_data;
extern estimation_output_t global_kf_data;
extern dt_telemetry_trigger_t dt_telemetry_trigger;

/** Timers **/
extern uint32_t num_timers;
extern cats_timer_t ev_timers[8];

/** Recorder Queue **/
extern osMessageQueueId_t rec_queue;
extern osMessageQueueId_t event_queue;

/** Tracing Channels **/

#if (configUSE_TRACE_FACILITY == 1)
#include "tracing/trcRecorder.h"
extern traceString baro_channel;
extern traceString flash_channel;
#endif

extern volatile bool global_usb_detection;
extern volatile bool usb_communication_complete;

/* recorder status is controlled by output functions, do not set manually! */
extern volatile recorder_status_e global_recorder_status;

extern event_action_map_elem_t *event_action_map;
