//
// Created by stoja on 28.12.20.
//

#ifndef CATS_REV1_GLOBALS_H
#define CATS_REV1_GLOBALS_H

#include "util/types.h"
#include "util/recorder.h"
#include "drivers/icm20601.h"
#include "drivers/ms5607.h"
#include "drivers/buzzer.h"
#include "drivers/servo.h"
#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"

/** Sampling Frequencies **/
#define CONTROL_SAMPLING_FREQ 100

/** Device Handles **/

extern const ICM20601 ICM1;
extern const ICM20601 ICM2;
extern const ICM20601 ICM3;

extern MS5607 MS1;
extern MS5607 MS2;
extern MS5607 MS3;

extern BUZ BUZZER;

extern SERVO SERVO1;
extern SERVO SERVO2;

/** State Estimation **/

extern baro_data_t global_baro[3];
extern imu_data_t global_imu[3];
extern flight_fsm_t global_flight_state;
extern estimation_output_t global_kf_data;

/** Initialization End Time **/

extern timestamp_t init_end_time;

#ifdef FLASH_TESTING
/** Recorder Queue **/

extern osMessageQueueId_t rec_queue;
#endif

/** Tracing Channels **/

#if (configUSE_TRACE_FACILITY == 1)
#include "tracing/trcRecorder.h"
extern traceString baro_channel;
extern traceString flash_channel;
#endif

extern uint8_t usb_receive_buffer[APP_RX_DATA_SIZE];
extern volatile bool usb_msg_received;
extern volatile bool usb_communication_complete;

#endif  // CATS_REV1_GLOBALS_H
