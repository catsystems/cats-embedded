//
// Created by stoja on 26.12.20.
//

#ifndef CATS_REV1_RECORDER_H
#define CATS_REV1_RECORDER_H

#include "util/types.h"
#include "cmsis_os.h"

/** RECORDER TYPES **/

typedef enum {
  IMU0 = 1,
  IMU1,
  IMU2,
  BARO0,
  BARO1,
  BARO2,
  FLIGHT_INFO,
  FLIGHT_STATE
} rec_entry_type_e;

typedef struct {
  timestamp_t ts;
  float height;
  float velocity;
} flight_info_t;

typedef struct {
  timestamp_t ts;
  flight_fsm_e flight_state;
} flight_state_t;

typedef union {
  imu_data_t imu;
  baro_data_t baro;
  flight_info_t flight_info;
  flight_state_t flight_state;
} rec_elem_u;

typedef struct {
  rec_entry_type_e rec_type;
  rec_elem_u u;
} rec_elem_t;

/** RECORDER FUNCTIONS **/

void record(rec_entry_type_e rec_type, const void *rec_value);

/** RECORDER CONSTS **/

const static uint32_t REC_QUEUE_SIZE = 256;

/** RECORDER EXTERNS **/

extern osMessageQueueId_t rec_queue;

/** RECORDER DEFINES **/

/* TODO: remove this when flash logging is working */
#ifdef FLASH_TESTING
#undef FLASH_TESTING
#endif
//#define FLASH_TESTING

#endif  // CATS_REV1_RECORDER_H
