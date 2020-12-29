//
// Created by stoja on 28.12.20.
//

#ifndef CATS_REV1_GLOBALS_H
#define CATS_REV1_GLOBALS_H

#include "util/types.h"
#include "drivers/icm20601.h"
#include "drivers/ms5607.h"
#include "drivers/buzzer.h"
#include "stm32l4xx_hal.h"

/** Device Handles **/

extern const ICM20601 ICM1;
extern const ICM20601 ICM2;
extern const ICM20601 ICM3;

extern MS5607 MS1;
extern MS5607 MS2;
extern MS5607 MS3;

extern BUZ BUZZER;

/** State Estimation **/

extern baro_data_t global_baro[3];
extern imu_data_t global_imu[3];
extern flight_fsm_t global_flight_state;

#endif  // CATS_REV1_GLOBALS_H
