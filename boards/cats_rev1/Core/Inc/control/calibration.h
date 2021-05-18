/*
 * calibration.h
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#pragma once

#include "util/types.h"

void calibrate_imu(imu_data_t *imu_data, calibration_data_t *, sensor_elimination_t *elimination);
void calibrate_magneto(magneto_data_t *magneto_data, magneto_calibration_data_t *calibration_data);
