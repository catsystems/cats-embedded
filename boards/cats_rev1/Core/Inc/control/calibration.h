/*
 * calibration.h
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#pragma once

#include "util/types.h"

void calibrate_imu(imu_data_t *imu_data, calibration_data_t *, sensor_elimination_t *elimination);
