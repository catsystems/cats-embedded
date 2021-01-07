/*
 * calibration.h
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#ifndef INC_CONTROL_CALIBRATION_H_
#define INC_CONTROL_CALIBRATION_H_

#include "util/types.h"

void calibrate_imu(imu_data_t *imu_data, calibration_data_t *,
                   sensor_elimination_t *elimination);

#endif /* INC_CONTROL_CALIBRATION_H_ */
