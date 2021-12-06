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

#include "config/sensor_config.h"
#include "util/types.h"

/* Todo: check divisions */
/* CAREFUL: always add acclerometer settings after IMU settings */
sens_info_t acc_info[NUM_IMU + NUM_ACCELEROMETER] = {[0].sens_type = ICM20601_ID_ACC,
                                                     [0].upper_limit = 32.0f,
                                                     [0].lower_limit = -32.0f,
                                                     [0].resolution = 1.0f,
                                                     [0].conversion_to_SI = 9.81f / 1024.0f,
                                                     [1].sens_type = ICM20601_ID_ACC,
                                                     [1].upper_limit = 32.0f,
                                                     [1].lower_limit = -32.0f,
                                                     [1].resolution = 1.0f,
                                                     [1].conversion_to_SI = 9.81f / 1024.0f,
                                                     [2].sens_type = H3LIS100DL_ID,
                                                     [2].upper_limit = 100.0f,
                                                     [2].lower_limit = -100.0f,
                                                     [2].resolution = 1.0f,
                                                     [2].conversion_to_SI = 7.6640625f};

sens_info_t gyro_info[NUM_IMU] = {[0].sens_type = ICM20601_ID_GYRO,
                                  [0].upper_limit = 2000.0f,
                                  [0].lower_limit = -2000.0f,
                                  [0].resolution = 1.0f,
                                  [0].conversion_to_SI = 1.0f / 16.4f,
                                  [1].sens_type = ICM20601_ID_GYRO,
                                  [1].upper_limit = 2000.0f,
                                  [1].lower_limit = -2000.0f,
                                  [1].resolution = 1.0f,
                                  [1].conversion_to_SI = 1.0f / 16.4f};

sens_info_t mag_info[NUM_MAGNETO] = {[0].sens_type = MMC5983MA_ID,
                                     [0].upper_limit = 2.0f,
                                     [0].lower_limit = 2.0f,
                                     [0].resolution = 1.0f,
                                     [0].conversion_to_SI = 1.0f};

sens_info_t baro_info[NUM_BARO] = {
    [0].sens_type = MS5607_ID,   [0].upper_limit = 200000.0f, [0].lower_limit = 10.0f,     [0].resolution = 1.0f,
    [0].conversion_to_SI = 1.0f, [1].sens_type = MS5607_ID,   [1].upper_limit = 200000.0f, [1].lower_limit = 10.0f,
    [1].resolution = 1.0f,       [1].conversion_to_SI = 1.0f, [2].sens_type = MS5607_ID,   [2].upper_limit = 200000.0f,
    [2].lower_limit = 10.0f,     [2].resolution = 1.0f,       [2].conversion_to_SI = 1.0f};
