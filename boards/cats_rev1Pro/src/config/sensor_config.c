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
#if defined(CATS_ORION)
sens_info_t acc_info[NUM_IMU + NUM_ACCELEROMETER] = {{
                                                         .sens_type = ICM20601_ID_ACC,
                                                         .conversion_to_SI = 9.81f / 1024.0f,
                                                         .upper_limit = 32.0f * 9.81f,
                                                         .lower_limit = -32.0f * 9.81f,
                                                         .resolution = 1.0f,
                                                     },
                                                     {
                                                         .sens_type = ICM20601_ID_ACC,
                                                         .conversion_to_SI = 9.81f / 1024.0f,
                                                         .upper_limit = 32.0f * 9.81f,
                                                         .lower_limit = -32.0f * 9.81f,
                                                         .resolution = 1.0f,
                                                     },
                                                     {
                                                         .sens_type = H3LIS100DL_ID,
                                                         .conversion_to_SI = 7.6640625f,
                                                         .upper_limit = 100.0f * 9.81f,
                                                         .lower_limit = -100.0f * 9.81f,
                                                         .resolution = 1.0f,
                                                     }};

sens_info_t gyro_info[NUM_IMU] = {{
                                      .sens_type = ICM20601_ID_GYRO,
                                      .conversion_to_SI = 1.0f / 16.4f,
                                      .upper_limit = 2000.0f,
                                      .lower_limit = -2000.0f,
                                      .resolution = 1.0f,
                                  },
                                  {
                                      .sens_type = ICM20601_ID_GYRO,
                                      .conversion_to_SI = 1.0f / 16.4f,
                                      .upper_limit = 2000.0f,
                                      .lower_limit = -2000.0f,
                                      .resolution = 1.0f,
                                  }};

sens_info_t mag_info[NUM_MAGNETO] = {{
    .sens_type = MMC5983MA_ID,
    .conversion_to_SI = 1.0f,
    .upper_limit = 2.0f,
    .lower_limit = 2.0f,
    .resolution = 1.0f,
}};

sens_info_t baro_info[NUM_BARO] = {{
                                       .sens_type = MS5607_ID,
                                       .conversion_to_SI = 1.0f,
                                       .upper_limit = 200000.0f,
                                       .lower_limit = 10.0f,
                                       .resolution = 1.0f,
                                   },
                                   {
                                       .sens_type = MS5607_ID,
                                       .conversion_to_SI = 1.0f,
                                       .upper_limit = 200000.0f,
                                       .lower_limit = 10.0f,
                                       .resolution = 1.0f,
                                   },
                                   {
                                       .sens_type = MS5607_ID,
                                       .conversion_to_SI = 1.0f,
                                       .upper_limit = 200000.0f,
                                       .lower_limit = 10.0f,
                                       .resolution = 1.0f,
                                   }};
#elif defined(CATS_VEGA)
sens_info_t acc_info[NUM_IMU + NUM_ACCELEROMETER] = {{.sens_type = ICM20601_ID_ACC,
                                                      .conversion_to_SI = 9.81f / 1024.0f,
                                                      .upper_limit = 32.0f * 9.81f,
                                                      .lower_limit = -32.0f * 9.81f,
                                                      .resolution = 1.0f}};

sens_info_t gyro_info[NUM_IMU] = {{.sens_type = ICM20601_ID_GYRO,
                                   .conversion_to_SI = 1.0f / 16.4f,
                                   .upper_limit = 2000.0f,
                                   .lower_limit = -2000.0f,
                                   .resolution = 1.0f}};

sens_info_t mag_info[NUM_MAGNETO] = {};

sens_info_t baro_info[NUM_BARO] = {{.sens_type = MS5607_ID,
                                    .conversion_to_SI = 1.0f,
                                    .upper_limit = 200000.0f,
                                    .lower_limit = 10.0f,
                                    .resolution = 1.0f}};
#endif
