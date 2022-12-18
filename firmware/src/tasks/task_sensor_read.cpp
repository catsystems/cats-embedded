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

#include "tasks/task_sensor_read.h"
#include "util/task_util.h"
#include "cmsis_os.h"
#include "config/globals.h"
#include "flash/recorder.h"
#include "sensors/h3lis100dl.h"

#if IMU_TYPE == LSM6DSR_TYPE

#include "sensors/lsm6dsr.h"

#elif IMU_TYPE == ICM20601_TYPE
#include "sensors/icm20601.h"
#endif

#include "sensors/mmc5983ma.h"
#include "sensors/ms5607.h"
#include "util/log.h"

#include <cstring>

/** Private Function Declarations **/

static void read_imu(vi16_t &gyroscope, vi16_t &acceleration, int32_t id);

static void prepare_temp();

static void prepare_pres();

static void get_temp_pres(int32_t *temperature, int32_t *pressure);

static void read_baro();

namespace task {

    std::optional<SensorRead> SensorRead::s_instance = std::nullopt;

/* Todo: Check with Trace if can be reduced */
    SET_TASK_PARAMS(task_sensor_read, 1024)

    void task::SensorRead::Run() {
        osThreadNew(task_sensor_read, nullptr, &task_sensor_read_attributes);
    }

//    bool task::SensorRead::PreparePressure() {
//        if (this->current_readout == task::SensorRead::BaroReadoutType::kReadBaroPressure) {
//            this->current_readout = task::SensorRead::BaroReadoutType::kReadBaroTemperature;
//            return true;
//        } else {
//            this->current_readout = task::SensorRead::BaroReadoutType::kReadBaroPressure;
//            return false;
//        }
//    }


/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_sens_read thread.
 * @param argument: Not used
 * @retval None
 */
    [[noreturn]] void task_sensor_read(void *argument [[maybe_unused]]) {
        /* Initialize IMU data variables */

        auto task = SensorRead::GetInstance();

        prepare_temp();
        osDelay(5);

        /* initialize MAGNETO data variables */
        float32_t mag_data[3] = {};

        uint32_t tick_count = osKernelGetTickCount();
        /* This task is sampled with 2 times the control sampling frequency to maximize speed of the barometer. In one
         * timestep the Baro pressure is read out and then the Baro Temperature. The other sensors are only read out one in
         * two times. */
        const uint32_t tick_update = osKernelGetTickFreq() / (2 * CONTROL_SAMPLING_FREQ);
        while (true) {
            // Readout the baro register
            read_baro();

            // Prepare new readout for the baro
            if (task.m_current_readout == SensorRead::BaroReadoutType::kReadBaroPressure) {
                prepare_pres();
                task.m_current_readout = SensorRead::BaroReadoutType::kReadBaroTemperature;
            } else {
                prepare_temp();
                task.m_current_readout = SensorRead::BaroReadoutType::kReadBaroPressure;
                /* For Simulator */
                if (simulation_started) {
                    for (int i = 0; i < NUM_BARO; i++) {
                        task.m_baro_data[i].pressure = global_baro_sim[i].pressure;
                    }
                } else {
                    get_temp_pres(&(task.m_baro_data[0].temperature), &(task.m_baro_data[0].pressure));
                }

                /* Read and Save Barometric Data */
                for (int i = 0; i < NUM_BARO; i++) {
                    record(tick_count, add_id_to_record_type(BARO, i), &(task.m_baro_data[0]));
                }

                /* Read and Save Magnetometer Data */
                for (int i = 0; i < NUM_MAGNETO; i++) {
                    mmc5983ma_read_calibrated(&MAG, mag_data);
                    memcpy(&(global_magneto[i].x), &mag_data, 3 * sizeof(float));
                    record(tick_count, add_id_to_record_type(MAGNETO, i), &(global_magneto[i]));
                }

                /* Read and Save High-G ACC Data */
                for (int i = 0; i < NUM_ACCELEROMETER; i++) {
                    int8_t tmp_data[3];
                    h3lis100dl_read_raw(&ACCEL, tmp_data);
                    memcpy(&(global_accel[i].x), &tmp_data, 3 * sizeof(int8_t));
                    record(tick_count, add_id_to_record_type(ACCELEROMETER, i), &(global_accel[i]));
                }

                /* Read and Save IMU Data */
                for (int i = 0; i < NUM_IMU; i++) {
                    if (simulation_started) {
                        task.m_imu_data[i].acc = global_imu_sim[i].acc;
                    } else {
                        read_imu(task.m_imu_data[i].gyro, task.m_imu_data[i].acc, i);
                    }
                    record(tick_count, add_id_to_record_type(IMU, i), &(global_imu[i]));
                    log_debug("IMU_Ax %hd, IMU_Gx %hd, Baro %lu", task.m_imu_data[i].acc.x, task.m_imu_data[i].gyro.x,
                              task.m_baro_data[i].pressure);
                }
            }

            tick_count += tick_update;
            osDelayUntil(tick_count);
        }
    }
}

/** Private Function Definitions **/

static void read_imu(vi16_t &gyroscope, vi16_t &acceleration, int32_t id) {
    int16_t acc[3] = {};
    int16_t gyro[3] = {};
    if (id >= NUM_IMU) return;
#if IMU_TYPE == ICM20601_TYPE
    icm20601_read_accel_raw(&IMU_DEV[id], acceleration);
    icm20601_read_gyro_raw(&IMU_DEV[id], gyroscope);
#elif IMU_TYPE == LSM6DSR_TYPE
    //lsm6dsr_read_accel_raw(&IMU_DEV[id], acc);
    //lsm6dsr_read_gyro_raw(&IMU_DEV[id], gyro);
#endif
    memcpy(&acceleration, acc, 3 * sizeof(int16_t));
    memcpy(&gyroscope, gyro, 3 * sizeof(int16_t));
}

static void prepare_temp() {
    for (int32_t i = 0; i < NUM_BARO; ++i) {
        ms5607_prepare_temp(&BARO_DEV[i]);
    }
}

//
static void prepare_pres() {
    for (int32_t i = 0; i < NUM_BARO; ++i) {
        ms5607_prepare_pres(&BARO_DEV[i]);
    }
}

static void read_baro() {
    for (int32_t i = 0; i < NUM_BARO; ++i) {
        ms5607_read_raw(&BARO_DEV[i]);
    }
}

static void get_temp_pres(int32_t *temperature, int32_t *pressure) {
    for (int32_t i = 0; i < NUM_BARO; ++i) {
        ms5607_get_temp_pres(&BARO_DEV[i], &temperature[i], &pressure[i]);
    }
}

