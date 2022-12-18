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

#pragma once

#include <optional>

#include "util/types.h"
#include "util/log.h"

namespace task {
    [[noreturn]] void task_sensor_read(void *argument);


    class SensorRead {
    public:
        friend void task_sensor_read(void *argument);

        void Run();

        enum class BaroReadoutType {
            kReadBaroTemperature = 1,
            kReadBaroPressure = 2,
        };

        bool PreparePressure();

        static SensorRead &GetInstance() {
            if (!s_instance) {
                s_instance = SensorRead();
            }
            return *s_instance;
        }

    private:
        imu_data_t m_imu_data[NUM_IMU]{};
        baro_data_t m_baro_data[NUM_BARO]{};
        BaroReadoutType m_current_readout{BaroReadoutType::kReadBaroTemperature};

        static std::optional<SensorRead> s_instance;
    };
} // namespace task
