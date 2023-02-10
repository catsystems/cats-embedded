/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#include "tasks/task_simulator.hpp"
#include "cli/cli.hpp"
#include "config/globals.hpp"
#include "target.h"
#include "util/log.h"
#include "util/task_util.hpp"

#include <cstdlib>

/** Private Constants **/

float32_t rand_bounds(float32_t lower_b, float32_t upper_b);

/** Exported Function Definitions **/

namespace task {

void Simulator::SetCoefficients(int32_t sim_decision) {
  /* Toyger */
  if (sim_decision == 0) {
    float64_t pressure_coeff[POLYNOM_SIZE] = {-6.09846193e-17, 2.79599150e-14, -5.67967562e-12, 6.78499400e-10,
                                              -5.32390218e-08, 2.89827980e-06, -1.12597279e-04, 3.15968503e-03,
                                              -6.40383671e-02, 9.26415883e-01, -9.32380945e+00, 6.24363813e+01,
                                              -2.60715710e+02, 6.73219986e+02, -1.60135414e+03, 9.81306274e+04};

    float64_t accel_coeff_thrusting[POLYNOM_SIZE] = {-7.74384180e+06, 6.08296377e+07,  -2.13568117e+08, 4.41789674e+08,
                                                     -5.97189439e+08, 5.52253223e+08,  -3.54499448e+08, 1.55874816e+08,
                                                     -4.42126329e+07, 6.42928691e+06,  3.24708141e+05,  -3.53925326e+05,
                                                     7.33555220e+04,  -7.72424267e+03, 4.30117076e+02,  1.12707667e+00};

    float64_t accel_coeff_coasting[POLYNOM_SIZE] = {-8.45958797e-07, 5.61687651e-05, -1.71045643e-03, 3.16557314e-02,
                                                    -3.97736659e-01, 3.58938089e+00, -2.40047305e+01, 1.20982114e+02,
                                                    -4.62655535e+02, 1.34060682e+03, -2.91533086e+03, 4.66615138e+03,
                                                    -5.31417862e+03, 4.06084709e+03, -1.85964050e+03, 3.83546444e+02};
    memcpy(m_sim_coeff.acceleration_coeff_thrusting, accel_coeff_thrusting,
           sizeof(m_sim_coeff.acceleration_coeff_thrusting));
    memcpy(m_sim_coeff.acceleration_coeff_coasting, accel_coeff_coasting,
           sizeof(m_sim_coeff.acceleration_coeff_coasting));
    memcpy(m_sim_coeff.pressure_coeff, pressure_coeff, sizeof(m_sim_coeff.pressure_coeff));
    m_sim_coeff.switch_time = 1.1F;
    m_sim_coeff.acc_end_time = 6.5F;
    m_sim_coeff.end_time = 45.0F;
  }
  /* Piccard */
  else if (sim_decision == 1) {
    float64_t pressure_coeff[POLYNOM_SIZE] = {-9.89440933e-16, 4.00054035e-13, -7.31010282e-11, 7.97805710e-09,
                                              -5.79007252e-07, 2.94427600e-05, -1.07681202e-03, 2.86244809e-02,
                                              -5.52660057e-01, 7.67223421e+00, -7.51413877e+01, 5.03602583e+02,
                                              -2.17466122e+03, 5.04630426e+03, -4.60095566e+03, 1.00888989e+05};

    float64_t accel_coeff_thrusting[POLYNOM_SIZE] = {3.61459094e-02, -1.13220884e+00, 1.59256346e+01, -1.32825680e+02,
                                                     7.30507566e+02, -2.78686715e+03, 7.55213783e+03, -1.46355473e+04,
                                                     2.01425971e+04, -1.92885323e+04, 1.24052703e+04, -5.10474479e+03,
                                                     1.29652104e+03, -2.09483859e+02, 6.43292235e+00, -1.21991272e+00};

    float64_t accel_coeff_coasting[POLYNOM_SIZE] = {2.26958347e-17, -8.44586803e-15, 1.43443916e-12, -1.47229752e-10,
                                                    1.01924268e-08, -5.02955339e-07, 1.82277767e-05, -4.92550515e-04,
                                                    9.97088339e-03, -1.50607679e-01, 1.67589821e+00, -1.34232022e+01,
                                                    7.45195574e+01, -2.69574673e+02, 5.68434117e+02, -5.33345180e+02};
    memcpy(m_sim_coeff.acceleration_coeff_thrusting, accel_coeff_thrusting,
           sizeof(m_sim_coeff.acceleration_coeff_thrusting));
    memcpy(m_sim_coeff.acceleration_coeff_coasting, accel_coeff_coasting,
           sizeof(m_sim_coeff.acceleration_coeff_coasting));
    memcpy(m_sim_coeff.pressure_coeff, pressure_coeff, sizeof(m_sim_coeff.pressure_coeff));
    m_sim_coeff.switch_time = 4.2F;
    m_sim_coeff.acc_end_time = 30.0F;
    m_sim_coeff.end_time = 44.2F;
  }
}

void Simulator::ComputeSimValues(float32_t time) {
  m_current_press = m_sim_coeff.pressure_coeff[POLYNOM_SIZE - 1];

  if (time > m_sim_coeff.switch_time) {
    m_current_acc = m_sim_coeff.acceleration_coeff_coasting[POLYNOM_SIZE - 1];
  } else {
    m_current_acc = m_sim_coeff.acceleration_coeff_thrusting[POLYNOM_SIZE - 1];
  }

  /* First Check if we are in idle time */
  if (time < 0) {
    return;
  }

  /* Check if we are at the end of the simulation; if so, keep the linear acceleration and pressure the same */
  if (time > m_sim_coeff.end_time) {
    time = m_sim_coeff.end_time;
  }

  float64_t time_pow = 1.0;
  for (int32_t i = POLYNOM_SIZE - 2; i >= 0; i--) {
    time_pow = time_pow * static_cast<float64_t>(time);

    m_current_press += time_pow * m_sim_coeff.pressure_coeff[i];

    if (time > m_sim_coeff.switch_time) {
      m_current_acc += time_pow * m_sim_coeff.acceleration_coeff_coasting[i];
    } else {
      m_current_acc += time_pow * m_sim_coeff.acceleration_coeff_thrusting[i];
    }
  }

  /* Remove acc when we get close to Apogee */
  if (time > m_sim_coeff.acc_end_time) {
    m_current_acc = 0.0;
  }
}

/**
 * @brief Function implementing the task_state_est thread.
 * @param argument: Simulation Choice
 * @retval None
 */
[[noreturn]] void Simulator::Run() noexcept {
  imu_data_t sim_imu_data[NUM_IMU] = {};

  log_mode_e prev_log_mode = log_get_mode();
  log_set_mode(LOG_MODE_SIM);

  /* RNG Init with known seed */
  srand(m_sim_config.noise_seed);

  SetCoefficients(m_sim_config.simulation_option);

  uint32_t tick_count = osKernelGetTickCount();
  constexpr uint32_t tick_update = sysGetTickFreq() / CONTROL_SAMPLING_FREQ;

  /* initialise time */
  timestamp_t sim_start = osKernelGetTickCount();
  float32_t time_to_liftoff = 0.0F;  // This is in seconds

  while (true) {
    auto new_enum = static_cast<flight_fsm_e>(osEventFlagsWait(fsm_flag_id, 0xFF, osFlagsNoClear, 0));
    if (new_enum > TOUCHDOWN || new_enum < MOVING) {
      new_enum = INVALID;
    }

    time_to_liftoff = (static_cast<float32_t>(osKernelGetTickCount() - sim_start)) / 1000.0F - m_idle_time;
    /* Compute new values */
    ComputeSimValues(time_to_liftoff);

    /* Compute wanted acceleration */
    switch (m_sim_config.sim_axis) {
      case 0:
        for (auto &data : sim_imu_data) {
          data.acc.x = static_cast<int16_t>(
              (static_cast<float32_t>(m_current_acc) + rand_bounds(-m_acc_noise, m_acc_noise)) * m_acc_factor);
          data.acc.y = static_cast<int16_t>(rand_bounds(-m_acc_noise, m_acc_noise));
          data.acc.z = static_cast<int16_t>(rand_bounds(-m_acc_noise, m_acc_noise));
        }
        break;
      case 1:
        for (auto &data : sim_imu_data) {
          data.acc.x = static_cast<int16_t>(rand_bounds(-m_acc_noise, m_acc_noise));
          data.acc.y = static_cast<int16_t>(
              (static_cast<float32_t>(m_current_acc) + rand_bounds(-m_acc_noise, m_acc_noise)) * m_acc_factor);
          data.acc.z = static_cast<int16_t>(rand_bounds(-m_acc_noise, m_acc_noise));
        }
        break;
      case 2:
        for (auto &data : sim_imu_data) {
          data.acc.x = static_cast<int16_t>(rand_bounds(-m_acc_noise, m_acc_noise));
          data.acc.y = static_cast<int16_t>(rand_bounds(-m_acc_noise, m_acc_noise));
          data.acc.z = static_cast<int16_t>(
              (static_cast<float32_t>(m_current_acc) + rand_bounds(-m_acc_noise, m_acc_noise)) * m_acc_factor);
        }
        break;
    }

    /* Write into global imu sim variable */
    for (int i = 0; i < NUM_IMU; i++) {
      memcpy(&global_imu_sim[i].acc, &sim_imu_data[i].acc, sizeof(vi16_t));
    }

    /* Write into global pressure sim variable */
    for (int i = 0; i < NUM_BARO; i++) {
      global_baro_sim[i].pressure =
          static_cast<int32_t>(static_cast<float32_t>(m_current_press) + rand_bounds(-m_press_noise, m_press_noise));
    }

    if (new_enum == TOUCHDOWN) {
      log_raw("Simulation Successful.");
      log_set_mode(prev_log_mode);
      cli_enter();
      osThreadExit();
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

}  // namespace task

/** Private Function Definitions **/

float32_t rand_bounds(float32_t lower_b, float32_t upper_b) {
  float32_t var = (static_cast<float32_t>(rand() % 1000)) / 1000;
  return var * (upper_b - lower_b) - lower_b;
}

void start_simulation(char *args) {
  if (simulation_started) {
    log_raw("Simulation already started.");
    return;
  }

  cats_sim_config_t sim_config{.noise_seed = 1, .sim_axis = 0, .simulation_option = 0};

  char *token = strtok(args, " ");

  while (token != nullptr) {
    if (strcmp(token, "--toyger") == 0) {
      sim_config.simulation_option = 0;
    }
    if (strcmp(token, "--piccard") == 0) {
      sim_config.simulation_option = 1;
    }
    if (strcmp(token, "--z") == 0) {
      sim_config.sim_axis = 2;
    }

    if (strcmp(token, "--x") == 0) {
      sim_config.sim_axis = 0;
    }
    if (strcmp(token, "--y") == 0) {
      sim_config.sim_axis = 1;
    }
    if (strcmp(token, "--z") == 0) {
      sim_config.sim_axis = 2;
    }
    if (strcmp(token, "--ns1") == 0) {
      sim_config.noise_seed = 1;
    }
    if (strcmp(token, "--ns2") == 0) {
      sim_config.noise_seed = 2;
    }
    if (strcmp(token, "--ns3") == 0) {
      sim_config.noise_seed = 3;
    }
    token = strtok(nullptr, " ");
  }
  simulation_started = true;
  log_info("Starting Simulation");

  task::Simulator::Start(sim_config);
}
