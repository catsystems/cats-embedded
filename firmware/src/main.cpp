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

#include "main.h"
#include "cmsis_os.h"
#include "target.h"

#include "drivers/adc.hpp"

#include "config/globals.hpp"
#include "util/battery.hpp"
#include "util/log.h"
#include "util/task_util.hpp"

#include "init/config.hpp"
#include "init/system.hpp"

#include "drivers/gpio.hpp"
#include "drivers/pwm.hpp"
#include "sensors/lsm6dso32.hpp"
#include "sensors/ms5607.hpp"

#include "tasks/task_buzzer.hpp"
#include "tasks/task_flight_fsm.hpp"
#include "tasks/task_health_monitor.hpp"
#include "tasks/task_peripherals.hpp"
#include "tasks/task_preprocessing.hpp"
#include "tasks/task_recorder.hpp"
#include "tasks/task_sensor_read.hpp"
#include "tasks/task_state_est.hpp"
#include "tasks/task_telemetry.hpp"

extern driver::Servo* global_servo1;
extern driver::Servo* global_servo2;

#include "tusb.h"
#include "usb/cdc/cdc_if.hpp"

static void init_logging() {
  log_set_level(LOG_TRACE);
  log_enable();
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* MCU Configuration--------------------------------------------------------*/
  target_pre_init();

  /* Initialize RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(&RTC_HANDLE);
  HAL_PWR_EnableBkUpAccess();

  /* Jump to bootloader if RTC register matches pattern */
  if (HAL_RTCEx_BKUPRead(&RTC_HANDLE, RTC_BKP_DR0) == BOOTLOADER_MAGIC_PATTERN) {
    HAL_RTCEx_BKUPWrite(&RTC_HANDLE, RTC_BKP_DR0, 0);  // Reset register
    BootLoaderJump();                                  // Does not return!
  }

  usb_device_initialized = target_init();

  // Build digital io
  static driver::OutputPin imu_cs(GPIOB, 0U);
  static driver::OutputPin barometer_cs(GPIOB, 1U);
  static driver::OutputPin status_led(GPIOC, 14U);

  // Build the SPI driver
  static driver::Spi spi1(&hspi1);

  // Build the PWM channels
  static driver::Pwm pwm_buzzer(BUZZER_TIMER_HANDLE, BUZZER_TIMER_CHANNEL);

  static driver::Pwm pwm_servo1(SERVO_TIMER_HANDLE, SERVO_TIMER_CHANNEL_1);
  static driver::Pwm pwm_servo2(SERVO_TIMER_HANDLE, SERVO_TIMER_CHANNEL_2);

  // Build the servos
  static driver::Servo servo1(pwm_servo1, 50U);
  static driver::Servo servo2(pwm_servo2, 50U);

  // Build the buzzer
  static driver::Buzzer buzzer(pwm_buzzer);

  // Build the sensors
  static sensor::Lsm6dso32 imu(spi1, imu_cs);
  static sensor::Ms5607 barometer(spi1, barometer_cs);

  global_servo1 = &servo1;
  global_servo2 = &servo2;

  init_logging();
  log_info("System initialization complete.");

  HAL_Delay(10);
  init_storage();
  log_info("LFS initialization complete.");

  HAL_Delay(10);
  load_and_set_config();
  log_info("Config load complete.");

  // After loading the config we can set the servos to the initial position
  servo1.SetPosition(global_cats_config.initial_servo_position[0]);
  servo2.SetPosition(global_cats_config.initial_servo_position[1]);

  // Set the buzzer to max volume in testing mode otherwise to user config
  if (global_cats_config.enable_testing_mode) {
    buzzer.SetVolume(100U);
  } else {
    buzzer.SetVolume(static_cast<uint16_t>(global_cats_config.buzzer_volume));
  }

  // Start the pwm channels
  servo1.Start();
  servo2.Start();

  HAL_Delay(100);
  init_devices(imu, barometer);
  log_info("Device initialization complete.");

  HAL_Delay(10);
  adc_init();
  battery_monitor_init(global_cats_config.battery_type);
  log_info("Battery monitor initialization complete.");

  /* Init scheduler */
  osKernelInitialize();

  // TODO: Check rec_queue for validity here
  rec_queue = osMessageQueueNew(REC_QUEUE_SIZE, sizeof(rec_elem_t), nullptr);
  rec_cmd_queue = osMessageQueueNew(REC_CMD_QUEUE_SIZE, sizeof(rec_cmd_type_e), nullptr);
  event_queue = osMessageQueueNew(EVENT_QUEUE_SIZE, sizeof(cats_event_e), nullptr);

  static const task::Buzzer& task_buzzer = task::Buzzer::Start(buzzer);

  task::Peripherals::Start();

  task::HealthMonitor::Start(task_buzzer);

  static const task::StateEstimation* task_state_estimation_ptr = nullptr;

  /* If we are in testing mode, we do not want to start the estimation tasks */
  if (!global_cats_config.enable_testing_mode) {
    task::Recorder::Start();

    static const task::SensorRead& task_sensor_read = task::SensorRead::Start(&imu, &barometer);

    static const task::Preprocessing& task_preprocessing = task::Preprocessing::Start(task_sensor_read);

    static const task::StateEstimation& task_state_estimation = task::StateEstimation::Start(task_preprocessing);

    task::FlightFsm::Start(task_preprocessing, task_state_estimation);

    task_state_estimation_ptr = &task_state_estimation;
  }

  task::Telemetry::Start(task_state_estimation_ptr, task_buzzer);

  log_info("Task initialization complete.");

  log_disable();

  rtos_started = true;

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (true) {
  }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }

  if (htim->Instance == TIMUsb) {
    cdc_transmit_elapsed();
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
