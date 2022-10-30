/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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
#include "usbd_cdc_if.h"

#include "drivers/adc.h"

#include "util/battery.h"
#include "util/buzzer_handler.h"
#include "util/log.h"
#include "util/task_util.h"
#include "util/types.h"

#include "init/config.h"
#include "init/system.h"
#include "init/tasks.h"

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

  target_init();

  init_logging();
  HAL_Delay(100);
  log_info("System initialization complete.");

  HAL_Delay(100);
  init_storage();
  log_info("LFS initialization complete.");

  HAL_Delay(100);
  load_and_set_config();
  log_info("Config load complete.");

  HAL_Delay(100);
  init_devices();
  log_info("Device initialization complete.");

  HAL_Delay(100);
  adc_init();
  battery_monitor_init();
  log_info("Battery monitor initialization complete.");

#if (configUSE_TRACE_FACILITY == 1)
  vTraceEnable(TRC_INIT);
#endif

  /* Init scheduler */
  osKernelInitialize();

  init_tasks();
  log_info("Task initialization complete.");

  buzzer_queue_status(CATS_BUZZ_BOOTUP);
  log_disable();

  rtos_started = true;

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1) {
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }

  if (htim->Instance == TIMUsb) {
    CDC_Transmit_Elapsed();
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
