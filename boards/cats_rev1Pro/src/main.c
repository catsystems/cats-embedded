/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "target.h"
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;

/* Private variables ---------------------------------------------------------*/

/* Definitions for task_init */
uint32_t task_init_buffer[256];
osStaticThreadDef_t task_init_control_block;
const osThreadAttr_t task_init_attributes = {
    .name = "task_init",
    .stack_mem = &task_init_buffer[0],
    .stack_size = sizeof(task_init_buffer),
    .cb_mem = &task_init_control_block,
    .cb_size = sizeof(task_init_control_block),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
void task_init(void *argument);

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

#if (configUSE_TRACE_FACILITY == 1)
  vTraceEnable(TRC_INIT);
#endif

  /* Init scheduler */
  osKernelInitialize();

  /* creation of defaultTask */
  osThreadNew(task_init, NULL, &task_init_attributes);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
