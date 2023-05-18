/*
 Copyright (c) 2014-present PlatformIO <contact@platformio.org>

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
**/
#ifdef STM32

#include "unity_config.h"

#include "stm32g0xx_hal.h"

static UART_HandleTypeDef UartHandle;

void unityOutputStart() {
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();

  __HAL_RCC_USART2_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  UartHandle.Instance = USART2;

  UartHandle.Init.BaudRate = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  UartHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  UartHandle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&UartHandle) != HAL_OK) {
    while (1) {
    }
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&UartHandle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
    while (1) {
    }
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&UartHandle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
    while (1) {
    }
  }
  if (HAL_UARTEx_DisableFifoMode(&UartHandle) != HAL_OK) {
    while (1) {
    }
  }
}

void unityOutputChar(char c) { HAL_UART_Transmit(&UartHandle, (uint8_t*)(&c), 1, 1000); }

void unityOutputFlush() {}

void unityOutputComplete() {
  __HAL_RCC_USART2_CLK_DISABLE();
  __HAL_RCC_GPIOA_CLK_DISABLE();
}
#endif