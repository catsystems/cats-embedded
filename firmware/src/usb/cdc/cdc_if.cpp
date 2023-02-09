//
// Created by Luca on 20/08/2022.
//

#include "target.h"
#include "tusb.h"

#include "comm/fifo.hpp"
#include "comm/stream_group.hpp"

TIM_HandleTypeDef TimHandle;

void cdc_init() {
  /* Set TIMUsb instance */
  TimHandle.Instance = TIMUsb;

  TimHandle.Init.Period = (CDC_POLLING_INTERVAL * 1000) - 1;
  TimHandle.Init.Prescaler = (SystemCoreClock / 2 / (1000000)) - 1;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

  /* Enable TIM peripherals Clock */
  TIMUsb_CLK_ENABLE();

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the NVIC for TIMx */
  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriority(TIMUsb_IRQn, 6, 0);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIMUsb_IRQn);

  // if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK) {
  /* Starting Error */
  // Error_Handler();
  //}
}

/*
uint8_t txbuf[256];

void TIMUsb_IRQHandler(void) { HAL_TIM_IRQHandler(&TimHandle); }
*/
void cdc_transmit_elapsed() {
  /*
  uint32_t len = stream_length(USB_SG.out);
  if (len > 0 && stream_read(USB_SG.out, txbuf, len)) {
    //tud_cdc_write(txbuf, len);
    //tud_cdc_write_flush();
  }*/
}
