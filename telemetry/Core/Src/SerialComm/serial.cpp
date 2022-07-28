

#include "Parser.h"
#include "common.h"
#include "gps.h"

extern Parser p;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern uint8_t c1;

extern uint32_t lr1;

extern TinyGPSPlus gps;

static uint8_t c2;
extern uint32_t lr2;

void start_serial() {
  lr2 = HAL_GetTick();
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&c2, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&c1, 1);
    lr1 = HAL_GetTick();
    Uart1Buffer.push(c1);
  } else if (huart == &huart2) {
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&c2, 1);
    lr2 = HAL_GetTick();
    Uart2Buffer.push(c2);
  }
}
