

#include "Parser.h"
#include "common.h"
#include "gps.h"

extern Parser p;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern uint8_t c1;
extern uint32_t last_package;

extern TinyGPSPlus gps;

static uint8_t c2;

void start_serial() { HAL_UART_Receive_IT(&huart2, (uint8_t *)&c2, 1); }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    // HAL_UART_Receive_IT(&huart1, (uint8_t*)& c1, 1);
    // last_package = HAL_GetTick();
    // gps.encode(c1);
  } else if (huart == &huart2) {
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&c2, 1);
    p.process(c2);
  }
}
