#ifdef STM32
#include <stdbool.h>
#include <unity.h>
#include <main.hpp>

void setUp(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void tearDown(void) {}

void test_led_builtin_pin_number(void) { TEST_ASSERT_EQUAL(GPIO_PIN_5, LED_Pin); }

void test_led_state_high(void) {
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  TEST_ASSERT_EQUAL(GPIO_PIN_SET, HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin));
}

void test_led_state_low(void) {
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  TEST_ASSERT_EQUAL(GPIO_PIN_RESET, HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin));
}

int main() {
  HAL_Init();       // initialize the HAL library
  HAL_Delay(2000);  // service delay

  UNITY_BEGIN();
  RUN_TEST(test_led_builtin_pin_number);

  for (unsigned int i = 0; i < 5; i++) {
    RUN_TEST(test_led_state_high);
    HAL_Delay(500);
    RUN_TEST(test_led_state_low);
    HAL_Delay(500);
  }

  UNITY_END();  // stop unit testing

  while (1) {
  }
}

extern "C" void SysTick_Handler(void) { HAL_IncTick(); }

#endif