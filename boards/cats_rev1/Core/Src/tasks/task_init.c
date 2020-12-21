//
// Created by stoja on 20.12.20.
//

#include "cmsis_os.h"
#include "drivers/w25qxx.h"
#include "config/cats_config.h"

void vTaskInit(void* argument) {
  osDelay(3000);
#if (configUSE_TRACE_FACILITY == 1)
  vTraceEnable(TRC_START_AWAIT_HOST);
  HAL_GPIO_TogglePin(GPIOC, LED_STATUS_Pin);
#endif
  osDelay(5000);
  //  W25qxx_Init();
  //  uint32_t i = 0;
  /* Infinite loop */
  for (;;) {
    //    /* fill the config with some values */
    //    fill_config(i / 2.f, i / 3.f, i, "Hello there!");
    //    /* persist it to flash */
    //    save_config();
    //    /* fill it with some temporary values that are not persisted */
    //    fill_config(-1.f, -2.f, 666, "blah");
    //    /* load the values from the flash back to the config */
    //    load_config();
    //    /* print out the config */
    //    print_config();
    //    ++i;

    osDelay(100);
  }
  /* USER CODE END 5 */
}
