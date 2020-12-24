//
// Created by stoja on 20.12.20.
//

#include "cmsis_os.h"
#include "drivers/w25qxx.h"
#include "config/cats_config.h"
#include "util.h"
#include "drivers/buzzer.h"

BUZ BUZZER = BUZZER_INIT();

void vTaskInit(void* argument) {
  osDelay(3000);
#if (configUSE_TRACE_FACILITY == 1)
  vTraceEnable(TRC_START_AWAIT_HOST);
  HAL_GPIO_TogglePin(GPIOC, LED_STATUS_Pin);
#endif
  // osDelay(5000);
  //  W25qxx_Init();
  uint32_t i = 0;
  buzzer_set_freq(&BUZZER, 1000);
  buzzer_set_volume(&BUZZER, 1);
  /* Infinite loop */
  for (;;) {
    //    /* fill the config with some values */
    //    cc_init(i / 2.f, i / 3.f, i, "Hello there!");
    //    /* persist it to flash */
    //    cc_save();
    //    /* fill it with some temporary values that are not persisted */
    //    cc_init(-1.f, -2.f, 666, "blah");
    //    /* load the values from the flash back to the config */
    //    cc_load();
    //    /* print out the config */
    //    cc_print();
    //    ++i;

    if (i == 100) {
      buzzer_beep(&BUZZER, 500);
      i = 0;
    } else
      i++;

    buzzer_update(&BUZZER);

    osDelay(10);
  }
  /* USER CODE END 5 */
}
