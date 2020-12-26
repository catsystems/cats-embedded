//
// Created by stoja on 20.12.20.
//

#include "cmsis_os.h"
#include "drivers/w25qxx.h"
#include "config/cats_config.h"
#include "util/log.h"
#include "drivers/buzzer.h"
#include <stdlib.h>

BUZ BUZZER = BUZZER_INIT();

static void init_system();

void vTaskInit(void *argument) {
  osDelay(3000);
  init_system();
  uint32_t i = 1;

  uint8_t *send_buf = calloc(512, sizeof(uint8_t));
  uint8_t *rec_buf = calloc(512, sizeof(uint8_t));
  for (int j = 0; j < 512; ++j) {
    send_buf[j] = 511 - j;
  }
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

    w25qxx_write_sector(send_buf, i, 0, 512);

    w25qxx_read_sector(rec_buf, i, 0, 512);

    log_raw("sector: %lu", i);
    for (int j = 0; j < 512; ++j) {
      log_rawr("%hu ", rec_buf[j]);
    }

    log_raw("\n\n");

    ++i;
    //    if (i == 100) {
    //      buzzer_beep(&BUZZER, 500);
    //      i = 0;
    //    } else
    //      i++;

    // buzzer_update(&BUZZER);

    osDelay(10);
  }
  /* USER CODE END 5 */
}

static void init_system() {
#if (configUSE_TRACE_FACILITY == 1)
  vTraceEnable(TRC_START_AWAIT_HOST);
  HAL_GPIO_TogglePin(GPIOC, LED_STATUS_Pin);
#endif
  log_set_level(LOG_DEBUG);
  log_enable();
  w25qxx_init();
  for (uint32_t i = 1; i < 512; i++) {
    w25qxx_erase_sector(i);
    log_debug("Erasing sector %lu", i);
  }
  buzzer_set_freq(&BUZZER, 1000);
  buzzer_set_volume(&BUZZER, 1);
}
