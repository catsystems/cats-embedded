//
// Created by stoja on 20.12.20.
//

#include "cmsis_os.h"
#include "drivers/w25qxx.h"
#include "config/cats_config.h"
#include "util/log.h"
#include "drivers/buzzer.h"
#include "main.h"
#include "util/recorder.h"
#include <stdlib.h>

BUZ BUZZER = BUZZER_INIT();

static void init_system();
static void init_tasks();

void task_init(void *argument) {
  osDelay(2000);
  init_system();
  osDelay(1000);
  init_tasks();

  uint32_t i = 1;
  //  uint8_t *send_buf = calloc(512, sizeof(uint8_t));
  //  uint8_t *rec_buf = calloc(512, sizeof(uint8_t));
  //  for (int j = 0; j < 512; ++j) {
  //    send_buf[j] = 511 - j;
  //  }
  /* Infinite loop */
  for (;;) {
    //    w25qxx_write_sector(send_buf, i, 0, 512);
    //
    //    w25qxx_read_sector(rec_buf, i, 0, 512);

    //    log_raw("sector: %lu", i);
    //    for (int j = 0; j < 512; ++j) {
    //      log_rawr("%hu ", rec_buf[j]);
    //    }
    //
    //    log_raw("\n\n");

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

extern void task_baro_read(void *argument);

extern void task_imu_read(void *argument);

extern void task_state_est(void *argument);

extern void task_flight_fsm(void *argument);

extern void task_recorder(void *argument);

extern const osThreadAttr_t task_baro_read_attributes;
extern const osThreadAttr_t task_imu_read_attributes;
extern const osThreadAttr_t task_state_est_attributes;
extern const osThreadAttr_t task_flight_fsm_attributes;
extern const osThreadAttr_t task_recorder_attributes;

static void init_system() {
#if (configUSE_TRACE_FACILITY == 1)
  vTraceEnable(TRC_START_AWAIT_HOST);
  HAL_GPIO_TogglePin(GPIOC, LED_STATUS_Pin);
#endif
  log_set_level(LOG_TRACE);
  log_enable();

  buzzer_set_freq(&BUZZER, 1000);
  buzzer_set_volume(&BUZZER, 1);

#ifdef FLASH_TESTING
  w25qxx_init();
  /* TODO: We should have a config flag that can be set from PC which says if we
   * should erase the entire flash chip */
  for (uint32_t i = 1; i < 127; i++) {
    w25qxx_erase_sector(i);
    log_debug("Erasing sector %lu", i);
  }

  // log_debug("Erasing chip...");
  // w25qxx_erase_chip();
  /* fill the config with some values */
  cc_init(0);
  /* persist it to flash */
  cc_save();
  /* load the values from the flash back to the config */
  cc_load();
  /* print out the config */
  cc_print();

  /* set the first writable sector as the last recorded sector + 1 */
  uint16_t first_writable_sector = cc_get_last_recorded_sector() + 1;
  /* increment the first writable sector as long as the current sector is not
   * empty */
  while (
      first_writable_sector < w25qxx.sector_count &&
      !w25qxx_is_empty_sector(first_writable_sector, 0, w25qxx.sector_size)) {
    ++first_writable_sector;
  }

  /* if the first writable sector is not immediately following the last recorded
   * sector, update the config */
  if (first_writable_sector != cc_get_last_recorded_sector() + 1) {
    log_warn(
        "Last recorded sector was: %hu and first writable sector is: "
        "%hu!",
        cc_get_last_recorded_sector(), first_writable_sector);
    uint16_t actual_last_recorded_sector = first_writable_sector - 1;
    log_info("Updating last recorded sector to %hu",
             actual_last_recorded_sector);
    cc_set_last_recorded_sector(actual_last_recorded_sector);
    cc_save();
  }

  if (first_writable_sector >= w25qxx.sector_count) {
    log_error("No empty sectors left!");
  } else if (first_writable_sector >= w25qxx.sector_count - 16) {
    log_warn("Less than 16 sectors left!");
  }
#endif
}

static void init_tasks() {
  /* creation of task_baro_read */
  osThreadNew(task_baro_read, NULL, &task_baro_read_attributes);

  /* creation of task_imu_read */
  osThreadNew(task_imu_read, NULL, &task_imu_read_attributes);

  /* creation of task_state_est */
  osThreadNew(task_state_est, NULL, &task_state_est_attributes);

  /* creation of task_state_est */
  osThreadNew(task_flight_fsm, NULL, &task_flight_fsm_attributes);

  /* creation of task_recorder */
#ifdef FLASH_TESTING
  osThreadNew(task_recorder, NULL, &task_recorder_attributes);
#endif
}