//
// Created by stoja on 20.12.20.
//

#include "drivers/w25qxx.h"
#include "config/cats_config.h"
#include "config/globals.h"
#include "util/log.h"
#include "util/recorder.h"
#include "drivers/buzzer.h"
#include "drivers/adc.h"
#include "util/battery.h"
#include "util/buzzer_handler.h"
#include "drivers/servo.h"
#include "tasks/task_baro_read.h"
#include "tasks/task_flight_fsm.h"
#include "tasks/task_imu_read.h"
#include "tasks/task_init.h"
#include "tasks/task_recorder.h"
#include "tasks/task_state_est.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdlib.h>

/** Task Definitions **/

/* Definitions for task_baro_read */
uint32_t task_baro_read_buffer[256];
StaticTask_t task_baro_read_control_block;
const osThreadAttr_t task_baro_read_attributes = {
    .name = "task_baro_read",
    .stack_mem = &task_baro_read_buffer[0],
    .stack_size = sizeof(task_baro_read_buffer),
    .cb_mem = &task_baro_read_control_block,
    .cb_size = sizeof(task_baro_read_control_block),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Definitions for task_imu_read */
uint32_t task_imu_read_buffer[256];
StaticTask_t task_imu_read_control_block;
const osThreadAttr_t task_imu_read_attributes = {
    .name = "task_imu_read",
    .stack_mem = &task_imu_read_buffer[0],
    .stack_size = sizeof(task_imu_read_buffer),
    .cb_mem = &task_imu_read_control_block,
    .cb_size = sizeof(task_imu_read_control_block),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Definitions for task_state_est */
uint32_t task_state_est_buffer[1024];
StaticTask_t task_state_est_control_block;
const osThreadAttr_t task_state_est_attributes = {
    .name = "task_state_est",
    .stack_mem = &task_state_est_buffer[0],
    .stack_size = sizeof(task_state_est_buffer),
    .cb_mem = &task_state_est_control_block,
    .cb_size = sizeof(task_state_est_control_block),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Definitions for task_flight_fsm */
uint32_t task_flight_fsm_buffer[256];
StaticTask_t task_flight_fsm_control_block;
const osThreadAttr_t task_flight_fsm_attributes = {
    .name = "task_flight_fsm",
    .stack_mem = &task_flight_fsm_buffer[0],
    .stack_size = sizeof(task_flight_fsm_buffer),
    .cb_mem = &task_flight_fsm_control_block,
    .cb_size = sizeof(task_flight_fsm_control_block),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Definitions for task_recorder */
uint32_t task_recorder_buffer[256];
StaticTask_t task_recorder_control_block;
const osThreadAttr_t task_recorder_attributes = {
    .name = "task_recorder",
    .stack_mem = &task_recorder_buffer[0],
    .stack_size = sizeof(task_recorder_buffer),
    .cb_mem = &task_recorder_control_block,
    .cb_size = sizeof(task_recorder_control_block),
    .priority = (osPriority_t)osPriorityNormal,
};

/** Private Constants **/

/** Private Function Declarations **/

static void init_system();
static void init_devices();
static void init_tasks();
static void init_imu();
static void init_baro();
static void init_buzzer();

/** Exported Function Definitions **/

void task_init(void *argument) {
  osDelay(2000);
  init_system();
  log_info("System initialization complete.");

  osDelay(1000);
  init_devices();
  log_info("Device initialization complete.");

  /* TODO: this should be set from PC and read from config afterwards */
  cc_init(0, 0, CATS_FLIGHT);

  osDelay(1000);
  init_tasks();
  log_info("Task initialization complete.");

  init_end_time = osKernelGetTickCount();

  //  uint8_t *send_buf = calloc(512, sizeof(uint8_t));
  //  uint8_t *rec_buf = calloc(512, sizeof(uint8_t));
  //  for (int j = 0; j < 512; ++j) {
  //    send_buf[j] = 511 - j;
  //  }
  //  w25qxx_init();
  //  /* TODO: We should have a config flag that can be set from PC which says
  //  if we
  //   * should erase the entire flash chip */
  //  for (uint32_t j = 1; j < 127; j++) {
  //    w25qxx_erase_sector(j);
  //    log_debug("Erasing sector %lu", j);
  //  }
  servo_set_position(&SERVO1, 90);
  servo_set_position(&SERVO2, 180);
  servo_start(&SERVO1);
  servo_start(&SERVO2);

  // adc test
  adc_init();
  osDelay(100);
  battery_monitor_init();
  /* Infinite loop */
  for (;;) {
    //    w25qxx_write_sector(send_buf, i, 0, 512);
    //
    //    w25qxx_read_sector(rec_buf, i, 0, 512);
    //
    //    log_raw("sector: %lu", i);
    //    for (int j = 0; j < 512; ++j) {
    //      log_rawr("%hu ", rec_buf[j]);
    //    }
    //
    //    log_raw("\n\n");

    battery_level_e level = battery_level();
    if (level == BATTERY_CRIT)
      error_buzzer(CATS_ERROR_BAT_CRIT);
    else if (level == BATTERY_LOW)
      error_buzzer(CATS_ERROR_BAT_LOW);
    else
      error_buzzer(CATS_ERROR_OK);

    buzzer_update(&BUZZER);

    osDelay(10);
  }
  /* USER CODE END 5 */
}

/** Private Function Definitions **/

static void init_system() {
#if (configUSE_TRACE_FACILITY == 1)
  vTraceEnable(TRC_START_AWAIT_HOST);
  HAL_GPIO_TogglePin(GPIOC, LED_STATUS_Pin);
#endif
  log_set_level(LOG_TRACE);
  log_enable();
}

static void init_devices() {
  /* IMU */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_SET);
  osDelay(10);
  init_imu();
  osDelay(10);
  /* BARO */
  init_baro();
  osDelay(10);
  /* BUZZER */
  init_buzzer();

  /* FLASH */
#ifdef FLASH_TESTING
  w25qxx_init();
  /* TODO: We should have a config flag that can be set from PC which says if we
   * should erase the entire flash chip */
  for (uint32_t i = 1; i < 11; i++) {
    w25qxx_erase_sector(i);
    log_debug("Erasing sector %lu", i);
  }

  // log_debug("Erasing chip...");
  // w25qxx_erase_chip();
  /* fill the config with some values */
  cc_init(0, 0, CATS_FLIGHT);
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
  switch (cc_get_boot_state()) {
    case CATS_FLIGHT: {
#if (configUSE_TRACE_FACILITY == 1)
      baro_channel = xTraceRegisterString("Baro Channel");
      flash_channel = xTraceRegisterString("Flash Channel");
#endif

      /* creation of task_recorder */
#ifdef FLASH_TESTING
      rec_queue = osMessageQueueNew(REC_QUEUE_SIZE, sizeof(rec_elem_t), NULL);
#if (configUSE_TRACE_FACILITY == 1)
      vTraceSetQueueName(rec_queue, "Recorder Queue");
#endif

      osThreadNew(task_recorder, NULL, &task_recorder_attributes);
#endif

      /* creation of task_baro_read */
      osThreadNew(task_baro_read, NULL, &task_baro_read_attributes);

      /* creation of task_imu_read */
      osThreadNew(task_imu_read, NULL, &task_imu_read_attributes);

      /* creation of task_flight_fsm */
      osThreadNew(task_flight_fsm, NULL, &task_flight_fsm_attributes);

      /* creation of task_state_est */
      osThreadNew(task_state_est, NULL, &task_state_est_attributes);
    } break;
    case CATS_CONFIG:
    case CATS_TIMER:
    case CATS_DROP:
      break;
    default:
      log_fatal("Wrong boot state!");
  }
}

static void init_imu() {
  /* TODO: this delay until 1000 prob. isn't needed anymore */
  osDelayUntil(1000);
  while (!icm20601_init(&ICM1)) {
    osDelay(10);
    log_error("IMU 1 initialization failed");
  }

  while (!icm20601_init(&ICM2)) {
    osDelay(10);
    log_error("IMU 2 initialization failed");
  }

  while (!icm20601_init(&ICM3)) {
    osDelay(10);
    log_error("IMU 3 initialization failed");
  }
#define CALIBRATE_ACCEL

#ifdef CALIBRATE_ACCEL
  icm20601_accel_calib(&ICM1, 2);  // Axis 0 = x, 1 = y, 2 = z
  icm20601_accel_calib(&ICM2, 2);
  icm20601_accel_calib(&ICM3, 2);
#endif
}

static void init_baro() {
  ms5607_init(&MS1);
  osDelay(10);
  ms5607_init(&MS2);
  osDelay(10);
  ms5607_init(&MS3);
  osDelay(10);
}

static void init_buzzer() {
  buzzer_set_freq(&BUZZER, 4000);
  buzzer_set_volume(&BUZZER, 1);
}
