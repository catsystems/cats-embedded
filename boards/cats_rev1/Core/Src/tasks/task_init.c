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
#include "util/outputs.h"
#include "drivers/servo.h"
#include "tasks/task_baro_read.h"
#include "tasks/task_flight_fsm.h"
#include "tasks/task_drop_test_fsm.h"
#include "tasks/task_imu_read.h"
#include "tasks/task_init.h"
#include "tasks/task_recorder.h"
#include "tasks/task_state_est.h"
#include "tasks/task_peripherals.h"
#include "tasks/task_flash_reader.h"
#include "tasks/task_usb_communicator.h"
#include "tasks/task_receiver.h"
#include "tasks/task_health_monitor.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <stdbool.h>

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

/* Definitions for task_receiver */
uint32_t task_receiver_buffer[256];
StaticTask_t task_receiver_control_block;
const osThreadAttr_t task_receiver_attributes = {
    .name = "task_receiver",
    .stack_mem = &task_receiver_buffer[0],
    .stack_size = sizeof(task_receiver_buffer),
    .cb_mem = &task_receiver_control_block,
    .cb_size = sizeof(task_receiver_control_block),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Definitions for health_monitor_receiver */
uint32_t task_health_monitor_buffer[256];
StaticTask_t task_health_monitor_control_block;
const osThreadAttr_t task_health_monitor_attributes = {
    .name = "task_health_monitor",
    .stack_mem = &task_health_monitor_buffer[0],
    .stack_size = sizeof(task_health_monitor_buffer),
    .cb_mem = &task_health_monitor_control_block,
    .cb_size = sizeof(task_health_monitor_control_block),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Definitions for task_state_est */
uint32_t task_state_est_buffer[2048];
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

/* Definitions for task_drop_test_fsm */
uint32_t task_drop_test_fsm_buffer[256];
StaticTask_t task_drop_test_fsm_control_block;
const osThreadAttr_t task_drop_test_fsm_attributes = {
    .name = "task_drop_test_fsm",
    .stack_mem = &task_drop_test_fsm_buffer[0],
    .stack_size = sizeof(task_drop_test_fsm_buffer),
    .cb_mem = &task_drop_test_fsm_control_block,
    .cb_size = sizeof(task_drop_test_fsm_control_block),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Definitions for task_peripherals */
uint32_t task_peripherals_buffer[256];
StaticTask_t task_peripherals_control_block;
const osThreadAttr_t task_peripherals_attributes = {
    .name = "task_peripherals",
    .stack_mem = &task_peripherals_buffer[0],
    .stack_size = sizeof(task_peripherals_buffer),
    .cb_mem = &task_peripherals_control_block,
    .cb_size = sizeof(task_peripherals_control_block),
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

/* Definitions for task_flash_reader */
uint32_t task_flash_reader_buffer[256];
StaticTask_t task_flash_reader_control_block;
const osThreadAttr_t task_flash_reader_attributes = {
    .name = "task_flash_reader",
    .stack_mem = &task_flash_reader_buffer[0],
    .stack_size = sizeof(task_flash_reader_buffer),
    .cb_mem = &task_flash_reader_control_block,
    .cb_size = sizeof(task_flash_reader_control_block),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Definitions for task_usb_communicator */
uint32_t task_usb_communicator_buffer[256];
StaticTask_t task_usb_communicator_control_block;
const osThreadAttr_t task_usb_communicator_attributes = {
    .name = "task_usb_communicator",
    .stack_mem = &task_usb_communicator_buffer[0],
    .stack_size = sizeof(task_usb_communicator_buffer),
    .cb_mem = &task_usb_communicator_control_block,
    .cb_size = sizeof(task_usb_communicator_control_block),
    .priority = (osPriority_t)osPriorityNormal,
};

/** Private Constants **/

/** Private Function Declarations **/

static void init_system();
static void init_devices();
static void init_communication();
static void init_tasks();
static void init_imu();
static void init_baro();
static void init_buzzer();
static void init_timers();
static void create_event_map();

/** Exported Function Definitions **/

void task_init(void *argument) {
  osDelay(200);
  init_system();
  log_info("System initialization complete.");

  osDelay(100);
  init_devices();
  log_info("Device initialization complete.");

  osDelay(100);
  init_communication();
  /* In order to change what is logged just remove it from the following OR:
   * In the given example, BARO1 and FLIGHT_STATE ARE MISSING */
  //  uint32_t selected_entry_types = IMU0 | IMU1 | IMU2 | BARO0 | BARO2 |
  //                                  FLIGHT_INFO | COVARIANCE_INFO |
  //                                  SENSOR_INFO;
  //  cc_set_recorder_mask(selected_entry_types);
  cc_set_recorder_mask(UINT32_MAX);
  cc_set_boot_state(CATS_FLIGHT);
  cc_set_clear_flash(false);

  /* After this point the cats config is either updated or the old config is
   * loaded from the flash. */
  cs_load();
  if (cc_get_boot_state() != CATS_CONFIG && cc_get_clear_flash() == true) {
    /* TODO: when we know how many logs we have we don't have to erase the
     * entire chip */
    log_info("Erasing chip...");
    w25qxx_erase_chip();
    cs_init(CATS_STATUS_SECTOR, 0);
    cs_save();
  }
  osDelay(10);

  uint16_t num_flights = cs_get_num_recorded_flights();
  log_trace("Number of recorded flights: %hu", num_flights);
  for (uint16_t i = 0; i < num_flights; i++) {
    log_trace("Last sectors of flight %hu: %hu", i,
              cs_get_last_sector_of_flight(i));
  }

  /* Check if the FSM configurations make sense */
  if (cc_get_apogee_timer() < 3) {
    log_error("Apogee Timer is not configured Properly!");
  }
  if (cc_get_second_stage_timer() < 5) {
    log_error("Second Stage Timer is not configured Properly!");
  }
  if (cc_get_liftoff_acc_threshold() < 1500) {
    log_error("Acceleration Threshold is not configured Properly!");
    cc_set_liftoff_acc_threshold(1499.0f);
  }

  osDelay(100);

  create_event_map();
  init_timers();

  init_tasks();
  log_info("Task initialization complete.");

  init_end_time = osKernelGetTickCount();

  // servo_set_onoff(&SERVO1, 0);
  // servo_set_onoff(&SERVO2, 1);

  // adc test
  adc_init();
  osDelay(100);
  battery_monitor_init();
  buzzer_queue_status(CATS_STATUS_BOOTUP);

  /* Infinite loop */
  for (;;) {
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
  w25qxx_init();
#ifdef FLASH_TESTING
  osDelay(10);
  cs_load();

  if (cs_get_num_recorded_flights() > 32) {
    cs_init(CATS_STATUS_SECTOR, 0);
    cs_save();
  }

  /* set the first writable sector as the last recorded sector + 1 */
  uint16_t first_writable_sector = cs_get_last_recorded_sector() + 1;
  /* increment the first writable sector as long as the current sector is not
   * empty */
  while (
      first_writable_sector < w25qxx.sector_count &&
      !w25qxx_is_empty_sector(first_writable_sector, 0, w25qxx.sector_size)) {
    ++first_writable_sector;
  }

  /* if the first writable sector is not immediately following the last recorded
   * sector, update the config */
  if (first_writable_sector != cs_get_last_recorded_sector() + 1) {
    log_warn(
        "Last recorded sector was: %hu and first writable sector is: "
        "%hu!",
        cs_get_last_recorded_sector(), first_writable_sector);
    uint16_t actual_last_recorded_sector = first_writable_sector - 1;
    log_info("Updating last recorded sector to %hu",
             actual_last_recorded_sector);
    cs_set_last_recorded_sector(actual_last_recorded_sector);
    cs_save();
  }

  if (first_writable_sector >= w25qxx.sector_count) {
    log_error("No empty sectors left!");
  } else if (first_writable_sector >= w25qxx.sector_count - 256) {
    log_warn("Less than 256 sectors left!");
  }
#else
  if (cc_get_boot_state() == CATS_CONFIG) {
    // TODO This makes no sense
    w25qxx_init();
  }
#endif
}

static void init_communication() {
  /**
   * Comm steps:
   *  1) While response_received == true or 30 seconds passed:
   *        Write "hello" to USB every second
   *  2) If response received == true:
   *        parse config_buffer (this should be enough for now...)
   *        update in-memory config
   *        update flash config
   *     Else:
   *        continue by reading the setup from config
   */

  //#define AUTO_USB_CONFIG

#ifdef AUTO_USB_CONFIG
  if (global_usb_detection) {
    usb_communication_complete = true;
    cc_load();
    osThreadNew(task_usb_communicator, NULL, &task_usb_communicator_attributes);
  }
#else
  log_raw("Waiting 10s for usb connection");
  uint32_t comm_start_time = osKernelGetTickCount();
  while (osKernelGetTickCount() - comm_start_time < 10000 &&
         usb_communication_complete != true) {
    if (usb_msg_received) {
      usb_msg_received = false;
      uint8_t buffer[20];
      for (int i = 0; i < 20; i++) buffer[i] = 0;
      int i = 0;
      while (i < 20 &&
             !(usb_receive_buffer[i] == ' ' || usb_receive_buffer[i] == '\r' ||
               usb_receive_buffer[i] == '\n')) {
        buffer[i] = usb_receive_buffer[i];
        i++;
      }

      if (!strcmp((const char *)buffer, "config")) {
        usb_communication_complete = true;
        cc_load();
        osThreadNew(task_usb_communicator, NULL,
                    &task_usb_communicator_attributes);
      }
    }
    osDelay(100);
  }
#endif

  cc_load();
  //  uint32_t comm_start_time = osKernelGetTickCount();
  //  while (usb_communication_complete != true &&
  //         (osKernelGetTickCount() - comm_start_time < 10000)) {
  //    log_raw("What is my purpose?");
  //    osDelay(1000);
  //  }
  //  if (usb_communication_complete == true) {
  //    log_raw("USB communication complete, config updated.");
  //  } else {
  //    log_raw("No USB communication detected, reusing old config");
  //    /* Load old config from the flash. */
  //    cc_load();
  //  }
  //  cc_print();
}

static void init_tasks() {
  if (usb_communication_complete == true) {
    // osThreadNew(task_flash_reader, NULL, &task_flash_reader_attributes);
  } else {
    switch (cc_get_boot_state()) {
      case CATS_FLIGHT: {
#if (configUSE_TRACE_FACILITY == 1)
        baro_channel = xTraceRegisterString("Baro Channel");
        flash_channel = xTraceRegisterString("Flash Channel");
#endif
        /* creation of task_recorder */
#ifdef FLASH_TESTING
        // TODO: Check rec_queue for validity here
        rec_queue = osMessageQueueNew(REC_QUEUE_SIZE, sizeof(rec_elem_t), NULL);
        event_queue =
            osMessageQueueNew(EVENT_QUEUE_SIZE, sizeof(cats_event_e), NULL);
#if (configUSE_TRACE_FACILITY == 1)
        vTraceSetQueueName(rec_queue, "Recorder Queue");
#endif

        osThreadNew(task_recorder, NULL, &task_recorder_attributes);
#endif

        /* creation of task_baro_read */
        osThreadNew(task_baro_read, NULL, &task_baro_read_attributes);

        /* creation of task_baro_read */
        osThreadNew(task_receiver, NULL, &task_receiver_attributes);

        /* creation of task_imu_read */
        osThreadNew(task_imu_read, NULL, &task_imu_read_attributes);

        /* creation of task_flight_fsm */
        osThreadNew(task_flight_fsm, NULL, &task_flight_fsm_attributes);

        /* creation of task_drop_test_fsm */
        //        osThreadNew(task_drop_test_fsm, NULL,
        //        &task_drop_test_fsm_attributes);
        /* creation of task_peripherals */
        osThreadNew(task_peripherals, NULL, &task_peripherals_attributes);

        /* creation of task_state_est */
        osThreadNew(task_state_est, NULL, &task_state_est_attributes);

        /* creation of task_health_monitor */
        osThreadNew(task_health_monitor, NULL, &task_health_monitor_attributes);
      } break;
      case CATS_CONFIG:
        /* creation of task_flash_reader */
        osThreadNew(task_flash_reader, NULL, &task_flash_reader_attributes);
        break;
      case CATS_TIMER:
      case CATS_DROP:
        break;
      default:
        log_fatal("Wrong boot state!");
    }
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
  //#define CALIBRATE_ACCEL

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
  buzzer_set_freq(&BUZZER, 3500);
  buzzer_set_volume(&BUZZER, 60);
}

static void create_event_map() {
  /* number of event types + 0th element */
  /* TODO: move number of events to a constant */
  /* TODO: where to free this? */
  event_output_map = calloc(7, sizeof(event_output_map_elem_t));

  //  event_output_map[EV_IDLE].num_outputs = 1;
  //  event_output_map[EV_IDLE].output_list = calloc(1,
  //  sizeof(peripheral_out_t));
  //  event_output_map[EV_IDLE].output_list[0].func_ptr =
  //      output_table[RECORDER_STATE];
  //  event_output_map[EV_IDLE].output_list[0].func_arg = REC_WRITE_TO_FLASH;
  //  event_output_map[EV_IDLE].output_list[0].delay_ms = 0;
  //
  //  event_output_map[EV_APOGEE].num_outputs = 5;
  //  event_output_map[EV_APOGEE].output_list = calloc(5,
  //  sizeof(peripheral_out_t));
  //  event_output_map[EV_APOGEE].output_list[0].func_ptr =
  //      output_table[OUT_LOW_LEVEL_ONE];                      // IO1
  //  event_output_map[EV_APOGEE].output_list[0].func_arg = 1;  // Set HIGH
  //  event_output_map[EV_APOGEE].output_list[0].delay_ms = 0;
  //  event_output_map[EV_APOGEE].output_list[1].func_ptr =
  //      output_table[OUT_LOW_LEVEL_TWO];                      // IO2
  //  event_output_map[EV_APOGEE].output_list[1].func_arg = 1;  // Set HIGH
  //  event_output_map[EV_APOGEE].output_list[1].delay_ms = 0;
  //  event_output_map[EV_APOGEE].output_list[2].func_ptr =
  //      output_table[OUT_HIGH_CURRENT_TWO];                   // PY2
  //  event_output_map[EV_APOGEE].output_list[2].func_arg = 1;  // Turn ON
  //  event_output_map[EV_APOGEE].output_list[2].delay_ms = 0;
  //  event_output_map[EV_APOGEE].output_list[3].func_ptr =
  //      output_table[OUT_LOW_LEVEL_FOUR];                     // S2
  //  event_output_map[EV_APOGEE].output_list[3].func_arg = 0;  // Set LOW
  //  event_output_map[EV_APOGEE].output_list[3].delay_ms = 0;
  //  event_output_map[EV_APOGEE].output_list[4].func_ptr =
  //      output_table[OUT_HIGH_CURRENT_THREE];                 // PY3
  //  event_output_map[EV_APOGEE].output_list[4].func_arg = 1;  // Turn ON
  //  event_output_map[EV_APOGEE].output_list[4].delay_ms = 0;
  //
  //  event_output_map[EV_POST_APOGEE].num_outputs = 2;
  //  event_output_map[EV_POST_APOGEE].output_list =
  //      calloc(2, sizeof(peripheral_out_t));
  //  event_output_map[EV_POST_APOGEE].output_list[0].func_ptr =
  //      output_table[OUT_LOW_LEVEL_THREE];                         // S1
  //  event_output_map[EV_POST_APOGEE].output_list[0].func_arg = 1;  // Set HIGH
  //  event_output_map[EV_POST_APOGEE].output_list[0].delay_ms = 0;
  //  event_output_map[EV_POST_APOGEE].output_list[1].func_ptr =
  //      output_table[OUT_HIGH_CURRENT_ONE];                        // PY1
  //  event_output_map[EV_POST_APOGEE].output_list[1].func_arg = 1;  // Turn ON
  //  event_output_map[EV_POST_APOGEE].output_list[1].delay_ms = 0;
  //
  //  event_output_map[EV_TOUCHDOWN].num_outputs = 1;
  //  event_output_map[EV_TOUCHDOWN].output_list =
  //      calloc(1, sizeof(peripheral_out_t));
  //  event_output_map[EV_TOUCHDOWN].output_list[0].func_ptr =
  //      output_table[RECORDER_STATE];
  //  event_output_map[EV_TOUCHDOWN].output_list[0].func_arg = REC_OFF;
  //  event_output_map[EV_TOUCHDOWN].output_list[0].delay_ms = 0;
  /* ................ */
}

static void init_timers() {
  ev_timers = calloc(num_timers, sizeof(cats_timer_t));
  /* Timer 1 */
  ev_timers[0].timer_init_event = EV_LIFTOFF;
  ev_timers[0].execute_event = EV_APOGEE;
  if (cc_get_apogee_timer() < 3) {
    ev_timers[0].timer_duration = 10000;
  } else {
    ev_timers[0].timer_duration = (uint32_t)cc_get_apogee_timer() * 1000;
  }

  /* Timer 2 */
  ev_timers[1].timer_init_event = EV_LIFTOFF;
  ev_timers[1].execute_event = EV_POST_APOGEE;
  if (cc_get_apogee_timer() < 5) {
    ev_timers[1].timer_duration = 20000;
  } else {
    ev_timers[1].timer_duration = (uint32_t)cc_get_second_stage_timer() * 1000;
  }
}
