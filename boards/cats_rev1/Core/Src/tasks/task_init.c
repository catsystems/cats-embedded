//
// Created by stoja on 20.12.20.
//

#include "drivers/w25qxx.h"
#include "config/cats_config.h"
#include "config/globals.h"
#include "util/log.h"
#include "drivers/adc.h"
#include "drivers/eeprom_emul.h"
#include "util/battery.h"
#include "util/buzzer_handler.h"
#include "util/actions.h"
#include "tasks/task_baro_read.h"
#include "tasks/task_flight_fsm.h"
#include "tasks/task_drop_test_fsm.h"
#include "tasks/task_imu_read.h"
#include "tasks/task_init.h"
#include "tasks/task_recorder.h"
#include "tasks/task_state_est.h"
#include "tasks/task_peripherals.h"
#include "tasks/task_usb_communicator.h"
#include "tasks/task_receiver.h"
#include "tasks/task_health_monitor.h"
#include "util/fifo.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <stdbool.h>

/** Task Definitions **/

#define SET_TASK_PARAMS(task, stack_sz)           \
  uint32_t task##_buffer[stack_sz];               \
  StaticTask_t task##_control_block;              \
  const osThreadAttr_t task##_attributes = {      \
      .name = #task,                              \
      .stack_mem = &task##_buffer[0],             \
      .stack_size = sizeof(task##_buffer),        \
      .cb_mem = &task##_control_block,            \
      .cb_size = sizeof(task##_control_block),    \
      .priority = (osPriority_t)osPriorityNormal, \
  };

SET_TASK_PARAMS(task_baro_read, 256)
SET_TASK_PARAMS(task_imu_read, 256)
SET_TASK_PARAMS(task_receiver, 256)
SET_TASK_PARAMS(task_health_monitor, 256)
SET_TASK_PARAMS(task_state_est, 2048)
SET_TASK_PARAMS(task_flight_fsm, 256)
SET_TASK_PARAMS(task_drop_test_fsm, 256)
SET_TASK_PARAMS(task_peripherals, 256)
SET_TASK_PARAMS(task_recorder, 256)
SET_TASK_PARAMS(task_usb_communicator, 1024)

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

_Noreturn void task_init(__attribute__((unused)) void *argument) {
  osDelay(200);
  init_system();
  log_info("System initialization complete.");

  osDelay(100);
  init_devices();
  log_info("Device initialization complete.");

  osDelay(100);
  /* In order to change what is logged just remove it from the following OR:
   * In the given example, BARO1 and FLIGHT_STATE ARE MISSING */
  //  uint32_t selected_entry_types = IMU0 | IMU1 | IMU2 | BARO0 | BARO2 |
  //                                  FLIGHT_INFO | COVARIANCE_INFO |
  //                                  SENSOR_INFO;
  //  cc_set_recorder_mask(selected_entry_types);
  cc_set_recorder_mask(UINT32_MAX);
  cc_set_boot_state(CATS_FLIGHT);

  osDelay(10);

  uint16_t num_flights = cs_get_num_recorded_flights();
  log_trace("Number of recorded flights: %hu", num_flights);
  for (uint16_t i = 0; i < num_flights; i++) {
    log_trace("Last sectors of flight %hu: %hu", i, cs_get_last_sector_of_flight(i));
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

  servo_set_position(&SERVO1, 0);
  servo_set_position(&SERVO2, 0);

  servo_start(&SERVO1);
  servo_start(&SERVO2);
  // adc test
  adc_init();
  osDelay(100);
  battery_monitor_init();
  buzzer_queue_status(CATS_BUZZ_BOOTUP);

  // Fifo init
  fifo_init(&usb_input_fifo, usb_fifo_in_buffer, 64);
  log_disable();
  /* Infinite loop */
  HAL_FLASH_Unlock();
//  EE_Status ee_status = EE_Init(EE_FORCED_ERASE);
//  for(int i = 0; i < 3000; i++){
//	  ee_status = EE_WriteVariable32bits(12, 12);
//	  if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {
//		  ee_status|= EE_CleanUp();
//	  }
//  }
//
//  uint32_t data[4];
//  EE_ReadVariable32bits(5,&data[0]);
//  EE_ReadVariable32bits(6,&data[1]);
//  EE_ReadVariable32bits(7,&data[2]);
//  EE_ReadVariable32bits(4,&data[3]);

  while (1) {
    if (global_usb_detection == true && usb_communication_complete == false) {
      init_communication();
    }



    osDelay(100);
  }
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
  osDelay(10);
  cs_load();

  /* TODO: throw a warning instead of setting to 0 */
  if (cs_get_num_recorded_flights() > 32) {
    cs_init(CATS_STATUS_SECTOR, 0);
    cs_save();
  }

  /* set the first writable sector as the last recorded sector + 1 */
  uint16_t first_writable_sector = cs_get_last_recorded_sector() + 1;
  /* increment the first writable sector as long as the current sector is not
   * empty */
  while (first_writable_sector < w25qxx.sector_count &&
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
    log_info("Updating last recorded sector to %hu", actual_last_recorded_sector);
    cs_set_last_recorded_sector(actual_last_recorded_sector);
    cs_save();
  }

  if (first_writable_sector >= w25qxx.sector_count) {
    log_error("No empty sectors left!");
  } else if (first_writable_sector >= w25qxx.sector_count - 256) {
    log_warn("Less than 256 sectors left!");
  }
}

static void init_communication() {
  osThreadNew(task_usb_communicator, NULL, &task_usb_communicator_attributes);
  usb_communication_complete = true;
}

static void init_tasks() {
  switch (cc_get_boot_state()) {
    case CATS_FLIGHT: {
#if (configUSE_TRACE_FACILITY == 1)
      baro_channel = xTraceRegisterString("Baro Channel");
      flash_channel = xTraceRegisterString("Flash Channel");
#endif
      /* creation of task_recorder */
      // TODO: Check rec_queue for validity here
      rec_queue = osMessageQueueNew(REC_QUEUE_SIZE, sizeof(rec_elem_t), NULL);
      event_queue = osMessageQueueNew(EVENT_QUEUE_SIZE, sizeof(cats_event_e), NULL);
#if (configUSE_TRACE_FACILITY == 1)
      vTraceSetQueueName(rec_queue, "Recorder Queue");
#endif

      osThreadNew(task_recorder, NULL, &task_recorder_attributes);

      /* creation of task_baro_read */
      osThreadNew(task_baro_read, NULL, &task_baro_read_attributes);

      /* creation of receiver */
      // osThreadNew(task_receiver, NULL, &task_receiver_attributes);

      /* creation of task_imu_read */
      osThreadNew(task_imu_read, NULL, &task_imu_read_attributes);

      /* creation of task_flight_fsm */
      osThreadNew(task_flight_fsm, NULL, &task_flight_fsm_attributes);

      /* creation of task_drop_test_fsm */
      // osThreadNew(task_drop_test_fsm, NULL, &task_drop_test_fsm_attributes);
      /* creation of task_peripherals */
      osThreadNew(task_peripherals, NULL, &task_peripherals_attributes);

      /* creation of task_state_est */
      osThreadNew(task_state_est, NULL, &task_state_est_attributes);

      /* creation of task_health_monitor */
      osThreadNew(task_health_monitor, NULL, &task_health_monitor_attributes);
    } break;
    case CATS_CONFIG:
      break;
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
  event_action_map = calloc(9, sizeof(event_action_map_elem_t));

  event_action_map[EV_IDLE].num_actions = 1;
  event_action_map[EV_IDLE].action_list = calloc(1, sizeof(peripheral_act_t));
  event_action_map[EV_IDLE].action_list[0].func_ptr = action_table[ACT_SET_RECORDER_STATE];
  event_action_map[EV_IDLE].action_list[0].func_arg = REC_FILL_QUEUE;

  // Liftoff
  event_action_map[EV_LIFTOFF].num_actions = 1;
  event_action_map[EV_LIFTOFF].action_list = calloc(1, sizeof(peripheral_act_t));
  event_action_map[EV_LIFTOFF].action_list[0].func_ptr = action_table[ACT_SET_RECORDER_STATE];
  event_action_map[EV_LIFTOFF].action_list[0].func_arg = REC_WRITE_TO_FLASH;

  // Apogee / Drogue
  event_action_map[EV_APOGEE].num_actions = 1;
  event_action_map[EV_APOGEE].action_list = calloc(1, sizeof(peripheral_act_t));
  event_action_map[EV_APOGEE].action_list[0].func_ptr = action_table[ACT_HIGH_CURRENT_ONE];
  event_action_map[EV_APOGEE].action_list[0].func_arg = 1;

  // Low Altitude / Main
  event_action_map[EV_POST_APOGEE].num_actions = 1;
  event_action_map[EV_POST_APOGEE].action_list = calloc(1, sizeof(peripheral_act_t));
  event_action_map[EV_POST_APOGEE].action_list[0].func_ptr = action_table[ACT_HIGH_CURRENT_TWO];
  event_action_map[EV_POST_APOGEE].action_list[0].func_arg = 1;

  // Timer 1 / Drogue
  event_action_map[EV_TIMER_1].num_actions = 1;
  event_action_map[EV_TIMER_1].action_list = calloc(1, sizeof(peripheral_act_t));
  event_action_map[EV_TIMER_1].action_list[0].func_ptr = action_table[ACT_HIGH_CURRENT_ONE];
  event_action_map[EV_TIMER_1].action_list[0].func_arg = 1;

  // Timer 2 / Main
  event_action_map[EV_TIMER_2].num_actions = 1;
  event_action_map[EV_TIMER_2].action_list = calloc(1, sizeof(peripheral_act_t));
  event_action_map[EV_TIMER_2].action_list[0].func_ptr = action_table[ACT_HIGH_CURRENT_TWO];
  event_action_map[EV_TIMER_2].action_list[0].func_arg = 1;
  /* ................ */
}

static void init_timers() {
  ev_timers = calloc(num_timers, sizeof(cats_timer_t));
  /* Timer 1 */
  ev_timers[0].timer_init_event = EV_LIFTOFF;
  ev_timers[0].execute_event = EV_TIMER_1;
  if (cc_get_apogee_timer() < 3) {
    ev_timers[0].timer_duration_ticks = 10000;
  } else {
    ev_timers[0].timer_duration_ticks = (uint32_t)(cc_get_apogee_timer() * 1000);
  }

  /* Timer 2 */
  ev_timers[1].timer_init_event = EV_LIFTOFF;
  ev_timers[1].execute_event = EV_TIMER_2;
  if (cc_get_apogee_timer() < 5) {
    ev_timers[1].timer_duration_ticks = 20000;
  } else {
    ev_timers[1].timer_duration_ticks = (uint32_t)(cc_get_second_stage_timer() * 1000);
  }

  /* Create Timers */
  for (uint32_t i = 0; i < num_timers; i++) {
    ev_timers[i].timer_id = osTimerNew((void *)trigger_event, osTimerOnce, (void *)ev_timers[i].execute_event, NULL);
  }
}
