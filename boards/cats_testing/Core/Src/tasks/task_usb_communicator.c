//
// Created by stoja on 31.01.21.
//

#include "util/log.h"
#include "config/cats_config.h"
#include "config/globals.h"
#include "tasks/task_usb_communicator.h"
#include "util/reader.h"

#include <stdint.h>
#include <stdlib.h>

/**
 *
 * @return True if config update successful
 */

const char usb_config_command_list[USB_COMMAND_NR][15] = {"save",        "exit", "status", "version", "dump",
                                                          "flash_erase", "set",  "get",    "read",    "help"};

const char usb_config_variable_list[USB_VARIABLE_NR][15] = {"lf_acc_th",  "conf_fil", "apo_timer1",
                                                            "stag_2_tim", "stages",   "boot_state"};

cats_usb_commands parse_usb_cmd();
cats_usb_commands parse_usb_var(uint16_t *value);

// static bool update_config() {
//  /* Start from "CFG:" onwards */
//  char *token = strtok((char *)(&usb_receive_buffer[4]), ";");
//  cats_boot_state rcv_boot_state = strtol(token, NULL, 10);
//
//  token = strtok(NULL, ";");
//  bool rcv_clear_flash = strtol(token, NULL, 10);
//
//  //  bool clear_log_info = strtol(token, NULL, 10);
//  //
//  //  if (clear_log_info) {
//  //    log_info("Clear log info received");
//  //    cs_init(CATS_STATUS_SECTOR, 0);
//  //    cs_save();
//  //  }
//
//  if (rcv_boot_state != CATS_INVALID) {
//    cc_init(rcv_boot_state, rcv_clear_flash, UINT32_MAX);
//    cc_save();
//    return true;
//  }
//  return false;
//}

void task_usb_communicator(void *argument) {
  log_raw("USB config started");
  log_raw("C.A.T.S. is now ready to receive commands...");
  while (1) {
    if (usb_msg_received == true) {
      usb_msg_received = false;
      //      if (!strcmp((const char *)usb_receive_buffer, "Wait for
      //      instructions!")) {
      //        log_raw("OK!");
      //      } else if (!strcmp((const char *)usb_receive_buffer, "Hello from
      //      PC!")) {
      //        log_raw("Waiting for config");
      //      } else if (usb_receive_buffer[0] == 'C' && usb_receive_buffer[1]
      //      == 'F' &&
      //                 usb_receive_buffer[2] == 'G') {
      //        /* USB communication is complete if the config is updated */
      //        usb_communication_complete = update_config();
      //      } else {
      //        log_raw("bad message received: \"%s\"", usb_receive_buffer);
      //      }
      uint16_t value = 0;
      cats_usb_commands command = parse_usb_cmd();
      cats_usb_variables variable;

      switch (command) {
        case CATS_USB_CMD_SAVE:
          cc_save();
          log_raw("saving...");
          break;
        case CATS_USB_CMD_EXIT:
          log_raw("disconnected");
          NVIC_SystemReset();
          break;
        case CATS_USB_CMD_STATUS:
          log_raw("Number of recorded flights: %hu", cs_get_num_recorded_flights());
          log_raw("Number of recorded sectors: %hu", cs_get_last_recorded_sector());
          for (int i = 0; i < cs_get_num_recorded_flights(); i++) {
            log_raw("Stats for Flight %d:", i);
            log_raw(
                "Max Altitude: %d m | Max Velocity: %d m/s | Max Acceleration "
                "%d m/s^2",
                (int)cs_get_max_altitude(i), (int)cs_get_max_velocity(i), (int)(cs_get_max_acceleration(i) * 9.81f));
          }
          break;
        case CATS_USB_CMD_VERSION:
          log_raw("still in beta");
          break;
        case CATS_USB_CMD_DUMP:
          log_raw("nothing special");
          break;
        case CATS_USB_CMD_FLASH_ERASE:
          log_raw("erasing all your files in 3..2..1");
          erase_recordings();
          log_raw("done");
          break;
        case CATS_USB_CMD_SET:
          variable = parse_usb_var(&value);
          switch (variable) {
            case CATS_USB_VAR_LIFTOFF_ACC_THRESH:
              if (value >= 1500) {
                cc_set_liftoff_acc_threshold(value);
                log_raw("%s set to %d/1000 G", usb_config_variable_list[variable], (int)value);
              } else {
                log_raw(
                    "Acceleration Threshold needs to be larger than 1500 -> "
                    "1.5 G");
              }
              break;
            case CATS_USB_VAR_FILTER_CONF:
              break;
            case CATS_USB_VAR_APOGEE_TIMER1:
              if (value >= 3) {
                cc_set_apogee_timer(value);
                log_raw("%s set to %d seconds", usb_config_variable_list[variable], (int)value);
              } else {
                log_raw("Apogee cannot be Reached in less than 3 Seconds!");
              }
              break;
            case CATS_USB_VAR_APOGEE_TIMER2:
              if (value >= 5) {
                cc_set_second_stage_timer(value);
                log_raw("%s set to %d seconds", usb_config_variable_list[variable], (int)value);
              } else {
                log_raw("Second Stage Chute cannot be set to less than 5 seconds!");
              }
              break;
            case CATS_USB_VAR_STAGES:
              break;
            case CATS_USB_VAR_BOOT_STATE:
              if (value >= 0 && value < 6) {
                cc_set_boot_state(value);
                log_raw("%s set to %hu", usb_config_variable_list[variable], value);
              } else
                log_raw("boot_state needs to be between 0 and 6");
              break;
            case CATS_USB_VAR_UNKNOWN:
              for (int i = 0; i < USB_VARIABLE_NR; i++) log_raw("%s", usb_config_variable_list[i]);
              break;
            default:
              break;
          }

          break;
        case CATS_USB_CMD_GET:
          variable = parse_usb_var(&value);
          switch (variable) {
            case CATS_USB_VAR_LIFTOFF_ACC_THRESH:
              log_raw("Thrust Threshold is set to %d/1000 G", (int)cc_get_liftoff_acc_threshold());
              break;
            case CATS_USB_VAR_FILTER_CONF:
              break;
            case CATS_USB_VAR_APOGEE_TIMER1:
              log_raw("Apogee Timer is set to %d s", (int)cc_get_apogee_timer());
              break;
            case CATS_USB_VAR_APOGEE_TIMER2:
              log_raw("Second Stage Timer is set to %d s", (int)cc_get_second_stage_timer());
              break;
            case CATS_USB_VAR_STAGES:
              break;
            case CATS_USB_VAR_BOOT_STATE:
              log_raw("boot_state is %hu", cc_get_boot_state());
              break;
            case CATS_USB_VAR_UNKNOWN:
              for (int i = 0; i < USB_VARIABLE_NR; i++) log_raw("%s", usb_config_variable_list[i]);
              break;
            default:
              break;
          }
          break;
        case CATS_USB_CMD_READ:
          osDelay(2000);
          if (value > cs_get_num_recorded_flights() - 1) log_raw("Error: Value to big");
          for (int i = 0; i < cs_get_num_recorded_flights(); i++) {
            print_recording(i);
          }

          break;
        case CATS_USB_CMD_HELP:
          log_raw("Full command list:");
          for (int i = 0; i < USB_COMMAND_NR; i++) log_raw("%s", usb_config_command_list[i]);
          break;
        case CATS_USB_CMD_UNKNOWN:
          log_raw("Unknown command! Send help for the command list");
          break;
        default:
          log_raw("Something went seriously wrong :(");
          break;
      }
    }
    osDelay(100);
  }
}

cats_usb_commands parse_usb_cmd() {
  uint8_t cmd_buffer[20];
  for (int i = 0; i < 20; i++) cmd_buffer[i] = 0;
  int i = 0;
  while (i < 20 && !(usb_receive_buffer[i] == ' ' || usb_receive_buffer[i] == '\r' || usb_receive_buffer[i] == '\n')) {
    cmd_buffer[i] = usb_receive_buffer[i];
    i++;
  }

  for (int j = 0; j < USB_COMMAND_NR; j++) {
    if (!strcmp((const char *)cmd_buffer, usb_config_command_list[j])) {
      return j;
    }
  }
  return CATS_USB_CMD_UNKNOWN;
}

cats_usb_commands parse_usb_var(uint16_t *value) {
  uint8_t var_buffer[20];
  for (int i = 0; i < 20; i++) var_buffer[i] = 0;
  int i = 0;
  int n = 0;
  while (i < 20 && usb_receive_buffer[i] != ' ') i++;
  if (i == 20) {
    *value = 0;
    return CATS_USB_VAR_UNKNOWN;
  }
  i++;
  while (n < 20 && !(usb_receive_buffer[i] == ' ' || usb_receive_buffer[i] == '\r' || usb_receive_buffer[i] == '\n')) {
    var_buffer[n] = usb_receive_buffer[i];
    i++;
    n++;
  }
  if (n == 20) {
    *value = 0;
    return CATS_USB_VAR_UNKNOWN;
  }

  for (int j = 0; j < USB_VARIABLE_NR; j++) {
    if (!strcmp((const char *)var_buffer, usb_config_variable_list[j])) {
      uint8_t num_buffer[10];
      for (int j1 = 0; j1 < 10; j1++) num_buffer[j1] = 0;
      for (int j1 = 0; j1 < 9; j1++) num_buffer[j1] = usb_receive_buffer[i + j1];
      // TODO This breaks sometimes
      sscanf(num_buffer, "%hu", value);
      //*value = 1;
      return j;
    }
  }
  return CATS_USB_VAR_UNKNOWN;
}
