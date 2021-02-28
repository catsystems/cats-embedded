//
// Created by stoja on 31.01.21.
//

#include "util/log.h"
#include "config/cats_config.h"
#include "config/globals.h"

#include <stdint.h>
#include <stdlib.h>

/**
 *
 * @return True if config update successful
 */
static bool update_config() {
  /* Start from "CFG:" onwards */
  char *token = strtok((char *)(&usb_receive_buffer[4]), ";");
  cats_boot_state rcv_boot_state = strtol(token, NULL, 10);

  token = strtok(NULL, ";");
  bool rcv_clear_flash = strtol(token, NULL, 10);

  //  bool clear_log_info = strtol(token, NULL, 10);
  //
  //  if (clear_log_info) {
  //    log_info("Clear log info received");
  //    cs_init(CATS_STATUS_SECTOR, 0);
  //    cs_save();
  //  }

  if (rcv_boot_state != CATS_INVALID) {
    cc_init(rcv_boot_state, rcv_clear_flash);
    cc_save();
    return true;
  }
  return false;
}

void task_usb_communicator(void *argument) {
  while (1) {
    if (usb_msg_received == true) {
      usb_msg_received = false;
      if (!strcmp((const char *)usb_receive_buffer, "Wait for instructions!")) {
        log_raw("OK!");
      } else if (!strcmp((const char *)usb_receive_buffer, "Hello from PC!")) {
        log_raw("Waiting for config");
      } else if (usb_receive_buffer[0] == 'C' && usb_receive_buffer[1] == 'F' &&
                 usb_receive_buffer[2] == 'G') {
        /* USB communication is complete if the config is updated */
        usb_communication_complete = update_config();
      } else {
        log_raw("bad message received: \"%s\"", usb_receive_buffer);
      }
    }
    osDelay(100);
  }
}