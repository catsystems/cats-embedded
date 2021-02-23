//
// Created by stoja on 31.01.21.
//

#include "util/log.h"
#include "config/cats_config.h"
#include "config/globals.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

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
        /* Start from CFG: onwards */
        char *token = strtok((char *)(&usb_receive_buffer[4]), ";");
        // loop through the string to extract all other tokens
        while (token != NULL) {
          log_raw(" %s", token);  // printing each token
          token = strtok(NULL, ";");
        }
        usb_communication_complete = true;
      } else {
        log_raw("bad message received: \"%s\"", usb_receive_buffer);
      }
    }
    { osDelay(100); }
  }
}