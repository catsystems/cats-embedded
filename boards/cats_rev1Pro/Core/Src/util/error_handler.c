//
// Created by stoja on 13.05.21.
//

#include "util/error_handler.h"
#include "util/recorder.h"
#include "util/log.h"

#include <stdint.h>

void error_handler(cats_error_e err) {
  /* TODO: see if it makes sense to log from which task the error came from */
  // osThreadId_t task_id = osThreadGetId();
  if (err != CATS_ERR_OK) {
    log_error("Encountered error 0x%x", err);
    // uint32_t ts = osKernelGetTickCount();
    //    error_info_t error_info = {.ts = ts, .error = err};
    //    record(ERROR_INFO, &error_info);
  }
}