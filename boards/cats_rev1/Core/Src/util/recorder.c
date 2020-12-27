//
// Created by stoja on 26.12.20.
//

#include "util/recorder.h"
#include "util/log.h"
#include "cmsis_os.h"

void record(rec_entry_type_e rec_type, const void *rec_value) {
#ifdef FLASH_TESTING
  rec_elem_t e = {.rec_type = rec_type};
  switch (rec_type) {
    case IMU0:
    case IMU1:
    case IMU2:
      e.u.imu = *((imu_data_t *)rec_value);
      break;
    case BARO0:
    case BARO1:
    case BARO2:
      e.u.baro = *((baro_data_t *)rec_value);
      break;
    case FLIGHT_INFO:
      e.u.flight_info = *((flight_info_t *)rec_value);
      break;
    case FLIGHT_STATE:
      e.u.flight_state = *((flight_state_t *)rec_value);
      break;
    default:
      log_fatal("Impossible recorder entry type!");
      break;
  }

  osStatus_t ret = osMessageQueuePut(rec_queue, &e, 0U, 0U);
  if (ret != osOK) {
    log_error("Inserting an element to the recorder queue failed! Error: %d",
              ret);
  }
#endif
}