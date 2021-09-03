//
// Created by stoja on 26.12.20.
//

#include "util/recorder.h"
#include "util/log.h"
#include "config/globals.h"
#include "config/cats_config.h"

/* TODO: set this queue to 256 at least */
const uint32_t REC_QUEUE_SIZE = 256;
const float REC_QUEUE_PRE_THRUSTING_FILL_RATIO = 0.75f;
const uint32_t REC_QUEUE_PRE_THRUSTING_LIMIT = 200;
// TODO Element is not constant error
// REC_QUEUE_PRE_THRUSTING_FILL_RATIO * REC_QUEUE_SIZE;

/**
 * Checks whether the given rec_type should be recorded.
 *
 * @param rec_type - recorder entry type
 * @return true if the given rec_type should be recorded
 */
static inline bool should_record(rec_entry_type_e rec_type) {
  return (global_cats_config.config.recorder_mask & rec_type) > 0;
}

void record(rec_entry_type_e rec_type, const void *rec_value) {
  if (global_recorder_status >= REC_FILL_QUEUE && should_record(rec_type)) {
    rec_elem_t e = {.rec_type = rec_type};
    switch (rec_type) {
      case IMU0:
      case IMU1:
      case IMU2:
        // log_warn("logging IMU %d", rec_type);
        e.u.imu = *((imu_data_t *)rec_value);
        break;
      case BARO0:
      case BARO1:
      case BARO2:
        // log_warn("logging BARO %d", rec_type);
        e.u.baro = *((baro_data_t *)rec_value);
        break;
      case MAGNETO:
        // log_warn("logging MAGNETO_INFO");
        e.u.magneto_info = *((magneto_data_t *)rec_value);
        break;
      case ACCELEROMETER:
        // log_warn("logging ACCELEROMETER_INFO");
        e.u.accel_data = *((accel_data_t *)rec_value);
        break;
      case FLIGHT_INFO:
        // log_warn("logging FLIGHT_INFO");
        e.u.flight_info = *((flight_info_t *)rec_value);
        break;
      case ORIENTATION_INFO:
        // log_warn("logging ORIENTATION INFO");
        e.u.orientation_info = *((orientation_info_t *)rec_value);
        break;
      case FILTERED_DATA_INFO:
        // log_warn("logging FILTERED_DATA_INFO");
        e.u.filtered_data_info = *((filtered_data_info_t *)rec_value);
        break;
      case FLIGHT_STATE:
        // log_warn("logging FLIGHT_STATE");
        e.u.flight_state = *((flight_state_t *)rec_value);
        break;
      case COVARIANCE_INFO:
        // log_warn("logging COVARIANCE_INFO");
        e.u.covariance_info = *((covariance_info_t *)rec_value);
        break;
      case SENSOR_INFO:
        // log_warn("logging SENSOR_INFO");
        e.u.sensor_info = *((sensor_info_t *)rec_value);
        break;
      case EVENT_INFO:
        // log_warn("logging EVENT_INFO");
        e.u.event_info = *((event_info_t *)rec_value);
        break;
      case ERROR_INFO:
        // log_warn("logging EVENT_INFO");
        e.u.error_info = *((error_info_t *)rec_value);
        break;
      default:
        log_fatal("Impossible recorder entry type %d!", rec_type);
        break;
    }

    osStatus_t ret = osMessageQueuePut(rec_queue, &e, 0U, 0U);
    if (ret != osOK) {
      log_error("Inserting an element to the recorder queue failed! Error: %d", ret);
    }
  }
}
