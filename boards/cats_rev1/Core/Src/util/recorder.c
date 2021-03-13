//
// Created by stoja on 26.12.20.
//

#include "util/recorder.h"
#include "util/log.h"
#include "config/globals.h"
#include "cmsis_os.h"

const uint32_t REC_QUEUE_SIZE = 128;

void record(rec_entry_type_e rec_type, const void *rec_value) {
#ifdef FLASH_TESTING
  // TODO: remove this condition
  if (global_flight_state.flight_state >= THRUSTING_1 &&
      global_flight_state.flight_state < TOUCHDOWN) {
    rec_elem_t e = {.rec_type = rec_type};
    switch (rec_type) {
      case IMU0:
      case IMU1:
      case IMU2:
        e.u.imu = *((imu_data_t *)rec_value);
        //        log_info("IMU %u: RAW Gx: %hd, Gy:%hd, Gz:%hd; Ax: %hd,
        //        Ay:%hd, Az:%hd",
        //                 rec_type, e.u.imu.gyro_x, e.u.imu.gyro_y,
        //                 e.u.imu.gyro_z, e.u.imu.acc_x, e.u.imu.acc_y,
        //                 e.u.imu.acc_z);
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
      case COVARIANCE_INFO:
        e.u.covariance_info = *((covariance_info_t *)rec_value);
        break;
      case SENSOR_INFO:
        e.u.sensor_info = *((sensor_info_t *)rec_value);
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
  }
#endif
}
