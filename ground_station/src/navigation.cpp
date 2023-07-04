#include "navigation.hpp"
#include "console.hpp"
#include "utils.hpp"

#define NAVIGATION_TASK_FREQUENCY 50

bool Navigation::begin() {
  initialized = true;

  compass.init();
  if (imu.begin(Wire, 0x6A) != 1) console.error.println("IMU init failed!");

  filter.begin(NAVIGATION_TASK_FREQUENCY);

  calibration = false;
  compass.setCalibration(-851, 2160, -1223, 1563, -1036, 1832);

  xTaskCreate(navigationTask, "task_recorder", 1024, this, 1, NULL);
  return true;
}

void Navigation::navigationTask(void* pvParameter) {
  Navigation* ref = (Navigation*)pvParameter;

  TickType_t task_last_tick = xTaskGetTickCount();

  uint32_t count = 0;

  // float magBias[3] = {0, 0, 0};
  // float magScale[3] = {0, 0, 0};
  float magMax[3] = {-32768, -32768, -32768};
  float magMin[3] = {32767, 32767, 32767};

  enum {
    POINTS = 72,
  };

  // bool calibrationPoints[POINTS] = {};
  uint32_t axis = 0;
  // int32_t angle;
  uint32_t n = 0;
  // int32_t currentAngle;

  while (ref->initialized) {
    ref->compass.read();

    if (ref->calibration) {
      ref->compass.readRaw(ref->m);
    } else {
      ref->compass.readCalibrated(ref->m);
    }

    ref->imu.readAcceleration(ref->ax, ref->ay, ref->az);
    ref->imu.readGyroscope(ref->gx, ref->gy, ref->gz);

    ref->filter.update(ref->gy, ref->gx, -ref->gz, ref->ay, ref->ax, -ref->az, -ref->m[0], ref->m[1], -ref->m[2]);

    ref->filter.getQuaternion(&ref->q0, &ref->q1, &ref->q2, &ref->q3);

    if (ref->calibration) {
      for (int i = 0; i < 3; i++) {
        if (ref->m[i] > magMax[i]) magMax[i] = ref->m[i];
        if (ref->m[i] < magMin[i]) magMin[i] = ref->m[i];
      }

      if (axis == 0) {
        // angle = ref->filter.getYaw();
        // currentAngle = 0;
      } else if (axis == 1) {
        // angle = ref->filter.getRoll();
        // currentAngle = -180;
      } else {
        // angle = ref->filter.getPitch();
        // currentAngle = -180;
      }

      n++;
      /*
      for(int i = 0; i < POINTS; i++){
          if(angle > currentAngle && angle < (currentAngle + (360/POINTS)) && !calibrationPoints[i]){
              calibrationPoints[i] = true;
              n++;
              //console.log.print("Added Point: "); console.log.println(i);
              break;
          }
          currentAngle += (360/POINTS);
      }*/

      if (n == 500) {  // POINTS){
        // TODO also calibrate pitch
        /*if(axis < 0){
            memset(calibrationPoints, false, POINTS);
            n = 0;
            axis++;
            console.log.println("Axis Calibration Done");
        } else {*/
        ref->calibration = false;
        ref->compass.setCalibration(magMin[0], magMax[0], magMin[1], magMax[1], magMin[2], magMax[2]);

        //}
      }
    }
    /*
    console.log.print("q0 "); console.log.print(ref->q0);
    console.log.print(" q1 "); console.log.print(ref->q1);
    console.log.print(" q2 "); console.log.print(ref->q2);
    console.log.print(" q3 "); console.log.println(ref->q3);
    */
    if (count >= 10) {
      if (ref->pointA.lat != 0 && ref->pointA.lon != 0 && ref->pointB.lat != 0 && ref->pointB.lon != 0) {
        ref->calculateDistanceDirection();
        ref->updated = true;
      }
      count = 0;
    }
    count++;

    vTaskDelayUntil(&task_last_tick, (const TickType_t)1000 / NAVIGATION_TASK_FREQUENCY);
  }
  vTaskDelete(NULL);
}

void Navigation::calculateDistanceDirection() {
  float dy_dphi = (2 * R * PI_F) / (2 * PI_F);
  float dx_dtheta = cos(pointA.lat * (PI_F / 180)) * (2 * R * PI_F) / (2 * PI_F);

  float dy = (pointB.lat - pointA.lat) * (PI_F / 180) * dy_dphi;
  float dx = (pointB.lon - pointA.lon) * (PI_F / 180) * dx_dtheta;
  float dz = pointB.alt - pointA.alt;

  dist = sqrt(dx * dx + dy * dy + dz * dz);
  azimuth = atan2(dx, dy);
  elevation = atan2(dz, sqrt(dx * dx + dy * dy));
}
