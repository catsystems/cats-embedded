#include "navigation.h"
#include "console.h"

#include "config.h"

#define NAVIGATION_TASK_FREQUENCY 50

bool Navigation::begin() {
  initialized = true;

  compass.init();
  if (imu.begin(Wire, 0x6A) != 1)
    console.error.println("IMU init failed!");

  filter.begin(NAVIGATION_TASK_FREQUENCY);

  calibration = false;

  xTaskCreate(navigationTask, "task_recorder", 1024, this, 1, NULL);
  return true;
}

void Navigation::calibrate(float *val) {

  /* Dumb Calibration Approach */

  /* Hard Iron */
  for (uint32_t i = 0; i < 3; i++) {

    if (mag_calib.max_vals[i] < val[i]) {
      mag_calib.max_vals[i] = val[i];
    }

    if (mag_calib.min_vals[i] > val[i]) {
      mag_calib.min_vals[i] = val[i];
    }

    mag_calib.offset[i] = (mag_calib.max_vals[i] + mag_calib.min_vals[i]) / 2.0F;
  }
  /* Soft Iron */

  float offseted_val[3];
  float avg_scale[3];

  for (uint32_t i = 0; i < 3; i++) {
    /* offset value */
    offseted_val[i] = val[i] - mag_calib.offset[i];

    if (mag_calib.max_vals_scal[i] < offseted_val[i]) {
      mag_calib.max_vals_scal[i] = offseted_val[i];
    }

    if (mag_calib.min_vals_scal[i] > offseted_val[i]) {
      mag_calib.min_vals_scal[i] = offseted_val[i];
    }

    /* average scaling */
    avg_scale[i] =
        (mag_calib.max_vals_scal[i] - mag_calib.min_vals_scal[i]) / 2.0F;
  }

  float avg = (avg_scale[0] + avg_scale[1] + avg_scale[2]) / 3.0F;

  for (uint32_t i = 0; i < 3; i++) {

    mag_calib.scaling[i] = avg / avg_scale[i];

  }
}

void Navigation::check_rotation(){

  int8_t index_ax = static_cast<int8_t>(ax * 10.0F - 10.0F);
  int8_t index_ay = static_cast<int8_t>(ay * 10.0F - 10.0F);
  int8_t index_az = static_cast<int8_t>(az * 10.0F - 10.0F);

  if(index_ax > 19){
    index_ax = 19;
  }
  if(index_ax < 0){
    index_ax = 0;
  }

  mag_calib.ax_rot[index_ax] = true;

  if(index_ay > 19){
    index_ay = 19;
  }
  if(index_ay < 0){
    index_ay = 0;
  }

  mag_calib.ay_rot[index_ay] = true;

  if(index_az > 19){
    index_az = 19;
  }
  if(index_az < 0){
    index_az = 0;
  }

  mag_calib.az_rot[index_az] = true;

} 

void Navigation::compute_calibration_status(){

float ax_percentage = 0;
float ay_percentage = 0;
float az_percentage = 0;

for(uint8_t i = 0; i < 20; i++){
  if(mag_calib.ax_rot[i]){
    ax_percentage += 1.0F / 20.0F;
  }
  if(mag_calib.ay_rot[i]){
    ay_percentage += 1.0F / 20.0F;
  }
  if(mag_calib.az_rot[i]){
    az_percentage += 1.0F / 20.0F;
  }
}
  
console.log.print(ax_percentage);
console.log.print("; ");
console.log.print(ay_percentage);
console.log.print("; ");
console.log.println(az_percentage);


}


void Navigation::set_saved_calib(mag_calibration_t mag_calib){
    systemConfig.config.mag_calib.mag_offset_x = static_cast<int32_t>(mag_calib.offset[0]*1000.0F);
    systemConfig.config.mag_calib.mag_offset_y = static_cast<int32_t>(mag_calib.offset[1]*1000.0F);
    systemConfig.config.mag_calib.mag_offset_z = static_cast<int32_t>(mag_calib.offset[2]*1000.0F);
    
    systemConfig.config.mag_calib.mag_scale_x = static_cast<int32_t>(mag_calib.scaling[0]*1000.0F);
    systemConfig.config.mag_calib.mag_scale_y = static_cast<int32_t>(mag_calib.scaling[1]*1000.0F);
    systemConfig.config.mag_calib.mag_scale_z = static_cast<int32_t>(mag_calib.scaling[2]*1000.0F);
}

void Navigation::get_saved_calib(){
    mag_calib.offset[0] = static_cast<float>(systemConfig.config.mag_calib.mag_offset_x) / 1000.0F;
    mag_calib.offset[1] = static_cast<float>(systemConfig.config.mag_calib.mag_offset_y) / 1000.0F;
    mag_calib.offset[2] = static_cast<float>(systemConfig.config.mag_calib.mag_offset_z) / 1000.0F;
    
    mag_calib.scaling[0] = static_cast<float>(systemConfig.config.mag_calib.mag_scale_x) / 1000.0F;
    mag_calib.scaling[1] = static_cast<float>(systemConfig.config.mag_calib.mag_scale_y) / 1000.0F;
    mag_calib.scaling[2] = static_cast<float>(systemConfig.config.mag_calib.mag_scale_z) / 1000.0F;
}

void Navigation::transform(float *val, float *output) {

  for (uint32_t i = 0; i < 3; i++) {
    output[i] = (val[i] - mag_calib.offset[i]) * mag_calib.scaling[i];
  }
}

void Navigation::resetCalib(){
  for(uint32_t i = 0; i < 3; i++){
    mag_calib.max_vals[i] = -100;
    mag_calib.min_vals[i] = 100;
  }

  for(uint32_t i = 0; i < 20; i++){
    mag_calib.ax_rot[i] = false;
    mag_calib.ay_rot[i] = false;
    mag_calib.az_rot[i] = false;
  }
  
}

void Navigation::navigationTask(void *pvParameter) {
  Navigation *ref = (Navigation *)pvParameter;

  TickType_t task_last_tick = xTaskGetTickCount();

  uint32_t count = 0;

  ref->get_saved_calib();

  int32_t angle;
  uint32_t n = 0;

  while (ref->initialized) {
    ref->compass.read();

    ref->compass.readRaw(ref->raw_m);
    ref->transform(ref->raw_m, ref->m);

    ref->imu.readAcceleration(ref->ax, ref->ay, ref->az);
    ref->imu.readGyroscope(ref->gx, ref->gy, ref->gz);

    ref->filter.update(ref->gy, ref->gx, -ref->gz, ref->ay, ref->ax, -ref->az,
                       -ref->m[0], ref->m[1], -ref->m[2]);

    ref->filter.getQuaternion(&ref->q0, &ref->q1, &ref->q2, &ref->q3);

    //console.log.print(ref->ax);
    //console.log.print("; ");
    //console.log.print(ref->ay);
    //console.log.print("; ");
    //console.log.println(ref->az);

    if (ref->calibration) {

      n++;

      ref->calibrate(ref->raw_m);
      ref->check_rotation();
      ref->compute_calibration_status();

      if (n >= 5000) {
        ref->setCalibrationBool(false);
        ref->set_saved_calib(ref->mag_calib);
        ref->resetCalib();
        n = 0;
      }
    }

    if (count >= 10) {
      if (ref->pointA.lat != 0 && ref->pointA.lon != 0 &&
          ref->pointB.lat != 0 && ref->pointB.lon != 0) {
        ref->calculateDistanceDirection();
        ref->updated = true;
      }
      count = 0;
    }
    count++;

    vTaskDelayUntil(&task_last_tick,
                    (const TickType_t)1000 / NAVIGATION_TASK_FREQUENCY);
  }
  vTaskDelete(NULL);
}

void Navigation::calculateDistanceDirection() {
  float dy_dphi = (2 * R * PI) / (2 * PI);
  float dx_dtheta = cos(pointA.lat * (PI / 180)) * (2 * R * PI) / (2 * PI);

  float dy = (pointB.lat - pointA.lat) * (PI / 180) * dy_dphi;
  float dx = (pointB.lon - pointA.lon) * (PI / 180) * dx_dtheta;
  float dz = pointB.alt - pointA.alt;

  dist = sqrt(dx * dx + dy * dy + dz * dz);
  azimuth = atan2(dx, dy);
  elevation = atan2(dz, sqrt(dx * dx + dy * dy));
}
