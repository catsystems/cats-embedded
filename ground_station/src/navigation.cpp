/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "navigation.hpp"
#include "config.hpp"
#include "console.hpp"
#include "utils.hpp"

constexpr uint8_t NAVIGATION_TASK_FREQUENCY = 50;

bool Navigation::begin() {
  initialized = true;

  compass = CreateCompass::createSensor();

  compass->init();

  if (imu.begin(Wire, 0x6A) != 1) {
    console.warning.println("IMU init failed!");
  }

  filter.begin(NAVIGATION_TASK_FREQUENCY);

  calibration = CALIB_CONCLUDED;

  xTaskCreate(navigationTask, "task_navigation", 1024, this, 1, nullptr);
  return true;
}

void Navigation::initFibonacciSphere() {
  const float golden_angle = PI_F * (3.0F - sqrtf(5.0F));

  for (uint32_t i = 0; i < n_points; i++) {
    const float theta = static_cast<float>(i) * golden_angle;
    const float z = (1.0F - 1.0F / n_points) * (1.0F - 2.0F / (n_points - 1.0F) * static_cast<float>(i));
    const float radius = sqrt(1.0F - z * z);

    sphere_points[i][0] = radius * cos(theta);
    sphere_points[i][1] = radius * sin(theta);
    sphere_points[i][2] = z;
  }
}

void Navigation::calibrate(const float *val) {
  /* Dumb Calibration Approach */

  /* Hard Iron */
  for (uint32_t i = 0; i < 3; i++) {
    if (mag_calib_temp.max_vals[i] < val[i]) {
      mag_calib_temp.max_vals[i] = val[i];
    }

    if (mag_calib_temp.min_vals[i] > val[i]) {
      mag_calib_temp.min_vals[i] = val[i];
    }

    mag_calib_temp.offset[i] = (mag_calib_temp.max_vals[i] + mag_calib_temp.min_vals[i]) / 2.0F;
  }
  /* Soft Iron */

  float offseted_val[3];
  float avg_scale[3];

  for (uint32_t i = 0; i < 3; i++) {
    /* offset value */
    offseted_val[i] = val[i] - mag_calib_temp.offset[i];

    if (mag_calib_temp.max_vals_scal[i] < offseted_val[i]) {
      mag_calib_temp.max_vals_scal[i] = offseted_val[i];
    }

    if (mag_calib_temp.min_vals_scal[i] > offseted_val[i]) {
      mag_calib_temp.min_vals_scal[i] = offseted_val[i];
    }

    /* average scaling */
    avg_scale[i] = (mag_calib_temp.max_vals_scal[i] - mag_calib_temp.min_vals_scal[i]) / 2.0F;
  }

  const float avg = (avg_scale[0] + avg_scale[1] + avg_scale[2]) / 3.0F;

  for (uint32_t i = 0; i < 3; i++) {
    mag_calib_temp.scaling[i] = avg / avg_scale[i];
  }
}

void Navigation::check_rotation() {
  // Check if we move slowly enough
  if (fabsf(1 - sqrt(ax * ax + ay * ay + az * az)) > 0.05F) {
    return;
  }

  for (uint32_t i = 0; i < n_points; i++) {
    if (fabsf(ax - sphere_points[i][0]) < max_sphere_error) {
      if (fabsf(ay - sphere_points[i][1]) < max_sphere_error) {
        if (fabsf(az - sphere_points[i][2]) < max_sphere_error) {
          sphere_checked[i] = true;
        }
      }
    }
  }
}

void Navigation::compute_calibration_status() {
  float counter = 0;

  // NOLINTNEXTLINE(modernize-loop-convert)
  for (uint32_t i = 0; i < n_points; i++) {
    if (sphere_checked[i]) {
      ++counter;
    }
  }

  calibration_progress = counter / n_points * 130.0F;
}

void Navigation::set_saved_calib(mag_calibration_t mag_calib) {
  systemConfig.config.mag_calib.mag_offset_x = static_cast<int32_t>(mag_calib.offset[0] * 1000.0F);
  systemConfig.config.mag_calib.mag_offset_y = static_cast<int32_t>(mag_calib.offset[1] * 1000.0F);
  systemConfig.config.mag_calib.mag_offset_z = static_cast<int32_t>(mag_calib.offset[2] * 1000.0F);

  systemConfig.config.mag_calib.mag_scale_x = static_cast<int32_t>(mag_calib.scaling[0] * 1000.0F);
  systemConfig.config.mag_calib.mag_scale_y = static_cast<int32_t>(mag_calib.scaling[1] * 1000.0F);
  systemConfig.config.mag_calib.mag_scale_z = static_cast<int32_t>(mag_calib.scaling[2] * 1000.0F);
}

void Navigation::get_saved_calib() {
  mag_calib.offset[0] = static_cast<float>(systemConfig.config.mag_calib.mag_offset_x) / 1000.0F;
  mag_calib.offset[1] = static_cast<float>(systemConfig.config.mag_calib.mag_offset_y) / 1000.0F;
  mag_calib.offset[2] = static_cast<float>(systemConfig.config.mag_calib.mag_offset_z) / 1000.0F;

  mag_calib.scaling[0] = static_cast<float>(systemConfig.config.mag_calib.mag_scale_x) / 1000.0F;
  mag_calib.scaling[1] = static_cast<float>(systemConfig.config.mag_calib.mag_scale_y) / 1000.0F;
  mag_calib.scaling[2] = static_cast<float>(systemConfig.config.mag_calib.mag_scale_z) / 1000.0F;
}

void Navigation::transform(const float *val, float *output) {
  for (uint32_t i = 0; i < 3; i++) {
    output[i] = (val[i] - mag_calib.offset[i]) * mag_calib.scaling[i];
  }
}

void Navigation::resetCalib() {
  for (uint32_t i = 0; i < 3; i++) {
    mag_calib_temp.max_vals[i] = -100;
    mag_calib_temp.min_vals[i] = 100;
  }

  // NOLINTNEXTLINE(modernize-loop-convert)
  for (uint32_t i = 0; i < n_points; i++) {
    sphere_checked[i] = false;
  }
}

void Navigation::navigationTask(void *pvParameter) {
  auto *ref = static_cast<Navigation *>(pvParameter);

  TickType_t task_last_tick = xTaskGetTickCount();

  uint32_t count = 0;

  ref->get_saved_calib();
  ref->initFibonacciSphere();

  while (ref->initialized) {
    ref->compass->read();

    ref->compass->readRaw(ref->raw_m);
    ref->transform(ref->raw_m, ref->m);

    ref->imu.readAcceleration(ref->ax, ref->ay, ref->az);
    ref->imu.readGyroscope(ref->gx, ref->gy, ref->gz);

    ref->filter.update(ref->gy, ref->gx, -ref->gz, ref->ay, ref->ax, -ref->az, -ref->m[0], ref->m[1], -ref->m[2]);

    ref->filter.getQuaternion(&ref->q0, &ref->q1, &ref->q2, &ref->q3);

    if (ref->calibration == CALIB_ONGOING) {
      ref->calibrate(ref->raw_m);
      ref->check_rotation();
      ref->compute_calibration_status();

      if (ref->calibration_progress >= 100.0F) {
        ref->setCalibrationState(CALIB_CONCLUDED);
        ref->mag_calib = ref->mag_calib_temp;
        ref->set_saved_calib(ref->mag_calib);
        ref->resetCalib();
      }

    } else if (ref->calibration == CALIB_CANCELLED) {
      ref->setCalibrationState(CALIB_CONCLUDED);
      ref->resetCalib();
    }

    if (count >= 10) {
      ref->updated = true;
      if (ref->pointA.lat != 0 && ref->pointA.lon != 0 && ref->pointB.lat != 0 && ref->pointB.lon != 0) {
        ref->calculateDistanceDirection();
      }
      count = 0;
    }
    count++;

    vTaskDelayUntil(&task_last_tick, static_cast<TickType_t>(1000) / NAVIGATION_TASK_FREQUENCY);
  }
  vTaskDelete(nullptr);
}

void Navigation::calculateDistanceDirection() {
  const float dy_dphi = (2 * R * PI_F) / (2 * PI_F);
  const float dx_dtheta = cos(pointA.lat * (PI_F / 180)) * (2 * R * PI_F) / (2 * PI_F);

  const float dy = (pointB.lat - pointA.lat) * (PI_F / 180) * dy_dphi;
  const float dx = (pointB.lon - pointA.lon) * (PI_F / 180) * dx_dtheta;
  const float dz = pointB.alt - pointA.alt;

  dist = sqrt(dx * dx + dy * dy + dz * dz);
  azimuth = atan2(dx, dy);
  elevation = atan2(dz, sqrt(dx * dx + dy * dy));
}
