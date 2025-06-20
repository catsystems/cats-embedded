/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "console.hpp"
#include "utils.hpp"

// clang-format off
#include <LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <QMC5883Compass.hpp>
#include <Wire.h>
// clang-format on

#include <cmath>
#include <memory>

constexpr float R = 6378100.0F;   // Earth radius in m (zero tide radius IAU)
constexpr float C = 40075017.0F;  // Earth circumference in m

class EarthPoint3D {
 public:
  explicit EarthPoint3D(float lat = 0, float lon = 0, float alt = 0) : lat(lat), lon(lon), alt(alt) {}

  EarthPoint3D deg() { return *this; }

  EarthPoint3D rad() const {
    EarthPoint3D res;
    res.lat = lat * (PI_F / 180);
    res.lon = lon * (PI_F / 180);
    return res;
  }

  void print() const {
    console.log.print("Lat:");
    console.log.println(lat);
    console.log.print("Lon:");
    console.log.println(lon);
    console.log.print("Alt:");
    console.log.println(alt);
  }

  float lat;
  float lon;
  float alt;
};

class Navigation {
 public:
  bool begin();

  void setPointA(EarthPoint3D point) { pointA = point; }
  void setPointA(float lat, float lon, float height = 0) { pointA = EarthPoint3D(lat, lon, height); }

  void setPointB(EarthPoint3D point) { pointB = point; }
  void setPointB(float lat, float lon, float height = 0) { pointB = EarthPoint3D(lat, lon, height); }

  inline EarthPoint3D getPointA() const { return pointA; }

  inline EarthPoint3D getPointB() const { return pointB; }

  inline float getNorth() {
    updated = false;
    return filter.getYawRadians();
  }

  inline float getPitch() {
    updated = false;
    return filter.getPitchRadians();
  }

  inline float getRoll() {
    updated = false;
    return filter.getRollRadians();
  }

  inline float getGX() const { return gx; }

  inline float getGY() const { return gy; }

  inline float getGZ() const { return gz; }

  inline float getAX() const { return ax; }

  inline float getAY() const { return ay; }

  inline float getAZ() const { return az; }

  inline float getMX() const { return m[0]; }

  inline float getMY() const { return m[1]; }

  inline float getMZ() const { return m[2]; }

  inline float getCalibrationPercentage() const { return calibration_progress; }

  inline bool isUpdated() const { return updated; }

  inline float getDistance() {
    updated = false;
    return dist;
  }

  inline float getAzimuth() {
    updated = false;
    return azimuth;
  }

  inline float getElevation() {
    updated = false;
    return elevation;
  }

  inline float computeBearing() const { return (azimuth - PI_F / 2) / (2 * PI_F / 360); }

  struct mag_calibration_t {
    float offset[3];
    float scaling[3];
    float max_vals[3];
    float min_vals[3];
    float max_vals_scal[3];
    float min_vals_scal[3];
  };

  enum calibration_state_e { INV_CALIB = 0, CALIB_ONGOING, CALIB_CANCELLED, CALIB_CONCLUDED };

  static void set_saved_calib(mag_calibration_t mag_calib);

  void get_saved_calib();

  void setCalibrationState(calibration_state_e setCalibration) { calibration = setCalibration; }
  calibration_state_e getCalibrationState() { return calibration; }

  void print() {
    console.log.println("Point A:");
    pointA.print();
    console.log.println("Point B:");
    pointB.print();
  }

 private:
  bool initialized = false;
  bool updated = false;
  calibration_state_e calibration = INV_CALIB;

  EarthPoint3D pointA;
  EarthPoint3D pointB;

  std::unique_ptr<QMC5883Compass> compass;
  LSM6DS3Class imu;

  Madgwick filter;

  float gx, gy, gz, ax, ay, az;
  float m[3];
  float raw_m[3];
  static constexpr uint32_t n_points = 64;
  static constexpr float max_sphere_error = 0.1F;
  float sphere_points[n_points][3];
  bool sphere_checked[n_points];
  float calibration_progress = 0;

  float q0, q1, q2, q3;

  float dist = 0;
  float azimuth = 0;
  float elevation = 0;

  mag_calibration_t mag_calib_temp = {.offset = {0, 0, 0},
                                      .scaling = {1.0F, 1.0F, 1.0F},
                                      .max_vals = {-100, -100, -100},
                                      .min_vals = {100, 100, 100},
                                      .max_vals_scal = {-100, -100, -100},
                                      .min_vals_scal = {100, 100, 100}};

  mag_calibration_t mag_calib = {.offset = {0, 0, 0},
                                 .scaling = {1.0F, 1.0F, 1.0F},
                                 .max_vals = {-100, -100, -100},
                                 .min_vals = {100, 100, 100},
                                 .max_vals_scal = {-100, -100, -100},
                                 .min_vals_scal = {100, 100, 100}};

  static void navigationTask(void *pvParameter);
  void calculateDistanceDirection();
  void initFibonacciSphere();
  void calibrate(const float *val);
  void transform(const float *val, float *output);
  void check_rotation();
  void compute_calibration_status();

  void resetCalib();
};
