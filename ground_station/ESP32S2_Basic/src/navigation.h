#pragma once

#include "console.h"
#include <Arduino.h>
#include <LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <QMC5883LCompass.h>
#include <Wire.h>
#include <math.h>

const float R = 6378100.0f;  // Earth radius in m (zero tide radius IAU)
const float C = 40075017.0f; // Earth circumference in m

class EarthPoint3D {
public:
  // Point3D() : x(0), y(0), z(0) { }
  // Point3D(float x, float y) : x(x), y(y), z(0) { }
  EarthPoint3D(float lat = 0, float lon = 0, float alt = 0)
      : lat(lat), lon(lon), alt(alt) {}

  EarthPoint3D operator=(EarthPoint3D const &other) {
    // Guard self assignment
    if (this == &other)
      return *this;
    lat = other.lat;
    lon = other.lon;
    alt = other.alt;
    return *this;
  }

  EarthPoint3D deg() { return *this; }

  EarthPoint3D rad() {
    EarthPoint3D res;
    res.lat = lat * (PI / 180);
    res.lon = lon * (PI / 180);
    return res;
  }

  void print() {
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

private:
};

class Navigation {
public:
  bool begin();

  void setPointA(EarthPoint3D point) { pointA = point; }
  void setPointA(float lat, float lon, float height = 0) {
    pointA = EarthPoint3D(lat, lon, height);
  }

  void setPointB(EarthPoint3D point) { pointB = point; }
  void setPointB(float lat, float lon, float height = 0) {
    pointB = EarthPoint3D(lat, lon, height);
  }

  inline EarthPoint3D getPointA() const { return pointA; }

  inline EarthPoint3D getPointB() const { return pointB; }

  inline float getNorth() {
    updated = false;
    // return filter.getYawRadians();

    float angle = atan(m[1] / m[0]);

    if (m[0] < 0 && m[1] > 0) {
      angle = PI + angle;
    } else if(m[0] < 0 && m[1] < 0){
      angle = PI + angle;
    } else if(m[0] > 0 && m[1] < 0){
      angle = 2 * PI + angle;
    }

    return (angle + PI / 2);
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

  struct mag_calibration_t {
    float offset[3];
    float scaling[3];
    float max_vals[3];
    float min_vals[3];
    float max_vals_scal[3];
    float min_vals_scal[3];
  };

  void set_saved_calib(mag_calibration_t mag_calib);

  void get_saved_calib();

  void setCalibrationBool(bool setCalibration) { calibration = setCalibration; }

  void print() {
    console.log.println("Point A:");
    pointA.print();
    console.log.println("Point B:");
    pointB.print();
  }

private:
  bool initialized = false;
  bool updated = false;
  bool calibration = false;

  EarthPoint3D pointA;
  EarthPoint3D pointB;

  QMC5883LCompass compass;
  LSM6DS3Class imu;

  Madgwick filter;

  float gx, gy, gz, ax, ay, az;
  float m[3];
  float raw_m[3];

  float q0, q1, q2, q3;

  float dist, azimuth, elevation;

  mag_calibration_t mag_calib = {.offset = {0, 0, 0},
                                 .scaling = {1.0f, 1.0f, 1.0f},
                                 .max_vals = {-100, -100, -100},
                                 .min_vals = {100, 100, 100},
                                 .max_vals_scal = {-100, -100, -100},
                                 .min_vals_scal = {100, 100, 100}};

  static void navigationTask(void *pvParameter);
  void calculateDistanceDirection();
  void calibrate(float *val);
  void transform(float *val, float *output);

  void resetCalib();
};