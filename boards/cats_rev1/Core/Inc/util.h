/*
 * util.h
 *
 *  Created on: Feb 24, 2020
 *      Author: stoja
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "cmsis_os.h"

/* Constants */
#define PREPROCESS_QUEUE_SIZE 32
#define BARO_MUTEX_TIMEOUT 0
#define IMU_MUTEX_TIMEOUT 0

/** BASIC TYPES **/

/* Timestamp */
typedef uint32_t timestamp_t;

/** SENSOR DATA TYPES **/

/* IMU data */
typedef struct {
	timestamp_t ts;
	int16_t gyro_x, gyro_y, gyro_z;
	int16_t acc_x, acc_y, acc_z;
} imu_data_t;

/* Barometer data */
typedef struct {
	timestamp_t ts;
	int32_t pressure;
	int32_t temperature;
} baro_data_t;

typedef struct {
	float pressure[3];
	float temperature[3];
	float acceleration[3];
	float calculated_AGL[3];
	float height;
	float velocity;
	float acc_bias;
	timestamp_t ts;
} state_estimation_data_t;

typedef struct {
	float Ad[3][3];
	float Ad_T[3][3];
	float Gd[3];
	float Bd[3];
	float Q;
	float GdQGd_T[3][3];
	float H_full[3][3];
	float H_full_T[3][3];
	float H_eliminated[2][3];
	float H_eliminated_T[3][2];
	float R_full[3][3];
	float R_eliminated[2][2];
	float K_full[3][3];
	float K_eliminated[2][2];
	float x_bar[3];
	float P_bar[3][3];
	float x_hat[3];
	float P_hat[3][3];
	float pressure_0;
	float t_sampl;
} kalman_filter_t;


static const imu_data_t EMPTY_IMU = { 0 };

/** DEBUGGING **/
/* Read In Fake Sensor Data */
#define USB_DATA_ENABLE 0

/* Debug flag */
#ifdef DEBUG
#undef DEBUG
#endif
/* Comment the next line in order to disable debug mode */
// #define DEBUG

#ifdef DEBUG
osMutexId_t print_mutex;
#define PRINT_BUFFER_LEN 200
char print_buffer[PRINT_BUFFER_LEN];
#endif

uint8_t UsbPrint(const char *format, ...);

#endif /* INC_UTIL_H_ */
