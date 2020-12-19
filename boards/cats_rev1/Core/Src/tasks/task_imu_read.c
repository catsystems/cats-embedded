/*
 * task_imu_read.c
 *
 *  Created on: Nov 3, 2019
 *      Author: Jonas
 */

#include "tasks/task_imu_read.h"
#include "drivers/icm20601.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"


void vInitImu20601();
void vReadImu20601(int16_t gyroscope_data[], int16_t acceleration[], int16_t *temperature, int32_t id);

ICM20601 ICM1 = ICM20601_INIT1();
ICM20601 ICM2 = ICM20601_INIT2();
ICM20601 ICM3 = ICM20601_INIT3();

/**
 * @brief Function implementing the task_baro_read thread.
 * @param argument: Not used
 * @retval None
 */
void vTaskImuRead(void *argument) {
	uint32_t tick_count, tick_update;

	/* initialize data variables */
	int16_t gyroscope_data[3] = { 0 }; /* 0 = x, 1 = y, 2 = z */
	int16_t acceleration[3] = { 0 }; /* 0 = x, 1 = y, 2 = z */
	int16_t temperature;

	int32_t imu_idx = 0;

	/* initialize queue message */
	//imu_data_t queue_data = { 0 };
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);
	vInitImu20601();

	/* Infinite loop */
	tick_count = osKernelGetTickCount();
	tick_update = osKernelGetTickFreq() / IMU20601_SAMPLING_FREQ;

	for (;;) {

		tick_count += tick_update;
		vReadImu20601(gyroscope_data, acceleration, &temperature, imu_idx);

		/* Debugging */

//		UsbPrint("IMU %ld: RAW Gx: %ld, Gy:%ld, Gz:%ld; Ax: %ld, Ay:%ld, Az:%ld, T:%ld; \n",
//				imu_idx, gyroscope_data[0], gyroscope_data[1], gyroscope_data[2],
//				acceleration[0], acceleration[1], acceleration[2], temperature);

		global_imu[imu_idx].acc_x = acceleration[0];
		global_imu[imu_idx].acc_y = acceleration[1];
		global_imu[imu_idx].acc_z = acceleration[2];
		global_imu[imu_idx].gyro_x = gyroscope_data[0];
		global_imu[imu_idx].gyro_x = gyroscope_data[1];
		global_imu[imu_idx].gyro_x = gyroscope_data[2];
		global_imu[imu_idx].ts = tick_count;


		imu_idx = (imu_idx + 1) % 3;
		osDelayUntil(tick_count);
	}
}

void vInitImu20601() {
	osDelayUntil(1000);
	uint8_t r = 0;
	do {
		r = icm20601_init(&ICM1);
		HAL_Delay(10);
		//UsbPrint("Init1 failed!");
	} while(!r);
	do {
		r = icm20601_init(&ICM2);
		HAL_Delay(10);
		//UsbPrint("Init2 failed!");
		} while(!r);

	do {
		r = icm20601_init(&ICM3);
		HAL_Delay(10);
		//UsbPrint("Init3 failed!");
		} while(!r);

}

void vReadImu20601(int16_t gyroscope_data[], int16_t acceleration[], int16_t *temperature, int32_t id) {
	switch(id){
		case 0:
			icm20601_read_accel_raw(&ICM1, acceleration);
			icm20601_read_gyro_raw(&ICM1, gyroscope_data);
			icm20601_read_temp_raw(&ICM1, temperature);
		break;
		case 1:
			icm20601_read_accel_raw(&ICM2, acceleration);
			icm20601_read_gyro_raw(&ICM2, gyroscope_data);
			icm20601_read_temp_raw(&ICM2, temperature);
		break;
		case 2:
			icm20601_read_accel_raw(&ICM3, acceleration);
			icm20601_read_gyro_raw(&ICM3, gyroscope_data);
			icm20601_read_temp_raw(&ICM3, temperature);
		break;
		default:
		break;
	}

}

