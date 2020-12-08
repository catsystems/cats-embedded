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
void vReadImu20601(int16_t gyroscope_data[], int16_t acceleration[], int16_t offset[]);

ICM20601 ICM = ICM20601_INIT();

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

	uint16_t imu_chip_selects[3] = {IMU0_SC_Pin, IMU1_SC_Pin, IMU2_SC_Pin};
	uint16_t imu_idx = 0;

	/* initialize queue message */
	//imu_data_t queue_data = { 0 };

	vInitImu20601();

	/* Infinite loop */
	tick_count = osKernelGetTickCount();
	tick_update = osKernelGetTickFreq() / IMU20601_SAMPLING_FREQ;
	for (;;) {
		HAL_GPIO_WritePin(GPIOB, imu_chip_selects[imu_idx], GPIO_PIN_SET);
		tick_count += tick_update;
		vReadImu20601(gyroscope_data, acceleration, &temperature);

		/* Debugging */

		UsbPrint("[DBG] IMU %hu: RAW Gx: %ld, Gy:%ld, Gz:%ld; Ax: %ld, Ay:%ld, Az:%ld, T:%ld; \n",
				imu_idx, gyroscope_data[0], gyroscope_data[1], gyroscope_data[2],
				acceleration[0], acceleration[1], acceleration[2], temperature);

		//TODO HIE AUE STUFF WO MUES GMACHT WERDE MIT DENE DATE

//		queue_data.gyro_x = gyroscope_data[0];
//		queue_data.gyro_y = gyroscope_data[1];
//		queue_data.gyro_z = gyroscope_data[2];
//		queue_data.acc_x = acceleration[0];
//		queue_data.acc_y = acceleration[1];
//		queue_data.acc_z = acceleration[2];
//		queue_data.ts = osKernelGetTickCount();

		/* Send Data to Queue */
		//osMessageQueuePut(preprocess_queue, &queue_data, 0U, 0U);
		HAL_GPIO_WritePin(GPIOB, imu_chip_selects[imu_idx], GPIO_PIN_RESET);
		imu_idx = (imu_idx + 1) % 3;
		osDelayUntil(tick_count);
	}
}

void vInitImu20601() {
	osDelayUntil(1000);
	uint8_t r = 0;
	do {
		r = icm20601_init(&ICM);
		HAL_Delay(10);
	} while(!r);

}

void vReadImu20601(int16_t gyroscope_data[], int16_t acceleration[], int16_t *temperature) {
	icm20601_read_accel_raw(&ICM, acceleration);
	icm20601_read_gyro_raw(&ICM, gyroscope_data);
	icm20601_read_temp_raw(&ICM, temperature);
}

