/*
 * _drv_bmp280.c
 *
 *  Created on: Jun 18, 2025
 *      Author: Admin
 */

#include <_drv_bmp388.h>
#include "BMP388.h"

#include "smtc_hal_dbg_trace.h"

#define BMP388_POLL_PERIOD 1000*60*120 //120 минут

void DRV_bmp388_init(Sensor_device_t* device, I2C_HandleTypeDef* interface, uint32_t upt[]) {

	if (device == NULL || interface == NULL) {
		return; // Можно заменить на HAL_ERROR
	}

	// Прямое присваивание через union
	device->deviceContext.bmp388.i2cHandle = interface;
	device->deviceContext.bmp388.lastPoll = BMP388_POLL_PERIOD;
	device->deviceContext.bmp388.driverState = 0;

	SMTC_HAL_TRACE_INFO("Init bmp388\n");

	return;

}

void DRV_bmp388_run(Sensor_device_t* device, drv_Drv_status* status, uint32_t timer, uint32_t upt[]) {

	device->deviceContext.bmp388.lastPoll += timer;

	uint8_t nextState = device->deviceContext.bmp388.driverState;

	BMP388_HandleTypeDef hbmp388;
	hbmp388.hi2c = device->deviceContext.bmp388.i2cHandle;

	switch (device->deviceContext.bmp388.driverState) {
		case 0: { //ожидание периода опроса

			if (device->deviceContext.bmp388.lastPoll > BMP388_POLL_PERIOD) {
				nextState = 1;
			}
			break;
		}

		case 1: { //инициализация bmp280

			HAL_StatusTypeDef bstatus;

			bstatus = BMP388_Init(&hbmp388);


			if (bstatus != HAL_OK) {
				SMTC_HAL_TRACE_ERROR("BMP388init failed!\n");
				nextState = 0;
				device->deviceContext.bmp388.lastPoll = 0;
			} else {
				BMP388_StartSingleMeasurement(&hbmp388);
				nextState = 2;
			}

			break;
		}

		case 2: { //ожидание измерения

			if (BMP388_Drdy(&hbmp388)) {
				uint32_t raw_press = 0;
				uint32_t raw_temp = 0;
				uint32_t sensor_time = 0;
				float press = 0;
				float temp = 0;


				if (BMP388_ReadRawPressTempTime(&hbmp388, &raw_press, &raw_temp, &sensor_time) != HAL_OK) {
					SMTC_HAL_TRACE_ERROR("BMP388 read failed.\n");
					device->deviceContext.bmp388.lastPoll = 0;
					nextState = 0;
					break;
				} else {

					BMP388_Init(&hbmp388);
					BMP388_CompensateRawPressTemp(&hbmp388, raw_press, raw_temp, &press, &temp);

					SMTC_HAL_TRACE_INFO("TEMP: %f \n", temp);
					SMTC_HAL_TRACE_INFO("PRESS: %f \n", press);

					device->sensorData.dataLen = 2;
					float32_to_fixed_width(temp, -20.0f, 60.0f, 1, device->sensorData.deviceData);
					float32_to_fixed_width(press / 100, 300.0f, 1100.0f, 1, device->sensorData.deviceData+1);

					if (BMP388_EnterSleepMode(&hbmp388) != HAL_OK) {
						SMTC_HAL_TRACE_ERROR("BMP388 Sleep mode failed!\n");
					}
				}

				device->deviceContext.bmp388.lastPoll = 0;
				nextState = 0;
			}




			break;
		}

		default:
			break;
	}

	device->deviceContext.bmp388.driverState = nextState;




}

