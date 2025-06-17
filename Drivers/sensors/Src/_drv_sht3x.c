/*
 * _drv_sht3x.c
 *
 *  Created on: Jun 9, 2025
 *      Author: admin
 */

#include "_drv_sht3x.h"
#include "smtc_hal_dbg_trace.h"

void DRV_sht3x_init(Sensor_device_t* device, I2C_HandleTypeDef* interface, uint32_t upt[]) {
    if (device == NULL || interface == NULL) {
        return; // Можно заменить на HAL_ERROR
    }

    // Прямое присваивание через union
    device->deviceContext.sht3x.i2cHandle = interface;
    device->deviceContext.sht3x.lastPoll = 60*1000;
    device->deviceContext.sht3x.pollPeriod = 60*1000;

    SMTC_HAL_TRACE_INFO("Init sht3x\n");

    return;
}

void DRV_sht3x_run(Sensor_device_t* device, drv_Drv_status* status, uint32_t timer, uint32_t upt[]) {

	device->deviceContext.sht3x.lastPoll += timer;

	if (device->deviceContext.sht3x.lastPoll > device->deviceContext.sht3x.pollPeriod) {
		device->deviceContext.sht3x.lastPoll = 0;
		sht3x_handle_t devHandle;
		devHandle.i2c_handle = device->deviceContext.sht3x.i2cHandle;
		devHandle.device_address = device->address;

		sht3x_init(&devHandle);

		float temperature, humidity;

		if (!sht3x_read_temperature_and_humidity(&devHandle, &temperature, &humidity)) {
			SMTC_HAL_TRACE_INFO("SHT3x read failed.\n");
			return;
		}

		SMTC_HAL_TRACE_INFO("SHT3x TEMP: %d \n", (int)temperature);
		SMTC_HAL_TRACE_INFO("SHT3x HUMID: %d \n", (int)humidity);
	}


	return;
}

