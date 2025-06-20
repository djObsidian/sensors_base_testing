/*
 * _drv_scd41.c
 *
 *  Created on: Jun 5, 2025
 *      Author: admin
 */

#include "_drv_scd41.h"

#include "scd4x_i2c.h"
#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"

#include "smtc_hal_dbg_trace.h"

#define SCD41_POLL_PERIOD 1000*60*15 //15 минут

static int16_t error = NO_ERROR;

void DRV_scd41_init(Sensor_device_t* device, I2C_HandleTypeDef* interface, uint32_t upt[]) {

	SMTC_HAL_TRACE_INFO("Init scd41\n");

	device->deviceContext.scd41.lastPoll = SCD41_POLL_PERIOD;
	device->deviceContext.scd41.i2cHandle = interface;
	device->deviceContext.scd41.driverState = 0;

	scd4x_init(SCD41_I2C_ADDR_62);

	sensirion_i2c_hal_init(interface);


	return;
}

void DRV_scd41_run(Sensor_device_t* device, drv_Drv_status* status, uint32_t timer, uint32_t upt[]) {

	device->deviceContext.scd41.lastPoll += timer;

	uint8_t nextState = device->deviceContext.scd41.driverState;

	bool data_ready = false;

	uint16_t co2;
	int32_t temperature, humidity;

	switch (device->deviceContext.scd41.driverState) {
		case 0: {
			scd4x_init(SCD41_I2C_ADDR_62);
			error = scd4x_wake_up();
			error = scd4x_stop_periodic_measurement();
			error = scd4x_reinit();
			if (error != NO_ERROR) {
				SMTC_HAL_TRACE_INFO("Fuck!\n");
			}
			nextState = 1;
			break;
		}

		case 1: {
			if (device->deviceContext.scd41.lastPoll > 20) {
				device->deviceContext.scd41.lastPoll = 0;

				error = scd4x_measure_single_shot();
				nextState = 2;

				//SMTC_HAL_TRACE_INFO("Sensor awaken\n");
				if (error != NO_ERROR) {
					SMTC_HAL_TRACE_INFO("Fuck!\n");
				}

			}
			break;
		}

		case 2: {
			error = scd4x_get_data_ready_status(&data_ready);
			if (device->deviceContext.scd41.lastPoll > 5000 || data_ready) {
				device->deviceContext.scd41.lastPoll = 0;
				error = scd4x_read_measurement(&co2, &temperature, &humidity);
				error = scd4x_measure_single_shot();
				nextState = 3;
				//SMTC_HAL_TRACE_INFO("CO2: First measurement!\n");
			}

			break;
		}

		case 3: {
			error = scd4x_get_data_ready_status(&data_ready);
			if (device->deviceContext.scd41.lastPoll > 5000 || data_ready) {
				device->deviceContext.scd41.lastPoll = 0;
				error = scd4x_read_measurement(&co2, &temperature, &humidity);
				error = scd4x_measure_single_shot();
				nextState = 4;
				SMTC_HAL_TRACE_INFO("CO2: %d, temp: %f, humidity: %f%%\n", co2, temperature/1000.0f, humidity/1000.0f);

				device->sensorData.dataLen = 5;
				device->sensorData.deviceData[0] = (co2 >> 8) & 0xFF;
				device->sensorData.deviceData[1] = co2 & 0xFF;
				float32_to_fixed_width(temperature, -40.0f, 100.0f, 2, device->sensorData.deviceData+2);
				device->sensorData.deviceData[4] = humidity/1000;

			}

			break;
		}

		case 4: {
			if (device->deviceContext.scd41.lastPoll > SCD41_POLL_PERIOD) {
				device->deviceContext.scd41.lastPoll = 0;
				nextState = 1;
			}

			break;
		}

		default:
			break;
	}

	device->deviceContext.scd41.driverState = nextState;

	return;
}
