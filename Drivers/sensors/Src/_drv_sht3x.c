/*
 * _drv_sht3x.c
 *
 *  Created on: Jun 9, 2025
 *      Author: admin
 */

#include "_drv_sht3x.h"
#include "smtc_hal_dbg_trace.h"

void DRV_SHT3x_run(uint32_t* lastPoll, sht3x_handle_t *handle, uint32_t timer, uint32_t pollPeriod, drv_SHT3x_results* results, drv_Drv_status* status) {

	if (timer - *lastPoll > pollPeriod) {

		if (!sht3x_init(handle)) {
			SMTC_HAL_TRACE_INFO("SHT3x access failed.\n");

		}

		float temperature, humidity;
		if (!sht3x_read_temperature_and_humidity(handle, &temperature, &humidity)) {
			SMTC_HAL_TRACE_INFO("SHT3x read failed.\n");
			return;
		}

		SMTC_HAL_TRACE_INFO("SHT3x TEMP: %d \n", (int)temperature);
		SMTC_HAL_TRACE_INFO("SHT3x HUMID: %d \n", (int)humidity);

		results->humidity = humidity;
		results->temperature = temperature;

		*lastPoll = timer;
		*status = DRV_DataReady;
		return;

	} else {
		status = DRV_Wait;
		return;
	}

}

