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

static int16_t error = NO_ERROR;

void DRV_scd41_init(Sensor_device_t* device, I2C_HandleTypeDef* interface, uint32_t upt[]) {

	SMTC_HAL_TRACE_INFO("Init scd41\n");

	return;
}

void DRV_scd41_run(Sensor_device_t* device, drv_Drv_status* status, uint32_t timer, uint32_t upt[]) {
	return;
}
