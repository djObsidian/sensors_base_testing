/*
 * drv_scd41.h
 *
 *  Created on: Jun 5, 2025
 *      Author: admin
 */

#ifndef SENSORS_INC__DRV_SCD41_H_
#define SENSORS_INC__DRV_SCD41_H_

#include "main.h"
#include "sensirion_common.h"

typedef enum {
	S_START,
	S_AWAIT_START,
	S_AWAIT_STABILIZE_MEASURE,
	S_AWAIT_DATA,
	S_SLEEP
} SCD41_State_t;

typedef struct {
	I2C_HandleTypeDef *i2c_handle;
	uint32_t soft_timer_start;
	SCD41_State_t driver_state;

} drv_SCD41_Context_t;

typedef struct {
	uint16_t co2_concentration;
	int32_t temperature;
	int32_t relative_humidity;
} drv_SCD41_results;

void DRV_SCD41_run(drv_SCD41_Context_t* context, drv_SCD41_results* results, uint32_t timer, uint8_t* status);


#endif /* SENSORS_INC__DRV_SCD41_H_ */
