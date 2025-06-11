/*
 * _drv_sht3x.h
 *
 *  Created on: Jun 9, 2025
 *      Author: admin
 */

#ifndef SENSORS_INC__DRV_SHT3X_H_
#define SENSORS_INC__DRV_SHT3X_H_

#include "sht3x.h"

typedef struct {
	float temperature, humidity;
} drv_SHT3x_results;

typedef enum {
	DRV_Wait,
	DRV_DataReady
} drv_SHT3x_Drv_status;

void DRV_SHT3x_run(uint32_t* lastPoll, sht3x_handle_t *handle, uint32_t timer, uint32_t pollPeriod, drv_SHT3x_results* results, drv_SHT3x_Drv_status* status);


#endif /* SENSORS_INC__DRV_SHT3X_H_ */
