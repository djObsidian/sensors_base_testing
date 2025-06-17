/*
 * drv_scd41.h
 *
 *  Created on: Jun 5, 2025
 *      Author: admin
 */

#ifndef SENSORS_INC__DRV_SCD41_H_
#define SENSORS_INC__DRV_SCD41_H_

#include "sensor2_data_types.h"


void DRV_scd41_init(Sensor_device_t* device, I2C_HandleTypeDef* interface, uint32_t upt[]);
void DRV_scd41_run(Sensor_device_t* device, drv_Drv_status* status, uint32_t timer, uint32_t upt[]);

#endif /* SENSORS_INC__DRV_SCD41_H_ */
