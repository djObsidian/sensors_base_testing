/*
 * _drv_sht3x.h
 *
 *  Created on: Jun 9, 2025
 *      Author: admin
 */

#ifndef SENSORS_INC__DRV_SHT3X_H_
#define SENSORS_INC__DRV_SHT3X_H_

#include "sensor2_data_types.h"

#include "sht3x.h"


void DRV_sht3x_init(Sensor_device_t* device, I2C_HandleTypeDef* interface);
void DRV_sht3x_run(Sensor_device_t* device, drv_Drv_status* status, uint32_t timer);

#endif /* SENSORS_INC__DRV_SHT3X_H_ */
