/*
 * _drv_bmp280.h
 *
 *  Created on: Jun 18, 2025
 *      Author: Admin
 */

#ifndef SENSORS_INC__DRV_BMP388_H_
#define SENSORS_INC__DRV_BMP388_H_

#include "sensor2_data_types.h"

void DRV_bmp388_init(Sensor_device_t* device, I2C_HandleTypeDef* interface, uint32_t upt[]);
void DRV_bmp388_run(Sensor_device_t* device, drv_Drv_status* status, uint32_t timer, uint32_t upt[]);

#endif /* SENSORS_INC__DRV_BMP388_H_ */
