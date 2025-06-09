/*
 * sensor2_i2c.h
 *
 *  Created on: Jun 6, 2025
 *      Author: Admin
 */

#ifndef INC_SENSOR2_I2C_H_
#define INC_SENSOR2_I2C_H_

#include "main.h"
#include "sensor2_data_types.h"

// Максимальное количество устройств и интерфейсов
#define I2C_MAX_DEVICES 32
#define I2C_MAX_INTERFACES 3

HAL_StatusTypeDef I2C_Sensor_Init(I2C_HandleTypeDef* i2c_handles[], uint8_t num_interfaces);

HAL_StatusTypeDef I2C_Sensor_Discover_Devices(uint8_t* device_count);

HAL_StatusTypeDef I2C_Sensor_Get_Discovered_Devices(Sensor_device_t* device_info);

HAL_StatusTypeDef I2C_Sensor_Run(uint32_t current_time);

HAL_StatusTypeDef I2C_Sensor_Fetch_Data(SensorData_t* data_array, uint8_t data_array_size);




#endif /* INC_SENSOR2_I2C_H_ */
