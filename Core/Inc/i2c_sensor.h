/*
 * i2c_sensor.h
 *
 *  Created on: Mar 11, 2025
 *      Author: Admin
 */

#ifndef INC_I2C_SENSOR_H_
#define INC_I2C_SENSOR_H_

#include "main.h"
#include "sensor_struct.h"

// Максимальное количество устройств и интерфейсов
#define I2C_MAX_DEVICES 32
#define I2C_MAX_INTERFACES 3

// Контекст для работы с I2C-датчиками
typedef struct {
    I2C_HandleTypeDef *i2c_handles[I2C_MAX_INTERFACES]; // Указатели на интерфейсы
    uint8_t num_interfaces;                              // Количество интерфейсов
    Device_t devices[I2C_MAX_DEVICES];                   // Статический массив устройств
    uint8_t num_devices;                                 // Текущее количество устройств
} I2C_Sensor_Context_t;

// Инициализация контекста I2C-датчиков
HAL_StatusTypeDef I2C_Sensor_Init(I2C_Sensor_Context_t *context, I2C_HandleTypeDef *i2c_handles[], uint8_t num_interfaces);

// Получение количества найденных устройств
HAL_StatusTypeDef I2C_Sensor_DiscoverDevices(I2C_Sensor_Context_t *context, uint8_t *device_count);

// Чтение данных со всех датчиков
HAL_StatusTypeDef I2C_Sensor_ReadAll(I2C_Sensor_Context_t *context, SensorData_t *data_array, uint8_t data_array_size);

// Преобразование float в фиксированный формат
HAL_StatusTypeDef float32_to_fixed_width(float input_val, float min_v, float max_v, uint8_t bytes, uint8_t* out_data);

#endif /* INC_I2C_SENSOR_H_ */
