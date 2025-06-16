/*
 * sensor2_data_types.h
 *
 *  Created on: Jun 6, 2025
 *      Author: Admin
 */

#ifndef INC_SENSOR2_DATA_TYPES_H_
#define INC_SENSOR2_DATA_TYPES_H_

#include "stdbool.h"
#include "main.h"

#include "_drv_common.h"


#define SENSOR_MAX_CONTEXT_SIZE 16
#define SENSOR_MAX_DATA_SIZE 16

typedef struct {
	I2C_HandleTypeDef* i2cHandle;
	uint32_t lastPoll;
	uint32_t pollPeriod;
} drv_sht3x_context_t;

typedef struct {
	I2C_HandleTypeDef* i2cHandle;
	uint32_t lastPoll;
} drv_scd41_context_t;

// Union для всех возможных контекстов
// Тут надо соблюдать чтобы каждый тип не занимал много памяти
typedef union {
	drv_sht3x_context_t sht3x;              		// Контекст для SHT3x
    drv_scd41_context_t scd41;             			// Контекст для SCD41
} Sensor_context_t;

// Данные с сенсора
typedef struct {
	uint8_t dataLen; 								// Длина данных
	bool newAvaliable;								// Доступны ли новые
	uint8_t devceData[SENSOR_MAX_DATA_SIZE];		// Данные
} Sensor_data_t;

// Данные о датчике
typedef struct {
    uint8_t interface;								// Номер интерфейса
    uint8_t address;								// Адрес устройства
    Sensor_context_t deviceContext;					// Контекст устройства
    Sensor_data_t sensorData;						// Данные с сенсора
} Sensor_device_t;

// Указатель на функцию инициализации драйвера устройства
typedef void (*sensor_driver_init_t)(Sensor_device_t* device, I2C_HandleTypeDef* interface);

// Указатель на функцию цикла драйвера устройства
typedef void (*sensor_driver_run_t)(Sensor_device_t* device, drv_Drv_status* status, uint32_t timer);

typedef struct {
    uint8_t device_address;
    sensor_driver_init_t driver_init;
    sensor_driver_run_t driver_run;
} Driver_map_t;


HAL_StatusTypeDef float32_to_fixed_width(float input_val, float min_v, float max_v, uint8_t bytes, uint8_t* out_data);




#endif /* INC_SENSOR2_DATA_TYPES_H_ */
