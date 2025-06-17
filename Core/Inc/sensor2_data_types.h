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


#define SENSOR_MAX_DATA_SIZE 16

typedef enum {
	DRV_Wait,
	DRV_DataReady,
	DRV_Sleep
} drv_Drv_status;


/*=== Контексты  драйверов ===*/
typedef struct {
	I2C_HandleTypeDef* i2cHandle;
	uint32_t lastPoll;
	uint32_t pollPeriod;
} drv_sht3x_context_t; //12 байт

typedef struct {
	I2C_HandleTypeDef* i2cHandle;
	uint32_t lastPoll;
	uint8_t driverState;
} drv_scd41_context_t; //9 байт

// Union для всех возможных контекстов
// Зато никаких кастов типов!
// Тут надо соблюдать чтобы каждый тип не занимал много памяти, желательно до 16 байт
// Если тебя когда-то дёрнет чёрт сделать это packed, то бойся невырованенного доступа! Как правильно:
// https://developer.arm.com/documentation/100748/0607/writing-optimized-code/packing-data-structures
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


/*=== Универсальный интерфейс драйверов ===*/
// Указатель на функцию инициализации драйвера устройства
typedef void (*sensor_driver_init_t)(Sensor_device_t* device, I2C_HandleTypeDef* interface, uint32_t upt[]);

// Указатель на функцию цикла драйвера устройства
typedef void (*sensor_driver_run_t)(Sensor_device_t* device, drv_Drv_status* status, uint32_t timer, uint32_t upt[]);

typedef struct {
    uint8_t device_address;
    sensor_driver_init_t driver_init;
    sensor_driver_run_t driver_run;
} Driver_map_t;

/*=== Универсальная таблица параметров ===*/
// Некоторым датчикам требуются калибровочные коэффициенты или другие внешние параметры
// Чтобы не ебаться с интерфейсами это просто будет храниться в карте параметров
typedef enum {
	SP_PRESSURE = 0,
	SP_CAL_PAR_GREEN = 1,
	SP_CAL_PAR_RED = 2,
	SP_CAL_PAR_BLUE = 3,
	SP_CAL_PAR_CLEAR = 4,
	SP_CAL_PYR_GREEN = 5,
	SP_CAL_PYR_RED = 6,
	SP_CAL_PYR_BLUE = 7,
	SP_CAL_PYR_VIS = 8,
	SP_CAL_PYR_IR = 9,
	SP_CAL_PYR_CLEAR = 10
} universalParameterTable_e;





HAL_StatusTypeDef float32_to_fixed_width(float input_val, float min_v, float max_v, uint8_t bytes, uint8_t* out_data);




#endif /* INC_SENSOR2_DATA_TYPES_H_ */
