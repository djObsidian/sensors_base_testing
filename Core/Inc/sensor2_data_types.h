/*
 * sensor2_data_types.h
 *
 *  Created on: Jun 6, 2025
 *      Author: Admin
 */

#ifndef INC_SENSOR2_DATA_TYPES_H_
#define INC_SENSOR2_DATA_TYPES_H_

#include "stdbool.h"

typedef struct {
    uint8_t interface;
    uint8_t address;
    uint8_t poll_period;
} Sensor_device_t;

typedef struct {
    Sensor_device_t sensor;
    bool new_avaliable;
    uint8_t dataLen;
    uint8_t data[8];
} Sensor_store_data_t;

typedef struct {
    uint8_t interface;
    uint8_t address;
    uint8_t dataLen;
    uint8_t data[8];
} SensorData_t;




#endif /* INC_SENSOR2_DATA_TYPES_H_ */
