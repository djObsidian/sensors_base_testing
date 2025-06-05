/*
 * sensor_struct.h
 *
 *  Created on: Apr 15, 2025
 *      Author: Admin
 */

#ifndef INC_SENSOR_STRUCT_H_
#define INC_SENSOR_STRUCT_H_


typedef struct {
    uint8_t interface;
    uint8_t address;
    uint8_t dataLen;
    uint8_t data[8];
} SensorData_t;

typedef struct {
    uint8_t interface;
    uint8_t address;
} Device_t;


#endif /* INC_SENSOR_STRUCT_H_ */
