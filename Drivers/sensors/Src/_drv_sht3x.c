/*
 * _drv_sht3x.c
 *
 *  Created on: Jun 9, 2025
 *      Author: admin
 */

#include "_drv_sht3x.h"
#include "smtc_hal_dbg_trace.h"

void DRV_sht3x_init(Sensor_device_t* device, I2C_HandleTypeDef* interface) {
    if (device == NULL || interface == NULL) {
        return; // Можно заменить на HAL_ERROR
    }

    // Прямое присваивание через union
    device->context.sht3x.i2cHandle = interface;
    device->context.sht3x.lastPoll = 0;
    device->context.sht3x.pollPeriod = 1000;
}

