/*
 * sensor2_i2c.c
 *
 *  Created on: Jun 6, 2025
 *      Author: Admin
 */

#include "sensor2_i2c.h"
#include "smtc_hal_dbg_trace.h"

// Хуй

#include "_drv_scd41.h"

static I2C_HandleTypeDef *sensor2_i2c_handles[I2C_MAX_INTERFACES];
static uint8_t sensor2_i2c_num_interfaces;                              // Количество интерфейсов
static Sensor_device_t sensor2_i2c_devices[I2C_MAX_DEVICES];                   // Статический массив устройств
static uint8_t sensor2_i2c_num_devices;

static Sensor_store_data_t readout_data_pool[I2C_MAX_DEVICES];

HAL_StatusTypeDef I2C_Sensor_Init(I2C_HandleTypeDef *i2c_handles[], uint8_t num_interfaces) {
    if (i2c_handles == NULL || num_interfaces == 0 || num_interfaces > I2C_MAX_INTERFACES) {
        return HAL_ERROR;
    }

    // Копируем указатели на интерфейсы
    for (uint8_t i = 0; i < num_interfaces; i++) {
        if (i2c_handles[i] == NULL) {
            return HAL_ERROR;
        }
        sensor2_i2c_handles[i] = i2c_handles[i];
    }
    sensor2_i2c_num_interfaces = num_interfaces;
    sensor2_i2c_num_devices = 0;

    return HAL_OK;
}

HAL_StatusTypeDef I2C_Sensor_Discover_Devices(uint8_t *device_count) {
    if (device_count == NULL) {
        return HAL_ERROR;
    }

    sensor2_i2c_num_devices = 0;

    // Scan devices on I2C buses
    for (uint8_t interface = 0; interface < sensor2_i2c_num_interfaces; interface++) {
        for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
            if (HAL_I2C_IsDeviceReady(sensor2_i2c_handles[interface], addr << 1, 3, 100) == HAL_OK) {
                SMTC_HAL_TRACE_INFO("Found at %#04x at i2c %d\n", addr, interface);
                if (addr == 0x62) {
                	sensirion_i2c_hal_init(sensor2_i2c_handles[interface]);
                }
                if (sensor2_i2c_num_devices >= I2C_MAX_DEVICES) {
                    break; // Reached device limit
                }
                sensor2_i2c_devices[sensor2_i2c_num_devices].interface = interface;
                sensor2_i2c_devices[sensor2_i2c_num_devices].address = addr;
                sensor2_i2c_num_devices++;
            }
        }
    }

    *device_count = sensor2_i2c_num_devices;
    return HAL_OK;
}

HAL_StatusTypeDef I2C_Sensor_Run(uint32_t current_time) {
	if (sensor2_i2c_num_devices == 0 || sensor2_i2c_num_interfaces == 0) {
		return HAL_ERROR;
	}

	for (int i=0; i< sensor2_i2c_num_devices; i++) {
		switch (sensor2_i2c_devices[i].address) {
			case 0x62:
				//SCD41
				static drv_SCD41_Context_t contextDRV;
				static drv_SCD41_results results;
				static uint8_t status;

				DRV_SCD41_run(&contextDRV, &results, current_time, &status);


				break;

			default:
				break;
		}
	}
}



