/*
 * sensor2_i2c.c
 *
 *  Created on: Jun 6, 2025
 *      Author: Admin
 */

#include "sensor2_i2c.h"
#include "smtc_hal_dbg_trace.h"


static I2C_HandleTypeDef *sensor2_i2c_handles[I2C_MAX_INTERFACES];		// Указатели на интерфейсы
static uint8_t sensor2_i2c_num_interfaces;                              // Количество интерфейсов

static Sensor_device_t sensor2_i2c_devices[I2C_MAX_DEVICES];			// Статический массив устройств
static uint8_t sensor2_i2c_num_devices;									// Количество устройств

// Таблица драйверов
Driver_map_t driver_map[] = {
    { 0x62,  DRV_scd41_init, DRV_scd41_run},
    { 0x33, DRV_sht3x_init, DRV_sht3x_run}
    // Другие устройства
};


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

    // Поиск устройств на шинах i2c
    for (uint8_t interface = 0; interface < sensor2_i2c_num_interfaces; interface++) {
        for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
            if (HAL_I2C_IsDeviceReady(sensor2_i2c_handles[interface], addr << 1, 3, 100) == HAL_OK) {
                SMTC_HAL_TRACE_INFO("Found at %#04x at i2c %d\n", addr, interface);
                sensor2_i2c_devices[sensor2_i2c_num_devices].interface = interface;
                sensor2_i2c_devices[sensor2_i2c_num_devices].address = addr;
                sensor2_i2c_num_devices++;
            }
        }
    }

    *device_count = sensor2_i2c_num_devices;

    for (int i = 0; i<sensor2_i2c_num_devices;i++)
    {
		for (size_t j = 0; j < sizeof(driver_map) / sizeof(driver_map[0]); j++)
		{
			if (sensor2_i2c_devices[i].address == driver_map[j].device_address)
			{
				driver_map[j].driver_init(
						&sensor2_i2c_devices[i],
						sensor2_i2c_handles[sensor2_i2c_devices[i].interface]
											);

				break;
			}
		}
    }

    return HAL_OK;
}



