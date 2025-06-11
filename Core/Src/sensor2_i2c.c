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


// Статические переменные

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
			case 0x33:
				break;


			case 0x62: //SCD41
				static drv_SCD41_Context_t contextDRV;
				static drv_SCD41_results results;
				static drv_SDC41_Drv_status status;

				DRV_SCD41_run(&contextDRV, &results, current_time, &status);

				if (status == DRV_DataReady) {
					readout_data_pool[i].sensor = sensor2_i2c_devices[i];
					readout_data_pool[i].new_avaliable = true;
					readout_data_pool[i].dataLen = 5;

					readout_data_pool[i].data[0] = (results.co2_concentration >> 8) & 0xFF;
					readout_data_pool[i].data[1] = results.co2_concentration & 0xFF;

					float32_to_fixed_width(results.temperature/1000.0f, -40.0f, 100.0f, 2, readout_data_pool[i].data+2);

					readout_data_pool[i].data[4] = (uint8_t)results.relative_humidity/1000;

				}


				break;

			default:
				break;
		}
	}
}

HAL_StatusTypeDef float32_to_fixed_width(float input_val, float min_v, float max_v, uint8_t bytes, uint8_t* out_data) {
    if (out_data == NULL || (bytes != 1 && bytes != 2)) {
        return HAL_ERROR;
    }

    if (input_val < min_v) {
        for (int i = 0; i < bytes; i++) {
            out_data[i] = 0x0;
        }
        return HAL_OK;
    }

    if (input_val > max_v) {
        for (int i = 0; i < bytes; i++) {
            out_data[i] = 0xFF;
        }
        return HAL_OK;
    }

    // Линейная интерполяция
    float fx0 = 0.0f + 1;
    float fx1;

    switch (bytes) {
        case 1:
            fx1 = 254.0f; // max uint8 - 1
            break;
        case 2:
            fx1 = 65534.0f; // max uint16 - 1
            break;
        default:
            return HAL_ERROR; // Уже проверено выше, но для безопасности
    }

    float linresult = fx0 + (fx1 - fx0) / (max_v - min_v) * (input_val - min_v);

    switch (bytes) {
        case 1:
            out_data[0] = (uint8_t)linresult;
            break;
        case 2:
            uint16_t result16 = (uint16_t)linresult;
            out_data[0] = (result16 >> 8) & 0xFF;
            out_data[1] = (result16 & 0xFF);
            break;
        default:
            return HAL_ERROR; // Уже проверено выше
    }

    return HAL_OK;
}



