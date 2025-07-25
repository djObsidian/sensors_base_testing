/*
 * sensor2_i2c.c
 *
 *  Created on: Jun 6, 2025
 *      Author: Admin
 */

#include <_drv_bmp388.h>
#include "sensor2_i2c.h"
#include "smtc_hal_dbg_trace.h"

#include "_drv_sht3x.h"
#include "_drv_scd41.h"

#define MAX_PCKT_LEN 51

static I2C_HandleTypeDef *sensor2_i2c_handles[I2C_MAX_INTERFACES];		// Указатели на интерфейсы
static uint8_t sensor2_i2c_num_interfaces;                              // Количество интерфейсов

static Sensor_device_t sensor2_i2c_devices[I2C_MAX_DEVICES];			// Статический массив устройств
static uint8_t sensor2_i2c_num_devices;									// Количество устройств

// Таблица драйверов
Driver_map_t driver_map[] = {
    { 0x62, DRV_scd41_init, DRV_scd41_run},
    { 0x44, DRV_sht3x_init, DRV_sht3x_run},
	{ 0x45, DRV_sht3x_init, DRV_sht3x_run},
	{ 0x76, DRV_bmp388_init, DRV_bmp388_run}
    // Другие устройства
};

// Универсальная таблица параметров
static upt_cell_t universalParamTable[UPT_MAX_SIZE];


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
						sensor2_i2c_handles[sensor2_i2c_devices[i].interface],
						universalParamTable
						);

				break;
			}
		}
    }

    return HAL_OK;
}

HAL_StatusTypeDef I2C_Sensor_Run(uint32_t deltaTime) {

	// Осторожно, точные тайминги тут не поддерживаются, т.к. драйвера не имеют фиксированного времени выполнения
	// Последующий вызов может случиться с реальным deltaTime больше чем данное
	for (int i = 0; i<sensor2_i2c_num_devices;i++)
	{
		for (size_t j = 0; j < sizeof(driver_map) / sizeof(driver_map[0]); j++)
		{
			if (sensor2_i2c_devices[i].address == driver_map[j].device_address)
			{
				drv_Drv_status statusNow;

				driver_map[j].driver_run(
						&sensor2_i2c_devices[i],
						&statusNow,
						deltaTime,
						universalParamTable
						);

				break;
			}
		}
	}

	return HAL_OK;
}

#define MAX_PCKT_LEN 255 // Максимальная длина пакета (uint8_t ограничение)

HAL_StatusTypeDef I2C_Sensor_Fetch_Data(uint8_t data_array[], uint8_t* data_array_size)
{
    uint16_t current_index = 0; // Текущая позиция в data_array
    uint8_t last_interface = 0xFF; // Последний записанный интерфейс (инициализируем несуществующим значением)

    // Проверка входных параметров
    if (data_array == NULL || data_array_size == NULL) {
        return HAL_ERROR;
    }

    // Проходим по всем устройствам
    for (int i = 0; i < sensor2_i2c_num_devices; i++) {
        Sensor_device_t* device = &sensor2_i2c_devices[i];

        // Проверяем наличие новых данных
        if (device->sensorData.newAvaliable) {
            // Проверяем, достаточно ли места для интерфейса, если он изменился
            if (device->interface != last_interface) {
                if (current_index >= MAX_PCKT_LEN) {
                    break; // Прекращаем упаковку, если превышен лимит
                }
                data_array[current_index++] = device->interface + 1;
                last_interface = device->interface;
            }

            // Проверяем, достаточно ли места для адреса и данных
            if (current_index + 1 + device->sensorData.dataLen > MAX_PCKT_LEN) {
                break; // Прекращаем упаковку, если превышен лимит
            }

            // Записываем адрес устройства
            data_array[current_index++] = device->address;

            // Копируем данные сенсора
            for (uint8_t j = 0; j < device->sensorData.dataLen; j++) {
                data_array[current_index++] = device->sensorData.deviceData[j];
            }

            // Сбрасываем флаг новых данных
            device->sensorData.newAvaliable = false;
        }
    }

    // Записываем фактический размер
    *data_array_size = (uint8_t)current_index;

    return current_index > 0 ? HAL_OK : HAL_BUSY; // HAL_OK если данные были, HAL_BUSY если нет
}



