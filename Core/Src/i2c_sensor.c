/*
 * i2c_sensor.c
 *
 *  Created on: Mar 11, 2025
 *      Author: Admin
 */

#include "i2c_sensor.h"
#include "smtc_hal_dbg_trace.h"

#include "bmp280.h" // Датчик давления и температуры воздуха
#include "BH1750.h" // Датчик освещенности
#include "sht3x.h"  // Датчик температуры и влажности воздуха

HAL_StatusTypeDef I2C_Sensor_Init(I2C_Sensor_Context_t *context, I2C_HandleTypeDef *i2c_handles[], uint8_t num_interfaces) {
    if (context == NULL || i2c_handles == NULL || num_interfaces == 0 || num_interfaces > I2C_MAX_INTERFACES) {
        return HAL_ERROR;
    }

    // Копируем указатели на интерфейсы
    for (uint8_t i = 0; i < num_interfaces; i++) {
        if (i2c_handles[i] == NULL) {
            return HAL_ERROR;
        }
        context->i2c_handles[i] = i2c_handles[i];
    }
    context->num_interfaces = num_interfaces;
    context->num_devices = 0;

    return HAL_OK;
}

HAL_StatusTypeDef I2C_Sensor_DiscoverDevices(I2C_Sensor_Context_t *context, uint8_t *device_count) {
    if (context == NULL || device_count == NULL) {
        return HAL_ERROR;
    }

    context->num_devices = 0;

    // Сканируем устройства на I2C-шинах
    for (uint8_t interface = 0; interface < context->num_interfaces; interface++) {
        for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
            if (HAL_I2C_IsDeviceReady(context->i2c_handles[interface], addr << 1, 3, 100) == HAL_OK) {
                SMTC_HAL_TRACE_INFO("Found at %#04x \n", addr);
                if (context->num_devices >= I2C_MAX_DEVICES) {
                    break; // Достигнут лимит устройств
                }
                context->devices[context->num_devices].interface = interface;
                context->devices[context->num_devices].address = addr;
                context->num_devices++;
            }
        }
    }

    *device_count = context->num_devices;
    return HAL_OK;
}

// Вспомогательные функции чтения с датчиков

static HAL_StatusTypeDef read_device_bmp280(I2C_HandleTypeDef *hi2c, uint8_t *data, uint8_t *data_len) {
    BMP280_HandleTypedef bmp280;
    bmp280_params_t bmpParams;

    bmpParams.filter = BMP280_FILTER_2;
    bmpParams.mode = BMP280_MODE_FORCED;
    bmpParams.oversampling_humidity = BMP280_STANDARD;
    bmpParams.oversampling_pressure = BMP280_STANDARD;
    bmpParams.oversampling_temperature = BMP280_STANDARD;
    bmpParams.standby = 0;

    float pressure, temperature, humidity;

    bmp280.addr = BMP280_I2C_ADDRESS_0;
    bmp280.i2c = hi2c;

    if (!bmp280_init(&bmp280, &bmpParams)) {
        SMTC_HAL_TRACE_INFO("BMP280 init failed.\n");
        *data_len = 0;
        return HAL_ERROR;
    }

    bmp280_force_measurement(&bmp280);

    while (bmp280_is_measuring(&bmp280)) {}

    if (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
        SMTC_HAL_TRACE_INFO("BMP280 read failed.\n");
        *data_len = 0;
        return HAL_ERROR;
    }

    bmpParams.mode = BMP280_MODE_SLEEP;
    if (!bmp280_init(&bmp280, &bmpParams)) {
        SMTC_HAL_TRACE_INFO("BMP280 sleep mode failed.\n");
        *data_len = 0;
        return HAL_ERROR;
    }

    SMTC_HAL_TRACE_INFO("TEMP: %d \n", (int)temperature);
    SMTC_HAL_TRACE_INFO("PRESS: %d \n", (int)pressure);

    if (float32_to_fixed_width(temperature, -20.0f, 60.0f, 1, data) != HAL_OK ||
        float32_to_fixed_width(pressure / 100, 300.0f, 1100.0f, 2, data + 1) != HAL_OK) {
        *data_len = 0;
        return HAL_ERROR;
    }

    *data_len = 3;
    return HAL_OK;
}

static HAL_StatusTypeDef read_device_bh1750(I2C_HandleTypeDef *hi2c, uint8_t *data, uint8_t *data_len) {
    BH1750_device_t bh1750;

    HAL_StatusTypeDef status = BH1750_Init_Device(&bh1750, hi2c, 1);
    if (status != HAL_OK) {
        SMTC_HAL_TRACE_INFO("BH1750 init failed.\n");
        *data_len = 0;
        return HAL_ERROR;
    }

    status = BH1750_Get_Lumen(&bh1750);
    if (status != HAL_OK) {
        SMTC_HAL_TRACE_INFO("LUX FUCK! \n");
        *data_len = 0;
        return HAL_ERROR;
    }

    SMTC_HAL_TRACE_INFO("LUX: %d \n", bh1750.value);
    data[0] = (bh1750.value >> 8) & 0xFF;
    data[1] = (bh1750.value & 0xFF);

    *data_len = 2;
    return HAL_OK;
}

static HAL_StatusTypeDef read_device_sht3x(I2C_HandleTypeDef *hi2c, uint8_t *data, uint8_t *data_len) {
    sht3x_handle_t handle = {
        .i2c_handle = hi2c,
        .device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW
    };

    if (!sht3x_init(&handle)) {
        SMTC_HAL_TRACE_INFO("SHT3x access failed.\n");
        *data_len = 0;
        return HAL_ERROR;
    }

    float temperature, humidity;
    if (!sht3x_read_temperature_and_humidity(&handle, &temperature, &humidity)) {
        SMTC_HAL_TRACE_INFO("SHT3x read failed.\n");
        *data_len = 0;
        return HAL_ERROR;
    }

    SMTC_HAL_TRACE_INFO("SHT3x TEMP: %d \n", (int)temperature);
    SMTC_HAL_TRACE_INFO("SHT3x HUMID: %d \n", (int)humidity);

    if (float32_to_fixed_width(temperature, -40.0f, 100.0f, 2, data) != HAL_OK) {
        *data_len = 0;
        return HAL_ERROR;
    }
    data[2] = (uint8_t)humidity;

    *data_len = 3;
    return HAL_OK;
}

static HAL_StatusTypeDef read_device_sht3x_alt(I2C_HandleTypeDef *hi2c, uint8_t *data, uint8_t *data_len) {
    sht3x_handle_t handle = {
        .i2c_handle = hi2c,
        .device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_HIGH
    };

    if (!sht3x_init(&handle)) {
        SMTC_HAL_TRACE_INFO("SHT3x access failed.\n");
        *data_len = 0;
        return HAL_ERROR;
    }

    float temperature, humidity;
    if (!sht3x_read_temperature_and_humidity(&handle, &temperature, &humidity)) {
        SMTC_HAL_TRACE_INFO("SHT3x read failed.\n");
        *data_len = 0;
        return HAL_ERROR;
    }

    SMTC_HAL_TRACE_INFO("SHT3x_alt TEMP: %d \n", (int)temperature);

    if (float32_to_fixed_width(temperature, -40.0f, 100.0f, 2, data) != HAL_OK) {
        *data_len = 0;
        return HAL_ERROR;
    }

    *data_len = 2;
    return HAL_OK;
}

HAL_StatusTypeDef I2C_Sensor_ReadAll(I2C_Sensor_Context_t *context, SensorData_t *data_array, uint8_t data_array_size) {
    if (context == NULL || data_array == NULL || data_array_size == 0 || data_array_size < context->num_devices) {
        return HAL_ERROR;
    }

    for (uint8_t i = 0; i < context->num_devices; i++) {
        data_array[i].interface = context->devices[i].interface;
        data_array[i].address = context->devices[i].address;
        data_array[i].dataLen = 0;

        HAL_StatusTypeDef status = HAL_OK;
        switch (context->devices[i].address) {
            case 0x23:
                status = read_device_bh1750(context->i2c_handles[context->devices[i].interface], data_array[i].data, &data_array[i].dataLen);
                break;
            case 0x44:
                status = read_device_sht3x(context->i2c_handles[context->devices[i].interface], data_array[i].data, &data_array[i].dataLen);
                break;
            case 0x45:
                status = read_device_sht3x_alt(context->i2c_handles[context->devices[i].interface], data_array[i].data, &data_array[i].dataLen);
                break;
            case 0x76:
                status = read_device_bmp280(context->i2c_handles[context->devices[i].interface], data_array[i].data, &data_array[i].dataLen);
                break;
            default:
                data_array[i].dataLen = 0;
                status = HAL_ERROR;
                break;
        }

        if (status != HAL_OK) {
            data_array[i].dataLen = 0;
            return HAL_ERROR; // Можно продолжить чтение, убрав return, если хотите обработать остальные устройства
        }
    }

    return HAL_OK;
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
