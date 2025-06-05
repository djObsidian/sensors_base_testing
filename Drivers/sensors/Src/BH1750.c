#include "BH1750.h"
#include <string.h>

// Вспомогательная функция для отправки команды
static HAL_StatusTypeDef BH1750_Send_Command(BH1750_device_t* dev, uint8_t cmd) {
    if (dev == NULL || dev->i2c_handle == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
        dev->i2c_handle, // I2C Handle
        dev->address_w,  // Адрес устройства для записи
        &cmd,            // Команда
        1,               // Размер данных (1 байт)
        10               // Тайм-аут (мс)
    );

    return status;
}

// Инициализация структуры устройства
HAL_StatusTypeDef BH1750_Init_Device(BH1750_device_t* dev, I2C_HandleTypeDef* i2c_handle, uint8_t addr_grounded) {
    if (dev == NULL || i2c_handle == NULL) {
        return HAL_ERROR;
    }

    // Заполняем структуру
    dev->i2c_handle = i2c_handle;
    if (addr_grounded) {
        dev->address_r = BH1750_GROUND_ADDR_READ;  // 0x47
        dev->address_w = BH1750_GROUND_ADDR_WRITE; // 0x46
    } else {
        dev->address_r = BH1750_NO_GROUND_ADDR_READ;  // 0xB9
        dev->address_w = BH1750_NO_GROUND_ADDR_WRITE; // 0xB8
    }
    dev->value = 0;
    dev->buffer[0] = 0;
    dev->buffer[1] = 0;
    dev->mode = CMD_H_RES_MODE;

    // Инициализация устройства
    HAL_StatusTypeDef status = BH1750_Send_Command(dev, CMD_POWER_ON);
    if (status != HAL_OK) {
        return status;
    }
    status = BH1750_Send_Command(dev, CMD_RESET);
    if (status != HAL_OK) {
        return status;
    }
    status = BH1750_Send_Command(dev, dev->mode);
    return status;
}

// Чтение сырых данных с устройства
HAL_StatusTypeDef BH1750_Read_Device(BH1750_device_t* dev) {
    if (dev == NULL || dev->i2c_handle == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
        dev->i2c_handle, // I2C Handle
        dev->address_r,  // Адрес устройства для чтения
        dev->buffer,     // Буфер для данных
        2,               // Размер данных (2 байта)
        10               // Тайм-аут (мс)
    );

    return status;
}

// Конвертация сырых данных в люксы
static HAL_StatusTypeDef BH1750_Convert(BH1750_device_t* dev) {
    if (dev == NULL) {
        return HAL_ERROR;
    }

    // Сборка 16-битного значения из двух байтов
    dev->value = (uint16_t)(dev->buffer[0] << 8) | dev->buffer[1];

    // Преобразование в люксы: делим на 1.2 с использованием фиксированной точки
    dev->value = (dev->value * 10) / 12;

    return HAL_OK;
}

// Получение значения освещённости
HAL_StatusTypeDef BH1750_Get_Lumen(BH1750_device_t* dev) {
    if (dev == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = BH1750_Read_Device(dev);
    if (status != HAL_OK) {
        return status;
    }

    return BH1750_Convert(dev);
}
