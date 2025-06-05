/*
 * BH1750.h
 *
 *  Created on: Mar 18, 2025
 *      Author: Admin
 */

#ifndef BH1750_H_
#define BH1750_H_

#include <stdint.h>
#include "main.h" // Заменяем stm32f4xx_hal.h на main.h

// Адреса устройства (7-битные преобразованы в 8-битные для STM32)
#define BH1750_NO_GROUND_ADDR_WRITE     (0x5C << 1) // 0xB8
#define BH1750_NO_GROUND_ADDR_READ      ((0x5C << 1) | 0x01) // 0xB9
#define BH1750_GROUND_ADDR_WRITE        (0x23 << 1) // 0x46
#define BH1750_GROUND_ADDR_READ         ((0x23 << 1) | 0x01) // 0x47

// Команды (из даташита)
#define CMD_POWER_DOWN          0x00
#define CMD_POWER_ON            0x01
#define CMD_RESET               0x03
#define CMD_H_RES_MODE          0x10
#define CMD_H_RES_MODE2         0x11
#define CMD_L_RES_MODE          0x13
#define CMD_ONE_H_RES_MODE      0x20
#define CMD_ONE_H_RES_MODE2     0x21
#define CMD_ONE_L_RES_MODE      0x23
#define CMD_CNG_TIME_HIGH       0x30 // 3 LSB для времени
#define CMD_CNG_TIME_LOW        0x60 // 5 LSB для времени

// Структура устройства BH1750
typedef struct {
    I2C_HandleTypeDef* i2c_handle; // Указатель на I2C
    uint8_t address_r;             // Адрес для чтения
    uint8_t address_w;             // Адрес для записи
    uint16_t value;                // Последнее измеренное значение (люксы)
    uint8_t buffer[2];             // Буфер для сырых данных
    uint8_t mode;                  // Текущий режим работы
} BH1750_device_t;

// Функции
HAL_StatusTypeDef BH1750_Init_Device(BH1750_device_t* dev, I2C_HandleTypeDef* i2c_handle, uint8_t addr_grounded);
HAL_StatusTypeDef BH1750_Read_Device(BH1750_device_t* dev);
HAL_StatusTypeDef BH1750_Get_Lumen(BH1750_device_t* dev);

#endif /* BH1750_H_ */
