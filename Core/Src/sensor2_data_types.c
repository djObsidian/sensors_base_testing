/*
 * sensor2_data_types.c
 *
 *  Created on: Jun 16, 2025
 *      Author: admin
 */

#include "sensor2_data_types.h"

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

