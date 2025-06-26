#include "llcc68_hal.h"
#include "llcc68_context.h"
#include "main.h" // Замените на вашу серию STM32

// Вспомогательная функция для управления NSS
static inline void set_nss(llcc68_context_t* ctx, uint8_t state) {
    HAL_GPIO_WritePin(ctx->nss_port, ctx->nss_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Сброс радио
llcc68_hal_status_t llcc68_hal_reset(const void* context) {
    llcc68_context_t* ctx = (llcc68_context_t*)context;
    HAL_GPIO_WritePin(ctx->reset_port, ctx->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(10); // Задержка 10 мс
    HAL_GPIO_WritePin(ctx->reset_port, ctx->reset_pin, GPIO_PIN_SET);
    HAL_Delay(10); // Задержка для стабилизации
    return LLCC68_HAL_STATUS_OK;
}

// Пробуждение радио
llcc68_hal_status_t llcc68_hal_wakeup(const void* context) {
    llcc68_context_t* ctx = (llcc68_context_t*)context;
    set_nss(ctx, 0); // Активировать NSS
    HAL_Delay(1);
    set_nss(ctx, 1); // Деактивировать NSS
    return LLCC68_HAL_STATUS_OK;
}

// Запись данных через SPI
llcc68_hal_status_t llcc68_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length) {
    llcc68_context_t* ctx = (llcc68_context_t*)context;

    set_nss(ctx, 0); // Активировать NSS

    // Отправка команды
    if (HAL_SPI_Transmit(ctx->spi, (uint8_t*)command, command_length, HAL_MAX_DELAY) != HAL_OK) {
        set_nss(ctx, 1);
        return LLCC68_HAL_STATUS_ERROR;
    }

    // Отправка данных
    if (data_length > 0) {
        if (HAL_SPI_Transmit(ctx->spi, (uint8_t*)data, data_length, HAL_MAX_DELAY) != HAL_OK) {
            set_nss(ctx, 1);
            return LLCC68_HAL_STATUS_ERROR;
        }
    }

    set_nss(ctx, 1); // Деактивировать NSS
    return LLCC68_HAL_STATUS_OK;
}

// Чтение данных через SPI
llcc68_hal_status_t llcc68_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length) {
    llcc68_context_t* ctx = (llcc68_context_t*)context;

    set_nss(ctx, 0); // Активировать NSS

    // Отправка команды
    if (HAL_SPI_Transmit(ctx->spi, (uint8_t*)command, command_length, HAL_MAX_DELAY) != HAL_OK) {
        set_nss(ctx, 1);
        return LLCC68_HAL_STATUS_ERROR;
    }

    // Чтение данных
    if (HAL_SPI_Receive(ctx->spi, data, data_length, HAL_MAX_DELAY) != HAL_OK) {
        set_nss(ctx, 1);
        return LLCC68_HAL_STATUS_ERROR;
    }

    set_nss(ctx, 1); // Деактивировать NSS
    return LLCC68_HAL_STATUS_OK;
}
