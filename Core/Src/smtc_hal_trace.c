/*!
 * \file      smtc_hal_trace.c
 *
 * \brief     Trace Print Hardware Abstraction Layer implementation (memory-safe)
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "smtc_hal_trace.h"

#define TRACE_BUFFER_SIZE 2048
static uint8_t trace_buffer[TRACE_BUFFER_SIZE];
volatile uint32_t buf_head = 0, buf_tail = 0;
volatile uint8_t uart_busy = 0;
static UART_HandleTypeDef *uartHandle; // Ваш UART хэндл
static volatile uint32_t last_transmit_len = 0; // Для хранения длины текущей передачи

void start_next_transmission(void);

void hal_trace_print_init(UART_HandleTypeDef *huart) {
    uartHandle = huart;
}

void hal_trace_print_var(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    hal_trace_print(fmt, args);
    va_end(args);
}

void hal_trace_print(const char* fmt, va_list argp)
{
    char temp[256];
    // Формируем строку
    int written = vsnprintf(temp, sizeof(temp), fmt, argp);
    if (written < 0 || written >= (int)sizeof(temp)) {
        written = snprintf(temp, sizeof(temp), "[TRACE ERROR]\r\n");
    }

    // Проверяем доступное место в буфере
    uint32_t space = (buf_tail - buf_head - 1 + TRACE_BUFFER_SIZE) % TRACE_BUFFER_SIZE;
    if ((uint32_t)written > space) {
        // Буфер переполнен, пропускаем (можно добавить логирование ошибки)
        return;
    }

    // Копируем данные в кольцевой буфер
    for (int i = 0; i < written; i++) {
        trace_buffer[buf_head] = temp[i];
        buf_head = (buf_head + 1) % TRACE_BUFFER_SIZE;
    }

    // Запускаем передачу, если UART свободен
    if (!uart_busy) {
        start_next_transmission();
    }
}

void start_next_transmission(void)
{
    if (buf_head == buf_tail) {
        uart_busy = 0; // Нет данных для отправки
        return;
    }

    // Определяем размер следующего блока
    uint32_t len = (buf_head >= buf_tail) ? buf_head - buf_tail : TRACE_BUFFER_SIZE - buf_tail;
    uart_busy = 1;
    last_transmit_len = len; // Сохраняем длину для callback
    HAL_UART_Transmit_DMA(uartHandle, &trace_buffer[buf_tail], len);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // Обновляем buf_tail на количество переданных байтов
        buf_tail = (buf_tail + last_transmit_len) % TRACE_BUFFER_SIZE;
        // Запускаем следующую передачу
        start_next_transmission();
    }
}
