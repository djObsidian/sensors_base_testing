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

#define PRINT_BUFFER_SIZE 511

static char string[PRINT_BUFFER_SIZE];

static UART_HandleTypeDef *uartHandle = NULL;

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


    // Use vsnprintf to avoid buffer overflow
    int written = vsnprintf(string, sizeof(string), fmt, argp);

    if (written < 0)
    {
        // Encoding error — optionally print fixed error message
        const char* err = "[TRACE ERROR: format error]\r\n";
        HAL_UART_Transmit(uartHandle, (uint8_t *)err, strlen(err), 1000);
        return;
    }

    // If truncated, we can mark it (optional)
    if ((size_t)written >= sizeof(string))
    {
        // Optional: indicate string was truncated
        const char trunc[] = "...[TRUNC]\r\n";
        // Move last part of buffer to make space for suffix if possible
        size_t trunc_len = strlen(trunc);
        if (trunc_len < sizeof(string)) {
            strncpy(&string[sizeof(string) - trunc_len - 1], trunc, trunc_len);
            string[sizeof(string) - 1] = '\0';
        }
    }

    HAL_UART_Transmit(uartHandle, (uint8_t *)string, strlen(string), 1000);
}
