#ifndef SX126X_CONTEXT_H
#define SX126X_CONTEXT_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "main.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Context structure for SX126X HAL
 */
typedef struct {
    SPI_HandleTypeDef* spi;  // Указатель на SPI (hspi1)
    GPIO_TypeDef* nss_port;  // Порт NSS
    uint16_t nss_pin;        // Пин NSS
    GPIO_TypeDef* reset_port;// Порт RESET
    uint16_t reset_pin;      // Пин RESET
    GPIO_TypeDef* busy_port; // Порт BUSY
    uint16_t busy_pin;       // Пин BUSY
} llcc68_context_t;

//void sx126x_context_init(const sx126x_context_t* ctx);

#ifdef __cplusplus
}
#endif

#endif // SX126X_CONTEXT_H
