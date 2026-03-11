/*****************************************************************
 * File Name          : spi_bus_config.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/12/29
 * Description        : SPI bus configuration (multiple devices on
 * 						the same bus with different speeds)
*****************************************************************/


#include "spi_bus_config.h"

static volatile uint8_t spi1_lock = 0;

static void lock_spi1(void) {
    for (;;) {
        __disable_irq();
        if (spi1_lock == 0) {
            spi1_lock = 1;
            __enable_irq();
            break;
        }
        __enable_irq();
    }
}

static void unlock_spi1(void) {
    __disable_irq();
    spi1_lock = 0;
    __enable_irq();
}

void SPI_Bus_Begin(spi_bus_token_t *t, SPI_HandleTypeDef *hspi, uint32_t prescaler_br) {
    t->hspi = hspi;
    t->locked = 0;
    t->saved_br = 0;

    if (hspi == NULL || hspi->Instance == NULL) return;

    if (hspi->Instance == SPI1) {
        lock_spi1();
        t->locked = 1;
    }

    while ((hspi->Instance->SR & SPI_SR_BSY) != 0U) {}

    t->saved_br = (hspi->Instance->CR1 & SPI_CR1_BR_Msk);
    hspi->Instance->CR1 = (hspi->Instance->CR1 & ~SPI_CR1_BR_Msk) | prescaler_br;
}

void SPI_Bus_End(spi_bus_token_t *t) {
    if (t == NULL || t->hspi == NULL || t->hspi->Instance == NULL) return;

    while ((t->hspi->Instance->SR & SPI_SR_BSY) != 0U) {}

    t->hspi->Instance->CR1 = (t->hspi->Instance->CR1 & ~SPI_CR1_BR_Msk) | t->saved_br;

    if (t->locked && t->hspi->Instance == SPI1) {
        unlock_spi1();
    }
}
