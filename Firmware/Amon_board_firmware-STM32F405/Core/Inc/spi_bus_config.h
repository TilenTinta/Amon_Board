/*****************************************************************
 * File Name          : spi_bus_config.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/12/29
 * Description        : SPI bus configuration (multiple devices on
 * 						the same bus with different speeds)
*****************************************************************/

#ifndef INC_SPI_BUS_CONFIG_H_
#define INC_SPI_BUS_CONFIG_H_

#include "stm32f4xx_hal.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    uint32_t saved_br;
    uint8_t locked;
} spi_bus_token_t;

void SPI_Bus_Begin(spi_bus_token_t *t, SPI_HandleTypeDef *hspi, uint32_t prescaler_br);
void SPI_Bus_End(spi_bus_token_t *t);


#endif /* INC_SPI_BUS_CONFIG_H_ */
