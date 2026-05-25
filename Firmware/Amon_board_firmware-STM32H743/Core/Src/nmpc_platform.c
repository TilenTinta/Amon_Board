/*****************************************************************
 * File Name          : nmpc_platform.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/05/20
 * Description        : Solution for library conflicts between acados and STM32 HAL
*****************************************************************/

#include "nmpc_platform.h"

#include "stm32h7xx_hal.h"

uint32_t NMPC_PlatformGetTickMs(void)
{
    return HAL_GetTick();
}
