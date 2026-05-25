/*****************************************************************
 * File Name          : nmpc_platform.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/05/20
 * Description        : Solution for library conflicts between acados and STM32 HAL
*****************************************************************/

#ifndef INC_NMPC_PLATFORM_H_
#define INC_NMPC_PLATFORM_H_

#include <stdint.h>

uint32_t NMPC_PlatformGetTickMs(void);

#endif /* INC_NMPC_PLATFORM_H_ */
