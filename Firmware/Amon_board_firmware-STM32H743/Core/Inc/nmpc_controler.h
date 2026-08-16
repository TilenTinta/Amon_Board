/*****************************************************************
 * File Name          : nmpc_controler.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/08/04
 * Description        : NMPC controller wrapper - no acados code
*****************************************************************/

#ifndef INC_NMPC_CONTROLER_H_
#define INC_NMPC_CONTROLER_H_

#include <stdint.h>
#include "autopilot.h"
#include "drone_data.h"
#include "PWM.h"
#include "NMPC.h"

/*###########################################################################################################################################################*/
/* Functions */

//uint32_t NMPC_PlatformGetTickMs(void);
void NMPC_ControllerUpdate(s_NMPC *nmpc, s_drone_data *drone);

#endif /* INC_NMPC_CONTROLER_H_ */
