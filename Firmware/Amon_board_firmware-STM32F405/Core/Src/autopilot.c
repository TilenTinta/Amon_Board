
/*****************************************************************
 * File Name          : autopilot.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/04/05
   Description        : Path commander and planning for drone
*****************************************************************/

#include "autopilot.h"



/*###########################################################################################################################################################*/
/* Private functions */



/*###########################################################################################################################################################*/
/* Functions */

uint8_t execute_flight_command(s_path *path, uint8_t step)
{

	switch(command->type)
	{
	    case COMM_TAKE_OFF:
	        //execute_takeoff(...)
	        break;

	    case COMM_FORWARD:
	        //execute_forward(...)
	        break;

	    case COMM_WAIT:
	        //execute_wait(...)
	        break;
	}
};

