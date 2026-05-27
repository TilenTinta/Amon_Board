
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

uint8_t execute_flight_command(s_path *path_data, double x_ref[NMPC_NX_SIZE])
{
	s_flight_command current_command = path_data->flight_path[path_data->path_index]; // Extract current command

	switch (current_command.command)
	{
		// ============================================================
		// TAKE OFF
		// ============================================================
		case COMM_TAKE_OFF:
		{
			s_data_takeoff comm = current_command.takeoff;

			x_ref[2] = comm.height_cm / 100.0;	// Height [m]
			x_ref[6] = 1.0;						// Vertical orientation
			break;
		}

		// ============================================================
		// HOVER
		// ============================================================
		case COMM_HOVER:
		{
			s_data_hover comm = current_command.hover;

			path_data->command_timeout_s = comm.time_s;

			x_ref[0] = 0.0f;
			x_ref[1] = 0.0f;
			x_ref[2] = comm.height_cm / 100.0;
			x_ref[3] = 0.0f;
			x_ref[4] = 0.0f;
			x_ref[5] = 0.0f;
			x_ref[6] = 1.0;		// Keep upright
			break;
		}

		// ============================================================
		// LAND
		// ============================================================
		case COMM_LAND:
		{
			s_data_land comm = current_command.land;

			path_data->command_timeout_s = comm.delay_s;

			x_ref[2] = 0.0f;
			x_ref[3] = 0.0f;
			x_ref[4] = 0.0f;
			x_ref[5] = 0.0f;
			x_ref[6] = 1.0;
			break;
		}

		// ============================================================
		// CHANGE HEIGHT
		// ============================================================
		case COMM_HEIGHT:
		{
			s_data_height comm = current_command.height;

			x_ref[2] = comm.height_cm / 100.0;
			x_ref[5] = comm.speed_cm_s / 100.0;	// Vertical speed

			x_ref[6] = 1.0;
			break;
		}

		// ============================================================
		// FORWARD
		// ============================================================
		case COMM_FORWARD:
		{
			s_data_forward comm = current_command.forward;

			// Move in +X direction
			x_ref[0] += comm.target_distance_cm / 100.0;
			x_ref[3] = comm.speed_cm_s / 100.0;

			break;
		}

		// ============================================================
		// BACKWARD
		// ============================================================
		case COMM_BACKWARD:
		{
			s_data_backward comm = current_command.backward;

			x_ref[0] -= comm.target_distance_cm / 100.0;
			x_ref[3] = -(comm.speed_cm_s / 100.0);

			break;
		}

		// ============================================================
		// LEFT
		// ============================================================
		case COMM_LEFT:
		{
			s_data_left comm = current_command.left;

			x_ref[1] -= comm.target_distance_cm / 100.0;
			x_ref[4] = -(comm.speed_cm_s / 100.0);

			break;
		}

		// ============================================================
		// RIGHT
		// ============================================================
		case COMM_RIGHT:
		{
			s_data_right comm = current_command.right;

			x_ref[1] += comm.target_distance_cm / 100.0;
			x_ref[4] = comm.speed_cm_s / 100.0;

			break;
		}

		// ============================================================
		// ROTATE CW
		// ============================================================
		case COMM_ROTATE_CW:
		{
			s_data_rotate_cw comm = current_command.rotate_cw;

			// Desired yaw rate [rad/s]
			x_ref[12] = -(comm.speed_deg_s * M_PI / 180.0);

			break;
		}

		// ============================================================
		// ROTATE CCW
		// ============================================================
		case COMM_ROTATE_CCW:
		{
			s_data_rotate_ccw comm = current_command.rotate_ccw;

			x_ref[12] = (comm.speed_deg_s * M_PI / 180.0);

			break;
		}

		// ============================================================
		// WAIT
		// ============================================================
		case COMM_WAIT:
		{
			s_data_wait comm = current_command.wait;

			path_data->command_timeout_s = comm.time_s;

			// Hold current state
			break;
		}

		// ============================================================
		// FOLLOW
		// ============================================================
		case COMM_FOLLOW:
		{
			s_data_follow comm = current_command.follow;

			path_data->command_timeout_s = comm.timeout_s;

			// TODO:
			// GPS tracking
			// Line tracking
			// ArUco tracking

			break;
		}

		// ============================================================
		// ACTION
		// ============================================================
		case COMM_ACTION:
		{
			s_data_action comm = current_command.action;

			switch (comm.action_id)
			{
				case ACTION_TAKE_PHOTO:
					break;

				case ACTION_VIDEO_START:
					break;

				case ACTION_VIDEO_STOP:
					break;

				case ACTION_ACTUATOR:
					break;

				case ACTION_LED_ON:
					break;

				case ACTION_LED_OFF:
					break;

				default:
					return 1;
			}

			break;
		}

		// ============================================================
		// RETURN HOME
		// ============================================================
		case COMM_RETURN_HOME:
		{
			s_data_return_home comm = current_command.return_home;

			x_ref[0] = 0.0f;
			x_ref[1] = 0.0f;
			x_ref[2] = comm.height_cm / 100.0;

			x_ref[3] = 0.0f;
			x_ref[4] = 0.0f;
			x_ref[5] = 0.0f;

			x_ref[6] = 1.0f;

			break;
		}

		default:
			// Command does not exist
			return 1; // Error
	}

	return 0; // OK
}

