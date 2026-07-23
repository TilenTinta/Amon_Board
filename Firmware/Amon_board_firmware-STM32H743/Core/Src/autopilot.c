
/*****************************************************************
 * File Name          : autopilot.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/04/05
   Description        : Path commander and planning for drone
*****************************************************************/

#include "autopilot.h"
#include "PWM.h"

/*###########################################################################################################################################################*/
/* Info */

/*
 * ## NMPC state
 * Both following arrays have the same fields:
 * 	- x_current = measured/estimated current drone state
 * 	- x_ref     = desired/reference drone state for NMPC cost
 *
 * Common states for all model types:
 * 	x[0]  = px [m]
 * 	x[1]  = py [m]
 * 	x[2]  = pz [m]
 *
 * 	x[3]  = vx [m/s]
 * 	x[4]  = vy [m/s]
 * 	x[5]  = vz [m/s]
 *
 * 	x[6]  = qw [-]
 * 	x[7]  = qx [-]
 * 	x[8]  = qy [-]
 * 	x[9]  = qz [-]
 *
 * 	x[10] = wx [rad/s]
 * 	x[11] = wy [rad/s]
 * 	x[12] = wz [rad/s]
 *
 * Instant model:
 * 	NMPC_NX_SIZE = 13
 *
 * First order actuator model:
 * 	NMPC_NX_SIZE = 18
 * 	x[13] = T       [N]
 * 	x[14] = delta_1 [rad]
 * 	x[15] = delta_2 [rad]
 * 	x[16] = delta_3 [rad]
 * 	x[17] = delta_4 [rad]
 *
 * Second order actuator model:
 * 	NMPC_NX_SIZE = 23
 * 	x[13] = T           [N]
 * 	x[14] = T_dot       [N/s]
 * 	x[15] = delta_1     [rad]
 * 	x[16] = delta_2     [rad]
 * 	x[17] = delta_3     [rad]
 * 	x[18] = delta_4     [rad]
 * 	x[19] = delta_dot_1 [rad/s]
 * 	x[20] = delta_dot_2 [rad/s]
 * 	x[21] = delta_dot_3 [rad/s]
 * 	x[22] = delta_dot_4 [rad/s]
 *
 *
 *
 * NMPC input vector - size NMPC_NU = 5
 * Both following arrays has the same fields:
 * 	- u_ref = desired nominal input for NMPC cost, usually hover thrust and neutral trims.
 * 	- u     = optimized control output from NMPC.
 *
 * 	Fields:
 * 	u[0] = thrust / EDF command [%], range 0..100
 * 	u[1] = servo x+ command [deg], range -45..45
 * 	u[2] = servo x- command [deg], range -45..45
 * 	u[3] = servo y+ command [deg], range -45..45
 * 	u[4] = servo y- command [deg], range -45..45
 *
 */



/*###########################################################################################################################################################*/
/* Private functions */
static void ref_clear(double x_ref[NMPC_NX_SIZE]);
static void ref_upright(double x_ref[NMPC_NX_SIZE]);
static void apply_vertical_outer_loop(s_path *path_data, double z_current_m,
		double vz_min_m_s, double vz_slew_m_s2,
		double x_ref[NMPC_NX_SIZE]);

static void ref_take_off(s_data_takeoff *data, double x_ref[NMPC_NX_SIZE]);
static void ref_land(s_data_land *data, double x_ref[NMPC_NX_SIZE]);
static void ref_height(s_data_height *data, double x_ref[NMPC_NX_SIZE]);
static void ref_forward(s_data_forward *data, double x_ref[NMPC_NX_SIZE]);
static void ref_backward(s_data_backward *data, double x_ref[NMPC_NX_SIZE]);
static void ref_left(s_data_left *data, double x_ref[NMPC_NX_SIZE]);
static void ref_right(s_data_right *data, double x_ref[NMPC_NX_SIZE]);
static void ref_rotate_cw(s_data_rotate_cw *data, double x_ref[NMPC_NX_SIZE]);
static void ref_rotate_ccw(s_data_rotate_ccw *data, double x_ref[NMPC_NX_SIZE]);
static void ref_hover(s_data_hover *data, double x_ref[NMPC_NX_SIZE]);
static void ref_return_home(s_data_return_home *data, double x_ref[NMPC_NX_SIZE]);


/*###########################################################################################################################################################*/
/* Functions */

uint8_t execute_flight_command(s_path *path_data, double x_ref[NMPC_NX_SIZE], double z_current_m)
{
	s_flight_command current_command = path_data->flight_path[path_data->command_index]; // Extract current command
	double vz_min_m_s = VZ_REF_MIN_M_S;

	switch (current_command.command)
	{
		// ============================================================
		// TAKE OFF
		// ============================================================
		case COMM_TAKE_OFF:
		{
			ref_take_off(&current_command.takeoff, x_ref);
			path_data->altitude_ref_m = x_ref[2];
			path_data->command_timeout_s = 0;
			break;
		}

		// ============================================================
		// HOVER
		// ============================================================
		case COMM_HOVER:
		{
			path_data->command_timeout_s = current_command.hover.time_s;
			ref_hover(&current_command.hover, x_ref);
			path_data->altitude_ref_m = x_ref[2];
			break;
		}

		// ============================================================
		// LAND
		// ============================================================
		case COMM_LAND:
		{
			//path_data->command_timeout_s = current_command.land.delay_s;
			path_data->command_timeout_s = 0;
			ref_land(&current_command.land, x_ref);
			path_data->altitude_ref_m = x_ref[2];
			vz_min_m_s = VZ_REF_LAND_MIN_M_S;
			break;
		}

		// ============================================================
		// CHANGE HEIGHT
		// ============================================================
		case COMM_HEIGHT:
		{
			ref_height(&current_command.height, x_ref);
			path_data->altitude_ref_m = x_ref[2];
			path_data->command_timeout_s = 0;
			break;
		}

		// ============================================================
		// FORWARD
		// ============================================================
		case COMM_FORWARD:
		{
			ref_forward(&current_command.forward, x_ref);
			path_data->command_timeout_s = 0;
			break;
		}

		// ============================================================
		// BACKWARD
		// ============================================================
		case COMM_BACKWARD:
		{
			ref_backward(&current_command.backward, x_ref);
			path_data->command_timeout_s = 0;
			break;
		}

		// ============================================================
		// LEFT
		// ============================================================
		case COMM_LEFT:
		{
			ref_left(&current_command.left, x_ref);
			path_data->command_timeout_s = 0;
			break;
		}

		// ============================================================
		// RIGHT
		// ============================================================
		case COMM_RIGHT:
		{
			ref_right(&current_command.right, x_ref);
			path_data->command_timeout_s = 0;
			break;
		}

		// ============================================================
		// ROTATE CW
		// ============================================================
		case COMM_ROTATE_CW:
		{
			ref_rotate_cw(&current_command.rotate_cw, x_ref);
			path_data->command_timeout_s = 0;
			break;
		}

		// ============================================================
		// ROTATE CCW
		// ============================================================
		case COMM_ROTATE_CCW:
		{
			ref_rotate_ccw(&current_command.rotate_ccw, x_ref);
			path_data->command_timeout_s = 0;
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
			//path_data->command_timeout_s = comm.timeout_s;
			path_data->command_timeout_s = 0;

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
			ref_return_home(&current_command.return_home, x_ref);
			path_data->altitude_ref_m = x_ref[2];
			path_data->command_timeout_s = 0;
			break;
		}

		default:
			// Command does not exist
			return 1; // Error
	}

	// Tune vertical speed to stabilize altitude
	x_ref[2] = path_data->altitude_ref_m;
	apply_vertical_outer_loop(path_data, z_current_m, vz_min_m_s, VZ_REF_SLEW_M_S2, x_ref);

	return 0; // OK
}


/*********************************************************************
 * @fcn    apply_vertical_outer_loop
 *
 * @param  z_current_m: filtered current altitude [m]
 * @param  x_ref: NMPC reference vector; x_ref[2] must contain z target
 *
 * @brief  Convert altitude error into a bounded vertical-speed reference.
 *
 * @return none
 */
static void apply_vertical_outer_loop(s_path *path_data, double z_current_m, double vz_min_m_s, double vz_slew_m_s2, double x_ref[NMPC_NX_SIZE])
{
	// Calculate speed and direction based on error/reference
	double z_err = x_ref[2] - z_current_m;
	double vz_ref_target;

	if (fabs(z_err) < Z_DEADBAND_M)
	{
		vz_ref_target = 0.0;
	}
	else
	{
		vz_ref_target = Z_OUTER_KP * z_err;
	}

    // Use defined values based on direction (up/down)
    if (vz_ref_target > VZ_REF_MAX_M_S) vz_ref_target = VZ_REF_MAX_M_S;
    if (vz_ref_target < vz_min_m_s) vz_ref_target = vz_min_m_s;

    path_data->vz_ref_target_m_s = vz_ref_target;

    float max_step = (float)(vz_slew_m_s2 * AUTOPILOT_UPDATE_DT_S);
    float vz_ref = SlewLimit((float)vz_ref_target, (float)path_data->vz_ref_m_s, max_step);

    path_data->vz_ref_m_s = vz_ref;
    x_ref[5] = vz_ref;
}



/*********************************************************************
 * @fcn    	ref_clear
 *
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Clear complete reference array
 *
 * @return  none
 */
static void ref_clear(double x_ref[NMPC_NX_SIZE])
{
    for (uint8_t i = 0; i < NMPC_NX_SIZE; i++)
    {
        x_ref[i] = 0.0;
    }

}



/*********************************************************************
 * @fcn    	ref_upright
 *
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for quaternion to keep
 * 			drone upright oriented
 *
 * @return  none
 */
static void ref_upright(double x_ref[NMPC_NX_SIZE])
{
    x_ref[6] = 1.0;   // qw
    x_ref[7] = 0.0;   // qx
    x_ref[8] = 0.0;   // qy
    x_ref[9] = 0.0;   // qz

}



/*********************************************************************
 * @fcn    	ref_take_off
 *
 * @param 	*data: pointer to all command data
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for current command
 *
 * @return  none
 */
static void ref_take_off(s_data_takeoff *data, double x_ref[NMPC_NX_SIZE])
{
	ref_clear(x_ref);						// Set everything to zero
	ref_upright(x_ref);						// Set quaternion at 1,0,0,0 to keep drone upright

	// Position
	x_ref[0] = 0.0;                          // hold X
	x_ref[1] = 0.0;                          // hold Y
	x_ref[2] = data->height_cm / 100.0;      // target altitude

	// Velocity
	x_ref[3] = 0.0;
	x_ref[4] = 0.0;
	x_ref[5] = 0.0; // If value is fixed drone overshoots and never reaches the target - maybe use scaling based on height

}



/*********************************************************************
 * @fcn    	ref_land
 *
 * @param 	*data: pointer to all command data
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for current command
 *
 * @return  none
 */
static void ref_land(s_data_land *data, double x_ref[NMPC_NX_SIZE])
{
    ref_clear(x_ref);
    ref_upright(x_ref);

    x_ref[2] = 0.0;
    x_ref[5] = 0.0;      // landing speed, hardcoded for first test

}



/*********************************************************************
 * @fcn    	ref_height
 *
 * @param 	*data: pointer to all command data
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for current command
 *
 * @return  none
 */
static void ref_height(s_data_height *data, double x_ref[NMPC_NX_SIZE])
{
    ref_clear(x_ref);
    ref_upright(x_ref);

    x_ref[2] = data->height_cm / 100.0;
    x_ref[5] = data->speed_cm_s / 100.0;

}



/*********************************************************************
 * @fcn    	ref_forward
 *
 * @param 	*data: pointer to all command data
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for current command
 *
 * @return  none
 */
static void ref_forward(s_data_forward *data, double x_ref[NMPC_NX_SIZE])
{
    ref_clear(x_ref);
    ref_upright(x_ref);

    x_ref[0] = data->target_distance_cm / 100.0;
    x_ref[3] = data->speed_cm_s / 100.0;

}



/*********************************************************************
 * @fcn    	ref_backward
 *
 * @param 	*data: pointer to all command data
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for current command
 *
 * @return  none
 */
static void ref_backward(s_data_backward *data, double x_ref[NMPC_NX_SIZE])
{
    ref_clear(x_ref);
    ref_upright(x_ref);

    x_ref[0] = -(data->target_distance_cm / 100.0);
    x_ref[3] = -(data->speed_cm_s / 100.0);

}



/*********************************************************************
 * @fcn    	ref_left
 *
 * @param 	*data: pointer to all command data
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for current command
 *
 * @return  none
 */
static void ref_left(s_data_left *data, double x_ref[NMPC_NX_SIZE])
{
    ref_clear(x_ref);
    ref_upright(x_ref);

    x_ref[1] = -(data->target_distance_cm / 100.0);
    x_ref[4] = -(data->speed_cm_s / 100.0);

}



/*********************************************************************
 * @fcn    	ref_right
 *
 * @param 	*data: pointer to all command data
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for current command
 *
 * @return  none
 */
static void ref_right(s_data_right *data, double x_ref[NMPC_NX_SIZE])
{
    ref_clear(x_ref);
    ref_upright(x_ref);

    x_ref[1] = data->target_distance_cm / 100.0;
    x_ref[4] = data->speed_cm_s / 100.0;

}



/*********************************************************************
 * @fcn    	ref_rotate_cw
 *
 * @param 	*data: pointer to all command data
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for current command
 *
 * @return  none
 */
static void ref_rotate_cw(s_data_rotate_cw *data, double x_ref[NMPC_NX_SIZE])
{
    ref_clear(x_ref);
    ref_upright(x_ref);

    x_ref[12] = -(data->speed_deg_s * M_PI / 180.0);

}



/*********************************************************************
 * @fcn    	ref_rotate_ccw
 *
 * @param 	*data: pointer to all command data
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for current command
 *
 * @return  none
 */
static void ref_rotate_ccw(s_data_rotate_ccw *data, double x_ref[NMPC_NX_SIZE])
{
    ref_clear(x_ref);
    ref_upright(x_ref);

    x_ref[12] = data->speed_deg_s * M_PI / 180.0;

}



/*********************************************************************
 * @fcn    	ref_hover
 *
 * @param 	*data: pointer to all command data
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for current command
 *
 * @return  none
 */
static void ref_hover(s_data_hover *data, double x_ref[NMPC_NX_SIZE])
{
    ref_clear(x_ref);
    ref_upright(x_ref);

    x_ref[0] = 0.0;
    x_ref[1] = 0.0;
    x_ref[2] = data->height_cm / 100.0;

    x_ref[3] = 0.0;
    x_ref[4] = 0.0;
    x_ref[5] = 0.0;

}



/*********************************************************************
 * @fcn    	ref_return_home
 *
 * @param 	*data: pointer to all command data
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Fill reference value for current command
 *
 * @return  none
 */
static void ref_return_home(s_data_return_home *data, double x_ref[NMPC_NX_SIZE])
{
    ref_clear(x_ref);
    ref_upright(x_ref);

    x_ref[0] = 0.0;
    x_ref[1] = 0.0;
    x_ref[2] = data->height_cm / 100.0;

    x_ref[3] = 0.0;
    x_ref[4] = 0.0;
    x_ref[5] = data->speed_cm_s / 100.0;

}



/*********************************************************************
 * @fcn    	land_now
 *
 * @param 	pos_x: data for x position
 * @param 	pos_y: data for y position
 *
 * @brief   Fill reference value for current command - land now
 *
 * @return  none
 */
void land_now(s_path *path_data, double x_ref[NMPC_NX_SIZE], double current_x_m, double current_y_m, double z_current_m)
{
    if (path_data != NULL)
    {
        path_data->command_cnt = 0;
        path_data->command_index = 0;
        path_data->command_time_s = 0.0f;
        path_data->command_timeout_s = 0;
        path_data->altitude_ref_m = 0.0;
    }

    ref_clear(x_ref);
    ref_upright(x_ref);

    // Hold horizontal position where land_now was requested
    x_ref[0] = current_x_m;
    x_ref[1] = current_y_m;

    // Land target
    x_ref[2] = 0.0;             // ground height
    x_ref[3] = 0.0;             // no X velocity
    x_ref[4] = 0.0;             // no Y velocity
    apply_vertical_outer_loop(path_data, z_current_m,
			VZ_REF_LAND_MIN_M_S, VZ_REF_LAND_NOW_SLEW_M_S2, x_ref);

    // Upright attitude is already set by ref_upright()
    x_ref[10] = 0.0;
    x_ref[11] = 0.0;
    x_ref[12] = 0.0;
}



/*********************************************************************
 * @fcn    	command_position_reached
 *
 * @param 	*cmd: pointer to current command data
 * @param 	*x_current: nmpc current drone state array
 * @param 	*x_ref: nmpc reference array
 *
 * @brief   Compare if drone reached required position
 *
 * @return  none
 */
uint8_t command_position_reached(s_flight_command *cmd, double x_current[NMPC_NX_SIZE], double x_ref[NMPC_NX_SIZE])
{
    double dx = x_current[0] - x_ref[0];
    double dy = x_current[1] - x_ref[1];
    double dz = x_current[2] - x_ref[2];

    // fabs() - floating point absolute value

    // Commands with similar goals use the same limits
    switch (cmd->command)
    {
        case COMM_TAKE_OFF:
        case COMM_HEIGHT:
        //case COMM_HOVER:
            return fabs(dz) < ALT_TOL_M;	// Check if position in z-axis is reached

        case COMM_FORWARD:
        case COMM_BACKWARD:
            return fabs(dx) < POS_TOL_M;	// Check if position in x-axis is reached

        case COMM_LEFT:
        case COMM_RIGHT:
            return fabs(dy) < POS_TOL_M;	// Check if position in y-axis is reached

        case COMM_RETURN_HOME:
            return sqrt((dx * dx) + (dy * dy) + (dz * dz)) < POS_TOL_M;	// Check if diagonal distance is reached (test)

        case COMM_LAND:
            return x_current[2] < ALT_TOL_M; // Check if position in z-axis is reached

        default:
            return 0;
    }
}
