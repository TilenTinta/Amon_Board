/*****************************************************************
 * File Name          : nmpc_controler.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/08/04
 * Description        : NMPC controller wrapper - no acados code
*****************************************************************/

#include <nmpc_controler.h>
#include "stm32h7xx_hal.h"

#define NMPC_MODEL_MASS_KG  2.485
#define NMPC_GRAVITY_M_S2   9.81

/*###########################################################################################################################################################*/
/*Local fFunctions */


/*********************************************************************
 * @fn      EDF_LUT_Thrust_N
 *
 * @param   edf_percent: current hover trim edf power
 *
 * @brief   calculate current thrust from LUT with piece-wise logic
 *
 * @return  None
 */
static double EDF_LUT_Thrust_N(double edf_percent)
{
	/* MUST MATCH LUT USED IN ACADOS MODEL */
	static const double edf_points[] = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 75.0, 80.0, 85.0, 90.0, 100.0};
	static const double thrust_points[] = {
		0.0,
		1.4623894660200982,
		4.2485368817013320,
		7.0457390494971124,
		9.6132801575173321,
		12.042451609558308,
		14.740834272959603,
		18.395945471821904,
		20.824175499723022,
		22.838908040823828,
		24.758844578012344,
		26.752962991109627,
		27.0
	};
	const unsigned int point_count = sizeof(edf_points) / sizeof(edf_points[0]);

	if (edf_percent <= edf_points[0]) return thrust_points[0];
	if (edf_percent >= edf_points[point_count - 1]) return thrust_points[point_count - 1];

	for (unsigned int i = 0; i < point_count - 1; i++)
	{
		if (edf_percent <= edf_points[i + 1])
		{
			double fraction = (edf_percent - edf_points[i]) / (edf_points[i + 1] - edf_points[i]);
			return thrust_points[i] + fraction * (thrust_points[i + 1] - thrust_points[i]);
		}
	}

	return thrust_points[point_count - 1];
}

/*###########################################################################################################################################################*/
/* Functions */

/*********************************************************************
 * @fn      NMPC_ControllerUpdate
 *
 * @param   *nmpc: pointer to nmpc struct
 * @param   *drone: pointer to drone data struct
 *
 * @brief   Save current solver values for
 * 			next iteration of optimization
 *
 * @return  None
 */
void NMPC_ControllerUpdate(s_NMPC *nmpc, s_drone_data *drone)
{
	// Refresh all values in state vector //
	static double x_current[NMPC_NX] = {0}; 	// Reset everything

	// --- Position [0:3] ---
	x_current[0] = drone->position.position_x;              // [m]
	x_current[1] = drone->position.position_y;              // [m]
	x_current[2] = drone->position.height_TOF_m_filtered;  	// pos z [m]

	// --- Velocity [3:6] ---
	x_current[3] = drone->position.velocity_x;  // vx [m/s] - optical flow, world frame
	x_current[4] = drone->position.velocity_y;  // vy [m/s] - optical flow, world frame
	x_current[5] = drone->position.velocity_z;			// vz [m/s] - kalman

	// --- Quaternion [6:10] ---
	x_current[6]  = drone->position.quaternion[0];		// qw
	x_current[7]  = drone->position.quaternion[1];		// qx
	x_current[8]  = drone->position.quaternion[2];		// qy
	x_current[9]  = drone->position.quaternion[3];		// qz

	// --- Angular rates [10:13] ---
	// gyro values must be in rad/s
	x_current[10] = drone->position.Pitch_rate;
	x_current[11] = drone->position.Roll_rate;
	x_current[12] = drone->position.Yaw_rate;

	// --------------------------------------------------------------------------------------------------------
	// Set NMPC controller reference values //
	static double x_ref[NMPC_NX] = {0};		// Reset everything

	if (drone->data.flag_land_now)
	{
	    // Cancel current flight path execution
		drone->flight_status = STATUS_FLIGHT_LANDING;
		drone->regulators.nmpc_set_new_ref = 1;
		land_now(&drone->flight_path, x_ref, drone->position.position_x, drone->position.position_y, x_current[2]);
	}
	else
	{
		execute_flight_command(&drone->flight_path, x_ref, x_current[2]);
	}

	static double u_ref[NMPC_NU] = {
	    U_HOVER_TRIM_INITIAL_PERCENT, // adapted below for battery/temperature changes
	    0.0,   // servo XP neutral or trim/offset
	    0.0,   // servo XN neutral or trim/offset
	    0.0,   // servo YP neutral or trim/offset
	    0.0    // servo YN neutral or trim/offset
	};

#ifdef USE_HOVER_TRIM
	u_ref[0] = drone->flight_path.u_hover_trim_percent;
#endif

	// Correct model LUT for EDF percentage currently estimated to hover (use value 1 to eliminate this function)
	double eta_T = 1.0;
#ifdef USE_HOVER_TRIM
	double hover_lut_thrust_N = EDF_LUT_Thrust_N(drone->flight_path.u_hover_trim_percent);
	if (hover_lut_thrust_N > 0.0)
	{
		eta_T = (NMPC_MODEL_MASS_KG * NMPC_GRAVITY_M_S2) / hover_lut_thrust_N;
	}
#endif
	NMPC_SetThrustScale(nmpc, eta_T);

	// Last commands actually sent to the actuators. NMPC uses these as centers of the next stage-0 slew-rate bounds
	const double u_applied[NMPC_NU] = {
		(double)drone->actuators.edf_percent,
		(double)drone->actuators.servo_xp,
		(double)drone->actuators.servo_xn,
		(double)drone->actuators.servo_yp,
		(double)drone->actuators.servo_yn
	};

#if NMPC_MODEL_SELECTION == NMPC_MODEL_FIRST_ORDER
	// --- First order actuator model [13:18] ---
	// Estimate physical actuator states because no thrust or servo feedback is available
	const double commanded_thrust_N = nmpc->eta_T * EDF_LUT_Thrust_N(u_applied[0]);
	NMPC_UpdateActuatorEstimator(nmpc, u_applied, commanded_thrust_N, NMPC_DT_S);

	x_current[NMPC_X_THRUST_N] = nmpc->estimated_thrust_N;
	x_current[NMPC_X_DELTA_1_RAD] = nmpc->estimated_servo_rad[0];
	x_current[NMPC_X_DELTA_2_RAD] = nmpc->estimated_servo_rad[1];
	x_current[NMPC_X_DELTA_3_RAD] = nmpc->estimated_servo_rad[2];
	x_current[NMPC_X_DELTA_4_RAD] = nmpc->estimated_servo_rad[3];

	// Extend the flight-path reference with nominal physical actuator states.
	x_ref[NMPC_X_THRUST_N] = nmpc->eta_T * EDF_LUT_Thrust_N(u_ref[0]);
	x_ref[NMPC_X_DELTA_1_RAD] = u_ref[1] * NMPC_DEG_TO_RAD;
	x_ref[NMPC_X_DELTA_2_RAD] = u_ref[2] * NMPC_DEG_TO_RAD;
	x_ref[NMPC_X_DELTA_3_RAD] = u_ref[3] * NMPC_DEG_TO_RAD;
	x_ref[NMPC_X_DELTA_4_RAD] = u_ref[4] * NMPC_DEG_TO_RAD;
#endif

	// Set NMPC init value
	NMPC_SetState(nmpc, x_current);
	NMPC_SetEdfOperatingLimits(nmpc, drone->flight_path.edf_min_percent, drone->flight_path.edf_max_percent);
	NMPC_SetAppliedControl(nmpc, u_applied);

	// Needs to be set each time for height purposes
	drone->regulators.nmpc_set_new_ref = 0;
	NMPC_SetReference(nmpc, x_ref, u_ref);

	// Reference/state values actually supplied to the NMPC this cycle
	drone->regulators.nmpc_z_ref = (float)x_ref[2];
	drone->regulators.nmpc_z_current = (float)x_current[2];
	drone->regulators.nmpc_z_error = (float)(x_ref[2] - x_current[2]);
	drone->regulators.nmpc_vz_ref_target = (float)drone->flight_path.vz_ref_target_m_s;
	drone->regulators.nmpc_vz_ref_slew = (float)x_ref[5];
	drone->regulators.nmpc_vz_current = (float)x_current[5];
	drone->regulators.nmpc_u0_ref = (float)u_ref[0];



	// --------------------------------------------------------------------------------------------------------
	// Run NMPC control logic //
	uint32_t c0 = DWT->CYCCNT;
	int ret = NMPC_Solve(nmpc);	// Solve optimization problem
	uint32_t cycles = DWT->CYCCNT - c0;
	float us = (float)cycles / (SystemCoreClock / 1000000.0f);  // µs

	// Running sum - averaging
	nmpc->nmpc_solve_time_sum -= nmpc->nmpc_solve_time_arr[nmpc->nmpc_solve_time_cnt];
	nmpc->nmpc_solve_time_arr[nmpc->nmpc_solve_time_cnt] = us;
	nmpc->nmpc_solve_time_sum += us;
	nmpc->nmpc_solve_time_cnt++;
	if (nmpc->nmpc_solve_time_cnt >= 10) nmpc->nmpc_solve_time_cnt = 0;
	if (nmpc->nmpc_solve_time_samples < 10) nmpc->nmpc_solve_time_samples++;

	nmpc->nmpc_solve_time = nmpc->nmpc_solve_time_sum / nmpc->nmpc_solve_time_samples;

	// Solver statistic
	drone->regulators.nmpc_solver_time = nmpc->nmpc_solve_time;
	drone->regulators.nmpc_solve_status = ret;
	drone->regulators.nmpc_last_qp_iter = nmpc->nmpc_last_qp_iter;
	drone->regulators.nmpc_last_qp_status = nmpc->nmpc_last_qp_status;
	drone->regulators.nmpc_acados_status = nmpc->nmpc_acados_status;
	drone->regulators.nmpc_time_tot_ms = (float)(nmpc->nmpc_time_tot_s * 1000.0);
	drone->regulators.nmpc_time_qp_ms = (float)(nmpc->nmpc_time_qp_s * 1000.0);
	drone->regulators.nmpc_sqp_iter = nmpc->nmpc_sqp_iter;
	drone->regulators.nmpc_u0_opt = (float)nmpc->u_opt[0];
	drone->regulators.nmpc_u0_lbu = (float)nmpc->u0_lbu;
	drone->regulators.nmpc_u0_ubu = (float)nmpc->u0_ubu;
	drone->regulators.nmpc_model_selection = nmpc->model_selection;
	drone->regulators.nmpc_model_nx = nmpc->model_nx;
	drone->regulators.nmpc_model_horizon = nmpc->model_horizon;
	drone->regulators.nmpc_actuator_estimator_valid = nmpc->actuator_estimator_valid;
	drone->regulators.nmpc_estimated_thrust_N = (float)nmpc->estimated_thrust_N;
	drone->regulators.nmpc_estimator_edf_alpha = (float)nmpc->estimator_edf_alpha;
	drone->regulators.nmpc_estimator_servo_alpha = (float)nmpc->estimator_servo_alpha;
	for (int i = 0; i < 4; i++)
	{
		drone->regulators.nmpc_estimated_servo_rad[i] = (float)nmpc->estimated_servo_rad[i];
	}

	// --- Solver result monitoring ---
	if (ret == NMPC_OK)
	{
		drone->regulators.nmpc_solver_fail_cnt = 0;

	#ifdef USE_HOVER_TRIM
		// Body-Z projection in world-Z from quaternion: X = 1 - 2(qx^2 + qy^2).
		double thrust_vertical_factor = 1.0 - 2.0 * (x_current[7] * x_current[7] + x_current[8] * x_current[8]);
		if (thrust_vertical_factor > 1.0) thrust_vertical_factor = 1.0;
		if (thrust_vertical_factor < -1.0) thrust_vertical_factor = -1.0;

		// Learn battery/temperature-dependent hover EDF for the next NMPC cycle.
		update_hover_trim(&drone->flight_path, x_ref, x_current[2], x_current[5], nmpc->u_opt[0], thrust_vertical_factor);
	#endif
	}
	else
	{
		drone->regulators.nmpc_solver_fail_cnt++;

		if (drone->regulators.nmpc_solver_fail_cnt > 5)
		{
			// 5 solver fails back-to-back -> disarm or hold last good u
			drone->regulators.NMPC_enable = 1; // 0
			// optionally: keep last good actuator values / use PID + disarm
		}
	}



	// --------------------------------------------------------------------------------------------------------
	// Check solver status:
	// 	- OK: Limit actuator values and write them to actuators
	// 	- NOK: Keep last used values of actuators

	if (ret == NMPC_OK ) // || ret == NMPC_SOLVER_ERR
	{
#if NMPC_MODEL_SELECTION == NMPC_MODEL_INSTANT
	    static float servo_xp_old = 0.0f;
	    static float servo_xn_old = 0.0f;
	    static float servo_yp_old = 0.0f;
	    static float servo_yn_old = 0.0f;
#endif

		double u[NMPC_NU];
		NMPC_GetControl(nmpc, u);

		// Clamp before applying - defensive programming
		u[0] = fmax(nmpc->u0_operating_min, fmin(nmpc->u0_operating_max, u[0]));  // thrust [%]
		u[1] = fmax(NMPC_UX_MIN, fmin(NMPC_UX_MAX, u[1]));  // servo	[-45..45]
		u[2] = fmax(NMPC_UX_MIN, fmin(NMPC_UX_MAX, u[2]));
		u[3] = fmax(NMPC_UX_MIN, fmin(NMPC_UX_MAX, u[3]));
		u[4] = fmax(NMPC_UX_MIN, fmin(NMPC_UX_MAX, u[4]));

		// Servo slew-rate limitation ("jumpy and spiny servos") - sort of real servo feedback
#if NMPC_MODEL_SELECTION == NMPC_MODEL_INSTANT
		float servo_xp_cmd = SlewLimit((float)u[1], servo_xp_old, SERVO_MAX_STEP_DEG);
		float servo_xn_cmd = SlewLimit((float)u[2], servo_xn_old, SERVO_MAX_STEP_DEG);
		float servo_yp_cmd = SlewLimit((float)u[3], servo_yp_old, SERVO_MAX_STEP_DEG);
		float servo_yn_cmd = SlewLimit((float)u[4], servo_yn_old, SERVO_MAX_STEP_DEG);
#else
		float servo_xp_cmd = (float)u[1];
		float servo_xn_cmd = (float)u[2];
		float servo_yp_cmd = (float)u[3];
		float servo_yn_cmd = (float)u[4];
#endif

		//AmonDrone.actuators.edf_percent = (float)u[0];
		drone->actuators.edf_percent = (uint8_t)round((float)u[0]);
		drone->actuators.servo_xp = servo_xp_cmd;
		drone->actuators.servo_xn = servo_xn_cmd;
		drone->actuators.servo_yp = servo_yp_cmd;
		drone->actuators.servo_yn = servo_yn_cmd;

		// Save current values for next iteration check
#if NMPC_MODEL_SELECTION == NMPC_MODEL_INSTANT
		servo_xp_old = drone->actuators.servo_xp;
		servo_xn_old = drone->actuators.servo_xn;
		servo_yp_old = drone->actuators.servo_yp;
		servo_yn_old = drone->actuators.servo_yn;
#endif
	}

	if (nmpc->nmpc_limiter_enable == 0 && drone->actuators.edf_percent >= EDF_MIN_FLIGHT_PERCENT)
	{
		nmpc->nmpc_limiter_enable = 1;
	}

	// --------------------------------------------------------------------------------------------------------
	/* Command check - time or position */
	uint8_t time_done = 0;
	uint8_t position_done = 0;
	s_flight_command *cmd = NULL;
	s_flight_command land_now_cmd = {0};


	if (drone->data.flag_land_now)
	{
	    land_now_cmd.command = COMM_LAND;
	    cmd = &land_now_cmd;
	}
	else
	{
	    cmd = &drone->flight_path.flight_path[drone->flight_path.command_index];
	}

	// Check if current command reached its timeout value (if 0 then the command dont have timeout)
	if (drone->flight_path.command_timeout_s != 0)
	{
		if (drone->flight_path.command_time_s >= drone->flight_path.command_timeout_s)
		{
			time_done = 1;
		}
		else
		{
			drone->flight_path.command_time_s += TIM_50HZ_DT; // Current command elapsed time in seconds
		}
	}

	if (cmd != NULL)
	{
		position_done = command_position_reached(cmd, x_current, x_ref);
	}

	if (time_done || position_done)
	{
		drone->flight_path.command_time_s = 0; // Reset elapsed time command counter
		drone->flight_path.command_index++;
		drone->regulators.nmpc_set_new_ref = 1;
		if (drone->flight_path.command_index >= drone->flight_path.command_cnt)
		{
			//TODO: Check last command and land if not already
			drone->DroneStatus = STATUS_FLY_OVER;
		}
	}
}
