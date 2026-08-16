/*****************************************************************
 * File Name          : autopilot.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/04/05
   Description        : Path commander and planning for drone
*****************************************************************/

#ifndef INC_AUTOPILOT_H_
#define INC_AUTOPILOT_H_


/*###########################################################################################################################################################*/
/* Includes */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "nmpc_config.h"



/*###########################################################################################################################################################*/
/* Defines */

// Tolerance for positions
#define POS_TOL_M       	 			0.05   // 5 cm
#define ALT_TOL_M       	 			0.05   // 5 cm

// Outer altitude loop: altitude error [m] -> vertical velocity reference [m/s]
#define Z_OUTER_KP                  	0.45f	// 0.35, 0.45, 0.55
#define Z_DEADBAND_M                	0.05f
#define VZ_REF_MIN_M_S              	-0.40f	// -0.35, -0.25. -30.0
#define VZ_REF_LAND_MIN_M_S         	-0.45f
#define VZ_REF_LAND_NEAR_GROUND_MIN_M_S -0.18f
#define VZ_REF_MAX_M_S              	0.25f // 0.20f, 0.25
#define VZ_REF_SLEW_M_S2            	1.0f	// 0.6, 0.4, 0.6
#define VZ_REF_LAND_NOW_SLEW_M_S2		1.5f
#define AUTOPILOT_UPDATE_DT_S			0.02f // Must be based on NMPC loop timer


// EDF operating limits selected by the active flight command
#define EDF_MIN_FLIGHT_PERCENT      	60.0
#define EDF_MIN_LAND_PERCENT        	65.0
#define EDF_MAX_PERCENT             	90.0
#define EDF_MAX_LAND_NEAR_GROUND_PERCENT 85.0
#define LAND_NEAR_GROUND_ALT_M        0.35

// Slow adaptation of nominal hover EDF command for battery/temperature changes
#define U_HOVER_TRIM_INITIAL_PERCENT   	85.0
#define U_HOVER_TRIM_MIN_PERCENT      	70.0
#define U_HOVER_TRIM_MAX_PERCENT       	86.0
#define U_HOVER_TRIM_ALPHA              0.005
#define U_HOVER_TRIM_KI_Z               2.0 // %/(m*s)
#define U_HOVER_TRIM_KI_VZ              6.0 // %/((m/s)*s)
#define U_HOVER_ADAPT_Z_REF_MIN_M       0.5
#define U_HOVER_ADAPT_Z_ERR_MAX_M       0.05
#define U_HOVER_ADAPT_VZ_MAX_M_S        0.05
#define U_HOVER_ADAPT_U0_MIN_PERCENT    72.0
#define U_HOVER_ADAPT_U0_MAX_PERCENT    88.0
#define U_HOVER_ADAPT_VERTICAL_FACTOR_MIN 0.85 // Reject attitude beyond approximately 32 deg tilt

/*###########################################################################################################################################################*/
/* Structs & enums */

// Commands
typedef enum {
	COMM_TAKE_OFF,
	COMM_LAND,
	COMM_HEIGHT,
	COMM_FORWARD,
	COMM_BACKWARD,
	COMM_LEFT,
	COMM_RIGHT,
	COMM_ROTATE_CW,
	COMM_ROTATE_CCW,
	COMM_WAIT,
	COMM_HOVER,
	COMM_FOLLOW,
	COMM_ACTION,
	COMM_RETURN_HOME

} e_flight_command;


// Follow mode
typedef enum {
	FOLLOW_MODE_GPS,
	FOLLOW_MODE_LINE,
	FOLLOW_MODE_ARUCO

} e_follow_mode;


// Action ID
typedef enum {
	ACTION_TAKE_PHOTO,
	ACTION_VIDEO_START,
	ACTION_VIDEO_STOP,
	ACTION_ACTUATOR,
	ACTION_LED_ON,
	ACTION_LED_OFF

} e_action_id;


// Command data - take off
typedef struct {
    uint16_t height_cm;

} s_data_takeoff;


// Command data - land
typedef struct {
    uint16_t delay_s;

} s_data_land;


// Command data - height (higher or lower)
typedef struct {
    uint16_t height_cm;
    uint16_t speed_cm_s;

} s_data_height;


// Command data - forward
typedef struct {
    uint16_t target_distance_cm;
    uint16_t speed_cm_s;

} s_data_forward;


// Command data - backward
typedef struct {
    uint16_t target_distance_cm;
    uint16_t speed_cm_s;

} s_data_backward;


// Command data - left
typedef struct {
    uint16_t target_distance_cm;
    uint16_t speed_cm_s;

} s_data_left;


// Command data - right
typedef struct {
    uint16_t target_distance_cm;
    uint16_t speed_cm_s;

} s_data_right;


// Command data - rotate clockwise
typedef struct {
    uint16_t angle_deg;
    uint16_t speed_deg_s;

} s_data_rotate_cw;


// Command data - rotate counter clockwise
typedef struct {
    uint16_t angle_deg;
    uint16_t speed_deg_s;

} s_data_rotate_ccw;


// Command data - wait
typedef struct {
    uint16_t time_s;

} s_data_wait;


// Command data - hover
typedef struct {
	uint16_t height_cm;
    uint16_t time_s;

} s_data_hover;


// Command data - follow (TBD)
typedef struct {
    e_follow_mode  	follow_mode;
    uint16_t 		distance_cm;
    uint16_t 		timeout_s;

} s_data_follow;


// Command data - action (TBD)
typedef struct {
    e_action_id action_id;
    uint16_t 	parameter1;
    uint16_t 	parameter2;

} s_data_action;


// Command data - return home
typedef struct {
    uint16_t height_cm;
    uint16_t speed_cm_s;

} s_data_return_home;


// Single step in path definition
typedef struct {
    e_flight_command 	command;	// "Name" of command
    uint8_t				command_id;	// Command index (from 0 to 100)

    // Based on command, only its struct is used
    union {
    	s_data_takeoff       takeoff;
		s_data_land          land;
		s_data_height        height;

		s_data_forward       forward;
		s_data_backward      backward;
		s_data_left          left;
		s_data_right         right;

		s_data_rotate_cw     rotate_cw;
		s_data_rotate_ccw    rotate_ccw;

		s_data_wait          wait;
		s_data_hover         hover;

		s_data_follow        follow;
		s_data_action        action;

		s_data_return_home   return_home;

    };

} s_flight_command;


// Flight path data
typedef struct {
	s_flight_command		flight_path[100];			// Complete flight path
	uint8_t 				command_index;				// Current active command from flight - counter
	uint8_t					command_cnt;				// Counter for amount of commands

	float					command_time_s;				// Time elapsed since current command was started
	uint16_t				command_timeout_s;			// Timeout of each command in flight path

	double					altitude_ref_m;				// Active altitude target retained between commands [m]
	double					vz_ref_m_s;					// Slew-limited vertical-speed reference [m/s]
	double					vz_ref_target_m_s;			// Clamped reference before slew limiting [m/s]
	double					edf_min_percent;			// Active minimum EDF command: flight or landing [%]
	double					edf_max_percent;			// Active maximum EDF command [%]
	double					u_hover_trim_percent;		// Slowly adapted nominal hover EDF command [%]
	uint32_t 				flight_start_time;			// Start time of flight


} s_path;


/*###########################################################################################################################################################*/
/* Functions */

uint8_t execute_flight_command(s_path *path_data, double x_ref[NMPC_NX_SIZE], double z_current_m);
uint8_t command_position_reached(s_flight_command *cmd, double x_current[NMPC_NX_SIZE], double x_ref[NMPC_NX_SIZE]);
void land_now(s_path *path_data, double x_ref[NMPC_NX_SIZE], double current_x_m, double current_y_m, double z_current_m);
void update_hover_trim(s_path *path_data, const double x_ref[NMPC_NX_SIZE],
		double z_current_m, double vz_current_m_s, double u0_opt_percent,
		double thrust_vertical_factor);

#endif /* INC_AUTOPILOT_H_ */
