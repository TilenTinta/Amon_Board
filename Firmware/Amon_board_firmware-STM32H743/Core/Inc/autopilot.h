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


/*###########################################################################################################################################################*/
/* Defines */
#define NMPC_NX_SIZE		 23		// Number of states in model

// Speed for take-off and landing
#define TAKEOFF_SPEED_M_S    0.5
#define LAND_SPEED_M_S   	-0.3

// Tolerance for positions
#define POS_TOL_M       	 0.05   // 5 cm
#define ALT_TOL_M       	 0.05   // 5 cm
#define VEL_TOL_M_S     	 0.10


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

	float					command_time_s;				// Time elapsed since current command was started
	uint16_t				command_timeout_s;			// Timeout of each command in flight path
	uint32_t 				flight_start_time;			// Start time of flight


} s_path;


/*###########################################################################################################################################################*/
/* Functions */

uint8_t execute_flight_command(s_path *path_data, double x_ref[NMPC_NX_SIZE]);
uint8_t command_position_reached(s_flight_command *cmd, double x_current[NMPC_NX_SIZE], double x_ref[NMPC_NX_SIZE]);


#endif /* INC_AUTOPILOT_H_ */
