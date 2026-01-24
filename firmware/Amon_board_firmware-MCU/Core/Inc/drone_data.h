/*****************************************************************
 * File Name          : DroneData.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2023/05/17
 * Description        : All drone data and variables
*****************************************************************/

#ifndef INC_DRONE_DATA_H_
#define INC_DRONE_DATA_H_

#include <stdint.h>
#include "data_transcode.h"
//#include "GNSS.h"


/*###########################################################################################################################################################*/
/* Defines */

//#define CALIBRATION				// Uncomment to enable gyro calibration mode
#define CAL_GYRO_X			0
#define CAL_GYRO_Y			0
#define CAL_GYRO_Z			0
#define CAL_ACCEL_X			0
#define CAL_ACCEL_Y			0
#define CAL_ACCEL_Z			0
#define CAL_PITCH			0
#define CAL_ROLL			0
#define CAL_LIDAR			0

#define RADIO_NUM			2 	// Set number of radios mounted on board
#define CONN_TIMEOUT_SEC	3	// Amount off seconds that triggers timeout/lost connection

// Select GPS decoding (comment/uncomment for enable/disable)
#define USE_GPS
#define USE_GPS_GGA
#define USE_GPS_GLL
#define USE_GPS_GSA
#define USE_GPS_GSV
#define USE_GPS_RMC
#define USE_GPS_VTG



/*###########################################################################################################################################################*/
/* Structs and enums */

//typedef enum {
//	SERVO_XP = 1,
//	SERVO_XN = 2,
//	SERVO_YP = 3,
//	SERVO_YN = 4,
//	PWM_EDF = 5
//}e_servos;


// Drone: status
typedef enum {
	STATUS_STARTUP,						// Status when power on
	STATUS_IDLE,						// Idle
	STATUS_ERROR,						// Error
	STATUS_ARM,							// State before flight
	STATUS_FLY,							// Flying
	STATUS_FLY_OVER,					// After slight
	STATUS_GYRO_CALIB					// When calibrating gyro, not for flight
} e_drone_status;


// Drone: date-time
typedef struct {
	uint8_t		day;
	uint8_t		month;
	uint16_t	year;
	uint8_t 	hour;
	uint8_t 	minutes;
	uint8_t 	seconds;
} s_date_time;


// Drone: radios
typedef enum {
	CONN_STATUS_CONNECTED,
	CONN_STATUS_DISCONNECTED
} e_connection_status;

typedef struct {
	e_connection_status conn_status;			// Status of connection with ground station
	uint8_t				flag_connection_lost;	// Flag indicating lost of connection
	uint8_t				flag_connection_begin;	// Flag for connecting procedure (default-0, START-2, STATUS-1)
	uint8_t				flag_stream_data;		// Flag for indicating data stream - different radio settings

	uint32_t 			packet_tx_cnt;			// Counter of transmitted packets
	uint32_t 			packet_fail_cnt;		// Counter of transmitted packets (max_rxs)
	uint8_t				connection_timeout;		// Seconds counter for connection timeout

    uint8_t     		flag_new_rf_rx_data;    // Flag indicating a new data has arrived
    uint8_t     		flag_new_rf_tx_data;    // Flag indicating a new data is ready to send
    uint8_t				flag_telemetry_send;	// Flag indicating telemetry send procedure

    // PC -> Drone and Drone -> PC //
    uint8_t     		data_buffer[64];        // Buffer for saving data

} s_radios;


// Drone: position data
typedef struct {
	float 				Pitch;
	float 				PitchOld;

	float 				Roll;
	float 				RollOld;

	float 				Yaw;
	float 				YawOld;

	/* Data before flight to initialize orientation */
	float 				PitchMean;
	float 				RollMean;

	/* Height of drone (when on ground the height is 0, offset on sensor set to 130mm) */
	uint16_t Height;

} s_position;


// Drone: Error codes
typedef struct {
	uint8_t				err_main_bat;			// Error flag - main battery voltage
	uint8_t				err_edf_bat;			// Error flag - edf battery voltage
	uint8_t				err_bme280;				// Error flag - bme280 sensor error
	uint8_t				err_mpu6050;			// Error flag - mpu6050 sensor error
	uint8_t				err_vl53l1x;			// Error flag - vl53l1x sensor error
	uint8_t				err_radio1;				// Error flag - radio1 / nrf24l01 radio error
	uint8_t				err_radio2;				// Error flag - radio2 / nrf24l01 radio error

} s_errors;


typedef struct {

	/* Base data */
	e_drone_status 		DroneStatus;	 		// Status of drone
	s_date_time 		date_time;				// Current time - GPS
	uint16_t 			battery_main_voltage;	// Voltage of main board battery
	uint16_t 			battery_edf_voltage;	// Voltage of EDF fan battery
	s_errors			error_code;				// Error codes of drone

	/* Radio */
	s_radios			radio_data;				// All data for radio communication

	/* Orientation of drone */
	s_position			position;				// Drone position (angled, speed, etc.)

	/* Data from GPS */
	//s_GPS				gps_data;


} s_drone_data;



/*###########################################################################################################################################################*/
/* Functions */

uint8_t RF_packet_decode(s_packets *packets, s_drone_data *drone_data);
void packet_create_telemetry(s_packets *packets, s_drone_data *drone_data);


#endif /* INC_DRONE_DATA_H_ */
