/*****************************************************************
 * File Name          : DroneData.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2023/05/17
 * Description        : All drone data and data logic for telemetry
*****************************************************************/

#ifndef INC_DRONE_DATA_H_
#define INC_DRONE_DATA_H_

#include <stdint.h>
#include <math.h>
#include "data_transcode.h"
#include "GNSS.h"
#include "autopilot.h"


/*###########################################################################################################################################################*/
/* Defines */

// Calibration
//#define CALIBRATION				// Uncomment to enable gyro calibration mode (set 1/0 to output value or not)
#define IDENTIFICATION
#define TEST_MOMENTS				// Uncomment to enable serial print over GPS connector - testing of fin moments

#define TUNE_KALMAN
#ifdef TUNE_KALMAN
	#define CAL_GYRO_X		1
	#define CAL_GYRO_Y		1
	#define CAL_GYRO_Z		1
	#define CAL_ACCEL_X		1
	#define CAL_ACCEL_Y		1
	#define CAL_ACCEL_Z		1
#endif

#define CAL_PITCH			0
#define CAL_ROLL			0
#define CAL_LIDAR			0

#define GYRO_KALMAN					// Use Kalman filter - Comment this: use complementary filter

#define LOG_ENABLE					// Enable logging of telemetry data

#define ANGLE_SCALE			100.0f	// Factor for angle scaling of angle

#define RADIO_NUM			2 		// Set number of radios mounted on board
#define CONN_TIMEOUT_SEC	3		// Amount off seconds that triggers timeout/lost connection
//#define CONN_STEPS_2				// If using only OPT_PAIR_START signal for pairing comment this
#define STREAM_EN_TIME		3		// Time in seconds over which stream mode is enabled

// Select GPS decoding (comment/uncomment for enable/disable)
//#define USE_GPS
#define USE_GPS_GGA
//#define USE_GPS_GLL
#define USE_GPS_GSA
#define USE_GPS_GSV
#define USE_GPS_RMC
#define USE_GPS_VTG

#define ALTITUDE_M			98		// Height where drone will take off
#define DECLINATION_DEG		4.34f	// Deskle declination = +4.28deg (+4.34deg) (source: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?)

#define MAIN_BOARD_V		3.27	// Main board voltage
#define R1_MAIN_BAT			100000	// Voltage divider resistor R1 - main battery
#define R2_MAIN_BAT		 	10000	// Voltage divider resistor R2 - main battery
#define R1_EDF_BAT			100000	// Voltage divider resistor R1 - EDF battery
#define R2_EDF_BAT		 	10000	// Voltage divider resistor R2 - EDF battery
#define R1_5V_BUCK			100000	// Voltage divider resistor R1 - 5V buck converter
#define R2_5V_BUCK		 	150000	// Voltage divider resistor R2 - 5V buck converter
#define R1_7V2_BUCK			100000	// Voltage divider resistor R1 - 7.2V buck converter
#define R2_7V2_BUCK		 	75000	// Voltage divider resistor R2 - 7.2V buck converter
#define DISABLE				0		// Helper define for enable/disable buck
#define ENABLE				1		// Helper define for enable/disable buck



#define TOF_OFFSET			121		// Height of TOF sensor of the ground when device is on the ground

#define EDF_RAMP_UP_EN				// Enable EDF slow ramp-up procedure


/*###########################################################################################################################################################*/
/* Structs and enums */

// Drone: status
typedef enum {
	STATUS_STARTUP,						// Status when power on
	STATUS_IDLE,						// Idle
	STATUS_ERROR,						// Error
	STATUS_ARM,							// State before flight
	STATUS_FLY,							// Flying
	STATUS_FLY_OVER,					// After slight
	STATUS_CALIB						// When calibrating gyro, not for flight
} e_drone_status;


// Drone flight status
typedef enum {
	STATUS_FLIGHT_GROUND,				// Drone still on the ground
	STATUS_FLIGHT_TAKEOFF,				// Take-off event
	STATUS_FLIGHT_FLYING,				// Flying
	STATUS_FLIGHT_LANDING				// Landing event
} e_flight_status;


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
	volatile e_connection_status conn_status;	// Status of connection with ground station
	volatile uint8_t	flag_connection_lost;	// Flag indicating lost of connection
	volatile uint8_t	flag_connection_begin;	// Flag for connecting procedure (default-0, START-2, STATUS-1)
	volatile uint8_t	flag_stream_data;		// Flag for indicating data stream - different radio settings

	uint32_t 			packet_tx_cnt;			// Counter of transmitted packets
	uint32_t 			packet_fail_cnt;		// Counter of transmitted packets (max_rxs)
	uint8_t				connection_timeout;		// Seconds counter for connection timeout

	volatile uint8_t    flag_new_rf_rx_data;    // Flag indicating a new data has arrived
	volatile uint8_t    flag_new_rf_tx_data;    // Flag indicating a new data is ready to send
	volatile uint8_t	flag_telemetry_send;	// Flag indicating telemetry send procedure

    // PC -> Drone and Drone -> PC //
    uint8_t     		data_buffer[64];        // Buffer for saving data

} s_radios;


// Drone: position data
typedef struct {
	float 				Pitch;
	float 				Roll;
	float 				Yaw;

	float				position_x;	// Drone body position in space - x axis
	float				position_y;	// Drone body position in space - y axis
	float				position_z;	// Drone body position in space - z axis

	uint16_t			gyroTemp;	// Temperature of IMU
	float				accel_x;	// Raw data - acceleration x
	float				accel_y;	// Raw data - acceleration y
	float				accel_z;	// Raw data - acceleration z
	float				gyro_x;		// Raw data - gyro x
	float				gyro_y;		// Raw data - gyro y
	float				gyro_z;		// Raw data - gyro z
	float				quaternion[4];	// Quaternion from Euler angles: Body to World system

	/* Height of drone (when on ground the height is 0, offset on sensor set to 130mm) */
	uint16_t 			height_TOF_mm;
	uint16_t			height_baro_m;

	// Compass
    float               x_gauss;	// X axis in Gauss
    float               y_gauss;	// Y axis in Gauss
    float               z_gauss;	// Z axis in Gauss
    float				heading_deg;// Heading direction in degrees

} s_position;


// Drone: environment and vehicle status
typedef struct {
	uint16_t			bat_main_v[10];			// Main battery voltage averaging array
	uint16_t			bat_edf_v[10];			// EDF battery voltage averaging array
	uint16_t			buck_5V_v[10];			// 5V buck voltage averaging array
	uint16_t			buck_7V2_v[10];			// 7.2V buck voltage averaging array
	uint16_t 			battery_main_voltage;	// Voltage of main board battery
	uint16_t 			battery_edf_voltage;	// Voltage of EDF fan battery
	uint16_t 			buck_5v_voltage;		// Voltage of 5V buck converter
	uint16_t 			buck_7v2_voltage;		// Voltage of 7.2V buck converter
	uint8_t				buck_5v_enable;			// Enable flag: buck 5V
	uint8_t				buck_7v2_enable;		// Enable flag: buck 7.2V

	uint16_t			temperature;			// Temperature
	uint8_t				humidity;				// Humidity
	uint32_t			pressure;				// Pressure
	uint16_t			take_off_alt_m;			// Take off altitude in meters
	uint8_t				edf_throttle;			// EDF throttle level

	uint8_t				servo_enable;			// Flag for enabling servo motors
	uint8_t				edf_enable;				// Flag for enabling edf motor
	uint8_t				NMPC_enable;			// Flag for enabling NMPC regulator
	uint8_t				nmpc_solver_fail_cnt;	// NMPC solver fails counter

}s_data;


// UART buffers and flags
typedef struct {
	uint8_t				buffer_temp[1];			// Small buffer for received byte //2
    uint8_t     		buffer_UART[64];        // Buffer for saving USB data
    uint8_t     		flag_new_uart_rx_data;  // Flag indicating a new data has arrived (packet is not complete)
    uint8_t     		flag_new_uart_tx_data;  // Flag indicating a new data is ready to send
    uint8_t     		flag_USB_RX_new;        // Flag for new complete USB packet (PC -> link) - start decode

    uint8_t				flag_logging_active;	// Flag for logging in progress
    uint8_t				flag_log_available;		// Flag for indicating log available in flash

    const char		    *log_file;				// Name of log file
    uint8_t				flag_log_dump;			// Flag indication complete log dump over UART
    uint8_t				flag_log_remove;		// Flag for deleting log file

    s_packets			packets;				// UART and RF packets

} s_uart_buffers;


// Actuators values
typedef struct {
	uint8_t 			edf_percent;			// Percents of power on EDF
    float 				servo_xp;				// Servo angle - X+
    float 				servo_xn;				// Servo angle - X-
    float 				servo_yp;				// Servo angle - Y+
    float 				servo_yn;				// Servo angle - Y-

    uint8_t				rampUpEnable;			// Slow ramp-up enable flag
    uint8_t				rampUpDone;				// Slow ramp-up finished flag
    uint8_t				rampUpTarget;			// Slow ramp-up percent goal
    uint32_t			rampUpTime;				// Slow ramp-up time goal
    uint8_t				rampUpStep;				// Slow ramp-up power step

} s_actuators;


// Drone: Error codes
typedef struct {
	uint8_t				err_main_bat;			// Error flag - main battery voltage
	uint8_t				err_edf_bat;			// Error flag - edf battery voltage
	uint8_t				err_buck_5v;			// Error flag - buck 5V voltage
	uint8_t				err_buck_7v2;			// Error flag - buck 7.2V voltage
	uint8_t				err_bme280;				// Error flag - bme280 sensor error
	uint8_t				err_mpu6050;			// Error flag - mpu6050 sensor error
	uint8_t				err_vl53l1x;			// Error flag - vl53l1x sensor error
	uint8_t				err_radio1;				// Error flag - radio1 / nrf24l01 radio error
	uint8_t				err_radio2;				// Error flag - radio2 / nrf24l01 radio error
	uint8_t				err_hmc5883l;			// Error flag - hmc5883l sensor error
	uint8_t				err_pmw3901;			// Error flag - pmw3901 sensorerror

} s_errors;


// Drone: identification tests
typedef struct {
	uint8_t				flag_test_identification;	// Identification in progress
	uint8_t				flag_test_edf;				// Test EDF thrust
	uint8_t				flag_test_servo;			// Test servo motors
	uint8_t				flag_test_moment;			// Test drone moments

} s_identification;


// Main drone struct with all data
typedef struct {
	/* Base data */
	volatile e_drone_status  DroneStatus;	 	// Status of drone
	volatile e_flight_status flight_status;		// Status of drone when flying
	s_date_time 		date_time;				// Current time - GPS
	s_errors			error_code;				// Error codes of drone
	s_uart_buffers		uart_buffer;			// Buffer for uart data
	s_identification	identifications;		// Identifivation test flags

	/* Radio */
	s_radios			radio_data;				// All data for radio communication

	/* Data */
	s_position			position;				// Drone position (angled, speed, etc.)
	s_GNSS				gps_data;				// GPS data
	s_data				data;					// Other drone data (batterys, temp, hum, press...)
	s_path				flight_path;			// Flight path data
	s_actuators			actuators;				// Drone actuators value

} s_drone_data;



/*###########################################################################################################################################################*/
/* Functions */

uint8_t RF_packet_decode(s_packets *packets, s_drone_data *drone_data);
void packet_create_telemetry(s_packets *packets, s_drone_data *drone_data);
void packet_create_uart_data(s_packets *packets, s_drone_data *drone_data);

#endif /* INC_DRONE_DATA_H_ */
