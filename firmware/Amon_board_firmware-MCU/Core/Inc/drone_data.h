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


/*###########################################################################################################################################################*/
/* Defines */

// Calibration
//#define CALIBRATION				// Uncomment to enable gyro calibration mode (set 1/0 to output value or not)

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
#define USE_GPS_GLL
#define USE_GPS_GSA
#define USE_GPS_GSV
#define USE_GPS_RMC
#define USE_GPS_VTG

#define ALTITUDE_M			98		// Height where drone will take off

#define MAIN_BOARD_V		3.3		// Main board voltage
#define R1_MAIN_BAT			100000	// Voltage divider resistor R1 - main battery
#define R2_MAIN_BAT		 	10000	// Voltage divider resistor R2 - main battery
#define R1_EDF_BAT			100000	// Voltage divider resistor R1 - EDF battery
#define R2_EDF_BAT		 	10000	// Voltage divider resistor R2 - EDF battery

#define TOF_OFFSET			121		// Height of TOF sensor of the ground when device is on the ground


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

	uint16_t			gyroTemp;	// Temperature of IMU
	float				accel_x;	// Raw data - acceleration x
	float				accel_y;	// Raw data - acceleration y
	float				accel_z;	// Raw data - acceleration z
	float				gyro_x;		// Raw data - gyro x
	float				gyro_y;		// Raw data - gyro y
	float				gyro_z;		// Raw data - gyro z

	/* Data before flight to initialize orientation */
//	float 				PitchMean;
//	float 				RollMean;

	/* Height of drone (when on ground the height is 0, offset on sensor set to 130mm) */
	uint16_t 			height_TOF_mm;
	uint16_t			height_baro_m;

	// Compass
	uint16_t			heading_deg;
} s_position;


// Drone: environment and vehicle status
typedef struct {
	uint16_t			bat_main_v[10];			// Main battery voltage averaging array
	uint16_t			bat_edf_v[10];			// EDF battery voltage averaging array
	uint16_t 			battery_main_voltage;	// Voltage of main board battery
	uint16_t 			battery_edf_voltage;	// Voltage of EDF fan battery
	uint16_t			temperature;			// Temperature
	uint8_t				humidity;				// Humidity
	uint32_t			pressure;				// Pressure
	uint16_t			take_off_alt_m;			// Take off altitude in meters
}s_data;


// UART buffers and flags
typedef struct {
	uint8_t				buffer_temp[2];			// Small buffer for received byte
    uint8_t     		buffer_UART[64];        // Buffer for saving USB data
    uint8_t     		flag_new_uart_rx_data;  // Flag indicating a new data has arrived (packet is not complete)
    uint8_t     		flag_new_uart_tx_data;  // Flag indicating a new data is ready to send
    uint8_t     		flag_USB_RX_new;        // Flag for new complete USB packet (PC -> link) - start decode

    uint8_t				flag_logging_active;	// Flag for logging in progress
    uint8_t				flag_log_available;		// Flag for indicating log available in flash

    char		     	log_file;				// Name of log file
    uint8_t				flag_log_dump;			// Flag indication complete log dump over UART
} s_uart_buffers;


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


// Main drone struct with all data
typedef struct {
	/* Base data */
	volatile e_drone_status DroneStatus;	 	// Status of drone
	volatile e_flight_status flight_status;		// Status of drone when flying
	s_date_time 		date_time;				// Current time - GPS
	s_errors			error_code;				// Error codes of drone
	s_uart_buffers		uart_buffer;			// Buffer for uart data

	/* Radio */
	s_radios			radio_data;				// All data for radio communication

	/* Data */
	s_position			position;				// Drone position (angled, speed, etc.)
	s_GNSS				gps_data;				// GPS data
	s_data				data;					// Other drone data (batterys, temp, hum, press...)

} s_drone_data;



/*###########################################################################################################################################################*/
/* Functions */

uint8_t RF_packet_decode(s_packets *packets, s_drone_data *drone_data);
void packet_create_telemetry(s_packets *packets, s_drone_data *drone_data);


#endif /* INC_DRONE_DATA_H_ */
