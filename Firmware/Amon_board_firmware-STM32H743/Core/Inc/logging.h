/*****************************************************************
 * File Name          : logging.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/02/26
 * Description        : Log data to flash / SD card
*****************************************************************/

#ifndef INC_LOGGING_H_
#define INC_LOGGING_H_

#include <stdint.h>
#include "flash_W25QXXX.h"
//#include "flash.h"
#include "../Drivers/littlefs/lfs.h"

#include "drone_data.h"

#if defined(LOG_LITTLEFS) && defined(LOG_RAW_FLASH)
#error "Select only one logging backend: LOG_LITTLEFS or LOG_RAW_FLASH"
#endif

/* IDEAS:
 * - Use multiple logs and delete the oldest:
 * 		- flight_0001.bin
 *		- flight_0002.bin
 *		- flight_0003.bin
 *
 *
 * */

/*###########################################################################################################################################################*/
/* Defines */
#define LOG_BUFFER_SIZE		50	// data size is aligned with frequency at which data is collected


/*###########################################################################################################################################################*/
/* Structs and enums */

typedef enum {
    ext_flash,
    int_flash,
    sd_card
} e_storage_target;


// Keep field order aligned with ground-control LOG_RECORD_STRUCT.
typedef struct {
    uint32_t	timestamp;

    float 		servo_xp;
    float 		servo_xn;
    float 		servo_yp;
    float 		servo_yn;

    float 	    nmpc_solver_time;
    int			nmpc_solve_status;
    int         nmpc_last_qp_iter;
    int         nmpc_last_qp_status;

    float       nmpc_z_ref;              // NMPC height reference [m]
    float       nmpc_z_current;          // Current height passed to NMPC [m]
    float       nmpc_z_error;            // Height error: z_ref - z_current [m]
    float       nmpc_vz_ref_target;      // Vertical speed target before slew limiter [m/s]
    float       nmpc_vz_ref_slew;        // Vertical speed reference after slew limiter [m/s]
    float       nmpc_vz_current;         // Current vertical speed passed to NMPC [m/s]
    float       nmpc_u0_ref;             // Reference value of the first NMPC input
    float       nmpc_u0_opt;             // Optimized value of the first NMPC input
    float       nmpc_u0_lbu;             // Lower bound of the first NMPC input
    float       nmpc_u0_ubu;             // Upper bound of the first NMPC input

    float 		heading_deg;
    float 		Pitch;
    float 		Roll;
    float 		Yaw;

    float 		accel_x;				// Raw data - acceleration x
    float 		accel_y;				// Raw data - acceleration y
    float		accel_z;				// Raw data - acceleration z

    float 		gyro_x;					// Raw data - gyro x
    float 		gyro_y;					// Raw data - gyro y
    float 		gyro_z;					// Raw data - gyro z

    float		quaternion[4];

    float 		height_TOF_m_filtered;
    uint16_t 	gyroTemp;
    uint16_t 	height_TOF_mm;

    uint16_t 	height_baro_m;

    uint16_t 	battery_main_voltage;	// Voltage of main board battery
    uint16_t 	battery_edf_voltage;	// Voltage of EDF fan battery

    uint16_t 	temperature;

    uint32_t 	pressure;
    uint8_t 	humidity;
    uint8_t 	edf_percent;			// Percents of power on EDF

} s_logging_buffer;


/*###########################################################################################################################################################*/
/* Functions */

/* LITTLEFS */
int log_init(void);
void log_test_write(void);
void log_test_read(void);
int log_open_file(void);
int log_close_file(void);
void log_add_sample(s_position *pos, s_data *data, s_regulators *reg, s_actuators *actuators);
int log_dump_uart(const char *path, UART_HandleTypeDef *huart);
void log_remove(void);


/* RAW FLASH */


#endif /* INC_LOGGING_H_ */
