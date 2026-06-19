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
#define LOG_BUFFER_SIZE		50	// data size is aligned with frequency at which data is collected (saved at 1Hz)


/*###########################################################################################################################################################*/
/* Structs and enums */

typedef enum {
    ext_flash,
    int_flash,
    sd_card
} e_storage_target;


// Size: 60 bytes, 50Hz = 3000 B/s, 93 minutes
// Optimize: Pitch = int16 (deg * 100), size cca. 28 bytes, 3h - 4h at 50Hz
typedef struct {
    uint32_t timestamp;

    float Pitch;
    float Roll;
    float Yaw;

    float accel_x;					// Raw data - acceleration x
    float accel_y;					// Raw data - acceleration y
    float accel_z;					// Raw data - acceleration z

    float gyro_x;					// Raw data - gyro x
    float gyro_y;					// Raw data - gyro y
    float gyro_z;					// Raw data - gyro z

    float servo_xp;					// Servo angle - X+
    float servo_xn;					// Servo angle - X-
    float servo_yp;					// Servo angle - Y+
    float servo_yn;					// Servo angle - Y-

    uint32_t pressure;

    uint16_t gyroTemp;
    uint16_t height_TOF_mm;
    uint16_t height_baro_m;

    uint16_t battery_main_voltage;	// Voltage of main board battery
    uint16_t battery_edf_voltage;	// Voltage of EDF fan battery

    uint16_t temperature;

    uint8_t humidity;
    uint8_t edf_percent;			// Percents of power on EDF

} s_logging_buffer;


/*###########################################################################################################################################################*/
/* Functions */

/* LITTLEFS */
int log_init(void);
void log_test_write(void);
void log_test_read(void);
int log_open_file(void);
int log_close_file(void);
void log_add_sample(s_position *pos, s_data *data, s_actuators *actuators);
int log_dump_uart(const char *path, UART_HandleTypeDef *huart);
void log_remove(void);


#endif /* INC_LOGGING_H_ */
