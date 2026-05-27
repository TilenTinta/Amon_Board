/*****************************************************************
 * File Name          : PMW3901.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/05/24
 * Description        : Driver for PMW3901 optical flow sensor
*****************************************************************/

#ifndef INC_PMW3901_H_
#define INC_PMW3901_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <stdint.h>
#include <string.h>
#include "stm32h7xx_hal.h"

/*###########################################################################################################################################################*/
/* Defines */

/*
 * PMW3901 SPI:
 * - 4-wire SPI
 * - max SCLK: 2 MHz
 * - SPI mode normally used by PMW3901 modules: mode 3 (CPOL=1, CPHA=1)
 * - Register read:  send reg & 0x7F, wait tSRAD, then clock dummy byte
 * - Register write: send reg | 0x80, then value
 */
#define PMW3901_SPI_DUMMY              	0x00U

#define PMW3901_READ_MASK              	0x7FU
#define PMW3901_WRITE_MASK             	0x80U

/* Important timing */
#define PMW3901_TSRAD_US               	35U     // address-to-data delay for read
#define PMW3901_TSRR_US                	20U     // delay after read before next command
#define PMW3901_TSWW_US                	45U     // delay after write before next command
#define PMW3901_RESET_DELAY_MS         	50U     // reset to valid motion
#define PMW3901_NCS_BURST_EXIT_US      	1U

/* Register map - common PMW3901 register addresses */
#define PMW3901_REG_PRODUCT_ID         	0x00U
#define PMW3901_REG_REVISION_ID        	0x01U
#define PMW3901_REG_MOTION             	0x02U
#define PMW3901_REG_DELTA_X_L          	0x03U
#define PMW3901_REG_DELTA_X_H          	0x04U
#define PMW3901_REG_DELTA_Y_L          	0x05U
#define PMW3901_REG_DELTA_Y_H          	0x06U
#define PMW3901_REG_SQUAL              	0x07U
#define PMW3901_REG_RAW_DATA_SUM       	0x08U
#define PMW3901_REG_MAX_RAW_DATA       	0x09U
#define PMW3901_REG_MIN_RAW_DATA       	0x0AU
#define PMW3901_REG_SHUTTER_LOWER      	0x0BU
#define PMW3901_REG_SHUTTER_UPPER      	0x0CU
#define PMW3901_REG_OBSERVATION        	0x15U
#define PMW3901_REG_MOTION_BURST       	0x16U
#define PMW3901_REG_POWER_UP_RESET     	0x3AU
#define PMW3901_REG_SHUTDOWN           	0x3BU
#define PMW3901_REG_INVERSE_PRODUCT_ID 	0x5FU

#define PMW3901_PRODUCT_ID             	0x49U

// Algorithm variables
#define PMW3901_SQUAL_MIN_VALID     	20U		// Surface quality threshold (<10: terrible, 20: acceptable, 40+ good, 80+: very good)
#define PMW3901_DT						0.005f	// Time of sensor reading period
//#define PIXART_INIT

/*###########################################################################################################################################################*/
/* Structs and enums */

typedef enum {
    PMW3901_OK,
    PMW3901_ERR,
    PMW3901_ERR_NULL,
    PMW3901_ERR_SPI,
    PMW3901_ERR_ID

} e_status;


typedef struct {
    SPI_HandleTypeDef   *SPIx;          // SPI Handle
    GPIO_TypeDef        *CS_Port;       // chip select port
    uint16_t            CS_Pin;         // chip select pin
    GPIO_TypeDef        *RST_Port;      // RESET port
    uint16_t            RST_Pin;        // RESET pin
    GPIO_TypeDef        *MOT_Port;      // MOTION interrupt port
    uint16_t            MOT_Pin;        // MOTION interrupt pin

} s_pinout;


typedef struct {
	uint8_t 	motion;
	int16_t 	delta_x;
	int16_t 	delta_y;

	uint8_t 	squal;
	uint8_t 	raw_data_sum;
	uint8_t 	max_raw_data;
	uint8_t 	min_raw_data;

	uint16_t 	shutter;

} s_raw_data;


typedef struct {
	// Height
	float 	altitude_m;

    // Processed / validated data
    uint8_t motion_valid;
    uint8_t surface_valid;

    // Accumulated optical flow
    int32_t position_x;
    int32_t position_y;

    // Filtered accumulated position
    int32_t filtered_position_x;
    int32_t filtered_position_y;

    // Gyro data for canceling rotation
    float 	gyro_x_rad_s;
    float 	gyro_y_rad_s;
    float 	gyro_z_rad_s;

} s_motion_data;


typedef struct {
	float 	velocity_x_mps;
	float 	velocity_y_mps;

	float 	position_x_m;
	float 	position_y_m;

	float 	flow_scale;

	float 	flow_x;
	float 	flow_y;

	float 	flow_x_corrected;
	float 	flow_y_corrected;

} s_measurements;


typedef struct {

    s_pinout 	pinout;

    volatile uint8_t motion_irq_flag;

    // Device info
    uint8_t 	product_id;
    uint8_t 	revision_id;

    // Raw sensor registers
    s_raw_data 	raw_data;

    // Processed / validated data
    s_motion_data data;

    // Statistics
    uint32_t 	motion_read_counter;
    uint32_t 	motion_rejected_counter;

    // Measurements
    s_measurements measurements;

} s_PMW3901;

/*###########################################################################################################################################################*/
/* Functions */

void PMW3901_pin_config(s_PMW3901 *dev, SPI_HandleTypeDef *SPIx, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *rst_port, uint16_t rst_pin, GPIO_TypeDef *mot_port, uint16_t mot_pin);

uint8_t PMW3901_SPI_Write(s_PMW3901 *dev, const uint8_t *tx, uint16_t len);
uint8_t PMW3901_SPI_Read(s_PMW3901 *dev, uint8_t *rx, uint16_t len, uint8_t fill_byte);
uint8_t PMW3901_SPI_Transceive(s_PMW3901 *dev, const uint8_t *tx, uint8_t *rx, uint16_t len);

uint8_t PMW3901_ReadRegister(s_PMW3901 *dev, uint8_t reg, uint8_t *value);
uint8_t PMW3901_WriteRegister(s_PMW3901 *dev, uint8_t reg, uint8_t value);

void PMW3901_Reset(s_PMW3901 *dev);
uint8_t PMW3901_ReadProductId(s_PMW3901 *dev);
uint8_t PMW3901_ReadMotion(s_PMW3901 *dev);
uint8_t PMW3901_ReadMotionBurst(s_PMW3901 *dev);
uint8_t PMW3901_MotionDetected(s_PMW3901 *dev);
uint8_t PMW3901_init(s_PMW3901 *dev);

uint8_t PMW3901_MotionValid(s_PMW3901 *dev);
void PMW3901_ClearMotion(s_PMW3901 *dev);
uint8_t PMW3901_GetSQUAL(s_PMW3901 *dev);
uint16_t PMW3901_GetShutter(s_PMW3901 *dev);
void PMW3901_AccumulatePosition(s_PMW3901 *dev);

void PMW3901_Update(s_PMW3901 *dev);


#ifdef __cplusplus
}
#endif

#endif /* INC_PMW3901_H_ */
