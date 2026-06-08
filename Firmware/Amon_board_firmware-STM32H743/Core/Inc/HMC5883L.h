
/*****************************************************************
 * File Name          : HMC5883L.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/03/27
 * Description        : Driver for three-axis digital compass
*****************************************************************/

#ifndef INC_HMC5883L_H_
#define INC_HMC5883L_H_

/* Normal setup:
 *  - SAMPLES_8
 *  - DTR_15
 *  - MEAS_MODE_NORMAL
 *  - GAIN_1_3GA
 *  - OP_MODE_CONTINUOUS
 */

/* Compass results - heading 0-360deg
 * 0deg = North
 * 90deg = East
 * 180deg = South
 * 270deg = West
 */

/*###########################################################################################################################################################*/
/* Includes */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "stm32h7xx_hal.h"


/*###########################################################################################################################################################*/
/* Defines */

/* Device I2C address [7-bit address 0x1E, 8-bit HAL address 0x3C/0x3D] */
#define HMC5883L_ID                     (0x1E << 1)

/* Device registers */
#define CONFIG_A                        0x00    // Configuration Register A
#define CONFIG_B                        0x01    // Configuration Register B
#define MODE_REG                        0x02    // Mode Register
#define DATA_OUT_X_MSB                 	0x03    // Data Output X MSB Register
#define DATA_OUT_X_LSB                 	0x04    // Data Output X LSB Register
#define DATA_OUT_Z_MSB                 	0x05    // Data Output Z MSB Register
#define DATA_OUT_Z_LSB                 	0x06    // Data Output Z LSB Register
#define DATA_OUT_Y_MSB                 	0x07    // Data Output Y MSB Register
#define DATA_OUT_Y_LSB                 	0x08    // Data Output Y LSB Register
#define STATUS_REG                      0x09    // Status Register
#define ID_A                            0x0A    // Identification Register A
#define ID_B                            0x0B    // Identification Register B
#define ID_C                            0x0C    // Identification Register C

/* Configuration Register A masks */
#define CRA7_MASK                       0x00    // Bit 7 must be cleared
#define MA_MASK                         0x60    // Samples averaged bits
#define DO_MASK                         0x1C    // Data output rate bits
#define MS_MASK                         0x03    // Measurement mode bits

/* Configuration Register B masks */
#define GN_MASK                         0xE0    // Gain bits

/* Mode Register masks */
#define MD_MASK                         0x03    // Operating mode bits

/* Status register bits */
#define STATUS_LOCK                     (1 << 1)
#define STATUS_RDY                      (1 << 0)

/* Overflow / invalid measurement value */
#define HMC5883L_OVERFLOW_VALUE         ((int16_t)-4096)


// Hard-iron correction offsets
/* Correction definition
 * 	 X_OFFSET = (x_min + x_max) / 2.0f;
 * 	 Y_OFFSET = (y_min + y_max) / 2.0f;
 */
#define X_GAUSS_CORR					0.0f
#define Y_GAUSS_CORR					0.008f
#define Z_GAUSS_CORR					0.040f


/*###########################################################################################################################################################*/
/* Structs & enums */

/* Configuration Register A: MA1 | MA0 */
typedef enum {
    SAMPLES_1 = 0x00,
    SAMPLES_2 = 0x20,
    SAMPLES_4 = 0x40,
    SAMPLES_8 = 0x60
} e_averaging;

/* Configuration Register A: DO2 | DO1 | DO0 */
typedef enum {
    DTR_0_75 = 0x00,
    DTR_1_5  = 0x04,
    DTR_3    = 0x08,
    DTR_7_5  = 0x0C,
    DTR_15   = 0x10,
    DTR_30   = 0x14,
    DTR_75   = 0x18
} e_output_data_rate;

/* Configuration Register A: MS1 | MS0 */
typedef enum {
    MEAS_MODE_NORMAL         = 0x00,
    MEAS_MODE_POSITIVE_BIAS  = 0x01,
    MEAS_MODE_NEGATIVE_BIAS  = 0x02
} e_measurement_mode;

/* Configuration Register B: GN2 | GN1 | GN0 */
typedef enum {
    GAIN_0_88GA = 0x00,
    GAIN_1_3GA  = 0x20,
    GAIN_1_9GA  = 0x40,
    GAIN_2_5GA  = 0x60,
    GAIN_4_0GA  = 0x80,
    GAIN_4_7GA  = 0xA0,
    GAIN_5_6GA  = 0xC0,
    GAIN_8_1GA  = 0xE0
} e_gain;

/* Mode Register: MD1 | MD0 */
typedef enum {
    OP_MODE_CONTINUOUS = 0x00,
    OP_MODE_SINGLE     = 0x01,
    OP_MODE_IDLE       = 0x02
} e_operating_mode;

/* Device configurations */
typedef struct {
    e_averaging         sample_avgeraging;
    e_output_data_rate  data_rate;
    e_measurement_mode  measurement_mode;
    e_gain              gain;
    e_operating_mode    operating_mode;
} s_config;


typedef struct {
    I2C_HandleTypeDef   *i2cHandle;     // i2c Handle
    s_config            config;         // Device config registers

    int16_t             X_Axis;         // Raw X axis data
    int16_t             Y_Axis;         // Raw Y axis data
    int16_t             Z_Axis;         // Raw Z axis data

    float               X_Gauss;        // X axis in Gauss
    float               Y_Gauss;        // Y axis in Gauss
    float               Z_Gauss;        // Z axis in Gauss

    float               HeadingDeg;     // Heading in degrees 0-360
    uint8_t             DataReady;      // RDY flag from status register
    uint8_t             DataLocked;     // LOCK flag from status register
    uint8_t             Overflow;       // 1 if one axis contains overflow value

} s_HMC5883L;


/*###########################################################################################################################################################*/
/* Functions */

/* Initialize device with values saved in dev->config */
uint8_t HMC5883L_Init(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle);

/* Check identification registers: 'H', '4', '3' */
uint8_t HMC5883L_CheckID(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle);

/* Write complete device configuration from struct */
uint8_t HMC5883L_SetConfig(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle);

/* Set operating mode only */
uint8_t HMC5883L_SetMode(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle, e_operating_mode mode);

/* Read status register */
uint8_t HMC5883L_ReadStatus(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle, uint8_t *status);

/* Read raw X, Y and Z axis values */
uint8_t HMC5883L_ReadRawData(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle);

/* Read raw data and convert to Gauss */
uint8_t HMC5883L_ReadGaussData(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle);

/* Read data and calculate heading in degrees */
uint8_t HMC5883L_ReadHeading(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle, float declination_deg);

/* Run built-in self test in positive bias mode */
uint8_t HMC5883L_SelfTest(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle);

#endif /* INC_HMC5883L_H_ */
