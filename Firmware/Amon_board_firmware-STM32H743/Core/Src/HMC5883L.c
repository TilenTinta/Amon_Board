
/*****************************************************************
 * File Name          : HMC5883L.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/03/27
 * Description        : Driver for three-axis digital compass
*****************************************************************/

#ifndef HMC5883L_C_
#define HMC5883L_C_

#include "HMC5883L.h"


/*###########################################################################################################################################################*/
/* Private functions */

static HAL_StatusTypeDef HMC5883L_ReadRegister(s_HMC5883L *dev, uint8_t reg, uint8_t *data);                        // Read one register from device
static HAL_StatusTypeDef HMC5883L_ReadRegisters(s_HMC5883L *dev, uint8_t reg, uint8_t *data, uint8_t lenght);       // Read multiple registers from device
static HAL_StatusTypeDef HMC5883L_WriteRegister(s_HMC5883L *dev, uint8_t reg, uint8_t data);                        // Write register to device
static float HMC5883L_GetLsbPerGauss(s_HMC5883L *dev);                                                              // Return selected sensitivity in LSB/Gauss
static float HMC5883L_NormalizeHeading(float heading);                                                              // Normalize heading to 0-360 degrees


/*###########################################################################################################################################################*/
/* Functions */


/*********************************************************************
* @fn     	HMC5883L_CheckID
*
* @param 	*dev: struct to device data
* @param 	*i2cHandle: i2c handle struct
*
* @brief   	Read identification registers and check device ID
* 		   	Expected: H43
*
* @return  	OK: 0, NOK: 1
*/
uint8_t HMC5883L_CheckID(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle)
{
    dev -> i2cHandle = i2cHandle;

    HAL_StatusTypeDef status;
    uint8_t idData[3] = {};

    status = HMC5883L_ReadRegisters(dev, ID_A, idData, 3);

    if (status != HAL_OK)
    {
        return 1;   // NOK
    }
    else
    {
        if ((idData[0] == 'H') && (idData[1] == '4') && (idData[2] == '3'))
        {
            return 0; // OK
        }
        else
        {
            return 1; // NOK
        }
    }
}



/*********************************************************************
* @fn     	HMC5883L_Init
*
* @param 	*dev: struct to device data
* @param 	*i2cHandle: i2c handle struct
*
* @brief   	Initialize device with configuration from struct
*
* @return  	OK: 0, NOK: 1
*/
uint8_t HMC5883L_Init(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle)
{
    dev -> i2cHandle = i2cHandle;

    if (HMC5883L_CheckID(dev, i2cHandle) != 0)
    {
        return 1;   // NOK
    }

    dev->X_Axis = 0;
    dev->Y_Axis = 0;
    dev->Z_Axis = 0;

    dev->X_Gauss = 0.0f;
    dev->Y_Gauss = 0.0f;
    dev->Z_Gauss = 0.0f;

    dev->HeadingDeg = 0.0f;
    dev->DataReady = 0;
    dev->DataLocked = 0;
    dev->Overflow = 0;

    if (HMC5883L_SetConfig(dev, i2cHandle) != 0)
    {
        return 1;   // NOK
    }
    else
    {
        return 0;   // OK
    }
}



/*********************************************************************
* @fn     	HMC5883L_SetConfig
*
* @param 	*dev: struct to device data
* @param 	*i2cHandle: i2c handle struct
*
* @brief   	Write complete device configuration
*
* @return  	OK: 0, NOK: 1
*/
uint8_t HMC5883L_SetConfig(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle)
{
    dev -> i2cHandle = i2cHandle;

    HAL_StatusTypeDef status;
    uint8_t regData;

    /* Configuration register A */
    regData = CRA7_MASK |
              (dev->config.sample_avgeraging & MA_MASK) |
              (dev->config.data_rate & DO_MASK) |
              (dev->config.measurement_mode & MS_MASK);

    status = HMC5883L_WriteRegister(dev, CONFIG_A, regData);

    if (status != HAL_OK)
    {
        return 1;   // NOK
    }

    /* Configuration register B */
    regData = (dev->config.gain & GN_MASK);
    status = HMC5883L_WriteRegister(dev, CONFIG_B, regData);

    if (status != HAL_OK)
    {
        return 1;   // NOK
    }

    /* Mode register */
    regData = (dev->config.operating_mode & MD_MASK);
    status = HMC5883L_WriteRegister(dev, MODE_REG, regData);

    if (status != HAL_OK)
    {
        return 1;   // NOK
    }
    else
    {
        return 0;   // OK
    }
}



/*********************************************************************
* @fn    	HMC5883L_SetMode
*
* @param 	*dev: struct to device data
* @param 	*i2cHandle: i2c handle struct
* @param 	mode: operating mode
*
* @brief   	Set operating mode
* 			If init is caled this is not needed.
* 			Use it only when want to switch mode mid operation
*
* @return  	OK: 0, NOK: 1
*/
uint8_t HMC5883L_SetMode(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle, e_operating_mode mode)
{
    dev -> i2cHandle = i2cHandle;

    HAL_StatusTypeDef status;
    dev->config.operating_mode = mode;

    status = HMC5883L_WriteRegister(dev, MODE_REG, (uint8_t)(mode & MD_MASK));

    if (status != HAL_OK)
    {
        return 1;   // NOK
    }
    else
    {
        return 0;   // OK
    }
}



/*********************************************************************
* @fn     	HMC5883L_ReadStatus
*
* @param 	*dev: struct to device data
* @param 	*i2cHandle: i2c handle struct
* @param 	*status: pointer to returned status register value
*
* @brief   	Read status register
*
* @return  	OK: 0, NOK: 1
*/
uint8_t HMC5883L_ReadStatus(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle, uint8_t *status)
{
    dev -> i2cHandle = i2cHandle;

    HAL_StatusTypeDef status_hal;

    status_hal = HMC5883L_ReadRegister(dev, STATUS_REG, status);

    if (status_hal != HAL_OK)
    {
        return 1;   // NOK
    }

    dev->DataReady = ((*status & STATUS_RDY) != 0);
    dev->DataLocked = ((*status & STATUS_LOCK) != 0);

    return 0; // OK
}



/*********************************************************************
* @fn     	HMC5883L_ReadRawData
*
* @param 	*dev: struct to device data
* @param 	*i2cHandle: i2c handle struct
*
* @brief   	Read raw X, Y and Z axis data
*
* @return  	OK: 0, NOK: 1
*/
uint8_t HMC5883L_ReadRawData(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle)
{
    dev -> i2cHandle = i2cHandle;

    HAL_StatusTypeDef status;
    uint8_t Data[6] = {};
    uint8_t regStatus = 0;

    if (HMC5883L_ReadStatus(dev, i2cHandle, &regStatus) != 0)
    {
        return 1;   // NOK
    }

    status = HMC5883L_ReadRegisters(dev, DATA_OUT_X_MSB, Data, 6);

    if (status != HAL_OK)
    {
        return 1;   // NOK
    }

    dev->X_Axis = (int16_t)((Data[0] << 8) | Data[1]);
    dev->Z_Axis = (int16_t)((Data[2] << 8) | Data[3]);
    dev->Y_Axis = (int16_t)((Data[4] << 8) | Data[5]);

    dev->Overflow = 0;

    if ((dev->X_Axis == HMC5883L_OVERFLOW_VALUE) ||
        (dev->Y_Axis == HMC5883L_OVERFLOW_VALUE) ||
        (dev->Z_Axis == HMC5883L_OVERFLOW_VALUE))
    {
        dev->Overflow = 1;
    }

    if (dev->config.operating_mode == OP_MODE_SINGLE)
    {
        dev->config.operating_mode = OP_MODE_IDLE;
    }

    return 0; // OK
}



/*********************************************************************
* @fn     	HMC5883L_ReadGaussData
*
* @param 	*dev: struct to device data
* @param 	*i2cHandle: i2c handle struct
*
* @brief   	Read raw data and convert it to Gauss
*
* @return  	OK: 0, NOK: 1
*/
uint8_t HMC5883L_ReadGaussData(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle)
{
    dev -> i2cHandle = i2cHandle;

    float scale;

    if (HMC5883L_ReadRawData(dev, i2cHandle) != 0)
    {
        return 1;   // NOK
    }

    scale = HMC5883L_GetLsbPerGauss(dev);

    if (scale <= 0.0f)
    {
        return 1;   // NOK
    }

    dev->X_Gauss = (float)dev->X_Axis / scale;
    dev->Y_Gauss = (float)dev->Y_Axis / scale;
    dev->Z_Gauss = (float)dev->Z_Axis / scale;

    return 0; // OK
}



/*********************************************************************
* @fn    	 HMC5883L_ReadHeading
*
* @param 	*dev: struct to device data
* @param 	*i2cHandle: i2c handle struct
* @param 	declination_deg: magnetic declination in degrees
*
* @brief   	Read data and calculate heading in degrees
*
* @return 	OK: 0, NOK: 1
*/
uint8_t HMC5883L_ReadHeading(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle, float declination_deg)
{
    dev -> i2cHandle = i2cHandle;

    float headingRad;
    const float pi = 3.14159265359f;

    if (HMC5883L_ReadGaussData(dev, i2cHandle) != 0)
    {
        return 1;   // NOK
    }

    if (dev->Overflow != 0)
    {
        return 1;   // NOK
    }

    // Flat PCB on a desk
//  float x_corr = dev->X_Gauss + X_GAUSS_CORR; // 0.049f
//  float y_corr = dev->Y_Gauss + Y_GAUSS_CORR; // 0.012f
//  headingRad = atan2f(dev->Y_Gauss, dev->X_Gauss); // Flat PCB orientation

    // IC on drone mapping
    float y_corr = dev->Y_Gauss - Y_GAUSS_CORR;
    float z_corr = dev->Z_Gauss - Z_GAUSS_CORR;
    headingRad = atan2f(z_corr, y_corr); // Vertical PCB orientation
//    headingRad = atan2f(-z_corr, y_corr);
//    headingRad = atan2f(z_corr, -y_corr);

    headingRad += (declination_deg * pi / 180.0f);

    dev->HeadingDeg = HMC5883L_NormalizeHeading(headingRad * (180.0f / pi));

    return 0; // OK
}



/*********************************************************************
* @fn     	HMC5883L_WaitReady
*
* @param 	*dev: struct to device data
* @param 	*i2cHandle: i2c handle struct
* @param 	timeout_ms: timeout value in miliseconds
*
* @brief   	Wait on status ready from device
* 			If device is not ready it returns error
*
* @return  	OK: 0, NOK: 1
*/
uint8_t HMC5883L_WaitReady(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle, uint32_t timeout_ms)
{
    uint8_t status = 0;
    uint32_t start = HAL_GetTick();

    do
    {
        if (HMC5883L_ReadStatus(dev, i2cHandle, &status) != 0)
            return 1;

        if (status & STATUS_RDY)
            return 0;

    } while ((HAL_GetTick() - start) < timeout_ms);

    return 1;
}



/*********************************************************************
* @fn     	HMC5883L_SelfTest
*
* @param 	*dev: struct to device data
* @param 	*i2cHandle: i2c handle struct
*
* @brief   	Run built-in self test in positive bias mode
*
* @return  	OK: 0, NOK: 1
*/
uint8_t HMC5883L_SelfTest(s_HMC5883L *dev, I2C_HandleTypeDef *i2cHandle)
{
    dev -> i2cHandle = i2cHandle;

    s_config oldConfig = dev->config;

    // Clear possible old/locked data
    uint8_t dummy[6];
    HMC5883L_ReadRegisters(dev, DATA_OUT_X_MSB, dummy, 6);

    /* Positive bias mode, gain 2.5 Ga (0x60) as recommended in datasheet */
    dev->config.measurement_mode = MEAS_MODE_POSITIVE_BIAS;
    dev->config.gain = GAIN_2_5GA;
    dev->config.operating_mode = OP_MODE_SINGLE;

    if (HMC5883L_SetConfig(dev, i2cHandle) != 0)
    {
        dev->config = oldConfig;
        return 1;   // NOK
    }

    if (HMC5883L_WaitReady(dev, i2cHandle, 100) != 0)
    {
        dev->config = oldConfig;
        HMC5883L_SetConfig(dev, i2cHandle);
        return 1;
    }

    if (HMC5883L_ReadRawData(dev, i2cHandle) != 0)
    {
        dev->config = oldConfig;
        HMC5883L_SetConfig(dev, i2cHandle);
        return 1;   // NOK
    }

    dev->config = oldConfig;

    if (HMC5883L_SetConfig(dev, i2cHandle) != 0)
    {
        return 1;   // NOK
    }

    /* Typical expected values at gain 2.5 Ga:
     * X, Y around +766 LSB
     * Z around +713 LSB
     */
    if ((dev->X_Axis < 400 || dev->X_Axis > 1100) || (dev->Y_Axis < 400 || dev->Y_Axis > 1100) || (dev->Z_Axis < 350 || dev->Z_Axis > 1000))
    {
        return 1; // NOK
    }

    return 0; // OK
}



/*********************************************************************
* @fn     	HMC5883L_GetLsbPerGauss
*
* @param 	*dev: struct to device data
*
* @brief   	Convert selected gain to LSB/Gauss
* 			Choose a lower gain value (higher GN#) when
* 			total field strength causes overflow in one of
* 			the data output registers (saturation).
*
* @return  	sensitivity
*/
static float HMC5883L_GetLsbPerGauss(s_HMC5883L *dev)
{

    switch (dev->config.gain)
    {
        case GAIN_0_88GA:
            return 1370.0f;

        case GAIN_1_3GA:
            return 1090.0f;

        case GAIN_1_9GA:
            return 820.0f;

        case GAIN_2_5GA:
            return 660.0f;

        case GAIN_4_0GA:
            return 440.0f;

        case GAIN_4_7GA:
            return 390.0f;

        case GAIN_5_6GA:
            return 330.0f;

        case GAIN_8_1GA:
            return 230.0f;

        default:
            return 0.0f;
    }
}



/*********************************************************************
* @fn     	HMC5883L_NormalizeHeading
*
* @param 	heading: heading in degrees
*
* @brief   	Normalize heading to 0-360 degrees
*
* @return  	normalized heading
*/
static float HMC5883L_NormalizeHeading(float heading)
{

    while (heading < 0.0f)
    {
        heading += 360.0f;
    }

    while (heading >= 360.0f)
    {
        heading -= 360.0f;
    }

    return heading;
}



/* LL Drivers */
static HAL_StatusTypeDef HMC5883L_ReadRegister(s_HMC5883L *dev, uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read (dev -> i2cHandle, HMC5883L_ID, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

static HAL_StatusTypeDef HMC5883L_ReadRegisters(s_HMC5883L *dev, uint8_t reg, uint8_t *data, uint8_t lenght)
{
    return HAL_I2C_Mem_Read (dev -> i2cHandle, HMC5883L_ID, reg, I2C_MEMADD_SIZE_8BIT, data, lenght, 100);
}

static HAL_StatusTypeDef HMC5883L_WriteRegister(s_HMC5883L *dev, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write (dev -> i2cHandle, HMC5883L_ID, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

#endif /* HMC5883L_C_ */
