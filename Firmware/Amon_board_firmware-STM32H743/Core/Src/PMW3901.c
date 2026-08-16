/*****************************************************************
 * File Name          : PMW3901.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/05/24
 * Description        : Driver for PMW3901 optical flow sensor
*****************************************************************/

/* Includes */
#include "PMW3901.h"
#include <math.h>

/*###########################################################################################################################################################*/
/* Private functions - used to manipulate pin states and SPI bus */

// Reset CS pin
static inline void pmw_cs_low(s_PMW3901 *dev)
{
    HAL_GPIO_WritePin(dev->pinout.CS_Port, dev->pinout.CS_Pin, GPIO_PIN_RESET);
}

// Set CS pin
static inline void pmw_cs_high(s_PMW3901 *dev)
{
    HAL_GPIO_WritePin(dev->pinout.CS_Port, dev->pinout.CS_Pin, GPIO_PIN_SET);
}

// Reset RST pin
static inline void pmw_reset_low(s_PMW3901 *dev)
{
    if (dev->pinout.RST_Port != NULL)
    {
        HAL_GPIO_WritePin(dev->pinout.RST_Port, dev->pinout.RST_Pin, GPIO_PIN_RESET);
    }
}

// Set RST pin
static inline void pmw_reset_high(s_PMW3901 *dev)
{
    if (dev->pinout.RST_Port != NULL)
    {
        HAL_GPIO_WritePin(dev->pinout.RST_Port, dev->pinout.RST_Pin, GPIO_PIN_SET);
    }
}

// Send and receive data over SPI
static inline uint8_t pmw_spi_txrx(s_PMW3901 *dev, uint8_t byte)
{
    uint8_t rx = 0;

    if (HAL_SPI_TransmitReceive(dev->pinout.SPIx, &byte, &rx, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return 0xFF;
    }

    return rx;
}

// Delay used in driver
static inline void pmw_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks);
}



/*###########################################################################################################################################################*/
/* Low-level functions */

/*********************************************************************
 * @fn      PMW3901_pin_config
 *
 * @param   *dev: pointer to device struct
 * @param   *SPIx: pointer to SPI peripheral
 * @param   *cs_port: pointer to chip select port
 * @param   cs_pin: chip select pin number
 * @param   *rst_port: pointer to reset port
 * @param   rst_pin: reset pin number
 * @param   *rst_port: pointer to chip enable port
 * @param   rst_pin: chip enable pin number
 *
 * @brief   Set SPI and CS/RST pins to a device struct
 *
 * @return  None
 */
void PMW3901_pin_config(s_PMW3901 *dev,
                        SPI_HandleTypeDef *SPIx,
                        GPIO_TypeDef *cs_port,
                        uint16_t cs_pin,
                        GPIO_TypeDef *rst_port,
                        uint16_t rst_pin,
                        GPIO_TypeDef *mot_port,
                        uint16_t mot_pin)
{
    if (dev == NULL)
    {
        return;
    }

    memset(dev, 0, sizeof(*dev));

    dev->pinout.SPIx     = SPIx;
    dev->pinout.CS_Port  = cs_port;
    dev->pinout.CS_Pin   = cs_pin;
    dev->pinout.RST_Port = rst_port;
    dev->pinout.RST_Pin  = rst_pin;
    dev->pinout.MOT_Port = mot_port;
    dev->pinout.MOT_Pin  = mot_pin;

    pmw_cs_high(dev);
    pmw_reset_high(dev);
}



/*********************************************************************
 * @fn      PMW3901_SPI_Write
 *
 * @param   *dev: device struct
 * @param   *tx: pointer to data for transmition
 * @param   len: lenght of data to tansmit
 *
 * @brief   Write/send raw bytes
 *          Discarding RX bytes
 *
 * @return  0 OK, 1 NOK
 */
uint8_t PMW3901_SPI_Write(s_PMW3901 *dev, const uint8_t *tx, uint16_t len)
{
    if (dev == NULL || tx == NULL || len == 0U)
    {
        return PMW3901_ERR_NULL;
    }

    pmw_cs_low(dev);

    for (uint16_t i = 0; i < len; i++)
    {
        (void)pmw_spi_txrx(dev, tx[i]);
    }

    pmw_cs_high(dev);

    return PMW3901_OK;
}



/*********************************************************************
 * @fn      PMW3901_SPI_Read
 *
 * @param   *dev: device struct
 * @param   *rx: pointer to received data (where to save data)
 * @param   len: lenght of data that will be reveived
 * @param   fill_byte: NOP (send NOP data just to get response)
 *
 * @brief   Read raw bytes
 *
 * @return  0 OK, 1 NOK
 */
uint8_t PMW3901_SPI_Read(s_PMW3901 *dev, uint8_t *rx, uint16_t len, uint8_t fill_byte)
{
    if (dev == NULL || rx == NULL || len == 0U)
    {
        return PMW3901_ERR_NULL;
    }

    pmw_cs_low(dev);

    for (uint16_t i = 0; i < len; i++)
    {
        rx[i] = pmw_spi_txrx(dev, fill_byte);
    }

    pmw_cs_high(dev);

    return PMW3901_OK;
}



/*********************************************************************
 * @fn      PMW3901_SPI_Transceive
 *
 * @param   *dev: device struct
 * @param   *tx: pointer to data you want to transmit
 * @param   *rx: pointer to location you want to save received data
 * @param   len: lenght of transmited data
 *
 * @brief   Send and receive data from the device
 *
 * @return  0 OK, 1 NOK
 */
uint8_t PMW3901_SPI_Transceive(s_PMW3901 *dev, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (dev == NULL || tx == NULL || rx == NULL || len == 0U)
    {
        return PMW3901_ERR_NULL;
    }

    pmw_cs_low(dev);

    for (uint16_t i = 0; i < len; i++)
    {
        rx[i] = pmw_spi_txrx(dev, tx[i]);
    }

    pmw_cs_high(dev);

    return PMW3901_OK;
}



/*********************************************************************
 * @fn      PMW3901_ReadRegister
 *
 * @param   *dev: device struct
 * @param   reg: register value you want to read
 * @param   *value: pointer to returned value
 * @param   *status_out: pointer to returned value of device status
 *
 * @brief   Read one register
 *
 * @return  0 OK, 1 NOK
 */
uint8_t PMW3901_ReadRegister(s_PMW3901 *dev, uint8_t reg, uint8_t *value)
{
    if (dev == NULL || value == NULL)
    {
        return PMW3901_ERR_NULL;
    }

    pmw_cs_low(dev);

    (void)pmw_spi_txrx(dev, reg & PMW3901_READ_MASK);
    pmw_delay_us(PMW3901_TSRAD_US);

    *value = pmw_spi_txrx(dev, PMW3901_SPI_DUMMY);

    pmw_cs_high(dev);
    pmw_delay_us(PMW3901_TSRR_US);

    return PMW3901_OK;
}



/*********************************************************************
 * @fn      PMW3901_WriteRegister
 *
 * @param   *dev: device struct
 * @param   reg: register value you want to read
 * @param   value: value you want to send
 * @param   *status_out: pointer to returned value of device status
 *
 * @brief   Write one register
 *
 * @return  0 OK, 1 NOK
 */
uint8_t PMW3901_WriteRegister(s_PMW3901 *dev, uint8_t reg, uint8_t value)
{
    if (dev == NULL)
    {
        return PMW3901_ERR_NULL;
    }

    // PMW3901 write: bit7 must be 1
    pmw_cs_low(dev);

    (void)pmw_spi_txrx(dev, reg | PMW3901_WRITE_MASK);
    (void)pmw_spi_txrx(dev, value);

    pmw_cs_high(dev);
    pmw_delay_us(PMW3901_TSWW_US);

    return PMW3901_OK;
}

/*###########################################################################################################################################################*/
/* API functions */


/*********************************************************************
 * @fn      PMW3901_Reset
 *
 * @param   *dev: device struct
 *
 * @brief   Reset device
 *
 * @return  0 OK, 1 NOK
 */
void PMW3901_Reset(s_PMW3901 *dev)
{
    if (dev == NULL)
    {
        return;
    }

    pmw_cs_high(dev);

    pmw_reset_low(dev);
    HAL_Delay(1);
    pmw_reset_high(dev);

    HAL_Delay(PMW3901_RESET_DELAY_MS);
}



/*********************************************************************
 * @fn      PMW3901_ReadProductId
 *
 * @param   *dev: device struct
 *
 * @brief   Read device product id
 *
 * @return  0 OK, 1 NOK
 */
uint8_t PMW3901_ReadProductId(s_PMW3901 *dev)
{
    if (dev == NULL)
    {
        return PMW3901_ERR_NULL;
    }

    return PMW3901_ReadRegister(dev, PMW3901_REG_PRODUCT_ID, &dev->product_id);
}



/*********************************************************************
 * @fn      PMW3901_init
 *
 * @param   *dev: device struct
 *
 * @brief	Device inicialization
 *
 * @return  0 OK, 1 NOK
 */
uint8_t PMW3901_init(s_PMW3901 *dev)
{
    uint8_t id = 0;
    uint8_t rev = 0;
    uint8_t tmp = 0;

    if (dev == NULL)
    {
        return PMW3901_ERR_NULL;
    }

    pmw_cs_high(dev);
    PMW3901_Reset(dev);

    // Software reset / power-up reset
    PMW3901_WriteRegister(dev, PMW3901_REG_POWER_UP_RESET, 0x5AU);
    HAL_Delay(20);

    // Clear motion and delta registers after power-up reset.
    for (uint8_t reg = PMW3901_REG_MOTION; reg <= PMW3901_REG_DELTA_Y_H; reg++)
    {
        PMW3901_ReadRegister(dev, reg, &tmp);
    }

    PMW3901_ReadRegister(dev, PMW3901_REG_PRODUCT_ID, &id);
    PMW3901_ReadRegister(dev, PMW3901_REG_REVISION_ID, &rev);

    dev->product_id = id;
    dev->revision_id = rev;

    if (id != PMW3901_PRODUCT_ID)
    {
        return PMW3901_ERR_ID;
    }


    /* WARNING!!!
     * PixArt PMW3901 initialization magic register sequence
     * Without that sequence many PMW3901 modules will return ID correctly
     * but motion data may not work correctly.
     */
#ifdef PIXART_INIT
    uint8_t c1 = 0;
    uint8_t c2 = 0;

    PMW3901_WriteRegister(dev, 0x7F, 0x00);
    PMW3901_WriteRegister(dev, 0x55, 0x01);
    PMW3901_WriteRegister(dev, 0x50, 0x07);
    PMW3901_WriteRegister(dev, 0x7F, 0x0E);
    PMW3901_WriteRegister(dev, 0x43, 0x10);
    PMW3901_ReadRegister(dev, 0x67, &tmp);
    PMW3901_WriteRegister(dev, 0x48, (tmp & 0x80U) ? 0x04 : 0x02);
    PMW3901_WriteRegister(dev, 0x7F, 0x00);
    PMW3901_WriteRegister(dev, 0x51, 0x7B);
    PMW3901_WriteRegister(dev, 0x50, 0x00);
    PMW3901_WriteRegister(dev, 0x55, 0x00);
    PMW3901_WriteRegister(dev, 0x7F, 0x0E);
    PMW3901_ReadRegister(dev, 0x73, &tmp);

    if (tmp == 0x00U)
    {
        PMW3901_ReadRegister(dev, 0x70, &c1);
        PMW3901_ReadRegister(dev, 0x71, &c2);

        c1 = (c1 <= 28U) ? (uint8_t)(c1 + 14U) : (uint8_t)(c1 + 11U);
        if (c1 > 0x3FU) c1 = 0x3FU;
        c2 = (uint8_t)(((uint16_t)c2 * 45U) / 100U);

        PMW3901_WriteRegister(dev, 0x7F, 0x00);
        PMW3901_WriteRegister(dev, 0x61, 0xAD);
        PMW3901_WriteRegister(dev, 0x51, 0x70);
        PMW3901_WriteRegister(dev, 0x7F, 0x0E);
        PMW3901_WriteRegister(dev, 0x70, c1);
        PMW3901_WriteRegister(dev, 0x71, c2);
    }

    PMW3901_WriteRegister(dev, 0x7F, 0x00);
    PMW3901_WriteRegister(dev, 0x61, 0xAD);
    PMW3901_WriteRegister(dev, 0x7F, 0x03);
    PMW3901_WriteRegister(dev, 0x40, 0x00);
    PMW3901_WriteRegister(dev, 0x7F, 0x05);
    PMW3901_WriteRegister(dev, 0x41, 0xB3);
    PMW3901_WriteRegister(dev, 0x43, 0xF1);
    PMW3901_WriteRegister(dev, 0x45, 0x14);
    PMW3901_WriteRegister(dev, 0x5B, 0x32);
    PMW3901_WriteRegister(dev, 0x5F, 0x34);
    PMW3901_WriteRegister(dev, 0x7B, 0x08);
    PMW3901_WriteRegister(dev, 0x7F, 0x06);
    PMW3901_WriteRegister(dev, 0x44, 0x1B);
    PMW3901_WriteRegister(dev, 0x40, 0xBF);
    PMW3901_WriteRegister(dev, 0x4E, 0x3F);
    PMW3901_WriteRegister(dev, 0x7F, 0x08);
    PMW3901_WriteRegister(dev, 0x65, 0x20);
    PMW3901_WriteRegister(dev, 0x6A, 0x18);
    PMW3901_WriteRegister(dev, 0x7F, 0x09);
    PMW3901_WriteRegister(dev, 0x4F, 0xAF);
    PMW3901_WriteRegister(dev, 0x5F, 0x40);
    PMW3901_WriteRegister(dev, 0x48, 0x80);
    PMW3901_WriteRegister(dev, 0x49, 0x80);
    PMW3901_WriteRegister(dev, 0x57, 0x77);
    PMW3901_WriteRegister(dev, 0x60, 0x78);
    PMW3901_WriteRegister(dev, 0x61, 0x78);
    PMW3901_WriteRegister(dev, 0x62, 0x08);
    PMW3901_WriteRegister(dev, 0x63, 0x50);
    PMW3901_WriteRegister(dev, 0x7F, 0x0A);
    PMW3901_WriteRegister(dev, 0x45, 0x60);
    PMW3901_WriteRegister(dev, 0x7F, 0x00);
    PMW3901_WriteRegister(dev, 0x4D, 0x11);
    PMW3901_WriteRegister(dev, 0x55, 0x80);
    PMW3901_WriteRegister(dev, 0x74, 0x21);
    PMW3901_WriteRegister(dev, 0x75, 0x1F);
    PMW3901_WriteRegister(dev, 0x4A, 0x78);
    PMW3901_WriteRegister(dev, 0x4B, 0x78);
    PMW3901_WriteRegister(dev, 0x44, 0x08);
    PMW3901_WriteRegister(dev, 0x45, 0x50);
    PMW3901_WriteRegister(dev, 0x64, 0xFF);
    PMW3901_WriteRegister(dev, 0x65, 0x1F);
    PMW3901_WriteRegister(dev, 0x7F, 0x14);
    PMW3901_WriteRegister(dev, 0x65, 0x67);
    PMW3901_WriteRegister(dev, 0x66, 0x08);
    PMW3901_WriteRegister(dev, 0x63, 0x70);
    PMW3901_WriteRegister(dev, 0x7F, 0x15);
    PMW3901_WriteRegister(dev, 0x48, 0x48);
    PMW3901_WriteRegister(dev, 0x7F, 0x07);
    PMW3901_WriteRegister(dev, 0x41, 0x0D);
    PMW3901_WriteRegister(dev, 0x43, 0x14);
    PMW3901_WriteRegister(dev, 0x4B, 0x0E);
    PMW3901_WriteRegister(dev, 0x45, 0x0F);
    PMW3901_WriteRegister(dev, 0x44, 0x42);
    PMW3901_WriteRegister(dev, 0x4C, 0x80);
    PMW3901_WriteRegister(dev, 0x7F, 0x10);
    PMW3901_WriteRegister(dev, 0x5B, 0x02);
    PMW3901_WriteRegister(dev, 0x7F, 0x07);
    PMW3901_WriteRegister(dev, 0x40, 0x41);
    PMW3901_WriteRegister(dev, 0x70, 0x00);

    HAL_Delay(10);

    PMW3901_WriteRegister(dev, 0x32, 0x44);
    PMW3901_WriteRegister(dev, 0x7F, 0x07);
    PMW3901_WriteRegister(dev, 0x40, 0x40);
    PMW3901_WriteRegister(dev, 0x7F, 0x06);
    PMW3901_WriteRegister(dev, 0x62, 0xF0);
    PMW3901_WriteRegister(dev, 0x63, 0x00);
    PMW3901_WriteRegister(dev, 0x7F, 0x0D);
    PMW3901_WriteRegister(dev, 0x48, 0xC0);
    PMW3901_WriteRegister(dev, 0x6F, 0xD5);
    PMW3901_WriteRegister(dev, 0x7F, 0x00);
    PMW3901_WriteRegister(dev, 0x5B, 0xA0);
    PMW3901_WriteRegister(dev, 0x4E, 0xA8);
    PMW3901_WriteRegister(dev, 0x5A, 0x50);
    PMW3901_WriteRegister(dev, 0x40, 0x80);

    HAL_Delay(240);

    PMW3901_WriteRegister(dev, 0x7F, 0x14);
    PMW3901_WriteRegister(dev, 0x6F, 0x1C);
    PMW3901_WriteRegister(dev, 0x7F, 0x00);

#endif

    PMW3901_ClearMotion(dev);
    dev->motion_irq_flag = 0;
    dev->measurements.flow_scale = PMW3901_FLOW_SCALE_RAD_COUNT;
    dev->measurements.velocity_x_mps = 0.0f;
    dev->measurements.velocity_y_mps = 0.0f;
    dev->measurements.position_x_m = 0.0f;
    dev->measurements.position_y_m = 0.0f;
    dev->last_motion_read_ms = 0U;
    dev->last_valid_motion_ms = 0U;

    return PMW3901_OK;
}



/*********************************************************************
 * @fn      PMW3901_MotionDetected
 *
 * @param   *dev: device struct
 *
 * @brief   Check if device senses motion
 *
 * @return  0 OK, 1 NOK
 */
uint8_t PMW3901_MotionDetected(s_PMW3901 *dev)
{
    if (dev == NULL || dev->pinout.MOT_Port == NULL)
    {
        return 0;
    }

    // MOTION pin is active low
    return (HAL_GPIO_ReadPin(dev->pinout.MOT_Port, dev->pinout.MOT_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}



/*********************************************************************
 * @fn      PMW3901_ReadMotion
 *
 * @param   *dev: device struct
 *
 * @brief
 *
 * @return  0 OK, 1 NOK
 */
uint8_t PMW3901_ReadMotion(s_PMW3901 *dev)
{
	uint8_t xl, xh, yl, yh;
	uint8_t shutter_l, shutter_h;

	if (dev == NULL)
	{
		return PMW3901_ERR_NULL;
	}

	PMW3901_ReadRegister(dev, PMW3901_REG_MOTION, &dev->raw_data.motion);

	PMW3901_ReadRegister(dev, PMW3901_REG_DELTA_X_L, &xl);
	PMW3901_ReadRegister(dev, PMW3901_REG_DELTA_X_H, &xh);

	PMW3901_ReadRegister(dev, PMW3901_REG_DELTA_Y_L, &yl);
	PMW3901_ReadRegister(dev, PMW3901_REG_DELTA_Y_H, &yh);

	PMW3901_ReadRegister(dev, PMW3901_REG_SQUAL, &dev->raw_data.squal);

	PMW3901_ReadRegister(dev, PMW3901_REG_RAW_DATA_SUM, &dev->raw_data.raw_data_sum);

	PMW3901_ReadRegister(dev, PMW3901_REG_MAX_RAW_DATA, &dev->raw_data.max_raw_data);

	PMW3901_ReadRegister(dev, PMW3901_REG_MIN_RAW_DATA, &dev->raw_data.min_raw_data);

	PMW3901_ReadRegister(dev, PMW3901_REG_SHUTTER_LOWER, &shutter_l);

	PMW3901_ReadRegister(dev, PMW3901_REG_SHUTTER_UPPER, &shutter_h);

	dev->raw_data.delta_x = (int16_t)((uint16_t)xl | ((uint16_t)xh << 8));
	dev->raw_data.delta_y = (int16_t)((uint16_t)yl | ((uint16_t)yh << 8));

	dev->raw_data.shutter = (uint16_t)shutter_l | ((uint16_t)shutter_h << 8);

	return PMW3901_OK;
}



/*********************************************************************
 * @fn      PMW3901_ReadMotionBurst
 *
 * @param   *dev: device struct
 *
 * @brief   Read coherent motion burst frame
 *
 * @return  0 OK, 1 NOK
 */
uint8_t PMW3901_ReadMotionBurst(s_PMW3901 *dev)
{
    uint8_t burst[12];

    if (dev == NULL)
    {
        return PMW3901_ERR_NULL;
    }

    pmw_cs_low(dev);

    pmw_spi_txrx(dev, PMW3901_REG_MOTION_BURST & PMW3901_READ_MASK);

    pmw_delay_us(PMW3901_TSRAD_US);

    for (uint8_t i = 0; i < sizeof(burst); i++)
    {
        burst[i] = pmw_spi_txrx(dev, PMW3901_SPI_DUMMY);
    }

    pmw_cs_high(dev);

    pmw_delay_us(PMW3901_TSRR_US);

    dev->raw_data.motion = burst[0];

    dev->raw_data.delta_x =(int16_t)((uint16_t)burst[2] | ((uint16_t)burst[3] << 8));

    dev->raw_data.delta_y = (int16_t)((uint16_t)burst[4] | ((uint16_t)burst[5] << 8));

    dev->raw_data.squal = burst[6];
    dev->raw_data.raw_data_sum = burst[7];
    dev->raw_data.max_raw_data = burst[8];
    dev->raw_data.min_raw_data = burst[9];

    dev->raw_data.shutter = ((uint16_t)burst[10] << 8) | (uint16_t)burst[11];

    return PMW3901_OK;
}



/*********************************************************************
 * @fn      PMW3901_MotionValid
 *
 * @param   *dev: device struct
 *
 * @brief   Check if latest motion data is valid
 *
 * @return  1 valid, 0 invalid
 */
uint8_t PMW3901_MotionValid(s_PMW3901 *dev)
{
    if (dev == NULL)
    {
        return 0U;
    }

    // Motion bit
    dev->data.motion_valid = ((dev->raw_data.motion & 0x80U) != 0U) ? 1U : 0U;

    // Low SQUAL is rejected only together with a saturated shutter.
    uint8_t shutter_upper = (uint8_t)(dev->raw_data.shutter >> 8);
    dev->data.surface_valid = !((dev->raw_data.squal < PMW3901_SQUAL_MIN_VALID) && (shutter_upper == 0x1FU));

    return (dev->data.motion_valid && dev->data.surface_valid) ? 1U : 0U;
}


/*********************************************************************
 * @fn      PMW3901_ClearMotion
 *
 * @param   *dev: device struct
 *
 * @brief   Clear latest motion data
 *
 * @return  None
 */
void PMW3901_ClearMotion(s_PMW3901 *dev)
{
    if (dev == NULL)
    {
        return;
    }

    dev->raw_data.motion = 0U;

    dev->raw_data.delta_x = 0;
    dev->raw_data.delta_y = 0;

    dev->raw_data.squal = 0U;
    dev->raw_data.raw_data_sum = 0U;
    dev->raw_data.max_raw_data = 0U;
    dev->raw_data.min_raw_data = 0U;

    dev->raw_data.shutter = 0U;

    dev->data.motion_valid = 0U;
    dev->data.surface_valid = 0U;
}



/*********************************************************************
 * @fn      PMW3901_GetSQUAL
 *
 * @param   *dev: device struct
 *
 * @brief   Return current surface quality
 *
 * @return  Measurements in struct
 */
uint8_t PMW3901_GetSQUAL(s_PMW3901 *dev)
{
    if (dev == NULL)
    {
        return 0U;
    }

    return dev->raw_data.squal;
}



/*********************************************************************
 * @fn      PMW3901_GetShutter
 *
 * @param   *dev: device struct
 *
 * @brief   Return current shutter value
 *
 * @return  Measurements in struct
 */
uint16_t PMW3901_GetShutter(s_PMW3901 *dev)
{
    if (dev == NULL)
    {
        return 0U;
    }

    return dev->raw_data.shutter;
}



/*********************************************************************
 * @fn      PMW3901_AccumulatePosition
 *
 * @param   *dev: device struct
 *
 * @brief   Integrate optical flow position
 *
 * @return  None
 */
void PMW3901_AccumulatePosition(s_PMW3901 *dev)
{
    if (dev == NULL)
    {
        return;
    }

    if (PMW3901_MotionValid(dev))
    {
        dev->data.position_x += dev->raw_data.delta_x;
        dev->data.position_y += dev->raw_data.delta_y;

        dev->data.filtered_position_x = dev->data.position_x;
        dev->data.filtered_position_y = dev->data.position_y;

        dev->motion_read_counter++;
    }
    else
    {
        dev->motion_rejected_counter++;
    }
}



/*********************************************************************
 * @fn      PMW3901_Update
 *
 * @param   *dev: device struct
 *
 * @brief   Complete PMW3901 processing cycle
 *
 * @return  None
 */
void PMW3901_Update(s_PMW3901 *dev)
{
    if (dev == NULL)
    {
        return;
    }

    // Read sensor frame
    if (PMW3901_ReadMotionBurst(dev) != PMW3901_OK)
    {
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    float motion_dt = PMW3901_DT;

    // delta is for interval since previous burst read even if frame was rejected
    if (dev->last_motion_read_ms != 0U)
    {
        uint32_t elapsed_ms = (uint32_t)(now_ms - dev->last_motion_read_ms);
        if (elapsed_ms > 0U)
        {
            motion_dt = (float)elapsed_ms * 0.001f;
        }
        if (motion_dt > PMW3901_MAX_VALID_DT_S) motion_dt = PMW3901_MAX_VALID_DT_S;
    }
    dev->last_motion_read_ms = now_ms;

    // Use only motion frames with sufficient surface quality.
    if (!PMW3901_MotionValid(dev))
    {
        if (dev->last_valid_motion_ms != 0U &&
            (uint32_t)(now_ms - dev->last_valid_motion_ms) >= PMW3901_VELOCITY_TIMEOUT_MS)
        {
            dev->measurements.velocity_x_mps = 0.0f;
            dev->measurements.velocity_y_mps = 0.0f;
            dev->last_valid_motion_ms = 0U;
        }

        dev->motion_rejected_counter++;
        return;
    }

    dev->last_valid_motion_ms = now_ms;

    /* Flow calculation based on height */
    // Sensor flow and gyro rotation are angular displacements over one sample [rad]
    // Sensor axes are swapped relative to the drone body axes
    dev->measurements.flow_x = (float)dev->raw_data.delta_y * dev->measurements.flow_scale;
    dev->measurements.flow_y = (float)dev->raw_data.delta_x * dev->measurements.flow_scale;

    float rot_x = dev->data.gyro_y_rad_s * motion_dt;
    float rot_y = dev->data.gyro_x_rad_s * motion_dt;

    // Remove rotational image motion before converting angular flow to translation
    // PMW flow axes are mapped to positive drone translation, body rotation is inverted
    dev->measurements.flow_x_corrected = dev->measurements.flow_x + rot_x;
    dev->measurements.flow_y_corrected = dev->measurements.flow_y + rot_y;

    float displacement_x_body_m = dev->measurements.flow_x_corrected * dev->data.altitude_m;
    float displacement_y_body_m = dev->measurements.flow_y_corrected * dev->data.altitude_m;

    // Rotate body-frame optical flow displacement into the world frame used by NMPC.
    float cos_yaw = cosf(dev->data.yaw_rad);
    float sin_yaw = sinf(dev->data.yaw_rad);
    float displacement_x_world_m = cos_yaw * displacement_x_body_m - sin_yaw * displacement_y_body_m;
    float displacement_y_world_m = sin_yaw * displacement_x_body_m + cos_yaw * displacement_y_body_m;

    float velocity_x_mps = displacement_x_world_m / motion_dt;
    float velocity_y_mps = displacement_y_world_m / motion_dt;
    float velocity_alpha = 1.0f - expf(-motion_dt / PMW3901_VELOCITY_LPF_TAU_S);

    dev->measurements.velocity_x_mps += velocity_alpha * (velocity_x_mps - dev->measurements.velocity_x_mps);
    dev->measurements.velocity_y_mps += velocity_alpha * (velocity_y_mps - dev->measurements.velocity_y_mps);

    dev->measurements.position_x_m += displacement_x_world_m;
    dev->measurements.position_y_m += displacement_y_world_m;
    dev->motion_read_counter++;
}
