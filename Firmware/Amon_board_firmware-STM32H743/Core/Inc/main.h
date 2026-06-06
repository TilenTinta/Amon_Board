/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "BME280.h"
#include "PWM.h"
#include "NRF24L01.h"
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include "HMC5883L.h"
#include "PMW3901.h"

#include "drone_data.h"
#include "data_transcode.h"
#include "filters.h"
#include "logging.h"
#include "autopilot.h"
#include "NMPC.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SCK_F_Pin GPIO_PIN_2
#define SCK_F_GPIO_Port GPIOE
#define CS_F_Pin GPIO_PIN_3
#define CS_F_GPIO_Port GPIOE
#define MISO_F_Pin GPIO_PIN_5
#define MISO_F_GPIO_Port GPIOE
#define MOSI_F_Pin GPIO_PIN_6
#define MOSI_F_GPIO_Port GPIOE
#define ADC1_10_EDF_Pin GPIO_PIN_0
#define ADC1_10_EDF_GPIO_Port GPIOC
#define ADC1_11_MAIN_Pin GPIO_PIN_1
#define ADC1_11_MAIN_GPIO_Port GPIOC
#define SERVO_XP_Pin GPIO_PIN_0
#define SERVO_XP_GPIO_Port GPIOA
#define SERVO_XN_Pin GPIO_PIN_1
#define SERVO_XN_GPIO_Port GPIOA
#define SERVO_YP_Pin GPIO_PIN_2
#define SERVO_YP_GPIO_Port GPIOA
#define SERVO_YN_Pin GPIO_PIN_3
#define SERVO_YN_GPIO_Port GPIOA
#define ADC1_18_BUCK_5V_Pin GPIO_PIN_4
#define ADC1_18_BUCK_5V_GPIO_Port GPIOA
#define EN_BUCK_5V_Pin GPIO_PIN_5
#define EN_BUCK_5V_GPIO_Port GPIOA
#define ADC1_3_BUCK_7V2_Pin GPIO_PIN_6
#define ADC1_3_BUCK_7V2_GPIO_Port GPIOA
#define EN_BUCK_7V2_Pin GPIO_PIN_7
#define EN_BUCK_7V2_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_4
#define LED_RED_GPIO_Port GPIOC
#define LED_WHITE_Pin GPIO_PIN_5
#define LED_WHITE_GPIO_Port GPIOC
#define AUX_PORT_Pin GPIO_PIN_0
#define AUX_PORT_GPIO_Port GPIOB
#define EDF_Pin GPIO_PIN_1
#define EDF_GPIO_Port GPIOB
#define CS_OF_Pin GPIO_PIN_11
#define CS_OF_GPIO_Port GPIOE
#define OF_RST_Pin GPIO_PIN_12
#define OF_RST_GPIO_Port GPIOE
#define OF_MOT_Pin GPIO_PIN_13
#define OF_MOT_GPIO_Port GPIOE
#define OF_MOT_EXTI_IRQn EXTI15_10_IRQn
#define CS_SD_Pin GPIO_PIN_14
#define CS_SD_GPIO_Port GPIOE
#define IRQ_DRDY_Pin GPIO_PIN_15
#define IRQ_DRDY_GPIO_Port GPIOE
#define I2C_SCL_S_Pin GPIO_PIN_10
#define I2C_SCL_S_GPIO_Port GPIOB
#define I2C_SDA_S_Pin GPIO_PIN_11
#define I2C_SDA_S_GPIO_Port GPIOB
#define UART5_RX_USB_Pin GPIO_PIN_12
#define UART5_RX_USB_GPIO_Port GPIOB
#define UART5_TX_USB_Pin GPIO_PIN_13
#define UART5_TX_USB_GPIO_Port GPIOB
#define UART1_TX_GPS_Pin GPIO_PIN_14
#define UART1_TX_GPS_GPIO_Port GPIOB
#define UART1_RX_GPS_Pin GPIO_PIN_15
#define UART1_RX_GPS_GPIO_Port GPIOB
#define I2C_SCL_EXT_Pin GPIO_PIN_12
#define I2C_SCL_EXT_GPIO_Port GPIOD
#define I2C_SDA_EXT_Pin GPIO_PIN_13
#define I2C_SDA_EXT_GPIO_Port GPIOD
#define RF1_IRQ_Pin GPIO_PIN_12
#define RF1_IRQ_GPIO_Port GPIOA
#define RF1_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define SPI3_SCK_RF_Pin GPIO_PIN_10
#define SPI3_SCK_RF_GPIO_Port GPIOC
#define SPI3_MISO_RF_Pin GPIO_PIN_11
#define SPI3_MISO_RF_GPIO_Port GPIOC
#define SPI3_MOSI_RF_Pin GPIO_PIN_12
#define SPI3_MOSI_RF_GPIO_Port GPIOC
#define RF1_CSN_Pin GPIO_PIN_0
#define RF1_CSN_GPIO_Port GPIOD
#define RF1_CE_Pin GPIO_PIN_1
#define RF1_CE_GPIO_Port GPIOD
#define RF2_IRQ_Pin GPIO_PIN_5
#define RF2_IRQ_GPIO_Port GPIOD
#define RF2_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define RF2_CSN_Pin GPIO_PIN_6
#define RF2_CSN_GPIO_Port GPIOD
#define RF2_CE_Pin GPIO_PIN_7
#define RF2_CE_GPIO_Port GPIOD
#define RGB_R_Pin GPIO_PIN_6
#define RGB_R_GPIO_Port GPIOB
#define RGB_G_Pin GPIO_PIN_7
#define RGB_G_GPIO_Port GPIOB
#define RGB_B_Pin GPIO_PIN_8
#define RGB_B_GPIO_Port GPIOB
#define LED_BRD_Pin GPIO_PIN_9
#define LED_BRD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
