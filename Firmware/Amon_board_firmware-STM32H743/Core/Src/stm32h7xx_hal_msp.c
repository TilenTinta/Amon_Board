/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                                            /**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief ADC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hadc->Instance==ADC1)
  {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M = 32;
    PeriphClkInitStruct.PLL2.PLL2N = 129;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_INP10
    PC1     ------> ADC1_INP11
    PA4     ------> ADC1_INP18
    PA6     ------> ADC1_INP3
    */
    GPIO_InitStruct.Pin = ADC1_10_EDF_Pin|ADC1_11_MAIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC1_18_BUCK_5V_Pin|ADC1_3_BUCK_7V2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */

  }

}

/**
  * @brief ADC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_INP10
    PC1     ------> ADC1_INP11
    PA4     ------> ADC1_INP18
    PA6     ------> ADC1_INP3
    */
    HAL_GPIO_DeInit(GPIOC, ADC1_10_EDF_Pin|ADC1_11_MAIN_Pin);

    HAL_GPIO_DeInit(GPIOA, ADC1_18_BUCK_5V_Pin|ADC1_3_BUCK_7V2_Pin);

    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  }

}

/**
  * @brief I2C MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hi2c->Instance==I2C2)
  {
    /* USER CODE BEGIN I2C2_MspInit 0 */

    /* USER CODE END I2C2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = I2C_SCL_S_Pin|I2C_SDA_S_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
    /* USER CODE BEGIN I2C2_MspInit 1 */

    /* USER CODE END I2C2_MspInit 1 */
  }
  else if(hi2c->Instance==I2C4)
  {
    /* USER CODE BEGIN I2C4_MspInit 0 */

    /* USER CODE END I2C4_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
    PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**I2C4 GPIO Configuration
    PD12     ------> I2C4_SCL
    PD13     ------> I2C4_SDA
    */
    GPIO_InitStruct.Pin = I2C_SCL_EXT_Pin|I2C_SDA_EXT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C4_CLK_ENABLE();
    /* USER CODE BEGIN I2C4_MspInit 1 */

    /* USER CODE END I2C4_MspInit 1 */
  }

}

/**
  * @brief I2C MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C2)
  {
    /* USER CODE BEGIN I2C2_MspDeInit 0 */

    /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(I2C_SCL_S_GPIO_Port, I2C_SCL_S_Pin);

    HAL_GPIO_DeInit(I2C_SDA_S_GPIO_Port, I2C_SDA_S_Pin);

    /* USER CODE BEGIN I2C2_MspDeInit 1 */

    /* USER CODE END I2C2_MspDeInit 1 */
  }
  else if(hi2c->Instance==I2C4)
  {
    /* USER CODE BEGIN I2C4_MspDeInit 0 */

    /* USER CODE END I2C4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C4_CLK_DISABLE();

    /**I2C4 GPIO Configuration
    PD12     ------> I2C4_SCL
    PD13     ------> I2C4_SDA
    */
    HAL_GPIO_DeInit(I2C_SCL_EXT_GPIO_Port, I2C_SCL_EXT_Pin);

    HAL_GPIO_DeInit(I2C_SDA_EXT_GPIO_Port, I2C_SDA_EXT_Pin);

    /* USER CODE BEGIN I2C4_MspDeInit 1 */

    /* USER CODE END I2C4_MspDeInit 1 */
  }

}

/**
  * @brief RTC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hrtc->Instance==RTC)
  {
    /* USER CODE BEGIN RTC_MspInit 0 */

    /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();
    /* USER CODE BEGIN RTC_MspInit 1 */

    /* USER CODE END RTC_MspInit 1 */

  }

}

/**
  * @brief RTC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    /* USER CODE BEGIN RTC_MspDeInit 0 */

    /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
    /* USER CODE BEGIN RTC_MspDeInit 1 */

    /* USER CODE END RTC_MspDeInit 1 */
  }

}

/**
  * @brief SPI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PC2_C     ------> SPI2_MISO
    PC3_C     ------> SPI2_MOSI
    PA9     ------> SPI2_SCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI2_MspInit 1 */

    /* USER CODE END SPI2_MspInit 1 */
  }
  else if(hspi->Instance==SPI3)
  {
    /* USER CODE BEGIN SPI3_MspInit 0 */

    /* USER CODE END SPI3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = SPI3_SCK_RF_Pin|SPI3_MISO_RF_Pin|SPI3_MOSI_RF_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI3_MspInit 1 */

    /* USER CODE END SPI3_MspInit 1 */
  }
  else if(hspi->Instance==SPI4)
  {
    /* USER CODE BEGIN SPI4_MspInit 0 */

    /* USER CODE END SPI4_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
    PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI4_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SPI4 GPIO Configuration
    PE2     ------> SPI4_SCK
    PE5     ------> SPI4_MISO
    PE6     ------> SPI4_MOSI
    */
    GPIO_InitStruct.Pin = SPI4_SCK_F_Pin|SPI4_MISO_F_Pin|SPI4_MOSI_F_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI4_MspInit 1 */

    /* USER CODE END SPI4_MspInit 1 */
  }

}

/**
  * @brief SPI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspDeInit 0 */

    /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PC2_C     ------> SPI2_MISO
    PC3_C     ------> SPI2_MOSI
    PA9     ------> SPI2_SCK
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

    /* USER CODE BEGIN SPI2_MspDeInit 1 */

    /* USER CODE END SPI2_MspDeInit 1 */
  }
  else if(hspi->Instance==SPI3)
  {
    /* USER CODE BEGIN SPI3_MspDeInit 0 */

    /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOC, SPI3_SCK_RF_Pin|SPI3_MISO_RF_Pin|SPI3_MOSI_RF_Pin);

    /* USER CODE BEGIN SPI3_MspDeInit 1 */

    /* USER CODE END SPI3_MspDeInit 1 */
  }
  else if(hspi->Instance==SPI4)
  {
    /* USER CODE BEGIN SPI4_MspDeInit 0 */

    /* USER CODE END SPI4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI4_CLK_DISABLE();

    /**SPI4 GPIO Configuration
    PE2     ------> SPI4_SCK
    PE5     ------> SPI4_MISO
    PE6     ------> SPI4_MOSI
    */
    HAL_GPIO_DeInit(GPIOE, SPI4_SCK_F_Pin|SPI4_MISO_F_Pin|SPI4_MOSI_F_Pin);

    /* USER CODE BEGIN SPI4_MspDeInit 1 */

    /* USER CODE END SPI4_MspDeInit 1 */
  }

}

/**
  * @brief TIM_Base MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */

  }

}

/**
  * @brief TIM_PWM MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_pwm: TIM_PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspInit 0 */

    /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* USER CODE BEGIN TIM3_MspInit 1 */

    /* USER CODE END TIM3_MspInit 1 */
  }
  else if(htim_pwm->Instance==TIM4)
  {
    /* USER CODE BEGIN TIM4_MspInit 0 */

    /* USER CODE END TIM4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* USER CODE BEGIN TIM4_MspInit 1 */

    /* USER CODE END TIM4_MspInit 1 */
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM2)
  {
    /* USER CODE BEGIN TIM2_MspPostInit 0 */

    /* USER CODE END TIM2_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    PA2     ------> TIM2_CH3
    PA3     ------> TIM2_CH4
    */
    GPIO_InitStruct.Pin = TIM2_CH1_XP_Pin|TIM2_CH2_XN_Pin|TIM2_CH3_YP_Pin|TIM2_CH4_YN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM2_MspPostInit 1 */

    /* USER CODE END TIM2_MspPostInit 1 */
  }
  else if(htim->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspPostInit 0 */

    /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB1     ------> TIM3_CH4
    */
    GPIO_InitStruct.Pin = TIM3_CH4_EDF_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(TIM3_CH4_EDF_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM3_MspPostInit 1 */

    /* USER CODE END TIM3_MspPostInit 1 */
  }
  else if(htim->Instance==TIM4)
  {
    /* USER CODE BEGIN TIM4_MspPostInit 0 */

    /* USER CODE END TIM4_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    PB8     ------> TIM4_CH3
    */
    GPIO_InitStruct.Pin = TIM4_CH1_RGB_R_Pin|TIM4_CH2_RGB_G_Pin|TIM4_CH3_RGB_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM4_MspPostInit 1 */

    /* USER CODE END TIM4_MspPostInit 1 */
  }

}
/**
  * @brief TIM_Base MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
    /* USER CODE BEGIN TIM2_MspDeInit 0 */

    /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
    /* USER CODE BEGIN TIM2_MspDeInit 1 */

    /* USER CODE END TIM2_MspDeInit 1 */
  }

}

/**
  * @brief TIM_PWM MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param htim_pwm: TIM_PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspDeInit 0 */

    /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
    /* USER CODE BEGIN TIM3_MspDeInit 1 */

    /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(htim_pwm->Instance==TIM4)
  {
    /* USER CODE BEGIN TIM4_MspDeInit 0 */

    /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();
    /* USER CODE BEGIN TIM4_MspDeInit 1 */

    /* USER CODE END TIM4_MspDeInit 1 */
  }

}

/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(huart->Instance==UART5)
  {
    /* USER CODE BEGIN UART5_MspInit 0 */

    /* USER CODE END UART5_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PB12     ------> UART5_RX
    PB13     ------> UART5_TX
    */
    GPIO_InitStruct.Pin = UART5_RX_USB_Pin|UART5_TX_USB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_UART5;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN UART5_MspInit 1 */

    /* USER CODE END UART5_MspInit 1 */
  }
  else if(huart->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspInit 0 */

    /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB14     ------> USART1_TX
    PB15     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = UART1_TX_GPS_Pin|UART1_RX_GPS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN USART1_MspInit 1 */

    /* USER CODE END USART1_MspInit 1 */
  }

}

/**
  * @brief UART MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==UART5)
  {
    /* USER CODE BEGIN UART5_MspDeInit 0 */

    /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PB12     ------> UART5_RX
    PB13     ------> UART5_TX
    */
    HAL_GPIO_DeInit(GPIOB, UART5_RX_USB_Pin|UART5_TX_USB_Pin);

    /* USER CODE BEGIN UART5_MspDeInit 1 */

    /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(huart->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspDeInit 0 */

    /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB14     ------> USART1_TX
    PB15     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, UART1_TX_GPS_Pin|UART1_RX_GPS_Pin);

    /* USER CODE BEGIN USART1_MspDeInit 1 */

    /* USER CODE END USART1_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
