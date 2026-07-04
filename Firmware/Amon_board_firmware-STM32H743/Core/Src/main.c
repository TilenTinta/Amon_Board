/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RGB_MAX 20000    // PWM max
#define RGB_DIM 4000     // Low brightness
#define RGB_MED 12000    // Medium brightness
#define RGB_BRIGHT 20000 // High brightness
#define STEP    1        // speed of transition

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Functions init */
uint16_t ADC_Read_Main_Battery();
uint16_t ADC_Read_EDF_Battery();
uint16_t ADC_Read_5V_Buck();
uint16_t ADC_Read_7V2_Buck();
uint8_t TVCServoEnable();
uint8_t TVCServoDisable();
void servoTest();
uint8_t EDFEnable();
uint8_t EDFDisable();
uint8_t enable_5v_buck(uint8_t pinState);
uint8_t enable_7v2_buck(uint8_t pinState);
void StatusLED(uint8_t Status);



/* Variables */
volatile uint8_t DroneStatusLocal = 0;
volatile uint8_t DroneStatusOld = 20;
volatile uint8_t StartupInit = 0;
volatile uint8_t armCheck = 0;

uint8_t LED_blink_cnt_ON;
uint8_t LED_blink_cnt_double;
uint8_t LED_blink_cnt_OFF;

volatile uint8_t RF_IRQ1_EN = 0;			// flag to enable irq code
volatile uint8_t RF_IRQ2_EN = 0;			// flag to enable irq code

// Gyroscope calibration
uint8_t  GyroCalibTrig = 0;
uint16_t RGB_Red = 0;
uint16_t RGB_Green = 300;
uint16_t RGB_Blue = 600;
uint16_t RGB_RedMax = 0;
uint16_t RGB_GreenMax = 0;
uint16_t RGB_BlueMax = 0;

uint8_t timer_divider_200to100 = 0;		// Test to convert 100Hz timer to 50Hz

// Battery data
volatile uint8_t ADC_DMA_DataRdy = 0;
volatile uint8_t adc_avg_initialized = 0;
__attribute__((section(".dma_buffer")))
__attribute__((aligned(32)))
volatile uint16_t adc_raw[4];

// GPS data
//uint8_t  USART1_GPSRX_DMA[426] = {"\0"};
__attribute__((section(".dma_buffer")))
__attribute__((aligned(32)))
uint8_t  USART1_GPSRX_DMA[512] = {"\0"};
volatile uint8_t NewGPSData = 0;

// Timers IRQ enable flag
volatile uint8_t IRQ_1HzLoopEN = 0;
volatile uint8_t IRQ_50HzLoopEN = 0;
volatile uint8_t IRQ_200HzLoopEN = 0;
volatile uint8_t IRQ_NMPCLoopEN = 0;



/* STRUCTS */
s_MPU6050 			mpu6050; 		// MPU6050 device driver
s_BME280 			bme280;			// bme280 device driver
VL53L1_DEV 			vl53l1Dev;		// VL53L1 device driver
s_HMC5883L			hmc5883l;		// HMC5883L device driver
s_PMW3901 			pmw3901;		// PMW3901 device driver
//VL53L1X_Version_t vl53l1xVersion_t; // Lidar - unused
//VL53L1X_Result_t vl53l1xResult_t;   // Lidar - unused
s_nRF24L01 			radio1;			// nRF24L01 device driver
s_nRF24L01 			radio2;			// nRF24L01 device driver
s_drone_data 		AmonDrone; 		// All drone data
s_packets 			data_packets;	// Data structure for communication packets

s_Kalman			kalman_pitch;	// Kalman values for pitch
s_Kalman			kalman_roll;	// Kalman values for roll
s_Kalman			kalman_yaw;		// Kalman values for yaw
s_KalmanZ			kalman_tof;		// Kalman values for height
s_Quaternion		quaternion;		// Quaternion based on Euler angles
s_NMPC				nmpc = {0};		// NMPC regulator

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Init
    AmonDrone.DroneStatus = STATUS_STARTUP;
    AmonDrone.flight_status = STATUS_FLIGHT_GROUND;

    // Radio 1 configurations (application specific)
    static const s_nrf_config radio_tx_normal_cfg = {
        .channel = 100,	//42
        .addr_width = AW_5BYTE,
	#ifdef RADIO_HW_ACK
        .auto_ack = 1,
	#else
		.auto_ack = 0,
	#endif
        .dynamic_payload = 1,
        .retries = 15,
        .retry_delay = ARD_3500us, //ARD_1250us
        .datarate = NRF_DATARATE_1MBPS,
        .power = NRF_POWER_0DBM,
    };

    static const s_nrf_config radio_tx_stream_cfg = {
        .channel = 100,	//42
        .addr_width = AW_5BYTE,
        .auto_ack = 0,
        .dynamic_payload = 1,
        .retries = 15,
        .retry_delay = ARD_1250us, //ARD_1250us
        .datarate = NRF_DATARATE_1MBPS,
        .power = NRF_POWER_0DBM,
    };

    static const s_pipe_addr radio_tx_addr = {
        //.tx_addr = { 0xE7, 0xE7, 0xE7, 0xE7, 0xEE } //E8
        .tx_addr = { 0xC2, 0xA1, 0x55, 0x12, 0x01 }
    };

    // Radio 1 configurations (application specific)
    static const s_nrf_config radio_rx_normal_cfg = {
        .channel = 70,
        .addr_width = AW_5BYTE,
	#ifdef RADIO_HW_ACK
        .auto_ack = 1,
	#else
		.auto_ack = 0,
	#endif
		.dynamic_payload = 1,
        .retries = 0,
        .retry_delay = 0,
        .datarate = NRF_DATARATE_1MBPS,
        .power = NRF_POWER_MINUS_6DBM,
    };

    static const s_nrf_config radio_rx_stream_cfg = {
        .channel = 70,
        .addr_width = AW_5BYTE,
        .auto_ack = 0,
        .dynamic_payload = 1,
        .retries = 0,
        .retry_delay = 0,
        .datarate = NRF_DATARATE_1MBPS,
        .power = NRF_POWER_MINUS_6DBM,
    };

    static const s_pipe_addr radio_rx_addr = {
        //.pipe0_rx_addr = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 }
        .pipe0_rx_addr = { 0xD4, 0xB2, 0xAA, 0x78, 0x50 }
    };

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*##### Changing drone status #####*/
	  #ifndef CALIBRATION
	  if (AmonDrone.DroneStatus != DroneStatusOld)
	  {
		  DroneStatusOld = AmonDrone.DroneStatus;
		  DroneStatusLocal = AmonDrone.DroneStatus;
		  StatusLED(AmonDrone.DroneStatus);
	  }
	  #endif

	  	  /*##### RGB cycling when calibrating #####*/
	  #ifdef CALIBRATION
		  AmonDrone.DroneStatus = STATUS_CALIB;
		  DroneStatusLocal = AmonDrone.DroneStatus;
	  #endif




	  /*#############################################################################################################################################
	  #################################################################### RADIO ####################################################################
	  #############################################################################################################################################*/

	  	  /*##### RADIO IRQ HANDLING #####*/
	  	  if (radio1.irq_flag == 1)
	  	  {
			  radio1.irq_flag = 0;

	  		  // TX
	  		  NRF24_HandleIRQ(&radio1);
	  		  radio1.buffers.pipe_data = radio1.irq_on_pipe;
	  		  radio1.irq_on_pipe = 0xFF;
	  		  if (radio1.role == NRF_ROLE_PRX) AmonDrone.radio_data.connection_timeout = 0;	// Reset timeout timer
	  	  }

	  	  if (radio2.irq_flag == 1)
	  	  {
			  radio2.irq_flag = 0;
			  HAL_GPIO_WritePin(LED_BRD_GPIO_Port, LED_BRD_Pin, GPIO_PIN_SET);

	  		  // RX
	  		  NRF24_HandleIRQ(&radio2);
	  		  radio2.buffers.pipe_data = radio2.irq_on_pipe;
	  		  radio2.irq_on_pipe = 0xFF;
	  		  if (radio2.role == NRF_ROLE_PRX) AmonDrone.radio_data.connection_timeout = 0;	// Reset timeout timer
	  	  }

	  	  // Radio watchdog
	  	  NRF24_PollWatchdog(&radio1);
	  	  NRF24_PollWatchdog(&radio2);

	  	  /*##### RADIO - transmit report #####*/
	  	  if (radio1.buffers.flag_tx_done)
	  	  {
	  		  // success
	  		  AmonDrone.radio_data.packet_tx_cnt++;
	  		  radio1.buffers.flag_tx_done = 0;
	  	  }

	  	  if (radio1.buffers.flag_max_rxs_reached)
	  	  {
	  		  // fail
	  		  AmonDrone.radio_data.packet_fail_cnt++;
	  		  radio1.buffers.flag_max_rxs_reached = 0;

	  		  // connection lost detection
	  //		  if (AmonDrone.radio_data.packet_fail_cnt >= 5)
	  //		  {
	  //			  AmonDrone.radio_data.flag_connection_lost = 1;
	  //			  AmonDrone.radio_data.packet_fail_cnt = 0;
	  //			  AmonDrone.radio_data.conn_status = CONN_STATUS_DISCONNECTED;
	  //		  }
	  	  }


	  	  /*##### RADIO - Data received -> read #####*/
	  	  if (radio2.buffers.flag_new_rx)
	  	  {
	  		  NRF24_ReadRXPayload(&radio2);
	  		  AmonDrone.radio_data.flag_new_rf_rx_data = 1;
	  	  }

	  	  /*##### RADIO - Decode, encode, send #####*/
	  	  if (AmonDrone.DroneStatus != STATUS_STARTUP || AmonDrone.DroneStatus != STATUS_ERROR) // &&
	  	  {
	  		  // RX packets
	  		  if (AmonDrone.radio_data.flag_new_rf_rx_data == 1)
	  		  {
	  			  AmonDrone.radio_data.flag_new_rf_rx_data = 0;
	  			  uint8_t ret = RF_decode(radio2.buffers.RX_FIFO, &data_packets, NULL);

	  			  if (ret == TRANSCODE_OK || ret == TRANSCODE_BROADCAST)
	  			  {
	  				  RF_packet_decode(&data_packets, &AmonDrone);
	  			  }

	  			  // ACK required - better link connection
	  			  if (AmonDrone.radio_data.flag_stream_data == 1)
	  			  {
	  				  AmonDrone.radio_data.flag_stream_data = 0;
					#ifdef RADIO_HW_ACK
	  				  radio1.config = &radio_tx_normal_cfg;
	  				  NRF24_init(&radio1);
	  				  HAL_Delay(10);
					#endif
	  			  }
	  		  }


	  		  // TX packets - received and requires ACK
	  		  if (AmonDrone.radio_data.flag_new_rf_tx_data == 1)
	  		  {
	  			  AmonDrone.radio_data.flag_new_rf_tx_data = 0;
	  			  memset(radio1.buffers.TX_FIFO, 0, sizeof(radio1.buffers.TX_FIFO));
	  			  RF_encode(&data_packets, radio1.buffers.TX_FIFO, &radio1.buffers.tx_lenght);
	  			  NRF24_Send(&radio1);
	  		  }
	  	  }


	  	  /*##### RADIO - Send telemetry data or not #####*/
	  	  switch(AmonDrone.radio_data.conn_status) {

	  		// STATE: Drone is NOT connected with ground station
	  		case CONN_STATUS_DISCONNECTED:

	  			// ACK required - better link connection
	  			if (AmonDrone.radio_data.flag_stream_data == 1)
	  			{
	  				AmonDrone.radio_data.flag_stream_data = 0;
				#ifdef RADIO_HW_ACK
	  				radio1.config = &radio_tx_normal_cfg;
	  				NRF24_init(&radio1);
	  				HAL_Delay(2);
				#endif
	  			}

	  			// Detect successful connection
	  			if (AmonDrone.radio_data.flag_connection_begin == 1)
	  			{
	  				AmonDrone.radio_data.conn_status = CONN_STATUS_CONNECTED;
	  			}

	  			break;


	  		// STATE: Drone IS connected with ground station
	  		case CONN_STATUS_CONNECTED:

	  			// TX packets - telemetry send / stream
	  			if (AmonDrone.radio_data.flag_telemetry_send == 1 && AmonDrone.radio_data.flag_new_rf_tx_data == 0)
	  			{
	  				AmonDrone.radio_data.flag_telemetry_send = 0;
	  				packet_create_telemetry(&data_packets, &AmonDrone);
	  				memset(radio1.buffers.TX_FIFO, 0, sizeof(radio1.buffers.TX_FIFO));
	  				RF_encode(&data_packets, radio1.buffers.TX_FIFO, &radio1.buffers.tx_lenght);
	  				//radio1.flag_tx_in_progress = 0;
	  				NRF24_Send(&radio1);
	  				HAL_GPIO_TogglePin(LED_BRD_GPIO_Port, LED_BRD_Pin);
	  			}

	  			break;


	  		// STATE: Default state
	  		default:
	  			// Default
	  			break;
	  		}



	  /*##############################################################################################################################################
	  ##################################################################### UART #####################################################################
	  ##############################################################################################################################################*/

	  	  /* USB TX - TRANSMITING HANDLING */
	  	  if (AmonDrone.uart_buffer.flag_new_uart_tx_data == 1)
	  	  {
	  		  AmonDrone.uart_buffer.flag_new_uart_tx_data = 0;
	  		  packet_create_uart_data(&data_packets, &AmonDrone);
	  		  UART_encode(&data_packets, AmonDrone.uart_buffer.buffer_UART_TX);
	  		  HAL_UART_Transmit(&huart5, (uint8_t*)AmonDrone.uart_buffer.buffer_UART_TX, AmonDrone.uart_buffer.buffer_UART_TX[1] + 2, HAL_MAX_DELAY);
	  		  memset(AmonDrone.uart_buffer.buffer_UART_TX, 0, sizeof(AmonDrone.uart_buffer.buffer_UART_TX));
	  	  }

	  	  /* USB RX - TRANSCODING HANDLING */
	  	  if (AmonDrone.uart_buffer.flag_new_uart_rx_data == 1)
	  	  {
	  		  AmonDrone.uart_buffer.flag_new_uart_rx_data = 0;

	  		  uint8_t ret = UART_decode(AmonDrone.uart_buffer.buffer_UART_RX, &data_packets, &AmonDrone.uart_buffer.flag_new_uart_tx_data, &data_packets.calib_data);
	  		  memset(AmonDrone.uart_buffer.buffer_UART_RX, 0, sizeof(AmonDrone.uart_buffer.buffer_UART_RX));

	  		  // Based on return values trigger events
	  		  switch (ret)
	  		  {
	  			  case TRANSCODE_BOOT_PKT:
	  				  // Unused
	  				  break;

	  			  case TRANSCODE_BROADCAST:
	  				  // Unused
	  				  break;

	  			  case TRANSCODE_VER_ERR:                 	// Wrong version of packet
	  				  break;

	  			  case TRANSCODE_DEST_ERR:                	// Wrong destination address
	  				  break;

	  			  case TRANSCODE_CRC_ERR:                	// Corupted frame / CRC
	  				  break;

	  			  case TRANSCODE_DEST_RF:                 	// Packet for RF transmit
	  				  // Unused
	  				  break;

	  			  case TRANSCODE_DEST_LINK:               	// Packet for link device
	  				  // Unused
	  				  break;

	  			  case TRANSCODE_LOG_DUMP:					// Serial request for flight log dump
	  			  	  //AmonDrone.uart_buffer.flag_log_dump = 1;
	  				  (void)log_dump_uart(AmonDrone.uart_buffer.log_file, &huart5); // "log.txt"
	  			  	  break;

	  			  case TRANSCODE_LOG_RM:					// Serial request for flight log remove
	  				  //AmonDrone.uart_buffer.flag_log_remove = 1;
	  				  log_remove();
	  				  break;

	  			  case TRANSCODE_EN_IDENTI:					// Enable identification flag
	  				  AmonDrone.identifications.flag_test_identification = 1;
	  				  break;

	  			  case TRANSCODE_IDENTI_EDF:				// Enable edf identification flag
	  				  AmonDrone.identifications.flag_test_edf = 1;
	  				  break;

	  			  case TRANSCODE_IDENTI_SERVO:				// Enable servo identification flag
	  				  AmonDrone.identifications.flag_test_servo = 1;
	  				  break;

	  			  case TRANSCODE_IDENTI_MOMENT:				// Enable moment identification flag
	  				  AmonDrone.identifications.flag_test_moment = 1;
	  				  break;

	  			  case TRANSCODE_CAL_COMM:					// Serial calibration commands
	  				  // Full packet required (all fields must have value)
					#ifdef IDENTIFICATION
	  				  // Identification test - EDF and Servos - UART RX
	  				  if (AmonDrone.actuators.edf_enable == 0) EDFEnable();
	  				  PowerToPWMValue(data_packets.calib_data.edf_pwr_percent);
	  				  AmonDrone.actuators.edf_percent = data_packets.calib_data.edf_pwr_percent;

	  				  if (AmonDrone.actuators.servo_enable == 0)
					  {
						enable_7v2_buck(ENABLE);
						TVCServoEnable();
					  } 
	  				  DegresToCCR(data_packets.calib_data.x_plus_angle, SERVO_XP);
	  				  DegresToCCR(data_packets.calib_data.x_minus_angle, SERVO_XN);
	  				  DegresToCCR(data_packets.calib_data.y_plus_angle, SERVO_YP);
	  				  DegresToCCR(data_packets.calib_data.y_minus_angle, SERVO_YN);
					#endif
	  				  break;

	  			  default:                                	// Default state
	  				  break;
	  		  }
	  	  }



	  //#ifdef LOG_ENABLE
	  //	  if (AmonDrone.uart_buffer.flag_log_dump == 1)
	  //	  {
	  //		  AmonDrone.uart_buffer.flag_log_dump = 0;
	  //		  (void)log_dump_uart(AmonDrone.uart_buffer.log_file, &huart5); // "log.txt"
	  //	  }
	  //
	  //	  if (AmonDrone.uart_buffer.flag_log_remove == 1)
	  //	  {
	  //		  AmonDrone.uart_buffer.flag_log_remove = 0;
	  //		  log_remove();
	  //	  }
	  //#endif



	  /*##############################################################################################################################################
	  #################################################################### TIMERS ####################################################################
	  ##############################################################################################################################################*/


	  	  /*##### TIMERS - Timer flag triggers #####*/
	  	  if (AmonDrone.DroneStatus != STATUS_STARTUP ) // && AmonDrone.DroneStatus != STATUS_ERROR
	  	  {

	  		  // --- Timer 6 - 200Hz / 5ms ---
	  		  if (IRQ_200HzLoopEN == 1)
	  		  {
	  			  IRQ_200HzLoopEN = 0;

	  			  timer_divider_200to100++;

	  			  // Timer divider: 200Hz to 100Mz
				  if (timer_divider_200to100 >= 2)
				  {
					  timer_divider_200to100 = 0;

					  // Kalman filter for height - ToF //
					  KalmanZ_Predict(&kalman_tof, TIM_100HZ_DT);

					  if (AmonDrone.position.flag_new_ToF_data)
					  {
						  float z_meas = AmonDrone.position.height_TOF_mm / 1000.0f;
						  KalmanZ_Update(&kalman_tof, z_meas);
						  AmonDrone.position.flag_new_ToF_data = 0;
						  AmonDrone.position.height_TOF_mm_filtered = kalman_tof.z;    // position z [m]
						  AmonDrone.position.velocity_z = kalman_tof.vz;	// vz [m/s]
					  }

					  if (AmonDrone.identifications.flag_test_moment) AmonDrone.uart_buffer.flag_new_uart_tx_data = 1; // Enable data transition over UART for identification
				  }

	  			  // Read sensors //
	  			  MPU6050_ReadAllDirect(&mpu6050, &hi2c2);

	  			  /* Axis mapping: Sensor -> Drone frame
				   * Drone X+ = Right
				   * Drone Y+ = Forward
				   * Drone Z+ = Up
				   *
				   * body X =  sensor Y
				   * body Y = -sensor Z
				   * body Z =  sensor X
				   */
	  			  AmonDrone.position.gyroTemp = (uint16_t)mpu6050.Temp_C;

				  AmonDrone.position.accel_x =  mpu6050.ACCEL_Y;
				  AmonDrone.position.accel_y = -mpu6050.ACCEL_Z;
				  AmonDrone.position.accel_z =  mpu6050.ACCEL_X;

				  AmonDrone.position.gyro_x  =  mpu6050.GYRO_Y;
				  AmonDrone.position.gyro_y  = -mpu6050.GYRO_Z;
				  AmonDrone.position.gyro_z  =  mpu6050.GYRO_X;


	  			  // Gyro filters //
				#ifndef GYRO_KALMAN
	  			  // Complementary filter
	  			  Complementary_deg(&mpu6050, &AmonDrone); // calculate data to pitch and roll (and yaw)
				#else
	  			  // Kalman Filter - angles and axis described in filters.h
	  			  float roll_angle_accel  = 0;
	  			  float pitch_angle_accel = 0;

	  			  Kalman_rawToAngles(&AmonDrone, &roll_angle_accel, &pitch_angle_accel);
				  roll_angle_accel  = unwrap_to_ref(roll_angle_accel,  kalman_roll.angle);
				  pitch_angle_accel = unwrap_to_ref(pitch_angle_accel, kalman_pitch.angle);

	  			  AmonDrone.position.Pitch = Kalman_Update(&kalman_pitch, -AmonDrone.position.gyro_x, pitch_angle_accel, DT);
	  			  AmonDrone.position.Roll = Kalman_Update(&kalman_roll, -AmonDrone.position.gyro_y, roll_angle_accel, DT);


	  			  uint8_t mag_valid = hmc5883l.flag_new_data;
	  			  hmc5883l.flag_new_data = 0;

	  			  AmonDrone.position.Yaw = Kalman_UpdateYaw(&kalman_yaw, AmonDrone.position.gyro_z, AmonDrone.position.heading_deg, mag_valid, DT);

	  			  //quaternion = eulerToQuaternion(AmonDrone.position.Roll, AmonDrone.position.Pitch, AmonDrone.position.Yaw);
	  			  //gyroToQuaternion(&quaternion, AmonDrone.position.gyro_x, AmonDrone.position.gyro_y, AmonDrone.position.gyro_z, DT); // Check order of angles
	  			  EulerQuaternion_Complementary(&quaternion, AmonDrone.position.gyro_x, AmonDrone.position.gyro_y, AmonDrone.position.gyro_z, AmonDrone.position.Roll, AmonDrone.position.Pitch, AmonDrone.position.Yaw, DT, ALPHA_EQ);
	  			  AmonDrone.position.quaternion[0] = quaternion.w;
	  			  AmonDrone.position.quaternion[1] = quaternion.x;
	  			  AmonDrone.position.quaternion[2] = quaternion.y;
	  			  AmonDrone.position.quaternion[3] = quaternion.z;
				#endif

				#ifdef USE_OPTICAL_FLOW
	  			  // PMW3901 optical flow sensor //
	  			  pmw3901.data.altitude_m = AmonDrone.position.height_TOF_mm / 1000.0; // mm to m
	  			  pmw3901.data.gyro_x_rad_s = AmonDrone.position.gyro_x * DEG_TO_RAD;
	  			  pmw3901.data.gyro_y_rad_s = AmonDrone.position.gyro_y * DEG_TO_RAD;
	  			  pmw3901.data.gyro_z_rad_s = AmonDrone.position.gyro_z * DEG_TO_RAD;
	  			  PMW3901_Update(&pmw3901);
				#endif

	  			  AmonDrone.position.position_x = pmw3901.measurements.position_x_m;
	  			  AmonDrone.position.position_y = pmw3901.measurements.position_y_m;

	  		  } // TIMER 200Hz



	  		  // --- Timer 5 - 50Hz / 20ms ---
	  		  if (IRQ_50HzLoopEN == 1)
	  		  {
	  			  IRQ_50HzLoopEN = 0;


	  			  // Read sensors //

	  			  // VL53L1X
	  			  static uint8_t dataRdy = 0;
	  			  VL53L1X_CheckForDataReady(&vl53l1Dev, &hi2c2, &dataRdy);
	  			  if (dataRdy)
	  			  {
	  				  VL53L1X_GetDistance(&vl53l1Dev, &hi2c2, &AmonDrone.position.height_TOF_mm);
	  				  VL53L1X_ClearInterrupt(&vl53l1Dev, &hi2c2);
	  				  AmonDrone.position.flag_new_ToF_data = 1;
	  				  dataRdy = 0; // remove?
	  			  }

	  			  // BME280
	  			  BME280_ReadAllData(&bme280, &hi2c2);
	  			  BME280_PressToAlt(&bme280, &hi2c2);
	  			  AmonDrone.data.temperature = (uint16_t)bme280.Temp_C;
	  			  AmonDrone.data.humidity = (uint8_t)bme280.Hum_Perc;
	  			  AmonDrone.data.pressure = bme280.Press_Pa;
	  			  AmonDrone.position.height_baro_m = bme280.altitude_m;

	  			  // HMC5883L
	  			  /* Axis mapping: Sensor -> drone body
	  			   * body X/right   = -compass Y
	  			   * body Y/forward = -compass Z
	  			   * body Z/up      =  compass X
	  			   *
	  			   * */
	  			  uint8_t status = 0;
	  			  HMC5883L_ReadStatus(&hmc5883l, &hi2c2, &status);
	  			  if (hmc5883l.DataReady)
	  			  {
	  				  //HMC5883L_ReadHeading(&hmc5883l, &hi2c2, DECLINATION_DEG);
	  				  if (HMC5883L_ReadHeadingTiltCompensated(&hmc5883l, &hi2c2, DECLINATION_DEG, AmonDrone.position.Roll, AmonDrone.position.Pitch) == 0)
	  				  {
						  AmonDrone.position.heading_deg = hmc5883l.HeadingDeg - (AmonDrone.position.heading_offset_deg);
						  AmonDrone.position.x_gauss = -hmc5883l.Y_Gauss;
						  AmonDrone.position.y_gauss = -hmc5883l.Z_Gauss;
						  AmonDrone.position.z_gauss = hmc5883l.X_Gauss;
						  hmc5883l.flag_new_data = 1;

						  if(AmonDrone.position.flag_zero_compas == 1)
						  {
							  AmonDrone.position.heading_offset_deg = hmc5883l.HeadingDeg;
							  AmonDrone.position.flag_zero_compas = 0;
						  }
	  				  }
	  			  }


	  			  // Start ADC DMA (read analog value) //
	  			  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 4);


	  			  // Send telemetry packet if stream mode is on//
	  			  if (AmonDrone.radio_data.flag_stream_data)
	  			  {
	  				  AmonDrone.radio_data.flag_telemetry_send = 1;
	  			  }


	  			  // EDF ramp-up //
	  	  	  #if defined(EDF_RAMP_UP_EN) && !defined(IDENTIFICATION)
	  			  if (!AmonDrone.actuators.rampUpDone && AmonDrone.DroneStatus == STATUS_FLY)
	  			  {
	  				  EDFSlowRamp(&AmonDrone.actuators);
	  			  }
	  	  	  #endif


	  		  } // TIMER 50Hz



	  		  // --- Timer 7 - 1Hz / 1sec ---
	  		  if (IRQ_1HzLoopEN == 1)
	  		  {
	  			  IRQ_1HzLoopEN = 0;

	  			  // Radio timeout //
	  			  if (AmonDrone.radio_data.conn_status == CONN_STATUS_CONNECTED) AmonDrone.radio_data.connection_timeout++;

  //			  if (AmonDrone.radio_data.connection_timeout >= CONN_TIMEOUT_SEC)
  //			  {
  //				  AmonDrone.radio_data.conn_status = CONN_STATUS_DISCONNECTED;
  //				  AmonDrone.radio_data.flag_connection_lost = 1;
  //			  }


	  			  // Stream mode switch
	  			  if (AmonDrone.radio_data.conn_status == CONN_STATUS_CONNECTED && AmonDrone.radio_data.flag_stream_data == 0)
	  			  {
	  				  static uint8_t stream_time_cnt = 0;

	  				  if (stream_time_cnt < STREAM_EN_TIME)
	  				  {
	  					  stream_time_cnt++;
	  				  }
	  				  else
	  				  {
	  					  stream_time_cnt = 0;
	  					  AmonDrone.radio_data.flag_stream_data = 1;
						#ifdef RADIO_HW_ACK
	  					  // Turn ON stream data mode
	  					  radio1.config = &radio_tx_stream_cfg;
	  					  NRF24_init(&radio1);
	  					  HAL_Delay(10);
						#endif
	  				  }
	  			  }

	  			  // After hopefully successful landing turn off logging with delay
	  			  if (AmonDrone.uart_buffer.flag_logging_active == 1 && AmonDrone.DroneStatus == STATUS_FLY_OVER)
	  			  {
	  				  if (AmonDrone.uart_buffer.log_save_delay >= LOG_DELAY)
	  				  {
	  					  AmonDrone.uart_buffer.flag_logging_active = 0;
	  				  }
	  				  else
	  				  {
	  					AmonDrone.uart_buffer.log_save_delay++;
	  				  }
	  			  }

	  			  // After hopefully successful landing turn off edf motor with delay
				  if (AmonDrone.actuators.edf_enable == 1 && AmonDrone.DroneStatus == STATUS_FLY_OVER)
				  {
					  if (AmonDrone.actuators.edf_off_delay >= EDF_DELAY)
					  {
						  AmonDrone.actuators.edf_enable = 0;
					  }
					  else
					  {
						AmonDrone.actuators.edf_off_delay++;
					  }
				  }

  				  // Path planing - regulator reference planner //
  				  if (AmonDrone.DroneStatus == STATUS_FLY) AmonDrone.flight_path.flight_start_time += TIM_1HZ_DT;			// Full flight time in seconds

	  		  } // TIMER 1Hz



	  		  // --- ADC read - DMA trigger ---
	  		  if (ADC_DMA_DataRdy)
	  		  {

	  			  uint16_t main_now = ADC_Read_Main_Battery();
	  			  uint16_t edf_now = ADC_Read_EDF_Battery();
	  			  uint16_t buck_5v_now = ADC_Read_5V_Buck();
	  			  uint16_t buck_7v2_now = ADC_Read_7V2_Buck();

	  			  // At boot ADC check
	  			  if (!adc_avg_initialized)
	  			  {
	  				 for (uint8_t i = 0; i < 10; i++)
	  				 {
	  					 AmonDrone.data.bat_main_v[i] = main_now;
	  					 AmonDrone.data.bat_edf_v[i] = edf_now;
	  					 AmonDrone.data.buck_5V_v[i] = buck_5v_now;
	  					 AmonDrone.data.buck_7V2_v[i] = buck_7v2_now;
	  				 }

	  				 adc_avg_initialized = 1;
	  			  }
	  			  else
	  			  {
	  				  // At runtime ADC check
					  for (uint8_t i = 1; i < 10; i++)
					  {
						  AmonDrone.data.bat_main_v[i-1] = AmonDrone.data.bat_main_v[i];
						  AmonDrone.data.bat_edf_v[i-1] = AmonDrone.data.bat_edf_v[i];
						  AmonDrone.data.buck_5V_v[i-1] = AmonDrone.data.buck_5V_v[i];
						  AmonDrone.data.buck_7V2_v[i-1] = AmonDrone.data.buck_7V2_v[i];
					  }

					  AmonDrone.data.bat_main_v[9] = main_now;
					  AmonDrone.data.bat_edf_v[9] = edf_now;
					  AmonDrone.data.buck_5V_v[9] = buck_5v_now;
					  AmonDrone.data.buck_7V2_v[9] = buck_7v2_now;
				  }

	  			  // Average calculation
				  uint16_t main_v_temp = 0;
				  uint16_t edf_v_temp = 0;
				  uint16_t buck_5v_temp = 0;
				  uint16_t buck_7v2_temp = 0;
				  for (uint8_t i = 0; i < 10; i++)
				  {
					  main_v_temp += AmonDrone.data.bat_main_v[i];
					  edf_v_temp += AmonDrone.data.bat_edf_v[i];
					  buck_5v_temp += AmonDrone.data.buck_5V_v[i];
					  buck_7v2_temp += AmonDrone.data.buck_7V2_v[i];
				  }

				  AmonDrone.data.battery_main_voltage = main_v_temp / (sizeof(AmonDrone.data.bat_main_v) / sizeof(uint16_t));
				  AmonDrone.data.battery_edf_voltage = edf_v_temp / (sizeof(AmonDrone.data.bat_edf_v) / sizeof(uint16_t));
				  AmonDrone.data.buck_5v_voltage = buck_5v_temp / (sizeof(AmonDrone.data.buck_5V_v) / sizeof(uint16_t));
				  AmonDrone.data.buck_7v2_voltage = buck_7v2_temp / (sizeof(AmonDrone.data.buck_7V2_v) / sizeof(uint16_t));


				  if (AmonDrone.data.battery_main_voltage < 1050) // less than 10.5V
				  {
					  AmonDrone.error_code.err_main_bat = 1;
				  }
				  else
				  {
					  AmonDrone.error_code.err_main_bat = 0;
				  }

				  if (AmonDrone.data.battery_edf_voltage < 2100) // less than 21V
				  {
					  AmonDrone.error_code.err_edf_bat = 1;
				  }
				  else
				  {
					  AmonDrone.error_code.err_edf_bat = 0;
				  }

	  			  if (AmonDrone.data.buck_5v_enable)
	  			  {
					  if (AmonDrone.data.buck_5v_voltage < 400) // less than 4V
					  {
						  AmonDrone.error_code.err_buck_5v = 1;
					  }
					  else
					  {
						  AmonDrone.error_code.err_buck_5v = 0;
					  }
	  			  }
	  			  else
				  {
					  AmonDrone.error_code.err_buck_5v = 0;
				  }

	  			  if (AmonDrone.data.buck_7v2_enable)
	  			  {
	  				  if (AmonDrone.data.buck_7v2_voltage < 500) // less than 5V
	  				  {
	  					  AmonDrone.error_code.err_buck_7v2 = 1;
	  				  }
	  				  else
	  				  {
	  					  AmonDrone.error_code.err_buck_7v2 = 0;
	  				  }
	  			  }
	  			  else
				  {
					  AmonDrone.error_code.err_buck_7v2 = 0;
				  }

	  			  ADC_DMA_DataRdy = 0;
	  		  }



	  		  // --- Timer 1 - 50Hz / 20ms (match acados/casadi model) ---
	  		  if (IRQ_NMPCLoopEN == 1)
	  		  {
				IRQ_NMPCLoopEN = 0;

				// =========================== NMPC ===========================
				if (AmonDrone.data.NMPC_enable)
				{
					// --------------------------------------------------------------------------------------------------------
					// Refresh all values in state vector //
					static double x_current[NMPC_NX] = {0}; 	// Reset everything

					// --- Position [0:3] ---
					x_current[0] = 0.0f;                                       // pos x (GPS/optical flow)
					x_current[1] = 0.0f;                                       // pos y (GPS/optical flow)
//	  				x_current[0] = AmonDrone.position.position_x;              // [m]
//	  				x_current[1] = AmonDrone.position.position_y;              // [m]
					x_current[2] = AmonDrone.position.height_TOF_mm_filtered;  // pos z [m]

					// --- Velocity [3:6] ---
					// You don't have direct velocity measurement — estimate or keep 0
					x_current[3] = 0.0;  // vx
					x_current[4] = 0.0;  // vy
					x_current[5] = AmonDrone.position.velocity_z;			// vz [m/s] - kalman

					// --- Quaternion [6:10] ---
					x_current[6]  = AmonDrone.position.quaternion[0];		// qw
					x_current[7]  = AmonDrone.position.quaternion[1];		// qx
					x_current[8]  = AmonDrone.position.quaternion[2];		// qy
					x_current[9]  = AmonDrone.position.quaternion[3];		// qz

					// --- Angular rates [10:13] ---
					// gyro values must be in rad/s
					x_current[10] = AmonDrone.position.gyro_x * DEG_TO_RAD;  // wx - *deg or rad?
					x_current[11] = AmonDrone.position.gyro_y * DEG_TO_RAD;  // wy - *deg or rad?
					x_current[12] = AmonDrone.position.gyro_z * DEG_TO_RAD;  // wz - *deg or rad?

				#ifdef MODEL_1ST_ORDER
					// --- First order actuator model [13:18] ---
					x_current[13] = AmonDrone.actuators.edf_percent;			// T
					x_current[14] = AmonDrone.actuators.servo_xp * DEG_TO_RAD;	// delta_1
					x_current[15] = AmonDrone.actuators.servo_xn * DEG_TO_RAD;	// delta_2
					x_current[16] = AmonDrone.actuators.servo_yp * DEG_TO_RAD;	// delta_3
					x_current[17] = AmonDrone.actuators.servo_yn * DEG_TO_RAD;	// delta_4
				#elif MODEL_2ND_ORDER
					// --- Second order actuator model [13:23] ---
					x_current[13] = AmonDrone.actuators.edf_percent;			// T
					x_current[14] = 0.0;										// T_dot

					x_current[15] = AmonDrone.actuators.servo_xp * DEG_TO_RAD;	// delta_1
					x_current[16] = AmonDrone.actuators.servo_xn * DEG_TO_RAD;	// delta_2
					x_current[17] = AmonDrone.actuators.servo_yp * DEG_TO_RAD;	// delta_3
					x_current[18] = AmonDrone.actuators.servo_yn * DEG_TO_RAD;	// delta_4

					x_current[19] = 0.0;										// delta_dot_1
					x_current[20] = 0.0;										// delta_dot_2
					x_current[21] = 0.0;										// delta_dot_3
					x_current[22] = 0.0;										// delta_dot_4
				#endif


					// --------------------------------------------------------------------------------------------------------
					// Set NMPC controller reference values //
					static double x_ref[NMPC_NX] = {0};		// Reset everything

					if (AmonDrone.data.flag_land_now)
					{
					    // Cancel current flight path execution
						AmonDrone.flight_status = STATUS_FLIGHT_LANDING;
						AmonDrone.data.nmpc_set_new_ref = 1;
						land_now(&AmonDrone.flight_path, x_ref, AmonDrone.position.position_x, AmonDrone.position.position_y);
					}
					else
					{
						execute_flight_command(&AmonDrone.flight_path, x_ref);
						// TODO: Trajectory generator?
					}


					static double u_ref[NMPC_NU] = {
					    87.0,  // hover thrust / EDF nominal
					    0.0,   // servo XP neutral or trim/offset
					    0.0,   // servo XN neutral or trim/offset
					    0.0,   // servo YP neutral or trim/offset
					    0.0    // servo YN neutral or trim/offset
					};

					// Set NMPC init value
					NMPC_SetState(&nmpc, x_current);

					// Set new reference only when command changes
					if (AmonDrone.data.nmpc_set_new_ref)
					{
						AmonDrone.data.nmpc_set_new_ref = 0;
						NMPC_SetReference(&nmpc, x_ref, u_ref);
					}



					// --------------------------------------------------------------------------------------------------------
					// Run NMPC control logic //
					uint32_t c0 = DWT->CYCCNT;
					int ret = NMPC_Solve(&nmpc);	// Solve optimization problem
					uint32_t cycles = DWT->CYCCNT - c0;
					float us = (float)cycles / (SystemCoreClock / 1000000.0f);  // µs

					// Running sum - averaging
					nmpc.nmpc_solve_time_sum -= nmpc.nmpc_solve_time_arr[nmpc.nmpc_solve_time_cnt];
					nmpc.nmpc_solve_time_arr[nmpc.nmpc_solve_time_cnt] = us;
					nmpc.nmpc_solve_time_sum += us;
					nmpc.nmpc_solve_time_cnt++;
					if (nmpc.nmpc_solve_time_cnt >= 10) nmpc.nmpc_solve_time_cnt = 0;
					if (nmpc.nmpc_solve_time_samples < 10) nmpc.nmpc_solve_time_samples++;

					nmpc.nmpc_solve_time = nmpc.nmpc_solve_time_sum / nmpc.nmpc_solve_time_samples;
					AmonDrone.data.nmpc_solver_time = nmpc.nmpc_solve_time;

					// --- Solver result monitoring ---
					if (ret == NMPC_OK)
					{
						AmonDrone.data.nmpc_solver_fail_cnt = 0;
					}
					else
					{
						AmonDrone.data.nmpc_solver_fail_cnt++;

						if (AmonDrone.data.nmpc_solver_fail_cnt > 5)
						{
							// 5 solver fails back-to-back -> disarm or hold last good u
							AmonDrone.data.NMPC_enable = 0;
							// optionally: keep last good actuator values
						}
					}


					// --------------------------------------------------------------------------------------------------------
					// Limit actuator values and write them to actuators //
					if (ret == NMPC_OK ) // || ret == NMPC_SOLVER_ERR
					{
					    static float servo_xp_old = 0.0f;
					    static float servo_xn_old = 0.0f;
					    static float servo_yp_old = 0.0f;
					    static float servo_yn_old = 0.0f;

						double u[NMPC_NU];
						NMPC_GetControl(&nmpc, u);

						// Clamp before applying - defensive programming
						u[0] = fmax(NMPC_U0_MIN, fmin(NMPC_U0_MAX, u[0]));  // thrust	[0..100]
						u[1] = fmax(NMPC_UX_MIN, fmin(NMPC_UX_MAX, u[1]));  // servo	[-45..45]
						u[2] = fmax(NMPC_UX_MIN, fmin(NMPC_UX_MAX, u[2]));
						u[3] = fmax(NMPC_UX_MIN, fmin(NMPC_UX_MAX, u[3]));
						u[4] = fmax(NMPC_UX_MIN, fmin(NMPC_UX_MAX, u[4]));

						// Servo slew-rate limitation ("jumpy and spiny servos") - sort of real servo feedback
						float servo_xp_cmd = SlewLimit((float)u[1], servo_xp_old, SERVO_MAX_STEP_DEG);
						float servo_xn_cmd = SlewLimit((float)u[2], servo_xn_old, SERVO_MAX_STEP_DEG);
						float servo_yp_cmd = SlewLimit((float)u[3], servo_yp_old, SERVO_MAX_STEP_DEG);
						float servo_yn_cmd = SlewLimit((float)u[4], servo_yn_old, SERVO_MAX_STEP_DEG);

						//AmonDrone.actuators.edf_percent = (float)u[0];
						AmonDrone.actuators.edf_percent = (uint8_t)round((float)u[0]);
						AmonDrone.actuators.servo_xp = servo_xp_cmd;
						AmonDrone.actuators.servo_xn = servo_xn_cmd;
						AmonDrone.actuators.servo_yp = servo_yp_cmd;
						AmonDrone.actuators.servo_yn = servo_yn_cmd;

						// Set motor to position/power
						PowerToPWMValue(AmonDrone.actuators.edf_percent);
						DegresToCCR(AmonDrone.actuators.servo_xn, SERVO_XN);
						DegresToCCR(AmonDrone.actuators.servo_xp, SERVO_XP);
						DegresToCCR(AmonDrone.actuators.servo_yn, SERVO_YN);
						DegresToCCR(AmonDrone.actuators.servo_yp, SERVO_YP);

						// Save current values for next iteration check
						servo_xp_old = AmonDrone.actuators.servo_xp;
						servo_xn_old = AmonDrone.actuators.servo_xn;
						servo_yp_old = AmonDrone.actuators.servo_yp;
						servo_yn_old = AmonDrone.actuators.servo_yn;
					}

					// --------------------------------------------------------------------------------------------------------
					/* Command check - time or position */
					uint8_t time_done = 0;
					uint8_t position_done = 0;
					s_flight_command *cmd = NULL;
					s_flight_command land_now_cmd = {0};


					if (AmonDrone.data.flag_land_now)
					{
					    land_now_cmd.command = COMM_LAND;
					    cmd = &land_now_cmd;
					}
					else
					{
					    cmd = &AmonDrone.flight_path.flight_path[AmonDrone.flight_path.command_index];
					}

					// Check if current command reached its timeout value (if 0 then the command dont have timeout)
					if (AmonDrone.flight_path.command_timeout_s != 0)
					{
						if (AmonDrone.flight_path.command_time_s >= AmonDrone.flight_path.command_timeout_s)
						{
							time_done = 1;
						}
						else
						{
							AmonDrone.flight_path.command_time_s += TIM_50HZ_DT; // Current command elapsed time in seconds
						}
					}

					if (cmd != NULL)
					{
						position_done = command_position_reached(cmd, x_current, x_ref);
					}

					if (time_done || position_done)
					{
						AmonDrone.flight_path.command_time_s = 0; // Reset elapsed time command counter
						AmonDrone.flight_path.command_index++;
						AmonDrone.data.nmpc_set_new_ref = 1;
						if (AmonDrone.flight_path.command_index >= AmonDrone.flight_path.command_cnt)
						{
							//TODO: Check last command and land if not already
							AmonDrone.DroneStatus = STATUS_FLY_OVER;
						}
					}

					if (AmonDrone.uart_buffer.flag_logging_active)
					{
						AmonDrone.uart_buffer.flag_log_now = 1;
						log_add_sample(&AmonDrone.position, &AmonDrone.data, &AmonDrone.actuators);
					}
				}
	  		  } // TIMER 50Hz


	  		  // --- Logging to external flash - flight log ---
	  		  //TODO: add log data of NMPC
	  		  if (AmonDrone.uart_buffer.flag_log_now)
			  {
	  			  AmonDrone.uart_buffer.flag_log_now = 0;
	  			  //log_add_sample(&AmonDrone.position, &AmonDrone.data, &AmonDrone.actuators);
			  }



	  /*##############################################################################################################################################
	  ##################################################################### GNSS #####################################################################
	  ##############################################################################################################################################*/

	  		  /*##### GNSS/GPS - New packet available #####*/
	  		  if (NewGPSData == 1)
	  		  {
	  			  NewGPSData = 0;

	  			  // Used decoders selected with define in drone_data.h
	  	#ifdef USE_GPS_GGA
	  			  GPS_Decode_GGA(AmonDrone.gps_data.GPS_RX_buffer, &AmonDrone.gps_data.gga);
	  	#endif

	  	#ifdef USE_GPS_GLL
	  			  GPS_Decode_GLL(AmonDrone.gps_data.GPS_RX_buffer, &AmonDrone.gps_data.gll);
	  	#endif

	  	#ifdef USE_GPS_GSA
	  			  GPS_Decode_GSA(AmonDrone.gps_data.GPS_RX_buffer, &AmonDrone.gps_data.gsa);
	  	#endif

	  	#ifdef USE_GPS_GSV
	  			  GPS_Decode_GSV(AmonDrone.gps_data.GPS_RX_buffer, &AmonDrone.gps_data.gsv);
	  	#endif

	  	#ifdef USE_GPS_RMC
	  			  GPS_Decode_RMC(AmonDrone.gps_data.GPS_RX_buffer, &AmonDrone.gps_data.rmc);
	  	#endif

	  	#ifdef USE_GPS_VTG
	  			  GPS_Decode_VTG(AmonDrone.gps_data.GPS_RX_buffer, &AmonDrone.gps_data.vtg);
	  	#endif

	  		  }
	  	  } // Boot done / Startup




	  /*#####################################################################################################################################################
	  ###################################################################### INIT & CODE ####################################################################
	  #####################################################################################################################################################*/


	  	  /********************************************************
	  	   ******************** INIT  SEQUENCE ********************
	  	   ********************************************************/
	  	  if (StartupInit == STATUS_STARTUP)
	  	  {

	  		// Set special flags
	  #ifdef IDENTIFICATION
	  		AmonDrone.identifications.flag_test_identification = 1;
	  #endif

	  #ifdef TEST_MOMENTS
	  		AmonDrone.identifications.flag_test_moment = 1;
	  #endif

	  		/* Test all motors */
	  		AmonDrone.actuators.edf_percent = 0;
	  		AmonDrone.actuators.servo_xp = 0;
			AmonDrone.actuators.servo_xn = 0;
			AmonDrone.actuators.servo_yp = 0;
			AmonDrone.actuators.servo_yn = 0;

			HAL_Delay(1000); // Stabilize voltage for ADC reading

	  		/* Trigger ADC DMA to read battery and buck status */
	  		HAL_StatusTypeDef ret;
	  		ADC_DMA_DataRdy = 0;
	  		enable_7v2_buck(ENABLE);
			enable_5v_buck(ENABLE);

	  		ret = HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);	// ADC calibration
	  		if (ret != HAL_OK) Error_Handler();

	  		ret = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 4);		// Start ADC DMA (read analog value)
	  		if (ret != HAL_OK) Error_Handler();

			enable_5v_buck(DISABLE);

	#ifdef IDENTIFICATION
		if (AmonDrone.actuators.edf_enable == 0) EDFEnable();
		AmonDrone.actuators.edf_percent = 0;
		PowerToPWMValue(AmonDrone.actuators.edf_percent);
	#else
		enable_7v2_buck(DISABLE);
	#endif

			while (ADC_DMA_DataRdy == 0);
			AmonDrone.data.battery_main_voltage = ADC_Read_Main_Battery();
			AmonDrone.data.battery_edf_voltage = ADC_Read_EDF_Battery();
			AmonDrone.data.buck_5v_voltage = ADC_Read_5V_Buck();
			AmonDrone.data.buck_7v2_voltage = ADC_Read_7V2_Buck();

	  		/* Check battery voltage */
	  		if (AmonDrone.data.battery_main_voltage < 1050) // minimal voltage: 10.5V
	  		{
	  			AmonDrone.error_code.err_main_bat = 1;
	  		}
	  		if (AmonDrone.data.battery_edf_voltage < 2100) // minimal voltage per battery: 10.5V -> x2 = 21V
	  		{
	  			AmonDrone.error_code.err_edf_bat = 1;
	  		}

	  		HAL_Delay(500);

	  		/* Reset all devices */
	  		if (BME280_Reset(&bme280, &hi2c2) != 0) AmonDrone.error_code.err_bme280 = 1;
	  		if (MPU6050_Reset(&mpu6050, &hi2c2) != 0) AmonDrone.error_code.err_mpu6050 = 1;

	  		HAL_Delay(500); // delay sensors config to complete power on

	  		/* BME280 - temp, press, hum */
	  		if (BME280_ReadDeviceID(&bme280, &hi2c2) != 0) AmonDrone.error_code.err_bme280 = 1;
	  		if (BME280_ReadCalibData(&bme280, &hi2c2) != 0) AmonDrone.error_code.err_bme280 = 1;
	  		if (BME280_Init(&bme280, &hi2c2) != 0) AmonDrone.error_code.err_bme280 = 1;
	  		HAL_Delay(100);
	  		if (BME280_ReadAllData(&bme280, &hi2c2) != 0) AmonDrone.error_code.err_bme280 = 1;
	  		if (AmonDrone.data.take_off_alt_m == 0) AmonDrone.data.take_off_alt_m = ALTITUDE_M;
	  		if (BME280_Altitude_Init(&bme280, &hi2c2, AmonDrone.data.take_off_alt_m) != 0) AmonDrone.error_code.err_bme280 = 1;

	  		/* MPU6050 - gyro */
	  		if (MPU6050_ReadDeviceID(&mpu6050, &hi2c2) != 0) AmonDrone.error_code.err_mpu6050 = 1;
	  		if (MPU6050_ReadFactoryTrim(&mpu6050, &hi2c2) != 0) AmonDrone.error_code.err_mpu6050 = 1;
	  		if (MPU6050_Init(&mpu6050, &hi2c2) != 0) AmonDrone.error_code.err_mpu6050 = 1;
	  		if (MPU6050_ReadFactoryTrim(&mpu6050, &hi2c2) != 0) AmonDrone.error_code.err_mpu6050 = 1;
	  		if (MPU6050_ReadAllDirect(&mpu6050, &hi2c2) != 0) AmonDrone.error_code.err_mpu6050 = 1;
	  		if (MPU6050_SelfTest(&mpu6050, &hi2c2) != 0) AmonDrone.error_code.err_mpu6050 = 1;

	  		/* HMC5883L - compass */
	  		hmc5883l.config.sample_avgeraging = SAMPLES_4;
	  		hmc5883l.config.data_rate = DTR_75;
	  		hmc5883l.config.measurement_mode = MEAS_MODE_NORMAL;
	  		hmc5883l.config.gain = GAIN_1_3GA;
	  		hmc5883l.config.operating_mode = OP_MODE_CONTINUOUS;

	  		if (HMC5883L_CheckID(&hmc5883l, &hi2c2) != 0) AmonDrone.error_code.err_hmc5883l = 1;
	  		if (HMC5883L_Init(&hmc5883l, &hi2c2) != 0) AmonDrone.error_code.err_hmc5883l = 1;
	  		//if (HMC5883L_SelfTest(&hmc5883l, &hi2c2) != 0) AmonDrone.error_code.err_hmc5883l = 1; // Fail - can be faulty/cloned ic

	  		/* PMW3901 */
		#ifdef USE_OPTICAL_FLOW
	  		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	  		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	  		PMW3901_pin_config(&pmw3901, &hspi2, CS_OF_GPIO_Port, CS_OF_Pin, OF_RST_GPIO_Port, OF_RST_Pin, OF_MOT_GPIO_Port, OF_MOT_Pin);
	  		if (PMW3901_init(&pmw3901) != PMW3901_OK) AmonDrone.error_code.err_pmw3901 = 1;
		#endif

	  		/* Init Kalman filter */
	  #ifdef GYRO_KALMAN
	  		Kalman_Init(&kalman_pitch);
	  		Kalman_Init(&kalman_roll);
	  		Kalman_Init(&kalman_yaw);
	  		KalmanZ_Init(&kalman_tof, 0.001f, 0.05f, 0.02f);
	  #endif

	  		/* Filters and other drone flight variables */
	  		AmonDrone.position.quaternion[0] = 1.0f;
	  		AmonDrone.position.quaternion[1] = 0.0f;
	  		AmonDrone.position.quaternion[2] = 0.0f;
	  		AmonDrone.position.quaternion[3] = 0.0f;
	  		AmonDrone.position.position_x = 0.0f;
	  		AmonDrone.position.position_y = 0.0f;
	  		AmonDrone.position.position_z = 0.0f;

	  		AmonDrone.actuators.rampUpDone = 0;

	  		AmonDrone.uart_buffer.flag_new_uart_rx_data = 0;
	  		AmonDrone.uart_buffer.flag_new_uart_tx_data = 0;
	  		AmonDrone.uart_buffer.flag_log_available = 0;
	  		AmonDrone.uart_buffer.flag_log_remove = 0;
	  		AmonDrone.uart_buffer.flag_log_dump = 0;
	  		AmonDrone.uart_buffer.flag_logging_active = 0;
	  		AmonDrone.data.NMPC_enable = 0;
	  		AmonDrone.data.nmpc_solver_fail_cnt = 0;
	  		AmonDrone.data.buck_7v2_enable = 0;
	  		AmonDrone.data.buck_5v_enable = 0;

	  		AmonDrone.radio_data.conn_status = 0;
	  		AmonDrone.radio_data.flag_new_rf_rx_data = 0;
	  		AmonDrone.radio_data.flag_new_rf_tx_data = 0;
	  		AmonDrone.radio_data.flag_telemetry_send = 0;


	  		/* vl53l1x - ToF*/
	  		uint8_t bootOK = 0;
	  		uint8_t retries = 0;
	  		while (bootOK == 0 && retries <= 5)
	  		{
	  			if (VL53L1X_BootState(&vl53l1Dev, &hi2c2, &bootOK) != 0)
				{
	  				AmonDrone.error_code.err_vl53l1x = 1;
				}
	  			else
	  			{
	  				AmonDrone.error_code.err_vl53l1x = 0;
	  			}
	  			retries++;
	  		}

	  		if (AmonDrone.error_code.err_vl53l1x == 0)
	  		{
				if (VL53L1X_ReadID(&vl53l1Dev, &hi2c2) != 0) AmonDrone.error_code.err_vl53l1x = 1;
				if (VL53L1X_SensorInit(&vl53l1Dev, &hi2c2) != 0) AmonDrone.error_code.err_vl53l1x = 1;
				if (VL53L1X_SetTimingBudgetInMs(&vl53l1Dev, &hi2c2, 200) != 0) AmonDrone.error_code.err_vl53l1x = 1; // 140ms is min for 4m distance
				if (VL53L1X_SetOffset(&vl53l1Dev, &hi2c2, -121) != 0) AmonDrone.error_code.err_vl53l1x = 1; // Set height from ground to get zero -130
				if (VL53L1X_StartRanging(&vl53l1Dev, &hi2c2) != 0) AmonDrone.error_code.err_vl53l1x = 1;
	  		}

	  		/* NRF24L01 - radio */
	  		radio1.op_modes = NRF_MODE_PWR_ON_RST;              // set default radio state
	  		radio2.op_modes = NRF_MODE_PWR_ON_RST;              // set default radio state
	  		AmonDrone.radio_data.conn_status = CONN_STATUS_DISCONNECTED;
	  		AmonDrone.radio_data.flag_connection_lost = 0;
	  		AmonDrone.radio_data.flag_connection_begin = 0;
	  		radio1.irq_on_pipe = 0xFF;
	  		radio2.irq_on_pipe = 0xFF;
	  		radio1.irq_flag = 0;
	  		radio2.irq_flag = 0;
	  		AmonDrone.radio_data.flag_stream_data = 0;
	  		RF_IRQ1_EN = 0;
	  		RF_IRQ2_EN = 0;


	  		// Radios initialization and setup
	  		NRF24_pin_config(&radio1, &hspi3, RF1_CSN_GPIO_Port, RF1_CSN_Pin, RF1_CE_GPIO_Port, RF1_CE_Pin);        // Map pins for radio 1
	  		NRF24_pin_config(&radio2, &hspi3, RF2_CSN_GPIO_Port, RF2_CSN_Pin, RF2_CE_GPIO_Port, RF2_CE_Pin);        // Map pins for radio 2

	  		HAL_Delay(1000);

	  		uint8_t stat = 0;
	  		NRF24_ReadStatus(&radio1, &stat);
	  		if (stat == 0x00 || stat == 0xFF)
	  		{
	  		    radio1.radioErr = NRF_ERR_BOOT;
	  		    AmonDrone.error_code.err_radio1 = 1;
	  		}
	  		else
	  		{
	  		    // Radio answered
	  		    NRF24_WriteRegister(&radio1, RF_STATUS, RX_DR | TX_DS | MAX_RT, NULL);
	  		    radio1.radioErr = NRF_ERR_NONE;
	  		}

	  		HAL_Delay(10); // Safe delay for checking radio status

	  		NRF24_ReadStatus(&radio2, &stat);
			if (stat == 0x00 || stat == 0xFF)
			{
				radio2.radioErr = NRF_ERR_BOOT;
				AmonDrone.error_code.err_radio2 = 1;
			}
			else
			{
				// Radio answered
				NRF24_WriteRegister(&radio2, RF_STATUS, RX_DR | TX_DS | MAX_RT, NULL);
				radio2.radioErr = NRF_ERR_NONE;
			}

			HAL_Delay(10); // Safe delay before radio init

	  		if (radio1.radioErr == 1)
			{
	  			AmonDrone.error_code.err_radio1 = 1;
			}
	  		else
	  		{
	  			// Set radio configurations and init
				radio1.role     = NRF_ROLE_PTX;
				radio1.config   = &radio_tx_normal_cfg;
				radio1.address  = &radio_tx_addr;
				NRF24_SetTXAddress(&radio1, radio1.address->tx_addr);
				NRF24_init(&radio1);

				NRF24_ReadStatus(&radio1, &stat);
				if (stat != 0x0E) AmonDrone.error_code.err_radio1 = 1;
	  		}


	  		HAL_Delay(10); // Safe delay before radio init
	  		if (radio2.radioErr == 1)
	  		{
	  			AmonDrone.error_code.err_radio2 = 1;
	  		}
	  		else
	  		{
				// Set radio configurations and init
				radio2.role     = NRF_ROLE_PRX;
				radio2.config   = &radio_rx_normal_cfg;
				radio2.address  = &radio_rx_addr;
				NRF24_SetRXAddress(&radio2, 0, radio2.address->pipe0_rx_addr);
				NRF24_init(&radio2);

				NRF24_ReadStatus(&radio2, &stat);
				if (stat != 0x0E) AmonDrone.error_code.err_radio2 = 1;
	  		}

	  		// Enable radio IRQs after the radios are configured.
	  		__HAL_GPIO_EXTI_CLEAR_IT(RF1_IRQ_Pin);
	  		__HAL_GPIO_EXTI_CLEAR_IT(RF2_IRQ_Pin);
	  		HAL_NVIC_ClearPendingIRQ(RF1_IRQ_EXTI_IRQn);
	  		HAL_NVIC_ClearPendingIRQ(RF2_IRQ_EXTI_IRQn);
	  		RF_IRQ1_EN = 1;
			RF_IRQ2_EN = 1;
	  		HAL_NVIC_EnableIRQ(RF1_IRQ_EXTI_IRQn);
	  		HAL_NVIC_EnableIRQ(RF2_IRQ_EXTI_IRQn);

			if (HAL_GPIO_ReadPin(RF1_IRQ_GPIO_Port, RF1_IRQ_Pin) == GPIO_PIN_RESET)
			{
			    radio1.irq_flag = 1;
			}

			if (HAL_GPIO_ReadPin(RF2_IRQ_GPIO_Port, RF2_IRQ_Pin) == GPIO_PIN_RESET)
			{
			    radio2.irq_flag = 1;
			}


	  		/* Timers - NOT PWM! */
	  		HAL_TIM_Base_Start_IT(&htim5); // sensor read, leg leds (50Hz)
	  		HAL_TIM_Base_Start_IT(&htim6); // Complementary Filter
	  		HAL_TIM_Base_Start_IT(&htim7); // Second timer - time

	  		// Timer for NMPC solve time
	  		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	  		DWT->CYCCNT = 0;
	  		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

		#ifndef CALIBRATION
	  		HAL_UART_Receive_DMA(&huart1, USART1_GPSRX_DMA, sizeof(USART1_GPSRX_DMA)); //426
	  		memset(AmonDrone.uart_buffer.buffer_temp, 0, sizeof(AmonDrone.uart_buffer.buffer_temp));
	  		memset(AmonDrone.uart_buffer.buffer_UART_RX, 0, sizeof(AmonDrone.uart_buffer.buffer_UART_RX));
	  		memset(AmonDrone.uart_buffer.buffer_UART_TX, 0, sizeof(AmonDrone.uart_buffer.buffer_UART_TX));
	  		HAL_UART_Receive_IT(&huart5, AmonDrone.uart_buffer.buffer_temp, 1);
		#endif

	  		/* Logging */
	  		AmonDrone.uart_buffer.flag_log_available = 0;
	  		AmonDrone.uart_buffer.flag_logging_active = 0;
	  		AmonDrone.uart_buffer.log_file = "log.txt";

	  #ifdef LOG_ENABLE
	  		Flash_Init();     	// if not inside LOG_Init
	  		if (log_init() != 0) AmonDrone.error_code.err_flash = 1; // init flash and mount filesystem

	  		// LittleFS test //
			#ifdef TEST_LITTLEFS
						log_test_write();  	// write test file
						log_test_read();	// read test file back
			#endif
	  #endif

	  		if (bme280.dig_T1 == 0 || bme280.dig_T2 == 0) // for WTF error
	  		{
	  			AmonDrone.error_code.err_bme280 = 1;
	  		}


	  		// Start timers for sensors and LEDs & indicate end of init
	  		HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, GPIO_PIN_SET);
	  		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	  		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // RGB (50Hz)
	  		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // RGB (50Hz)
	  		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // RGB (50Hz)
			EDFEnable(); // Enabling EDF motor


	  		// Check if all init functions are OK
	  		if (AmonDrone.error_code.err_bme280 == 1   ||
	  			AmonDrone.error_code.err_hmc5883l == 1 ||
				AmonDrone.error_code.err_mpu6050 == 1  ||
				AmonDrone.error_code.err_pmw3901 == 1  ||
				AmonDrone.error_code.err_vl53l1x == 1  ||
				AmonDrone.error_code.err_radio1 == 1   ||
				AmonDrone.error_code.err_radio2 == 1   ||
				AmonDrone.error_code.err_main_bat == 1)
	  		{
	  			// ERROR
	  			StartupInit = 1;
				AmonDrone.DroneStatus = STATUS_ERROR;
	  			//AmonDrone.DroneStatus = STATUS_IDLE;
	  		}
	  		else
	  		{
	  			// OK
				StartupInit = 1;
				AmonDrone.DroneStatus = STATUS_IDLE;
	  		}
	  	 }




	  #ifndef CALIBRATION

	  	  /********************************************************
	  	   ************** SEQUENCE IDLE, ARM, FLY... **************
	  	   ********************************************************/

	  	  if (StartupInit != STATUS_STARTUP && AmonDrone.DroneStatus != STATUS_ERROR)
	  	  {

	  		  /*** MAIN STATE MACHINE ***/
	  		  switch(AmonDrone.DroneStatus) {

	  			  // STATE: Drone idling
	  			  case STATUS_IDLE:

	  	  	  	  #ifndef IDENTIFICATION
	  				  if (armCheck != 0) armCheck = 0;	// Clear sensor check at the beginning of arm state

	  				  // Logging - after delay turn it off
					  if (AmonDrone.uart_buffer.flag_logging_active == 1)
					  {
						  log_close_file();
						  AmonDrone.uart_buffer.flag_logging_active = 0;
					  }

					  if (AmonDrone.data.NMPC_enable == 1)
					  {
						AmonDrone.data.NMPC_enable = 0;
						memset(nmpc.nmpc_solve_time_arr, 0, sizeof(nmpc.nmpc_solve_time_arr));
						nmpc.nmpc_solve_time_cnt = 0;
						nmpc.nmpc_solve_time_samples = 0;
						nmpc.nmpc_solve_time_sum = 0;
						nmpc.nmpc_solve_time = 0;
						NMPC_DeInit(&nmpc);
						HAL_TIM_Base_Stop_IT(&htim1);
					  }

				  #endif

	  				  break;


	  			  // STATE: Drone armed and ready to take off
	  			  case STATUS_ARM:

	  	  	  	  #ifndef IDENTIFICATION
	  				  // Check sensor values and check flying path
	  				  if (armCheck == 0)
	  				  {
	  					  uint8_t err_cnt = 0;
//	  					  if (AmonDrone.radio_data.conn_status != CONN_STATUS_CONNECTED) err_cnt++;
//	  					  if (AmonDrone.error_code.err_bme280 == 1   ||
//							  AmonDrone.error_code.err_hmc5883l == 1 ||
//							  AmonDrone.error_code.err_mpu6050 == 1  ||
//							  AmonDrone.error_code.err_pmw3901 == 1  ||
//							  AmonDrone.error_code.err_vl53l1x == 1  ||
//							  AmonDrone.error_code.err_radio1 == 1   ||
//							  AmonDrone.error_code.err_radio2 == 1   ||
//							  AmonDrone.error_code.err_flash == 1    ||
//							  AmonDrone.error_code.err_main_bat == 1 ||
//							  AmonDrone.error_code.err_edf_bat == 1  ||
//							  AmonDrone.error_code.err_buck_7v2 == 1) err_cnt++;
//	  					  if (AmonDrone.position.height_TOF_mm > 100) err_cnt++;
//	  					  if (AmonDrone.position.Pitch > 10 || AmonDrone.position.Pitch < -10) err_cnt++;
//	  					  if (AmonDrone.position.Roll > 10 || AmonDrone.position.Roll < -10) err_cnt++;
//	  					  if (AmonDrone.position.accel_x > 10 || AmonDrone.position.accel_x < -10) err_cnt++;
//	  					  if (AmonDrone.position.accel_y > 10 || AmonDrone.position.accel_y < -10) err_cnt++;
//	  					  if (AmonDrone.position.accel_z > 10 || AmonDrone.position.accel_z < -10) err_cnt++;
//	  					  if (AmonDrone.position.gyro_x > 10 || AmonDrone.position.gyro_x < -10) err_cnt++;
//	  					  if (AmonDrone.position.gyro_y > 10 || AmonDrone.position.gyro_y < -10) err_cnt++;
//	  					  if (AmonDrone.position.gyro_z > 10 || AmonDrone.position.gyro_z < -10) err_cnt++;
//	  					  // TBD...

	  					  // Final check
	  					  if (err_cnt == 0 && armCheck == 0)
						  {
	  						  armCheck = 1;
	  						  AmonDrone.flight_path.command_index = 0;
	  						  AmonDrone.flight_path.flight_start_time = 0;
	  						  AmonDrone.flight_path.command_time_s = 0;
	  						  AmonDrone.flight_path.command_timeout_s = 0;
	  						  AmonDrone.actuators.rampUpDone = 0;
	  						  AmonDrone.actuators.rampUpEnable = 0;
	  						  AmonDrone.actuators.edf_off_delay = 0;
							  AmonDrone.uart_buffer.log_save_delay = 0;
							  AmonDrone.flight_status = STATUS_FLIGHT_GROUND;
							  AmonDrone.uart_buffer.flag_log_available = 0;
							  AmonDrone.data.flag_land_now = 0;
							  AmonDrone.data.flag_e_kill = 0;
							  AmonDrone.data.NMPC_enable = 0;
							  AmonDrone.data.nmpc_set_new_ref = 1;
							  AmonDrone.data.nmpc_solver_fail_cnt = 0;

							  AmonDrone.actuators.edf_enable = 0;
							  AmonDrone.actuators.servo_enable = 0;
	  						  AmonDrone.actuators.edf_percent = 0;
	  						  AmonDrone.actuators.servo_xp = 0;
	  						  AmonDrone.actuators.servo_xn = 0;
	  						  AmonDrone.actuators.servo_yp = 0;
	  						  AmonDrone.actuators.servo_yn = 0;
							  PowerToPWMValue(AmonDrone.actuators.edf_percent);
							  DegresToCCR(AmonDrone.actuators.servo_xn, SERVO_XN);
							  DegresToCCR(AmonDrone.actuators.servo_xp, SERVO_XP);
							  DegresToCCR(AmonDrone.actuators.servo_yn, SERVO_YN);
							  DegresToCCR(AmonDrone.actuators.servo_yp, SERVO_YP);

	  						  // Enable and test servo motors
	  						  if (AmonDrone.actuators.servo_enable == 0)
	  						  {
	  							  enable_7v2_buck(ENABLE);
	  							  HAL_Delay(100);
	  							  servoTest();
	  						  }

	  						  // Enable EDF if not
	  						  if (AmonDrone.actuators.edf_enable == 0)
	  						  {
								  EDFEnable();
							  }

	  						  // Logging
							#ifdef LOG_ENABLE
	  						  if (AmonDrone.uart_buffer.flag_logging_active == 0)
	  						  {
	  							  AmonDrone.uart_buffer.flag_logging_active = 1;
	  							  log_open_file();
	  						  }
							#endif

	  						  // Enable NMPC regulator
							  if (AmonDrone.data.NMPC_enable == 0)
							  {
								#ifdef COMPILE_MEM_ANALYZER
								  extern int NMPC_DebugAnalyzeMemory(void);
								#endif

								  if (NMPC_Init(&nmpc) != NMPC_OK)
								  {
									  AmonDrone.DroneStatus = STATUS_ERROR;
								  }
								#ifdef COMPILE_MEM_ANALYZER
								  NMPC_DebugAnalyzeMemory();
								#endif

							  }
						  }
	  					  else
	  					  {
	  						  AmonDrone.DroneStatus = STATUS_ERROR;
	  						  break;
	  					  }
	  				  }
	  	  	  	  #endif
	  				  break;


	  			  // STATE: Flying
	  			  case STATUS_FLY:

	  				  if (AmonDrone.data.flag_e_kill) AmonDrone.DroneStatus = STATUS_ERROR;

	  				  // Enable EDF if not already
	  				  if (AmonDrone.actuators.edf_enable == 0) EDFEnable();

	  				  // Enable NMPC
	  				  if (AmonDrone.actuators.edf_enable && AmonDrone.actuators.rampUpDone && AmonDrone.data.flag_e_kill == 0 && AmonDrone.data.flag_land_now == 0)
	  				  {
						  if (AmonDrone.data.NMPC_enable == 0)
						  {
							  AmonDrone.data.NMPC_enable = 1;
							  memset(nmpc.nmpc_solve_time_arr, 0, sizeof(nmpc.nmpc_solve_time_arr));
							  nmpc.nmpc_solve_time_cnt = 0;
							  nmpc.nmpc_solve_time_samples = 0;
							  nmpc.nmpc_solve_time_sum = 0;
							  nmpc.nmpc_solve_time = 0;
							  HAL_TIM_Base_Start_IT(&htim1);
						  }
	  				  }

	  				  // TODO: Regulator parameter switch? (take off more aggressive, flight smooth, landing slow)
	  				  switch (AmonDrone.flight_status)
	  				  {
	  					case STATUS_FLIGHT_GROUND:

	  						break;

	  					case STATUS_FLIGHT_TAKEOFF:

	  						break;

	  					case STATUS_FLIGHT_FLYING:

	  						break;

	  					case STATUS_FLIGHT_LANDING:

	  						break;

	  					default:
	  						// Undefined
	  						break;
	  				}

	  				  break;


	  			  // STATE: End of flight
	  			  case STATUS_FLY_OVER:

	  	  	  	  #ifndef IDENTIFICATION

					  if (AmonDrone.data.NMPC_enable == 1)
					  {
						AmonDrone.data.NMPC_enable = 0;
						NMPC_DeInit(&nmpc);
						HAL_TIM_Base_Stop_IT(&htim1);
					  }

					  if (armCheck)
					  {
						  AmonDrone.flight_status = STATUS_FLIGHT_GROUND;
						  AmonDrone.data.flag_land_now = 0;
						  AmonDrone.data.flag_e_kill = 0;
						  AmonDrone.flight_path.command_index = 0;
						  AmonDrone.flight_path.flight_start_time = 0;
						  AmonDrone.flight_path.command_time_s = 0;
						  AmonDrone.actuators.rampUpDone = 0;
						  AmonDrone.actuators.rampUpEnable = 0;
						  AmonDrone.data.nmpc_set_new_ref = 1;
						  AmonDrone.flight_path.command_timeout_s = 0;
						  AmonDrone.data.nmpc_solver_fail_cnt = 0;
						  AmonDrone.uart_buffer.flag_log_now = 0;
						  AmonDrone.actuators.edf_off_delay = 0;
						  AmonDrone.uart_buffer.log_save_delay = 0;
						  armCheck = 0;

						  memset(nmpc.nmpc_solve_time_arr, 0, sizeof(nmpc.nmpc_solve_time_arr));
						  nmpc.nmpc_solve_time_cnt = 0;
						  nmpc.nmpc_solve_time_samples = 0;
						  nmpc.nmpc_solve_time_sum = 0;
						  nmpc.nmpc_solve_time = 0;
						  AmonDrone.data.nmpc_solver_time = 0;

						  //AmonDrone.actuators.servo_enable = 0;
						  //AmonDrone.actuators.edf_enable = 0;
						  AmonDrone.actuators.edf_percent = 50;
						  AmonDrone.actuators.servo_xp = 0;
						  AmonDrone.actuators.servo_xn = 0;
						  AmonDrone.actuators.servo_yp = 0;
						  AmonDrone.actuators.servo_yn = 0;
						  PowerToPWMValue(AmonDrone.actuators.edf_percent);
						  DegresToCCR(AmonDrone.actuators.servo_xn, SERVO_XN);
						  DegresToCCR(AmonDrone.actuators.servo_xp, SERVO_XP);
						  DegresToCCR(AmonDrone.actuators.servo_yn, SERVO_YN);
						  DegresToCCR(AmonDrone.actuators.servo_yp, SERVO_YP);
						  HAL_Delay(100);
						  enable_7v2_buck(DISABLE);
					  }
				  #endif

	  				  // Logging - after delay turn it off
					#ifdef LOG_ENABLE
	  				  if (AmonDrone.uart_buffer.flag_logging_active == 1)
	  				  {
	  					  log_close_file();
	  					  AmonDrone.uart_buffer.flag_logging_active = 0;
	  				  }
					#endif

	  				  if (AmonDrone.actuators.edf_enable == 0)
					  {
	  					  AmonDrone.actuators.edf_percent = 0;
	  					  PowerToPWMValue(AmonDrone.actuators.edf_percent);
					  }

	  				  if (AmonDrone.uart_buffer.flag_logging_active == 0 && AmonDrone.uart_buffer.flag_log_available == 0) AmonDrone.uart_buffer.flag_log_available = 1;

	  				  break;

				  // STATE: Error
				  case STATUS_ERROR:

					  if (AmonDrone.actuators.servo_enable == 1)
					  {
						  TVCServoDisable();
						  enable_7v2_buck(DISABLE);
					  }

					  if (AmonDrone.actuators.edf_enable == 1) EDFDisable(); // Trigger beeping of EDF

					  if (AmonDrone.data.NMPC_enable == 1)
					  {
						AmonDrone.data.NMPC_enable = 0;
						NMPC_DeInit(&nmpc);
						HAL_TIM_Base_Stop_IT(&htim1);
					  }

				#ifdef LOG_ENABLE
					  log_close_file();
				#endif


	  			  // STATE: Wrong state
	  			  default:

	  				  AmonDrone.DroneStatus = STATUS_ERROR;
	  				  TVCServoDisable();
	  				  EDFDisable();
	  				  enable_5v_buck(DISABLE);
	  				  enable_7v2_buck(DISABLE);

	  				  break;
	  		  }
	  	  }


	  #endif


	  /*#####################################################################################################################################################
	  ###################################################################### CALIBRATION ####################################################################
	  #####################################################################################################################################################*/


	  	  /* ********* GYROSCOPE Calibration *********
	  	   * Before using gyroscope is necessary to set right offset values for all three accelerometers.
	  	   * This is set in MPU6050.h file (#define X_ACCEL_OFFSET, X_ACCEL_OFFSET, X_ACCEL_OFFSET)
	  	   * To get these values change CALIBRATION value to 1 and reupload code on board
	  	   * Disconnect everything from the board and unmount it from drone.
	  	   * ## For PCB_V2 - STM32F405
	  	   * 	On GPS port connect FTDI module to read UART communication. DO NOT CONNECT POWER PIN (3.3V) IF YOU ARE PLANNING TO POWER THE BOARD FROM BATTERY !!!
	  	   * 	Use parameter:
	  	   * 		- Baud rate: 9600
	  	   * 		- Data bit: 8
	  	   * 		- Stop bit: 1
	  	   * 		- Parity: None
	  	   * 		- Flow control: None
	  	   *
	  	   * ## For PCB_V3 - STMH743
	  	   * 	Connect USB port on board with cable directly to the PC (no need for additional FTDI converted)
	  	   * 	Use parameter:
	  	   * 		- Baud rate: 115200
	  	   * 		- Data bit: 8
	  	   * 		- Stop bit: 1
	  	   * 		- Parity: None
	  	   * 		- Flow control: None
	  	   *
	  	   * Open serial port on PC and connect to FTDI. After initialization the dron will start to read data from gyroscope and send it over UART.
	  	   * Rotate board in all three directions (flat, vertical and sideways) at 90deg. In every position the one value will be around 1.
	  	   * The difference value and 1 is the offset value. Enter that value in firmware as offset.
	  	   * Change GYRO_CALIB back to 0, reupload code and mount board bact to drone
	  	   */

		#ifdef CALIBRATION

	  		  StatusLED(AmonDrone.DroneStatus);

	  		  if (GyroCalibTrig == 1)
	  		  {
	  			  // Print data
	  			  char message[100] = {'\0'};

			#ifdef TUNE_KALMAN

	  			  sprintf((char *)message, "Accel X = %.3f , Y = %.3f, Z = %.3f\r\nGyro X = %.3f , Y = %.3f, Z = %.3f\r\n", mpu6050.ACCEL_X, mpu6050.ACCEL_Y, mpu6050.ACCEL_Z, mpu6050.GYRO_X, mpu6050.GYRO_Y, mpu6050.GYRO_Z);

			#else

	  			  if (CAL_ACCEL_X == 1 || CAL_ACCEL_Y == 1 || CAL_ACCEL_Z == 1) sprintf((char *)message, "Accel X = %.3f , Y = %.3f, Z = %.3f\r\n", mpu6050.ACCEL_X, mpu6050.ACCEL_Y, mpu6050.ACCEL_Z);

	  			  if (CAL_GYRO_X == 1 || CAL_GYRO_Y == 1 || CAL_GYRO_Z == 1) sprintf((char *)message, "Gyro X = %.3f , Y = %.3f, Z = %.3f\r\n", mpu6050.GYRO_X, mpu6050.GYRO_Y, mpu6050.GYRO_Z);

	  		  	  if (CAL_PITCH == 1 && CAL_ROLL == 1) sprintf((char *)message, "Pitch = %.3f, Roll = %.3f\r\n", AmonDrone.position.Pitch, AmonDrone.position.Roll);

	  		  	  if (CAL_ROLL == 1 && CAL_PITCH == 0) sprintf((char *)message, "Roll = %.3f\r\n", AmonDrone.position.Roll);

	  		  	  if (CAL_PITCH == 1 && CAL_ROLL == 0) sprintf((char *)message, "Pitch = %.3f\r\n", AmonDrone.position.Pitch);

	  		  	  if (CAL_LIDAR == 1) sprintf((char *)message, "Height = %dmm\r\n", AmonDrone.position.height_TOF_mm);

			#endif
	  	  	  	  HAL_UART_Transmit(&huart5, (uint8_t*)message, strlen((char*)message), 100);

	  	  	  	  GyroCalibTrig = 0;
	  		  }

		#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV12;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x307075B1;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 23999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 239;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 239;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 23999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 23999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 199;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 23999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 23999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_F_Pin|CS_OF_Pin|OF_RST_Pin|CS_SD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_BUCK_5V_Pin|EN_BUCK_7V2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_RED_Pin|LED_WHITE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AUX_PORT_Pin|LED_BRD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RF1_CSN_Pin|RF2_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RF1_CE_Pin|RF2_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_F_Pin CS_OF_Pin */
  GPIO_InitStruct.Pin = CS_F_Pin|CS_OF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_BUCK_5V_Pin EN_BUCK_7V2_Pin */
  GPIO_InitStruct.Pin = EN_BUCK_5V_Pin|EN_BUCK_7V2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_WHITE_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_WHITE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : AUX_PORT_Pin */
  GPIO_InitStruct.Pin = AUX_PORT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AUX_PORT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OF_RST_Pin CS_SD_Pin */
  GPIO_InitStruct.Pin = OF_RST_Pin|CS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OF_MOT_Pin */
  GPIO_InitStruct.Pin = OF_MOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OF_MOT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_DRDY_Pin */
  GPIO_InitStruct.Pin = IRQ_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF1_IRQ_Pin */
  GPIO_InitStruct.Pin = RF1_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF1_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF1_CSN_Pin RF1_CE_Pin RF2_CSN_Pin RF2_CE_Pin */
  GPIO_InitStruct.Pin = RF1_CSN_Pin|RF1_CE_Pin|RF2_CSN_Pin|RF2_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RF2_IRQ_Pin */
  GPIO_InitStruct.Pin = RF2_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF2_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BRD_Pin */
  GPIO_InitStruct.Pin = LED_BRD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED_BRD_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(RF2_IRQ_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RF2_IRQ_EXTI_IRQn);

  HAL_NVIC_SetPriority(RF1_IRQ_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RF1_IRQ_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_DisableIRQ(RF2_IRQ_EXTI_IRQn);
  HAL_NVIC_DisableIRQ(RF1_IRQ_EXTI_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


// UART interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		// Copy data to buffer for later decoding
		SCB_InvalidateDCache_by_Addr((uint32_t*)USART1_GPSRX_DMA, sizeof(USART1_GPSRX_DMA));
		HAL_UART_Receive_DMA(&huart1, USART1_GPSRX_DMA, 512); 	// enable USART receive again - 426
		memset(AmonDrone.gps_data.GPS_RX_buffer, 0, sizeof(AmonDrone.gps_data.GPS_RX_buffer));
		memcpy(AmonDrone.gps_data.GPS_RX_buffer, USART1_GPSRX_DMA, sizeof(AmonDrone.gps_data.GPS_RX_buffer));
		memset(USART1_GPSRX_DMA, 0, sizeof(USART1_GPSRX_DMA));
		NewGPSData = 1;											// set flag that new data has arrived
	}


	if (huart->Instance == UART5)
	{

		static uint8_t len_new_rx_data = 0;
		static volatile uint8_t cntBuffer_UART = 0;

		// Save received data
		volatile uint8_t data = AmonDrone.uart_buffer.buffer_temp[0];                   // Read only once
		AmonDrone.uart_buffer.buffer_UART_RX[cntBuffer_UART] = data;
		memset(AmonDrone.uart_buffer.buffer_temp, 0, sizeof(AmonDrone.uart_buffer.buffer_temp));
		cntBuffer_UART++;

		// Detect start of frame and set flags
		if (data == SIG_SOF && len_new_rx_data == 0)    						// No flag for new packet and SOA packet
		{
			AmonDrone.uart_buffer.flag_USB_RX_new = 1;                          // Indicate new data received
			AmonDrone.uart_buffer.flag_new_uart_rx_data = 0;                    // Clear end of packet flag
			len_new_rx_data = 0;                                                // Clear packet counter

			// New method
			cntBuffer_UART = 0;
			memset(AmonDrone.uart_buffer.buffer_UART_RX, 0, sizeof(AmonDrone.uart_buffer.buffer_UART_RX));
			AmonDrone.uart_buffer.buffer_UART_RX[cntBuffer_UART++] = data;
		}
		else if (AmonDrone.uart_buffer.flag_USB_RX_new == 1 && len_new_rx_data == 0) // Flag for new packet, but no lenght of packet yet
		{
			len_new_rx_data = data;                                             // Save packet lenght
			AmonDrone.uart_buffer.flag_USB_RX_new = 0;                          // Clear flag for new data
		}
		else if (cntBuffer_UART >= (len_new_rx_data + 2))                       // Detect end of complete packet
		{
			AmonDrone.uart_buffer.flag_new_uart_rx_data = 1;                    // Indicate end of packet
			cntBuffer_UART = 0;                                                 // Clear UART counter
			len_new_rx_data = 0;
		}

		HAL_UART_Receive_IT(&huart5, AmonDrone.uart_buffer.buffer_temp, 1);		// Re-arm UART interrupt
	}
}


// GPIO interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == RF1_IRQ_Pin && RF_IRQ1_EN)
	{
		radio1.irq_flag = 1;
	}

	if (GPIO_Pin == RF2_IRQ_Pin && RF_IRQ2_EN)
	{
		radio2.irq_flag = 1;
	}

    if(GPIO_Pin == OF_MOT_Pin)
	{
    	// ...
	}

    if(GPIO_Pin == IRQ_DRDY_Pin)
	{
    	// Disabled - used pulling at specific time
	}
}


// Enabling servos for TVC stabilization
uint8_t TVCServoEnable()
{
	uint8_t status = 0;
	AmonDrone.actuators.servo_enable = 1;

	status += HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	status += HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	status += HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	status += HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	return status;
}


// Disabling servos for TVC stabilization
uint8_t TVCServoDisable()
{
	uint8_t status = 0;
	AmonDrone.actuators.servo_enable = 0;

	status += HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	status += HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	status += HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	status += HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

	return status;
}


// Servo test sequence
void servoTest()
{
	// Set motors on neutral position
	DegresToCCR(AmonDrone.actuators.servo_xn, SERVO_XN);
	DegresToCCR(AmonDrone.actuators.servo_xp, SERVO_XP);
	DegresToCCR(AmonDrone.actuators.servo_yn, SERVO_YN);
	DegresToCCR(AmonDrone.actuators.servo_yp, SERVO_YP);

	HAL_Delay(500); // wait on motors to stop moving

	TVCServoEnable();	// Enable Servo timer

	float test_angles[4] = {-10.0f, 0.0f, 10.0f, 0.0f};

	for (uint8_t i = 0; i < 4; i++) // test steps
	{
		for (uint8_t j = 0; j <= SERVO_YN; j++) // servos
		{
			switch (j)
			{
				case SERVO_XP:
					AmonDrone.actuators.servo_xp = test_angles[i];
					DegresToCCR(AmonDrone.actuators.servo_xp, SERVO_XP);
					break;

				case SERVO_XN:
					AmonDrone.actuators.servo_xn = test_angles[i];
					DegresToCCR(AmonDrone.actuators.servo_xn, SERVO_XN);
					break;

				case SERVO_YP:
					AmonDrone.actuators.servo_yp = test_angles[i];
					DegresToCCR(AmonDrone.actuators.servo_yp, SERVO_YP);
					break;

				case SERVO_YN:
					AmonDrone.actuators.servo_yn = test_angles[i];
					DegresToCCR(AmonDrone.actuators.servo_yn, SERVO_YN);
					break;
			}
		}

		HAL_Delay(500); // wait on motors to stop moving
	}
}


// Enabling EDF
uint8_t EDFEnable()
{
	uint8_t status = 0;
	AmonDrone.actuators.edf_enable = 1;
	AmonDrone.actuators.edf_percent = 0;
	PowerToPWMValue(AmonDrone.actuators.edf_percent);

	status += HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	return status;
}


// Disabling EDF
uint8_t EDFDisable()
{
	uint8_t status = 0;
	AmonDrone.actuators.edf_enable = 0;
	AmonDrone.actuators.edf_percent = 0;
	PowerToPWMValue(AmonDrone.actuators.edf_percent);

	status += HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

	return status;
}


// Enable 5V buck regulator
uint8_t enable_5v_buck(uint8_t pinState)
{
	if (pinState == 0)
	{
		HAL_GPIO_WritePin(EN_BUCK_5V_GPIO_Port, EN_BUCK_5V_Pin, GPIO_PIN_RESET);
		AmonDrone.data.buck_5v_enable = pinState;
		return pinState;
	}
	else if (pinState == 1)
	{
		HAL_GPIO_WritePin(EN_BUCK_5V_GPIO_Port, EN_BUCK_5V_Pin, GPIO_PIN_SET);
		AmonDrone.data.buck_5v_enable = pinState;
		return pinState;
	}
	else
	{
		return 2;
	}
}


// Enable 7.2V buck regulator
uint8_t enable_7v2_buck(uint8_t pinState)
{
	if (pinState == 0)
	{
		HAL_GPIO_WritePin(EN_BUCK_7V2_GPIO_Port, EN_BUCK_7V2_Pin, GPIO_PIN_RESET);
		AmonDrone.data.buck_7v2_enable = pinState;
		return pinState;
	}
	else if (pinState == 1)
	{
		HAL_GPIO_WritePin(EN_BUCK_7V2_GPIO_Port, EN_BUCK_7V2_Pin, GPIO_PIN_SET);
		AmonDrone.data.buck_7v2_enable = pinState;
		return pinState;
	}
	else
	{
		return 2;
	}
}


// Status RGB LED
void StatusLED(uint8_t Status)
{
	// TIM4 period is 23999. Higher CCR value means brighter RGB output.

	switch(Status){
	case STATUS_STARTUP: // STARTUP (red)
		TIM4->CCR3 = 0; // LED-RGB (blue)
		TIM4->CCR2 = 0; // LED-RGB (green)
		TIM4->CCR1 = RGB_BRIGHT; // LED-RGB (red)
		break;

	case STATUS_IDLE: // IDLE (blue)
		TIM4->CCR3 = RGB_BRIGHT; // LED-RGB (blue)
		TIM4->CCR2 = 0; // LED-RGB (green)
		TIM4->CCR1 = 0; // LED-RGB (red)
		break;

	case STATUS_ERROR: // ERROR (red + brd led on)
		TIM4->CCR3 = 0; // LED-RGB (blue)
		TIM4->CCR2 = 0; // LED-RGB (green)
		TIM4->CCR1 = RGB_BRIGHT; // LED-RGB (red)
		break;

	case STATUS_ARM: // ARM (yellow-green)
		TIM4->CCR3 = 0; // LED-RGB (blue)
		TIM4->CCR2 = RGB_MED; // LED-RGB (green)
		TIM4->CCR1 = RGB_MED; // LED-RGB (red)
		break;

	case STATUS_FLY: // FLY ()
		TIM4->CCR3 = RGB_MED; // LED-RGB (blue)
		TIM4->CCR2 = 0; // LED-RGB (green)
		TIM4->CCR1 = RGB_MED; // LED-RGB (red)
		break;

	case STATUS_FLY_OVER: // FLY OVER (green)
		TIM4->CCR3 = 0; // LED-RGB (blue)
		TIM4->CCR2 = RGB_BRIGHT; // LED-RGB (green)
		TIM4->CCR1 = 0; // LED-RGB (red)
		break;

	case STATUS_CALIB: // CALIBRATION - RGB
		TIM4->CCR3 = RGB_Blue; // LED-RGB (blue)
		TIM4->CCR2 = RGB_Green; // LED-RGB (green)
		TIM4->CCR1 = RGB_Red; // LED-RGB (red)
		break;

	default: // DEFAULT STATE
		TIM4->CCR3 = RGB_DIM; // LED-RGB (blue)
		TIM4->CCR2 = RGB_DIM; // LED-RGB (green)
		TIM4->CCR1 = RGB_DIM; // LED-RGB (red)
		break;
	}
}


// Reading voltage of main board battery
uint16_t ADC_Read_Main_Battery()
{
	uint16_t adcVal = adc_raw[1];
	float temp = ((float)adcVal * MAIN_BOARD_V) / 65536; // 4095
	float voltage = (R1_MAIN_BAT + R2_MAIN_BAT) * (temp / R2_MAIN_BAT);

	return (uint16_t)(voltage * 100);
}


// Reading voltage of EDF battery
uint16_t ADC_Read_EDF_Battery()
{
	uint16_t adcVal = adc_raw[0];
	float temp = ((float)adcVal * MAIN_BOARD_V) / 65536; // 4095
	float voltage = (((R1_EDF_BAT + R2_EDF_BAT) / R2_EDF_BAT) * temp);

	return (uint16_t)(voltage * 100);
}


// Reading voltage of 5V buck converter
uint16_t ADC_Read_5V_Buck()
{
	uint16_t adcVal = adc_raw[2];
	float temp = ((float)adcVal * MAIN_BOARD_V) / 65536; // 4095
	float voltage = (((R1_5V_BUCK + R2_5V_BUCK) / R2_5V_BUCK) * temp);

	return (uint16_t)(voltage * 100);
}


// Reading voltage of 7.2V buck converter
uint16_t ADC_Read_7V2_Buck()
{
	uint16_t adcVal = adc_raw[3];
	float temp = ((float)adcVal * MAIN_BOARD_V) / 65536; // 4095
	float voltage = (((R1_7V2_BUCK + R2_7V2_BUCK) / R2_7V2_BUCK) * temp);

	return (uint16_t)(voltage * 100);
}


// DMA data from ADC ready
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	SCB_InvalidateDCache_by_Addr((uint32_t*)adc_raw, 32); // sizeof(adc_raw)
	ADC_DMA_DataRdy = 1;
}


/* Regulator loop interrupt - 200Hz, 100Hz, 50Hz, 1Hz
 * 	- Read sensors
 * 	- Calculate TVC
 * 	- Regulate servos
 * 	- send over RF
 * 	- save to flash
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	/* TIMER 1 - 100Hz */
	if (htim->Instance == TIM1)
	{
		IRQ_NMPCLoopEN = 1;
	}


	/* TIMER 5 - 50Hz */
	if (htim->Instance == TIM5)
	{

		IRQ_50HzLoopEN = 1;

#ifndef CALIBRATION

		// Idle - single blink
		if (DroneStatusLocal != STATUS_STARTUP && DroneStatusLocal != STATUS_ERROR && DroneStatusLocal != STATUS_FLY)
		{
			if (LED_blink_cnt_ON < 50) // LED OFF
			{
				LED_blink_cnt_ON++;
			}
			else
			{
				if (LED_blink_cnt_OFF < 5) // LED ON for short time
				{
					if (LED_blink_cnt_OFF == 0)
					{
						HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_SET);
					}
					LED_blink_cnt_OFF++;
				}
				else
				{
					HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_RESET);
					LED_blink_cnt_OFF = 0;
					LED_blink_cnt_ON = 0;
				}
			}
		}


		// Fly - dual blink
		if (DroneStatusLocal != STATUS_STARTUP && DroneStatusLocal != STATUS_ERROR && DroneStatusLocal == STATUS_FLY)
		{
			if (LED_blink_cnt_ON < 50) // LED OFF
			{
				LED_blink_cnt_ON++;
			}
			else
			{
				if (LED_blink_cnt_OFF < 10) // LED ON for short time
				{
					if (LED_blink_cnt_OFF <3)
					{
						HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_SET);
					}

					if (LED_blink_cnt_OFF >=3 && LED_blink_cnt_OFF <7)
					{
						HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_RESET);
					}

					if (LED_blink_cnt_OFF >=7)
					{
						HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_SET);
					}

					LED_blink_cnt_OFF++;
				}
				else
				{
					HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_RESET);
					LED_blink_cnt_OFF = 0;
					LED_blink_cnt_ON = 0;
				}
			}
		}

#endif


#ifdef CALIBRATION
		/* Calibration RGB cycling */
		if (GyroCalibTrig == 0)
		{
			if (LED_blink_cnt_ON >= 5)
			{
				LED_blink_cnt_ON = 0;
			}
			LED_blink_cnt_ON++;
		}
#endif

	} // TIM5



	/* TIMER 6 - 200Hz */

	if (htim->Instance == TIM6)
	{
		IRQ_200HzLoopEN = 1;


#ifdef CALIBRATION

		/* GYRO Calibration */
		if (GyroCalibTrig == 0)
		{
			GyroCalibTrig = 1;
		}

#endif


		//static uint16_t value = 0;
		static uint8_t phase = 0;

		switch (phase)
		{
		    case 0: // Red -> Yellow (increase Green)
		        RGB_Red   = RGB_MAX;
		        RGB_Green += STEP;
		        RGB_Blue  = 0;
		        if (RGB_Green >= RGB_MAX)
		        {
		            RGB_Green = RGB_MAX;
		            phase = 1;
		        }
		        break;

		    case 1: // Yellow -> Green (decrease Red)
		        RGB_Red   -= STEP;
		        RGB_Green = RGB_MAX;
		        RGB_Blue  = 0;
		        if (RGB_Red == 0)
		            phase = 2;
		        break;

		    case 2: // Green -> Cyan (increase Blue)
		        RGB_Red   = 0;
		        RGB_Green = RGB_MAX;
		        RGB_Blue  += STEP;
		        if (RGB_Blue >= RGB_MAX)
		        {
		            RGB_Blue = RGB_MAX;
		            phase = 3;
		        }
		        break;

		    case 3: // Cyan -> Blue (decrease Green)
		        RGB_Red   = 0;
		        RGB_Green -= STEP;
		        RGB_Blue  = RGB_MAX;
		        if (RGB_Green == 0)
		            phase = 4;
		        break;

		    case 4: // Blue -> Magenta (increase Red)
		        RGB_Red   += STEP;
		        RGB_Green = 0;
		        RGB_Blue  = RGB_MAX;
		        if (RGB_Red >= RGB_MAX)
		        {
		            RGB_Red = RGB_MAX;
		            phase = 5;
		        }
		        break;

		    case 5: // Magenta -> Red (decrease Blue)
		        RGB_Red   = RGB_MAX;
		        RGB_Green = 0;
		        RGB_Blue  -= STEP;
		        if (RGB_Blue == 0)
		            phase = 0;
		        break;
		}

	} // TIM6



	/* TIMER 7 - 1Hz */
	if (htim->Instance == TIM7)
	{
		IRQ_1HzLoopEN = 1;
	} // TIM7

}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
