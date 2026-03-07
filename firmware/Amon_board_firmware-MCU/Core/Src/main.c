/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RGB_MAX 500      // match your PWM max
#define STEP    1        // speed of transition


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Functions init */
uint16_t ADC_Read_Main_Battery();
uint16_t ADC_Read_EDF_Battery();
uint8_t TVCServoEnable();
uint8_t TVCServoDisable();
uint8_t EDFEnable();
uint8_t EDFDisable();
void StatusLED(uint8_t Status);



/* Variables */
uint8_t DroneStatusLocal = 0;
uint8_t DroneStatusOld = 20;
uint8_t StartupInit = 0;
uint8_t InitError = 0;

uint8_t LED_blink_cnt_ON;
uint8_t LED_blink_cnt_double;
uint8_t LED_blink_cnt_OFF;

uint8_t TVCServoEnableFlag = 0;
uint8_t EDFEnableFlag = 0;

uint8_t RF_IRQ1_EN = 0;			// flag to enable irq code
uint8_t RF_IRQ2_EN = 0;			// flag to enable irq code

float PitchDef[20] = {0};		// Default values for calculating angle before flight
float RollDef[20] = {0};
uint8_t  MeassCnt = 0;			// counting measurements

// Gyroscope calibration
uint8_t  GyroCalibTrig = 0;
uint16_t RGB_Red = 0;
uint16_t RGB_Green = 300;
uint16_t RGB_Blue = 600;
uint16_t RGB_RedMax = 0;
uint16_t RGB_GreenMax = 0;
uint16_t RGB_BlueMax = 0;

// Battery data
uint8_t  ADC_DMA_DataRdy = 0;
uint16_t ADC_BAT_Val[2] = {};

// GPS data
uint8_t  USART4_GPSRX_DMA[426] = {"\0"};
uint8_t  NewGPSData = 0;

// Timers IRQ enable flag
uint8_t  Reg1HzLoopEN = 0;
uint8_t  Reg50HzLoopEN = 0;
uint8_t  Reg200HzLoopEN = 0;



/* STRUCTS */
s_MPU6050 			mpu6050; 		// MPU6050 device driver
s_BME280 			bme280;			// bme280 device driver
VL53L1_DEV 			vl53l1Dev;		// VL53L1 device driver
//VL53L1X_Version_t vl53l1xVersion_t; // Lidar - unused
//VL53L1X_Result_t vl53l1xResult_t; // Lidar - unused
s_nRF24L01 			radio1;			// nRF24L01 device driver
s_nRF24L01 			radio2;			// nRF24L01 device driver
s_drone_data 		AmonDrone; 		// All drone data
s_packets 			data_packets;	// Data structure for communication packets

s_Kalman			kalman_pitch;	// Kalman values for pitch
s_Kalman			kalman_roll;	// Kalman values for roll
s_Kalman			kalman_yaw;		// Kalman values for yaw


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  MX_FATFS_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */


  // Init
  AmonDrone.DroneStatus = STATUS_STARTUP;
  AmonDrone.flight_status = STATUS_FLIGHT_GROUND;

  // Radio 1 configurations (application specific)
  static const s_nrf_config radio_tx_normal_cfg = {
      .channel = 100,	//42
      .addr_width = AW_5BYTE,
      .auto_ack = 1,
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
      .auto_ack = 1,
      .dynamic_payload = 1,
      .retries = 0,
      .retry_delay = 0,
      .datarate = NRF_DATARATE_1MBPS,
      .power = NRF_POWER_MINUS_6DBM,
  };

  static const s_nrf_config radio_rx_stream_cfg = {
      .channel = 70,
      .addr_width = AW_5BYTE,
      .auto_ack = 1,
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
		  // TX
		  NRF24_HandleIRQ(&radio1);
		  radio1.buffers.pipe_data = radio1.irq_on_pipe;
		  radio1.irq_flag = 0;
		  radio1.irq_on_pipe = 0xFF;
		  if (radio1.role == NRF_ROLE_PRX) AmonDrone.radio_data.connection_timeout = 0;	// Reset timeout timer
	  }

	  if (radio2.irq_flag == 1)
	  {
		  // RX
		  NRF24_HandleIRQ(&radio2);
		  radio2.buffers.pipe_data = radio2.irq_on_pipe;
		  radio2.irq_flag = 0;
		  radio2.irq_on_pipe = 0xFF;
		  if (radio2.role == NRF_ROLE_PRX) AmonDrone.radio_data.connection_timeout = 0;	// Reset timeout timer
	  }


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
				  radio1.config = &radio_tx_normal_cfg;
				  NRF24_init(&radio1);
				  HAL_Delay(10);
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
				radio1.config = &radio_tx_normal_cfg;
				NRF24_init(&radio1);
				HAL_Delay(10);
			}

			// Detect successful connection
			if (AmonDrone.radio_data.flag_connection_begin == 1)
			{
				AmonDrone.radio_data.conn_status = CONN_STATUS_CONNECTED;
				HAL_GPIO_WritePin(LED_Brd_GPIO_Port, LED_Brd_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(LED_Brd_GPIO_Port, LED_Brd_Pin, GPIO_PIN_RESET);
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


	  /* USB TRANSCODING HANDLING */
	  if (AmonDrone.uart_buffer.flag_new_uart_rx_data == 1)
	  {
		  AmonDrone.uart_buffer.flag_new_uart_rx_data = 0;

		  uint8_t ret = UART_decode(AmonDrone.uart_buffer.buffer_UART, &data_packets, &AmonDrone.uart_buffer.flag_new_uart_tx_data);
		  memset(AmonDrone.uart_buffer.buffer_UART, 0, sizeof(AmonDrone.uart_buffer.buffer_UART));

		  // Based on return values trigger events
		  switch (ret)
		  {
			  case TRANSCODE_BOOT_PKT:
				  // Unused
				  break;

			  case TRANSCODE_BROADCAST:
				  // Unused
				  break;

			  case TRANSCODE_VER_ERR:                 // Wrong version of packet
				  break;

			  case TRANSCODE_DEST_ERR:                // Wrong destination address
				  break;

			  case TRANSCODE_CRC_ERR:                 // Corupted frame / CRC
				  break;

			  case TRANSCODE_DEST_RF:                 // Packet for RF transmit
				  // Unused
				  break;

			  case TRANSCODE_DEST_LINK:               // Packet for link device
				  // Unused
				  break;

			  default:                                // Default state
				  break;
		  }
	  }

/*##############################################################################################################################################
#################################################################### TIMERS ####################################################################
##############################################################################################################################################*/


	  /*##### TIMERS - Timer flag triggers #####*/
	  if (AmonDrone.DroneStatus != STATUS_STARTUP ) // && AmonDrone.DroneStatus != STATUS_ERROR
	  {

		  // --- Timer 4 - 200Hz / 5ms ---
		  if (Reg200HzLoopEN == 1)
		  {
			  Reg200HzLoopEN = 0;

			  MPU6050_ReadAllDirect(&mpu6050, &hi2c3);

#ifndef GYRO_KALMAN
			  // Complementary filter
			  Complementary_deg(&mpu6050, &AmonDrone); // calculate data to pitch and roll (and yaw)
#else
			  // Kalman Filter - angles and axis described in filters.h
			  float roll_angle_accel  = 0;
			  float pitch_angle_accel = 0;

			  Kalman_rawToAngles(&mpu6050, &roll_angle_accel, &pitch_angle_accel);

			  // Unwrap accel based on current estimation
			  roll_angle_accel  = unwrap_to_ref(roll_angle_accel,  kalman_roll.angle);
			  pitch_angle_accel = unwrap_to_ref(pitch_angle_accel, kalman_pitch.angle);

			  AmonDrone.position.Pitch = Kalman_Update(&kalman_pitch, mpu6050.GYRO_Y, pitch_angle_accel, DT);
			  AmonDrone.position.Roll = Kalman_Update(&kalman_roll, mpu6050.GYRO_Z, roll_angle_accel, DT);
			  //AmonDrone.position.Yaw += mpu6050.GYRO_X * DT; // Yaw (gyro only)
			  AmonDrone.position.Yaw = mpu6050.GYRO_X * DT; // Yaw (gyro only)
#endif

			  // Save raw data for telemetry
			  AmonDrone.position.gyroTemp = (uint16_t)mpu6050.Temp_C;
			  AmonDrone.position.accel_x = mpu6050.ACCEL_X;
			  AmonDrone.position.accel_y = mpu6050.ACCEL_Y;
			  AmonDrone.position.accel_z = mpu6050.ACCEL_Z;
			  AmonDrone.position.gyro_x = mpu6050.GYRO_X;
			  AmonDrone.position.gyro_y = mpu6050.GYRO_Y;
			  AmonDrone.position.gyro_z = mpu6050.GYRO_Z;

		  } // TIMER 200Hz



		  // --- Timer 5 - 50Hz / 20ms ---
		  if (Reg50HzLoopEN == 1)
		  {
			  Reg50HzLoopEN = 0;

			  static uint8_t dataRdy = 0;
//			  while(dataRdy == 0)
//			  {
//				  VL53L1X_CheckForDataReady(&vl53l1Dev, &hi2c3, &dataRdy);
//			  }
//			  dataRdy = 0;
			  VL53L1X_CheckForDataReady(&vl53l1Dev, &hi2c3, &dataRdy);
			  if (dataRdy)
			  {
				  VL53L1X_GetDistance(&vl53l1Dev, &hi2c3, &AmonDrone.position.height_TOF_mm);
				  VL53L1X_ClearInterrupt(&vl53l1Dev, &hi2c3);
			  }

			  // BME280
			  BME280_ReadAllData(&bme280, &hi2c3);
			  BME280_PressToAlt(&bme280, &hi2c3);
			  AmonDrone.data.temperature = (uint16_t)bme280.Temp_C;
			  AmonDrone.data.humidity = (uint8_t)bme280.Hum_Perc;
			  AmonDrone.data.pressure = bme280.Press_Pa;
			  AmonDrone.position.height_baro_m = bme280.altitude_m;

			  // Start ADC DMA (read analog value)
			  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_BAT_Val, 2);

			  // Send telemetry packet if stream mode is on - test
			  if (AmonDrone.radio_data.flag_stream_data)
			  {
				  AmonDrone.radio_data.flag_telemetry_send = 1;
			  }

			  // Logging
//			  if (AmonDrone.DroneStatus == STATUS_ARM || AmonDrone.DroneStatus == STATUS_FLY) log_add_sample(&AmonDrone.position, &AmonDrone.data);

		  } // TIMER 50Hz



		  // --- Timer 6 - 1Hz / 1sec ---
		  if (Reg1HzLoopEN == 1)
		  {
			  Reg1HzLoopEN = 0;

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
					  // Turn ON stream data mode
					  AmonDrone.radio_data.flag_stream_data = 1;
					  radio1.config = &radio_tx_stream_cfg;
					  NRF24_init(&radio1);
					  HAL_Delay(10);
				  }
			  }
		  } // TIMER 1Hz



		  // --- ADC read - DMA trigger ---
		  if (ADC_DMA_DataRdy)
		  {
			  for (uint8_t i = 1; i < 10; i++)
			  {
				  AmonDrone.data.bat_main_v[i-1] = AmonDrone.data.bat_main_v[i];
				  AmonDrone.data.bat_edf_v[i-1] = AmonDrone.data.bat_edf_v[i];
			  }

			  AmonDrone.data.bat_main_v[9] = ADC_Read_Main_Battery();
			  AmonDrone.data.bat_edf_v[9] = ADC_Read_EDF_Battery();

			  uint16_t main_v_temp = 0;
			  uint16_t edf_v_temp = 0;
			  for (uint8_t i = 0; i < 10; i++)
			  {
				  main_v_temp += AmonDrone.data.bat_main_v[i];
				  edf_v_temp += AmonDrone.data.bat_edf_v[i];
			  }

			  AmonDrone.data.battery_main_voltage = main_v_temp / (sizeof(AmonDrone.data.bat_main_v) / sizeof(uint16_t));
			  AmonDrone.data.battery_edf_voltage = edf_v_temp / (sizeof(AmonDrone.data.bat_edf_v) / sizeof(uint8_t));

			  ADC_DMA_DataRdy = 0;
		  }



/*##############################################################################################################################################
##################################################################### GNSS #####################################################################
##############################################################################################################################################*/

		  /*##### GNSS/GPS - New packet available #####*/
#ifdef USE_GPS

		  if (NewGPSData == 1)
		  {
			  NewGPSData = 0;
			  memcpy(USART4_GPSRX_DMA, AmonDrone.gps_data.GPS_RX_buffer, sizeof(USART4_GPSRX_DMA));
			  memset(USART4_GPSRX_DMA, 0, sizeof(USART4_GPSRX_DMA));

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
#endif


	  } // TIMERS / Boot done


/*#####################################################################################################################################################
###################################################################### INIT & CODE ####################################################################
#####################################################################################################################################################*/


	  /********************************************************
	   ******************** INIT  SEQUENCE ********************
	   ********************************************************/
	  if (StartupInit == STATUS_STARTUP)
	  {

		//AmonDrone.DroneStatus = STATUS_STARTUP;
		uint8_t status = 0;

		/* Align all motors */
		// Set motors on neutral position
		DegresToCCR(0, SERVO_XN);
		DegresToCCR(0, SERVO_XP);
		DegresToCCR(0, SERVO_YN);
		DegresToCCR(0, SERVO_YP);
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
						DegresToCCR(test_angles[i], SERVO_XP);
						break;

					case SERVO_XN:
						DegresToCCR(test_angles[i], SERVO_XN);
						break;

					case SERVO_YP:
						DegresToCCR(test_angles[i], SERVO_YP);
						break;

					case SERVO_YN:
						DegresToCCR(test_angles[i], SERVO_YN);
						break;
				}
			}

			HAL_Delay(500); // wait on motors to stop moving
		}

		// Start timers for sensors and LEDs
		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_Brd_GPIO_Port, LED_Brd_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // RGB (50Hz)
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // RGB (50Hz)
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // RGB (50Hz)

		/* Read both batteries and save in drone data struct */
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_BAT_Val, 2); 		// Start ADC DMA (read analog value)

		while(ADC_DMA_DataRdy == 0);
		AmonDrone.data.battery_main_voltage = ADC_Read_Main_Battery();
		AmonDrone.data.battery_edf_voltage = ADC_Read_EDF_Battery();
		ADC_DMA_DataRdy = 0;

		// TODO: set voltage
		//if (AmonDrone.battery_main_voltage < 1000) status++; // check board battery voltage (more than XV)
		//if (AmonDrone.battery_edf_voltage < 2000) status++; // check board battery voltage (more than XV)

		HAL_Delay(500);

		/* Reset all devices */
		status += BME280_Reset(&bme280, &hi2c3);
		status += MPU6050_Reset(&mpu6050, &hi2c3);

		HAL_Delay(500); // delay sensors config to complete power on

		/* BME280 */
		status += BME280_ReadDeviceID(&bme280, &hi2c3);
		status += BME280_ReadCalibData(&bme280, &hi2c3);
		status += BME280_Init(&bme280, &hi2c3);
		HAL_Delay(100);
		status += BME280_ReadAllData(&bme280, &hi2c3);
		if (AmonDrone.data.take_off_alt_m == 0) AmonDrone.data.take_off_alt_m = ALTITUDE_M;
		status += BME280_Altitude_Init(&bme280, &hi2c3, AmonDrone.data.take_off_alt_m);

		/* MPU6050 */
		status += MPU6050_ReadDeviceID(&mpu6050, &hi2c3);
		status += MPU6050_ReadFactoryTrim(&mpu6050, &hi2c3);
		status += MPU6050_Init(&mpu6050, &hi2c3);
		status += MPU6050_ReadFactoryTrim(&mpu6050, &hi2c3);
		status += MPU6050_ReadAllDirect(&mpu6050, &hi2c3);
		status += MPU6050_SelfTest(&mpu6050, &hi2c3);

#ifdef GYRO_KALMAN
		Kalman_Init(&kalman_pitch);
		Kalman_Init(&kalman_roll);
		Kalman_Init(&kalman_yaw);
#endif

		/* vl53l1x */
		uint8_t bootOK = 0;
		while (bootOK == 0)
		{
			status += VL53L1X_BootState(&vl53l1Dev, &hi2c3, &bootOK);
		}
		status += VL53L1X_ReadID(&vl53l1Dev, &hi2c3);
		status += VL53L1X_SensorInit(&vl53l1Dev, &hi2c3);
		status += VL53L1X_SetTimingBudgetInMs(&vl53l1Dev, &hi2c3, 200); // 140ms is min for 4m distance
		status += VL53L1X_SetOffset(&vl53l1Dev, &hi2c3, -121); // Set height from ground to get zero -130
		VL53L1X_StartRanging(&vl53l1Dev, &hi2c3);

		/* NRF24L01 */
		radio1.op_modes = NRF_MODE_PWR_ON_RST;              // set default radio state
		radio2.op_modes = NRF_MODE_PWR_ON_RST;              // set default radio state
		AmonDrone.radio_data.conn_status = CONN_STATUS_DISCONNECTED;
		AmonDrone.radio_data.flag_connection_lost = 0;
		AmonDrone.radio_data.flag_connection_begin = 0;
		radio1.irq_on_pipe = 0xFF;
		radio2.irq_on_pipe = 0xFF;
		AmonDrone.radio_data.flag_stream_data = 0;

		// Radios initialization and setup
		NRF24_pin_config(&radio1, &hspi1, CS_RF1_GPIO_Port, CS_RF1_Pin, EN_RF1_GPIO_Port, EN_RF1_Pin);        // Map pins for radio 1
		NRF24_pin_config(&radio2, &hspi1, CS_RF2_GPIO_Port, CS_RF2_Pin, EN_RF2_GPIO_Port, EN_RF2_Pin);        // Map pins for radio 2

		HAL_Delay(10);
		// Read status before even test device - strange SPI behaver
		NRF24_ReadStatus(&radio1, NULL);
		NRF24_ReadStatus(&radio2, NULL);


		uint8_t stat = 0;
		if (NRF24_ReadStatus(&radio1, &stat) == 0)
		{
			if (stat != 0x0E)
			{
				radio1.radioErr = NRF_ERR_BOOT;
				AmonDrone.error_code.err_radio1 = 1;
			}
			else
			{
				radio1.radioErr = NRF_ERR_NONE;
				radio1.op_modes = NRF_MODE_PWR_DOWN;
			}
		}

		if (NRF24_ReadStatus(&radio2, &stat) == 0)
		{
			if (stat != 0x0E)
			{

				radio2.radioErr = NRF_ERR_BOOT;
				AmonDrone.error_code.err_radio2 = 1;
			}
			else
			{
				radio2.radioErr = NRF_ERR_NONE;
				radio2.op_modes = NRF_MODE_PWR_DOWN;
			}
		}

		if (radio1.radioErr == 1 || radio2.radioErr == 1) status++;

		// Set radio configurations and init
		radio1.role     = NRF_ROLE_PTX;
		radio1.config   = &radio_tx_normal_cfg;
		radio1.address  = &radio_tx_addr;
		radio1.id       = NRF_ID_1;
		NRF24_init(&radio1);
		NRF24_SetTXAddress(&radio1, radio1.address->tx_addr);

		radio2.role     = NRF_ROLE_PRX;
		radio2.config   = &radio_rx_normal_cfg;
		radio2.address  = &radio_rx_addr;
		radio2.id       = NRF_ID_2;
		NRF24_init(&radio2);
		NRF24_SetRXAddress(&radio2, 0, radio2.address->pipe0_rx_addr);


		/* Timers */
		HAL_TIM_Base_Start_IT(&htim4); // TVC LOOP, leg leds (50Hz)
		HAL_TIM_Base_Start_IT(&htim5); // Complementary Filter
		HAL_TIM_Base_Start_IT(&htim6); // Second timer - time

#ifndef CALIBRATION
	#ifdef USE_GPS
		// Do not enable UART DMA if calibrating
		HAL_UART_Receive_DMA(&huart4, USART4_GPSRX_DMA, sizeof(USART4_GPSRX_DMA)); //426
	#else
		memset(AmonDrone.uart_buffer.buffer_temp, 0, sizeof(AmonDrone.uart_buffer.buffer_temp));
		memset(AmonDrone.uart_buffer.buffer_UART, 0, sizeof(AmonDrone.uart_buffer.buffer_UART));
		HAL_UART_Receive_IT(&huart4, AmonDrone.uart_buffer.buffer_temp, 1);
	#endif

#endif


		/* Logging */
#ifdef LOG_ENABLE

//		Flash_Init();     	// if not inside LOG_Init
//		log_init();       	// mount filesystem

		// test
//		log_test_write();  	// write test file
//		log_test_read();	// read test file back

#endif

		if (bme280.dig_T1 == 0 || bme280.dig_T2 == 0) // for WTF error
		{
			status++;
		}

		InitError = status;

		// Check if all init functions are OK
		if (InitError == 0) // OK
		{
			StartupInit = 1;
			AmonDrone.DroneStatus = STATUS_IDLE;
		}
		else
		{
			StartupInit = 1;
			AmonDrone.DroneStatus = STATUS_ERROR; // ERROR
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

				  if (TVCServoEnableFlag == 1) TVCServoDisable();	// Disable TVC servos
				  if (EDFEnableFlag == 1) EDFDisable();				// Disable EDF

				  break;


			  // STATE: Drone armed and ready to take off
			  case STATUS_ARM:

				  //TODO: Check sensor values and check flying path
				  if (TVCServoEnableFlag == 0) TVCServoEnable();
				  if (EDFEnableFlag == 0) EDFEnable();


				  break;


			  // STATE: Flying
			  case STATUS_FLY:

				  // TODO: Regulator
				  switch (AmonDrone.flight_status) {
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

				  if (TVCServoEnableFlag == 1) TVCServoDisable();
				  if (EDFEnableFlag == 1) EDFDisable();


				  break;


			  // STATE: Wrong state
			  default:

				  AmonDrone.DroneStatus = STATUS_ERROR;
				  TVCServoDisable();
				  EDFDisable();

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
	   * On GPS port connect FTDI module to read UART communication. DO NOT CONNECT POWER PIN (3.3V) IF YOU ARE PLANNING TO POWER THE BOARD FROM BATTERY !!!
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
	  	  	  HAL_UART_Transmit(&huart4, (uint8_t*)message, strlen((char*)message), 100);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8399;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  htim3.Init.Prescaler = 83;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim5.Init.Prescaler = 4199;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Brd_GPIO_Port, LED_Brd_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_Flash_GPIO_Port, CS_Flash_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_Ext_Pin|CS_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Red_Pin|LED_White_Pin|CS_RF2_Pin|EN_RF2_Pin
                          |EN_RF1_Pin|CS_RF1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Brd_Pin CS_Flash_Pin */
  GPIO_InitStruct.Pin = LED_Brd_Pin|CS_Flash_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Ext_Pin CS_SD_Pin */
  GPIO_InitStruct.Pin = CS_Ext_Pin|CS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Red_Pin LED_White_Pin CS_RF2_Pin EN_RF2_Pin
                           EN_RF1_Pin CS_RF1_Pin */
  GPIO_InitStruct.Pin = LED_Red_Pin|LED_White_Pin|CS_RF2_Pin|EN_RF2_Pin
                          |EN_RF1_Pin|CS_RF1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_IRQ2_Pin */
  GPIO_InitStruct.Pin = RF_IRQ2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_IRQ2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_IRQ1_Pin */
  GPIO_InitStruct.Pin = RF_IRQ1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_IRQ1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


// UART interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

#ifdef USE_GPS

	HAL_UART_Receive_DMA(&huart4, USART4_GPSRX_DMA, 426); // enable USART Receive again
	NewGPSData = 1;		// set flag that new data has arrived

#else

	static uint8_t len_new_rx_data = 0;
	static uint8_t cntBuffer_UART = 0;

	// Save received data
	uint8_t data = AmonDrone.uart_buffer.buffer_temp[0];                   // Read only once
	AmonDrone.uart_buffer.buffer_UART[cntBuffer_UART] = data;
	memset(AmonDrone.uart_buffer.buffer_temp, 0, 2);
	cntBuffer_UART++;

	// Detect start of frame and set flags
	if (data == SIG_SOF && len_new_rx_data == 0)    						// No flag for new packet and SOA packet
	{
		AmonDrone.uart_buffer.flag_USB_RX_new = 1;                          // Indicate new data received
		AmonDrone.uart_buffer.flag_new_uart_rx_data = 0;                    // Clear end of packet flag
		len_new_rx_data = 0;                                                // Clear packet counter
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

	HAL_UART_Receive_IT(&huart4, AmonDrone.uart_buffer.buffer_temp, 1);		// Re-arm UART interrupt

#endif


}


// GPIO interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == RF_IRQ1_Pin)
    {
    	radio1.irq_flag = 1;
    }

    if(GPIO_Pin == RF_IRQ2_Pin)
	{
    	radio2.irq_flag = 1;
	}
}


// Enabling servos for TVC stabilization
uint8_t TVCServoEnable()
{
	uint8_t status = 0;
	TVCServoEnableFlag = !TVCServoEnableFlag;

	status += HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	status += HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	status += HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	status += HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	return status;
}


// Disabling servos for TVC stabilization
uint8_t TVCServoDisable()
{
	uint8_t status = 0;
	EDFEnableFlag = !EDFEnableFlag;

	status += HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	status += HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	status += HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	status += HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

	return status;
}


// Enabling EDF
uint8_t EDFEnable()
{
	uint8_t status = 0;

	status += HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	return status;
}


// Disabling EDF
uint8_t EDFDisable()
{
	uint8_t status = 0;

	status += HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

	return status;
}


// Status RGB LED
void StatusLED(uint8_t Status)
{
	// OFF: R=2400, G=0, B=0
	// ON (bright): 500
	// ON (medium): 1000

	switch(Status){
	case STATUS_STARTUP: // STARTUP (red)
		TIM1->CCR2 = 0; // LED-RGB (blue)
		TIM1->CCR3 = 0; // LED-RGB (green)
		TIM2->CCR3 = 500; // LED-RGB (red)
		break;

	case STATUS_IDLE: // IDLE (blue)
		TIM1->CCR2 = 1000; // LED-RGB (blue)
		TIM1->CCR3 = 0; // LED-RGB (green)
		TIM2->CCR3 = 0; // LED-RGB (red)
		break;

	case STATUS_ERROR: // ERROR (red + brd led on)
		TIM1->CCR2 = 0; // LED-RGB (blue)
		TIM1->CCR3 = 0; // LED-RGB (green)
		TIM2->CCR3 = 1000; // LED-RGB (red)
		break;

	case STATUS_ARM: // ARM (yellow-green)
		TIM1->CCR2 = 0; // LED-RGB (blue)
		TIM1->CCR3 = 100; // LED-RGB (green)
		TIM2->CCR3 = 100; // LED-RGB (red)
		break;

	case STATUS_FLY: // FLY ()
		TIM1->CCR2 = 100; // LED-RGB (blue)
		TIM1->CCR3 = 0; // LED-RGB (green)
		TIM2->CCR3 = 100; // LED-RGB (red)
		break;

	case STATUS_FLY_OVER: // FLY OVER (green)
		TIM1->CCR2 = 0; // LED-RGB (blue)
		TIM1->CCR3 = 100; // LED-RGB (green)
		TIM2->CCR3 = 500; // LED-RGB (red)
		break;

	case STATUS_CALIB: // CALIBRATION - RGB
		TIM1->CCR2 = RGB_Blue; // LED-RGB (blue)
		TIM1->CCR3 = RGB_Green; // LED-RGB (green)
		TIM2->CCR3 = RGB_Red; // LED-RGB (red)
		break;

	default: // DEFAULT STATE
		TIM1->CCR2 = 500; // LED-RGB (blue)
		TIM1->CCR3 = 500; // LED-RGB (green)
		TIM2->CCR3 = 500; // LED-RGB (red)
		break;
	}
}


// Reading voltage of main board battery
uint16_t ADC_Read_Main_Battery()
{
	uint16_t adcVal = ADC_BAT_Val[1];
	float temp = ((float)adcVal * MAIN_BOARD_V) / 4095;
	float voltage = (R1_MAIN_BAT + R2_MAIN_BAT) * (temp / R2_MAIN_BAT);

	return (uint16_t)(voltage*100);
}


// Reading voltage of EDF battery
uint16_t ADC_Read_EDF_Battery()
{
	uint16_t adcVal = ADC_BAT_Val[0];
	float temp = ((float)adcVal * 3.3) / 4095;
	float voltage = (((100000+10000)/10000) * temp);

	return (uint16_t)(voltage*100);
}


// DMA data from ADC ready
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	ADC_DMA_DataRdy = 1;
}


/* Regulator loop interrupt - 50Hz
 * 	- Read sensors
 * 	- Calculate TVC
 * 	- Regulate servos
 * 	- send over RF
 * 	- save to flash
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	/* TIMER 4 - 50Hz */
	if (htim->Instance == TIM4)
	{

		Reg50HzLoopEN = 1;

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
						HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_SET);
					}
					LED_blink_cnt_OFF++;
				}
				else
				{
					HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_RESET);
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
						HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_SET);
					}

					if (LED_blink_cnt_OFF >=3 && LED_blink_cnt_OFF <7)
					{
						HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_RESET);
					}

					if (LED_blink_cnt_OFF >=7)
					{
						HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_SET);
					}

					LED_blink_cnt_OFF++;
				}
				else
				{
					HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_RESET);
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

	} // TIM4



	/* TIMER 5 - 200Hz */

	if (htim->Instance == TIM5)
	{
		Reg200HzLoopEN = 1;


#ifdef CALIBRATION

		/* GYRO Calibration */
		if (GyroCalibTrig == 0)
		{
			GyroCalibTrig = 1;
		}

#endif


		static uint16_t value = 0;
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

	} // TIM5



	/* TIMER 6 - 1Hz */
	if (htim->Instance == TIM6)
	{
		Reg1HzLoopEN = 1;
	} // TIM6
}

/* USER CODE END 4 */

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
