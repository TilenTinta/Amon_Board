/*****************************************************************
 * File Name          : DroneData.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2023/05/17
 * Description        : All drone data, variables and data logic
*****************************************************************/

#include "drone_data.h"

/*###########################################################################################################################################################*/
/* Private functions */
static void decode_opcode(s_packets *packets, s_drone_data *drone_data);
static void packet_create_pair_start(s_packets *packets);
static void packet_create_pair_status(s_packets *packets, s_drone_data *drone_data);
static void packet_create_ping_pong(s_packets *packets);



/*********************************************************************
 * @fcn     RF_packet_decode
 *
 * @param *packets: pointer to received data over RF
 * @param *drone_data: pointer to struct of all drone data
 *
 * @brief   Decode payload received over RF
 *
 * @return  error code - watch defines
 */
uint8_t RF_packet_decode(s_packets *packets, s_drone_data *drone_data)
{
	switch(packets->rf_packet.flags){

	case FLAG_ACK:
		// Replay packet - TBD
		break;

	case FLAG_ERR:
		// Error packet - TBD
		break;

	case FLAG_STREAM:
		// Stream packet - Drone -> PC direction
		break;

	case FLAG_FRAG:
		// Fragmented packet - TBD
		break;

	case FLAG_DATA:
		decode_opcode(packets, drone_data);
		drone_data->radio_data.flag_new_rf_tx_data = 1;
		break;

	default:
		// Unused
		break;
	}



    return 0;
}



/*********************************************************************
 * @fcn     decode_opcode
 *
 * @param *opcode: pointer to struct of all data packets
 * @param *drone_data: pointer to struct of all drone data
 *
 * @brief   Decode opcode
 *
 * @return  none
 */
static void decode_opcode(s_packets *packets, s_drone_data *drone_data)
{

	switch(packets->rf_packet.opcode){

		case OPT_NOP:
			// NOP / Reserved
			break;

		case OPT_PING:
			packet_create_ping_pong(packets);
			break;

		case OPT_ERROR_REPORT:
			//
			break;

		case OPT_PAIR_STATUS:
			packet_create_pair_status(packets, drone_data);
			if (drone_data->radio_data.flag_connection_begin == 2)
			{
				drone_data->radio_data.flag_connection_begin = 1;
				drone_data->radio_data.flag_connection_lost = 0;
			}

			break;

		case OPT_PAIR_START:
			packet_create_pair_start(packets);
#ifdef CONN_STEPS_2
			drone_data->radio_data.flag_connection_begin = 2;
#else
			drone_data->radio_data.flag_connection_begin = 1;
#endif
			break;

		case OPT_LINK_GET_PARAMS:
			// Wrong device
			break;

		case OPT_LINK_SET_PARAMS:
			// Wrong device
			break;

		case OPT_DRONE_GET_PARAMS:
			//
			break;

		case OPT_DRONE_SET_PARAMS:
			//
			break;

		case OPT_DRONE_SET_STATE:
			//
			break;

		case OPT_DRONE_COMMAND:
			//
			break;

		case OPT_TELEMETRY:
			// Drone -> PC direction
			break;

		default:
			// Default
			break;

	}
}



/*********************************************************************
 * @fcn   	packet_create_pair_start
 *
 * @param 	*packets: pointer to all data packets
 *
 * @brief   Assemble packet for pairing ACK
 *
 * @return  none
 */
static void packet_create_pair_start(s_packets *packets)
{
	packets->rf_packet_drone.version = PROTOCOL_VER;
	packets->rf_packet_drone.flags = FLAG_ACK;
	packets->rf_packet_drone.src_id = ID_DRONE;
	packets->rf_packet_drone.dest_id = ID_PC;
	packets->rf_packet_drone.opcode = OPT_PAIR_START;
	packets->rf_packet_drone.plen = 0;
	//packets->rf_packet_drone.payload = NULL;
}



/*********************************************************************
 * @fcn    	packet_create_pair_start
 *
 * @param 	*packets: pointer to all data packets
 * @param 	*drone_data: pointer to all drone data
 *
 * @brief   Assemble packet for pairing ACK
 *
 * @return  none
 */
static void packet_create_pair_status(s_packets *packets, s_drone_data *drone_data)
{
	packets->rf_packet_drone.version = PROTOCOL_VER;
	packets->rf_packet_drone.flags = FLAG_ACK;
	packets->rf_packet_drone.src_id = ID_DRONE;
	packets->rf_packet_drone.dest_id = ID_PC;
	packets->rf_packet_drone.opcode = OPT_PAIR_STATUS;
	packets->rf_packet_drone.plen = 1;
	packets->rf_packet_drone.payload[0] = drone_data->DroneStatus;
}



/*********************************************************************
 * @fcn    	packet_create_ping_pong
 *
 * @param 	*packets: pointer to all data packets
 *
 * @brief   Assemble packet ping-pong test
 *
 * @return  none
 */
static void packet_create_ping_pong(s_packets *packets)
{
	packets->rf_packet_drone.version = PROTOCOL_VER;
	packets->rf_packet_drone.flags = FLAG_ACK;
	packets->rf_packet_drone.src_id = ID_DRONE;
	packets->rf_packet_drone.dest_id = ID_PC;
	packets->rf_packet_drone.opcode = OPT_PING;
	packets->rf_packet_drone.plen = 0;
}



/*********************************************************************
 * @fcn    	packet_create_telemetry
 *
 * @param 	*packets: pointer to all data packets
 * @param 	*drone_data: pointer to all drone data
 *
 * @brief   Assemble packet for telemetry sending
 *
 * @return  none
 */
void packet_create_telemetry(s_packets *packets, s_drone_data *drone_data)
{
	static uint8_t packet_num = 0;	// Counter of packets (each packet different data)
	uint8_t payload_cnt = 0;

	packets->rf_packet_drone.version = PROTOCOL_VER;
	packets->rf_packet_drone.flags = FLAG_STREAM;
	packets->rf_packet_drone.src_id = ID_DRONE;
	packets->rf_packet_drone.dest_id = ID_PC;
	packets->rf_packet_drone.opcode = OPT_TELEMETRY;

	// WARNING! Max payload size is 26-bytes only //
	switch(packet_num){

		case 0:
			// DRONE INFO

			/* Error codes in one byte */
			uint8_t error_code1 = (drone_data->error_code.err_main_bat    |
								 drone_data->error_code.err_edf_bat << 1 |
								 drone_data->error_code.err_radio1  << 2 |
								 drone_data->error_code.err_radio2  << 3 |
								 drone_data->error_code.err_bme280  << 4 |
								 drone_data->error_code.err_mpu6050 << 5 |
								 drone_data->error_code.err_vl53l1x << 6);
			uint8_t error_code2 = 0;

			/* Drone mode */
			packets->rf_packet_drone.payload[payload_cnt++] =  TVL_DRONE_MODE;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->DroneStatus;

			/* Error code */
			packets->rf_packet_drone.payload[payload_cnt++] =  TVL_ERR;
			packets->rf_packet_drone.payload[payload_cnt++] =  error_code1;
			packets->rf_packet_drone.payload[payload_cnt++] =  error_code2;

			/* Main battery voltage */
			packets->rf_packet_drone.payload[payload_cnt++] =  TVL_BAT_MAIN;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.battery_main_voltage >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->data.battery_main_voltage 	   & 0xFF;

			/* EDF battery voltage */
			packets->rf_packet_drone.payload[payload_cnt++] =  TVL_BAT_EDF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.battery_edf_voltage >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->data.battery_edf_voltage       & 0xFF;

			/* Date and time */
			packets->rf_packet_drone.payload[payload_cnt++] =  TVL_DATE_TIME;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->date_time.year;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->date_time.month;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->date_time.day;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->date_time.hour;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->date_time.minutes;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->date_time.seconds;

			/* Stream mode activated */
			packets->rf_packet_drone.payload[payload_cnt++] =  TVL_RF_STREAM;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->radio_data.flag_stream_data;

			packets->rf_packet_drone.plen = payload_cnt;

			packet_num++;

			break;

		case 1:
			// DRONE INFO & DATA_1

			/* TX packet counter */
			packets->rf_packet_drone.payload[payload_cnt++] = TVL_RF_TX_CNT;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->radio_data.packet_tx_cnt >> 24) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->radio_data.packet_tx_cnt >> 16) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->radio_data.packet_tx_cnt >>  8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->radio_data.packet_tx_cnt 	   & 0xFF;

			/* Fail TX packet counter */
			packets->rf_packet_drone.payload[payload_cnt++] = TVL_RF_FAIL_CNT;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->radio_data.packet_fail_cnt >> 24) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->radio_data.packet_fail_cnt >> 16) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->radio_data.packet_fail_cnt >>  8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->radio_data.packet_fail_cnt 		 & 0xFF;

			/* Temperature, humidity, pressure */
			packets->rf_packet_drone.payload[payload_cnt++] = TVL_THP;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.temperature >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->data.temperature 	  & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->data.humidity;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.pressure >> 24) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.pressure >> 16) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.pressure >>  8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->data.pressure 		& 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->position.height_baro_m >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->position.height_baro_m		& 0xFF;


			packets->rf_packet_drone.plen = payload_cnt;

			packet_num++;

			break;

		case 2:
			// DATA_2

			/* IMU temperature */
			packets->rf_packet_drone.payload[payload_cnt++] = TVL_IMU_TEMP;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->position.gyroTemp >> 8) & 0xFF;	// int16
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->position.gyroTemp       & 0xFF;  // int16


			/* IMU angles */
			packets->rf_packet_drone.payload[payload_cnt++] = TVL_ANGL;

			int16_t roll  = (int16_t)(drone_data->position.Roll  * ANGLE_SCALE);
			int16_t pitch = (int16_t)(drone_data->position.Pitch * ANGLE_SCALE);
			int16_t yaw   = (int16_t)(drone_data->position.Yaw   * ANGLE_SCALE);

			/* - Roll */
			packets->rf_packet_drone.payload[payload_cnt++] = (roll >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  roll       & 0xFF;

			/* - Pitch */
			packets->rf_packet_drone.payload[payload_cnt++] = (pitch >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  pitch       & 0xFF;

			/* - Yaw */
			packets->rf_packet_drone.payload[payload_cnt++] = (yaw >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  yaw       & 0xFF;


			/* IMU raw values */
			packets->rf_packet_drone.payload[payload_cnt++] = TVL_IMU;

			int16_t acc_x = (int16_t)(drone_data->position.accel_x * ANGLE_SCALE);
			int16_t acc_y = (int16_t)(drone_data->position.accel_y * ANGLE_SCALE);
			int16_t acc_z = (int16_t)(drone_data->position.accel_z * ANGLE_SCALE);

			/* - acceleration x */
			packets->rf_packet_drone.payload[payload_cnt++] = (acc_x >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  acc_x       & 0xFF;

			/* - acceleration y */
			packets->rf_packet_drone.payload[payload_cnt++] = (acc_y >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  acc_y       & 0xFF;

			/* - acceleration z */
			packets->rf_packet_drone.payload[payload_cnt++] = (acc_z >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  acc_z       & 0xFF;


			int16_t gyro_x = (int16_t)(drone_data->position.gyro_x * ANGLE_SCALE);
			int16_t gyro_y = (int16_t)(drone_data->position.gyro_y * ANGLE_SCALE);
			int16_t gyro_z = (int16_t)(drone_data->position.gyro_z * ANGLE_SCALE);

			/* - gyroscope x */
			packets->rf_packet_drone.payload[payload_cnt++] = (gyro_x >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  gyro_x       & 0xFF;

			/* - gyroscope y */
			packets->rf_packet_drone.payload[payload_cnt++] = (gyro_y >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  gyro_y       & 0xFF;

			/* - gyroscope z */
			packets->rf_packet_drone.payload[payload_cnt++] = (gyro_z >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  gyro_z       & 0xFF;

			packets->rf_packet_drone.plen = payload_cnt;

			packet_num = 0;

		case 100:
			// Last packet

			packet_num = 0;
			break;

		default:
			// Default
			break;
	}

}






