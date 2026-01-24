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
			packets->rf_packet_drone.payload[payload_cnt++] = TVL_THP;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.temperature >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->data.temperature 	  & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->data.humidity;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.pressure >> 24) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.pressure >> 16) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.pressure >>  8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->data.pressure 		& 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.battery_main_voltage >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->data.battery_main_voltage 	   & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] = (drone_data->data.battery_edf_voltage >> 8) & 0xFF;
			packets->rf_packet_drone.payload[payload_cnt++] =  drone_data->data.battery_edf_voltage       & 0xFF;

			packets->rf_packet_drone.plen = payload_cnt;
			//packet_num++; // test

			break;

		case 1:


			break;

		case 100:
			// Last packet

			packet_num = 0;
			break;

		default:
			// Default
			break;
	}

}






