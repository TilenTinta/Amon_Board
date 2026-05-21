/*****************************************************************
 * File Name          : GPS.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/01/16
 * Description        : GPS packet parser
*****************************************************************/

#include <GNSS.h>



/*###########################################################################################################################################################*/
/* Functions */

/*********************************************************************
 * @fcn     gps_split_packet
 *
 * @param	type: pointer to frame type string
 * @param	message: full input message
 * @param	frame: output of complete searched frame
 *
 * @brief   Isolate searched frame type
 *
 * @return  none
 */
static void gps_split_packet(uint8_t *type, uint8_t *message, uint8_t *frame)
{
    uint16_t i = 0;
    uint16_t j = 0;

    while(i < 426 && message[i] != '\0') // Buffer lenght
    {
    	// Detect start of the packet
        if(message[i] == '$')
        {
            if((message[i + 3] == type[0]) &&
               (message[i + 4] == type[1]) &&
               (message[i + 5] == type[2]))
            {
                j = 0;

                // Detect end of the packet (keep "\r -> used later to detect end of frame")
                while(message[i] != '\n' && j < 100)
                {
                    frame[j++] = message[i++];
                }

                frame[j] = 0;
                return;
            }
        }

        i++;
    }
}



/*********************************************************************
 * @fcn     gps_parse_float
 *
 * @param	data: pointer to frame type string
 * @param	len: length of input buffer
 *
 * @brief   Convert "string" from buffer to float
 *
 * @return  float value
 */
static float gps_parse_float(uint8_t *data, uint8_t len)
{
    float value = 0.0f;
    float fraction = 0.0f;
    float divisor = 1.0f;
    uint8_t decimal = 0;

    for(uint8_t i = 0; i < len; i++)
    {
    	// Detect decimal point
        if(data[i] == '.')
        {
            decimal = 1;
            continue;
        }

        // if data is empty return 0
        if(data[i] < '0' || data[i] > '9')
            break;

        uint8_t digit = data[i] - '0';

        if(!decimal)
        {
            value = value * 10.0f + digit;
        }
        else
        {
            divisor *= 10.0f;
            fraction += digit / divisor;
        }
    }

    return value + fraction; // 4 decimal points cca. 11m
}



/*********************************************************************
 * @fcn     GPS_Decode_GGA
 *
 * @param	GPSData: pointer to frame type string
 * @param	gps_gga: length of input buffer
 *
 * @brief   Parse raw GPS packet to GGA fields
 *
 * @return  none
 */
void GPS_Decode_GGA(uint8_t *GPSData, s_GGA *gps_gga)
{
	uint8_t frameId[3] = "GGA";
	uint8_t frame[100] ={0};

	gps_split_packet(frameId, GPSData, frame);

	uint8_t i = 0;
	uint8_t field = 0; // Field of frame

	while(frame[i] != '\r' && frame[i] != 0)
	{
		if (frame[i] == ',' || frame[i] == '$')
		{
			i++; // Comma is not saved so move on the next byte
			uint8_t tempFrame[10] = {0};

			switch(field)
			{
				// MessageID
				case 0:
					memset(gps_gga->MessageID, 0, sizeof(gps_gga->MessageID));

					for (uint8_t j = 0; j < sizeof(gps_gga->MessageID); j++)
					{
						if (frame[i + j] == ',') break;
						gps_gga->MessageID[j] = frame[i + j];
					}
					break;


				// UTCTime
				case 1:
					for (uint8_t j = 0; j < sizeof(gps_gga->UTCTime); j++)
					{
						if (frame[i + j] == ',') break;
						gps_gga->UTCTime[j] = frame[i + j];
					}
					break;


				// Latitude
				case 2:
					{
						uint8_t j;
						for (j = 0; j < sizeof(tempFrame); j++)
						{
							if (frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}
						gps_gga->Latitude = gps_parse_float(tempFrame, j);
					}
					break;


				// NSIncicator
				case 3:
					if(frame[i] != ',')
					{
						gps_gga->NSIncicator = frame[i];
					}
					else
					{
						gps_gga->NSIncicator = 0;
					}
					break;


				// Longitude
				case 4:
					{
						uint8_t j;
						for (j = 0; j < sizeof(tempFrame); j++)
						{
							if (frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}
						gps_gga->Longitude = gps_parse_float(tempFrame, j);
					}
					break;


				// EWIndicator
				case 5:
				    if(frame[i] != ',')
				    {
				        gps_gga->EWIndicator = frame[i];
				    }
				    else
				    {
				        gps_gga->EWIndicator = 0;
				    }
					break;


				// PositionFix
				case 6:
				    if(IS_DIGIT(frame[i]))
				    {
				        gps_gga->PositionFix = frame[i] - '0';
				    }
				    else
				    {
				        gps_gga->PositionFix = 0;
				    }
					break;


				// NoSatelites
				case 7:
				    if(IS_DIGIT(frame[i]))
				    {
				        if(IS_DIGIT(frame[i + 1]))
				            gps_gga->NoSatelites = (frame[i] - '0') * 10 + (frame[i + 1] - '0');
				        else
				            gps_gga->NoSatelites = frame[i] - '0';
				    }
				    else
				    {
				        gps_gga->NoSatelites = 0;
					}
					break;


				// HDOP
				case 8:
					{
						uint8_t j;
						for (j = 0; j < sizeof(tempFrame); j++)
						{
							if (frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}
						gps_gga->HDOP = gps_parse_float(tempFrame, j);
					}
					break;


				// MSLAltitude
				case 9:
					{
						uint8_t j;
						for (j = 0; j < sizeof(tempFrame); j++)
						{
							if (frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}
						gps_gga->MSLAltitude = gps_parse_float(tempFrame, j);
					}
					break;


				// unitsAlt
				case 10:
					if(frame[i] != ',')
					{
						gps_gga->unitsAlt = frame[i];
					}
					else
					{
						gps_gga->unitsAlt = 0;
					}
					break;


				// Geoid
				case 11:
					{
						uint8_t j;

						for (j = 0; j < sizeof(tempFrame); j++)
						{
							if (frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_gga->Geoid = gps_parse_float(tempFrame, j);
					}
					break;


				// unitsGeoid
				case 12:
					if(frame[i] != ',')
					{
						gps_gga->unitsGeoid = frame[i];
					}
					else
					{
						gps_gga->unitsGeoid = 0;
					}
					break;


				// AgeOfDiffCorr
				case 13:
					{
						if(frame[i] == ',')
						{
							gps_gga->AgeOfDiffCorr = 0;
							break;
						}

						uint8_t j;

						for (j = 0; j < sizeof(tempFrame); j++)
						{
							if (frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_gga->AgeOfDiffCorr = gps_parse_float(tempFrame, j);
					}
					break;


				// DiffRefStationId
				case 14:
					{
						uint16_t value = 0;

						for(uint8_t j = 0; j < 4; j++)
						{
							if(frame[i + j] < '0' || frame[i + j] > '9')
								break;

							value = value * 10 + (frame[i + j] - '0');
						}

						gps_gga->DiffRefStationId = value;
					}
					break;


				// checksum
				case 15:
					{
						uint8_t high = frame[i + 1];
						uint8_t low  = frame[i + 2];

						uint8_t value = 0;

						if(high >= 'A')
						{
							value = (high - 'A' + 10) << 4;
						}
						else
						{
							value = (high - '0') << 4;
						}

						if(low >= 'A')
						{
							value |= (low - 'A' + 10);
						}
						else
						{
							value |= (low - '0');
						}

						gps_gga->checksum = value;
					}
					break;


				// Default
				default:
					break;
			}

			field++; // When comma or dollar sign is detected move to next field
		}

		i++; // Next character of field
	}
}

void GPS_Decode_GLL(uint8_t *GPSData, s_GLL *gps_gll)
{
	uint8_t frameId[3] = "GLL";
	uint8_t frame[100] = {0};

	gps_split_packet(frameId, GPSData, frame);

	uint8_t i = 0;
	uint8_t field = 0; // Field of frame

	while(frame[i] != '\r' && frame[i] != 0)
	{
		if(frame[i] == ',' || frame[i] == '$')
		{
			i++;
			uint8_t tempFrame[10] = {0};

			switch(field)
			{
				// MessageID
				case 0:
					memset(gps_gll->MessageID, 0, sizeof(gps_gll->MessageID));

					for(uint8_t j = 0; j < sizeof(gps_gll->MessageID); j++)
					{
						if(frame[i + j] == ',') break;
						gps_gll->MessageID[j] = frame[i + j];
					}
					break;


				// Latitude
				case 1:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_gll->Latitude = gps_parse_float(tempFrame, j);
					}
					break;


				// NSIndicator
				case 2:
					if(frame[i] != ',')
					{
						gps_gll->NSIncicator = frame[i];
					}
					else
					{
						gps_gll->NSIncicator = 0;
					}
					break;


				// Longitude
				case 3:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_gll->Longitude = gps_parse_float(tempFrame, j);
					}
					break;


				// EWIndicator
				case 4:
					if(frame[i] != ',')
					{
						gps_gll->EWIndicator = frame[i];
					}
					else
					{
						gps_gll->EWIndicator = 0;
					}
					break;


				// UTCTime
				case 5:
					memset(gps_gll->UTCTime, 0, sizeof(gps_gll->UTCTime));

					for(uint8_t j = 0; j < sizeof(gps_gll->UTCTime); j++)
					{
						if(frame[i + j] == ',') break;
						gps_gll->UTCTime[j] = frame[i + j];
					}
					break;


				// Status
				case 6:
					if(frame[i] != ',')
					{
						gps_gll->Status = frame[i];
					}
					else
					{
						gps_gll->Status = 0;
					}
					break;


				// Mode
				case 7:
					if(frame[i] != ',')
					{
						gps_gll->Mode = frame[i];
					}
					else
					{
						gps_gll->Mode = 0;
					}
					break;


				// checksum
				case 8:
					{
						uint8_t high = frame[i + 1];
						uint8_t low  = frame[i + 2];

						uint8_t value = 0;

						if(high >= 'A')
						{
							value = (high - 'A' + 10) << 4;
						}
						else
						{
							value = (high - '0') << 4;
						}

						if(low >= 'A')
						{
							value |= (low - 'A' + 10);
						}
						else
						{
							value |= (low - '0');
						}

						gps_gll->checksum = value;
					}
					break;

				// Default
				default:
					break;
			}

			field++; // When comma or dollar sign is detected move to next field
		}

		i++; // Next character of field
	}
}

void GPS_Decode_GSA(uint8_t *GPSData, s_GSA *gps_gsa)
{
	uint8_t frameId[3] = "GSA";
	uint8_t frame[100] = {0};

	gps_split_packet(frameId, GPSData, frame);

	uint8_t i = 0;
	uint8_t field = 0; // Field of frame

	while(frame[i] != '\r' && frame[i] != 0)
	{
		if(frame[i] == ',' || frame[i] == '$')
		{
			i++;
			uint8_t tempFrame[10] = {0};

			switch(field)
			{
				// MessageID
				case 0:
					memset(gps_gsa->MessageID, 0, sizeof(gps_gsa->MessageID));

					for(uint8_t j = 0; j < sizeof(gps_gsa->MessageID); j++)
					{
						if(frame[i + j] == ',') break;
						gps_gsa->MessageID[j] = frame[i + j];
					}
					break;


				// Mode1
				case 1:
					if(frame[i] != ',')
					{
						gps_gsa->Mode1 = frame[i];
					}
					else
					{
						gps_gsa->Mode1 = 0;
					}
					break;


				// Mode2
				case 2:
					if(IS_DIGIT(frame[i]))
					{
						gps_gsa->Mode2 = frame[i] - '0';
					}
					else
					{
						gps_gsa->Mode2 = 0;
					}
					break;


				// Satellites used (fields 3-14, multi case)
				case 3: case 4: case 5: case 6:
				case 7: case 8: case 9: case 10:
				case 11: case 12: case 13: case 14:
					{
						uint8_t index = field - 3;

						if(IS_DIGIT(frame[i]))
						{
							if(IS_DIGIT(frame[i + 1]))
							{
								gps_gsa->SatUsed[index] = (frame[i] - '0') * 10 + (frame[i + 1] - '0');
							}
							else
							{
								gps_gsa->SatUsed[index] = frame[i] - '0';
							}
						}
						else
						{
							gps_gsa->SatUsed[index] = 0;
						}
					}
					break;


				// PDOP
				case 15:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_gsa->PDOP = gps_parse_float(tempFrame, j);
					}
					break;


				// HDOP
				case 16:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_gsa->HDOP = gps_parse_float(tempFrame, j);
					}
					break;


				// VDOP
				case 17:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == '*') break;
							tempFrame[j] = frame[i + j];
						}

						gps_gsa->VDOP = gps_parse_float(tempFrame, j);
					}
					break;


				// checksum
				case 18:
					{
						uint8_t high = frame[i + 1];
						uint8_t low  = frame[i + 2];

						uint8_t value = 0;

						if(high >= 'A')
						{
							value = (high - 'A' + 10) << 4;
						}
						else
						{
							value = (high - '0') << 4;
						}

						if(low >= 'A')
						{
							value |= (low - 'A' + 10);
						}
						else
						{
							value |= (low - '0');
						}

						gps_gsa->checksum = value;
					}
					break;


				// Default
				default:
					break;
			}

			field++; // When comma or dollar sign is detected move to next field
		}

		i++; // Next character of field
	}
}

void GPS_Decode_GSV(uint8_t *GPSData, s_GSV *gps_gsv)
{
	uint8_t frameId[3] = "GSV";
	uint8_t frame[100] = {0};

	gps_split_packet(frameId, GPSData, frame);

	uint8_t i = 0;
	uint8_t field = 0; // Field of frame

	while(frame[i] != '\r' && frame[i] != 0)
	{
		if(frame[i] == ',' || frame[i] == '$')
		{
			i++;
			uint8_t tempFrame[10] = {0};

			switch(field)
			{
				// MessageID
				case 0:
					memset(gps_gsv->MessageID, 0, sizeof(gps_gsv->MessageID));

					for(uint8_t j = 0; j < sizeof(gps_gsv->MessageID); j++)
					{
						if(frame[i + j] == ',') break;
						gps_gsv->MessageID[j] = frame[i + j];
					}
					break;


				// NumOfMess
				case 1:
					if(IS_DIGIT(frame[i]))
					{
						gps_gsv->NumOfMess = frame[i] - '0';
					}
					else
					{
						gps_gsv->NumOfMess = 0;
					}
					break;


				// MessNum
				case 2:
					if(IS_DIGIT(frame[i]))
					{
						gps_gsv->MessNum = frame[i] - '0';
					}
					else
					{
						gps_gsv->MessNum = 0;
					}
					break;


				// NoSat
				case 3:
					{
						if(IS_DIGIT(frame[i]))
						{
							if(IS_DIGIT(frame[i+1]))
							{
								gps_gsv->NoSat = (frame[i] - '0') * 10 + (frame[i + 1] - '0');
							}
							else
							{
								gps_gsv->NoSat = frame[i] - '0';
							}
						}
						else
						{
							gps_gsv->NoSat = 0;
						}
					}
					break;


				// Satellites
				case 4: case 8: case 12: case 16:
					{
						uint8_t satIndex = (field - 4) / 4;

						if(IS_DIGIT(frame[i]))
						{
							if(IS_DIGIT(frame[i + 1]))
							{
								gps_gsv->sat[satIndex].id = (frame[i] - '0') * 10 + (frame[i + 1] - '0');
							}
							else
							{
								gps_gsv->sat[satIndex].id = frame[i] - '0';
							}
						}
						else
						{
							gps_gsv->sat[satIndex].id = 0;
						}
					}
					break;


				// Elevation
				case 5: case 9: case 13: case 17:
					{
						uint8_t satIndex = (field - 5) / 4;

						uint8_t value = 0;

						if(IS_DIGIT(frame[i]))
						{
							value = frame[i] - '0';
							if(IS_DIGIT(frame[i + 1])) value = value*10 + (frame[i+1]-'0');
						}

						gps_gsv->sat[satIndex].elevation = value;
					}
					break;


				// Azimuth
				case 6: case 10: case 14: case 18:
					{
						uint8_t satIndex = (field - 6) / 4;

						uint16_t value = 0;

						for(uint8_t j = 0; j < 3; j++)
						{
							if(!IS_DIGIT(frame[i + j])) break;
							value = value * 10 + (frame[i + j] - '0');
						}

						gps_gsv->sat[satIndex].azimuth = value;
					}
					break;


				// SNR
				case 7: case 11: case 15: case 19:
				{
					uint8_t satIndex = (field - 7) / 4;

					uint8_t value = 0;

					if(IS_DIGIT(frame[i]))
					{
						value = frame[i] - '0';
						if(IS_DIGIT(frame[i + 1])) value = value * 10 + (frame[i + 1] - '0');
					}

					gps_gsv->sat[satIndex].snr = value;
				}
				break;


				// checksum
				case 20:
				{
					uint8_t high = frame[i + 1];
					uint8_t low  = frame[i + 2];

					uint8_t value = 0;

					if(high >= 'A')
					{
						value = (high - 'A' + 10) << 4;
					}
					else
					{
						value = (high - '0') << 4;
					}

					if(low >= 'A')
					{
						value |= (low - 'A' + 10);
					}
					else
					{
						value |= (low - '0');
					}

					gps_gsv->checksum = value;
				}
				break;

				// Default
				default:
					break;
			}

			field++; // When comma or dollar sign is detected move to next field
		}

		i++; // Next character of field
	}
}

void GPS_Decode_RMC(uint8_t *GPSData, s_RMC *gps_rmc)
{
	uint8_t frameId[3] = "RMC";
	uint8_t frame[100] = {0};

	gps_split_packet(frameId, GPSData, frame);

	uint8_t i = 0;
	uint8_t field = 0; // Field of frame

	while(frame[i] != '\r' && frame[i] != 0)
	{
		if(frame[i] == ',' || frame[i] == '$')
		{
			i++;
			uint8_t tempFrame[10] = {0};

			switch(field)
			{
				// MessageID
				case 0:
					memset(gps_rmc->MessageID, 0, sizeof(gps_rmc->MessageID));

					for(uint8_t j = 0; j < sizeof(gps_rmc->MessageID); j++)
					{
						if(frame[i + j] == ',') break;
						gps_rmc->MessageID[j] = frame[i + j];
					}
					break;


				// UTCTime
				case 1:
					memset(gps_rmc->UTCTime, 0, sizeof(gps_rmc->UTCTime));

					for(uint8_t j = 0; j < sizeof(gps_rmc->UTCTime); j++)
					{
						if(frame[i + j] == ',') break;
						gps_rmc->UTCTime[j] = frame[i + j];
					}
					break;


				// Status
				case 2:
					if(frame[i] != ',')
					{
						gps_rmc->Status = frame[i];
					}
					else
					{
						gps_rmc->Status = 0;
					}
					break;


				// Latitude
				case 3:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_rmc->Latitude = gps_parse_float(tempFrame, j);
					}
					break;


				// NSIndicator
				case 4:
					if(frame[i] != ',')
					{
						gps_rmc->NSIncicator = frame[i];
					}
					else
					{
						gps_rmc->NSIncicator = 0;
					}
					break;


				// Longitude
				case 5:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_rmc->Longitude = gps_parse_float(tempFrame, j);
					}
					break;


				// EWIndicator
				case 6:
					if(frame[i] != ',')
					{
						gps_rmc->EWIndicator = frame[i];
					}
					else
					{
						gps_rmc->EWIndicator = 0;
					}
					break;


				// Speed
				case 7:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_rmc->Speed = gps_parse_float(tempFrame, j);
					}
					break;


				// Course
				case 8:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_rmc->Course = gps_parse_float(tempFrame, j);
					}
					break;


				// Date
				case 9:
					memset(gps_rmc->Date, 0, sizeof(gps_rmc->Date));

					for(uint8_t j = 0; j < sizeof(gps_rmc->Date); j++)
					{
						if(frame[i + j] == ',') break;
						gps_rmc->Date[j] = frame[i + j];
					}
					break;


				// Magnetic variation
				case 10:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_rmc->MagVar = gps_parse_float(tempFrame, j);
					}
					break;


				// Magnetic variation indicator
				case 11:
					if(frame[i] != ',')
					{
						gps_rmc->MagEWIndicator = frame[i];
					}
					else
					{
						gps_rmc->MagEWIndicator = 0;
					}
					break;


				// Mode
				case 12:
					if(frame[i] != ',')
					{
						gps_rmc->Mode = frame[i];
					}
					else
					{
						gps_rmc->Mode = 0;
					}
					break;


				// checksum
				case 13:
					{
						uint8_t high = frame[i+1];
						uint8_t low  = frame[i+2];

						uint8_t value = 0;

						if(high >= 'A')
						{
							value = (high - 'A' + 10) << 4;
						}
						else
						{
							value = (high - '0') << 4;
						}

						if(low >= 'A')
						{
							value |= (low - 'A' + 10);
						}
						else
						{
							value |= (low - '0');
						}

						gps_rmc->checksum = value;
					}
					break;


				// Default
				default:
					break;
			}

			field++; // When comma or dollar sign is detected move to next field
		}

		i++; // Next character of field
	}
}

void GPS_Decode_VTG(uint8_t *GPSData, s_VTG *gps_vtg)
{
	uint8_t frameId[3] = "VTG";
	uint8_t frame[100] = {0};

	gps_split_packet(frameId, GPSData, frame);

	uint8_t i = 0;
	uint8_t field = 0; // Field of frame

	while(frame[i] != '\r' && frame[i] != 0)
	{
		if(frame[i] == ',' || frame[i] == '$')
		{
			i++;
			uint8_t tempFrame[10] = {0};

			switch(field)
			{
				// MessageID
				case 0:
					memset(gps_vtg->MessageID, 0, sizeof(gps_vtg->MessageID));

					for(uint8_t j = 0; j < sizeof(gps_vtg->MessageID); j++)
					{
						if(frame[i + j] == ',') break;
						gps_vtg->MessageID[j] = frame[i + j];
					}
					break;


				// Course1 (true course)
				case 1:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_vtg->Course1 = gps_parse_float(tempFrame, j);
					}
					break;


				// Ref1
				case 2:
					if(frame[i] != ',')
					{
						gps_vtg->Ref1 = frame[i];
					}
					else
					{
						gps_vtg->Ref1 = 0;
					}
					break;


				// Course2 (magnetic course)
				case 3:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_vtg->Course2 = gps_parse_float(tempFrame, j);
					}
					break;


				// Ref2
				case 4:
					if(frame[i] != ',')
					{
						gps_vtg->Ref2 = frame[i];
					}
					else
					{
						gps_vtg->Ref2 = 0;
					}
					break;


				// Speed in knots
				case 5:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_vtg->SpeedKn = gps_parse_float(tempFrame, j);
					}
					break;


				// UnitsKn
				case 6:
					if(frame[i] != ',')
					{
						gps_vtg->UnitsKn = frame[i];
					}
					else
					{
						gps_vtg->UnitsKn = 0;
					}
					break;


				// Speed km/h
				case 7:
					{
						uint8_t j;

						for(j = 0; j < sizeof(tempFrame); j++)
						{
							if(frame[i + j] == ',') break;
							tempFrame[j] = frame[i + j];
						}

						gps_vtg->SpeedKmH = gps_parse_float(tempFrame, j);
					}
					break;


				// UnitsKmH
				case 8:
					if(frame[i] != ',')
					{
						gps_vtg->UnitsKmH = frame[i];
					}
					else
					{
						gps_vtg->UnitsKmH = 0;
					}
					break;


				// Mode
				case 9:
					if(frame[i] != ',')
					{
						gps_vtg->Mode = frame[i];
					}
					else
					{
						gps_vtg->Mode = 0;
					}
					break;


				// checksum
				case 10:
					{
						uint8_t high = frame[i+1];
						uint8_t low  = frame[i+2];

						uint8_t value = 0;

						if(high >= 'A')
						{
							value = (high - 'A' + 10) << 4;
						}
						else
						{
							value = (high - '0') << 4;
						}

						if(low >= 'A')
						{
							value |= (low - 'A' + 10);
						}
						else
						{
							value |= (low - '0');
						}

						gps_vtg->checksum = value;
					}
					break;


				// Default
				default:
					break;
			}

			field++; // When comma or dollar sign is detected move to next field
		}

		i++; // Next character of field
	}
}



