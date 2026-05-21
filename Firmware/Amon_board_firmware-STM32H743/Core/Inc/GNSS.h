/*****************************************************************
 * File Name          : GPS.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/01/16
 * Description        : GPS packet parser
*****************************************************************/

#ifndef INC_GNSS_H_
#define INC_GNSS_H_

#include "stm32h7xx_hal.h"
#include "string.h"

/*###########################################################################################################################################################*/
/* Defines */
#define IS_DIGIT(x) ((x) >= '0' && (x) <= '9')


/*###########################################################################################################################################################*/
/* Structs */

// Used hardware
//	- NEO-6M

// GPS sub-packed - GGA format
typedef struct {
	uint8_t 	MessageID[5];		// $GPGGA
	uint8_t 	UTCTime[10];		// hhmmss.sss
	float 		Latitude;			// ddmm.mmmm
	char		NSIncicator;		// N = north / S = south
	float 		Longitude;			// dddmm.mmmm
	char 		EWIndicator;		// E = east / W = west
	uint8_t 	PositionFix;		// 0-6 values (3-5 not supported), values described bellow
	uint8_t 	NoSatelites;		// 0-12
	float 		HDOP;				// Horizontal Dilution of Precision
	float 		MSLAltitude;		// altitude
	char 		unitsAlt;			// M - meters
	float 		Geoid;				// Geoid-to-ellipsoid separation. Ellipsoid altitude = MSL Altitude + Geoid Separation
	char 		unitsGeoid;			// M - meters
	float		AgeOfDiffCorr;		// can be null(when DGPS not used) (unit: sec)
	uint16_t	DiffRefStationId;	// 0000
	uint8_t 	checksum;			// checksum
} s_GGA;

/*  # PositionFix - Position Fix Indicator table
 * |------------|-------------------------------------------|
 * |	Value	|	Description								|
 * |------------|-------------------------------------------|
 * |	0		|	Fix not available or invalid			|
 * |	1		|	GPS SPS Mode, fix valid					|
 * |	2		|	Differential GPS, SPS Mode, fix valid	|
 * |	3-5		|	Not supported							|
 * |	6		|	Dead Reckoning Mode, fix valid			|
 * |------------|-------------------------------------------|
 */


// GPS sub-packed - GLL format
typedef struct {
	uint8_t 	MessageID[6];		// $GPGGA
	float 		Latitude;			// ddmm.mmmm
	char 		NSIncicator;		// N = nort / S = south
	float 		Longitude;			// dddmm.mmmm
	char 		EWIndicator;		// E = east / W = west
	uint8_t 	UTCTime[10];		// hhmmss.sss
	char 		Status;				// A=data valid / V=data not valid
	char 		Mode;				// A = Autonomous, D = DGPS, E = DR (Only present in NMEA v3.00)
	uint8_t 	checksum;			// chacksum
} s_GLL;



// GPS sub-packed - GSA format
typedef struct {
	uint8_t 	MessageID[6]; 		// "$GPGSA"
	char 		Mode1;				// A - 2D Automatic / M - manual
	uint8_t 	Mode2;				// 1 - Fix not available / 2 - 2D (<4 SVs used) / 3 - 3D (>3 SVs used)
	uint8_t 	SatUsed[12];		// SV on Channel 1-12
	float 		PDOP;				// Position Dilution of Precision
	float 		HDOP;				// Horizontal Dilution of Precision
	float 		VDOP;				// Vertical Dilution of Precision
	uint8_t 	checksum;			// chacksum
} s_GSA;

/*  # Mode 1
 * |------------|-------------------------------------------------------|
 * |	Value	|	Description								  			|
 * |------------|-------------------------------------------------------|
 * |	M		|	Manual—forced to operate in 2D or 3D mode 		    |
 * |	A		|	2D Automatic—allowed to automatically switch 2D/3D  |
 * |------------|-------------------------------------------------------|
 *
 *  # Mode 2
 * |------------|-----------------------|
 * |	Value	|	Description			|
 * |------------|-----------------------|
 * |	1		|	Fix not available	|
 * |	2		|	2D (<4 SVs used)	|
 * |	3		|	3D (>3 SVs used)	|
 * |------------|-----------------------|
 */



// GPS sub-packed - GSV format
typedef struct {
    uint8_t 	id;
    uint8_t 	elevation;			// degress
    uint16_t 	azimuth;			// degress
    uint8_t 	snr;				// Range 0 to 99, null when not tracking
} s_satellite;


typedef struct {
	uint8_t 	MessageID[6]; 		// "$GPGSV"
	uint8_t 	NumOfMess;			// range 1 to 3
	uint8_t 	MessNum;			// range 1 to 3
	uint8_t 	NoSat;				// number of satelites in view
	s_satellite sat[4];				// Struct for each satelite
	uint8_t 	checksum;			// chacksum

} s_GSV;



// GPS sub-packed - RMC format - Important
typedef struct {
	uint8_t 	MessageID[6]; 		// "$GPRMC"
	uint8_t 	UTCTime[10];		// hhmmss.sss
	char 		Status;				// A=data valid / V=data not valid
	float 		Latitude;			// ddmm.mmmm
	char 		NSIncicator;		// N = nort / S = south
	float 		Longitude;			// dddmm.mmmm
	char 		EWIndicator;		// E = east / W = west
	float 		Speed;				// knots
	float 		Course;				// degress
	uint8_t 	Date[6];			// ddmmyy
	float	 	MagVar;				// E=east or W=west
	char 		MagEWIndicator;		// E=east
	char 		Mode;				// A = Autonomous, D = DGPS, E = DR
	uint8_t 	checksum;			// chacksum
} s_RMC;



// GPS sub-packed - VTG format
typedef struct {
	uint8_t 	MessageID[6]; 		// "$GPVTG"
	float 		Course1;			// degress
	char	 	Ref1;				// True
	float 		Course2;			// degress
	char	 	Ref2;				// Magnetic
	float 		SpeedKn;			// knots
	char 		UnitsKn;			// N = knots
	float 		SpeedKmH;			// knots
	char 		UnitsKmH;			// K = km / h
	char 		Mode;				// A = Autonomous, D = DGPS, E = DR
	uint8_t 	checksum;			// chacksum
} s_VTG;



// Main GPS packed
typedef struct {
	uint8_t  	GPS_RX_buffer[426];	// GPS data buffer
	s_GGA		gga;				// Packet format data - GGA (Position + altitude)
	s_GLL		gll;				// Packet format data - GLL (Lat/Lon)
	s_GSA		gsa;				// Packet format data - GSA (Fix type + DOP)
	s_GSV		gsv;				// Packet format data - GSV (SNR, elevation)
	s_RMC		rmc;				// Packet format data - RMC (Position + speed + time)
	s_VTG		vtg;				// Packet format data - VTG (Speed over ground)
} s_GNSS;



/*###########################################################################################################################################################*/
/* Functions */

void GPS_Decode_GGA(uint8_t *GPSData, s_GGA *gps_gga);
void GPS_Decode_GLL(uint8_t *GPSData, s_GLL *gps_gll);
void GPS_Decode_GSA(uint8_t *GPSData, s_GSA *gps_gsa);
void GPS_Decode_GSV(uint8_t *GPSData, s_GSV *gps_gsv);
void GPS_Decode_RMC(uint8_t *GPSData, s_RMC *gps_rmc);
void GPS_Decode_VTG(uint8_t *GPSData, s_VTG *gps_vtg);

#endif /* INC_GNSS_H_ */
