#ifndef __NEO_M9N_H
#define __NEO_M9N_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define RX_BUFFER_LEN 128

//UBX-NAV-POSLLH	(0x01 0x02) 0xb5 0x62 0x01 0x02 = 33645237
//UBX-NAV-STATUS	(0x01 0x03) 0xb5 0x62 0x01 0x03 = 50422453
//UBX-NAV-TIMEUTC (0x01 0x21) 0xb5 0x62 0x01 0x21 = 553738933
//UBX-NAV-PVT			(0x01 0x07) 0xb5 0x62 0x01 0x07 = 117531317
#define PVT_ID 117531317

#define CHECK_SUM_START_POS 2
#define CHECK_SUM_END_POS 101

#define YER_POS 10
#define MON_POS 12
#define DAY_POS 13
#define HOR_POS 14
#define MIN_POS 15
#define SEC_POS 16
#define FIX_POS 26
#define SAT_POS 29
#define LNG_POS 30
#define LAT_POS 34
#define MSL_POS 42

#define COORD_FACTOR 0.0000001f
#define ALTITUDE_FACTOR 0.001f

typedef enum {
	NoFix = 0,
	DeadReckoningOnly,
	Fix2D,
	Fix3D,
	Combined,
	Time,
	Reserved
} GPS_FIX_t;

typedef	struct GPS_DATA {
	uint8_t newData;
	float Latitude;
	float Longitude;
	float Altitude;
	uint8_t UTC_Day;
	uint8_t UTC_Month;
	uint16_t UTC_Year;
	uint8_t UTC_Hour;
	uint8_t UTC_Minute;
	float UTC_Second;
	uint8_t Satellites;
	uint8_t Fix_Type;
} GPS_DATA_t;

extern volatile uint8_t gpsRxBuffer[RX_BUFFER_LEN];
extern GPS_DATA_t myGpsData;

void processGPS(void);
void M9N_Reset();
void M9N_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* __NEO_M9N_H */
