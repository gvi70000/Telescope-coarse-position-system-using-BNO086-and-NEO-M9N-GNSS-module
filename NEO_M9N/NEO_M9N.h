/**
 * @file NEO_M9N.h
 * @brief Header file for the u-blox NEO-M9N GNSS receiver library.
 *
 * Provides definitions, data structures, and function prototypes for receiving
 * and parsing UBX-NAV-PVT packets from the NEO-M9N over USART3 DMA.
 *
 * The module is pre-configured via u-blox u-center (settings saved to flash):
 *   - 115200 baud on UART1
 *   - UBX protocol only (NMEA disabled)
 *   - UBX-NAV-PVT output at 4 Hz
 * No runtime CFG commands are required.
 *
 * UBX-NAV-PVT full packet structure (100 bytes):
 *   [0..1]   0xB5 0x62       sync header
 *   [2]      0x01            NAV class
 *   [3]      0x07            PVT message ID
 *   [4..5]   0x5C 0x00       payload length = 92 bytes (little-endian)
 *   [6..97]  payload         UBX-NAV-PVT fields
 *   [98]     CK_A            Fletcher-8 checksum over bytes [2..97]
 *   [99]     CK_B
 *
 * Reference: u-blox M9 SPG 4.04 Interface Description UBX-21022436, §3.15.11
 * Target:    STM32F302, Keil MDK, STM32 HAL
 */

#ifndef __NEO_M9N_H
#define __NEO_M9N_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Frame size
#define RX_BUFFER_LEN		100U	// Exact UBX-NAV-PVT packet size (6 header + 92 payload + 2 CRC)

// Sync + class + ID as a little-endian uint32
// Bytes [0..3] = {0xB5, 0x62, 0x01, 0x07} = 0x070162B5
#define PVT_ID				0x070162B5UL

// Checksum bounds — Fletcher-8 over bytes [2..97], CK_A at [98], CK_B at [99]
#define UBX_CHKSUM_START	2U
#define UBX_CHKSUM_END		98U
#define UBX_CKA_POS			98U
#define UBX_CKB_POS			99U

// UBX-NAV-PVT payload byte offsets (frame-absolute)
// Frame layout: [0..1] sync | [2] class | [3] id | [4..5] len | [6..97] payload | [98..99] CK
// Payload-relative offset = frame index - 6
// All offsets verified against UBX-21022436 §3.15.11 and live captured packets
#define VAL_POS				17U		// X1  validity flags              [payload offset 11]
#define YER_POS				10U		// U2  UTC year                    [payload offset  4]
#define MON_POS				12U		// U1  UTC month  1..12            [payload offset  6]
#define DAY_POS				13U		// U1  UTC day    1..31            [payload offset  7]
#define HOR_POS				14U		// U1  UTC hour   0..23            [payload offset  8]
#define MIN_POS				15U		// U1  UTC minute 0..59            [payload offset  9]
#define SEC_POS				16U		// U1  UTC second 0..60            [payload offset 10]
#define NANO_POS			22U		// I4  nanosecond fraction         [payload offset 16] range [-1e9..+1e9 ns]
#define FIX_POS				26U		// U1  fix type (GPS_FIX_t)        [payload offset 20]
#define FLAGS_POS			27U		// X1  fix status flags            [payload offset 21]
#define SAT_POS				29U		// U1  number of SVs used          [payload offset 23]
#define LNG_POS				30U		// I4  longitude [1e-7 deg]        [payload offset 24]
#define LAT_POS				34U		// I4  latitude  [1e-7 deg]        [payload offset 28]
#define MSL_POS				42U		// I4  height above MSL [mm]       [payload offset 36]
#define FLAGS3_POS			84U		// X2  additional flags            [payload offset 78]

// Validity and flag bits
#define UBX_VALID_DATE			0x01U	// VAL_POS bit 0: UTC date valid
#define UBX_VALID_TIME			0x02U	// VAL_POS bit 1: UTC time of day valid
#define UBX_FLAGS_GNSS_FIX_OK	0x01U	// FLAGS_POS bit 0: valid fix within DOP & accuracy masks
#define UBX_FLAGS3_INVALID_LLH	0x0001U	// FLAGS3_POS bit 0: lon/lat/alt marked invalid by module

// Scale factors
#define COORD_FACTOR		1e-7f	// Longitude/latitude: 1e-7 deg/LSB
#define ALTITUDE_MM_TO_M	1e-3f	// Altitude: 1 mm/LSB → m
#define NANO_TO_SEC			1e-9f	// Nanosecond fraction → seconds; UTC_Second = sec + nano * NANO_TO_SEC

// UBX-NAV-PVT fixType values — UBX-21022436 §3.15.11 Table 3-63
typedef enum {
    GPS_FIX_NONE           = 0,  // No fix
    GPS_FIX_DEAD_RECKONING = 1,  // Dead reckoning only
    GPS_FIX_2D             = 2,  // 2D fix
    GPS_FIX_3D             = 3,  // 3D fix
    GPS_FIX_GNSS_DR        = 4,  // GNSS + dead reckoning combined
    GPS_FIX_TIME_ONLY      = 5,  // Time-only fix
} GPS_FIX_t;

/**
 * @brief Parsed UBX-NAV-PVT navigation data.
 * @details Packed to exactly 24 bytes — matches the GPS_POS slot in txBuffer.
 *
 *   float    Latitude    4
 *   float    Longitude   4
 *   float    Altitude    4
 *   uint8_t  UTC_Day     1
 *   uint8_t  UTC_Month   1
 *   uint16_t UTC_Year    2
 *   uint8_t  UTC_Hour    1
 *   uint8_t  UTC_Minute  1
 *   float    UTC_Second  4   whole seconds + GPS-disciplined nanosecond fraction
 *   uint8_t  Satellites  1
 *   uint8_t  Fix_Type    1   GPS_FIX_t
 *                        ──
 *                        24
 */
typedef struct __attribute__((packed)) {
    float    Latitude;    // [deg] WGS-84, positive = North
    float    Longitude;   // [deg] WGS-84, positive = East
    float    Altitude;    // [m]   above mean sea level
    uint8_t  UTC_Day;
    uint8_t  UTC_Month;
    uint16_t UTC_Year;
    uint8_t  UTC_Hour;
    uint8_t  UTC_Minute;
    float    UTC_Second;  // Whole seconds + GPS-disciplined nanosecond fraction
    uint8_t  Satellites;  // Number of SVs used in navigation solution
    uint8_t  Fix_Type;    // GPS_FIX_t — GPS_FIX_3D for a valid 3D fix
} GPS_DATA_t;

// DMA receive buffer — written by USART3 DMA, read by processGPS()
extern uint8_t gpsData[RX_BUFFER_LEN];

// Latest decoded GPS data — written by processGPS(), read by sendData()
extern GPS_DATA_t myGpsData;

void processGPS(void);
void M9N_Reset(void);
void M9N_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* __NEO_M9N_H */
