/**
 * @file NEO_M9N.c
 * @brief Implementation file for the u-blox NEO-M9N GNSS receiver library.
 *
 * Provides DMA-driven reception and parsing of UBX-NAV-PVT packets from the
 * NEO-M9N over USART3 at 115200 baud.
 *
 * The module is pre-configured via u-blox u-center (settings saved to flash):
 *   - 115200 baud, UBX-only output, UBX-NAV-PVT at 4 Hz
 * No runtime CFG commands are required.
 *
 * Call sequence (from main.c):
 *   1. M9N_Reset()  — hardware reset via GPS_RST pin
 *   2. M9N_Start()  — flush UART RDR, arm USART3 DMA into gpsData[]
 *   3. HAL_UART_RxCpltCallback fires every 100 bytes:
 *        re-arm DMA → set newGpsData flag
 *   4. Main loop: if (newGpsData) { newGpsData = 0; processGPS(); sendData(); }
 *
 * Target: STM32F302, Keil MDK, STM32 HAL
 */

#include "NEO_M9N.h"
#include "usart.h"
#include "main.h"

// DMA receive buffer — written exclusively by USART3 DMA (one 100-byte transfer per packet)
// 4-byte aligned so the 32-bit casts in processGPS() are always word-aligned on Cortex-M4
uint8_t gpsData[RX_BUFFER_LEN] __attribute__((aligned(4)));

// Latest decoded GPS navigation data
GPS_DATA_t myGpsData;

// Fletcher-8 checksum over bytes [2..97], compare CK_A/CK_B at [98]/[99]
/**
 * @brief checkSum function implementation.
 * @details Covers NAV class byte through last payload byte.
 *          Returns 1 if both checksum bytes match, 0 otherwise.
 * @param None
 * @return 1 on pass, 0 on fail.
 */
static uint8_t checkSum(void) {
    uint8_t ckA = 0;
    uint8_t ckB = 0;
    for (uint8_t i = UBX_CHKSUM_START; i < UBX_CHKSUM_END; i++) {
        ckA += gpsData[i];
        ckB += ckA;
    }
    return (ckA == gpsData[UBX_CKA_POS] && ckB == gpsData[UBX_CKB_POS]) ? 1U : 0U;
}

/**
 * @brief processGPS function implementation.
 * @details Validates sync header, checksum, and UTC validity flags, then
 *          decodes time, fix quality, and position from gpsData[] into myGpsData.
 *          Called from the main loop after newGpsData is set by the DMA callback.
 *          Position is skipped if FLAGS3 marks lon/lat/alt invalid.
 * @param None
 * @return None.
 */
void processGPS(void) {
    // Step 1: Validate sync header + class + id
    // gpsData[] is 4-byte aligned — word cast is safe on Cortex-M4
    if (*(uint32_t *)&gpsData[0] != PVT_ID) return;

    // Step 2: Validate Fletcher-8 checksum
    if (!checkSum()) return;

    // Step 3: Reject frames where module has not yet acquired valid date/time
    // During acquisition, validFlags = 0 and date/time fields contain garbage
    if (!(gpsData[VAL_POS] & UBX_VALID_DATE) || !(gpsData[VAL_POS] & UBX_VALID_TIME)) return;

    // Step 4: Decode UTC time
    myGpsData.UTC_Year   = *(uint16_t *)&gpsData[YER_POS];
    myGpsData.UTC_Month  = gpsData[MON_POS];
    myGpsData.UTC_Day    = gpsData[DAY_POS];
    myGpsData.UTC_Hour   = gpsData[HOR_POS];
    myGpsData.UTC_Minute = gpsData[MIN_POS];
    // UTC_Second = whole seconds + GPS-disciplined nanosecond fraction
    // float gives ~10 us resolution (23-bit mantissa, seconds 0..60 use ~6 bits)
    myGpsData.UTC_Second = (float)gpsData[SEC_POS]
                         + (float)(*(int32_t *)&gpsData[NANO_POS]) * NANO_TO_SEC;

    // Step 5: Decode fix quality
    myGpsData.Fix_Type   = gpsData[FIX_POS];
    myGpsData.Satellites = gpsData[SAT_POS];

    // Step 6: Decode position — skip if module explicitly marks lon/lat/alt invalid
    // lon/lat: 1e-7 degrees (I4, little-endian); hMSL: 1 mm (I4, little-endian)
    if (!(*(uint16_t *)&gpsData[FLAGS3_POS] & UBX_FLAGS3_INVALID_LLH)) {
        myGpsData.Longitude = (float)(*(int32_t *)&gpsData[LNG_POS]) * COORD_FACTOR;
        myGpsData.Latitude  = (float)(*(int32_t *)&gpsData[LAT_POS]) * COORD_FACTOR;
        myGpsData.Altitude  = (float)(*(int32_t *)&gpsData[MSL_POS]) * ALTITUDE_MM_TO_M;
    }
}

/**
 * @brief M9N_Reset function implementation.
 * @details Asserts RESET_N low for 100 ms then releases and waits 100 ms
 *          for the module to complete its startup sequence.
 * @param None
 * @return None.
 */
void M9N_Reset(void) {
    GPS_RST_Off;    // Assert RESET_N low
    HAL_Delay(100);
    GPS_RST_On;     // Release RESET_N
    HAL_Delay(100); // Wait for module startup
}

/**
 * @brief M9N_Start function implementation.
 * @details Flushes the UART RDR then arms USART3 DMA for one complete
 *          100-byte UBX-NAV-PVT frame. HT interrupt is disabled — only
 *          full packet transfers are processed.
 *          The RDR flush is issued BEFORE arming DMA; flushing after would
 *          discard the first byte of the first valid packet.
 * @param None
 * @return None.
 */
void M9N_Start(void) {
    // Flush RDR — drain any stale byte that arrived during reset/startup
    __HAL_UART_SEND_REQ(&huart3, UART_RXDATA_FLUSH_REQUEST);

    // Arm DMA for one complete 100-byte UBX-NAV-PVT frame
    HAL_UART_Receive_DMA(&huart3, gpsData, RX_BUFFER_LEN);

    // Disable half-transfer interrupt — only full packets are processed
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}
