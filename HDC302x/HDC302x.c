/**
 * @file HDC302x.c
 * @brief Implementation file for HDC302x temperature and humidity sensor library.
 *
 * Contains functions for initializing, reading data, and configuring the HDC302x sensor.
 * Target: STM32F302, Keil MDK, STM32 HAL.
 */

#include "HDC302x.h"
#include "i2c.h"
#include <math.h>

// Helper: calculate CRC-8 (polynomial 0x31, init 0xFF) over 2 bytes
/**
 * @brief CalculateCRC function implementation.
 * @param data Pointer to 2-byte buffer.
 * @return Computed CRC byte.
 */
static uint8_t CalculateCRC(const uint8_t* data) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < 2; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

// Helper: verify CRC of a [MSB, LSB, CRC] triplet received from the sensor
/**
 * @brief VerifyCRC function implementation.
 * @param data Pointer to 3-byte buffer [MSB, LSB, CRC].
 * @return HAL_OK if CRC matches, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef VerifyCRC(const uint8_t* data) {
    return (CalculateCRC(data) == data[2]) ? HAL_OK : HAL_ERROR;
}

// Helper: send a 2-byte command with optional 3-byte config payload to the sensor
/**
 * @brief SendDeviceCommand function implementation.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
static HAL_StatusTypeDef SendDeviceCommand(HDC302x_t* sensorObj) {
    uint8_t buffer[5];  // 2 bytes command + up to 3 bytes config
    uint8_t len = 0;

    // Command bytes: MSB first (big-endian as required by HDC302x)
    buffer[0] = (uint8_t)(sensorObj->Cmd.Command >> 8);
    buffer[1] = (uint8_t)(sensorObj->Cmd.Command & 0xFF);
    len = 2;

    // Append config payload only when CRC is non-zero (indicates a valid config is set)
    if (sensorObj->Config.CFG_CRC != 0x00) {
        buffer[2] = sensorObj->Config.CFG_MSB;
        buffer[3] = sensorObj->Config.CFG_LSB;
        buffer[4] = sensorObj->Config.CFG_CRC;
        len = 5;
    }

    return HAL_I2C_Master_Transmit(sensorObj->hi2c, sensorObj->Address, buffer, len, I2C_TIMEOUT);
}

// Initialization
/**
 * @brief HDC302x_Init function implementation.
 * @param sensorObj Pointer to sensor handle. Caller must set Address and hi2c.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_Init(HDC302x_t* sensorObj) {
    HAL_StatusTypeDef status;

    // Step 1: Send soft reset
    sensorObj->Config.CFG_CRC = 0x00;  // No payload with the reset command
    sensorObj->Cmd = HDC302X_CMD_SOFT_RESET;
    status = SendDeviceCommand(sensorObj);
    if (status != HAL_OK) {
        return status;  // I2C error
    }
    HAL_Delay(2);  // 2ms delay after reset (datasheet minimum is 1ms)

    // Step 2: Read status register to confirm reset completed
    status = HDC302x_ReadStatus(sensorObj);
    if (status != HAL_OK) {
        return status;
    }

    // Step 3: Clear status register (clears device_reset flag and any stale alerts)
    status = HDC302x_ClearStatus(sensorObj);
    return status;
}

// Read temperature and humidity data
/**
 * @brief HDC302x_ReadData function implementation.
 * @details Sends the measurement readout command and reads 6 bytes:
 *          [T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC].
 *          Both CRCs are verified before converting raw values.
 *          Conversion formulas (datasheet):
 *            Temperature (C) = raw_T  * (165.0 / 65535.0) - 40.0
 *            Humidity    (%) = raw_RH * (100.0 / 65535.0)
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ReadData(HDC302x_t* sensorObj) {
    HAL_StatusTypeDef status;
    uint8_t rx[6];  // [T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC]

    // Send measurement read command
    sensorObj->Config.CFG_CRC = 0x00;
    sensorObj->Cmd = HDC302X_CMD_MEASURE_READ;
    status = SendDeviceCommand(sensorObj);
    if (status != HAL_OK) {
        return status;
    }

    // Receive 6 bytes
    status = HAL_I2C_Master_Receive(sensorObj->hi2c, sensorObj->Address, rx, sizeof(rx), I2C_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }

    // Verify CRC for temperature word and humidity word
    if (VerifyCRC(&rx[0]) != HAL_OK) {
        return HAL_ERROR;  // Temperature CRC mismatch
    }
    if (VerifyCRC(&rx[3]) != HAL_OK) {
        return HAL_ERROR;  // Humidity CRC mismatch
    }

    // Convert raw values to physical units
    uint16_t raw_t  = ((uint16_t)rx[0] << 8) | rx[1];
    uint16_t raw_rh = ((uint16_t)rx[3] << 8) | rx[4];

    sensorObj->Data.Temperature = ((float)raw_t  * HDC302X_TEMP_COEFF1) - HDC302X_TEMP_COEFF2;
    sensorObj->Data.Humidity    =  (float)raw_rh * HDC302X_RH_COEFF;

    // Clamp humidity to valid physical range [0, 100]
    if (sensorObj->Data.Humidity < 0.0f)    sensorObj->Data.Humidity = 0.0f;
    if (sensorObj->Data.Humidity > 100.0f)  sensorObj->Data.Humidity = 100.0f;

    return HAL_OK;
}

// Read status register
/**
 * @brief HDC302x_ReadStatus function implementation.
 * @details Reads 3 bytes [MSB, LSB, CRC] and verifies CRC.
 *          Value stored as (MSB << 8) | LSB so bitfield positions match the datasheet.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ReadStatus(HDC302x_t* sensorObj) {
    HAL_StatusTypeDef status;
    uint8_t rx[3];  // [MSB, LSB, CRC]

    sensorObj->Config.CFG_CRC = 0x00;
    sensorObj->Cmd = HDC302X_READ_STATUS;
    status = SendDeviceCommand(sensorObj);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_I2C_Master_Receive(sensorObj->hi2c, sensorObj->Address, rx, sizeof(rx), I2C_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }

    if (VerifyCRC(rx) != HAL_OK) {
        return HAL_ERROR;  // Status CRC mismatch
    }

    sensorObj->Status.Val.Value = ((uint16_t)rx[0] << 8) | rx[1];
    return HAL_OK;
}

// Clear status register
/**
 * @brief HDC302x_ClearStatus function implementation.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ClearStatus(HDC302x_t* sensorObj) {
    sensorObj->Config.CFG_CRC = 0x00;
    sensorObj->Cmd = HDC302X_CLEAR_STATUS;
    return SendDeviceCommand(sensorObj);
}

// Read min/max history
/**
 * @brief HDC302x_ReadHistory function implementation.
 * @details Issues four sequential read commands to retrieve minimum and maximum
 *          recorded temperature and humidity values. Results are stored in
 *          sensorObj->History.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ReadHistory(HDC302x_t* sensorObj) {
    HAL_StatusTypeDef status;
    uint8_t rx[3];

    typedef struct {
        HDC302x_Command_t	cmd;
        float*				dest;
        float				coeff;
        float				offset;
    } HistoryRead_t;

    HistoryRead_t reads[4] = {
        { HDC302X_CMD_READ_HISTORY_MIN_TEMPERATURE, &sensorObj->History.MIN.Temperature, HDC302X_TEMP_COEFF1, -HDC302X_TEMP_COEFF2 },
        { HDC302X_CMD_READ_HISTORY_MAX_TEMPERATURE, &sensorObj->History.MAX.Temperature, HDC302X_TEMP_COEFF1, -HDC302X_TEMP_COEFF2 },
        { HDC302X_CMD_READ_HISTORY_MIN_RH,          &sensorObj->History.MIN.Humidity,    HDC302X_RH_COEFF,     0.0f               },
        { HDC302X_CMD_READ_HISTORY_MAX_RH,          &sensorObj->History.MAX.Humidity,    HDC302X_RH_COEFF,     0.0f               },
    };

    for (uint8_t i = 0; i < 4; i++) {
        sensorObj->Config.CFG_CRC = 0x00;
        sensorObj->Cmd = reads[i].cmd;

        status = SendDeviceCommand(sensorObj);
        if (status != HAL_OK) {
            return status;
        }

        status = HAL_I2C_Master_Receive(sensorObj->hi2c, sensorObj->Address, rx, sizeof(rx), I2C_TIMEOUT);
        if (status != HAL_OK) {
            return status;
        }

        if (VerifyCRC(rx) != HAL_OK) {
            return HAL_ERROR;  // CRC mismatch on history read
        }

        uint16_t raw = ((uint16_t)rx[0] << 8) | rx[1];
        *reads[i].dest = ((float)raw * reads[i].coeff) + reads[i].offset;
    }

    return HAL_OK;
}

// Dew point calculation
/**
 * @brief HDC302x_GetDewPoint function implementation.
 * @details Uses the Magnus-Tetens approximation:
 *            gamma = (A * T / (B + T)) + ln(RH / 100)
 *            Td    = B * gamma / (A - gamma)
 *          where A = 17.27, B = 237.7 (degrees C).
 *          Requires HDC302x_ReadData() to have been called first.
 * @param sensorObj Pointer to sensor handle.
 * @return Dew point in degrees Celsius.
 */
float HDC302x_GetDewPoint(HDC302x_t* sensorObj) {
    float t     = sensorObj->Data.Temperature;
    float rh    = sensorObj->Data.Humidity;
    float gamma = ((DEW_POINT_CONST_A * t) / (DEW_POINT_CONST_B + t)) + logf(rh / 100.0f);
    return (DEW_POINT_CONST_B * gamma) / (DEW_POINT_CONST_A - gamma);
}
