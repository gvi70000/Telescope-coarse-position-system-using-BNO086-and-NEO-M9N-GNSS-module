/**
 * @file EEPROM_SPI.c
 * @brief Implementation file for the M95512 SPI EEPROM library.
 *
 * Provides blocking SPI read/write operations for the ST M95512 512-Kbit
 * EEPROM (64 KB, 128-byte pages) over SPI1.
 *
 * Protocol notes (M95512 datasheet):
 *   - All instructions are MSB-first
 *   - RDSR (Figure 10): CS low -> 0x05 -> read status byte -> CS high (single transaction)
 *   - WRITE (Figure 13): CS low -> 0x02 -> addr_hi -> addr_lo -> data bytes -> CS high
 *     The rising edge of CS triggers the internal write cycle (tW = 5 ms max)
 *   - WEL is automatically reset after each WRITE or WRSR completion
 *   - Address is 16-bit: MSB byte first, only bits [14:0] are significant
 *
 * Target: STM32F302, Keil MDK, STM32 HAL
 */

#include "EEPROM_SPI.h"
#include "spi.h"
#include "gpio.h"
#include <stdio.h>
// Wait for SPI peripheral to become ready before asserting CS
/**
 * @brief EEPROM_isSpiReady function implementation.
 * @details Polls HAL SPI state until READY or timeout.
 * @param hspi Pointer to SPI handle.
 * @return HAL status.
 */
static HAL_StatusTypeDef EEPROM_isSpiReady(SPI_HandleTypeDef *hspi) {
    uint32_t startTime = HAL_GetTick();
    while (hspi->State != HAL_SPI_STATE_READY) {
        if (HAL_GetTick() - startTime >= EEPROM_TIMEOUT) {
            return HAL_TIMEOUT;  // SPI peripheral did not become ready in time
        }
        HAL_Delay(1);
    }
    return HAL_OK;
}

// Poll WIP bit until internal write cycle completes or timeout expires
/**
 * @brief EEPROM_WaitStandbyState function implementation.
 * @details Issues repeated RDSR transactions (CS low -> 0x05 -> read byte -> CS high)
 *          as shown in M95512 datasheet Figure 10 until WIP = 0 or timeout.
 *          Each poll is a complete self-contained CS transaction; splitting the
 *          command and receive into separate CS assertions would return garbage data.
 * @param hspi Pointer to SPI handle.
 * @return HAL status.
 */
static HAL_StatusTypeDef EEPROM_WaitStandbyState(SPI_HandleTypeDef *hspi) {
    uint8_t cmd    = RDSR;
    uint8_t status = 0;
    uint32_t startTime = HAL_GetTick();

    while (1) {
        if (HAL_GetTick() - startTime >= EEPROM_WRITE_TIMEOUT) {
            return HAL_TIMEOUT;  // Write cycle did not complete within EEPROM_WRITE_TIMEOUT
        }
        // Complete RDSR transaction: CS low -> send RDSR -> read status byte -> CS high
        EEPROM_SELECT;
        if (HAL_SPI_Transmit(hspi, &cmd, 1, EEPROM_TIMEOUT) != HAL_OK) {
            EEPROM_RELEASE;
            return HAL_ERROR;  // Failed to transmit RDSR command
        }
        if (HAL_SPI_Receive(hspi, &status, 1, EEPROM_TIMEOUT) != HAL_OK) {
            EEPROM_RELEASE;
            return HAL_ERROR;  // Failed to receive status byte
        }
        EEPROM_RELEASE;

        if ((status & WIP) == 0) {
            return HAL_OK;  // Write cycle complete
        }
        HAL_Delay(1);  // Avoid hammering the SPI bus; tW is up to 5 ms
    }
}

// INST 0x06 - Write Enable
/**
 * @brief EEPROM_WriteEnable function implementation.
 * @details Sets the Write Enable Latch (WEL) bit. Must be called before
 *          every WRITE or WRSR instruction; WEL is automatically cleared
 *          on completion of any write cycle.
 * @param hspi Pointer to SPI handle.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteEnable(SPI_HandleTypeDef *hspi) {
    uint8_t cmd = WREN;
    EEPROM_SELECT;
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &cmd, 1, EEPROM_TIMEOUT);
    EEPROM_RELEASE;
    return status;
}

// INST 0x04 - Write Disable
/**
 * @brief EEPROM_WriteDisable function implementation.
 * @details Resets the Write Enable Latch (WEL) bit.
 * @param hspi Pointer to SPI handle.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteDisable(SPI_HandleTypeDef *hspi) {
    uint8_t cmd = WRDI;
    EEPROM_SELECT;
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &cmd, 1, EEPROM_TIMEOUT);
    EEPROM_RELEASE;
    return status;
}

// INST 0x05 - Read Status Register
/**
 * @brief EEPROM_ReadStatus function implementation.
 * @details Issues a single CS transaction: send RDSR (0x05), read 1 status byte.
 *          Status register format (Table 6): SRWD | 0 | 0 | 0 | BP1 | BP0 | WEL | WIP
 * @param hspi Pointer to SPI handle.
 * @param status Pointer to byte to receive the status register value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_ReadStatus(SPI_HandleTypeDef *hspi, uint8_t *status) {
    uint8_t cmd = RDSR;
    EEPROM_SELECT;
    if (HAL_SPI_Transmit(hspi, &cmd, 1, EEPROM_TIMEOUT) != HAL_OK) {
        EEPROM_RELEASE;
        return HAL_ERROR;  // Failed to transmit RDSR command
    }
    if (HAL_SPI_Receive(hspi, status, 1, EEPROM_TIMEOUT) != HAL_OK) {
        EEPROM_RELEASE;
        return HAL_ERROR;  // Failed to receive status byte
    }
    EEPROM_RELEASE;
    return HAL_OK;
}

// INST 0x01 - Write Status Register
/**
 * @brief EEPROM_WriteStatus function implementation.
 * @details Sets SRWD and BP1/BP0 bits. Requires WEL set first; automatically
 *          issues WREN, then WRSR, then polls WIP until the write cycle completes.
 * @param hspi Pointer to SPI handle.
 * @param status New status register value (only SRWD, BP1, BP0 bits are writable).
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteStatus(SPI_HandleTypeDef *hspi, const uint8_t status) {
    if (EEPROM_WriteEnable(hspi) != HAL_OK) {
        return HAL_ERROR;  // Failed to set Write Enable Latch
    }
    uint8_t cmd[2] = {WRSR, status};
    EEPROM_SELECT;
    HAL_StatusTypeDef result = HAL_SPI_Transmit(hspi, cmd, 2, EEPROM_TIMEOUT);
    EEPROM_RELEASE;
    if (result != HAL_OK) {
        return HAL_ERROR;  // Failed to transmit WRSR command
    }
    return EEPROM_WaitStandbyState(hspi);  // Poll WIP until write cycle completes
}

// INST 0x03 - Read Memory Array
/**
 * @brief EEPROM_ReadData function implementation.
 * @details Reads size bytes starting at address. READ supports sequential
 *          reads across page boundaries without re-issuing the command.
 * @param hspi Pointer to SPI handle.
 * @param address Start address (0x0000..0xFFFF for M95512).
 * @param data Pointer to receive buffer.
 * @param size Number of bytes to read.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_ReadData(SPI_HandleTypeDef *hspi, const uint16_t address, uint8_t *data, const uint16_t size) {
    if ((uint32_t)address + size > EEPROM_TOTAL_SIZE) {
        return HAL_ERROR;  // Read would exceed chip address space
    }
    if (EEPROM_isSpiReady(hspi) != HAL_OK) {
        return HAL_ERROR;  // SPI peripheral not ready
    }
    uint8_t cmd[3] = {READ, (uint8_t)(address >> 8), (uint8_t)address};
    EEPROM_SELECT;
    if (HAL_SPI_Transmit(hspi, cmd, 3, EEPROM_TIMEOUT) != HAL_OK) {
        EEPROM_RELEASE;
        return HAL_ERROR;  // Failed to transmit READ command
    }
    HAL_StatusTypeDef result = HAL_SPI_Receive(hspi, data, size, EEPROM_TIMEOUT);
    EEPROM_RELEASE;
    return result;
}

// INST 0x02 - Write Memory Array (single page)
/**
 * @brief EEPROM_WritePage function implementation.
 * @details Writes up to PAGE_SIZE bytes within a single page. The write must
 *          not cross a page boundary; if it would, HAL_ERROR is returned.
 *          The rising edge of CS at the end of the transaction triggers the
 *          internal write cycle. WaitStandbyState() polls WIP until tW completes.
 * @param hspi Pointer to SPI handle.
 * @param address Start address (must not cross a page boundary).
 * @param data Pointer to data buffer.
 * @param size Number of bytes to write (1..PAGE_SIZE).
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WritePage(SPI_HandleTypeDef *hspi, const uint16_t address, const uint8_t *data, const uint8_t size) {
    if (size > PAGE_SIZE || (address / PAGE_SIZE) != ((address + size - 1) / PAGE_SIZE)) {
        return HAL_ERROR;  // Exceeds page size or spans page boundary
    }
    if ((uint32_t)address + size > EEPROM_TOTAL_SIZE) {
        return HAL_ERROR;  // Exceeds chip address space
    }
    if (EEPROM_isSpiReady(hspi) != HAL_OK) {
        return HAL_ERROR;  // SPI peripheral not ready
    }
    if (EEPROM_WriteEnable(hspi) != HAL_OK) {
        return HAL_ERROR;  // Failed to set Write Enable Latch
    }
    uint8_t cmd[3] = {WRITE, (uint8_t)(address >> 8), (uint8_t)address};
    EEPROM_SELECT;
    if (HAL_SPI_Transmit(hspi, cmd, 3, EEPROM_TIMEOUT) != HAL_OK) {
        EEPROM_RELEASE;
        return HAL_ERROR;  // Failed to transmit WRITE command
    }
    if (HAL_SPI_Transmit(hspi, (uint8_t *)data, size, EEPROM_TIMEOUT) != HAL_OK) {
        EEPROM_RELEASE;
        return HAL_ERROR;  // Failed to transmit data bytes
    }
    EEPROM_RELEASE;  // Rising CS edge triggers the internal write cycle (tW)
    return EEPROM_WaitStandbyState(hspi);  // Poll WIP until write cycle completes
}

// INST 0x02 - Write Memory Array (multi-page)
/**
 * @brief EEPROM_WriteDataMultiPage function implementation.
 * @details Splits an arbitrarily-sized write across as many pages as needed,
 *          aligning each sub-write to the page boundary. Each page write is a
 *          separate WREN + WRITE + WaitStandbyState sequence.
 * @param hspi Pointer to SPI handle.
 * @param startAddress Start address (may span multiple pages).
 * @param data Pointer to data buffer.
 * @param size Total number of bytes to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteDataMultiPage(SPI_HandleTypeDef *hspi, const uint16_t startAddress, const uint8_t *data, const uint16_t size) {
    if ((uint32_t)startAddress + size > EEPROM_TOTAL_SIZE) {
        return HAL_ERROR;  // Write would exceed chip address space
    }
    uint16_t bytesWritten   = 0;
    const uint8_t *pData    = data;

    while (bytesWritten < size) {
        uint16_t currentAddress = startAddress + bytesWritten;
        uint16_t spaceInPage    = PAGE_SIZE - (currentAddress % PAGE_SIZE);
        uint16_t bytesToWrite   = ((size - bytesWritten) < spaceInPage) ? (size - bytesWritten) : spaceInPage;

        HAL_StatusTypeDef status = EEPROM_WritePage(hspi, currentAddress, pData, (uint8_t)bytesToWrite);
        if (status != HAL_OK) {
            return status;  // Propagate error from page write
        }
        bytesWritten += bytesToWrite;
        pData        += bytesToWrite;
    }
    return HAL_OK;
}

// Typed write helpers — pass value by address, let EEPROM_WritePage handle the bytes
/**
 * @brief EEPROM_Write_8 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_8(SPI_HandleTypeDef *hspi, const uint16_t address, int8_t value) {
    return EEPROM_WritePage(hspi, address, (uint8_t *)&value, 1);
}

/**
 * @brief EEPROM_Write_16 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_16(SPI_HandleTypeDef *hspi, const uint16_t address, int16_t value) {
    return EEPROM_WritePage(hspi, address, (uint8_t *)&value, 2);
}

/**
 * @brief EEPROM_Write_32 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_32(SPI_HandleTypeDef *hspi, const uint16_t address, int32_t value) {
    return EEPROM_WritePage(hspi, address, (uint8_t *)&value, 4);
}

/**
 * @brief EEPROM_Write_64 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_64(SPI_HandleTypeDef *hspi, const uint16_t address, int64_t value) {
    return EEPROM_WritePage(hspi, address, (uint8_t *)&value, 8);
}

/**
 * @brief EEPROM_Write_U8 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U8(SPI_HandleTypeDef *hspi, const uint16_t address, uint8_t value) {
    return EEPROM_WritePage(hspi, address, &value, 1);
}

/**
 * @brief EEPROM_Write_U16 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U16(SPI_HandleTypeDef *hspi, const uint16_t address, uint16_t value) {
    return EEPROM_WritePage(hspi, address, (uint8_t *)&value, 2);
}

/**
 * @brief EEPROM_Write_U32 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U32(SPI_HandleTypeDef *hspi, const uint16_t address, uint32_t value) {
    return EEPROM_WritePage(hspi, address, (uint8_t *)&value, 4);
}

/**
 * @brief EEPROM_Write_U64 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U64(SPI_HandleTypeDef *hspi, const uint16_t address, uint64_t value) {
    return EEPROM_WritePage(hspi, address, (uint8_t *)&value, 8);
}

/**
 * @brief EEPROM_Write_Float function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_Float(SPI_HandleTypeDef *hspi, const uint16_t address, float value) {
    return EEPROM_WritePage(hspi, address, (uint8_t *)&value, 4);
}

/**
 * @brief EEPROM_Write_Double function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_Double(SPI_HandleTypeDef *hspi, const uint16_t address, double value) {
    return EEPROM_WritePage(hspi, address, (uint8_t *)&value, 8);
}

// Typed read helpers
/**
 * @brief EEPROM_Read_8 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_8(SPI_HandleTypeDef *hspi, const uint16_t address, int8_t *value) {
    return EEPROM_ReadData(hspi, address, (uint8_t *)value, 1);
}

/**
 * @brief EEPROM_Read_16 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_16(SPI_HandleTypeDef *hspi, const uint16_t address, int16_t *value) {
    return EEPROM_ReadData(hspi, address, (uint8_t *)value, 2);
}

/**
 * @brief EEPROM_Read_32 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_32(SPI_HandleTypeDef *hspi, const uint16_t address, int32_t *value) {
    return EEPROM_ReadData(hspi, address, (uint8_t *)value, 4);
}

/**
 * @brief EEPROM_Read_64 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_64(SPI_HandleTypeDef *hspi, const uint16_t address, int64_t *value) {
    return EEPROM_ReadData(hspi, address, (uint8_t *)value, 8);
}

/**
 * @brief EEPROM_Read_U8 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U8(SPI_HandleTypeDef *hspi, const uint16_t address, uint8_t *value) {
    return EEPROM_ReadData(hspi, address, value, 1);
}

/**
 * @brief EEPROM_Read_U16 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U16(SPI_HandleTypeDef *hspi, const uint16_t address, uint16_t *value) {
    return EEPROM_ReadData(hspi, address, (uint8_t *)value, 2);
}

/**
 * @brief EEPROM_Read_U32 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U32(SPI_HandleTypeDef *hspi, const uint16_t address, uint32_t *value) {
    return EEPROM_ReadData(hspi, address, (uint8_t *)value, 4);
}

/**
 * @brief EEPROM_Read_U64 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U64(SPI_HandleTypeDef *hspi, const uint16_t address, uint64_t *value) {
    return EEPROM_ReadData(hspi, address, (uint8_t *)value, 8);
}

// REG Float - safe type-pun via union (C99/C11 defined behaviour)
/**
 * @brief EEPROM_Read_Float function implementation.
 * @details Reads 4 bytes directly into a union and returns the float member,
 *          avoiding the strict-aliasing UB of casting uint8_t* to float*.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_Float(SPI_HandleTypeDef *hspi, const uint16_t address, float *value) {
    union { uint8_t b[4]; float f; } pun;
    HAL_StatusTypeDef status = EEPROM_ReadData(hspi, address, pun.b, 4);
    if (status != HAL_OK) {
        return status;
    }
    *value = pun.f;
    return HAL_OK;
}

// REG Double - safe type-pun via union (C99/C11 defined behaviour)
/**
 * @brief EEPROM_Read_Double function implementation.
 * @details Reads 8 bytes directly into a union and returns the double member,
 *          avoiding the strict-aliasing UB of casting uint8_t* to double*.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_Double(SPI_HandleTypeDef *hspi, const uint16_t address, double *value) {
    union { uint8_t b[8]; double d; } pun;
    HAL_StatusTypeDef status = EEPROM_ReadData(hspi, address, pun.b, 8);
    if (status != HAL_OK) {
        return status;
    }
    *value = pun.d;
    return HAL_OK;
}

// EEPROM self-test — call once after EEPROM_RELEASE in USER CODE BEGIN 2
// Tests: status register, typed writes/reads, page boundary enforcement,
// and a full pattern fill/verify across the first two pages (256 bytes).
// All results printed via printf (SWO/UART debug).
// Returns HAL_OK if all sub-tests pass, HAL_ERROR on first failure.
/**
 * @brief EEPROM_SelfTest function implementation.
 * @details Runs a sequence of non-destructive and write/verify tests against
 *          the M95512. Address range 0x0000..0x00FF is used (first two pages).
 *          Original content is NOT restored — call after first power-on or
 *          when the test address range is expendable.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_SelfTest(void) {
    printf("EEPROM self-test starting...\n");

    // Step 1: Read status register — WIP and WEL should both be 0 at idle
    uint8_t sr = 0xFF;
    if (EEPROM_ReadStatus(&hspi1, &sr) != HAL_OK) {
        printf("EEPROM: RDSR failed\n");
        return HAL_ERROR;
    }
    if (sr & WIP) {
        printf("EEPROM: WIP set at idle (status=0x%02X) — previous write incomplete?\n", sr);
        return HAL_ERROR;
    }
    printf("EEPROM: status register OK (0x%02X)\n", sr);

    // Step 2: Write Enable / Write Disable round-trip — verify WEL bit
    if (EEPROM_WriteEnable(&hspi1) != HAL_OK) {
        printf("EEPROM: WREN failed\n");
        return HAL_ERROR;
    }
    if (EEPROM_ReadStatus(&hspi1, &sr) != HAL_OK || !(sr & WEL)) {
        printf("EEPROM: WEL not set after WREN (status=0x%02X)\n", sr);
        return HAL_ERROR;
    }
    if (EEPROM_WriteDisable(&hspi1) != HAL_OK) {
        printf("EEPROM: WRDI failed\n");
        return HAL_ERROR;
    }
    if (EEPROM_ReadStatus(&hspi1, &sr) != HAL_OK || (sr & WEL)) {
        printf("EEPROM: WEL still set after WRDI (status=0x%02X)\n", sr);
        return HAL_ERROR;
    }
    printf("EEPROM: WEL toggle OK\n");

    // Step 3: Typed write/read round-trips at page 0 (offsets stay within page 0)
    // uint8  @ 0x0000
    if (EEPROM_Write_U8(&hspi1, 0x0000, 0xA5) != HAL_OK) { printf("EEPROM: Write_U8 failed\n");  return HAL_ERROR; }
    uint8_t u8 = 0;
    if (EEPROM_Read_U8(&hspi1, 0x0000, &u8) != HAL_OK || u8 != 0xA5) {
        printf("EEPROM: Read_U8 mismatch (got 0x%02X)\n", u8);
        return HAL_ERROR;
    }
    printf("EEPROM: U8  write/read OK (0x%02X)\n", u8);

    // uint16 @ 0x0002
    if (EEPROM_Write_U16(&hspi1, 0x0002, 0x1234) != HAL_OK) { printf("EEPROM: Write_U16 failed\n"); return HAL_ERROR; }
    uint16_t u16 = 0;
    if (EEPROM_Read_U16(&hspi1, 0x0002, &u16) != HAL_OK || u16 != 0x1234) {
        printf("EEPROM: Read_U16 mismatch (got 0x%04X)\n", u16);
        return HAL_ERROR;
    }
    printf("EEPROM: U16 write/read OK (0x%04X)\n", u16);

    // uint32 @ 0x0004
    if (EEPROM_Write_U32(&hspi1, 0x0004, 0xDEADBEEF) != HAL_OK) { printf("EEPROM: Write_U32 failed\n"); return HAL_ERROR; }
    uint32_t u32 = 0;
    if (EEPROM_Read_U32(&hspi1, 0x0004, &u32) != HAL_OK || u32 != 0xDEADBEEF) {
        printf("EEPROM: Read_U32 mismatch (got 0x%08X)\n", u32);
        return HAL_ERROR;
    }
    printf("EEPROM: U32 write/read OK (0x%08X)\n", u32);

    // float @ 0x0008
    float wf = 3.14159f, rf = 0.0f;
    if (EEPROM_Write_Float(&hspi1, 0x0008, wf) != HAL_OK) { printf("EEPROM: Write_Float failed\n"); return HAL_ERROR; }
    if (EEPROM_Read_Float(&hspi1, 0x0008, &rf) != HAL_OK || rf != wf) {
        printf("EEPROM: Read_Float mismatch (got %f)\n", rf);
        return HAL_ERROR;
    }
    printf("EEPROM: Float write/read OK (%f)\n", rf);

    // Step 4: Page boundary enforcement
    // A write of 2 bytes starting at the last byte of page 0 (0x007F) crosses into page 1
    uint8_t boundary[2] = {0xAA, 0xBB};
    if (EEPROM_WritePage(&hspi1, 0x007F, boundary, 2) != HAL_ERROR) {
        printf("EEPROM: page boundary check FAILED — cross-page write should have returned HAL_ERROR\n");
        return HAL_ERROR;
    }
    printf("EEPROM: page boundary check OK (cross-page write correctly rejected)\n");

    // Step 5: Pattern fill and verify across pages 0 and 1 (0x0000..0x00FF, 256 bytes)
    // Fill buffer with incrementing pattern
    uint8_t txBuf[256];
    uint8_t rxBuf[256];
    for (uint16_t i = 0; i < 256; i++) {
        txBuf[i] = (uint8_t)i;
    }
    if (EEPROM_WriteDataMultiPage(&hspi1, 0x0000, txBuf, 256) != HAL_OK) {
        printf("EEPROM: WriteDataMultiPage failed\n");
        return HAL_ERROR;
    }
    if (EEPROM_ReadData(&hspi1, 0x0000, rxBuf, 256) != HAL_OK) {
        printf("EEPROM: ReadData (pattern verify) failed\n");
        return HAL_ERROR;
    }
    for (uint16_t i = 0; i < 256; i++) {
        if (rxBuf[i] != txBuf[i]) {
            printf("EEPROM: pattern mismatch at offset %u (wrote 0x%02X, read 0x%02X)\n",
                   i, txBuf[i], rxBuf[i]);
            return HAL_ERROR;
        }
    }
    printf("EEPROM: 256-byte pattern fill/verify OK\n");

    printf("EEPROM self-test PASSED\n");
    return HAL_OK;
}
