#include "EEPROM_SPI.h"
#include "spi.h"
#include "gpio.h"

// Wait for SPI peripheral to become ready
/**
 * @brief EEPROM_isSpiReady function implementation.
 * @param hspi Pointer to SPI handle.
 * @return HAL status.
 */
static HAL_StatusTypeDef EEPROM_isSpiReady(SPI_HandleTypeDef* hspi) {
	uint32_t startTime = HAL_GetTick();
	while (hspi->State != HAL_SPI_STATE_READY) {
		if (HAL_GetTick() - startTime >= EEPROM_TIMEOUT) {
			return HAL_TIMEOUT;  // SPI peripheral did not become ready in time
		}
		HAL_Delay(1);
	}
	return HAL_OK;
}

// Poll WIP bit until write cycle completes or timeout expires
/**
 * @brief EEPROM_WaitStandbyState function implementation.
 * @details Polls the Write In Progress (WIP) bit of the status register.
 *          Each poll issues a complete RDSR transaction (CS low -> 0x05 -> read byte -> CS high)
 *          as required by the M95512 datasheet (Figure 10). The previous implementation
 *          incorrectly released CS between the command and the read, returning garbage data.
 * @param hspi Pointer to SPI handle.
 * @return HAL status.
 */
static HAL_StatusTypeDef EEPROM_WaitStandbyState(SPI_HandleTypeDef* hspi) {
	uint8_t cmd    = RDSR;
	uint8_t status = 0;
	uint32_t startTime = HAL_GetTick();

	while (1) {
		if (HAL_GetTick() - startTime >= EEPROM_WRITE_TIMEOUT) {
			return HAL_TIMEOUT;  // Write cycle did not complete within EEPROM_WRITE_TIMEOUT
		}

		// Each iteration is a complete, self-contained RDSR transaction:
		// CS low -> send RDSR command -> read status byte -> CS high
		EEPROM_SELECT;
		if (HAL_SPI_Transmit(hspi, &cmd, SIZE_1, EEPROM_TIMEOUT) != HAL_OK) {
			EEPROM_RELEASE;
			return HAL_ERROR;  // Failed to transmit RDSR command
		}
		if (HAL_SPI_Receive(hspi, &status, SIZE_1, EEPROM_TIMEOUT) != HAL_OK) {
			EEPROM_RELEASE;
			return HAL_ERROR;  // Failed to receive status byte
		}
		EEPROM_RELEASE;

		if ((status & WIP) == 0) {
			return HAL_OK;  // Write cycle complete
		}
		HAL_Delay(1);  // Avoid hammering the SPI bus; tW is up to 5ms
	}
}

// Enable write operation
/**
 * @brief EEPROM_WriteEnable function implementation.
 * @param hspi Pointer to SPI handle.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteEnable(SPI_HandleTypeDef* hspi) {
	uint8_t cmd = WREN;
	EEPROM_SELECT;
	HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &cmd, SIZE_1, EEPROM_TIMEOUT);
	EEPROM_RELEASE;
	return status;
}

// Disable write operation
/**
 * @brief EEPROM_WriteDisable function implementation.
 * @param hspi Pointer to SPI handle.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteDisable(SPI_HandleTypeDef* hspi) {
	uint8_t cmd = WRDI;
	EEPROM_SELECT;
	HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &cmd, SIZE_1, EEPROM_TIMEOUT);
	EEPROM_RELEASE;
	return status;
}

// Read status register
/**
 * @brief EEPROM_ReadStatus function implementation.
 * @details Issues a single CS-low transaction: send RDSR (0x05), read 1 status byte.
 *          The previous implementation split this into two separate CS transactions,
 *          which meant the receive had no valid command context and returned garbage.
 * @param hspi Pointer to SPI handle.
 * @param status Pointer to byte to receive the status register value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_ReadStatus(SPI_HandleTypeDef* hspi, uint8_t* status) {
	uint8_t cmd = RDSR;
	EEPROM_SELECT;
	if (HAL_SPI_Transmit(hspi, &cmd, SIZE_1, EEPROM_TIMEOUT) != HAL_OK) {
		EEPROM_RELEASE;
		return HAL_ERROR;  // Failed to transmit RDSR command
	}
	if (HAL_SPI_Receive(hspi, status, SIZE_1, EEPROM_TIMEOUT) != HAL_OK) {
		EEPROM_RELEASE;
		return HAL_ERROR;  // Failed to receive status byte
	}
	EEPROM_RELEASE;
	return HAL_OK;
}

// Write status register
/**
 * @brief EEPROM_WriteStatus function implementation.
 * @param hspi Pointer to SPI handle.
 * @param status New status register value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteStatus(SPI_HandleTypeDef* hspi, const uint8_t status) {
	if (EEPROM_WriteEnable(hspi) != HAL_OK) {
		return HAL_ERROR;  // Failed to set Write Enable Latch
	}
	uint8_t cmd[SIZE_2] = {WRSR, status};
	EEPROM_SELECT;
	HAL_StatusTypeDef result = HAL_SPI_Transmit(hspi, cmd, SIZE_2, EEPROM_TIMEOUT);
	EEPROM_RELEASE;
	if (result != HAL_OK) {
		return HAL_ERROR;  // Failed to transmit WRSR command
	}
	return EEPROM_WaitStandbyState(hspi);  // Wait for write cycle to complete
}

// Read an amount of data from EEPROM
/**
 * @brief EEPROM_ReadData function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Start address.
 * @param data Pointer to receive buffer.
 * @param size Number of bytes to read.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_ReadData(SPI_HandleTypeDef* hspi, const uint16_t address, uint8_t* data, const uint16_t size) {
	if ((uint32_t)address + size > EEPROM_TOTAL_SIZE) {
		return HAL_ERROR;  // Read would exceed chip address space
	}
	if (EEPROM_isSpiReady(hspi) != HAL_OK) {
		return HAL_ERROR;  // SPI peripheral not ready
	}
	uint8_t cmd[SIZE_3] = {READ, (uint8_t)(address >> 8), (uint8_t)address};
	EEPROM_SELECT;
	if (HAL_SPI_Transmit(hspi, cmd, SIZE_3, EEPROM_TIMEOUT) != HAL_OK) {
		EEPROM_RELEASE;
		return HAL_ERROR;  // Failed to transmit READ command
	}
	HAL_StatusTypeDef result = HAL_SPI_Receive(hspi, data, size, EEPROM_TIMEOUT);
	EEPROM_RELEASE;
	return result;
}

// Write a single page of data (must not cross a page boundary)
/**
 * @brief EEPROM_WritePage function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Start address.
 * @param data Pointer to data buffer.
 * @param size Number of bytes to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WritePage(SPI_HandleTypeDef* hspi, const uint16_t address, const uint8_t* data, const uint8_t size) {
	// Validate: size must not exceed page size, and write must not cross a page boundary
	if (size > PAGE_SIZE || (address / PAGE_SIZE) != ((address + size - 1) / PAGE_SIZE)) {
		return HAL_ERROR;  // Exceeds page size or spans page boundary
	}
	// Validate: write must not exceed total chip address space
	if ((uint32_t)address + size > EEPROM_TOTAL_SIZE) {
		return HAL_ERROR;  // Exceeds chip address space
	}
	if (EEPROM_isSpiReady(hspi) != HAL_OK) {
		return HAL_ERROR;  // SPI peripheral not ready
	}
	if (EEPROM_WriteEnable(hspi) != HAL_OK) {
		return HAL_ERROR;  // Failed to set Write Enable Latch
	}
	uint8_t cmd[SIZE_3] = {WRITE, (uint8_t)(address >> 8), (uint8_t)address};
	EEPROM_SELECT;
	if (HAL_SPI_Transmit(hspi, cmd, SIZE_3, EEPROM_TIMEOUT) != HAL_OK) {
		EEPROM_RELEASE;
		return HAL_ERROR;  // Failed to transmit WRITE command
	}
	if (HAL_SPI_Transmit(hspi, (uint8_t*)data, size, EEPROM_TIMEOUT) != HAL_OK) {
		EEPROM_RELEASE;
		return HAL_ERROR;  // Failed to transmit data bytes
	}
	EEPROM_RELEASE;  // Rising CS edge triggers the internal write cycle (tW)
	return EEPROM_WaitStandbyState(hspi);  // Poll WIP until write cycle completes
}

// Write any amount of data, splitting across pages as needed
/**
 * @brief EEPROM_WriteDataMultiPage function implementation.
 * @param hspi Pointer to SPI handle.
 * @param startAddress Start address.
 * @param data Pointer to data buffer.
 * @param size Total number of bytes to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteDataMultiPage(SPI_HandleTypeDef* hspi, const uint16_t startAddress, const uint8_t* data, const uint16_t size) {
	if ((uint32_t)startAddress + size > EEPROM_TOTAL_SIZE) {
		return HAL_ERROR;  // Write would exceed chip address space
	}
	uint16_t bytesWritten = 0;
	const uint8_t* currentData = data;

	while (bytesWritten < size) {
		uint16_t currentAddress  = startAddress + bytesWritten;
		uint16_t spaceInPage     = PAGE_SIZE - (currentAddress % PAGE_SIZE);
		uint16_t bytesToWrite    = ((size - bytesWritten) < spaceInPage) ? (size - bytesWritten) : spaceInPage;

		HAL_StatusTypeDef status = EEPROM_WritePage(hspi, currentAddress, currentData, (uint8_t)bytesToWrite);
		if (status != HAL_OK) {
			return status;  // Propagate error from page write
		}
		bytesWritten += bytesToWrite;
		currentData  += bytesToWrite;
	}
	return HAL_OK;
}

// Typed write helpers
/**
 * @brief EEPROM_Write_8 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_8(SPI_HandleTypeDef* hspi, const uint16_t address, int8_t value) {
	return EEPROM_WritePage(hspi, address, (uint8_t*)&value, SIZE_1);
}

/**
 * @brief EEPROM_Write_16 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_16(SPI_HandleTypeDef* hspi, const uint16_t address, int16_t value) {
	return EEPROM_WritePage(hspi, address, (uint8_t*)&value, SIZE_2);
}

/**
 * @brief EEPROM_Write_32 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_32(SPI_HandleTypeDef* hspi, const uint16_t address, int32_t value) {
	return EEPROM_WritePage(hspi, address, (uint8_t*)&value, SIZE_4);
}

/**
 * @brief EEPROM_Write_64 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_64(SPI_HandleTypeDef* hspi, const uint16_t address, int64_t value) {
	return EEPROM_WritePage(hspi, address, (uint8_t*)&value, SIZE_8);
}

/**
 * @brief EEPROM_Write_U8 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U8(SPI_HandleTypeDef* hspi, const uint16_t address, uint8_t value) {
	return EEPROM_WritePage(hspi, address, &value, SIZE_1);
}

/**
 * @brief EEPROM_Write_U16 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U16(SPI_HandleTypeDef* hspi, const uint16_t address, uint16_t value) {
	return EEPROM_WritePage(hspi, address, (uint8_t*)&value, SIZE_2);
}

/**
 * @brief EEPROM_Write_U32 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U32(SPI_HandleTypeDef* hspi, const uint16_t address, uint32_t value) {
	return EEPROM_WritePage(hspi, address, (uint8_t*)&value, SIZE_4);
}

/**
 * @brief EEPROM_Write_U64 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U64(SPI_HandleTypeDef* hspi, const uint16_t address, uint64_t value) {
	return EEPROM_WritePage(hspi, address, (uint8_t*)&value, SIZE_8);
}

/**
 * @brief EEPROM_Write_Float function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_Float(SPI_HandleTypeDef* hspi, const uint16_t address, float value) {
	return EEPROM_WritePage(hspi, address, (uint8_t*)&value, SIZE_4);
}

/**
 * @brief EEPROM_Write_Double function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_Double(SPI_HandleTypeDef* hspi, const uint16_t address, double value) {
	return EEPROM_WritePage(hspi, address, (uint8_t*)&value, SIZE_8);
}

// Typed read helpers
/**
 * @brief EEPROM_Read_8 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_8(SPI_HandleTypeDef* hspi, const uint16_t address, int8_t* value) {
	return EEPROM_ReadData(hspi, address, (uint8_t*)value, SIZE_1);
}

/**
 * @brief EEPROM_Read_16 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_16(SPI_HandleTypeDef* hspi, const uint16_t address, int16_t* value) {
	return EEPROM_ReadData(hspi, address, (uint8_t*)value, SIZE_2);
}

/**
 * @brief EEPROM_Read_32 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_32(SPI_HandleTypeDef* hspi, const uint16_t address, int32_t* value) {
	return EEPROM_ReadData(hspi, address, (uint8_t*)value, SIZE_4);
}

/**
 * @brief EEPROM_Read_64 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_64(SPI_HandleTypeDef* hspi, const uint16_t address, int64_t* value) {
	return EEPROM_ReadData(hspi, address, (uint8_t*)value, SIZE_8);
}

/**
 * @brief EEPROM_Read_U8 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U8(SPI_HandleTypeDef* hspi, const uint16_t address, uint8_t* value) {
	return EEPROM_ReadData(hspi, address, value, SIZE_1);
}

/**
 * @brief EEPROM_Read_U16 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U16(SPI_HandleTypeDef* hspi, const uint16_t address, uint16_t* value) {
	return EEPROM_ReadData(hspi, address, (uint8_t*)value, SIZE_2);
}

/**
 * @brief EEPROM_Read_U32 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U32(SPI_HandleTypeDef* hspi, const uint16_t address, uint32_t* value) {
	return EEPROM_ReadData(hspi, address, (uint8_t*)value, SIZE_4);
}

/**
 * @brief EEPROM_Read_U64 function implementation.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U64(SPI_HandleTypeDef* hspi, const uint16_t address, uint64_t* value) {
	return EEPROM_ReadData(hspi, address, (uint8_t*)value, SIZE_8);
}

/**
 * @brief EEPROM_Read_Float function implementation.
 * @details Uses a byte-copy via memcpy-equivalent union to avoid strict aliasing violation.
 *          The previous implementation cast float* to int32_t* which is undefined behaviour.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_Float(SPI_HandleTypeDef* hspi, const uint16_t address, float* value) {
	uint8_t bytes[SIZE_4];
	HAL_StatusTypeDef status = EEPROM_ReadData(hspi, address, bytes, SIZE_4);
	if (status != HAL_OK) {
		return status;
	}
	// Safe type-pun via union (defined behaviour in C99/C11)
	union { uint8_t b[4]; float f; } pun;
	pun.b[0] = bytes[0];
	pun.b[1] = bytes[1];
	pun.b[2] = bytes[2];
	pun.b[3] = bytes[3];
	*value = pun.f;
	return HAL_OK;
}

/**
 * @brief EEPROM_Read_Double function implementation.
 * @details Uses a byte-copy via union to avoid strict aliasing violation.
 *          The previous implementation cast double* to int64_t* which is undefined behaviour.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_Double(SPI_HandleTypeDef* hspi, const uint16_t address, double* value) {
	uint8_t bytes[SIZE_8];
	HAL_StatusTypeDef status = EEPROM_ReadData(hspi, address, bytes, SIZE_8);
	if (status != HAL_OK) {
		return status;
	}
	// Safe type-pun via union (defined behaviour in C99/C11)
	union { uint8_t b[8]; double d; } pun;
	pun.b[0] = bytes[0]; pun.b[1] = bytes[1];
	pun.b[2] = bytes[2]; pun.b[3] = bytes[3];
	pun.b[4] = bytes[4]; pun.b[5] = bytes[5];
	pun.b[6] = bytes[6]; pun.b[7] = bytes[7];
	*value = pun.d;
	return HAL_OK;
}
