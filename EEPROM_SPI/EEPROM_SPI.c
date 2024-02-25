#include "EEPROM_SPI.h"
#include "spi.h"
#include "gpio.h"

// 
static HAL_StatusTypeDef EEPROM_isSpiReady(){
	uint32_t startTime = HAL_GetTick();
	while(hspi1.State != HAL_SPI_STATE_READY) {
		if (HAL_GetTick() - startTime >= EEPROM_TIMEOUT) return HAL_TIMEOUT;  // Return timeout status if we exceed the timeout duration
		HAL_Delay(1);
	}
	return HAL_OK;
}

static HAL_StatusTypeDef EEPROM_WaitStandbyState(void) {
	EEPROM_SELECT;
	// Transmit the command to read the status register
	if(HAL_SPI_Transmit(&hspi1, (uint8_t[]){RDSR}, SIZE_1, EEPROM_TIMEOUT) != HAL_OK) {
		EEPROM_RELEASE;
		return HAL_ERROR; // Failed to transmit the RDSR command
	}
	uint32_t startTime = HAL_GetTick(); // Store the start time
	uint8_t status = 0;
	while(1){
		// Check if the timeout has been exceeded
		if (HAL_GetTick() - startTime >= EEPROM_TIMEOUT) {
			EEPROM_RELEASE;
			return HAL_TIMEOUT; // Return timeout status if we exceed the timeout duration
		}
		// Receive the status register
		if(HAL_SPI_Receive(&hspi1, &status, SIZE_1, EEPROM_TIMEOUT) != HAL_OK){
			EEPROM_RELEASE;
			return HAL_ERROR; // Error in receiving data
		}
		// Check the Write In Progress bit
		if ((status & WIP) == 0) break; // Exit the loop if the EEPROM is not busy
		// Adding a delay to avoid spamming the SPI bus too aggressively
		HAL_Delay(1);
	};
	EEPROM_RELEASE;
	return HAL_OK; // EEPROM is ready
}

// Enable write operation
HAL_StatusTypeDef EEPROM_WriteEnable(void) {
	EEPROM_SELECT;
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, (uint8_t[]){WREN}, SIZE_1, EEPROM_TIMEOUT);
	EEPROM_RELEASE;
	return status;
}

// Disable write operation
HAL_StatusTypeDef EEPROM_WriteDisable(void) {
	EEPROM_SELECT;
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, (uint8_t[]){WRDI}, SIZE_1, EEPROM_TIMEOUT);
	EEPROM_RELEASE;
	return status;
}

// Read status register
uint8_t EEPROM_ReadStatus(void) {
	uint8_t status;
	EEPROM_SELECT;
	HAL_SPI_Transmit(&hspi1, (uint8_t[]){RDSR}, SIZE_1, EEPROM_TIMEOUT);
	EEPROM_RELEASE;
	EEPROM_SELECT;
	HAL_SPI_Receive(&hspi1, &status, SIZE_1, EEPROM_TIMEOUT);
	EEPROM_RELEASE;
	return status;
}

// Write status register
HAL_StatusTypeDef EEPROM_WriteStatus(const uint8_t status) {
	// Attempt to enable writing to the EEPROM first.
	if(EEPROM_WriteEnable() != HAL_OK) {
		return HAL_ERROR; // Early return if write enable fails.
	}
	// Prepare the command buffer with the Write Status Register command and the new status.
	uint8_t cmd[SIZE_2] = {WRSR, status};
	EEPROM_SELECT; // Select the EEPROM to start the communication.
	// Transmit the command and the new status to the EEPROM.
	HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, cmd, SIZE_2, EEPROM_TIMEOUT);
	EEPROM_RELEASE; // Release the EEPROM as soon as the transmission is done.
	// If the SPI transmission was successful, attempt to disable writing.
	// Otherwise, return HAL_ERROR immediately.
	if(result == HAL_OK) {
		return EEPROM_WriteDisable(); // Attempt to disable writing and return the result.
	}
	return HAL_ERROR; // Return HAL_ERROR if the transmission failed.
}

// Read an ammout of data from EEPROM
HAL_StatusTypeDef EEPROM_ReadData(const uint16_t address, uint8_t *data, const uint16_t size) {
	// First, check if SPI is ready for communication.
	if (EEPROM_isSpiReady() != HAL_OK) return HAL_ERROR; // SPI is not ready, return error immediately.
	EEPROM_SELECT; // Select the EEPROM to start the communication.
	// Prepare the command buffer with the Read command followed by the address.
	uint8_t cmd[SIZE_3] = {READ, (uint8_t)(address >> 8), (uint8_t)address};
	// Transmit the read command with the address.
	if (HAL_SPI_Transmit(&hspi1, cmd, SIZE_3, EEPROM_TIMEOUT) != HAL_OK) {
		EEPROM_RELEASE; // Make sure to release the EEPROM before returning.
		return HAL_ERROR; // Transmission failed, return error.
	}
	// Receive the data from EEPROM.
	HAL_StatusTypeDef result = HAL_SPI_Receive(&hspi1, data, size, EEPROM_TIMEOUT);
	EEPROM_RELEASE; // Release the EEPROM after receiving the data.
	return result; // Return the result of the SPI receive operation.
}

// Function to write a page of data to EEPROM at a specified address
HAL_StatusTypeDef EEPROM_WritePage(const uint16_t address, const uint8_t *data, const uint8_t size) {
	// Validate that the write operation does not exceed the EEPROM page boundaries.
	if (size > PAGE_SIZE || (address / PAGE_SIZE) != ((address + size - 1) / PAGE_SIZE)) {
		return HAL_ERROR; // Exceeds page size or spans across pages.
	}
	// Ensure SPI is ready and writing is enabled before proceeding.
	if (EEPROM_isSpiReady() != HAL_OK || EEPROM_WriteEnable() != HAL_OK) {
		return HAL_ERROR; // SPI not ready or unable to enable write operation.
	}
	EEPROM_SELECT; // Select the EEPROM to start the communication.
	// Prepare and transmit the WRITE command along with the address.
	uint8_t cmd[SIZE_3] = {WRITE, (uint8_t)(address >> 8), (uint8_t)address};
	if (HAL_SPI_Transmit(&hspi1, cmd, SIZE_3, EEPROM_TIMEOUT) != HAL_OK) {
		EEPROM_RELEASE;
		return HAL_ERROR; // Failed to transmit the write command.
	}
	// Transmit the data to be written to the EEPROM.
	if (HAL_SPI_Transmit(&hspi1, (uint8_t*)data, size, EEPROM_TIMEOUT) != HAL_OK) {
		EEPROM_RELEASE;
		return HAL_ERROR; // Failed to transmit the data.
	}
	EEPROM_RELEASE; // Release the EEPROM after the transmit operation.
	// Wait for the EEPROM to complete the write operation.
	if (EEPROM_WaitStandbyState() != HAL_OK) {
		return HAL_ERROR; // Write operation did not complete successfully.
	}
	// Finally, disable the write operation.
	return EEPROM_WriteDisable();
}

// Write any amount of data across several pages in an EEPROM
HAL_StatusTypeDef EEPROM_WriteDataMultiPage(const uint16_t startAddress, const uint8_t* data, const uint16_t size) {
	uint16_t bytesWritten = 0; // Track the number of bytes written
	const uint8_t* currentDataPointer = data; // Pointer to track the current position in the data buffer
	while (bytesWritten < size) {
		// Calculate the start address for the current chunk of data
		uint16_t currentAddress = startAddress + bytesWritten;
		// Calculate how much data can be written in the current page without crossing the boundary
		uint16_t currentPageSize = PAGE_SIZE - (currentAddress % PAGE_SIZE);
		uint16_t bytesToWrite = ((size - bytesWritten) < currentPageSize) ? (size - bytesWritten) : currentPageSize;
		// Write the data to the current page
		HAL_StatusTypeDef status = EEPROM_WritePage(currentAddress, currentDataPointer, bytesToWrite);
		if (status != HAL_OK) {
			// Handle error
			return status; // Return the error status if the write operation failed
		}
		// Update the number of bytes written and the data pointer
		bytesWritten += bytesToWrite;
		currentDataPointer += bytesToWrite;
	}
	return HAL_OK; // Return success after writing all data
}

HAL_StatusTypeDef EEPROM_Write_8(const uint16_t address, int8_t value) {
	return EEPROM_WritePage(address, (uint8_t *)&value, SIZE_1);
}

HAL_StatusTypeDef EEPROM_Write_16(const uint16_t address, int16_t value) {
	return EEPROM_WritePage(address, (uint8_t *)&value, SIZE_2);
}

HAL_StatusTypeDef EEPROM_Write_32(const uint16_t address, int32_t value) {
	return EEPROM_WritePage(address, (uint8_t *)&value, SIZE_4);
}

HAL_StatusTypeDef EEPROM_Write_64(const uint16_t address, int64_t value) {
	return EEPROM_WritePage(address, (uint8_t *)&value, SIZE_8);
}

HAL_StatusTypeDef EEPROM_Write_U8(const uint16_t address, uint8_t value) {
	return EEPROM_WritePage(address, &value, SIZE_1);
}

HAL_StatusTypeDef EEPROM_Write_U16(const uint16_t address, const uint16_t value) {
	return EEPROM_Write_16(address, (int16_t)value);
}

HAL_StatusTypeDef EEPROM_Write_U32(const uint16_t address, const uint32_t value) {
	return EEPROM_Write_32(address, (int32_t)value);
}

HAL_StatusTypeDef EEPROM_Write_Float(const uint16_t address, const float value) {
	return EEPROM_WritePage(address, (uint8_t *)&value, SIZE_4);
}

HAL_StatusTypeDef EEPROM_Write_U64(const uint16_t address, const uint64_t value) {
	return EEPROM_Write_64(address, (int64_t)value);
}

HAL_StatusTypeDef EEPROM_Write_Double(const uint16_t address, const double value) {
	return EEPROM_WritePage(address, (uint8_t *)&value, SIZE_8);
}

HAL_StatusTypeDef EEPROM_Read_8(const uint16_t address, int8_t *value) {
	return EEPROM_ReadData(address, (uint8_t *)value, SIZE_1);
}

HAL_StatusTypeDef EEPROM_Read_16(const uint16_t address, int16_t *value) {
	return EEPROM_ReadData(address, (uint8_t *)value, SIZE_2);
}

HAL_StatusTypeDef EEPROM_Read_32(const uint16_t address, int32_t *value) {
	return EEPROM_ReadData(address, (uint8_t *)value, SIZE_4);
}

HAL_StatusTypeDef EEPROM_Read_64(const uint16_t address, int64_t *value) {
	return EEPROM_ReadData(address, (uint8_t *)value, SIZE_8);
}

HAL_StatusTypeDef EEPROM_Read_U8(const uint16_t address, uint8_t *value) {
	return EEPROM_ReadData(address, value, SIZE_1);
}

HAL_StatusTypeDef EEPROM_Read_U16(const uint16_t address, uint16_t *value) {
	return EEPROM_Read_16(address, (int16_t *)value);
}

HAL_StatusTypeDef EEPROM_Read_U32(const uint16_t address, uint32_t *value) {
	return EEPROM_Read_32(address, (int32_t *)value);
}

HAL_StatusTypeDef EEPROM_Read_Float(const uint16_t address, float *value) {
	return EEPROM_Read_32(address, (int32_t *)value);
}

HAL_StatusTypeDef EEPROM_Read_U64(const uint16_t address, uint64_t *value) {
	return EEPROM_Read_64(address, (int64_t *)value);
}

HAL_StatusTypeDef EEPROM_Read_Double(const uint16_t address, double *value) {
	return EEPROM_Read_64(address, (int64_t *)value);
}