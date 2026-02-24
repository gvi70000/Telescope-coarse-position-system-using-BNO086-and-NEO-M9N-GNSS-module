#ifndef __EEPROM_SPI_H
#define __EEPROM_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

// Memory geometry — adjust PAGES to match your chip variant
#define PAGE_SIZE					128		// Bytes per page
#define PAGES						512		// Total pages (512 x 128 = 65536 bytes for M95512)
#define EEPROM_TOTAL_SIZE			(PAGE_SIZE * PAGES)		// Total addressable bytes

// Timeouts
#define EEPROM_TIMEOUT				100		// SPI transaction timeout (ms)
#define EEPROM_WRITE_TIMEOUT		20		// Max write cycle wait (ms) — datasheet tW = 5ms, 20ms adds margin

// CS pin control — direct register access for minimum SPI overhead on STM32F302
#define EEPROM_SELECT				CS_MEM_GPIO_Port->BRR  = (uint32_t)CS_MEM_Pin		// CS low  (assert)
#define EEPROM_RELEASE				CS_MEM_GPIO_Port->BSRR = (uint32_t)CS_MEM_Pin		// CS high (deassert)

// Buffer size constants
#define SIZE_1						1
#define SIZE_2						2
#define SIZE_3						3
#define SIZE_4						4
#define SIZE_8						8

// SPI instruction set (M95512 datasheet Table 4)
#define WREN						0x06	// Write Enable          0000 0110
#define WRDI						0x04	// Write Disable         0000 0100
#define RDSR						0x05	// Read Status Register  0000 0101
#define WRSR						0x01	// Write Status Register 0000 0001
#define READ						0x03	// Read Memory Array     0000 0011
#define WRITE						0x02	// Write Memory Array    0000 0010

// Status register bit masks
#define WIP							0x01	// Write In Progress bit (bit 0)
#define WEL							0x02	// Write Enable Latch bit (bit 1)

// Function prototypes
/**
 * @brief EEPROM_WriteEnable function.
 * @param hspi Pointer to SPI handle.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteEnable(SPI_HandleTypeDef* hspi);
/**
 * @brief EEPROM_WriteDisable function.
 * @param hspi Pointer to SPI handle.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteDisable(SPI_HandleTypeDef* hspi);
/**
 * @brief EEPROM_ReadStatus function.
 * @param hspi Pointer to SPI handle.
 * @param status Pointer to byte to receive the status register value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_ReadStatus(SPI_HandleTypeDef* hspi, uint8_t* status);
/**
 * @brief EEPROM_WriteStatus function.
 * @param hspi Pointer to SPI handle.
 * @param status New status register value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteStatus(SPI_HandleTypeDef* hspi, const uint8_t status);
/**
 * @brief EEPROM_ReadData function.
 * @param hspi Pointer to SPI handle.
 * @param address Start address to read from.
 * @param data Pointer to receive buffer.
 * @param size Number of bytes to read.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_ReadData(SPI_HandleTypeDef* hspi, const uint16_t address, uint8_t* data, const uint16_t size);
/**
 * @brief EEPROM_WritePage function.
 * @param hspi Pointer to SPI handle.
 * @param address Start address (must not cross a page boundary).
 * @param data Pointer to data buffer.
 * @param size Number of bytes to write (max PAGE_SIZE).
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WritePage(SPI_HandleTypeDef* hspi, const uint16_t address, const uint8_t* data, const uint8_t size);
/**
 * @brief EEPROM_WriteDataMultiPage function.
 * @param hspi Pointer to SPI handle.
 * @param startAddress Start address (may span multiple pages).
 * @param data Pointer to data buffer.
 * @param size Total number of bytes to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_WriteDataMultiPage(SPI_HandleTypeDef* hspi, const uint16_t startAddress, const uint8_t* data, const uint16_t size);

/**
 * @brief EEPROM_Write_8 function.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_8(SPI_HandleTypeDef* hspi, const uint16_t address, int8_t value);
/**
 * @brief EEPROM_Write_16 function.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_16(SPI_HandleTypeDef* hspi, const uint16_t address, int16_t value);
/**
 * @brief EEPROM_Write_32 function.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_32(SPI_HandleTypeDef* hspi, const uint16_t address, int32_t value);
/**
 * @brief EEPROM_Write_64 function.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_64(SPI_HandleTypeDef* hspi, const uint16_t address, int64_t value);
/**
 * @brief EEPROM_Write_U8 function.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U8(SPI_HandleTypeDef* hspi, const uint16_t address, uint8_t value);
/**
 * @brief EEPROM_Write_U16 function.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U16(SPI_HandleTypeDef* hspi, const uint16_t address, uint16_t value);
/**
 * @brief EEPROM_Write_U32 function.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U32(SPI_HandleTypeDef* hspi, const uint16_t address, uint32_t value);
/**
 * @brief EEPROM_Write_U64 function.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_U64(SPI_HandleTypeDef* hspi, const uint16_t address, uint64_t value);
/**
 * @brief EEPROM_Write_Float function.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_Float(SPI_HandleTypeDef* hspi, const uint16_t address, float value);
/**
 * @brief EEPROM_Write_Double function.
 * @param hspi Pointer to SPI handle.
 * @param address Target address.
 * @param value Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Write_Double(SPI_HandleTypeDef* hspi, const uint16_t address, double value);

/**
 * @brief EEPROM_Read_8 function.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_8(SPI_HandleTypeDef* hspi, const uint16_t address, int8_t* value);
/**
 * @brief EEPROM_Read_16 function.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_16(SPI_HandleTypeDef* hspi, const uint16_t address, int16_t* value);
/**
 * @brief EEPROM_Read_32 function.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_32(SPI_HandleTypeDef* hspi, const uint16_t address, int32_t* value);
/**
 * @brief EEPROM_Read_64 function.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_64(SPI_HandleTypeDef* hspi, const uint16_t address, int64_t* value);
/**
 * @brief EEPROM_Read_U8 function.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U8(SPI_HandleTypeDef* hspi, const uint16_t address, uint8_t* value);
/**
 * @brief EEPROM_Read_U16 function.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U16(SPI_HandleTypeDef* hspi, const uint16_t address, uint16_t* value);
/**
 * @brief EEPROM_Read_U32 function.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U32(SPI_HandleTypeDef* hspi, const uint16_t address, uint32_t* value);
/**
 * @brief EEPROM_Read_U64 function.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_U64(SPI_HandleTypeDef* hspi, const uint16_t address, uint64_t* value);
/**
 * @brief EEPROM_Read_Float function.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_Float(SPI_HandleTypeDef* hspi, const uint16_t address, float* value);
/**
 * @brief EEPROM_Read_Double function.
 * @param hspi Pointer to SPI handle.
 * @param address Source address.
 * @param value Pointer to receive the value.
 * @return HAL status.
 */
HAL_StatusTypeDef EEPROM_Read_Double(SPI_HandleTypeDef* hspi, const uint16_t address, double* value);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_SPI_H */
