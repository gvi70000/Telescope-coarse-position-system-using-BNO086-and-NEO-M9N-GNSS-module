/**
 * @file EEPROM_SPI.h
 * @brief Header file for the M95512 SPI EEPROM library.
 *
 * Provides definitions and function prototypes for reading and writing the
 * ST M95512 512-Kbit SPI EEPROM (64 KB, 128-byte pages) over SPI1.
 *
 * Instruction set and protocol are compatible with the M95256/M95512 family.
 * Reference: M95512 datasheet, ST DocID028475.
 *
 * CS pin is controlled directly via GPIO BRR/BSRR for minimum SPI overhead.
 * All multi-byte typed helpers use little-endian byte order (Cortex-M4 native).
 *
 * Target: STM32F302, Keil MDK, STM32 HAL
 */

#ifndef __EEPROM_SPI_H
#define __EEPROM_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

// Memory geometry — adjust PAGES to match your chip variant
#define PAGE_SIZE				128U					// Bytes per page
#define PAGES					512U					// Total pages (512 x 128 = 65536 bytes for M95512)
#define EEPROM_TOTAL_SIZE		(PAGE_SIZE * PAGES)		// Total addressable bytes

// Timeouts
#define EEPROM_TIMEOUT			100U	// SPI transaction timeout (ms)
#define EEPROM_WRITE_TIMEOUT	20U		// Max write cycle wait (ms) — datasheet tW = 5 ms, 20 ms adds margin

// CS pin control — direct register access for minimum SPI overhead on STM32F302
#define EEPROM_SELECT			CS_MEM_GPIO_Port->BRR  = (uint32_t)CS_MEM_Pin	// CS low  (assert)
#define EEPROM_RELEASE			CS_MEM_GPIO_Port->BSRR = (uint32_t)CS_MEM_Pin	// CS high (deassert)

// SPI instruction set (M95512 datasheet Table 4)
#define WREN					0x06	// Write Enable          0000 0110
#define WRDI					0x04	// Write Disable         0000 0100
#define RDSR					0x05	// Read Status Register  0000 0101
#define WRSR					0x01	// Write Status Register 0000 0001
#define READ					0x03	// Read Memory Array     0000 0011
#define WRITE					0x02	// Write Memory Array    0000 0010

// Status register bit masks (Table 6: SRWD | 0 | 0 | 0 | BP1 | BP0 | WEL | WIP)
#define WIP						0x01	// Write In Progress (bit 0) — 1 = write cycle in progress
#define WEL						0x02	// Write Enable Latch (bit 1) — 1 = write enabled

// Core read/write functions
HAL_StatusTypeDef EEPROM_WriteEnable(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef EEPROM_WriteDisable(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef EEPROM_ReadStatus(SPI_HandleTypeDef *hspi, uint8_t *status);
HAL_StatusTypeDef EEPROM_WriteStatus(SPI_HandleTypeDef *hspi, const uint8_t status);
HAL_StatusTypeDef EEPROM_ReadData(SPI_HandleTypeDef *hspi, const uint16_t address, uint8_t *data, const uint16_t size);
HAL_StatusTypeDef EEPROM_WritePage(SPI_HandleTypeDef *hspi, const uint16_t address, const uint8_t *data, const uint8_t size);
HAL_StatusTypeDef EEPROM_WriteDataMultiPage(SPI_HandleTypeDef *hspi, const uint16_t startAddress, const uint8_t *data, const uint16_t size);

// Typed write helpers — little-endian, no page-boundary restriction (single-type writes fit within one page)
HAL_StatusTypeDef EEPROM_Write_8(SPI_HandleTypeDef *hspi, const uint16_t address, int8_t value);
HAL_StatusTypeDef EEPROM_Write_16(SPI_HandleTypeDef *hspi, const uint16_t address, int16_t value);
HAL_StatusTypeDef EEPROM_Write_32(SPI_HandleTypeDef *hspi, const uint16_t address, int32_t value);
HAL_StatusTypeDef EEPROM_Write_64(SPI_HandleTypeDef *hspi, const uint16_t address, int64_t value);
HAL_StatusTypeDef EEPROM_Write_U8(SPI_HandleTypeDef *hspi, const uint16_t address, uint8_t value);
HAL_StatusTypeDef EEPROM_Write_U16(SPI_HandleTypeDef *hspi, const uint16_t address, uint16_t value);
HAL_StatusTypeDef EEPROM_Write_U32(SPI_HandleTypeDef *hspi, const uint16_t address, uint32_t value);
HAL_StatusTypeDef EEPROM_Write_U64(SPI_HandleTypeDef *hspi, const uint16_t address, uint64_t value);
HAL_StatusTypeDef EEPROM_Write_Float(SPI_HandleTypeDef *hspi, const uint16_t address, float value);
HAL_StatusTypeDef EEPROM_Write_Double(SPI_HandleTypeDef *hspi, const uint16_t address, double value);

// Typed read helpers — little-endian, float/double use union pun to avoid strict-aliasing UB
HAL_StatusTypeDef EEPROM_Read_8(SPI_HandleTypeDef *hspi, const uint16_t address, int8_t *value);
HAL_StatusTypeDef EEPROM_Read_16(SPI_HandleTypeDef *hspi, const uint16_t address, int16_t *value);
HAL_StatusTypeDef EEPROM_Read_32(SPI_HandleTypeDef *hspi, const uint16_t address, int32_t *value);
HAL_StatusTypeDef EEPROM_Read_64(SPI_HandleTypeDef *hspi, const uint16_t address, int64_t *value);
HAL_StatusTypeDef EEPROM_Read_U8(SPI_HandleTypeDef *hspi, const uint16_t address, uint8_t *value);
HAL_StatusTypeDef EEPROM_Read_U16(SPI_HandleTypeDef *hspi, const uint16_t address, uint16_t *value);
HAL_StatusTypeDef EEPROM_Read_U32(SPI_HandleTypeDef *hspi, const uint16_t address, uint32_t *value);
HAL_StatusTypeDef EEPROM_Read_U64(SPI_HandleTypeDef *hspi, const uint16_t address, uint64_t *value);
HAL_StatusTypeDef EEPROM_Read_Float(SPI_HandleTypeDef *hspi, const uint16_t address, float *value);
HAL_StatusTypeDef EEPROM_Read_Double(SPI_HandleTypeDef *hspi, const uint16_t address, double *value);

HAL_StatusTypeDef EEPROM_SelfTest(void);
#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_SPI_H */
