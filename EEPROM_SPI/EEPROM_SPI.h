#ifndef __EEPROM_SPI_H
#define __EEPROM_SPI_H

#ifdef __cplusplus
extern "C" {
#endif
#include "spi.h"

#define PAGE_SIZE		128 //bytes
#define PAGES				512 // Change it to fit your chip

#define EEPROM_RELEASE    HAL_GPIO_WritePin(CS_MEM_GPIO_Port, CS_MEM_Pin, GPIO_PIN_SET)
#define EEPROM_SELECT     HAL_GPIO_WritePin(CS_MEM_GPIO_Port, CS_MEM_Pin, GPIO_PIN_RESET)
//#define EEPROM_SELECT	CS_MEM_GPIO_Port->BRR = (uint32_t)CS_MEM_Pin
//#define EEPROM_RELEASE CS_MEM_GPIO_Port->BSRR = (uint32_t)CS_MEM_Pin
	
#define EEPROM_TIMEOUT   100
#define ZERO   0
#define SIZE_1 1
#define SIZE_2 2
#define SIZE_3 3
#define SIZE_4 4
#define SIZE_8 8

// Instructions
#define WREN	0x06 // Write Enable 0000 0110
#define WRDI	0x04 // Write Disable 0000 0100
#define RDSR	0x05 // Read Status Register 0000 0101
#define WRSR	0x01 // Write Status Register 0000 0001
#define READ	0x03 // Read from Memory Array 0000 0011
#define WRITE	0x02 // Write to Memory Array 0000 0010
#define WIP   0x01 // Write In Progress (WIP) flag

// Enable write operation
HAL_StatusTypeDef EEPROM_WriteEnable(void);
// Disable write operation
HAL_StatusTypeDef EEPROM_WriteDisable(void);
// Read status register
uint8_t EEPROM_ReadStatus(void);
// Write status register
HAL_StatusTypeDef EEPROM_WriteStatus(const uint8_t status);
// Read an ammout of data from EEPROM
HAL_StatusTypeDef EEPROM_ReadData(const uint16_t address, uint8_t *data, const uint16_t size);
// Function to write a page of data to EEPROM at a specified address
HAL_StatusTypeDef EEPROM_WritePage(const uint16_t address, const uint8_t *data, const uint8_t size);
// Write any amount of data across several pages in an EEPROM
HAL_StatusTypeDef EEPROM_WriteDataMultiPage(const uint16_t startAddress, const uint8_t* data, const uint16_t size);

HAL_StatusTypeDef EEPROM_Write_8(const uint16_t address, int8_t value);
HAL_StatusTypeDef EEPROM_Write_16(const uint16_t address, int16_t value);
HAL_StatusTypeDef EEPROM_Write_32(const uint16_t address, int32_t value);
HAL_StatusTypeDef EEPROM_Write_64(const uint16_t address, int64_t value);
HAL_StatusTypeDef EEPROM_Write_U8(const uint16_t address, uint8_t value);
HAL_StatusTypeDef EEPROM_Write_U16(const uint16_t address, const uint16_t value);
HAL_StatusTypeDef EEPROM_Write_U32(const uint16_t address, const uint32_t value);
HAL_StatusTypeDef EEPROM_Write_Float(const uint16_t address, const float value);
HAL_StatusTypeDef EEPROM_Write_U64(const uint16_t address, const uint64_t value);
HAL_StatusTypeDef EEPROM_Write_Double(const uint16_t address, const double value);

HAL_StatusTypeDef EEPROM_Read_8(const uint16_t address, int8_t *value);
HAL_StatusTypeDef EEPROM_Read_16(const uint16_t address, int16_t *value);
HAL_StatusTypeDef EEPROM_Read_32(const uint16_t address, int32_t *value);
HAL_StatusTypeDef EEPROM_Read_Float(const uint16_t address, float *value);
HAL_StatusTypeDef EEPROM_Read_64(const uint16_t address, int64_t *value);
HAL_StatusTypeDef EEPROM_Read_Double(const uint16_t address, double *value);
HAL_StatusTypeDef EEPROM_Read_U8(const uint16_t address, uint8_t *value);
HAL_StatusTypeDef EEPROM_Read_U16(const uint16_t address, uint16_t *value);
HAL_StatusTypeDef EEPROM_Read_U32(const uint16_t address, uint32_t *value);
HAL_StatusTypeDef EEPROM_Read_U64(const uint16_t address, uint64_t *value);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_SPI_H */
