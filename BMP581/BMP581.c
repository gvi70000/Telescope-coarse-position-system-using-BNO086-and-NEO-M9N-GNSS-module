#include "BMP581.h"
#include "i2c.h"
#include "gpio.h"

// Global instance of BMP581 registers
static BMP581_registers_t bmp581_regs;
extern volatile uint8_t i2c1_transfer_complete;

// Helper function to convert 3 bytes to a 32-bit integer
// static inline int32_t get_32bit(const uint8_t *data_ptr) {
//    return (int32_t)(data_ptr[2] << 16 | data_ptr[1] << 8 | data_ptr[0]);
//}
static inline int32_t get_32bit(const uint8_t *data_ptr) {
    return (int32_t)(*(uint32_t*)data_ptr & 0x00FFFFFF);  // Masking to keep 3 bytes
}

// I2C write function
static HAL_StatusTypeDef write(const uint8_t regAddress, const uint8_t* dataBuffer, uint8_t numBytes) {
    #ifdef USE_I2C_DMA
        i2c1_transfer_complete = 0;
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(&hi2c1, BMP581_ADDRESS, regAddress, ONE, (uint8_t *)dataBuffer, numBytes);
        if (status != HAL_OK) {
            return status;
        }
        uint32_t timeout = HAL_GetTick();
        while (!i2c1_transfer_complete) {
            if ((HAL_GetTick() - timeout) > I2C_TIMEOUT) {
                return HAL_TIMEOUT;
            }
        }
        return HAL_OK;
    #else
        return HAL_I2C_Mem_Write(&hi2c1, BMP581_ADDRESS, regAddress, ONE, (uint8_t *)dataBuffer, numBytes, I2C_TIMEOUT);
    #endif
}

// I2C read function
static HAL_StatusTypeDef read(const uint8_t regAddress, uint8_t* dataBuffer, const uint8_t numBytes) {
    #ifdef USE_I2C_DMA
        i2c1_transfer_complete = 0;
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1, BMP581_ADDRESS, regAddress, ONE, dataBuffer, numBytes);
        if (status != HAL_OK) {
            return status;
        }
        uint32_t timeout = HAL_GetTick();
        while (!i2c1_transfer_complete) {
            if ((HAL_GetTick() - timeout) > I2C_TIMEOUT) {
                return HAL_TIMEOUT;
            }
        }
        return HAL_OK;
    #else
        return HAL_I2C_Mem_Read(&hi2c1, BMP581_ADDRESS, regAddress, ONE, dataBuffer, numBytes, I2C_TIMEOUT);
    #endif
}

// Function to initialize the BMP581 sensor
HAL_StatusTypeDef BMP581_Init(void) {
    // Step 1: Send reset command
    if (BMP581_SendCommand(BMP581_CMD_RESET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(5);  // Use minimal delay as per the datasheet.

    // Step 2: Read and verify the CHIP ID
    uint8_t chip_id = BMP581_Get_CHIP_ID();
    if (chip_id != BMP581_CHIP_ID_PRIM) {
        return HAL_ERROR;  // Invalid CHIP_ID
    }

    // Step 3: Read the STATUS register to check NVM readiness and errors
    if (BMP581_Get_Status() != HAL_OK) {
        return HAL_ERROR;  // Failed to read STATUS register
    }
    if (bmp581_regs.STATUS.Val.BitField.status_nvm_rdy != 1 || bmp581_regs.STATUS.Val.BitField.status_nvm_err != 0) {
        return HAL_ERROR;  // NVM not ready or there's an NVM error
    }

    // Step 4: Check Power-On Reset (POR) completion
    if (BMP581_Get_INTStatus() != HAL_OK) {
        return HAL_ERROR;  // Failed to read INT_STATUS register
    }
    if (bmp581_regs.INT_STATUS.Val.BitField.por != 1) {
        return HAL_ERROR;  // Power-on reset not complete
    }

    // Step 5: Configure the sensor settings for continuous high-accuracy mode
    bmp581_regs.OSR_CONFIG.Val.BitField.osr_t = BMP581_OSR_128X;  // Oversampling for temperature
    bmp581_regs.OSR_CONFIG.Val.BitField.osr_p = BMP581_OSR_128X;  // Oversampling for pressure
    bmp581_regs.OSR_CONFIG.Val.BitField.press_en = 1;  // Enable pressure measurement
    if (BMP581_Set_OSRConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 6: Configure the IIR filters
    bmp581_regs.DSP_IIR_CONFIG.Val.BitField.set_iir_t = BMP581_IIR_COEFF_127;  // IIR filter for temperature
    bmp581_regs.DSP_IIR_CONFIG.Val.BitField.set_iir_p = BMP581_IIR_COEFF_127;  // IIR filter for pressure
    if (BMP581_Set_DSPIIRConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 7: Set the output data rate (ODR) and power mode
    bmp581_regs.ODR_CONFIG.Val.BitField.odr = BMP581_ODR_5_HZ;  // Set ODR to 5Hz
    bmp581_regs.ODR_CONFIG.Val.BitField.pwr_mode = BMP581_PWRMODE_NORMAL;  // Set to normal power mode
    bmp581_regs.ODR_CONFIG.Val.BitField.deep_dis = BMP581_DEEP_DISABLED;  // Disable deep standby
    if (BMP581_Set_ODRConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 8: Configure interrupts
    bmp581_regs.INT_CONFIG.Val.BitField.int_mode = BMP581_INT_MODE_PULSED;
    bmp581_regs.INT_CONFIG.Val.BitField.int_pol = BMP581_INT_POL_ACTIVE_HIGH;
    bmp581_regs.INT_CONFIG.Val.BitField.int_od = BMP581_INT_PIN_PUSH_PULL;
    bmp581_regs.INT_CONFIG.Val.BitField.int_en = BMP581_INT_ENABLE;
    if (BMP581_Set_INTConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 9: Enable data-ready interrupt
    bmp581_regs.INT_SOURCE.Val.BitField.drdy_data_reg_en = BMP581_INT_ENABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.fifo_full_en = BMP581_INT_DISABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.fifo_ths_en = BMP581_INT_DISABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.oor_p_en = BMP581_INT_DISABLE;
    if (BMP581_Set_INTSource() != HAL_OK) {
        return HAL_ERROR;
    }

    // Initialization complete
    return HAL_OK;
}

// Function to send commands to BMP581 sensor
HAL_StatusTypeDef BMP581_SendCommand(BMP581_cmd_t cmd) {
    return write(BMP581_REG_CMD, &cmd, ONE);
}

// Register functions

// REG 0x01 - CHIP ID
uint8_t BMP581_Get_CHIP_ID(void) {
    uint8_t id = 0;
    if (read(BMP581_REG_CHIP_ID, &id, ONE) == HAL_OK) {
        return id;
    }
    return 0;
}

// REG 0x02 - CHIP Revision ID
uint8_t BMP581_Get_CHIP_REV(void) {
    uint8_t rev = 0;
    if (read(BMP581_REG_REV_ID, &rev, ONE) == HAL_OK) {
        return rev;
    }
    return 0;
}

// REG 0x11 - CHIP STATUS
HAL_StatusTypeDef BMP581_Get_CHIPStatus(void) {
    return read(BMP581_REG_CHIP_STATUS, (uint8_t *)&bmp581_regs.CHIP_STATUS.Val, ONE);
}

// REG 0x13 - DRIVE CONFIG
HAL_StatusTypeDef BMP581_Set_DriveConfig(void) {
    return write(BMP581_REG_DRIVE_CONFIG, (uint8_t *)&bmp581_regs.DRIVE_CONFIG.Val, ONE);
}

HAL_StatusTypeDef BMP581_Get_DriveConfig(void) {
    return read(BMP581_REG_DRIVE_CONFIG, (uint8_t *)&bmp581_regs.DRIVE_CONFIG.Val, ONE);
}

// REG 0x14 - Interrupt Configuration
HAL_StatusTypeDef BMP581_Set_INTConfig(void) {
    return write(BMP581_REG_INT_CONFIG, (uint8_t *)&bmp581_regs.INT_CONFIG.Val, ONE);
}

HAL_StatusTypeDef BMP581_Get_INTConfig(void) {
    return read(BMP581_REG_INT_CONFIG, (uint8_t *)&bmp581_regs.INT_CONFIG.Val, ONE);
}

// REG 0x15 - Interrupt Source Configuration
HAL_StatusTypeDef BMP581_Set_INTSource(void) {
    return write(BMP581_REG_INT_SOURCE, (uint8_t *)&bmp581_regs.INT_SOURCE.Val, ONE);
}

HAL_StatusTypeDef BMP581_Get_INTSource(void) {
    return read(BMP581_REG_INT_SOURCE, (uint8_t *)&bmp581_regs.INT_SOURCE.Val, ONE);
}

// REG 0x16 - FIFO Configuration
HAL_StatusTypeDef BMP581_Set_FIFOConfig(void) {
    return write(BMP581_REG_FIFO_CONFIG, (uint8_t *)&bmp581_regs.FIFO_CONFIG.Val, ONE);
}

HAL_StatusTypeDef BMP581_Get_FIFOConfig(void) {
    return read(BMP581_REG_FIFO_CONFIG, (uint8_t *)&bmp581_regs.FIFO_CONFIG.Val, ONE);
}

// REG 0x17 - FIFO Count
HAL_StatusTypeDef BMP581_Get_FIFOCount(void) {
    return read(BMP581_REG_FIFO_COUNT, (uint8_t *)&bmp581_regs.FIFO_COUNT.Val, ONE);
}

// REG 0x18 - FIFO Frame Selection
HAL_StatusTypeDef BMP581_Get_FIFOFrameSel(void) {
    return write(BMP581_REG_FIFO_SEL, (uint8_t *)&bmp581_regs.FIFO_SEL.Val, ONE);
}

HAL_StatusTypeDef BMP581_Set_FIFOFrameSel(void) {
    return read(BMP581_REG_FIFO_SEL, (uint8_t *)&bmp581_regs.FIFO_SEL.Val, ONE);
}

// REG 0x1D - 0x1F Temperature and 0x20 - 0x22 Pressure Data
HAL_StatusTypeDef BMP581_Get_TempPressData(BMP581_sensor_data_t *data) {
    uint8_t reg_data[SIX] __attribute__((aligned(4)));  // Ensure 32-bit alignment
    if (read(BMP581_REG_TEMP_DATA_XLSB, reg_data, SIX) == HAL_OK) {
        // Convert raw temperature data
        int32_t raw_data = get_32bit(&reg_data[0]);  // Aligned access
        data->temperature = (float)raw_data / TEMP_COEFF;

        // Convert raw pressure data
        raw_data = get_32bit(&reg_data[3]);  // Aligned access
        data->pressure = (float)raw_data / PRESS_COEFF;
        return HAL_OK;
    }
    return HAL_ERROR;
}

// REG 0x27 - Interrupt Status
HAL_StatusTypeDef BMP581_Get_INTStatus(void) {
    return read(BMP581_REG_INT_STATUS, (uint8_t *)&bmp581_regs.INT_STATUS.Val, ONE);
}

// REG 0x28 - Status Register
HAL_StatusTypeDef BMP581_Get_Status(void) {
    return read(BMP581_REG_STATUS, (uint8_t *)&bmp581_regs.STATUS.Val, ONE);
}

// REG 0x29 - FIFO Data
HAL_StatusTypeDef BMP581_Get_FIFOData(void) {
    return read(BMP581_REG_FIFO_DATA, (uint8_t *)&bmp581_regs.FIFO_DATA, ONE);
}

// REG 0x2B - NVM Address
HAL_StatusTypeDef BMP581_Set_NVMAddr(void) {
    return write(BMP581_REG_NVM_ADDR, (uint8_t *)&bmp581_regs.NVM_ADDR.Val, ONE);
}

HAL_StatusTypeDef BMP581_Get_NVMAddr(void) {
    return read(BMP581_REG_NVM_ADDR, (uint8_t *)&bmp581_regs.NVM_ADDR.Val, ONE);
}

// REG 0x2C - 0x2D - NVM Data
HAL_StatusTypeDef BMP581_Set_NVMData(void) {
    return write(BMP581_REG_NVM_DATA_LSB, (uint8_t *)&bmp581_regs.NVM_DATA, TWO);
}

HAL_StatusTypeDef BMP581_Get_NVMData(void) {
    return read(BMP581_REG_NVM_DATA_LSB, (uint8_t *)&bmp581_regs.NVM_DATA, TWO);
}

// REG 0x30 - DSP Configuration
HAL_StatusTypeDef BMP581_Set_DSPConfig(void) {
    return write(BMP581_REG_DSP_CONFIG, (uint8_t *)&bmp581_regs.DSP_CONFIG.Val, ONE);
}

HAL_StatusTypeDef BMP581_Get_DSPConfig(void) {
    return read(BMP581_REG_DSP_CONFIG, (uint8_t *)&bmp581_regs.DSP_CONFIG.Val, ONE);
}

// REG 0x31 - DSP IIR Configuration
HAL_StatusTypeDef BMP581_Set_DSPIIRConfig(void) {
    return write(BMP581_REG_DSP_IIR, (uint8_t *)&bmp581_regs.DSP_IIR_CONFIG.Val, ONE);
}

HAL_StatusTypeDef BMP581_Get_DSPIIRConfig(void) {
    return read(BMP581_REG_DSP_IIR, (uint8_t *)&bmp581_regs.DSP_IIR_CONFIG.Val, ONE);
}

// REG 0x32 - 0x33 - OOR Pressure Threshold
HAL_StatusTypeDef BMP581_Set_OOR_PRESS_THR(void) {
    return write(BMP581_REG_OOR_THR_P_LSB, (uint8_t *)&bmp581_regs.OOR_PRESS_THR, TWO);
}

HAL_StatusTypeDef BMP581_Get_OOR_PRESS_THR(void) {
    return read(BMP581_REG_OOR_THR_P_LSB, (uint8_t *)&bmp581_regs.OOR_PRESS_THR, TWO);
}

// REG 0x34 - OOR Range
HAL_StatusTypeDef BMP581_Set_OORRange(void) {
    return write(BMP581_REG_OOR_RANGE, (uint8_t *)&bmp581_regs.OOR_PRESS_RNG, ONE);
}

HAL_StatusTypeDef BMP581_Get_OORRange(void) {
    return read(BMP581_REG_OOR_RANGE, (uint8_t *)&bmp581_regs.OOR_PRESS_RNG, ONE);
}

// REG 0x35 - OOR Configuration
HAL_StatusTypeDef BMP581_Set_OORConfig(void) {
    return write(BMP581_REG_OOR_CONFIG, (uint8_t *)&bmp581_regs.OOR_CONFIG.Val, ONE);
}

HAL_StatusTypeDef BMP581_Get_OORConfig(void) {
    return read(BMP581_REG_OOR_CONFIG, (uint8_t *)&bmp581_regs.OOR_CONFIG.Val, ONE);
}

// REG 0x36 - OSR Configuration
HAL_StatusTypeDef BMP581_Set_OSRConfig(void) {
    return write(BMP581_REG_OSR_CONFIG, (uint8_t *)&bmp581_regs.OSR_CONFIG.Val, ONE);
}

HAL_StatusTypeDef BMP581_Get_OSRConfig(void) {
    return read(BMP581_REG_OSR_CONFIG, (uint8_t *)&bmp581_regs.OSR_CONFIG.Val, ONE);
}

// REG 0x37 - ODR Configuration
HAL_StatusTypeDef BMP581_Set_ODRConfig(void) {
    return write(BMP581_REG_ODR_CONFIG, (uint8_t *)&bmp581_regs.ODR_CONFIG.Val, ONE);
}

HAL_StatusTypeDef BMP581_Get_ODRConfig(void) {
    return read(BMP581_REG_ODR_CONFIG, (uint8_t *)&bmp581_regs.ODR_CONFIG.Val, ONE);
}

// REG 0x38 - Effective OSR
HAL_StatusTypeDef BMP581_Get_OSREff(void) {
    return read(BMP581_REG_OSR_EFF, (uint8_t *)&bmp581_regs.OSR_EFF.Val, ONE);
}
