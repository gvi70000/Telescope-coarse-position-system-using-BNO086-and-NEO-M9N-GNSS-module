#include "BMP581.h"
#include "i2c.h"
#include "gpio.h"
#include "stm32f302xc.h"  // Device header for STM32F302 — pulls in core_cm4.h which provides __DSB()

// Global instance of BMP581 registers.
// 'volatile' is required because fields are written in main context and read in ISR context.
static volatile BMP581_registers_t bmp581_regs;

// Helper function to convert 3 bytes to a 32-bit integer
static inline int32_t get_24bit_signed(const uint8_t *data_ptr) {
    // Reconstruct the 24-bit value from 3 bytes (Little Endian: XLSB, LSB, MSB)
    uint32_t val = ((uint32_t)data_ptr[2] << 16) | ((uint32_t)data_ptr[1] << 8)  | (uint32_t)data_ptr[0];
    // Check if the sign bit (bit 23) is set
    if (val & 0x800000) {
        val |= 0xFF000000; // Sign extend to 32 bits by filling the top 8 bits with 1s
    }
    return (int32_t)val;
}

// Function to initialize the BMP581 sensor
/**
 * @brief BMP581_Init function implementation.
 * @details Detailed implementation for BMP581_Init.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Init(void) {
    // Step 1: Send reset command
    if (BMP581_SendCommand(BMP581_CMD_RESET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(10);  // 10ms delay after reset (datasheet minimum is 5ms; 10ms adds margin)

    // Step 2: Read and verify the CHIP ID
    uint8_t chip_id = BMP581_Get_CHIP_ID();
    if ((chip_id != BMP581_CHIP_ID_PRIM) && (chip_id != BMP581_CHIP_ID_SEC)) {
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

    // Step 6: Enable pressure and temperature compensation, and configure IIR output routing.
    // comp_pt_en MUST be set before the IIR filter coefficients take effect.
    bmp581_regs.DSP_CONFIG.Val.BitField.comp_pt_en         = BMP581_COMP_PRESS_TEMP_BOTH;  // Enable compensation for both P and T
    bmp581_regs.DSP_CONFIG.Val.BitField.iir_flush_forced_en = BMP581_IIR_FLUSH_ENABLED;    // Flush IIR on first measurement for clean startup
    bmp581_regs.DSP_CONFIG.Val.BitField.shdw_sel_iir_t     = BMP581_IIR_AFTER_FILTER;      // Output filtered temperature to data registers
    bmp581_regs.DSP_CONFIG.Val.BitField.shdw_sel_iir_p     = BMP581_IIR_AFTER_FILTER;      // Output filtered pressure to data registers
    if (BMP581_Set_DSPConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 7: Configure the IIR filter coefficients
    bmp581_regs.DSP_IIR_CONFIG.Val.BitField.set_iir_t = BMP581_IIR_COEFF_127;  // IIR filter for temperature
    bmp581_regs.DSP_IIR_CONFIG.Val.BitField.set_iir_p = BMP581_IIR_COEFF_127;  // IIR filter for pressure
    if (BMP581_Set_DSPIIRConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 8: Set the output data rate (ODR) and power mode
    bmp581_regs.ODR_CONFIG.Val.BitField.odr = BMP581_ODR_0_5_HZ;  // Set ODR to 0.5Hz (required for OSR 128x on both channels)
    bmp581_regs.ODR_CONFIG.Val.BitField.pwr_mode = BMP581_PWRMODE_NORMAL;  // Set to normal power mode
    bmp581_regs.ODR_CONFIG.Val.BitField.deep_dis = BMP581_DEEP_DISABLED;  // Disable deep standby
    if (BMP581_Set_ODRConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 9: Configure interrupts
    bmp581_regs.INT_CONFIG.Val.BitField.int_mode = BMP581_INT_MODE_PULSED;
    bmp581_regs.INT_CONFIG.Val.BitField.int_pol = BMP581_INT_POL_ACTIVE_HIGH;
    bmp581_regs.INT_CONFIG.Val.BitField.int_od = BMP581_INT_PIN_PUSH_PULL;
    bmp581_regs.INT_CONFIG.Val.BitField.int_en = BMP581_INT_ENABLE;
    if (BMP581_Set_INTConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 10: Enable data-ready interrupt
    bmp581_regs.INT_SOURCE.Val.BitField.drdy_data_reg_en = BMP581_DRDY_ENABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.fifo_full_en     = BMP581_FIFO_FULL_DISABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.fifo_ths_en      = BMP581_FIFO_THS_DISABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.oor_p_en         = BMP581_OOR_P_DISABLE;
    if (BMP581_Set_INTSource() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 11: Verify ODR/OSR combination is valid by reading back OSR_EFF register.
    // The STM32F302 Cortex-M4 write buffer can reorder writes; __DSB() ensures all
    // prior register writes have completed before we read back.
    __DSB();
    if (BMP581_Get_OSREff() != HAL_OK) {
        return HAL_ERROR;
    }
    if (bmp581_regs.OSR_EFF.Val.BitField.odr_is_valid != 1) {
        return HAL_ERROR;  // ODR/OSR combination rejected by sensor — adjust ODR or OSR
    }

    // Initialization complete
    return HAL_OK;
}

// Function to send commands to BMP581 sensor
/**
 * @brief BMP581_SendCommand function implementation.
 * @details Detailed implementation for BMP581_SendCommand.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_SendCommand(BMP581_cmd_t cmd) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_CMD, (uint8_t*)&cmd, ONE);
}

// REG 0x01 - CHIP ID
/**
 * @brief BMP581_Get_CHIP_ID function implementation.
 * @details Detailed implementation for BMP581_Get_CHIP_ID.
 * @param None
 * @return HAL status.
 */
uint8_t BMP581_Get_CHIP_ID(void) {
    uint8_t id = 0;
    if (ReadRegister(BMP581_ADDRESS, BMP581_REG_CHIP_ID, &id, ONE) == HAL_OK) {
        return id;
    }
    return 0xFF;  // Return invalid sentinel on I2C error (0xFF is not a valid chip ID)
}

// REG 0x02 - CHIP Revision ID
/**
 * @brief BMP581_Get_CHIP_REV function implementation.
 * @details Detailed implementation for BMP581_Get_CHIP_REV.
 * @param None
 * @return HAL status.
 */
uint8_t BMP581_Get_CHIP_REV(void) {
/**
 * @brief rev function implementation.
 * @details Detailed implementation for rev.
 * @param None
 * @return HAL status.
 */
    uint8_t rev = 0;
    if (ReadRegister(BMP581_ADDRESS, BMP581_REG_REV_ID, &rev, ONE) == HAL_OK) {
        return rev;
    }
    return 0;
}

// REG 0x11 - CHIP STATUS
/**
 * @brief BMP581_Get_CHIPStatus function implementation.
 * @details Detailed implementation for BMP581_Get_CHIPStatus.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_CHIPStatus(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_CHIP_STATUS, (uint8_t*)(void*)&bmp581_regs.CHIP_STATUS.Val, ONE);
}

// REG 0x13 - DRIVE CONFIG
/**
 * @brief BMP581_Set_DriveConfig function implementation.
 * @details Detailed implementation for BMP581_Set_DriveConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DriveConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_DRIVE_CONFIG, (uint8_t*)(void*)&bmp581_regs.DRIVE_CONFIG.Val, ONE);
}

/**
 * @brief BMP581_Get_DriveConfig function implementation.
 * @details Detailed implementation for BMP581_Get_DriveConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DriveConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_DRIVE_CONFIG, (uint8_t*)(void*)&bmp581_regs.DRIVE_CONFIG.Val, ONE);
}

// REG 0x14 - Interrupt Configuration
/**
 * @brief BMP581_Set_INTConfig function implementation.
 * @details Detailed implementation for BMP581_Set_INTConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_INTConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_INT_CONFIG, (uint8_t*)(void*)&bmp581_regs.INT_CONFIG.Val, ONE);
}

/**
 * @brief BMP581_Get_INTConfig function implementation.
 * @details Detailed implementation for BMP581_Get_INTConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_INT_CONFIG, (uint8_t*)(void*)&bmp581_regs.INT_CONFIG.Val, ONE);
}

// REG 0x15 - Interrupt Source Configuration
/**
 * @brief BMP581_Set_INTSource function implementation.
 * @details Detailed implementation for BMP581_Set_INTSource.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_INTSource(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_INT_SOURCE, (uint8_t*)(void*)&bmp581_regs.INT_SOURCE.Val, ONE);
}

/**
 * @brief BMP581_Get_INTSource function implementation.
 * @details Detailed implementation for BMP581_Get_INTSource.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTSource(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_INT_SOURCE, (uint8_t*)(void*)&bmp581_regs.INT_SOURCE.Val, ONE);
}

// REG 0x16 - FIFO Configuration
/**
 * @brief BMP581_Set_FIFOConfig function implementation.
 * @details Detailed implementation for BMP581_Set_FIFOConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_FIFOConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_FIFO_CONFIG, (uint8_t*)(void*)&bmp581_regs.FIFO_CONFIG.Val, ONE);
}

/**
 * @brief BMP581_Get_FIFOConfig function implementation.
 * @details Detailed implementation for BMP581_Get_FIFOConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_FIFO_CONFIG, (uint8_t*)(void*)&bmp581_regs.FIFO_CONFIG.Val, ONE);
}

// REG 0x17 - FIFO Count
/**
 * @brief BMP581_Get_FIFOCount function implementation.
 * @details Detailed implementation for BMP581_Get_FIFOCount.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOCount(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_FIFO_COUNT, (uint8_t*)(void*)&bmp581_regs.FIFO_COUNT.Val, ONE);
}

// REG 0x18 - FIFO Frame Selection
/**
 * @brief BMP581_Set_FIFOFrameSel function implementation.
 * @details Detailed implementation for BMP581_Set_FIFOFrameSel.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_FIFOFrameSel(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_FIFO_SEL, (uint8_t*)(void*)&bmp581_regs.FIFO_SEL.Val, ONE);
}

/**
 * @brief BMP581_Get_FIFOFrameSel function implementation.
 * @details Detailed implementation for BMP581_Get_FIFOFrameSel.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOFrameSel(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_FIFO_SEL, (uint8_t*)(void*)&bmp581_regs.FIFO_SEL.Val, ONE);
}

// REG 0x1D - 0x1F Temperature and 0x20 - 0x22 Pressure Data
/**
 * @brief BMP581_Get_TempPressData function implementation.
 * @details Detailed implementation for BMP581_Get_TempPressData.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_TempPressData(BMP581_sensor_data_t *data) {
    uint8_t reg_data[6];

    if (ReadRegister(BMP581_ADDRESS, BMP581_REG_TEMP_DATA_XLSB, reg_data, 6) != HAL_OK)
        return HAL_ERROR;

    int32_t t_raw = get_24bit_signed(&reg_data[0]);   // TEMP_XLSB..MSB
    int32_t p_raw = get_24bit_signed(&reg_data[3]);   // PRESS_XLSB..MSB

    // Datasheet: T = raw / 2^16, p = raw / 2^6
    data->temperature = (float)t_raw / TEMP_COEFF;
    data->pressure    = (float)p_raw / PRESS_COEFF;     // Pa

    return HAL_OK;
}

// REG 0x27 - Interrupt Status
/**
 * @brief BMP581_Get_INTStatus function implementation.
 * @details Detailed implementation for BMP581_Get_INTStatus.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTStatus(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_INT_STATUS, (uint8_t*)(void*)&bmp581_regs.INT_STATUS.Val, ONE);
}

// REG 0x28 - Status Register
/**
 * @brief BMP581_Get_Status function implementation.
 * @details Detailed implementation for BMP581_Get_Status.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_Status(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_STATUS, (uint8_t*)(void*)&bmp581_regs.STATUS.Val, ONE);
}

// REG 0x29 - FIFO Data
/**
 * @brief BMP581_Get_FIFOData function implementation.
 * @details Detailed implementation for BMP581_Get_FIFOData.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOData(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_FIFO_DATA, (uint8_t*)(void*)&bmp581_regs.FIFO_DATA, ONE);
}

// REG 0x2B - NVM Address
/**
 * @brief BMP581_Set_NVMAddr function implementation.
 * @details Detailed implementation for BMP581_Set_NVMAddr.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_NVMAddr(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_NVM_ADDR, (uint8_t*)(void*)&bmp581_regs.NVM_ADDR.Val, ONE);
}

/**
 * @brief BMP581_Get_NVMAddr function implementation.
 * @details Detailed implementation for BMP581_Get_NVMAddr.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_NVMAddr(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_NVM_ADDR, (uint8_t*)(void*)&bmp581_regs.NVM_ADDR.Val, ONE);
}

// REG 0x2C - 0x2D - NVM Data
/**
 * @brief BMP581_Set_NVMData function implementation.
 * @details Detailed implementation for BMP581_Set_NVMData.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_NVMData(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_NVM_DATA_LSB, (uint8_t*)(void*)&bmp581_regs.NVM_DATA, TWO);
}

/**
 * @brief BMP581_Get_NVMData function implementation.
 * @details Detailed implementation for BMP581_Get_NVMData.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_NVMData(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_NVM_DATA_LSB, (uint8_t*)(void*)&bmp581_regs.NVM_DATA, TWO);
}

// REG 0x30 - DSP Configuration
/**
 * @brief BMP581_Set_DSPConfig function implementation.
 * @details Detailed implementation for BMP581_Set_DSPConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DSPConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_DSP_CONFIG, (uint8_t*)(void*)&bmp581_regs.DSP_CONFIG.Val, ONE);
}

/**
 * @brief BMP581_Get_DSPConfig function implementation.
 * @details Detailed implementation for BMP581_Get_DSPConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DSPConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_DSP_CONFIG, (uint8_t*)(void*)&bmp581_regs.DSP_CONFIG.Val, ONE);
}

// REG 0x31 - DSP IIR Configuration
/**
 * @brief BMP581_Set_DSPIIRConfig function implementation.
 * @details Detailed implementation for BMP581_Set_DSPIIRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DSPIIRConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_DSP_IIR, (uint8_t*)(void*)&bmp581_regs.DSP_IIR_CONFIG.Val, ONE);
}

/**
 * @brief BMP581_Get_DSPIIRConfig function implementation.
 * @details Detailed implementation for BMP581_Get_DSPIIRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DSPIIRConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_DSP_IIR, (uint8_t*)(void*)&bmp581_regs.DSP_IIR_CONFIG.Val, ONE);
}

// REG 0x32 - 0x33 - OOR Pressure Threshold
/**
 * @brief BMP581_Set_OOR_PRESS_THR function implementation.
 * @details Detailed implementation for BMP581_Set_OOR_PRESS_THR.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OOR_PRESS_THR(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_OOR_THR_P_LSB, (uint8_t*)(void*)&bmp581_regs.OOR_PRESS_THR, TWO);
}

/**
 * @brief BMP581_Get_OOR_PRESS_THR function implementation.
 * @details Detailed implementation for BMP581_Get_OOR_PRESS_THR.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OOR_PRESS_THR(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_OOR_THR_P_LSB, (uint8_t*)(void*)&bmp581_regs.OOR_PRESS_THR, TWO);
}

// REG 0x34 - OOR Range
/**
 * @brief BMP581_Set_OORRange function implementation.
 * @details Detailed implementation for BMP581_Set_OORRange.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OORRange(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_OOR_RANGE, (uint8_t*)(void*)&bmp581_regs.OOR_PRESS_RNG, ONE);
}

/**
 * @brief BMP581_Get_OORRange function implementation.
 * @details Detailed implementation for BMP581_Get_OORRange.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OORRange(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_OOR_RANGE, (uint8_t*)(void*)&bmp581_regs.OOR_PRESS_RNG, ONE);
}

// REG 0x35 - OOR Configuration
/**
 * @brief BMP581_Set_OORConfig function implementation.
 * @details Detailed implementation for BMP581_Set_OORConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OORConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_OOR_CONFIG, (uint8_t*)(void*)&bmp581_regs.OOR_CONFIG.Val, ONE);
}

/**
 * @brief BMP581_Get_OORConfig function implementation.
 * @details Detailed implementation for BMP581_Get_OORConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OORConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_OOR_CONFIG, (uint8_t*)(void*)&bmp581_regs.OOR_CONFIG.Val, ONE);
}

// REG 0x36 - OSR Configuration
/**
 * @brief BMP581_Set_OSRConfig function implementation.
 * @details Detailed implementation for BMP581_Set_OSRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OSRConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_OSR_CONFIG, (uint8_t*)(void*)&bmp581_regs.OSR_CONFIG.Val, ONE);
}

/**
 * @brief BMP581_Get_OSRConfig function implementation.
 * @details Detailed implementation for BMP581_Get_OSRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OSRConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_OSR_CONFIG, (uint8_t*)(void*)&bmp581_regs.OSR_CONFIG.Val, ONE);
}

// REG 0x37 - ODR Configuration
/**
 * @brief BMP581_Set_ODRConfig function implementation.
 * @details Detailed implementation for BMP581_Set_ODRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_ODRConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_ODR_CONFIG, (uint8_t*)(void*)&bmp581_regs.ODR_CONFIG.Val, ONE);
}

/**
 * @brief BMP581_Get_ODRConfig function implementation.
 * @details Detailed implementation for BMP581_Get_ODRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_ODRConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_ODR_CONFIG, (uint8_t*)(void*)&bmp581_regs.ODR_CONFIG.Val, ONE);
}

// REG 0x38 - Effective OSR
/**
 * @brief BMP581_Get_OSREff function implementation.
 * @details Detailed implementation for BMP581_Get_OSREff.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OSREff(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_OSR_EFF, (uint8_t*)(void*)&bmp581_regs.OSR_EFF.Val, ONE);
}