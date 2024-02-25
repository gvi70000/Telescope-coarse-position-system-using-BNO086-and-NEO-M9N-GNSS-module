#include "BMP581.h"
#include "i2c.h"
#include "gpio.h"

// Place to store OSR/ODR config values
BMP581_osr_odr_press_config_t osrOdrConfig;

// Place to store FIFO config values
BMP581_fifo_t fifo;
BMP581_int_source_select_t interruptSource;
BMP581_iir_config_t iirConfig;
BMP581_osr_odr_eff_t eff;

static inline int32_t get_32bit(const uint8_t *data_ptr) {
    return ((int32_t)data_ptr[2] << 16) | ((int32_t)data_ptr[1] << 8) | data_ptr[0];
}

// I2C write function
static HAL_StatusTypeDef write(const uint8_t regAddress, const uint8_t* dataBuffer, uint8_t numBytes) {
	return HAL_I2C_Mem_Write(&hi2c1, BMP581_ADDRESS, regAddress, ONE, (uint8_t *)dataBuffer, numBytes, BMP581_RESPONCE_TIME);
}

// I2C read function
static HAL_StatusTypeDef read(const uint8_t regAddress, uint8_t* dataBuffer, const uint8_t numBytes) {
	return HAL_I2C_Mem_Read(&hi2c1, BMP581_ADDRESS, regAddress, ONE, dataBuffer, numBytes, BMP581_RESPONCE_TIME);
}

// Power-up check. The reset check is done in the init function
static HAL_StatusTypeDef power_up_check(void) {
	uint8_t nvm_status;
	if(read(BMP581_REG_STATUS, &nvm_status, ONE) == HAL_OK) {
		// Check if nvm_rdy status = 1 and nvm_err status = 0 to proceed
		if((nvm_status & BMP581_INT_NVM_RDY) && (!(nvm_status & BMP581_INT_NVM_ERR))) {
			return HAL_OK;
//			uint8_t por_status;
//			if(BMP581_getInterruptStatus(&por_status) == HAL_OK) {
//				// Check if por/soft-reset complete status = 1 to proceed
//        if(por_status & BMP581_INT_ASSERTED_POR_SOFTRESET_COMPLETE) {
//					return HAL_OK;
//				}
//			}
		}
	}
	return HAL_ERROR;
}

// Set power mode
static HAL_StatusTypeDef set_power_mode(const enum BMP581_powermode powermode) {
	uint8_t reg_data;
	if(read(BMP581_REG_ODR_CONFIG, &reg_data, ONE) == HAL_OK) {
		// Setting deep_dis = 1(BMP581_DEEP_DISABLED) disables the deep standby mode
		reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_DEEP_DISABLE, BMP581_DEEP_DISABLED);
		reg_data = BMP581_SET_BITS_POS_0(reg_data, BMP581_POWERMODE, powermode);
		return write(BMP581_REG_ODR_CONFIG, &reg_data, ONE);
	}
	return HAL_ERROR;
}

// Check deep standby mode
static HAL_StatusTypeDef check_deepstandby_mode(enum BMP581_powermode *powermode) {
	uint8_t fifo_frame_sel;
	BMP581_iir_config_t iir_cfg = {0};
	BMP581_osr_odr_press_config_t osr_odr_press_cfg = {0};
	HAL_StatusTypeDef result = read(BMP581_REG_FIFO_SEL, &fifo_frame_sel, ONE);
	fifo_frame_sel = BMP581_GET_BITS_POS_0(fifo_frame_sel, BMP581_FIFO_FRAME_SEL);
	if(result == HAL_OK) {
		result = BMP581_getOsrOdrPressConfig(&osr_odr_press_cfg);
		if(result == HAL_OK) {
			result = BMP581_getIirConfig(&iir_cfg);
		}
	}
	// As per datasheet odr should be less than 5Hz. But register value for 5Hz is less than 4Hz and so, thus in this below condition odr is checked whether greater than 5Hz macro.
	if((osr_odr_press_cfg.odr > BMP581_ODR_05_HZ) && (fifo_frame_sel == BMP581_DISABLE) && (iir_cfg.set_iir_t == BMP581_IIR_FILTER_BYPASS) && (iir_cfg.set_iir_p == BMP581_IIR_FILTER_BYPASS)) {
		*powermode = BMP581_POWERMODE_DEEP_STANDBY;
	}
	return result;
}

// Set deep standby mode
static HAL_StatusTypeDef set_deep_standby_mode(void) {
	uint8_t reg_data;
	HAL_StatusTypeDef result = read(BMP581_REG_ODR_CONFIG, &reg_data, ONE);
	if(result == HAL_OK) {
		// Setting deep_dis = 0(BMP581_DEEP_ENABLED) enables the deep standby mode
		reg_data = BMP581_SET_BIT_VAL_0(reg_data, BMP581_DEEP_DISABLE);
		// Set ODR less then 5Hz - ODR used is 1Hz
		reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_ODR, BMP581_ODR_01_HZ);
		// Write the value to the odr config register(0x37)
		result = write(BMP581_REG_ODR_CONFIG, &reg_data, ONE);
		if(result == HAL_OK) {
			result = read(BMP581_REG_DSP_IIR, &reg_data, ONE);
			if(result == HAL_OK) {
				// Set iir_t and iir_p as Bypass(0x00).
				// The register holds only iir_t and iir_p and the last 2 bits are reserved.
				// Thus using the macro BMP581_IIR_BYPASS(0xC0) the register value is set as zero.
				reg_data = reg_data & BMP581_IIR_BYPASS;
				// Write the value to the IIR register(0x31)
				result = write(BMP581_REG_DSP_IIR, &reg_data, ONE);
			}
		}
		if(result == HAL_OK) {
			result = read(BMP581_REG_FIFO_SEL, &reg_data, ONE);
			if(result == HAL_OK) {
				// Disable fifo frame selct
				reg_data = BMP581_SET_BIT_VAL_0(reg_data, BMP581_FIFO_FRAME_SEL);
				// Write the value to the fifo select register(0x18)
				result = write(BMP581_REG_FIFO_SEL, &reg_data, ONE);
			}
		}
	}
	return result;
}

// Set standby mode
static HAL_StatusTypeDef set_standby_mode(void) {
	enum BMP581_powermode pwrmode;
	HAL_StatusTypeDef result = BMP581_getPowerMode(&pwrmode);
	if(result == HAL_OK) {
		if(pwrmode == BMP581_POWERMODE_DEEP_STANDBY) {
			result = BMP581_setPowerMode(BMP581_POWERMODE_STANDBY);
		}
	}
	return result;
}

// This internal API is used to set the IIR for temperature and pressure.
static HAL_StatusTypeDef set_iir_config(const BMP581_iir_config_t *iir_cfg) {
	// Variable to set IIR config
	uint8_t reg_data[TWO];
	if(read(BMP581_REG_DSP_CONFIG, reg_data, TWO) == HAL_OK) {
		reg_data[ZERO] = BMP581_SET_BITSLICE(reg_data[ZERO], BMP581_SHDW_SET_IIR_TEMP, iir_cfg->shdw_set_iir_t);
		reg_data[ZERO] = BMP581_SET_BITSLICE(reg_data[ZERO], BMP581_SHDW_SET_IIR_PRESS, iir_cfg->shdw_set_iir_p);
		reg_data[ZERO] = BMP581_SET_BITSLICE(reg_data[ZERO], BMP581_IIR_FLUSH_FORCED_EN, iir_cfg->iir_flush_forced_en);
		reg_data[ONE] = iir_cfg->set_iir_t;
		reg_data[ONE] = BMP581_SET_BITSLICE(reg_data[ONE], BMP581_SET_IIR_PRESS, iir_cfg->set_iir_p);
		// Set IIR configuration
		return write(BMP581_REG_DSP_CONFIG, reg_data, TWO);
	}
	return HAL_ERROR;
}

static HAL_StatusTypeDef set_fifo_iir_config(const uint8_t set_fifo_iir_t, const uint8_t set_fifo_iir_p) {
	// Variable to store the function result
	HAL_StatusTypeDef result;
	// Variable to set fifo IIR config
	uint8_t reg_data = 0;
	// Variable to store existing powermode
	enum BMP581_powermode curr_pwrmode;
	result = BMP581_getPowerMode(&curr_pwrmode);
	if(result == HAL_OK) {
		// Fifo IIR configuration is writable only during STANDBY mode(as per datasheet)
		if(curr_pwrmode != BMP581_POWERMODE_STANDBY) {
			// If sensor is not in standby mode, set sensor in standby mode
			result = BMP581_setPowerMode(BMP581_POWERMODE_STANDBY);
		}
		if(result == HAL_OK) {
			// Get fifo IIR configuration
			result = read(BMP581_REG_DSP_CONFIG, &reg_data, ONE);
			if(result == HAL_OK) {
				reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_SET_FIFO_IIR_TEMP, set_fifo_iir_t);
				reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_SET_FIFO_IIR_PRESS, set_fifo_iir_p);
				// Set fifo IIR configuration
				result = write(BMP581_REG_DSP_CONFIG, &reg_data, ONE);
			}
		}
		// If previous mode is not standbymode return sensor to that previous mode after setting fifo iir configuration
		if(result == HAL_OK) {
			// Since fifo IIR works only in standby mode we are not re-writing to deepstandby mode as deep standby mode resets the fifo IIR settings to default
			if((curr_pwrmode != BMP581_POWERMODE_STANDBY) && (curr_pwrmode != BMP581_POWERMODE_DEEP_STANDBY)) {
				result = BMP581_setPowerMode(curr_pwrmode);
			}
		}
	}
	return result;
}

static HAL_StatusTypeDef set_fifo_threshold(uint8_t *reg_data, const BMP581_fifo_t *fifo) {
	if((fifo->frame_sel == BMP581_FIFO_TEMPERATURE_DATA) || (fifo->frame_sel == BMP581_FIFO_PRESSURE_DATA)) {
		// If Pressure or Temperature is selected, maximum 31 frames can be set as threshold
		if(fifo->threshold <= BMP581_FIFO_MAX_THRESHOLD_P_MODE) {
			reg_data[0] = BMP581_SET_BITS_POS_0(reg_data[0], BMP581_FIFO_THRESHOLD, fifo->threshold);
		} else {
			return HAL_ERROR;
		}
	} else if(fifo->frame_sel == BMP581_FIFO_PRESS_TEMP_DATA) {
		// If both pressure and temperature is selected, maximum 15 frames can be set as threshold
		if(fifo->threshold <= BMP581_FIFO_MAX_THRESHOLD_P_T_MODE)        {
			reg_data[0] = BMP581_SET_BITS_POS_0(reg_data[0], BMP581_FIFO_THRESHOLD, fifo->threshold);
		} else {
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

static HAL_StatusTypeDef unpack_sensor_data(BMP581_sensor_data_t *sensor_data, uint16_t *data_index, const BMP581_fifo_t *fifo) {
	int32_t raw_data_t = get_32bit(&fifo->data[*data_index]);
	uint32_t raw_data_p;
	switch (fifo->frame_sel) {
		case BMP581_FIFO_TEMPERATURE_DATA:
			// Check if fifo frame is empty using get_32bit function
			if(raw_data_t != CHECK_EMPTY){
			//if(!((fifo->data[*data_index] == BMP581_FIFO_EMPTY) && (fifo->data[*data_index + 1] == BMP581_FIFO_EMPTY) && (fifo->data[*data_index + 2] == BMP581_FIFO_EMPTY))) {
				//raw_data_t = (int32_t)((uint32_t)fifo->data[*data_index + 2] << 16 | (uint16_t)fifo->data[*data_index + 1] << 8 | fifo->data[*data_index]);
				// Division by 2^16(whose equivalent value is 65536) is performed to get temperature data in deg C
				sensor_data->temperature = ((float)raw_data_t / TEMP_COEFF);
				sensor_data->pressure = 0.0;
				*data_index += 3;
			} else {
				// Move the data index to the last byte to mark completion
				*data_index = fifo->length;
				return HAL_ERROR;
			}
		break;
		case BMP581_FIFO_PRESSURE_DATA:
			// Check if fifo frame is empty
			if(raw_data_t != CHECK_EMPTY){
			//if(!((fifo->data[*data_index] == BMP581_FIFO_EMPTY) && (fifo->data[*data_index + 1] == BMP581_FIFO_EMPTY) && (fifo->data[*data_index + 2] == BMP581_FIFO_EMPTY))) {
				// The value was calculated above as int32
				raw_data_p = (uint32_t)raw_data_t;//((uint32_t)fifo->data[(*data_index) + 2] << 16 | (uint16_t)fifo->data[(*data_index) + 1] << 8 | fifo->data[*data_index]);
				// Division by 2^6(whose equivalent value is 64) is performed to get pressure data in Pa
				sensor_data->pressure = ((float)raw_data_p / PRESS_COEFF);
				sensor_data->temperature = 0.0;
				*data_index += 3;
			} else {
				// Move the data index to the last byte to mark completion
				*data_index = fifo->length;
				return HAL_ERROR;
			}
		break;
		case BMP581_FIFO_PRESS_TEMP_DATA:
			raw_data_p = (uint32_t)get_32bit(&fifo->data[*data_index + 3]);
			if((raw_data_t != CHECK_EMPTY) && (raw_data_p != CHECK_EMPTY)){
			//if(!((fifo->data[*data_index] == BMP581_FIFO_EMPTY) && (fifo->data[*data_index + 1] == BMP581_FIFO_EMPTY) &&
				//(fifo->data[*data_index + 2] == BMP581_FIFO_EMPTY) && (fifo->data[*data_index + 3] == BMP581_FIFO_EMPTY) &&
				//(fifo->data[*data_index + 4] == BMP581_FIFO_EMPTY) && (fifo->data[*data_index + 5] == BMP581_FIFO_EMPTY))) {
				//raw_data_t = (int32_t)((uint32_t)fifo->data[*data_index + 2] << 16 | (uint16_t)fifo->data[*data_index + 1] << 8 | fifo->data[*data_index]);
				//raw_data_p = (uint32_t)((uint32_t)fifo->data[(*data_index) + 5] << 16 | (uint16_t)fifo->data[(*data_index) + 4] << 8 | fifo->data[*data_index + 3]);
				// Division by 2^16(whose equivalent value is 65536) is performed to get temperature data in deg C
				sensor_data->temperature = (float)(raw_data_t / 65536.0);
				// Division by 2^6(whose equivalent value is 64) is performed to get pressure data in Pa
				sensor_data->pressure = (float)(raw_data_p / 64.0);
				*data_index += 6;
			} else {
				// Move the data index to the last byte to mark completion
				*data_index = fifo->length;
				return HAL_ERROR;
			}
		break;
		default:
			sensor_data->pressure = 0.0;
			sensor_data->temperature = 0.0;
		break;
	}
	return HAL_OK;
}

static HAL_StatusTypeDef set_oor_iir_count_limit(const uint8_t set_oor_iir_p, const uint8_t set_count_limit) {
	HAL_StatusTypeDef result = HAL_OK;
	uint8_t reg_data = 0;
	//Variable to store existing powermode
	enum BMP581_powermode curr_pwrmode;
	result = BMP581_getPowerMode(&curr_pwrmode);
	if(result == HAL_OK) {
		//OOR IIR configuration and count limit is writable only during STANDBY mode(as per datasheet) */
		if(curr_pwrmode != BMP581_POWERMODE_STANDBY) {
			//If sensor is not in standby mode, set sensor in standby mode */
			result = BMP581_setPowerMode(BMP581_POWERMODE_STANDBY);
		}
		if(result == HAL_OK) {
			//Get OOR pressure IIR configuration
			result = read(BMP581_REG_DSP_CONFIG, &reg_data, ONE);
			if(result == HAL_OK) {
				reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_OOR_SEL_IIR_PRESS, set_oor_iir_p);
				//Set OOR pressure IIR configuration
				result = write(BMP581_REG_DSP_CONFIG, &reg_data, ONE);
			}
			if(result == HAL_OK) {
				//Get OOR pressure count limit
				result = read(BMP581_REG_OOR_CONFIG, &reg_data, ONE);
			}
			if(result == HAL_OK) {
				reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_OOR_COUNT_LIMIT, set_count_limit);
			}
			if(result == HAL_OK) {
				//Set OOR pressure count limit
				result = write(BMP581_REG_OOR_CONFIG, &reg_data, 1);
			}
		}
		//If previous mode is not standbymode return sensor to that previous mode after setting oor iir configuration and count limit
		if(result == HAL_OK) {
			//Since oor IIR and count limit works only in standby mode we are not re-writing to deepstandby mode as deep standby mode resets the oor iir configuration and count limit to default
			if((curr_pwrmode != BMP581_POWERMODE_STANDBY) && (curr_pwrmode != BMP581_POWERMODE_DEEP_STANDBY)) {
				result = BMP581_setPowerMode(curr_pwrmode);
			}
		}
	}
	return result;
}

static HAL_StatusTypeDef set_nvm_addr(const uint8_t nvm_addr, const uint8_t prog_en) {
	uint8_t nvm_status;
	// Check if NVM ready status = 1
	HAL_StatusTypeDef result = read(BMP581_REG_STATUS, &nvm_status, ONE);
	if(result == HAL_OK) {
		if(nvm_status & BMP581_INT_NVM_RDY) {
			uint8_t reg_data;
			result = read(BMP581_REG_NVM_ADDR, &reg_data, ONE);
			if(result == HAL_OK) {
				// Write the NVM address */
				reg_data = BMP581_SET_BITS_POS_0(reg_data, BMP581_NVM_ADDR, nvm_addr);
				if(prog_en == BMP581_ENABLE) {
				// Set bit nvm_prog_en as 1 if NVM write is selected
				reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_NVM_PROG_EN, prog_en);
				} else {
					// Set bit nvm_prog_en as 0 if NVM read is selected
					reg_data = BMP581_SET_BIT_VAL_0(reg_data, BMP581_NVM_PROG_EN);
				}
				result = write(BMP581_REG_NVM_ADDR, &reg_data, ONE);
			}
		}	else {
			return HAL_ERROR;
		}
	}
	return result;
}

static HAL_StatusTypeDef nvm_write_addr(const uint8_t nvm_addr, const uint8_t prog_en, enum BMP581_powermode *curr_pwrmode) {
	HAL_StatusTypeDef result = HAL_ERROR;
	if((nvm_addr >= BMP581_NVM_START_ADDR) && (nvm_addr <= BMP581_NVM_END_ADDR)) {
		// Get current powermode
		result = BMP581_getPowerMode(curr_pwrmode);
		if(result == HAL_OK) {
			// NVM write can be performed only during STANDBY mode
			if(*curr_pwrmode != BMP581_POWERMODE_STANDBY) {
				// If sensor is not in standby mode, set sensor in standby mode
				result = BMP581_setPowerMode(BMP581_POWERMODE_STANDBY);
			}
			if(result == HAL_OK) {
				// Write the NVM address and set NVM prog_en with respect to NVM read/write selected.
				result = set_nvm_addr(nvm_addr, prog_en);
			}
		}
	}
	return result;
}

static HAL_StatusTypeDef get_nvm_data(uint16_t *nvm_data) {
	uint8_t nvm_status;
	HAL_StatusTypeDef result = read(BMP581_REG_STATUS, &nvm_status, ONE);
	if(result == HAL_OK) {
		// Check if NVM ready status = 1, NVM error status = 0 and NVM command error status = 0
		if((nvm_status & BMP581_INT_NVM_RDY) && (!(nvm_status & BMP581_INT_NVM_ERR)) &&(!(nvm_status & BMP581_INT_NVM_CMD_ERR))) {
			// Read the NVM data from the register address
			uint8_t nvmdata[2];
			result = read(BMP581_REG_NVM_DATA_LSB, nvmdata, TWO);
			(*nvm_data) = (uint16_t)((nvmdata[1] << 8) | nvmdata[0]);
		} else {
			return HAL_ERROR;
		}
	}
	return result;
}

// This API is used to get interrupt status.
HAL_StatusTypeDef BMP581_getInterruptStatus(uint8_t *int_status) {
	return read(BMP581_REG_INT_STATUS, int_status, ONE);
}

HAL_StatusTypeDef BMP581_SoftReset(void) {
	uint8_t data = BMP581_SOFT_RESET_CMD;
	if(write(BMP581_REG_CMD, &data, ONE) == HAL_OK) {
		// Soft-reset execution takes 2 ms
		HAL_Delay(2);
		uint8_t por_status;
		if(BMP581_getInterruptStatus(&por_status) == HAL_OK) {
			if(por_status & BMP581_INT_ASSERTED_POR_SOFTRESET_COMPLETE) {
				return HAL_OK;
			}
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef BMP581_Init(void) {
	if(BMP581_SoftReset() == HAL_OK) {
		// Read chip_id 
		uint8_t chip_id;
		if(read(BMP581_REG_CHIP_ID, &chip_id, ONE) == HAL_OK) {
			if(chip_id != ZERO) {
				// Validate post power-up procedure
				if(power_up_check() == HAL_OK) {
					return (chip_id == BMP581_CHIP_ID_PRIM || chip_id == BMP581_CHIP_ID_SEC) ? HAL_OK : HAL_ERROR;
				}
			}
		}
	}
	return HAL_ERROR;
}

// Set BMP581 to get out precise data, no speed constraint
HAL_StatusTypeDef BMP581_setHighAccuracy(void) {
	HAL_StatusTypeDef result = BMP581_setPowerMode(BMP581_POWERMODE_STANDBY);
	if(BMP581_setPowerMode(BMP581_POWERMODE_STANDBY) == HAL_OK) {
		osrOdrConfig.odr = BMP581_ODR_05_HZ;
		osrOdrConfig.osr_p = BMP581_OVERSAMPLING_128X;
		osrOdrConfig.osr_t = BMP581_OVERSAMPLING_128X;
		osrOdrConfig.press_en = BMP581_ENABLE; // !!! Enable pressure
		if(BMP581_setOsrOdrPressConfig(&osrOdrConfig) == HAL_OK) {
			iirConfig.set_iir_t = BMP581_IIR_FILTER_COEFF_127;
			iirConfig.set_iir_p = BMP581_IIR_FILTER_COEFF_127;
			iirConfig.shdw_set_iir_p = BMP581_ENABLE;
			iirConfig.shdw_set_iir_t = BMP581_ENABLE;
			if(BMP581_setIirConfig(&iirConfig) == HAL_OK) {
				if(BMP581_ConfigureInterrupt(BMP581_PULSED, BMP581_ACTIVE_HIGH, BMP581_INTR_PUSH_PULL, BMP581_INTR_ENABLE) == HAL_OK) {						
					interruptSource.drdy_en = BMP581_ENABLE;				// Trigger interrupts when data is ready
					interruptSource.fifo_full_en = BMP581_DISABLE;	// Trigger interrupts when FIFO is full
					interruptSource.fifo_thres_en = BMP581_DISABLE;	// Trigger interrupts when FIFO threshold is reached
					interruptSource.oor_press_en = BMP581_DISABLE;	// Trigger interrupts when pressure goes out of range
					// Select INT_SOURCE after configuring interrupt
					if(BMP581_InterruptSourceSelect(&interruptSource) == HAL_OK) {
						result = BMP581_getOsrOdrEffective(&eff);
						if(result){}
						//Set powermode as continous
						return BMP581_setPowerMode(BMP581_POWERMODE_CONTINOUS);
					}
				}
			}
		}
	}
	return result;
}
// This API reads the temperature(deg C) or both pressure(Pa) and temperature(deg C) data from the sensor and store it in the BMP581_sensor_data structure instance passed by the user.
HAL_StatusTypeDef BMP581_getSensorData(BMP581_sensor_data_t *sensor_data) {
	uint8_t reg_data[SIX] = {0};
	if(read(BMP581_REG_TEMP_DATA_XLSB, reg_data, SIX) == HAL_OK) {
		int32_t raw_data_t = (int32_t) ((int32_t) ((uint32_t)(((uint32_t)reg_data[TWO] << 16) | ((uint16_t)reg_data[ONE] << 8) | reg_data[ZERO]) << 8) >> 8);
		// Division by 2^16(whose equivalent value is 65536) is performed to get temperature data in deg C
		sensor_data->temperature = ((float)raw_data_t / TEMP_COEFF);
		// ToDo Decide if I should keep this condition as pressuere will be enabled by default
//		if(osrOdrConfig.press_en) {
			uint32_t raw_data_p = (uint32_t)((uint32_t)(reg_data[FIVE] << 16) | (uint16_t)(reg_data[FOUR] << 8) | reg_data[THREE]);
			// Division by 2^6(whose equivalent value is 64) is performed to get pressure data in Pa
			sensor_data->pressure = ((float)raw_data_p / PRESS_COEFF);
//		} else {
//			sensor_data->pressure = 0.0;
//		}
		return HAL_OK;
	}
	return HAL_ERROR;
}

// This API is used to get powermode of the sensor.
HAL_StatusTypeDef BMP581_getPowerMode(enum BMP581_powermode *powermode) {
	uint8_t reg_data;
	// Read the power mode register
	if(read(BMP581_REG_ODR_CONFIG, &reg_data, ONE) == HAL_OK) {
		uint8_t pwrmode = BMP581_GET_BITS_POS_0(reg_data, BMP581_POWERMODE);
		uint8_t deep_dis;
		switch (pwrmode) {
			case BMP581_POWERMODE_STANDBY:
				// Getting deep disable status
				deep_dis = BMP581_GET_BITSLICE(reg_data, BMP581_DEEP_DISABLE);
				// Checking deepstandby status only when powermode is in standby mode
				// If deep_dis = 0(BMP581_DEEP_ENABLED) then deepstandby mode is enabled.
				// If deep_dis = 1(BMP581_DEEP_DISABLED) then deepstandby mode is disabled
				if(deep_dis == BMP581_DEEP_ENABLED) {
					return check_deepstandby_mode(powermode);
				} else {
					*powermode = BMP581_POWERMODE_STANDBY;
				}
			break;
			case BMP581_POWERMODE_NORMAL:
				*powermode = BMP581_POWERMODE_NORMAL;
			break;
			case BMP581_POWERMODE_FORCED:
				*powermode = BMP581_POWERMODE_FORCED;
			break;
			case BMP581_POWERMODE_CONTINOUS:
				*powermode = BMP581_POWERMODE_CONTINOUS;
			break;
		}
		return HAL_OK;
	}
	return HAL_ERROR;
}

// This API is used to set powermode of the sensor.
HAL_StatusTypeDef BMP581_setPowerMode(const enum BMP581_powermode powermode) {
	HAL_StatusTypeDef result = HAL_ERROR;
	enum BMP581_powermode lst_pwrmode;
	// Existing power mode of the device is received in lst_pwrmode
	result = BMP581_getPowerMode(&lst_pwrmode);
	if(result == HAL_OK) {
		// If the sensor is not in standby mode set the device to standby mode.
		if(lst_pwrmode != BMP581_POWERMODE_STANDBY) {
			// Device should be set to standby before transiting to forced mode or normal mode or continous mode.
			result = set_power_mode(BMP581_POWERMODE_STANDBY);
			if(result == HAL_OK) {
				// Give t_standby(as per data sheet) time for device to go into standby mode
				HAL_Delay(3);
			}
		}
		if(result == HAL_OK) {
			// Set the desired power mode
			switch (powermode) {
				case BMP581_POWERMODE_DEEP_STANDBY:
					result = set_deep_standby_mode();
				break;
				case BMP581_POWERMODE_STANDBY:
					// Since switching between powermodes require sensor to be in standby mode it is performed above. So it is not explicitly performed here.
				break;
				case BMP581_POWERMODE_NORMAL:
				case BMP581_POWERMODE_FORCED:
				case BMP581_POWERMODE_CONTINOUS:
					result = set_power_mode(powermode);
				break;
        default:
					result = HAL_ERROR;
        break;
			}
		}
	}
	return result;
}

HAL_StatusTypeDef BMP581_getIirConfig(BMP581_iir_config_t *iir_cfg) {
	// Variable to get IIR config
	uint8_t reg_data[TWO];
	// Get IIR configuration
	if(read(BMP581_REG_DSP_CONFIG, reg_data, TWO) == HAL_OK) {
		iir_cfg->shdw_set_iir_t = BMP581_GET_BITSLICE(reg_data[ZERO], BMP581_SHDW_SET_IIR_TEMP);
		iir_cfg->shdw_set_iir_p = BMP581_GET_BITSLICE(reg_data[ZERO], BMP581_SHDW_SET_IIR_PRESS);
		iir_cfg->iir_flush_forced_en = BMP581_GET_BITSLICE(reg_data[ZERO], BMP581_IIR_FLUSH_FORCED_EN);
		iir_cfg->set_iir_t = BMP581_GET_BITS_POS_0(reg_data[ONE], BMP581_SET_IIR_TEMP);
		iir_cfg->set_iir_p = BMP581_GET_BITSLICE(reg_data[ONE], BMP581_SET_IIR_PRESS);
		return HAL_OK;
	}
	return HAL_ERROR;
}

// This API sets the configuration for IIR of temperature and pressure.
// If IIR value for both temperature and pressure is set a value other than bypass then powermode is set
//  as standby mode, as IIR with value other than bypass without disabling deep-standby mode makes powermode invalid.
HAL_StatusTypeDef BMP581_setIirConfig(const BMP581_iir_config_t *iir_cfg) {
	HAL_StatusTypeDef result = HAL_OK;
	// Variable to store existing powermode
	enum BMP581_powermode curr_pwrmode;
	// If IIR value for both temperature and pressure is set a value other than bypass then powermode is set
	// as standby mode, as IIR with value other than bypass without disabling deep-standby mode makes powermode
	// invalid.
	if((iir_cfg->set_iir_t != BMP581_IIR_FILTER_BYPASS) || (iir_cfg->set_iir_p != BMP581_IIR_FILTER_BYPASS)) {
		result = set_standby_mode();
	}
	if(result == HAL_OK) {
		result = BMP581_getPowerMode(&curr_pwrmode);
		if(result == HAL_OK) {
			// IIR configuration is writable only during STANDBY mode(as per datasheet)
			if(curr_pwrmode != BMP581_POWERMODE_STANDBY) {
				result = BMP581_setPowerMode(BMP581_POWERMODE_STANDBY);
			}
			// If sensor is not in standby mode, set sensor in standby mode
			if(result == HAL_OK) {
				result = set_iir_config(iir_cfg);
			}
			// If previous mode is not standbymode return sensor to that previous mode after setting iir configuration
			if(result == HAL_OK) {
				// Since IIR works only in standby mode we are not re-writing to deepstandby mode	as deep standby mode resets the IIR settings to default
				if((curr_pwrmode != BMP581_POWERMODE_STANDBY) && (curr_pwrmode != BMP581_POWERMODE_DEEP_STANDBY)) {
					result = BMP581_setPowerMode(curr_pwrmode);
				}
			}
		}
	}
	return result;
}

HAL_StatusTypeDef BMP581_getOsrOdrPressConfig(BMP581_osr_odr_press_config_t *osr_odr_press_cfg) {
	// Variable to store OSR and ODR config
	uint8_t reg_data[TWO];
	// Get OSR and ODR configuration in burst read
	if(read(BMP581_REG_OSR_CONFIG, reg_data, TWO) == HAL_OK) {
		osr_odr_press_cfg->osr_t = BMP581_GET_BITS_POS_0(reg_data[ZERO], BMP581_TEMP_OS);
		osr_odr_press_cfg->osr_p = BMP581_GET_BITSLICE(reg_data[ZERO], BMP581_PRESS_OS);
		osr_odr_press_cfg->press_en = BMP581_GET_BITSLICE(reg_data[ZERO], BMP581_PRESS_EN);
		osr_odr_press_cfg->odr = BMP581_GET_BITSLICE(reg_data[ONE], BMP581_ODR);
		return HAL_OK;
	}
	return HAL_ERROR;
}


// This API sets the configuration for oversampling temperature, oversampling of
//  pressure and ODR configuration along with pressure enable.
// If ODR is set to a value higher than 5Hz then powermode is set as standby mode, as ODR value greater than 5HZ
//  without disabling deep-standby mode makes powermode invalid.
HAL_StatusTypeDef BMP581_setOsrOdrPressConfig(const BMP581_osr_odr_press_config_t *osr_odr_press_cfg) {
	HAL_StatusTypeDef result = HAL_OK;
	// Variable to set ODR and OSR config
	uint8_t reg_data[TWO] = {0};
	// If ODR is set to a value higher than 5Hz then powermode is set as standby mode, as ODR value greater than 5HZ
	// without disabling deep-standby mode makes powermode invalid.
	// NOTE: Register value for 5Hz is greater compared to ODRs higher than it. Thus in this below condition odr is checked whether less than 5Hz macro.
	if(osr_odr_press_cfg->odr < BMP581_ODR_05_HZ) {
		result = set_standby_mode();
	}
	if(result == HAL_OK) {
		if(read(BMP581_REG_OSR_CONFIG, reg_data, TWO) == HAL_OK) {
			reg_data[ZERO] = BMP581_SET_BITS_POS_0(reg_data[ZERO], BMP581_TEMP_OS, osr_odr_press_cfg->osr_t);
			reg_data[ZERO] = BMP581_SET_BITSLICE(reg_data[ZERO], BMP581_PRESS_OS, osr_odr_press_cfg->osr_p);
			reg_data[ZERO] = BMP581_SET_BITSLICE(reg_data[ZERO], BMP581_PRESS_EN, osr_odr_press_cfg->press_en);
			reg_data[ONE] = BMP581_SET_BITSLICE(reg_data[ONE], BMP581_ODR, osr_odr_press_cfg->odr);
			// Set ODR and OSR configuration
			result = write(BMP581_REG_OSR_CONFIG, reg_data, TWO);
		}
	}
	return result;
}

// This API is used to enable the interrupts(drdy interrupt, fifo full interrupt, fifo threshold enable and pressure data out of range interrupt).
HAL_StatusTypeDef BMP581_InterruptSourceSelect(const BMP581_int_source_select_t *int_source_select) {
	uint8_t reg_data;
	if(read(BMP581_REG_INT_SOURCE, &reg_data, ONE) == HAL_OK)        {
		reg_data = BMP581_SET_BITS_POS_0(reg_data, BMP581_INT_DRDY_EN, int_source_select->drdy_en);
		reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_INT_FIFO_FULL_EN, int_source_select->fifo_full_en);
		reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_INT_FIFO_THRES_EN, int_source_select->fifo_thres_en);
		reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_INT_OOR_PRESS_EN, int_source_select->oor_press_en);
		return write(BMP581_REG_INT_SOURCE, &reg_data, ONE);
	}
	return HAL_ERROR;
}

// This API is used to configure the interrupt settings.
HAL_StatusTypeDef BMP581_ConfigureInterrupt(const enum BMP581_intr_mode int_mode, const enum BMP581_intr_polarity int_pol, const enum BMP581_intr_drive int_od, const enum BMP581_intr_en_dis int_en) {
	// Variable to get interrupt configuration
	uint8_t reg_data = 0;
	// Get interrupt configuration
	if(read(BMP581_REG_INT_CONFIG, &reg_data, ONE) == HAL_OK)    {
		// Any change between latched/pulsed mode has to be applied while interrupt is disabled
		// Step 1 : Turn off all INT sources (INT_SOURCE -> 0x00)
		uint8_t int_source = 0;
		if(write(BMP581_REG_INT_SOURCE, &int_source, ONE) == HAL_OK) {
			// Step 2 : Read the INT_STATUS register to clear the status
			uint8_t int_status = 0;
			if(read(BMP581_REG_INT_STATUS, &int_status, ONE) == HAL_OK) {
				// Step 3 : Set the desired mode in INT_CONFIG.int_mode
				reg_data = BMP581_SET_BITS_POS_0(reg_data, BMP581_INT_MODE, int_mode);
				reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_INT_POL, int_pol);
				reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_INT_OD, int_od);
				reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_INT_EN, int_en);
				// Finally transfer the interrupt configurations
				return write(BMP581_REG_INT_CONFIG, &reg_data, ONE);
			}
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef BMP581_getOsrOdrEffective(BMP581_osr_odr_eff_t *osr_odr_eff) {
	//Variable to store effective OSR
	uint8_t reg_data;
	//Get effective OSR configuration and ODR valid status
	HAL_StatusTypeDef result = read(BMP581_REG_OSR_EFF, &reg_data, ONE);
	if(result == HAL_OK) {
		osr_odr_eff->osr_t_eff = BMP581_GET_BITS_POS_0(reg_data, BMP581_OSR_TEMP_EFF);
		osr_odr_eff->osr_p_eff = BMP581_GET_BITSLICE(reg_data, BMP581_OSR_PRESS_EFF);
		osr_odr_eff->odr_is_valid = BMP581_GET_BITSLICE(reg_data, BMP581_ODR_IS_VALID);
	}
	return result;
}

HAL_StatusTypeDef BMP581_getFifoConfiguration(BMP581_fifo_t *fifo) {
	uint8_t reg_data;
	//Get the fifo congifurations
	HAL_StatusTypeDef result = read(BMP581_REG_FIFO_CONFIG, &reg_data, ONE);
	if(result == HAL_OK) {
		fifo->threshold = BMP581_GET_BITS_POS_0(reg_data, BMP581_FIFO_THRESHOLD);
		fifo->mode = BMP581_GET_BITSLICE(reg_data, BMP581_FIFO_MODE);
		result = read(BMP581_REG_FIFO_SEL, &reg_data, ONE);
		if(result == HAL_OK) {
			fifo->frame_sel = BMP581_GET_BITS_POS_0(reg_data, BMP581_FIFO_FRAME_SEL);
			fifo->dec_sel = BMP581_GET_BITSLICE(reg_data, BMP581_FIFO_DEC_SEL);
		}
	}
	if(result == HAL_OK) {
		result = read(BMP581_REG_DSP_CONFIG, &reg_data, ONE);
		if(result == HAL_OK) {
			fifo->set_fifo_iir_t = BMP581_GET_BITSLICE(reg_data, BMP581_SET_FIFO_IIR_TEMP);
			fifo->set_fifo_iir_p = BMP581_GET_BITSLICE(reg_data, BMP581_SET_FIFO_IIR_PRESS);
		}
	}
	return result;
}

HAL_StatusTypeDef BMP581_set_FifoConfiguration(const BMP581_fifo_t *fifo) {
	HAL_StatusTypeDef result = HAL_OK;
	//If Fifo frame selection is enabled then powermode is set as standby mode, as fifo frame selection enabled without disabling deep-standby mode makes powermode invalid.
	if(fifo->frame_sel != BMP581_DISABLE) {
		result = set_standby_mode();
	}
	if(result == HAL_OK) {
		uint8_t set_fifo_iir_t, set_fifo_iit_p;
		set_fifo_iir_t = fifo->set_fifo_iir_t;
		set_fifo_iit_p = fifo->set_fifo_iir_p;
		result = set_fifo_iir_config(set_fifo_iir_t, set_fifo_iit_p);
		if(result == HAL_OK) {
			uint8_t reg_data;
			//Get the fifo congifurations
			result = read(BMP581_REG_FIFO_CONFIG, &reg_data, ONE);
			if(result == HAL_OK) {
				reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_FIFO_MODE, fifo->mode);
				result = set_fifo_threshold(&reg_data, fifo);
			}
			if(result == HAL_OK) {
				//Set the fifo congifurations
				result = write(BMP581_REG_FIFO_CONFIG, &reg_data, ONE);
			}
			if(result == HAL_OK) {
				result = read(BMP581_REG_FIFO_SEL, &reg_data, ONE);
			}
			if(result == HAL_OK) {
				reg_data = BMP581_SET_BITS_POS_0(reg_data, BMP581_FIFO_FRAME_SEL, fifo->frame_sel);
				reg_data = BMP581_SET_BITSLICE(reg_data, BMP581_FIFO_DEC_SEL, fifo->dec_sel);
			}
			if(result == HAL_OK) {
				//Set the fifo congifurations
				result = write(BMP581_REG_FIFO_SEL, &reg_data, ONE);
			}
		}
	}
	return result;
}

HAL_StatusTypeDef BMP581_getFifoLength(uint16_t *fifo_len, BMP581_fifo_t *fifo) {
	uint8_t reg_data;
	//Get the fifo frame count
	HAL_StatusTypeDef result = read(BMP581_REG_FIFO_COUNT, &reg_data, ONE);
	if(result == HAL_OK) {
		fifo->fifo_count = BMP581_GET_BITS_POS_0(reg_data, BMP581_FIFO_COUNT);
		if((fifo->frame_sel == BMP581_FIFO_TEMPERATURE_DATA) || (fifo->frame_sel == BMP581_FIFO_PRESSURE_DATA)) {
			//Maximum of 32 frames if either one of temperature or pressure is selected and each frame comprises of 3 bytes of data
			*fifo_len = fifo->fifo_count * 3;
		}else if(fifo->frame_sel == BMP581_FIFO_PRESS_TEMP_DATA) {
			//Maximum of 16 frames if both temperature and pressure is selected and each frame comprises of 6 bytes of data
			*fifo_len = fifo->fifo_count * 6;
		}
	}
	return result;
}

HAL_StatusTypeDef BMP581_get_FifoData(BMP581_fifo_t *fifo) {
	uint16_t fifo_len;
	//Get the fifo length
	HAL_StatusTypeDef result = BMP581_getFifoLength(&fifo_len, fifo);
	if(result == HAL_OK) {
		if(fifo->length > fifo_len) {
			fifo->length = fifo_len;
		}
		//Read the fifo data
		result = read(BMP581_REG_FIFO_DATA, fifo->data, fifo->length);
	}
	return result;
}

HAL_StatusTypeDef BMP581_ExtractFifoData(const BMP581_fifo_t *fifo, BMP581_sensor_data_t *sensor_data) {
	HAL_StatusTypeDef result = HAL_OK;
	uint8_t idx = 0;
	uint16_t data_indx;
	for (data_indx = 0; (data_indx < fifo->length) && (result != HAL_OK);) {
		//Inside unpack_sensor_data function, data_index is incremented
		result = unpack_sensor_data(&sensor_data[idx], &data_indx, fifo);
		idx++;
	}
	return result;
}

HAL_StatusTypeDef BMP581_getOorConfiguration(BMP581_oor_press_configuration_t *oor_press_config) {
	uint8_t reg_data[4];
	HAL_StatusTypeDef result = read(BMP581_REG_DSP_CONFIG, &reg_data[ZERO], ONE);
	if(result == HAL_OK) {
		oor_press_config->oor_sel_iir_p = BMP581_GET_BITSLICE(reg_data[ZERO], BMP581_OOR_SEL_IIR_PRESS);
		result = read(BMP581_REG_OOR_THR_P_LSB, reg_data, FOUR);
		if(result == HAL_OK) {
			oor_press_config->oor_thr_p = reg_data[ZERO] | (reg_data[ONE] << 8) | ((reg_data[THREE] & BMP581_OOR_THR_P_XMSB_REG_MSK) << 16);
			oor_press_config->oor_range_p = reg_data[TWO];
			oor_press_config->cnt_lim = BMP581_GET_BITSLICE(reg_data[3], BMP581_OOR_COUNT_LIMIT);
		}
	}
	return result;
}

HAL_StatusTypeDef BMP581_set_OorConfiguration(const BMP581_oor_press_configuration_t *oor_press_config) {
	uint8_t set_oor_iir_p, set_count_limit;
	HAL_StatusTypeDef result = set_oor_iir_count_limit(set_oor_iir_p, set_count_limit);
	if(result == HAL_OK) {
		uint8_t reg_data[4];
		//Get the OOR configurations
		result = read(BMP581_REG_OOR_THR_P_LSB, reg_data, FOUR);
		if(result == HAL_OK) {
			uint8_t thres_xmsb;
			set_oor_iir_p = oor_press_config->oor_sel_iir_p;
			set_count_limit = oor_press_config->cnt_lim;
			reg_data[ZERO] = BMP581_SET_BITS_POS_0(reg_data[ZERO], BMP581_OOR_THR_P_LSB, oor_press_config->oor_thr_p);
			reg_data[ONE] = (uint8_t)(BMP581_SET_BITS_POS_0(reg_data[ONE], BMP581_OOR_THR_P_MSB, oor_press_config->oor_thr_p) >> 8);
			reg_data[TWO] = oor_press_config->oor_range_p;
			thres_xmsb = BMP581_GET_BITSLICE(oor_press_config->oor_thr_p, BMP581_OOR_THR_P_XMSB);
			reg_data[3] = BMP581_SET_BITS_POS_0(reg_data[3], BMP581_OOR_THR_P_XMSB_REG, thres_xmsb);
			//Set the OOR congifurations
			result = write(BMP581_REG_OOR_THR_P_LSB, reg_data, FOUR);
		}
	}
	return result;
}

HAL_StatusTypeDef BMP581_NvmRead(const uint8_t nvm_addr, uint16_t *nvm_data) {
	uint8_t reg_data;
	//Variable to store existing powermode
	enum BMP581_powermode curr_pwrmode;
	//Proceed if null check is fine
	HAL_StatusTypeDef result = nvm_write_addr(nvm_addr, BMP581_DISABLE, &curr_pwrmode);
	if(result == HAL_OK) {
		//First NVM command for user read sequence
		reg_data = BMP581_NVM_FIRST_CMND;
		result = write(BMP581_REG_CMD, &reg_data, ONE);
	}

	if(result == HAL_OK) {
		//Read enable NVM command for user read sequence
		reg_data = BMP581_NVM_READ_ENABLE_CMND;
		result = write(BMP581_REG_CMD, &reg_data, ONE);
		//Delay required for NVM ready status to change to 1 before NVM read
		HAL_Delay(1); // 800us
	}
	if(result == HAL_OK) {
		result = get_nvm_data(nvm_data);
	}
	if(result == HAL_OK) {
		//If previous mode is not standbymode return sensor to that previous mode after performing NVM
		if(curr_pwrmode != BMP581_POWERMODE_STANDBY) {
			result = BMP581_setPowerMode(curr_pwrmode);
		}
	}
	return result;
}

HAL_StatusTypeDef BMP581_NvmWrite(const uint8_t nvm_addr, const uint16_t *nvm_data) {
	uint8_t nvm_status = 0;
	uint8_t reg_data = 0;
	uint8_t nvmdata[2];
	//Variable to store existing powermode
	enum BMP581_powermode curr_pwrmode;
	HAL_StatusTypeDef result = nvm_write_addr(nvm_addr, BMP581_ENABLE, &curr_pwrmode);
	if(result == HAL_OK) {
		//Write data to be written to NVM address
		result = read(BMP581_REG_NVM_DATA_LSB, nvmdata, TWO);
		if(result == HAL_OK) {
			nvmdata[0] = (uint8_t)(*nvm_data & BMP581_NVM_DATA_LSB_MSK);
			nvmdata[1] = (uint8_t)((*nvm_data & BMP581_NVM_DATA_MSB_MSK) >> 8);
			result = write(BMP581_REG_NVM_DATA_LSB, nvmdata, 2);
		}
	}
	if(result == HAL_OK) {
		//First NVM command for user write sequence
		reg_data = BMP581_NVM_FIRST_CMND;
		result = write(BMP581_REG_CMD, &reg_data, ONE);
	}
	if(result == HAL_OK) {
		//Write enable NVM command for user write sequence
		reg_data = BMP581_NVM_WRITE_ENABLE_CMND;
		result = write(BMP581_REG_CMD, &reg_data, ONE);
		//Delay required for NVM ready status to change to 1
		HAL_Delay(10);
	}
	if(result == HAL_OK) {
		//Check if NVM ready status = 1, NVM error status = 0 and NVM command error status = 0
		result = read(BMP581_REG_STATUS, &nvm_status, ONE);
		if((nvm_status & BMP581_INT_NVM_RDY) && (!(nvm_status & BMP581_INT_NVM_ERR)) &&(!(nvm_status & BMP581_INT_NVM_CMD_ERR))) {
			//Reset NVM prog_en
			reg_data = BMP581_SET_BIT_VAL_0(reg_data, BMP581_NVM_PROG_EN);
			result = write(BMP581_REG_NVM_ADDR, &reg_data, ONE);
		} else {
		result = HAL_ERROR;
		}
	}
	if(result == HAL_OK) {
		//If previous mode is not standbymode return sensor to that previous mode after performing NVM
		if(curr_pwrmode != BMP581_POWERMODE_STANDBY) {
			result = BMP581_setPowerMode(curr_pwrmode);
		}
	}
	return result;
}
