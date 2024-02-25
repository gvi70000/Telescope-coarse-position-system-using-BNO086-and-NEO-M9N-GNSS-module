#ifndef __BMP581_H
#define __BMP581_H

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

// ((int32_t)0X7F << 16) | ((int32_t)0X7F << 8) | 0X7F;
#define CHECK_EMPTY 8355711

#define BMP581_GET_BITSLICE(regvar, bitname)						((regvar & bitname##_MSK) >> bitname##_POS)
#define BMP581_SET_BITSLICE(regvar, bitname, val)				((regvar & ~bitname##_MSK) | ((val << bitname##_POS) & bitname##_MSK))

#define BMP581_GET_LSB(var)															(uint8_t)(var & BMP581_SET_LOW_BYTE)
#define BMP581_GET_MSB(var)															(uint8_t)((var & BMP581_SET_HIGH_BYTE) >> 8)

#define BMP581_SET_BIT_VAL_0(reg_data, bitname)					(reg_data & ~(bitname##_MSK))
#define BMP581_SET_BITS_POS_0(reg_data, bitname, data)	((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))
#define BMP581_GET_BITS_POS_0(reg_data, bitname)				(reg_data & (bitname##_MSK))

// Define constants
#define ZERO	0
#define ONE		1
#define TWO		2
#define THREE	3
#define FOUR	4
#define FIVE	5
#define SIX		6
#define TEMP_COEFF		65536.0f
#define PRESS_COEFF		64.0f

#define BMP581_RESPONCE_TIME											100
#define BMP581_ADDRESS														0x8C//0x46

// Chip id of BMP5
#define BMP581_CHIP_ID_PRIM                         0x50
#define BMP581_CHIP_ID_SEC                          0x51

#define BMP581_ENABLE                               1
#define BMP581_DISABLE                              0

// Register addresses
#define BMP581_REG_CHIP_ID                          0x01
#define BMP581_REG_REV_ID                           0x02
#define BMP581_REG_CHIP_STATUS                      0x11
#define BMP581_REG_DRIVE_CONFIG                     0x13
#define BMP581_REG_INT_CONFIG                       0x14
#define BMP581_REG_INT_SOURCE                       0x15
#define BMP581_REG_FIFO_CONFIG                      0x16
#define BMP581_REG_FIFO_COUNT                       0x17
#define BMP581_REG_FIFO_SEL                         0x18
#define BMP581_REG_TEMP_DATA_XLSB                   0x1D
#define BMP581_REG_TEMP_DATA_LSB                    0x1E
#define BMP581_REG_TEMP_DATA_MSB                    0x1F
#define BMP581_REG_PRESS_DATA_XLSB                  0x20
#define BMP581_REG_PRESS_DATA_LSB                   0x21
#define BMP581_REG_PRESS_DATA_MSB                   0x22
#define BMP581_REG_INT_STATUS                       0x27
#define BMP581_REG_STATUS                           0x28
#define BMP581_REG_FIFO_DATA                        0x29
#define BMP581_REG_NVM_ADDR                         0x2B
#define BMP581_REG_NVM_DATA_LSB                     0x2C
#define BMP581_REG_NVM_DATA_MSB                     0x2D
#define BMP581_REG_DSP_CONFIG                       0x30
#define BMP581_REG_DSP_IIR                          0x31
#define BMP581_REG_OOR_THR_P_LSB                    0x32
#define BMP581_REG_OOR_THR_P_MSB                    0x33
#define BMP581_REG_OOR_RANGE                        0x34
#define BMP581_REG_OOR_CONFIG                       0x35
#define BMP581_REG_OSR_CONFIG                       0x36
#define BMP581_REG_ODR_CONFIG                       0x37
#define BMP581_REG_OSR_EFF                          0x38
#define BMP581_REG_CMD                              0x7E

// I2C addresses
#define BMP581_I2C_ADDR_PRIM                        0x46
#define BMP581_I2C_ADDR_SEC                         0x47

// NVM addresses
#define BMP581_NVM_START_ADDR                       0x20
#define BMP581_NVM_END_ADDR                         0x22

// Interface settings
#define BMP581_SPI_RD_MASK                          0x80

// Delay definition
#define BMP581_DELAY_US_SOFT_RESET                  2000
#define BMP581_DELAY_US_STANDBY                     2500
#define BMP581_DELAY_US_NVM_READY_READ              800
#define BMP581_DELAY_US_NVM_READY_WRITE             10000

// Soft reset command
#define BMP581_SOFT_RESET_CMD                       0xB6

// NVM command
#define BMP581_NVM_FIRST_CMND                       0x5D
#define BMP581_NVM_READ_ENABLE_CMND                 0xA5
#define BMP581_NVM_WRITE_ENABLE_CMND                0xA0

// Deepstandby enable/disable
#define BMP581_DEEP_ENABLED                         0
#define BMP581_DEEP_DISABLED                        1

// ODR settings
#define BMP581_ODR_240_HZ                           0x00
#define BMP581_ODR_218_5_HZ                         0x01
#define BMP581_ODR_199_1_HZ                         0x02
#define BMP581_ODR_179_2_HZ                         0x03
#define BMP581_ODR_160_HZ                           0x04
#define BMP581_ODR_149_3_HZ                         0x05
#define BMP581_ODR_140_HZ                           0x06
#define BMP581_ODR_129_8_HZ                         0x07
#define BMP581_ODR_120_HZ                           0x08
#define BMP581_ODR_110_1_HZ                         0x09
#define BMP581_ODR_100_2_HZ                         0x0A
#define BMP581_ODR_89_6_HZ                          0x0B
#define BMP581_ODR_80_HZ                            0x0C
#define BMP581_ODR_70_HZ                            0x0D
#define BMP581_ODR_60_HZ                            0x0E
#define BMP581_ODR_50_HZ                            0x0F
#define BMP581_ODR_45_HZ                            0x10
#define BMP581_ODR_40_HZ                            0x11
#define BMP581_ODR_35_HZ                            0x12
#define BMP581_ODR_30_HZ                            0x13
#define BMP581_ODR_25_HZ                            0x14
#define BMP581_ODR_20_HZ                            0x15
#define BMP581_ODR_15_HZ                            0x16
#define BMP581_ODR_10_HZ                            0x17
#define BMP581_ODR_05_HZ                            0x18
#define BMP581_ODR_04_HZ                            0x19
#define BMP581_ODR_03_HZ                            0x1A
#define BMP581_ODR_02_HZ                            0x1B
#define BMP581_ODR_01_HZ                            0x1C
#define BMP581_ODR_0_5_HZ                           0x1D
#define BMP581_ODR_0_250_HZ                         0x1E
#define BMP581_ODR_0_125_HZ                         0x1F

// Oversampling for temperature and pressure
#define BMP581_OVERSAMPLING_1X                      0x00
#define BMP581_OVERSAMPLING_2X                      0x01
#define BMP581_OVERSAMPLING_4X                      0x02
#define BMP581_OVERSAMPLING_8X                      0x03
#define BMP581_OVERSAMPLING_16X                     0x04
#define BMP581_OVERSAMPLING_32X                     0x05
#define BMP581_OVERSAMPLING_64X                     0x06
#define BMP581_OVERSAMPLING_128X                    0x07

// IIR filter for temperature and pressure
#define BMP581_IIR_FILTER_BYPASS                    0x00
#define BMP581_IIR_FILTER_COEFF_1                   0x01
#define BMP581_IIR_FILTER_COEFF_3                   0x02
#define BMP581_IIR_FILTER_COEFF_7                   0x03
#define BMP581_IIR_FILTER_COEFF_15                  0x04
#define BMP581_IIR_FILTER_COEFF_31                  0x05
#define BMP581_IIR_FILTER_COEFF_63                  0x06
#define BMP581_IIR_FILTER_COEFF_127                 0x07

// Fifo frame configuration
#define BMP581_FIFO_EMPTY                           0X7F
#define BMP581_FIFO_MAX_THRESHOLD_P_T_MODE          0x0F
#define BMP581_FIFO_MAX_THRESHOLD_P_MODE            0x1F

// Macro is used to bypass both iir_t and iir_p together
#define BMP581_IIR_BYPASS                           0xC0

// Pressure Out-of-range count limit
#define BMP581_OOR_COUNT_LIMIT_1                    0x00
#define BMP581_OOR_COUNT_LIMIT_3                    0x01
#define BMP581_OOR_COUNT_LIMIT_7                    0x02
#define BMP581_OOR_COUNT_LIMIT_15                   0x03

// Interrupt configurations
#define BMP581_INT_MODE_PULSED                      0
#define BMP581_INT_MODE_LATCHED                     1

#define BMP581_INT_POL_ACTIVE_LOW                   0
#define BMP581_INT_POL_ACTIVE_HIGH                  1

#define BMP581_INT_OD_PUSHPULL                      0
#define BMP581_INT_OD_OPENDRAIN                     1

// NVM and Interrupt status asserted macros
#define BMP581_INT_ASSERTED_DRDY                    0x01
#define BMP581_INT_ASSERTED_FIFO_FULL               0x02
#define BMP581_INT_ASSERTED_FIFO_THRES              0x04
#define BMP581_INT_ASSERTED_PRESSURE_OOR            0x08
#define BMP581_INT_ASSERTED_POR_SOFTRESET_COMPLETE  0x10
#define BMP581_INT_NVM_RDY                          0x02
#define BMP581_INT_NVM_ERR                          0x04
#define BMP581_INT_NVM_CMD_ERR                      0x08

// Interrupt configurations
#define BMP581_INT_MODE_MSK                         0x01

#define BMP581_INT_POL_MSK                          0x02
#define BMP581_INT_POL_POS                          1

#define BMP581_INT_OD_MSK                           0x04
#define BMP581_INT_OD_POS                           2

#define BMP581_INT_EN_MSK                           0x08
#define BMP581_INT_EN_POS                           3

#define BMP581_INT_DRDY_EN_MSK                      0x01

#define BMP581_INT_FIFO_FULL_EN_MSK                 0x02
#define BMP581_INT_FIFO_FULL_EN_POS                 1

#define BMP581_INT_FIFO_THRES_EN_MSK                0x04
#define BMP581_INT_FIFO_THRES_EN_POS                2

#define BMP581_INT_OOR_PRESS_EN_MSK                 0x08
#define BMP581_INT_OOR_PRESS_EN_POS                 3

// ODR configuration
#define BMP581_ODR_MSK                              0x7C
#define BMP581_ODR_POS                              2

// OSR configurations
#define BMP581_TEMP_OS_MSK                          0x07

#define BMP581_PRESS_OS_MSK                         0x38
#define BMP581_PRESS_OS_POS                         3

// Pressure enable
#define BMP581_PRESS_EN_MSK                         0x40
#define BMP581_PRESS_EN_POS                         6

// IIR configurations
#define BMP581_SET_IIR_TEMP_MSK                     0x07

#define BMP581_SET_IIR_PRESS_MSK                    0x38
#define BMP581_SET_IIR_PRESS_POS                    3

#define BMP581_OOR_SEL_IIR_PRESS_MSK                0x80
#define BMP581_OOR_SEL_IIR_PRESS_POS                7

#define BMP581_SHDW_SET_IIR_TEMP_MSK                0x08
#define BMP581_SHDW_SET_IIR_TEMP_POS                3

#define BMP581_SHDW_SET_IIR_PRESS_MSK               0x20
#define BMP581_SHDW_SET_IIR_PRESS_POS               5

#define BMP581_SET_FIFO_IIR_TEMP_MSK                0x10
#define BMP581_SET_FIFO_IIR_TEMP_POS                4

#define BMP581_SET_FIFO_IIR_PRESS_MSK               0x40
#define BMP581_SET_FIFO_IIR_PRESS_POS               6

#define BMP581_IIR_FLUSH_FORCED_EN_MSK              0x04
#define BMP581_IIR_FLUSH_FORCED_EN_POS              2

// Effective OSR configurations and ODR valid status
#define BMP581_OSR_TEMP_EFF_MSK                     0x07

#define BMP581_OSR_PRESS_EFF_MSK                    0x38
#define BMP581_OSR_PRESS_EFF_POS                    3

#define BMP581_ODR_IS_VALID_MSK                     0x80
#define BMP581_ODR_IS_VALID_POS                     7

// Powermode
#define BMP581_POWERMODE_MSK                        0x03

#define BMP581_DEEP_DISABLE_MSK                     0x80
#define BMP581_DEEP_DISABLE_POS                     7

// Fifo configurations
#define BMP581_FIFO_THRESHOLD_MSK                   0x1F

#define BMP581_FIFO_MODE_MSK                        0x20
#define BMP581_FIFO_MODE_POS                        5

#define BMP581_FIFO_DEC_SEL_MSK                     0x1C
#define BMP581_FIFO_DEC_SEL_POS                     2

#define BMP581_FIFO_COUNT_MSK                       0x3F

#define BMP581_FIFO_FRAME_SEL_MSK                   0x03

// Out-of-range configuration
#define BMP581_OOR_THR_P_LSB_MSK                    0x0000FF

#define BMP581_OOR_THR_P_MSB_MSK                    0x00FF00

#define BMP581_OOR_THR_P_XMSB_MSK                   0x010000
#define BMP581_OOR_THR_P_XMSB_POS                   16

// Macro to mask xmsb value of oor threshold from register(0x35) value
#define BMP581_OOR_THR_P_XMSB_REG_MSK               0x01

#define BMP581_OOR_COUNT_LIMIT_MSK                  0xC0
#define BMP581_OOR_COUNT_LIMIT_POS                  6

// NVM configuration
#define BMP581_NVM_ADDR_MSK                         0x3F

#define BMP581_NVM_PROG_EN_MSK                      0x40
#define BMP581_NVM_PROG_EN_POS                      6

#define BMP581_NVM_DATA_LSB_MSK                     0x00FF

#define BMP581_NVM_DATA_MSB_MSK                     0xFF00

// Enumerator for interrupt enable disable
enum BMP581_intr_en_dis {
    // Interrupt diable
    BMP581_INTR_DISABLE = BMP581_DISABLE,
    // Interrupt enable
    BMP581_INTR_ENABLE = BMP581_ENABLE
};

// Enumerator for interrupt mode
enum BMP581_intr_mode {
    // Interrupt mode - pulsed
    BMP581_PULSED = BMP581_INT_MODE_PULSED,
    // Interrupt mode - latched
    BMP581_LATCHED = BMP581_INT_MODE_LATCHED
};

// Enumerator for interrupt polarity
enum BMP581_intr_polarity {
    // Interrupt polarity - active low
    BMP581_ACTIVE_LOW = BMP581_INT_POL_ACTIVE_LOW,
    // Interrupt polarity - active high
    BMP581_ACTIVE_HIGH = BMP581_INT_POL_ACTIVE_HIGH
};

// Enumerator for interrupt drive
enum BMP581_intr_drive {
    // Interrupt drive - push-pull
    BMP581_INTR_PUSH_PULL = BMP581_INT_OD_PUSHPULL,
    // Interrupt drive - open drain
    BMP581_INTR_OPEN_DRAIN = BMP581_INT_OD_OPENDRAIN
};

enum BMP581_fifo_frame_sel {
    // Fifo disabled 
    BMP581_FIFO_NOT_ENABLED,
    // Fifo temperature data only enabled
    BMP581_FIFO_TEMPERATURE_DATA,
    // Fifo pressure data only enabled
    BMP581_FIFO_PRESSURE_DATA,
    // Fifo pressure and temperature data enabled
    BMP581_FIFO_PRESS_TEMP_DATA
};

// Enumerator for fifo decimation factor(downsampling) selection
enum BMP581_fifo_dec_sel {
    BMP581_FIFO_NO_DOWNSAMPLING, //Fifo downsampling disabled
    BMP581_FIFO_DOWNSAMPLING_2X,
    BMP581_FIFO_DOWNSAMPLING_4X,
    BMP581_FIFO_DOWNSAMPLING_8X,
    BMP581_FIFO_DOWNSAMPLING_16X,
    BMP581_FIFO_DOWNSAMPLING_32X,
    BMP581_FIFO_DOWNSAMPLING_64X,
    BMP581_FIFO_DOWNSAMPLING_128X
};

// Enumerator for fifo mode selection
enum BMP581_fifo_mode {
    BMP581_FIFO_MODE_STREAMING,
    BMP581_FIFO_MODE_STOP_ON_FULL
};

// Enumerator for powermode selection
enum BMP581_powermode {
    // Standby powermode
    BMP581_POWERMODE_STANDBY,
    // Normal powermode
    BMP581_POWERMODE_NORMAL,
    // Forced powermode
    BMP581_POWERMODE_FORCED,
    // Continous powermode
    BMP581_POWERMODE_CONTINOUS,
    // Deep standby powermode
    BMP581_POWERMODE_DEEP_STANDBY
};

typedef struct BMP581_osr_odr_eff {
	// Effective temperature OSR
	uint8_t osr_t_eff;
	// Effective pressure OSR
	uint8_t osr_p_eff;
	// If asserted, the ODR parametrization is valid
	uint8_t odr_is_valid;
} BMP581_osr_odr_eff_t;

typedef struct BMP581_oor_press_configuration {
	uint32_t oor_thr_p;
	uint8_t oor_range_p;
	uint8_t cnt_lim;
	uint8_t oor_sel_iir_p;
} BMP581_oor_press_configuration_t;

// SR, ODR and pressure configuration structure
typedef struct BMP581_osr_odr_press_config {
    uint8_t osr_t;		// BMP581_OVERSAMPLING_1X, 2, 4, 8, 16, 32, 64, 128X
    uint8_t osr_p;		// BMP581_OVERSAMPLING_1X, 2, 4, 8, 16, 32, 64, 128X
    uint8_t press_en; // BMP581_ENABLE, BMP581_ENABLE
    uint8_t odr;			// Output Data Rate
} BMP581_osr_odr_press_config_t;

typedef struct BMP581_fifo {
    uint8_t *data;					// Pointer to fifo data
    uint16_t length;				// Length of user defined bytes of fifo to be read
    uint8_t frame_sel;			// BMP581_FIFO_NOT_ENABLED, BMP581_FIFO_TEMPERATURE_DATA, BMP581_FIFO_PRESSURE_DATA, BMP581_FIFO_PRESS_TEMP_DATA
    uint8_t dec_sel;				// BMP581_FIFO_NO_DOWNSAMPLING, 2X, 4X, 8X, 16X, 32X, 64X, 128X
    uint8_t fifo_count;			// Fifo frame count
    uint8_t mode;						// BMP581_FIFO_MODE_STREAMING, BMP581_FIFO_MODE_STOP_ON_FULL
    uint8_t threshold;			// Threshold for fifo
    uint8_t set_fifo_iir_t; //Fifo temperature IIR BMP581_ENABLE, BMP581_DISABLE
    uint8_t set_fifo_iir_p; // Fifo pressure IIR BMP581_ENABLE, BMP581_DISABLE
} BMP581_fifo_t;

// IIR configuration structure
typedef struct BMP581_iir_config {
    uint8_t set_iir_t;						// IIR configuration structure BMP581_IIR_FILTER_BYPASS...BMP581_IIR_FILTER_0_00220
    uint8_t set_iir_p;						// Pressure IIR BMP581_IIR_FILTER_BYPASS...BMP581_IIR_FILTER_0_00220
    uint8_t shdw_set_iir_t;				// Shadow IIR selection for temperature BMP581_ENABLE, BMP581_DISABLE
    uint8_t shdw_set_iir_p;				// Shadow IIR selection for pressure BMP581_ENABLE, BMP581_DISABLE
    uint8_t iir_flush_forced_en;	// IIR flush in forced mode enable BMP581_ENABLE, BMP581_DISABLE
}BMP581_iir_config_t;

// BMP5 interrupt source selection.
typedef struct BMP581_int_source_select {
	uint8_t drdy_en;				// Data ready interrupt enable
	uint8_t fifo_full_en;		// Fifo full interrupt enable
	uint8_t fifo_thres_en;	// Fifo threshold interrupt enable
	uint8_t oor_press_en;		// Pressure out of range interrupt enable
}BMP581_int_source_select_t;

// BMP5 sensor data structure which comprises of temperature and pressure.
// BMP5 sensor data structure which comprises of temperature and pressure in floating point with data type as float for pressure and temperature.
typedef	struct BMP581_sensor_data {
	//Pressure data
	float pressure;
	//Temperature data
	float temperature;
} BMP581_sensor_data_t;

// Used to get the interrupt status (data ready interrupt, fifo full interrupt, fifo threshold interrupt, pressure out of range interrupt and power-on reset/software reset complete interrupt).
HAL_StatusTypeDef BMP581_getInterruptStatus(uint8_t *int_status);
// Used to soft-reset the sensor where all the registers are reset to their default values.
HAL_StatusTypeDef BMP581_SoftReset(void);
// Call this API before using all other APIs
HAL_StatusTypeDef BMP581_Init(void);

HAL_StatusTypeDef BMP581_setHighAccuracy(void);
// Reads the temperature(deg C) and pressure(Pa) data from the sensor and store it in the BMP581_sensor_data_t structure instance passed by the user.
HAL_StatusTypeDef BMP581_getSensorData(BMP581_sensor_data_t *sensor_data);

// Gets the power mode of the sensor
HAL_StatusTypeDef BMP581_getPowerMode(enum BMP581_powermode *powermode);
// Sets the power mode of the sensor
HAL_StatusTypeDef BMP581_setPowerMode(enum BMP581_powermode powermode);

// Gets the configuration for IIR of temperature and pressure
HAL_StatusTypeDef BMP581_getIirConfig(BMP581_iir_config_t *iir_cfg);
// Sets the configuration for IIR of temperature and pressure
HAL_StatusTypeDef BMP581_setIirConfig(const BMP581_iir_config_t *iir_cfg);

// Gets the configuration for oversampling of temperature, oversampling of pressure and ODR configuration along with pressure enable.
HAL_StatusTypeDef BMP581_getOsrOdrPressConfig(BMP581_osr_odr_press_config_t *osr_odr_press_cfg);
// Sets the configuration for oversampling of temperature, oversampling of pressure and ODR configuration along with pressure enable.
HAL_StatusTypeDef BMP581_setOsrOdrPressConfig(const BMP581_osr_odr_press_config_t *osr_odr_press_cfg);

// Used to enable the interrupts(drdy interrupt, fifo full interrupt, fifo threshold enable and pressure data out of range interrupt).
HAL_StatusTypeDef BMP581_InterruptSourceSelect(const BMP581_int_source_select_t *int_source_select);
// Used to configure the interrupt settings
HAL_StatusTypeDef BMP581_ConfigureInterrupt(enum BMP581_intr_mode int_mode, enum BMP581_intr_polarity int_pol, enum BMP581_intr_drive int_od, enum BMP581_intr_en_dis int_en);

// Gets the configuration for effective OSR of temperature, effective OSR of pressure and ODR valid status
HAL_StatusTypeDef BMP581_getOsrOdrEffective(BMP581_osr_odr_eff_t *osr_odr_eff);

// Used to get the configurations of fifo in the sensor
HAL_StatusTypeDef BMP581_getFifoConfiguration(BMP581_fifo_t *fifo);
// Used to set the configurations of fifo in the sensor
HAL_StatusTypeDef BMP581_set_FifoConfiguration(const BMP581_fifo_t *fifo);
// Used to get the length of fifo from the sensor
HAL_StatusTypeDef BMP581_getFifoLength(uint16_t *fifo_len, BMP581_fifo_t *fifo);
// Used to get the fifo data from the sensor
HAL_StatusTypeDef BMP581_get_FifoData(BMP581_fifo_t *fifo);
// Extract the temperature and/or pressure data from the fifo data which is already read from the fifo.
HAL_StatusTypeDef BMP581_ExtractFifoData(const BMP581_fifo_t *fifo, BMP581_sensor_data_t *sensor_data);

// Gets the configuration for out-of-range pressure threshold, range count limit and IIR
HAL_StatusTypeDef BMP581_getOorConfiguration(BMP581_oor_press_configuration_t *oor_press_config);
// Sets the configuration for out-of-range pressure threshold, range count limit and IIR
HAL_StatusTypeDef BMP581_set_OorConfiguration(const BMP581_oor_press_configuration_t *oor_press_config);
// Used to perform NVM read
HAL_StatusTypeDef BMP581_NvmRead(uint8_t nvm_addr, uint16_t *nvm_data);
// Used to perform NVM write
HAL_StatusTypeDef BMP581_NvmWrite(uint8_t nvm_addr, const uint16_t *nvm_data);

#ifdef __cplusplus
}
#endif

#endif /* __BMP581_H */
