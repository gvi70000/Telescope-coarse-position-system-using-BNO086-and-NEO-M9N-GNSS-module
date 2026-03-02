/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "BNO_08x_I2C.h"
#include "NEO_M9N.h"
#include "BMP581.h"
#include "HDC302x.h"
#include "EEPROM_SPI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// USART1 transmit complete flag — cleared by HAL_UART_TxCpltCallback, polled by transmitFrame()
volatile uint8_t txNotComplete = 0;
// Command received from main controller over USART1
volatile uint8_t crtCmd = 0;

// BMP581 data-ready flag — set by HAL_GPIO_EXTI_Callback on BMP_INT_Pin
volatile uint8_t BMP_Ready = 0;
// BMP581 pressure and temperature result
BMP581_sensor_data_t BMP581_Data;

// BNO086 data-ready flag — set by HAL_GPIO_EXTI_Callback on BNO_INT_Pin
// sensorData and rpy externs come from BNO_08x_I2C.h
volatile uint8_t BNO_Ready = 0;

// HDC302x sensor handles — Address set before HDC302x_Init()
HDC302x_t Sensor1;
HDC302x_t Sensor2;
// HDC302x data-ready flags — set by HAL_GPIO_EXTI_Callback on A1_Pin / A2_Pin
volatile uint8_t HDC1_Ready = 0;
volatile uint8_t HDC2_Ready = 0;
// Timestamp of last HDC302x read, used to enforce HDC_READ_TIME interval
uint32_t readTime;

// Latest HDC302x readings — used to compute avgTemp and avgRH in sendData()
float temp1 = 0.0f;
float temp2 = 0.0f;
float rh1   = 0.0f;
float rh2   = 0.0f;

// GPS new-data flag — set in HAL_UART_RxCpltCallback, cleared in main loop
// myGpsData and gpsData declared in NEO_M9N.h / NEO_M9N.c
volatile uint8_t newGpsData = 0;

// Transmit buffer — aligned(4) so HAL_CRC_Calculate() can safely cast to uint32_t*
// Unaligned 32-bit reads are UB on Cortex-M4
__attribute__((aligned(4))) uint8_t txBuffer[TX_LONG_SIZE];

// EXTI callback — sets sensor ready flags
// NOTE: HAL_GPIO_EXTI_IRQHandler already clears the pending bit. Do NOT call
// __HAL_GPIO_EXTI_CLEAR_IT here; it can re-trigger the interrupt on some STM32F3 silicon.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BMP_INT_Pin) {
        BMP_Ready = 1;
    }
    if (GPIO_Pin == BNO_INT_Pin) {
        BNO_Ready = 1;
    }
    if (GPIO_Pin == A1_Pin) {   // HDC Sensor 1 INTDRDY (active-low, FALLING edge)
        HDC1_Ready = 1;
    }
    if (GPIO_Pin == A2_Pin) {   // HDC Sensor 2 INTDRDY (active-low, FALLING edge)
        HDC2_Ready = 1;
    }
}

// Calculate CRC32 over txBuffer[0..length-1] and write the result at CRC_POS
// length must be a multiple of 4; HAL_CRC_Calculate() counts uint32_t words
void calculateAndAppendCRC(uint16_t length) {
    uint32_t crcValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)txBuffer, length / 4);
    txBuffer[CRC_POS]     = (uint8_t)(crcValue & 0xFF);          // CRC LSB
    txBuffer[CRC_POS + 1] = (uint8_t)((crcValue >> 8) & 0xFF);  // CRC MSB
}

// USART1 DMA transmit complete — release transmitFrame() polling loop
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        txNotComplete = 0;
    }
}

// USART DMA receive complete callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Static: DMA writes into this buffer after the callback returns —
        // a local (stack) variable would be freed before DMA finishes
        static uint8_t devRxBuffer[TX_SHORT_SIZE];
        uint8_t data_length = TX_SHORT_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        HAL_UART_DMAStop(&huart1);

        if (data_length > 4) {
            // TODO: parse command from devRxBuffer
        }

        HAL_UART_Receive_DMA(&huart1, devRxBuffer, TX_SHORT_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    }

    if (huart->Instance == USART3) {
        // DMA TC fires when all RX_BUFFER_LEN=100 bytes are received —
        // one exact UBX-NAV-PVT packet. Re-arm DMA immediately so the next
        // packet starts filling gpsData[] without a gap. No memset needed;
        // DMA overwrites all 100 bytes each transfer.
        __HAL_UART_SEND_REQ(&huart3, UART_RXDATA_FLUSH_REQUEST);
        HAL_UART_Receive_DMA(&huart3, gpsData, RX_BUFFER_LEN);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

        // Sync-byte guard: reject mis-aligned transfers that can occur on the
        // very first packet after startup if the module was mid-sentence
        if (gpsData[0] == 0xB5) {
            newGpsData = 1;
        }
    }
}

// Transmit txBuffer over USART1 DMA and block until complete or timeout
void transmitFrame(const uint8_t length) {
    uint32_t startTime = HAL_GetTick();
    txNotComplete = 1;

    HAL_UART_Transmit_DMA(&huart1, txBuffer, length);

    while (txNotComplete) {
        if (HAL_GetTick() - startTime > TIMEOUT_COMM) {
            HAL_UART_AbortTransmit(&huart1);  // Abort on timeout
            MX_USART1_UART_Init();            // Re-initialize UART
            txNotComplete = 0;
            break;
        }
    }
}

// Build and transmit the full sensor data frame over USART1
void sendData(void) {
    // Snapshot GPS data atomically — USART3 DMA ISR can write myGpsData at any
    // point; preemption mid struct-copy yields torn data
    __disable_irq();
    GPS_DATA_t gpsSnapshot = myGpsData;
    __enable_irq();

    txBuffer[COUNT_POS] = LONG_DATA;
    txBuffer[CMD_POS]   = RESPONSE_DATA;

    // Average temperature — include HDC sensors only if they have been read
    float avgTemp     = BMP581_Data.temperature;
    uint8_t tempCount = 1;
    if (temp1 != 0.0f) { avgTemp += temp1; tempCount++; }
    if (temp2 != 0.0f) { avgTemp += temp2; tempCount++; }
    avgTemp /= (float)tempCount;

    // Use memcpy to write multi-byte values into the uint8_t buffer —
    // direct float* casts violate strict aliasing under ARMCC V6
    memcpy(&txBuffer[TEMP_POS],     &avgTemp,               sizeof(float));
    memcpy(&txBuffer[PRESSURE_POS], &BMP581_Data.pressure,  sizeof(float));

    float avgRH = (rh1 + rh2) / 2.0f;
    memcpy(&txBuffer[RH_POS],  &avgRH,       sizeof(float));
    memcpy(&txBuffer[BNO_POS], &rpy,          sizeof(BNO_RollPitchYaw_t));
    memcpy(&txBuffer[GPS_POS], &gpsSnapshot,  sizeof(GPS_DATA_t));

    calculateAndAppendCRC(CRC_POS);
    txBuffer[END_POS] = END_MARKER;
    transmitFrame(TX_LONG_SIZE);
}

// Build and transmit a short status/response message over USART1
void sendMessage(uint8_t msgType) {
    txBuffer[COUNT_POS] = SHORT_DATA;
    txBuffer[CMD_POS]   = msgType;
    calculateAndAppendCRC(CRC_POS_S);
    txBuffer[END_POS_S] = END_MARKER;
    transmitFrame(TX_SHORT_SIZE);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration -------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_CRC_Init();

    /* USER CODE BEGIN 2 */

    M9N_Reset();
    HDC302x_Reset();

    // Configure BNO086 boot pins — BOOT_On = normal operation (low = DFU mode, 0x52 I2C address)
    BNO_RST_On;
    BNO_BOOT_On;

    // Wait for all sensors to complete power-on sequence
    HAL_Delay(1000);

    // Set fixed frame header bytes
    txBuffer[0] = START_MARKER;
    txBuffer[1] = START_MARKER;

    // Deselect M95512 EEPROM (CS idle-high)
    EEPROM_RELEASE;

    // Start GPS DMA reception
    M9N_Start();

    // Initialize HDC302x Sensor 2
    Sensor2.Address = HDC302X_SENSOR_2_ADDR;
    if (HDC302x_Init(&Sensor2) != HAL_OK) {
        printf("Sensor 2 initialization failed!\n");
    } else {
        printf("Sensor 2 initialized successfully.\n");
    }

    // Initialize HDC302x Sensor 1
    Sensor1.Address = HDC302X_SENSOR_1_ADDR;
    if (HDC302x_Init(&Sensor1) != HAL_OK) {
        printf("Sensor 1 initialization failed!\n");
    } else {
        printf("Sensor 1 initialized successfully.\n");
    }

    // Start HDC302x auto-measurement at 4 Hz LPM0 — sensor measures continuously;
    // HDC302x_ReadData() fetches the latest result from the output register
    HDC302x_StartAutoMeasurement(&Sensor1, HDC302X_CMD_AUTO_MEASUREMENT_4_PER_SECOND_LPM0);
    HDC302x_StartAutoMeasurement(&Sensor2, HDC302X_CMD_AUTO_MEASUREMENT_4_PER_SECOND_LPM0);
    readTime = HAL_GetTick();

    // Initialize BMP581
    if (BMP581_Init() == HAL_OK) {
        sendMessage(RESPONSE_BMP_INIT_OK);
    } else {
        sendMessage(RESPONSE_BMP_INIT_FAIL);
        printf("BMP initialization failed!\n");
    }

	// Initialize BNO086 (currently disabled)
  if (BNO_Init() == HAL_OK) {
      if (BNO_setHighAccuracyMode() != HAL_OK) {
          // Sensor alive but rpy will stay zero
          sendMessage(RESPONSE_BNO_INIT_FAIL);
          printf("BNO high accuracy mode failed!\n");
      } else {
          sendMessage(RESPONSE_BNO_INIT_OK);
      }
  } else {
      sendMessage(RESPONSE_BNO_INIT_FAIL);
      printf("BNO initialization failed!\n");
  }

	HAL_Delay(TIMEOUT_COMM);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // Handle calibration command from main controller
        if (crtCmd == CMD_CALIBRATE) {
            crtCmd = 0;
            if (BNO_calibrateHighAccuracyAndReset() == HAL_OK) {
                sendMessage(RESPONSE_CALIBRATE_OK);
            } else {
                sendMessage(RESPONSE_CALIBRATE_FAIL);
            }
        }

      // BNO086 — update rpy (5 Hz, interrupt-driven)
      // NOTE: do NOT clear BNO_Ready here. waitInt() inside BNO_dataAvailable()
      // finds BNO_Ready set and clears it itself. Clearing it here first causes
      // waitInt() to always time out → rpy never updates.
      // NOTE: isResetOccurred() reads a flag set by processResponse() inside
      if (BNO_Ready) {
          BNO_dataAvailable();    // waitInt() clears BNO_Ready; quaternionUpdate() updates rpy
          if (isResetOccurred()) {
              BNO_setFeature(ROTATION_VECTOR, 200000, 0);
          }
      }

      // BMP581 — update pressure and temperature (4 Hz, interrupt-driven)
      if (BMP_Ready) {
				BMP_Ready = 0;
				BMP581_Get_TempPressData(&BMP581_Data);
      }

      // HDC302x — read humidity and temperature at 4 Hz (HDC_READ_TIME = 250 ms)
      // Auto-measurement runs continuously inside the sensor; ReadData() fetches
      // the latest result from the output register
      if ((HAL_GetTick() - readTime) > HDC_READ_TIME) {
				if (HDC302x_ReadData(&Sensor1) == HAL_OK) {
					temp1 = Sensor1.Data.Temperature;
					rh1   = Sensor1.Data.Humidity;
				}
				if (HDC302x_ReadData(&Sensor2) == HAL_OK) {
					temp2 = Sensor2.Data.Temperature;
					rh2   = Sensor2.Data.Humidity;
				}
				readTime = HAL_GetTick();
      }

      // GPS — process and transmit at 4 Hz (driven by GPS nav rate)
      // Clear flag before processGPS() so a GPS interrupt arriving during
      // sendData() is not lost
      if (newGpsData) {
				newGpsData = 0;
				processGPS();
				sendData();
      }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
      * in the RCC_OscInitTypeDef structure.
      */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB bus clocks. */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART3
                                        | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection  = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Usart3ClockSelection  = RCC_USART3CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection    = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the source file and line number where assert_param failed.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
