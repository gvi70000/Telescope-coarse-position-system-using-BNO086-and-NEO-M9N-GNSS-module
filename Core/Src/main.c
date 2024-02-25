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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "BNO_08x_I2C.h"
#include "NEO_M9N.h"
#include "BMP581.h"
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

// Used to indicate USART1 transmission complete
volatile uint8_t txNotComplete = 0;
// Holds the command received from main controller
volatile uint8_t crtCmd = 0;

// Indicates that BMP581 has raised interrupt
volatile uint8_t BMP_Ready;
// Holds pressure and temperature
BMP581_sensor_data_t BMP581_Data;

// Indicates that BNO086 has raised interrupt
extern volatile uint8_t BNO_Ready;
// Holds BNO086 data
extern BNO_SensorValue_t sensorData;

// Halds GPS data
extern GPS_DATA_t myGpsData;

// Buffer for USART1 Rx
extern volatile uint8_t gpsRxBuffer[RX_BUFFER_LEN];

// Interrupt callback function for BMP581 and BNO086
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == BMP_INT_Pin) {
    BMP_Ready = 1;
  }
	
  if(GPIO_Pin == BNO_INT_Pin) {
    BNO_Ready = 1;
  }
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
}

// Function to generate checksum and place it in the last position of the array
void generateChecksum(uint8_t *array, uint8_t length) {
	uint8_t checkSum = 0;
	if (length < 3) return; // Ensure there's enough length for start, char, and checksum
	for (uint16_t i = 1; i < length - 1; i++) {
		checkSum += array[i];
	}
	array[length - 1] = checkSum; // Store the checksum in the last position
}

// DMA transmit complete callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART1) {
		txNotComplete = 0;
	}
}

// DMA receive complete callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// Main board
	if(huart->Instance == USART1) {
		uint8_t devRxBuffer[DEV_RX_LEN];
		//Determine how many items of data have been received
		uint8_t data_length = DEV_RX_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);//DMA1_Stream0->NDTR;
		//Stop DMA	
		HAL_UART_DMAStop(&huart1);
		// Shortest messege is <C>
		if(data_length > 2) {
			if((devRxBuffer[START_POS] == START_MARKER) && (devRxBuffer[END_POS] == END_MARKER)) {
				crtCmd = devRxBuffer[CMD_POS];
			}
		}
		HAL_UART_Receive_DMA(&huart1, (uint8_t *)devRxBuffer, DEV_RX_LEN);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

	}
	// GPS
	if(huart->Instance == USART3) {
		//Determine how many items of data have been received
		uint8_t data_length = RX_BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart3.hdmarx);//DMA1_Stream0->NDTR;
		//Stop DMA	
		HAL_UART_DMAStop(&huart3);
		if(data_length > 99){
			processGPS();
		}
		//memset(gpsRxBuffer, 0, RX_BUFFER_LEN);
		HAL_UART_Receive_DMA(&huart3,  (uint8_t *)gpsRxBuffer, RX_BUFFER_LEN);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

	}
}

// Transmit data from sensor to main board on USART1
void transmitData(const uint8_t *array, const uint8_t length) {
	uint32_t startTime = HAL_GetTick();
	txNotComplete = 1; // Assume transmission is not complete
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)array, length);
	while(txNotComplete) {
		// Check for timeout
		if (HAL_GetTick() - startTime > TIMEOUT_SEND) {
			// Timeout occurred, abort transmission
			HAL_UART_AbortTransmit(&huart1);
			// Re-initialize UART
			MX_USART1_UART_Init(); 
			txNotComplete = 0; // Clear the transmission flag to exit the loop
			break; // Exit the loop
		}
	}
}

// Fill the USART 1 Tx buffer with data for sending
void sendData() {
	myGpsData.newData = 0;
	uint8_t txBuffer[DEV_TX_LEN];
	// Add Start marker and data response
	txBuffer[START_POS] = START_MARKER;
	txBuffer[CMD_POS] = RESPONSE_DATA;
	// Add BMP581 data 2x float 8 bytes
	*(BMP581_sensor_data_t *)&txBuffer[BMP_POS] = BMP581_Data;
	// Add BNO data 5x float = 20 bytes
	*(BNO_RotationVectorWAcc_t *)&txBuffer[BNO_POS] = sensorData.SenVal.RotationVector;
	// Add GPS data 25 bytes, first byte is not used
	*(GPS_DATA_t *)&txBuffer[GPS_POS] = myGpsData;
	//txBuffer[56] is checkSum
	generateChecksum(txBuffer, SUM_POS);
	txBuffer[SUM_POS] = END_MARKER;
	transmitData(txBuffer, DEV_TX_LEN);
}

// Send a messages to Main board on USART1
void sendMessage(const uint8_t msgType) {
	uint8_t txBuffer[DEV_RX_LEN] = {START_MARKER, msgType, END_MARKER};
	transmitData(txBuffer, DEV_RX_LEN);
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

  /* MCU Configuration--------------------------------------------------------*/

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
  /* USER CODE BEGIN 2 */
	
	// Set up pins for BNO 
	BNO_RST_On;
	BNO_BOOT_On; // Off only for DFU mode 0x52 I2C Address
	// Unselect M95512 EEPROM
	EEPROM_RELEASE;
	// Start GPS
	M9N_Start();

	// Init and start BMP581
	if((BMP581_Init() == HAL_OK) && (BMP581_setHighAccuracy() == HAL_OK)) {
		sendMessage(RESPONSE_BMP_INIT_OK);
	} else {
		sendMessage(RESPONSE_BMP_INIT_FAIL);
	}
	
	// Init and start BNO086
	if(BNO_Init() == HAL_OK) {
		sendMessage(RESPONSE_BNO_INIT_OK);
	} else {
		sendMessage(RESPONSE_BNO_INIT_FAIL);
	}
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// If GPS has new data
		if(myGpsData.newData) {
			sendData();
		}
		// If we have a command
		if(crtCmd == CMD_CALIBRATE) {
			crtCmd = 0;	
			if(BNO_calibrateHighAccuracyAndReset() == HAL_OK){
				sendMessage(RESPONSE_CALIBRATE_OK);
			} else {
				sendMessage(RESPONSE_CALIBRATE_FAIL);
			}
		}
		// If BMP581 triggers interrup, check for new data
		if(BMP_Ready) {
			BMP_Ready = 0;
			BMP581_getSensorData(&BMP581_Data);
			// if(BMP581_getSensorData(&BMP581_Data) == HAL_OK) {			}
		}
		// If BNO086 triggers interrup, check for new data
		if(BNO_Ready) {
			// Did we have a reset? SetUp sensor again
			if(isResetOccurred()) {
				if(BNO_setFeature(ROTATION_VECTOR, 100000, 0) != HAL_OK) {
					//printf("Reset Occurred. Set feature failed\r\n");
				}
			}
			// If we have new data from BNO
			if((BNO_dataAvailable() == HAL_OK) && (sensorData.sensorId == ROTATION_VECTOR)) {
				sensorData.sensorId = 0;
				//printf("Y = %.3lf P = %.3lf R = %.3lf | %d \r\n", rpy.Yaw, rpy.Pitch, rpy.Roll, HAL_GetTick() - BNO_Time);
			}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
