/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BNO_BOOT_Pin GPIO_PIN_13
#define BNO_BOOT_GPIO_Port GPIOC
#define BNO_RST_Pin GPIO_PIN_14
#define BNO_RST_GPIO_Port GPIOC
#define BNO_INT_Pin GPIO_PIN_0
#define BNO_INT_GPIO_Port GPIOA
#define BNO_INT_EXTI_IRQn EXTI0_IRQn
#define BMP_INT_Pin GPIO_PIN_2
#define BMP_INT_GPIO_Port GPIOA
#define BMP_INT_EXTI_IRQn EXTI2_TSC_IRQn
#define CS_MEM_Pin GPIO_PIN_0
#define CS_MEM_GPIO_Port GPIOB
#define GPS_RX_Pin GPIO_PIN_10
#define GPS_RX_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_11
#define GPS_TX_GPIO_Port GPIOB
#define GPS_RST_Pin GPIO_PIN_7
#define GPS_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define BNO_RST_On BNO_RST_GPIO_Port->BSRR = (uint32_t)BNO_RST_Pin;
#define BNO_RST_Off BNO_RST_GPIO_Port->BRR = (uint32_t)BNO_RST_Pin;

#define BNO_BOOT_On BNO_BOOT_GPIO_Port->BSRR = (uint32_t)BNO_BOOT_Pin;
#define BNO_BOOT_Off BNO_BOOT_GPIO_Port->BRR = (uint32_t)BNO_BOOT_Pin;

#define GPS_RST_On GPS_RST_GPIO_Port->BSRR = (uint32_t)GPS_RST_Pin;
#define GPS_RST_Off GPS_RST_GPIO_Port->BRR = (uint32_t)GPS_RST_Pin;
	
#define START_MARKER						60 // < Start marker for serial
#define END_MARKER							62 // > End marker for serial

#define CMD_CALIBRATE						67 // C Calibrate
#define RESPONSE_CALIBRATE_OK		CMD_CALIBRATE // C Calibrate OK
#define RESPONSE_CALIBRATE_FAIL	99 // c Calibrate command has failed
#define RESPONSE_DATA						68 // D Send sensor data to main board
#define RESPONSE_BMP_INIT_OK		75 // K
#define RESPONSE_BMP_INIT_FAIL	70 // F

#define RESPONSE_BNO_INIT_OK		107 // k
#define RESPONSE_BNO_INIT_FAIL	102 // f

// We get Start, Command and End
#define DEV_RX_LEN							3
// Rx buffer positions
#define START_POS								0
#define CMD_POS									1
#define END_POS									2

// Tx buffer positions 0 is < 1 is D, 2-7 BMP data, 8-19 BNO DATA, 20-55 GPS data, 56 CheckSum, 57 is >
#define BMP_POS									END_POS				// 
#define BNO_POS									BMP_POS + 8		// 10 
#define GPS_POS									BNO_POS + 20	// 30 
#define SUM_POS									GPS_POS + 26	// 56
#define DEV_TX_LEN							SUM_POS + 1		// 57

#define TIMEOUT_SEND						100
	
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
