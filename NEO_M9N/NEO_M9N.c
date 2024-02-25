#include "NEO_M9N.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
volatile uint8_t gpsRxBuffer[RX_BUFFER_LEN];
GPS_DATA_t myGpsData;
static uint8_t gpsSecond;
static uint32_t miliSecond;

static uint8_t checkSum(){
	uint8_t ckA = 0;
	uint8_t ckB = 0;
//	printf("----\n");
//	printf("0x%.2X 0x%.2X\n", gpsRxBuffer[CHECK_SUM_END_POS], gpsRxBuffer[CHECK_SUM_END_POS+1]);
	for(uint8_t i = gpsRxBuffer[CHECK_SUM_START_POS]; i < gpsRxBuffer[CHECK_SUM_END_POS]; i++){
		ckA += gpsRxBuffer[i];
		ckB = ckB+ckA;
	}
	if((ckA == gpsRxBuffer[CHECK_SUM_END_POS]) && (ckB == gpsRxBuffer[CHECK_SUM_END_POS+1])) return 1;
	return 0;	
}

void processGPS(void){
	// First message is the ID
	uint32_t ID = *(uint32_t *)gpsRxBuffer;
	// If we get the UBX-NAV-STATUS (uint32_t)(0xb5 0x62 0x01 0x03)
	if(PVT_ID == *(uint32_t *)gpsRxBuffer){
		if(checkSum()){
			// Set that flag that this is new data - to be used in Asto calculation
			myGpsData.newData = 1;
			myGpsData.UTC_Year = *(int16_t *)&gpsRxBuffer[YER_POS];
			myGpsData.UTC_Month = gpsRxBuffer[MON_POS];
			myGpsData.UTC_Day = gpsRxBuffer[DAY_POS];
			myGpsData.UTC_Hour = gpsRxBuffer[HOR_POS];
			myGpsData.UTC_Minute = gpsRxBuffer[MIN_POS];
			// Add milisecond to seconds - to be used in Asto calculation
			if(gpsSecond != gpsRxBuffer[SEC_POS]){
				gpsSecond = gpsRxBuffer[SEC_POS];
				myGpsData.UTC_Second = (float)gpsSecond;
			} else {
				myGpsData.UTC_Second += 0.2f;//(HAL_GetTick() - miliSecond) * ALTITUDE_FACTOR; //Can use +=0.2f at 5 Hz to save some CPU cycles
				//miliSecond = HAL_GetTick();
			}	
			myGpsData.UTC_Second = gpsRxBuffer[SEC_POS];
			myGpsData.Fix_Type = gpsRxBuffer[FIX_POS];
			myGpsData.Satellites = gpsRxBuffer[SAT_POS];
			myGpsData.Longitude = (float)(*(int32_t *)&gpsRxBuffer[LNG_POS]) * COORD_FACTOR;
			myGpsData.Latitude = (float)(*(int32_t *)&gpsRxBuffer[LAT_POS]) * COORD_FACTOR;
			myGpsData.Altitude = (float)(*(int32_t *)&gpsRxBuffer[MSL_POS]) * ALTITUDE_FACTOR;
		}
	}
}

	// Reset GPS
void M9N_Reset() {
	GPS_RST_Off;
	HAL_Delay(100);
	GPS_RST_On;
	HAL_Delay(100);
}

void M9N_Start(void){
	M9N_Reset();
	// Start receiveing
	HAL_UART_Receive_DMA(&huart3,  (uint8_t *)gpsRxBuffer, RX_BUFFER_LEN);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}



