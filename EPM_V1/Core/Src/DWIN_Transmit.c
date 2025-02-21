#include "DWIN_Transmit.h"
#include "DWIN_TransmitConfig.h"
#include "main.h"

/* --------------- Check Mainstream series --------------- */

	#ifdef STM32F0
		#include "stm32f0xx_hal.h"       /* Import HAL library */
		#include "stm32f0xx_hal_tim.h" 
	#elif defined(STM32F1)
		#include "stm32f1xx_hal.h"       /* Import HAL library */
		#include "stm32f1xx_hal_tim.h"
	#elif defined(STM32F2)
		#include "stm32f2xx_hal.h"       /* Import HAL library */
		#include "stm32f2xx_hal_tim.h"
	#elif defined(STM32F3)
		#include "stm32f3xx_hal.h"       /* Import HAL library */
		#include "stm32f3xx_hal_tim.h"
	#elif defined(STM32F4)
		#include "stm32f4xx_hal.h"       /* Import HAL library */
		#include "stm32f4xx_hal_tim.h"
	#elif defined(STM32F7)
		#include "stm32f7xx_hal.h"       /* Import HAL library */
		#include "stm32f7xx_hal_tim.h"
	#elif defined(STM32G0)
		#include "stm32g0xx_hal.h"       /* Import HAL library */
		#include "stm32g0xx_hal_tim.h"
	#elif defined(STM32G4)
		#include "stm32g4xx_hal.h"       /* Import HAL library */
		#include "stm32g4xx_hal_tim.h"

	/* ------------ Check High Performance series ------------ */

	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
		#include "stm32h7xx_hal_tim.h"

	/* ------------ Check Ultra low power series ------------- */

	#elif defined(STM32L0)
		#include "stm32l0xx_hal.h"       /* Import HAL library */
		#include "stm32l0xx_hal_tim.h"
	#elif defined(STM32L1)
		#include "stm32l1xx_hal.h"       /* Import HAL library */0
		#include "stm32l1xx_hal_tim.h"
	#elif defined(STM32L5)
		#include "stm32l5xx_hal.h"       /* Import HAL library */
		#include "stm32l5xx_hal_tim.h"
	#elif defined(STM32L4)
		#include "stm32l4xx_hal.h"       /* Import HAL library */
		#include "stm32l4xx_hal_tim.h"
	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
		#include "stm32h7xx_hal_tim.h"
	#else
	#endif /* STM32F1 */

//#############################################################################################

void Page_Change(UART_HandleTypeDef *huart,uint8_t page)
{
	uint8_t Buffer[10]={0x5A,0XA5,0X07,0X82,0X00,0X84,0X5A,0X01,0X00,0X00};
	Buffer[9]=page;
	HAL_UART_Transmit(huart,(uint8_t *)Buffer,10,100);
}

//#############################################################################################

void Page_Read(UART_HandleTypeDef *huart)
{
	uint8_t Buffer[7]={0x5A,0xA5,0x04,0x83,0x00,0x14,0x01};
	
	HAL_UART_Transmit(huart,(uint8_t *)Buffer,7,100);
}

//####################################################################################################################

void Send_Int_2Bytes(UART_HandleTypeDef *huart,uint16_t VpAddress,uint16_t data)
{
	uint8_t Buffer[8]={0X5A,0XA5,0X05,0X82,0X10,0X00,0X00,0x00};
	Buffer[4] = (VpAddress >> 8);  	// High byte
  Buffer[5] = VpAddress;					// Low byte
	Buffer[6] = (data >> 8);  	// High byte
  Buffer[7] = data;					// Low byte  
	HAL_UART_Transmit(huart,(uint8_t *)Buffer,8,100);
}

//####################################################################################################################

void Reset_CPU(UART_HandleTypeDef *huart)
{
	uint8_t Buffer[10]={0x5A,0xA5,0x07,0x82,0x00,0x04,0x55,0xAA,0x5A,0xA5};
	HAL_UART_Transmit(huart,(uint8_t *)Buffer,10,100);
}

//####################################################################################################################

void Read_Int_2Bytes(UART_HandleTypeDef *huart,uint16_t VpAddress,uint8_t data_lenght)
{
	uint8_t Buffer[7]={0x5A,0xA5,0x04,0x83,0x10,0x00,0x02};
	
	Buffer[4] = (VpAddress >> 8);  	// High byte
  Buffer[5] = VpAddress;					// Low byte
	Buffer[6] = data_lenght;

	HAL_UART_Transmit(huart,(uint8_t *)Buffer,7,100);
}