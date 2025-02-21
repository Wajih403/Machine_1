#ifndef DWIN_Transmit_H_
#define DWIN_Transmit_H_
#endif
/*
  Author:     Muhammad Tayyab
  WebSite:    https://github.com/MuhammadTayyab1002   
  
  Version:    0.1
  
*/

#ifdef __cplusplus
extern "C" {
#endif
  
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
//####################################################################################################################




		
//####################################################################################################################

void Page_Change(UART_HandleTypeDef *huart,uint8_t page);
void Send_Int_2Bytes(UART_HandleTypeDef *huart,uint16_t VpAddress,uint16_t data);
void Reset_CPU(UART_HandleTypeDef *huart);
void Read_Int_2Bytes(UART_HandleTypeDef *huart,uint16_t VpAddress,uint8_t data_lenght);
void Page_Read(UART_HandleTypeDef *huart);

//####################################################################################################################