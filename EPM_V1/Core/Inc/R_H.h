#ifndef R_H_H_
#define R_H_H_
#endif


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

//#############################################################################################

void Remote_Lck();
void Reset_supply();
void Supply_On();
void Supply_Off();
void Channel_Select(uint8_t CHN);
void Set_voltage(float Volt);
void Set_Current(float Curr);
void IDN(UART_HandleTypeDef *huart);
void Check_Apply();
void Set_Apply(float Volt,float Curr);





