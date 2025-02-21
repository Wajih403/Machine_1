#include "R_H.h"
#include "main.h"
#include "string.h"
#include "usbd_cdc_if.h"
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

// Description: Clear Status
const char* command_CLS = "*CLS *IDN\n";

// Description: Reset
const char* command_RST = "*RST *IDN\n";

// Description: Identification
const char* command_IDN = "*IDN?\n";

// Description: Error Check
const char* command_ERR_CHK = "SYST:ERR[:NEXT]? *IDN\n";

// Description: Local System
const char* command_SYS_LOC = "SYST:LOC *IDN\n";

// Description: Remote System
const char* command_SYS_REM = "SYST:REM *IDN\n";

// Description: Remote Lock System
const char* command_SYS_REM_LCK = "SYST:RWL *IDN\n";

// Description: Mix System
const char* command_SYS_MIX = "SYST:MIX *IDN\n";

// Description: System version Check
const char* command_SYS_VER = "SYST:VERS? *IDN\n";

// Description: Channel Check
const char* command_CHN_CHK = "INST? *IDN\n";

// Description: Channel Select
const char* command_CHN_SEL = "INST OUT1 *IDN\n";

// Description: Channel NCheck
const char* command_CHN_NCHK = "INST:NSEL? *IDN\n";

// Description: Channel NSelect
const char* command_CHN_NSEL = "INST:NSEL 1 *IDN\n";

// Description: Voltage Set
const char* command_VOL_SET = "VOLT 10 *IDN\n";

// Description: Voltage Check
const char* command_VOL_CHK = "VOLT? *IDN\n";

// Description: Voltage Step Set
const char* command_VOL_STP_SET = "VOLT:STEP 2 *IDN\n";

// Description: Voltage Step Check
const char* command_VOL_STP_CHK = "VOLT:STEP? *IDN\n";

// Description: Voltage Increment
const char* command_VOL_INC = "VOLT UP *IDN\n";

// Description: Voltage decrement
const char* command_VOL_DEC = "VOLT DOWN *IDN\n";

// Description: Current Set
const char* command_CUR_SET = "CURR 10 *IDN\n";

// Description: Current Check
const char* command_CUR_CHK = "CURR? *IDN\n";

// Description: Current Step Set
const char* command_CUR_STP_SET = "CURR:STEP 2 *IDN\n";

// Description: Current Step Check
const char* command_CUR_STP_CHK = "CURR:STEP? *IDN\n";

// Description: Current Increment
const char* command_CUR_INC = "CURR UP *IDN\n";

// Description: Current decrement
const char* command_CUR_DEC = "CURR DOWN *IDN\n";

// Description: Voltage & Current Set
const char* command_V_C_SET = "APPLY 6,2 *IDN\n";

// Description: Voltage & Current CHECK
const char* command_V_C_CHK = "APPLY? *IDN\n";

// Description: Channel On
const char* command_CHN_ON = "OUTP:SEL ON *IDN\n";

// Description: Channel Off
const char* command_CHN_OFF = "OUTP:SEL OFF *IDN\n";

// Description: Channel STAT ON
const char* command_CHN_STA_ON = "OUTP:STAT ON *IDN\n";

// Description: Channel STAT OFF
const char* command_CHN_STA_OFF = "OUTP:STAT OFF *IDN\n";

// Description: General ON
const char* command_GEN_ON = "OUTP:GEN ON *IDN\n";

// Description: General OFF
const char* command_GEN_OFF = "OUTP:GEN OFF *IDN\n";



//#############################################################################################

void IDN(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(huart,(uint8_t *)command_IDN, strlen(command_IDN),HAL_MAX_DELAY);
}

//void IDN()
//{
//	CDC_Transmit_FS((uint8_t *)command_IDN, strlen(command_IDN));
//}

//void Remote_Lck()
//{
//	CDC_Transmit_HS((uint8_t *)command_SYS_REM_LCK, strlen(command_SYS_REM_LCK));
//}

//void Reset_supply()
//{
//	CDC_Transmit_HS((uint8_t *)command_RST, strlen(command_RST));
//}

//void Supply_On()
//{
//	CDC_Transmit_HS((uint8_t *)command_GEN_ON, strlen(command_GEN_ON));
//}

//void Supply_Off()
//{
//	CDC_Transmit_HS((uint8_t *)command_GEN_OFF, strlen(command_GEN_OFF));
//}

//void Channel_Select(uint8_t CHN)
//{
//	char BUF;
//	sprintf(&BUF,"INST:NSEL %d *IDN\n",CHN);
//	CDC_Transmit_HS((uint8_t*)&BUF, strlen(&BUF));
//}


//void Set_voltage(float Volt)
//{
//	char BUF;
//	sprintf(&BUF,"VOLT %f *IDN\n",Volt);
//	CDC_Transmit_HS((uint8_t*)&BUF, strlen(&BUF));
//}

//void Set_Current(float Curr)
//{
//	char BUF;
//	sprintf(&BUF,"CURR %f *IDN\n",Curr);
//	CDC_Transmit_HS((uint8_t*)&BUF, strlen(&BUF));
//}

//void Set_Apply(float Volt,float Curr)
//{
//	char BUF;
//	sprintf(&BUF,"APPLY %f,%f *IDN\n",Volt,Curr);
//	CDC_Transmit_HS((uint8_t*)&BUF, strlen(&BUF));
//}
//void Check_Apply()
//{
//	CDC_Transmit_HS((uint8_t *)command_V_C_CHK, strlen(command_V_C_CHK));
//}