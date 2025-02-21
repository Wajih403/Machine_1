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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STM_TB6600.h"
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX6675_CS_PIN_1 GPIO_PIN_13
#define MAX6675_CS_PORT GPIOF
#define MAX6675_CS_PIN_2 GPIO_PIN_15

#define Temp_Addr  		0x080E0002
#define MM_Addr  			0x080E0004
#define Speed_Addr  	0x080E0006
//#define STEPS_PER_REV 6400        // Steps per revolution (including microstepping)
//#define LEAD_SCREW_PITCH 2        // Lead screw pitch in mm
//#define MICROSTEPS 43           // Microstepping factor
//#define TIMER_CLOCK_FREQ 72000000 // Timer clock frequency (72 MHz)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int tf;
 int total_step;
 int target_step;
 volatile uint32_t ticks;
 int time;
 int ii=0;
 volatile uint32_t pCount=0;
struct Distance
{
	float A;
	float B;
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TempControl */
osThreadId_t TempControlHandle;
const osThreadAttr_t TempControl_attributes = {
  .name = "TempControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TempRead */
osThreadId_t TempReadHandle;
const osThreadAttr_t TempRead_attributes = {
  .name = "TempRead",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Reset */
osThreadId_t ResetHandle;
const osThreadAttr_t Reset_attributes = {
  .name = "Reset",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for USB */
osThreadId_t USBHandle;
const osThreadAttr_t USB_attributes = {
  .name = "USB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

tb6600_t motor;
struct Distance Dist;
// float variable



// uint8_t variable

uint8_t Rx_Buffer[9];

uint16_t Temp=0,Temp_0,Temp_1,Temp_Set=30,Temp_Get;
uint8_t Stop_Process=0,Start_Process=0,Manual=0;
uint8_t buff[20];
uint8_t Code[6],check;
uint8_t xTemp,xMM,xSpeed;
//

// uint16_t variable

uint16_t VP_Address;
uint16_t Volt,Curr,Time,Mm,mSpeed;

//

// uint32_t variable

uint32_t mCount,cCount,lCount,Calculate;
int inc1,inc2,inc3;
volatile int Page,Func,Data;
//
//
const char* Return_Buf[10]={"*PRS:STR#","*PRS:CMP#","*PRS:STP#","*MAN:HTR#","*MAN:MOT#","*SET:TMP#","*SET:MM#","*SET:SPD#","*SET:SAV#","*ERR:001#"}; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTempControl(void *argument);
void StartTempRead(void *argument);
void StartReset(void *argument);
void StartUSB(void *argument);
volatile int step_count=0;
void UpdateStepperSpeed(int speed_mm_sec,int distance);

/* USER CODE BEGIN PFP */
uint8_t ReadFlashByte(uint32_t address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	MX_USB_DEVICE_Init();
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,1);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,0);
	Temp_Get=ReadFlashByte(Temp_Addr);
	Mm=ReadFlashByte(MM_Addr);
	mSpeed=ReadFlashByte(Speed_Addr);
//Temp_Set=30;
//Mm=50;
//mSpeed=200;
	//
	
	//
	tb6600_init_tim(&motor,GPIOG,GPIO_PIN_0,GPIOE,GPIO_PIN_7,htim1,TIM_CHANNEL_1);
	tb6600_Set(&motor,CW,Fast,6400,72);
	//

	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TempControl */
  TempControlHandle = osThreadNew(StartTempControl, NULL, &TempControl_attributes);

  /* creation of TempRead */
  TempReadHandle = osThreadNew(StartTempRead, NULL, &TempRead_attributes);

  /* creation of Reset */
  ResetHandle = osThreadNew(StartReset, NULL, &Reset_attributes);

  /* creation of USB */
  USBHandle = osThreadNew(StartUSB, NULL, &USB_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 65535;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|Direction_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, CS1_Pin|CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Enable_Pin|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 Direction_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|Direction_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3
                           PF4 PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Pin CS2_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Enable_Pin PG13 PG14 */
  GPIO_InitStruct.Pin = Enable_Pin|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//

uint16_t MAX6675_ReadTemp(void)
{
  uint8_t data[2];
  uint16_t value;

  // Pull the CS pin low to enable the MAX6675
  HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN_1, GPIO_PIN_RESET);

  // Receive 2 bytes of data
  HAL_SPI_Receive(&hspi1, data, 2, HAL_MAX_DELAY);

	
  // Pull the CS pin high to disable the MAX6675
  HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN_1, GPIO_PIN_SET);

  // Combine the two bytes into a single 16-bit value
  value = (data[0] << 8) | data[1];

  // The temperature data is in the top 12 bits of the 16-bit value
  value >>= 3;
	value*=0.25;
	return value;
  // Return the temperature value

}

//

//

uint16_t MAX6675_Read_Temp(void)
{
  uint8_t data[2];
  uint16_t value;

  // Pull the CS pin low to enable the MAX6675
  HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN_2, GPIO_PIN_RESET);

  // Receive 2 bytes of data
  HAL_SPI_Receive(&hspi1, data, 2, HAL_MAX_DELAY);

	
  // Pull the CS pin high to disable the MAX6675
  HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN_2, GPIO_PIN_SET);

  // Combine the two bytes into a single 16-bit value
  value = (data[0] << 8) | data[1];

  // The temperature data is in the top 12 bits of the 16-bit value
  value >>= 3;
	value*=0.25;
	return value;
  // Return the temperature value

}

//

//

void Reset_Timer (TIM_HandleTypeDef *htim)
{
		__HAL_TIM_SET_COUNTER(htim,0);
		HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);
		HAL_TIM_Base_Stop_IT(htim);
}


//

//
uint8_t ReadFlashByte(uint32_t address)
{
    return *(uint8_t*)address;
}
//

//
void StoreFlashByte(uint8_t A,uint8_t B,uint8_t C)
{
	HAL_FLASH_Unlock();
  // Erase the specified Flash sector
	FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector        = FLASH_SECTOR_11;
    EraseInitStruct.NbSectors     = 1;
	HAL_FLASHEx_Erase(&EraseInitStruct,(uint32_t *)SectorError);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,0x080E0000,0);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,0x080E0002,A);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,0x080E0004,B);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,0x080E0006,C);
	HAL_FLASH_Lock();

}

//


//

void Usb_Receive_Check()
{
	sscanf((char*)buff,"%*c%d:%d:%d%*c",&Page,&Func,&Data);
	/*
	if(bath Placed!=0)
	{
		CDC_Transmit_HS((uint8_t *)Return_Buf[9],9);
	}
	else

	*/
	switch(Page)
	{
		case 1: // PROCESS
			switch(Func)
			{
				case 0:
					{
						Stop_Process=1;
						Start_Process=0;
						CDC_Transmit_HS((uint8_t *)Return_Buf[2],9);
						osThreadFlagsSet(ResetHandle, 0x02);
					}
					break;
				case 1:
						Start_Process=Data;
						osThreadFlagsSet(defaultTaskHandle, 0x01);
					break;
			}
			break;
		case 2: // MANUAL
			{
				osThreadFlagsSet(defaultTaskHandle, 0x01);
			}
			break;
		case 3: // SETTING
			switch(Func)
			{
				case 1:
						CDC_Transmit_HS((uint8_t *)Return_Buf[5],9);
						xTemp=Data;
					break;
				case 2:
						CDC_Transmit_HS((uint8_t *)Return_Buf[6],9);
						xMM=Data;
					break;
				case 3:
						CDC_Transmit_HS((uint8_t *)Return_Buf[7],9);
						xSpeed=Data;
					break;
				case 4:
						//give notificattion to save
						{
							CDC_Transmit_HS((uint8_t *)Return_Buf[8],9);
							osThreadFlagsSet(defaultTaskHandle, 0x01);
						}
					break;
			}
			break;
	}
			
}
//
void Home()
{
	if(Dist.B!=0)
	{
		if(Dist.B<0)
		{
			tb6600_ChangeDirection(&motor,CCW);
			tb6600_Enable(&motor);
			HAL_TIM_Base_Start_IT(&htim1);
			tb6600_Movemm_tim(&motor,Dist.B);
			mCount=GETVALUE(&motor);
		}
		else if(Dist.B>0)
		{
			tb6600_ChangeDirection(&motor,CW);
			tb6600_Enable(&motor);
			HAL_TIM_Base_Start_IT(&htim1);
			tb6600_Movemm_tim(&motor,Dist.B);
			mCount=GETVALUE(&motor);			
		}
	}
}
//

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever); // waiting for uart notification

		switch(Page)
		{
			case 1:  // Process
				if(Start_Process==1)
				{					
					while(Temp<Temp_Set)//dont start process until it has reached the required temperature set by the user
					{
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,1);//heater on
						HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,1);//led on
					}
				// Start Stirrer 
					CDC_Transmit_HS((uint8_t *)Return_Buf[0],9);
				// Start to&fro motion
					Temp_Set=Temp_Get;
					tb6600_Enable(&motor);
					TIM1->CNT=0;
					Calculate=1;
					HAL_TIM_Base_Start_IT(&htim1);
					tb6600_ChangeDirection(&motor,CCW);
					
					//tb6600_Movemm_tim(&motor,Mm); //changed here
					//tb6600_calculate_motor_parm(&motor, mSpeed, Mm);
					//tb6600_ChangeSpeed(&motor, mSpeed);
				//	UpdateStepperSpeed(mSpeed,Mm);
					 tb6600_Movemm_tim_1(&motor,mSpeed,Mm);
					
				
					mCount=GETVALUE(&motor);
					//lCount=0;
					pCount=0;
				// wait for command from pc
					osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
					CDC_Transmit_HS((uint8_t *)Return_Buf[1],9);
					cCount=Dist.A;
				// Stop to&fro motion
					HAL_TIM_Base_Stop_IT(&htim1);
					tb6600_Stop_tim(&motor);
					tb6600_Disable(&motor);
				// Change Page
					Temp_Set=30;
					Start_Process=0;
					Dist.A=0;
					inc1=0;
					mCount=0;
				//	lCount=0;
				pCount=0;
				}
				break;
			case 2:  // Manual
				switch(Func)
				{
					case 1:     
						CDC_Transmit_HS((uint8_t *)Return_Buf[3],9);
						switch(Data)
						{
							case 0:
								// Heater Off
								HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,0);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,0);

								break;
							case 1:
								//Heater on
								HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,1);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,1);

								break;
						}
						break;
					case 2:
						CDC_Transmit_HS((uint8_t *)Return_Buf[4],9);
						switch(Data)
						{
							case 0:
								// Actuate forward
								tb6600_Enable(&motor);
								tb6600_ChangeDirection(&motor,CW);
								tb6600_Movemm_tim(&motor,10);
								//tb6600_ChangeSpeed(&motor, mSpeed);//changed here
							  //tb6600_calculate_motor_parm(&motor, mSpeed, Mm);
							  //UpdateStepperSpeed(mSpeed,Mm);
								HAL_TIM_Base_Start_IT(&htim1);
								mCount=GETVALUE(&motor);
								break;
							case 1:
								// Actuate Reverse
								tb6600_Enable(&motor);
								tb6600_ChangeDirection(&motor,CCW);
								tb6600_Movemm_tim(&motor,10);
								//tb6600_ChangeSpeed(&motor, mSpeed);
							  //tb6600_calculate_motor_parm(&motor, mSpeed, Mm);
							  //UpdateStepperSpeed(mSpeed);
							  //tb6600_Disable(&motor);
								HAL_TIM_Base_Start_IT(&htim1);
								mCount=GETVALUE(&motor);
								break;
						}
						break;
				}
				break;
			case 3:  // Setting
				StoreFlashByte(xTemp,xMM,xSpeed);
				Temp_Set=xTemp;
				Mm=xMM;
				mSpeed=xSpeed;
				break;
		}
		
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTempControl */
/**
* @brief Function implementing the TempControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTempControl */
void StartTempControl(void *argument)
{
  /* USER CODE BEGIN StartTempControl */
  /* Infinite loop */
  for(;;)
  {
		// if(bath_placed==0)
   if((Temp<Temp_Set) && (Data == 1))
	 {
		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,1);
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,1);
		 tf = 1;
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,0);
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,0);
		 tf = 0;
	 }
  }
  /* USER CODE END StartTempControl */
}

/* USER CODE BEGIN Header_StartTempRead */
/**
* @brief Function implementing the TempRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTempRead */
void StartTempRead(void *argument)
{
  /* USER CODE BEGIN StartTempRead */
  /* Infinite loop */
  for(;;)
  {
    Temp_0=MAX6675_ReadTemp();
    osDelay(500);
		Temp_1=MAX6675_Read_Temp();
    osDelay(500);
		Temp=Temp_0;
		//Temp=20;
		if(Start_Process==1 || Start_Process==0)
		{
			char BUF;
			sprintf(&BUF,"*1:2:%d#",Temp);
			CDC_Transmit_HS((uint8_t*)&BUF, strlen(&BUF));
		}
  }
  /* USER CODE END StartTempRead */
}

/* USER CODE BEGIN Header_StartReset */
/**
* @brief Function implementing the Reset thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReset */
void StartReset(void *argument)
{
  /* USER CODE BEGIN StartReset */
  /* Infinite loop */
  for(;;)
  {
    osThreadFlagsWait(0x02, osFlagsWaitAny, osWaitForever);
    if(Stop_Process == 1)
		{
			Reset_Timer (&htim1);
			HAL_TIM_Base_Stop_IT(&htim1);
			tb6600_Disable(&motor);
			vTaskSuspend(defaultTaskHandle);
			vTaskDelete(defaultTaskHandle);
			/* Definitions for defaultTask */
			osThreadId_t defaultTaskHandle;
			const osThreadAttr_t defaultTask_attributes = {
				.name = "defaultTask",
				.stack_size = 128 * 4,
				.priority = (osPriority_t) osPriorityNormal,
			};
			defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
			Stop_Process = 0;
			
		}
  }
  /* USER CODE END StartReset */
}

/* USER CODE BEGIN Header_StartUSB */
/**
* @brief Function implementing the USB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSB */
void StartUSB(void *argument)
{
  /* USER CODE BEGIN StartUSB */
  /* Infinite loop */
  for(;;)
  {
		if(check==1)
		{
			Usb_Receive_Check();
			check=0;
		}
		if(Calculate==1)
		{
			if(Start_Process==1)
			{
				Dist.A=inc1/800;  
			}
			else if(Manual==1)
			{
				
			}
			else
			{

			}
		}
  }
  /* USER CODE END StartUSB */
}


//void UpdateStepperSpeed(int speed_mm_sec,int distance)
//{
//	ii++;
//    __HAL_TIM_SET_AUTORELOAD(&htim1,0);
//    const int STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / LEAD_SCREW_PITCH;
//    int steps_per_sec = STEPS_PER_MM * speed_mm_sec;  // For 1mm/sec, this will be STEPS_PER_MM
//    uint32_t timer_period = (TIMER_CLOCK_FREQ / steps_per_sec) - 1;
//	 total_step = ((STEPS_PER_REV)/LEAD_SCREW_PITCH)*distance;
//	 target_step = total_step;
//	 time = distance/speed_mm_sec;
//	 uint32_t current_tick = HAL_GetTick();
//    if (timer_period > 0xFFFF) {
//        timer_period = 0xFFFF;  // Limit the period to 16-bit max value
//    }

//    __HAL_TIM_SET_AUTORELOAD(&htim1, timer_period);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, timer_period / 2); 

////    __HAL_TIM_SET_AUTORELOAD(&htim5, timer_period);
////    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, timer_period / 2); // 50% duty cycle
////		
//		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//		
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM1)
	{	
		if(HAL_GPIO_ReadPin(motor.direction_gpio,motor.direction_pin) == 1)
			{
				inc1++;
			}
			else if(HAL_GPIO_ReadPin(motor.direction_gpio,motor.direction_pin) == 0 && Dist.A!=0)
			{
				inc1--;
			}
			
		if(Start_Process==1)//ccw
		 {
			pCount++;
			if(pCount==mCount)
			{
				// change direction motor
				tb6600_ToggleDirection(&motor);
				pCount=0;
			}
		 }
		else
		{
			mCount--;
			if(mCount<=0)
			{
				// change direction motor
				tb6600_Stop_tim(&motor);
				tb6600_Disable(&motor);
				HAL_TIM_Base_Stop_IT(&htim1);
				Manual=0;
			}
		}
	}
  /* USER CODE END Callback 1 */
}





//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
  /* USER CODE BEGIN Callback 0 */
	
  /* USER CODE END Callback 0 */
  //if (htim->Instance == TIM2) {
   // HAL_IncTick();
  //}
  /* USER CODE BEGIN Callback 1 */
	//if (htim->Instance == TIM1)
	//{	
	//	if(HAL_GPIO_ReadPin(motor.direction_gpio,motor.direction_pin) == 1)
	//		{
		//		inc1++;
		//	}
		//	else if(HAL_GPIO_ReadPin(motor.direction_gpio,motor.direction_pin) == 0 && Dist.A!=0)
		//	{
		//		inc1--;
		//	}
			
	//	if(Start_Process==1)
		// {
		//	lCount++;
		//	if(lCount==mCount)
		//	{
				// change direction motor
			//	tb6600_ToggleDirection(&motor);
			//	lCount=0;
			//}
		 //}
		//else
		//{
		//	mCount--;
//			step_count=step_count+1;
//			if(step_count>=target_step)
//				{
//					step_count=0;
//					tb6600_Disable(&motor);
//		
//					HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
//			}
		//	if(mCount<=0)
		//	{
					
	
				// change direction motor
			//	tb6600_Stop_tim(&motor);
			//	tb6600_Disable(&motor);
			//	HAL_TIM_Base_Stop_IT(&htim1);
			//	Manual=0;
			//}
	//	}
	//}
  /* USER CODE END Callback 1 */
//}

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
