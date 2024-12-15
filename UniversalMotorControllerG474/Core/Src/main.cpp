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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UtiliteFunctions.h"
//#include <cstring>
//#include <cstdio>
extern "C"
{
#include "socket.h"
#include "string.h"
#include "W5500/w5500.h"
}
#include "TCPInterface.h"
#include "RING_BUFFER/CommandStructures.h"
#include <cmath>
#include <vector>

#include "DeviceSignalInterfaces.h"
#include "ControlClassesInternal.h"
#include "AimingModules.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
  //====================================
  uint16_t DAC_VALUE = 0;
  uint16_t DAC_AMPLITUDE = 2000;
  float ValueCounter = 0;
  uint16_t PERIOD = 360/2;
  std::pair<uint16_t,uint16_t> DACControlSignal;
  uint8_t DelayTime = 2;
  //====================================

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//template<typename V, typename DEV_T, int N_C = 1>
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* Definitions for TaskMonitoring */
osThreadId_t TaskMonitoringHandle;
const osThreadAttr_t TaskMonitoring_attributes = {
  .name = "TaskMonitoring",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow
};
/* Definitions for TaskTransmisson */
osThreadId_t TaskTransmissonHandle;
const osThreadAttr_t TaskTransmisson_attributes = {
  .name = "TaskTransmisson",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh
  //.priority = (osPriority_t) osPriorityRealtime
  //.priority = (osPriority_t) osPriorityISR
};
/* Definitions for TaskControl */
osThreadId_t TaskControlHandle;
const osThreadAttr_t TaskControl_attributes = {
  .name = "TaskControl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow
};
/* Definitions for QueueAimingControl */
osMessageQueueId_t QueueAimingControlHandle;
const osMessageQueueAttr_t QueueAimingControl_attributes = {
  .name = "QueueAimingControl"
};

/* Definitions for QueueMonitoring */
osMessageQueueId_t QueueMonitoringHandle;
const osMessageQueueAttr_t QueueMonitoring_attributes = {
  .name = "QueueMonitoring"
};


/* USER CODE BEGIN PV */
osMessageQueueId_t QueueAimingProcessingHandle;
const osMessageQueueAttr_t QueueAimingProcessing_attributes = {
  .name = "QueueAimingProcessing"
};

osMessageQueueId_t QueueAimingMonitoringHandle;
const osMessageQueueAttr_t QueueAimingMonitoring_attributes = {
  .name = "QueueAimingMonitoring"
};

std::map<SPI_TypeDef*,bool*> TransmissionFlagsSPI;
bool FLAG_CONTROL_COMMAND_GET = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
void RunTaskMonitoring(void *argument);
void RunTaskTransmission(void *argument);
void RunTaskControl(void *argument);

/* USER CODE BEGIN PFP */
DeviceSignalControl<DAC_HandleTypeDef, uint16_t,2> InternalDACControl(hdac1);
DeviceSignalControl<SPI_HandleTypeDef,uint16_t,2> ExternalDACControl(hspi3, std::make_pair(SPI3_CS_GPIO_Port, SPI3_CS_Pin));

AimingFixedStepClass<2> AimingInterpolation;

AimingDinamicClass<float,AimingType::DIRECT,2> AimingProlong;
AimingDinamicClass<float,AimingType::PID_VELOCITY,2> AimingLoopPID;

PassCoordTransform<float,uint16_t,SystemScaleSettings::ConvertPix_DAC> PixToDAC;
MessageInternalMonitoring MessageMonitoringStatic;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint64_t MicrosecondsCounter = 0;

std::pair<float, float> ControlSignal;
std::pair<uint16_t, uint16_t> ControlSignalDAC;

ControlMessage1 RemoteControlCommand;

void TimerMicrosecondsTickISR() { MicrosecondsCounter++; }


void TimerControlDirectISR()
{
	if(osMessageQueueGetCount(QueueAimingControlHandle) < 1 ) return;
     osMessageQueueGet(QueueAimingControlHandle,&RemoteControlCommand,0,0);

    ControlSignalDAC.first = RemoteControlCommand.Channel1.Position;
    ControlSignalDAC.second = RemoteControlCommand.Channel1.Position;
    //ExternalDACControl.SetCoord(ControlSignal);
    InternalDACControl.SetCoord(ControlSignalDAC);
}

void TimerAimingInterpolateISR()
{
	if(osMessageQueueGetCount(QueueAimingControlHandle) >= 3 )
  {
     osMessageQueueGet(QueueAimingControlHandle,&RemoteControlCommand,0,0);

		 ControlSignal.first = RemoteControlCommand.Channel1.Position;
		 ControlSignal.second = RemoteControlCommand.Channel2.Position;
		 ControlSignal | AimingInterpolation;

     osMessageQueueGet(QueueAimingControlHandle,&RemoteControlCommand,0,0);

		 ControlSignal.first = RemoteControlCommand.Channel1.Position;
		 ControlSignal.second = RemoteControlCommand.Channel2.Position;

		 ControlSignal | AimingInterpolation | PixToDAC | InternalDACControl;
  }
                     else
						         AimingInterpolation | PixToDAC | InternalDACControl; //IF NO UPDATE GET PROLONG AIMING
}

void TimerAimingProlongISR()
{
  if(osMessageQueueGetCount(QueueAimingControlHandle) > 0 )
  {
     osMessageQueueGet(QueueAimingControlHandle,&RemoteControlCommand,0,0);
     RemoteControlCommand.Channel1 | AimingProlong;
     RemoteControlCommand.Channel2 | AimingProlong | PixToDAC | InternalDACControl;

  }
                                    else
                                    {
                                    AimingProlong | PixToDAC | InternalDACControl; //IF NO UPDATE GET PROLONG AIMING
                                    }


}

void TimerAimingLoopISR()
{
  if(osMessageQueueGetCount(QueueAimingControlHandle) > 1 )
  {
     osMessageQueueGet(QueueAimingControlHandle,&RemoteControlCommand,0,0);

     //AimingLoopPID.SetState(RemoteControlCommand.Channel1);
     //AimingLoopPID.SetState(RemoteControlCommand.Channel2);

     RemoteControlCommand.Channel1 | AimingLoopPID;
     RemoteControlCommand.Channel1 | AimingLoopPID | PixToDAC | InternalDACControl;
  }
                                     else
                                     AimingLoopPID | PixToDAC | InternalDACControl; //IF NO UPDATE GET PROLONG AIMING

}

//GenericSignalControl* SignalControl;
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
  HAL_Delay(4000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  eprintf(" \r\n", 6); eprintf(" [ UNIVERSAL MOTOR CONTROLLER %d ] \r\n", 7); HAL_Delay(1000);

  uint16_t SysFreq = HAL_RCC_GetSysClockFreq()/1000000;
  eprintf(" [ SYS FREQ:  %d ] \r\n", SysFreq); HAL_Delay(100);

  InternalDACControl.Init();
  //ExternalDACControl.Init();

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

  /* Create the queue(s) */
  /* creation of QueueAimingControl */

  QueueAimingControlHandle = osMessageQueueNew (16, sizeof(ControlMessage1), &QueueAimingControl_attributes);
  // FILL IN TCP SERVER TO AIMING PERFORM ISR 

  /* creation of QueueMonitoring */
  QueueMonitoringHandle = osMessageQueueNew (8, sizeof(MessageInternalMonitoring), &QueueMonitoring_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  QueueAimingProcessingHandle = osMessageQueueNew (8, sizeof(ControlMessage1), &QueueAimingProcessing_attributes); 
  // FILL IN TCP SERVER TO AIMING PROCESSIGN TASK 
  QueueAimingMonitoringHandle = osMessageQueueNew (8, sizeof(int), &QueueAimingMonitoring_attributes); // FILL IN AIMING ISR
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskMonitoring */
  TaskMonitoringHandle = osThreadNew(RunTaskMonitoring, NULL, &TaskMonitoring_attributes);
  /* creation of TaskTransmisson */
  /* creation of TaskControl */
  TaskControlHandle = osThreadNew(RunTaskControl, NULL, &TaskControl_attributes);

  TaskTransmissonHandle = osThreadNew(RunTaskTransmission, NULL, &TaskTransmisson_attributes);

  //HAL_Delay(100);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 17;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100 -1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 170;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, W5500_Reset_Pin|SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI3_CS_Pin|EXTN_DAC_CLR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : W5500_Reset_Pin SPI2_CS_Pin */
  GPIO_InitStruct.Pin = W5500_Reset_Pin|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI3_CS_Pin EXTN_DAC_CLR_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin|EXTN_DAC_CLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if(TransmissionFlagsSPI.empty()) return;

  *TransmissionFlagsSPI[hspi->Instance] = true;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_RunTaskMonitoring */
/**
  * @brief  Function implementing the TaskMonitoring thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_RunTaskMonitoring */
void RunTaskMonitoring(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  eprintf("[START MONITOR TASK] \r\n");

  /* Infinite loop */
  MessageInternalMonitoring Monitoring;

  for(;;)
  {

	//if(osMessageQueueGetCount(QueueAimingMonitoringHandle) > 0 ) osMessageQueueGet(QueueAimingMonitoringHandle, &AimingMonitorParam, 0, 0);

	//if(osMessageQueueGetCount(QueueMonitoringHandle) == 0 ) { osDelay(1000); continue; }
	//   osMessageQueueGet(QueueMonitoringHandle, &Monitoring, 0, 0);

    //eprintf("[ MONITOR ] %d ms %d mks %d %d bytes \r\n" , DurationTransmission/100,);
    osDelay(2000);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_RunTaskTransmission */
/**
* @brief Function implementing the TaskTransmisson thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RunTaskTransmission */
void RunTaskTransmission(void *argument)
{
  /* USER CODE BEGIN RunTaskTransmission */
  /* Infinite loop */

	TCPInterface DeviceInterface;
	//eprintf("[START TASK TRANSMISSION]\r\n");
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim6);

                 //DeviceInterface.PerformTransmission();
                 DeviceInterface.PerformTransmissionUDP();

	for(;;)
	{
	eprintf("[SERVER STOPPED]\r\n");
	osDelay(1000);
	}
  /* USER CODE END RunTaskTransmission */
}

/* USER CODE BEGIN Header_RunTaskControl */
void delay(uint32_t time_delay) { uint32_t i; for(i = 0; i < time_delay; i++); }

uint8_t trbuff[3];
void WriteDACValue(uint16_t Value, uint8_t Channel)
{
	uint8_t ChannelID = 0x18; if(Channel == 2) ChannelID = 0x19;
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	trbuff[0] = ChannelID;
	trbuff[1] = Value >>8;
	trbuff[2] = Value;
	HAL_SPI_Transmit_IT(&hspi3, trbuff, 3);
	while( !(*TransmissionFlagsSPI[SPI3]) ){}; *TransmissionFlagsSPI[SPI3] = false;
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
}

void WaitQueueFill(osMessageQueueId_t& Queue, uint16_t MessageCount)
{
  while(osMessageQueueGetCount(Queue) < MessageCount)
  {
     eprintf("[ WAIT MOTOR COMMAND %d] \r\n",osMessageQueueGetCount(QueueAimingControlHandle));
     osDelay(2000);
  };
}

void DAC_SinusGenerateTest()
{
  ////====================================
  //uint16_t DAC_VALUE = 0;
  //uint16_t DAC_AMPLITUDE = 2000;
  //float ValueCounter = 0;
  //uint16_t PERIOD = 360/2;
  //std::pair<uint16_t,uint16_t> DACControlSignal;
  //uint8_t DelayTime = 2;
  ////====================================

  while(true)
  {
    ValueCounter++; if(ValueCounter == PERIOD) ValueCounter = 0;
    DAC_VALUE = DAC_AMPLITUDE + DAC_AMPLITUDE*std::sin(ValueCounter*2*M_PI/PERIOD);
    //WriteDACValue(DAC_VALUE, 1); osDelay(1);
    //WriteDACValue(DAC_VALUE, 2); osDelay(1);

                    DACControlSignal.first = DAC_VALUE;
                    DACControlSignal.second = DAC_VALUE/2;
          //ExternalDACControl.SetValue(DACControlSignal); osDelay(DelayTime);
          InternalDACControl.SetCoord(DACControlSignal); osDelay(DelayTime);
    //continue;
  }
}

void DAC_SinusGenerateTest2()
{
    ValueCounter++; if(ValueCounter == PERIOD) ValueCounter = 0;
    DAC_VALUE = DAC_AMPLITUDE + DAC_AMPLITUDE*std::sin(ValueCounter*2*M_PI/PERIOD);

              DACControlSignal.first = DAC_VALUE;
              DACControlSignal.second = DAC_VALUE/2;
    InternalDACControl.SetCoord(DACControlSignal); osDelay(DelayTime);
}


/**
* @brief Function implementing the TaskControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RunTaskControl */
void RunTaskControl(void *argument)
{
  /* USER CODE BEGIN RunTaskControl */
  /* Infinite loop */

  eprintf("[START AIMING TASK] \r\n"); osDelay(2);


  //TimerControl MessagePerformTimer(htim6);
  //ParamRegulator PeriodRegulator(2.3*10,2*2,MessagePerformTimer.Period);
  //uint16_t Val = 10;

  //Val | PeriodRegulator | MessagePerformTimer;

  ControlMessage1 AimingControlCommand;
  MessageInternalMonitoring MessageMonitoring;


  eprintf("[AIMING FREQ INTERP: %d] \r\n", AimingInterpolation.AimingChannel1.StepFreq); osDelay(2);
  int SummValue = 0;
  uint16_t MessageWait = 0;
  uint16_t ThinningCounter = 0;
  uint16_t AvarageCounter = 0;
  uint8_t DelayTime = 10000;
  std::pair<uint16_t,uint16_t> DACControlSignal;

  //DAC_SinusGenerateTest();
  //WaitQueueFill(QueueAimingControlHandle,10);
  while(true)
  {


  //if(osMessageQueueGetCount(QueueAimingProcessingHandle) < 1 ) { osDelay(DelayTime); continue;}
  //      osMessageQueueGet(QueueAimingProcessingHandle,&AimingControlCommand,0,0);

  //AimingControlCommand.Channel1.PositionRel = 200;
  //AimingControlCommand.Channel2.PositionRel++;
  //osMessageQueuePut(QueueAimingControlHandle, &AimingControlCommand, 0, 0);
  //if(osMessageQueueGetCount(QueueAimingControlHandle) > 0 ) FLAG_CONTROL_COMMAND_GET = true;

  osDelay(DelayTime);


  //eprintf("[ SEND TEST MOTOR COMMAND] \r\n");
	//==============================================================================
  //MONITOR REMOTE COMMANDS 

	//ThinningCounter++; if((ThinningCounter % 10) == 0)
	//{
	//	  MessageMonitoring.Param1 = AimingControlCommand.Channel1.Position;
	//	  MessageMonitoring.Param2 = AimingControlCommand.Channel1.Velocity;
	//	  MessageMonitoring.Param3 = AimingControlCommand.Channel1.Acceleration;
	//	  MessageMonitoring.Param4 = AimingControlCommand.Channel1.PositionRel;
	//	  MessageMonitoring.Param5 = AimingControlCommand.Channel2.PositionRel;

	//	osMessageQueuePut(QueueMonitoringHandle, &MessageMonitoring, 0, 0);
	//}
	//==============================================================================

  }

  /* USER CODE END RunTaskControl */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  *
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM1) {
    TimerMicrosecondsTickISR();
  }

  if (htim->Instance == TIM6) 
  {
    TimerControlDirectISR();
    //TimerAimingInterpolateISR();
    //TimerAimingProlongISR();
    //TimerAimingLoopISR();
    //MeasurePeriodWorkTimer();
  }

  /* USER CODE END Callback 1 */
}

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
