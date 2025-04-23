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
#include <cmath>
#include <vector>

#include "DeviceSignalInterfaces.h"
#include "ControlClassesInternal.h"

#include "TCPInterface.h"
#include "AimingModules.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//template<typename V, typename DEV_T, int N_C = 1>
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

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
  .priority = (osPriority_t) osPriorityNormal
};
/* Definitions for TaskControl */
osThreadId_t TaskControlHandle;
const osThreadAttr_t TaskControl_attributes = {
  .name = "TaskControl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime
};
/* Definitions for TaskMeasure */
osThreadId_t TaskMeasureHandle;
const osThreadAttr_t TaskMeasure_attributes = {
  .name = "TaskMeasure",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal
};
/* Definitions for QueueMonitoring */
osMessageQueueId_t QueueMonitoringHandle;
const osMessageQueueAttr_t QueueMonitoring_attributes = {
  .name = "QueueMonitoring"
};
/* USER CODE BEGIN PV */


osMessageQueueId_t QueueMessageCalibrationHandle;
const osMessageQueueAttr_t QueueMessageCalibration_attributes = {
  .name = "QueueMessageCalibration"
};

osMessageQueueId_t QueueMessagePositionStateHandle;
const osMessageQueueAttr_t QueueMessagePositionState_attributes = {
  .name = "QueueMessagePosition"
};

std::map<SPI_TypeDef*,bool*> TransmissionFlagsSPI;
std::map<ADC_TypeDef*,DeviceSignalInput<1>::StateSwitcher*> SignalReadStates;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void RunTaskMonitoring(void *argument);
void RunTaskControl(void *argument);
void RunTaskMeasure(void *argument);

/* USER CODE BEGIN PFP */

uint8_t ID1 = TypeRegister<MessageAimingDual>::RegisterType();
uint8_t ID2 = TypeRegister<MessageCalibration>::RegisterType();
uint8_t ID3 = TypeRegister<MessageCommand>::RegisterType();

uint8_t ID4 = TypeRegister<MessagePositionState>::RegisterType();
uint8_t ID5 = TypeRegister<MessageMeasure1>::RegisterType();

uint8_t ID6 = TypeRegister<MessageCheckConnection>::RegisterType();
uint8_t ID7 = TypeRegister<MessageCloseConnection>::RegisterType();

//=========================================================================================
bool FLAG_MEASURE_THREAD_SUSPEND = false;
bool FLAG_DELAY_TEST = false;
DeviceSignalInput<1> DeviceSignal1(&hadc1,ADC_CHANNEL_1);
DeviceSignalInput<1> DeviceSignal2(&hadc2,ADC_CHANNEL_2);

using InternalDACType = DeviceSignalControl<DAC_HandleTypeDef,uint16_t,1, DAC_INTERNAL>;
using InternalDACDualType = DeviceSignalControl<DAC_HandleTypeDef,uint16_t,2, DAC_INTERNAL>;

using ExternalDACType = DeviceSignalControl<SPI_HandleTypeDef, uint16_t,1,DAC_SPI_8653> ;
using ExternalDACDualType = DeviceSignalControl<SPI_HandleTypeDef, uint16_t,2,DAC_SPI_8653> ;


//=========================================================================================
InternalDACType InternalDACControlChannel1(hdac1,DAC_CHANNEL_1);
InternalDACType InternalDACControlChannel2(hdac1,DAC_CHANNEL_2);

InternalDACDualType InternalDACControl(hdac1);
ExternalDACDualType ExternalDACControl(hspi3, std::make_pair(SPI3_CS_GPIO_Port, SPI3_CS_Pin));

AimingControlClass<ExternalDACDualType>  ControlAiming{&ExternalDACControl}; 

                             AimingFixedStepClass<2> AimingInterpolation;
      AimingDinamicClass<float,AimingType::DIRECT,2> AimingProlong;
AimingDinamicClass<float,AimingType::PID_VELOCITY,2> AimingLoopPID;

//=========================================================================================
PassCoordTransform<float,uint16_t,SystemScaleSettings::ConvertPix_DAC> PixToDAC;
MessageInternalMonitoring MessageMonitoringStatic;
//=========================================================================================

UDPConnectionInterface ConnectionControl;

void PrintStartMessages()
{
  uint16_t SysFreq = HAL_RCC_GetSysClockFreq()/1000000;

  eprintf(" [ SCANATOR CONTROL ] \r\n");       HAL_Delay(100);
  eprintf(" [ SYS FREQ:  %d ] \r\n", SysFreq); HAL_Delay(100);
  eprintf(" [ ADC CHANNELS:  %d %d] \r\n", ADC_CHANNEL_1, ADC_CHANNEL_2); HAL_Delay(100);
  eprintf(" [ ADC CHANNELS:  %d %d] \r\n", DeviceSignal1.DeviceChannel, DeviceSignal2.DeviceChannel); HAL_Delay(100);
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint64_t MicrosecondsCounter = 0;

void TimerMicrosecondsTickISR() { MicrosecondsCounter++; }

void TimerControlDirectISR()
{

//    ControlSignalDAC.first = RemoteControlCommand.Channel1.Position;
//    ControlSignalDAC.second = RemoteControlCommand.Channel1.Position;
//    //ExternalDACControl.SetCoord(ControlSignal);
//    InternalDACControl.SetCoord(ControlSignalDAC);
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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  PrintStartMessages();

  InternalDACControl.Init();

  DeviceSignal1.Init();
  DeviceSignal2.Init();
  DeviceSignal1.ExposeStateSwitcher(SignalReadStates);
  DeviceSignal2.ExposeStateSwitcher(SignalReadStates);
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
  /* creation of QueueMonitoring */
  QueueMonitoringHandle = osMessageQueueNew (16, sizeof(uint32_t), &QueueMonitoring_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  QueueMessageCalibrationHandle = osMessageQueueNew (4, sizeof(MessageCalibration), &QueueMessageCalibration_attributes);
  QueueMessagePositionStateHandle = osMessageQueueNew (1, sizeof(MessagePositionState), &QueueMessagePositionState_attributes);

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskMonitoring */
  //TaskMonitoringHandle = osThreadNew(RunTaskMonitoring, NULL, &TaskMonitoring_attributes);

  /* creation of TaskControl */
  TaskControlHandle = osThreadNew(RunTaskControl, NULL, &TaskControl_attributes);

  /* creation of TaskMeasure */
  //TaskMeasureHandle = osThreadNew(RunTaskMeasure, NULL, &TaskMeasure_attributes);

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  //sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  htim1.Init.Period = 9;
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
  htim6.Init.Prescaler = 10;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMAMUX_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX_OVR_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMAMUX_OVR_IRQn);

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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   *SignalReadStates[hadc->Instance] = DeviceSignalInput<1>::StateSwitcher::SignalFillEnd;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
   *SignalReadStates[hadc->Instance] = DeviceSignalInput<1>::StateSwitcher::SignalFillTop;
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

/* USER CODE BEGIN Header_RunTaskControl */

void DAC_SinusGenerateTest()
{
  //====================================
  uint16_t DAC_VALUE = 0;
  uint16_t DAC_AMPLITUDE = 2000;
  float ValueCounter = 0;
  uint16_t PERIOD = 360/2;
  std::pair<uint16_t,uint16_t> DACControlSignal;
  uint8_t DelayTime = 1;
  //====================================

  while(true)
  {
    ValueCounter++; if(ValueCounter == PERIOD) ValueCounter = 0;
    DAC_VALUE = DAC_AMPLITUDE + DAC_AMPLITUDE*std::sin(ValueCounter*2*M_PI/PERIOD);

                    DACControlSignal.first = DAC_VALUE;
                    DACControlSignal.second = DAC_VALUE/2;

          //ExternalDACControl.SetValue(DACControlSignal);
          //InternalDACControl.SetCoord(DACControlSignal); osDelay(DelayTime);
          //InternalDACControlChannel1.SetValue(DACControlSignal.first); 
          //InternalDACControlChannel2.SetValue(DACControlSignal.second);
           DACControlSignal.first | InternalDACControlChannel1;
          DACControlSignal.second | InternalDACControlChannel2;

            osDelay(DelayTime);
  }
}

void DAC_SinusGenerateTestStep(int& PhaseCounter, uint16_t& DAC_VALUE)
{
  PhaseCounter++; if(PhaseCounter == 360) PhaseCounter = 0;
  DAC_VALUE = 1500 + 1500*std::sin(4*PhaseCounter*2*M_PI/360);
  DAC_VALUE | InternalDACControlChannel1;
}


void InitMessageProcess(DispatcherType* Dispatcher)
{
	            MessageAimingDual* MessageAiming   = nullptr;
    std::pair<uint16_t,uint16_t> DACControlSignal{0,0};
    Dispatcher->AppendCallback<MessageAimingDual> ( [MessageAiming, DACControlSignal](MessageType& Message) mutable
    {
       MessageAiming = DispatcherType::ExtractData<MessageAimingDual>(&Message);
       DACControlSignal.first  =  MessageAiming->Channel1.Position;
       DACControlSignal.second = -MessageAiming->Channel2.Position;

       DACControlSignal | InternalDACControl;

       //eprintf("[ GET AIMING MESSAGE %d %d] \r\n", (int)MessageAiming->Channel1.Position,
       //                                            (int)MessageAiming->Channel2.Position);
    });


    Dispatcher->AppendCallback<MessageCalibration> ( [](MessageType& Message)
    {
       auto MessageGetCalibration = DispatcherType::ExtractData<MessageCalibration>(&Message);

	     //osMessageQueuePut(QueueMessageCalibrationHandle, MessageGetCalibration, 0, 0);
       //eprintf("[ GET CALIBRATION MESSAGE %x] \r\n", (int)MessageGetCalibration->Command);
       if(MessageGetCalibration->Command == COMMAND_MEASURE_DELAY)
       {
        FLAG_DELAY_TEST = !FLAG_DELAY_TEST;
        eprintf("MEASURE DELAY START TEST %d", FLAG_DELAY_TEST);
       }

       if(MessageGetCalibration->Command == COMMAND_RESPONSE)
       {
        MessageGetCalibration->Position | InternalDACControlChannel2;
       }

       //if(FLAG_MEASURE_THREAD_SUSPEND) 
       //{ 
       // eprintf("MEASURE TASK RESUM \r\n");
       // osThreadResume(TaskMeasureHandle);  FLAG_MEASURE_THREAD_SUSPEND = false; 
       //}
    });

    Dispatcher->AppendCallback<MessageCommand> ( [](MessageType& Message)
    {
       auto MessageSetCommand = DispatcherType::ExtractData<MessageCommand>(&Message);

       eprintf("[ GET COMMAND MESSAGE %d] \r\n", (int)MessageSetCommand->Command);

       //sendData((uint8_t*)(&MessagePosition),MessagePosition.GetSize());
    });


    Dispatcher->AppendCallback<MessageCheckConnection> ( [](MessageType& Message)
    {
       auto MessageCheck = DispatcherType::ExtractData<MessageCheckConnection>(&Message);
       eprintf("[ CHECK CONNECTION MESSAGE ] \r\n");
    });

    Dispatcher->AppendCallback<MessageCloseConnection> ( [](MessageType& Message)
    {
       auto MessageClose = DispatcherType::ExtractData<MessageCloseConnection>(&Message);
       eprintf("[ CLOSE CONNECTION MESSAGE ] \r\n");
    });
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

  eprintf("[START CONTROl TASK] \r\n"); osDelay(2);
  //HAL_TIM_Base_Start_IT(&htim1);
  //HAL_TIM_Base_Start_IT(&htim6);

  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID1, TypeRegister<MessageAimingDual>::GetTypeSize()); osDelay(10);
  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID2, TypeRegister<MessageCalibration>::GetTypeSize()); osDelay(10);
  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID3, TypeRegister<MessageCommand>::GetTypeSize()); osDelay(10);
  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID4, TypeRegister<MessagePositionState>::GetTypeSize()); osDelay(10);
  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID5, TypeRegister<MessageMeasure1>::GetTypeSize()); osDelay(10);
  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID6, TypeRegister<MessageCheckConnection>::GetTypeSize()); osDelay(10);

         MessagePositionState DevicePosition;
  MessageInternalMonitoring MessageMonitoring;

                            ConnectionControl.Init();
             DispatcherType Dispatcher; InitMessageProcess(&Dispatcher);
  //DAC_SinusGenerateTest();
  int PhaseCounter = 0;
  uint16_t DacValue = 0;
  MessageGeneric<MessagePositionState, HEADER_TYPE> MessagePosition;
  //FLAG_DELAY_TEST = true;

  while(true)
  {
	  ConnectionControl.PerformTransmission();
	 *ConnectionControl.RingBufferMessages | Dispatcher;

   if(FLAG_DELAY_TEST)
   {
   DAC_SinusGenerateTestStep(PhaseCounter, DacValue);
   MessagePosition.DATA.Position1 = DacValue;
   ConnectionControl.sendCommand((uint8_t*)&MessagePosition, sizeof(MessageGeneric<MessagePositionState, HEADER_TYPE>));
   }

   osDelay(1);
	 //MessageMonitoring.Param1 = AimingControlCommand.Channel1.Position;
	 //osMessageQueuePut(QueueMonitoringHandle, &MessageMonitoring, 0, 0);

   //if(osMessageQueueGetCount(QueueMessagePositionStateHandle) == 0 ) continue;
   //         osMessageQueueGet(QueueMessagePositionStateHandle,&DevicePosition,0,0);
   //                               ConnectionControl.sendMessage(DevicePosition);
  }

  /* USER CODE END RunTaskControl */
}

/* USER CODE BEGIN Header_RunTaskMeasure */
/**
* @brief Function implementing the TaskMeasure thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RunTaskMeasure */
void RunTaskMeasure(void *argument)
{
  /* USER CODE BEGIN RunTaskMeasure */
  eprintf("[RUN TASK MEASURE]");

  //DeviceCalibrationClass<DeviceSignalInput<1>, ExternalDACType, InternalDACType>
  //                                             ModuleCalibration{ &DeviceSignal1, 
  //                                              &ExternalDACControl.Channel1,
  //                                              &InternalDACControlChanne1
  //                                            };

  DeviceCalibrationClass<DeviceSignalInput<1>, InternalDACType, InternalDACType>
                                               ModuleCalibration{ &DeviceSignal1, 
                                                &InternalDACControlChannel1,
                                                &InternalDACControlChannel2
                                              };

  MessageCalibration CommandCalibration;

  FLAG_MEASURE_THREAD_SUSPEND = true; osThreadSuspend(TaskMeasureHandle);
  for(;;)
  {
      if(osMessageQueueGetCount(QueueMessageCalibrationHandle) != 0)
      {
	     osMessageQueueGet(QueueMessageCalibrationHandle, &CommandCalibration, 0, 0);
	     CommandCalibration >> ModuleCalibration;
      }

      if(ModuleCalibration.STATE_IDLE) 
      {
       eprintf("MEASURE TASK SUSPEND \r\n");
       FLAG_MEASURE_THREAD_SUSPEND = true;
       osThreadSuspend(TaskMeasureHandle);
      }
      osDelay(1);
       
  }

  /* USER CODE END RunTaskMeasure */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
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
