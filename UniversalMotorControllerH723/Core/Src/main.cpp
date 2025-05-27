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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac1;

OSPI_HandleTypeDef hospi1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Definitions for TaskMonitoring */
osThreadId_t TaskMonitoringHandle;
const osThreadAttr_t TaskMonitoring_attributes = {
  .name = "TaskMonitoring",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for TaskControl */
osThreadId_t TaskControlHandle;
const osThreadAttr_t TaskControl_attributes = {
  .name = "TaskControl",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskMeasure */
osThreadId_t TaskMeasureHandle;
const osThreadAttr_t TaskMeasure_attributes = {
  .name = "TaskMeasure",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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

std::map<SPI_TypeDef*,bool*> FlagSwitchersSPI;
std::map<ADC_TypeDef*,DeviceTypeADC<1>::StateSwitcher*> StateSwitchersADC;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void RunTaskMonitoring(void *argument);
void RunTaskControl(void *argument);
void RunTaskMeasure(void *argument);

/* USER CODE BEGIN PFP */

//=========================================================================================
uint8_t ID1 = TypeRegister<MessageAimingDual>::RegisterType();
uint8_t ID2 = TypeRegister<MessageCalibration>::RegisterType();
uint8_t ID3 = TypeRegister<MessageCommand>::RegisterType();

uint8_t ID4 = TypeRegister<MessagePositionState>::RegisterType();
uint8_t ID5 = TypeRegister<MessageMeasure1>::RegisterType();

uint8_t ID6 = TypeRegister<MessageCheckConnection>::RegisterType();
uint8_t ID7 = TypeRegister<MessageCloseConnection>::RegisterType();

//=========================================================================================
bool FLAG_MEASURE_THREAD_SUSPEND = false;

using DeviceTypeDACInternal     = DeviceDACControl<DAC_HandleTypeDef, uint16_t,1, DAC_INTERNAL>;
using DeviceTypeDACInternalDual = DeviceDACControl<DAC_HandleTypeDef, uint16_t,2, DAC_INTERNAL>;

using DeviceTypeDACExternal     = DeviceDACControl<SPI_HandleTypeDef, int16_t,1, DAC_SPI_8550> ;
using DeviceTypeDACExternalDual = DeviceDACControl<SPI_HandleTypeDef, int16_t,2, DAC_SPI_8550> ;

//=========================================================================================
MessageInternalMonitoring MessageMonitoringStatic;
//=========================================================================================
DeviceTypeADC<1> DeviceADC1{&hadc1,ADC_CHANNEL_4};
DeviceTypeADC<1> DeviceADC2{&hadc2,ADC_CHANNEL_5};
//================================
//TO DELETE
bool FLAG_ADC1_END = true;
bool FLAG_ADC2_END = true;
uint16_t ADC1_BUFFER[1000];
uint16_t ADC2_BUFFER[1000];
uint16_t ADCMeasure1;
uint16_t ADCMeasure2;
uint16_t CounterMeasure = 0;;
int TimerCount = 0;

bool FLAG_CONTINOUS_MODE = false;
bool FLAG_SINGLE_SERIES_MODE = true;
bool FLAG_SINGLE_MODE = false;
//================================


DeviceTypeDACInternal DeviceDACInternalChannel1{hdac1,DAC_CHANNEL_1};
DeviceTypeDACInternal DeviceDACInternalChannel2{hdac1,DAC_CHANNEL_2};

DeviceTypeDACInternalDual DeviceDACInternal{hdac1};
//DeviceTypeDACExternalDual DeviceDACExternal{hspi3, std::make_pair(SPI3_CS_GPIO_Port, SPI3_CS_Pin)};
DeviceTypeDACExternalDual DeviceDACExternal{hspi3,std::make_pair(SPI3_CS_GPIO_Port, SPI3_CS_Pin),
	                                        hspi4,std::make_pair(SPI4_CS_GPIO_Port, SPI4_CS_Pin)};


DeviceTypeConnectionUDP ConnectionRemote;
DeviceTypeScanator<DeviceTypeADC<1>, DeviceTypeDACExternalDual>
                        DeviceScanator{&DeviceADC1,&DeviceADC2,&DeviceDACExternal};
//=========================================================================================

ModuleTypeDelayMeasure<DeviceTypeConnectionUDP, DeviceTypeDACInternal, DeviceTypeDACInternal>
                                              ModuleDelayMeasureConnection
                                              { &ConnectionRemote,         //LOOP

                                                &DeviceDACInternalChannel1, //LOOP  MONITOR
                                                &DeviceDACInternalChannel2, //DELAY MONITOR
                                              };

//ModuleTypeDelayMeasure<DeviceTypeDACExternal, DeviceTypeDACInternal, DeviceTypeDACInternal>
//                                              ModuleDelayMeasureControlLoop
//                                              { &DeviceDACExternal.Channel1, //LOOP
//
//                                                 &DeviceDACInternalChannel1, //LOOP  MONITOR
//                                                 &DeviceDACInternalChannel2, //DELAY MONITOR
//                                              };
//
ModuleTypeResponceMeasure<DeviceTypeADC<1>, DeviceTypeDACExternal>
                                              ModuleResponceMeasure
                                              { &DeviceADC1,
                                                &DeviceDACExternal.Channel2,
                                              };

//ModuleTypeResponceMeasure<DeviceTypeADC<1>, DeviceTypeDACInternal>
//                                              ModuleResponceMeasure
//                                              { &DeviceADC1,
//                                                &DeviceDACInternalChannel1,
//                                              };
//=========================================================================

void PrintStartMessages()
{
  uint16_t SysFreq = HAL_RCC_GetSysClockFreq()/1000000;

  eprintf(" [ SCANATOR CONTROL ] \r\n");       HAL_Delay(100);
  eprintf(" [ SYS FREQ:  %d ] \r\n", SysFreq); HAL_Delay(100);
  eprintf(" [ ADC CHANNELS:  %d %d] \r\n", ADC_CHANNEL_1, ADC_CHANNEL_2); HAL_Delay(100);
  eprintf(" [ ADC CHANNELS:  %d %d] \r\n", DeviceADC1.DeviceChannel, DeviceADC2.DeviceChannel); HAL_Delay(100);
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint64_t MicrosecondsCounter = 0;

void TimerMicrosecondsTickISR() { MicrosecondsCounter++; }

void TimerControlDirectISR()
{
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  //HAL_Delay(100);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_OCTOSPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  PrintStartMessages();

  DeviceDACInternal.Init();
  DeviceADC1.Init();
  DeviceADC2.Init();
  DeviceADC1.ExposeStateSwitcher(StateSwitchersADC);
  DeviceADC2.ExposeStateSwitcher(StateSwitchersADC);

  DeviceDACExternal.RegisterFlag(FlagSwitchersSPI);

  decltype(DeviceDACExternal.OutputSignal) Signal{0,0};
  Signal | DeviceDACExternal;
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
  TaskMonitoringHandle = osThreadNew(RunTaskMonitoring, NULL, &TaskMonitoring_attributes);

  /* creation of TaskControl */
  TaskControlHandle = osThreadNew(RunTaskControl, NULL, &TaskControl_attributes);

  /* creation of TaskMeasure */
  TaskMeasureHandle = osThreadNew(RunTaskMeasure, NULL, &TaskMeasure_attributes);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI3
                              |RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.PLL2.PLL2M = 32;
  PeriphClkInitStruct.PLL2.PLL2N = 129;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 32;
  PeriphClkInitStruct.PLL3.PLL3N = 129;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
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
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
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
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
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
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
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
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
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
  htim6.Init.Prescaler = 550;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, W5500_Reset_Pin|SPI2_CS_Pin|SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : W5500_Reset_Pin SPI2_CS_Pin SPI3_CS_Pin */
  GPIO_InitStruct.Pin = W5500_Reset_Pin|SPI2_CS_Pin|SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI4_CS_Pin */
  GPIO_InitStruct.Pin = SPI4_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI4_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if(FlagSwitchersSPI.empty()) return;

  *FlagSwitchersSPI[hspi->Instance] = true;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   *StateSwitchersADC[hadc->Instance] = DeviceTypeADC<1>::StateSwitcher::SignalFillEnd;

   //if(hadc->Instance == ADC1) FLAG_ADC1_END = true;
   //if(hadc->Instance == ADC2) FLAG_ADC2_END = true;

   FLAG_ADC1_END = true;
   FLAG_ADC2_END = true;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
   *StateSwitchersADC[hadc->Instance] = DeviceTypeADC<1>::StateSwitcher::SignalFillTop;

   //if(hadc->Instance == ADC1) FLAG_ADC1_END = false;
   //if(hadc->Instance == ADC2) FLAG_ADC2_END = false;
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
void ADCTest()
{
  uint16_t Value1 = 0;
  uint16_t Value2 = 0;
  DeviceADC1.SetModeReady();
  DeviceADC2.SetModeReady();

  while(true)
  {
    Value1 = DeviceADC1.GetValue();
    Value2 = DeviceADC2.GetValue();
    eprintf("[ ADC TEST ] %d  %d  \r\n", Value1, Value2);
    osDelay(500);
  }

};


void InitMessageProcessing(DispatcherType* Dispatcher)
{
       MessageAimingDual* MessageAiming   = nullptr;
    MessagePositionState* MessagePosition = nullptr;

    std::pair<uint16_t,uint16_t> ControlOutputSignal;

    Dispatcher->AppendCallback<MessageAimingDual> ( [MessageAiming, ControlOutputSignal](MessageType& Message) mutable
    {
       MessageAiming = DispatcherType::ExtractData<MessageAimingDual>(&Message);
       ControlOutputSignal.first  =  MessageAiming->Channel1.Position;
       ControlOutputSignal.second = -MessageAiming->Channel2.Position;
       ControlOutputSignal | DeviceDACInternal;
         
       *MessageAiming | DeviceScanator;

       //eprintf("[ GET AIMING MESSAGE %d %d] \r\n", (int)MessageAiming->Channel1.Position,
       //                                            (int)MessageAiming->Channel2.Position);
    });

    Dispatcher->AppendCallback<MessagePositionState> ( [MessagePosition](MessageType& Message) mutable
    {
       
       MessagePosition = DispatcherType::ExtractData<MessagePositionState>(&Message);

       *MessagePosition | ModuleDelayMeasureConnection;
    });


    Dispatcher->AppendCallback<MessageCalibration> ( [](MessageType& Message)
    {
       auto CommandCalibration = DispatcherType::ExtractData<MessageCalibration>(&Message);

       if(CommandCalibration->NodeType == MODULE_TYPE_DELAY_MEASURE) *CommandCalibration | ModuleDelayMeasureConnection;

       if(CommandCalibration->NodeType == MODULE_TYPE_RESPONCE_MEASURE)
	     osMessageQueuePut(QueueMessageCalibrationHandle, CommandCalibration, 0, 0);

       if(CommandCalibration->NodeType == MODULE_TYPE_RESPONCE_MEASURE  && FLAG_MEASURE_THREAD_SUSPEND) 
       { 
        osThreadResume(TaskMeasureHandle);  FLAG_MEASURE_THREAD_SUSPEND = false; eprintf("MEASURE TASK RESUME \r\n");
       }
    });

    Dispatcher->AppendCallback<MessageCommand> ( [](MessageType& Message)
    {
       auto Command = DispatcherType::ExtractData<MessageCommand>(&Message);

         if(Command->NodeType == DEVICE_TYPE_SCANATOR) *Command | DeviceScanator;

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
  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID1, TypeRegister<MessageAimingDual>::GetTypeSize()); osDelay(10);
  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID2, TypeRegister<MessageCalibration>::GetTypeSize()); osDelay(10);
  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID3, TypeRegister<MessageCommand>::GetTypeSize()); osDelay(10);
  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID4, TypeRegister<MessagePositionState>::GetTypeSize()); osDelay(10);
  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID5, TypeRegister<MessageMeasure1>::GetTypeSize()); osDelay(10);
  eprintf("REGISTER MESSAGE TYPE %d SIZE %d \r\n", ID6, TypeRegister<MessageCheckConnection>::GetTypeSize()); osDelay(10);

  //HAL_TIM_Base_Start_IT(&htim1);
  //HAL_TIM_Base_Start_IT(&htim6);
  //===============================================================
       MessagePositionState DevicePosition;
  MessageInternalMonitoring MessageMonitoring;

  DispatcherType Dispatcher; InitMessageProcessing(&Dispatcher);
            ConnectionRemote.InitModule();
  //===============================================================
  MessageGeneric<MessagePositionState, HEADER_TYPE> MessagePosition;

  DeviceADC1.SetModeReady();
  DeviceADC2.SetModeReady();

  //DeviceADC1.SetModeContinous(true);
  //DeviceADC2.SetModeContinous(true);

  //TestSinusDAC_SPI();
    //ADCTest();
  while(true)
  {
	  ConnectionRemote.GetIncommingMessages();
	 *ConnectionRemote.RingBufferMessages | Dispatcher;
    ConnectionRemote.SendPendingMessage();

    //ModuleDelayMeasureConnection.MeasureClosedLoopDelay();
    //ModuleDelayMeasureControlLoop.MeasureClosedLoopDelay();
    osDelay(1);
  }

	//MessageMonitoring.Param1 = AimingControlCommand.Channel1.Position;
	//osMessageQueuePut(QueueMonitoringHandle, &MessageMonitoring, 0, 0);
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
  osDelay(20);
  eprintf("[RUN TASK MEASURE] \r\n");

  ModuleResponceMeasure.InitModule();

  MessageCalibration CommandCalibration;

  FLAG_MEASURE_THREAD_SUSPEND = true; osThreadSuspend(TaskMeasureHandle);
  for(;;)
  {
      if(osMessageQueueGetCount(QueueMessageCalibrationHandle) != 0)
      {
	     osMessageQueueGet(QueueMessageCalibrationHandle, &CommandCalibration, 0, 0);
	     CommandCalibration >> ModuleResponceMeasure;
      }

      if(ModuleResponceMeasure.STATE_IDLE)
      {
       eprintf("MEASURE TASK SUSPEND \r\n");
       FLAG_MEASURE_THREAD_SUSPEND = true;
       osThreadSuspend(TaskMeasureHandle);
      }
      osDelay(1);

  }

  /* USER CODE END RunTaskMeasure */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  if (htim->Instance == TIM1)
  {
    TimerMicrosecondsTickISR();
  }

  if (htim->Instance == TIM6)
  {
    //TimerControlDirectISR();
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
