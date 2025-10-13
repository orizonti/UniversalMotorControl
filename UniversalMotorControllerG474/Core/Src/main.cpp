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
#include <array>

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

FDCAN_HandleTypeDef hfdcan1;

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
  .priority = (osPriority_t) osPriorityLow7
};
/* Definitions for TaskControl */
osThreadId_t TaskControlHandle;
const osThreadAttr_t TaskControl_attributes = {
  .name = "TaskControl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh
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

std::map<SPI_TypeDef*,bool*> FlagSwitchersSPI;
std::map<ADC_TypeDef*,DeviceTypeADC<1>::StateSwitcher*> StateSwitchersADC;

std::map<FDCAN_HandleTypeDef*,bool*> FlagSwitrchersCAN;

//===============================
//FDCAN TEST
//FDCAN_TxHeaderTypeDef   CanTxHeader;  //TO_DELTE
//FDCAN_RxHeaderTypeDef   CanRxHeader;  //TO_DELTE
//uint8_t               CanTxData[64];  //TO_DELTE
//uint8_t               CanRxData[64];  //TO_DELTE

//==========================================================================================
class CanConnectionInterface
{
public:
	explicit CanConnectionInterface(FDCAN_HandleTypeDef* CanDevice)
	{
		Device = CanDevice;
		initTransmit();
	}
FDCAN_TxHeaderTypeDef   CanTxHeader;
FDCAN_RxHeaderTypeDef   CanRxHeader;
uint8_t               CanTxData[8];
uint8_t               CanRxData[8];
FDCAN_HandleTypeDef* Device = nullptr;
bool FLAG_CAN_DATA_AVAILABLE = false;

int Result = 0;
CanConnectionInterface* connectionLinked = nullptr;
void linkModule(CanConnectionInterface* connection) { connectionLinked = connection; }

void initTransmit()
{
  CanTxHeader.Identifier = 0x1;
  CanTxHeader.IdType      = FDCAN_STANDARD_ID;
  CanTxHeader.TxFrameType = FDCAN_DATA_FRAME;
  CanTxHeader.DataLength  = FDCAN_DLC_BYTES_8;
  CanTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  CanTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  CanTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  CanTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  CanTxHeader.MessageMarker = 0;
}

std::array<uint32_t,8> DLC_VECTOR = {FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};
void sendData(uint8_t ID, uint8_t Size, uint8_t* Data)
{
  CanTxHeader.Identifier = ID;
  CanTxHeader.DataLength = DLC_VECTOR[Size-1];
  CanTxHeader.MessageMarker = 0;

  std::copy_n(Data, Size, CanTxData);
  Result = HAL_FDCAN_AddMessageToTxFifoQ(Device, &CanTxHeader, CanTxData);
  //Result = HAL_FDCAN_AddMessageToTxFifoQ(Device, &CanTxHeader, Data);
}

void receiveData()
{
	if(!FLAG_CAN_DATA_AVAILABLE) return; FLAG_CAN_DATA_AVAILABLE = false;
	HAL_FDCAN_GetRxMessage(Device, FDCAN_RX_FIFO0, &CanRxHeader, CanRxData);

	if(CanRxHeader.Identifier == TypeRegister<CommandSetPos<0>>::GetTypeID())
      CommandDispatcher<CommandSetPos<0>>::processDispatch((CommandSetPos<0>*)CanRxData);

	if(CanRxHeader.Identifier == TypeRegister<CommandSetPos<1>>::GetTypeID())
      CommandDispatcher<CommandSetPos<1>>::processDispatch((CommandSetPos<1>*)CanRxData);

	if(CanRxHeader.Identifier == TypeRegister<CommandDevice<0>>::GetTypeID())
      CommandDispatcher<CommandDevice<0>>::processDispatch((CommandDevice<0>*)CanRxData);

	if(CanRxHeader.Identifier == TypeRegister<CommandSetPosRotary>::GetTypeID())
      CommandDispatcher<CommandCheckConnection>::processDispatch((CommandCheckConnection*)CanRxData);
}

void transmitData()
{
	if(!FLAG_CAN_DATA_AVAILABLE) return; FLAG_CAN_DATA_AVAILABLE = false;

	HAL_FDCAN_GetRxMessage(Device, FDCAN_RX_FIFO0, &CanRxHeader, CanRxData);

	if(connectionLinked != nullptr) connectionLinked->sendData(CanRxHeader.Identifier, CanRxHeader.DataLength, CanRxData);
}


void exposeFlagTransmission(std::map<FDCAN_HandleTypeDef*,bool*>& Flags)
{
	Flags[Device] = &FLAG_CAN_DATA_AVAILABLE;
}

};

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
   *FlagSwitrchersCAN[hfdcan] = true;
  }
}

//void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
//{
//  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
//  {
//    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) ;
//  }
//}

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
static void MX_FDCAN1_Init(void);
void RunTaskMonitoring(void *argument);
void RunTaskControl(void *argument);
void RunTaskMeasure(void *argument);

/* USER CODE BEGIN PFP */

//=========================================================================================
uint8_t ID0 = TypeRegister<CommandSetPosScanator>::RegisterType();
uint8_t ID1 = TypeRegister<CommandSetPosRotary>::RegisterType();
uint8_t ID2 = TypeRegister<CommandDeviceController>::RegisterType();
uint8_t ID3 = TypeRegister<CommandDeviceLaserPower>::RegisterType();
uint8_t ID4 = TypeRegister<CommandDeviceLaserPointer>::RegisterType();
uint8_t ID5 = TypeRegister<CommandDeviceFocusator>::RegisterType();

uint8_t ID2_0 = TypeRegister<MessageDeviceController>::RegisterType();
uint8_t ID2_1 = TypeRegister<MessageDeviceLaserPower>::RegisterType();
uint8_t ID2_2 = TypeRegister<MessageDeviceLaserPointer>::RegisterType();
uint8_t ID2_3 = TypeRegister<MessageDeviceFocusator>::RegisterType();

uint8_t ID3_0 = TypeRegister<CommandCalibration>::RegisterType();
uint8_t ID3_1 = TypeRegister<MessagePositionState>::RegisterType();
uint8_t ID3_2 = TypeRegister<CommandCheckConnection>::RegisterType();
uint8_t ID3_3 = TypeRegister<CommandCloseConnection>::RegisterType();

uint8_t ID3_4 = TypeRegister<MessageMeasure1>::RegisterType();

void printfRegisteredTypes()
{
  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<CommandSetPos<0>>::GetTypeID(), TypeRegister<CommandSetPosScanator>::GetTypeSize());
  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<CommandSetPos<1>>::GetTypeID(), TypeRegister<CommandSetPosRotary>::GetTypeSize());

  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<CommandDevice<0>>::GetTypeID(), TypeRegister<CommandDevice<0>>::GetTypeSize());
  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<CommandDevice<1>>::GetTypeID(), TypeRegister<CommandDevice<1>>::GetTypeSize());
  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<CommandDevice<2>>::GetTypeID(), TypeRegister<CommandDevice<2>>::GetTypeSize());
  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<CommandDevice<3>>::GetTypeID(), TypeRegister<CommandDevice<3>>::GetTypeSize());

  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<MessageDevice<0>>::GetTypeID(), TypeRegister<MessageDevice<0>>::GetTypeSize());
  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<MessageDevice<1>>::GetTypeID(), TypeRegister<MessageDevice<1>>::GetTypeSize());
  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<MessageDevice<2>>::GetTypeID(), TypeRegister<MessageDevice<2>>::GetTypeSize());
  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<MessageDevice<3>>::GetTypeID(), TypeRegister<MessageDevice<3>>::GetTypeSize());

  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<CommandCalibration>::GetTypeID(), TypeRegister<CommandCalibration>::GetTypeSize());
  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<MessagePositionState>::GetTypeID(), TypeRegister<MessagePositionState>::GetTypeSize());
  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<CommandCheckConnection>::GetTypeID(), TypeRegister<CommandCheckConnection>::GetTypeSize());

  eprintf("REGISTER TYPE %d SIZE %d \r\n", TypeRegister<MessageMeasure1>::GetTypeID(), TypeRegister<MessageMeasure1>::GetTypeSize());
}

void setPosDevice1(CommandSetPos<0>* command)
{
  int POS_X = command->POS_X;
  int POS_Y = command->POS_Y;
 eprintf("[GET COMMAND SCANATOR %d %d] \r\n", POS_X, POS_Y);
};
void setPosDevice2(CommandSetPos<1>* command)
{

  int POS_X = command->POS_X;
  int POS_Y = command->POS_Y;
 eprintf("[GET COMMAND ROTARY %d %d] \r\n", POS_X, POS_Y);
};

void setCommandDevice1(CommandDevice<0>* command)
{
 eprintf("[GET COMMAND CONTROLLER] \r\n");
};

void setCommandDevice2(CommandCheckConnection* command)
{
 eprintf("[CHECK CONNECTION \r\n]");
};

void initProcessingCommand()
{
	CommandDispatcher<CommandSetPos<0>>::setCallBack(setPosDevice1);
	CommandDispatcher<CommandSetPos<1>>::setCallBack(setPosDevice2);

	CommandDispatcher<CommandDevice<0>>::setCallBack(setCommandDevice1);
	CommandDispatcher<CommandCheckConnection>::setCallBack(setCommandDevice2);
}
//=========================================================================================

bool FLAG_MEASURE_THREAD_SUSPEND = false;

using DeviceTypeDACInternal     = DeviceDACControl<DAC_HandleTypeDef, uint16_t,1, DAC_INTERNAL>;
using DeviceTypeDACInternalDual = DeviceDACControl<DAC_HandleTypeDef, uint16_t,2, DAC_INTERNAL>;

using DeviceTypeDACExternal     = DeviceDACControl<SPI_HandleTypeDef, uint16_t,1, DAC_SPI_8653> ;
using DeviceTypeDACExternalDual = DeviceDACControl<SPI_HandleTypeDef, uint16_t,2, DAC_SPI_8653> ;

//=========================================================================================
MessageInternalMonitoring MessageMonitoringStatic;
//=========================================================================================
DeviceTypeADC<1> DeviceADC1{&hadc1,ADC_CHANNEL_1};
DeviceTypeADC<1> DeviceADC2{&hadc2,ADC_CHANNEL_2};

DeviceTypeDACInternal DeviceDACInternalChannel1{hdac1,DAC_CHANNEL_1};
DeviceTypeDACInternal DeviceDACInternalChannel2{hdac1,DAC_CHANNEL_2};

DeviceTypeDACInternalDual DeviceDACInternal{hdac1};
DeviceTypeDACExternalDual DeviceDACExternal{hspi3, std::make_pair(SPI3_CS_GPIO_Port, SPI3_CS_Pin)};

DeviceTypeConnectionUDP ConnectionRemote;
DeviceTypeScanator<DeviceTypeADC<1>, DeviceTypeDACInternalDual>  
                        DeviceScanator{&DeviceADC1,&DeviceADC2,&DeviceDACInternal};
//=========================================================================================

ModuleTypeDelayMeasure<DeviceTypeConnectionUDP, DeviceTypeDACInternal, DeviceTypeDACInternal>
                                              ModuleDelayMeasureConnection
                                              { &ConnectionRemote,         //LOOP

                                                &DeviceDACInternalChannel1, //LOOP  MONITOR
                                                &DeviceDACInternalChannel2, //DELAY MONITOR
                                              };

ModuleTypeDelayMeasure<DeviceTypeDACExternal, DeviceTypeDACInternal, DeviceTypeDACInternal>
                                              ModuleDelayMeasureControlLoop
                                              { &DeviceDACExternal.Channel1, //LOOP

                                                 &DeviceDACInternalChannel1, //LOOP  MONITOR
                                                 &DeviceDACInternalChannel2, //DELAY MONITOR
                                              };

ModuleTypeResponceMeasure<DeviceTypeADC<1>, DeviceTypeDACInternal>
                                              ModuleResponceMeasure
                                              { &DeviceADC1, 
                                                &DeviceDACInternalChannel1,
                                              };
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
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  PrintStartMessages();

  DeviceDACInternal.Init();

  DeviceADC1.Init();
  DeviceADC2.Init();
  DeviceADC1.ExposeStateSwitcher(StateSwitchersADC);
  DeviceADC2.ExposeStateSwitcher(StateSwitchersADC);
  //DeviceDACExternal.Init();

  //======================================================================================================
  //FDCAN INIT BEGIN
  if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();
  if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) Error_Handler();

  //======================================================================================================

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
  QueueMessageCalibrationHandle = osMessageQueueNew (4, sizeof(CommandCalibration), &QueueMessageCalibration_attributes);
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
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
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 26;
  hfdcan1.Init.NominalTimeSeg1 = 143;
  hfdcan1.Init.NominalTimeSeg2 = 26;
  hfdcan1.Init.DataPrescaler = 10;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 12;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x32;
  sFilterConfig.FilterID2 = 0x32;
      eprintf("CAN FILTER CONFIG %d \r\n", 0x32);
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
      eprintf("CAN FILTER CONFIG ERROR %d \r\n", hfdcan1.ErrorCode);
	  Error_Handler();
  }

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x33;
  sFilterConfig.FilterID2 = 0x33;
      eprintf("CAN FILTER CONFIG %d \r\n", 0x33);
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
      eprintf("CAN FILTER CONFIG ERROR %d \r\n", hfdcan1.ErrorCode);
	  Error_Handler();
  }
  /* USER CODE END FDCAN1_Init 2 */

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
  if(FlagSwitchersSPI.empty()) return;

  *FlagSwitchersSPI[hspi->Instance] = true;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   *StateSwitchersADC[hadc->Instance] = DeviceTypeADC<1>::StateSwitcher::SignalFillEnd;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
   *StateSwitchersADC[hadc->Instance] = DeviceTypeADC<1>::StateSwitcher::SignalFillTop;
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
  for(;;)
  {

	//if(FLAG_CAN_DATA_AVAILABLE)
	//{
    //   FLAG_CAN_DATA_AVAILABLE = false;

    //HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &CanRxHeader, CanRxData);
    //eprintf("[ GET MESSAGE %d : %d %d %d %d] \r\n", CanRxHeader.Identifier, CanRxData[0], CanRxData[1], CanRxData[2], CanRxData[3]);
	//}
    osDelay(1);

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

void DAC_SinusGenerateTest()
{
  //====================================
  uint16_t DAC_VALUE = 0;
  uint16_t DAC_AMPLITUDE = 2000;
  float ValueCounter = 0;
  uint16_t PERIOD = 360/2;
  std::pair<uint16_t,uint16_t> DeviceDACSignal;
  uint8_t DelayTime = 1;
  //====================================

  while(true)
  {
    ValueCounter++; if(ValueCounter == PERIOD) ValueCounter = 0;
    DAC_VALUE = DAC_AMPLITUDE + DAC_AMPLITUDE*std::sin(ValueCounter*2*M_PI/PERIOD);

                    DeviceDACSignal.first = DAC_VALUE;
                    DeviceDACSignal.second = DAC_VALUE/2;

           DeviceDACSignal | DeviceDACInternal;
          //DeviceDACExternal.SetValue(DeviceDACSignal);
          //DeviceDACInternal.SetCoord(DeviceDACSignal); osDelay(DelayTime);
          //DeviceDACInternalChannel1.SetValue(DeviceDACSignal.first); 
          //DeviceDACInternalChannel2.SetValue(DeviceDACSignal.second);
          //DeviceDACSignal.first | DeviceDACInternalChannel1;
          //DeviceDACSignal.second | DeviceDACInternalChannel2;

            osDelay(DelayTime);
  }
}

void DAC_SinusGenerateTestStep(int& PhaseCounter, uint16_t& DAC_VALUE)
{
  PhaseCounter++; if(PhaseCounter == 360) PhaseCounter = 0;
  DAC_VALUE = 1500 + 1500*std::sin(4*PhaseCounter*2*M_PI/360);
  DAC_VALUE | DeviceDACInternalChannel1;
}

void InitMessageProcessing(DispatcherType* Dispatcher)
{
   CommandSetPosScanator* CommandAiming   = nullptr;
    MessagePositionState* MessagePosition = nullptr;

    std::pair<uint16_t,uint16_t> ControlOutputSignal;

    Dispatcher->AppendCallback<CommandSetPosScanator> ( [CommandAiming, ControlOutputSignal](MessageType& Message) mutable
    {
       CommandAiming = DispatcherType::ExtractData<CommandSetPosScanator>(&Message);
       ControlOutputSignal.first  =  CommandAiming->POS_X;
       ControlOutputSignal.second = -CommandAiming->POS_Y;
       ControlOutputSignal | DeviceDACInternal;
         
       //*MessageAiming | DeviceScanator;

       //eprintf("[ GET AIMING MESSAGE %d %d] \r\n", (int)MessageAiming->Channel1.Position,
       //                                            (int)MessageAiming->Channel2.Position);
    });

    Dispatcher->AppendCallback<MessagePositionState> ( [MessagePosition](MessageType& Message) mutable
    {
        MessagePosition = DispatcherType::ExtractData<MessagePositionState>(&Message);
       *MessagePosition | ModuleDelayMeasureConnection;
    });


    Dispatcher->AppendCallback<CommandCalibration> ( [](MessageType& Message)
    {
       auto Command= DispatcherType::ExtractData<CommandCalibration>(&Message);

       if(Command->NodeType == MODULE_TYPE_DELAY_MEASURE) *Command | ModuleDelayMeasureConnection;

       if(Command->NodeType == MODULE_TYPE_RESPONCE_MEASURE)
	     osMessageQueuePut(QueueMessageCalibrationHandle, Command, 0, 0);

       if(Command->NodeType == MODULE_TYPE_RESPONCE_MEASURE  && FLAG_MEASURE_THREAD_SUSPEND)
       { 
        osThreadResume(TaskMeasureHandle);  FLAG_MEASURE_THREAD_SUSPEND = false; eprintf("MEASURE TASK RESUME \r\n");
       }
    });

    Dispatcher->AppendCallback<CommandDevice<0>> ( [](MessageType& Message)
    {
       auto Command = DispatcherType::ExtractData<CommandDeviceController>(&Message);

         if(Command->DeviceID == DEVICE_TYPE_SCANATOR) *Command | DeviceScanator;

    });


    Dispatcher->AppendCallback<CommandCheckConnection> ( [](MessageType& Message)
    {
       auto MessageCheck = DispatcherType::ExtractData<CommandCheckConnection>(&Message);
       eprintf("[ CHECK CONNECTION MESSAGE ] \r\n");
    });

    Dispatcher->AppendCallback<CommandCloseConnection> ( [](MessageType& Message)
    {
       auto MessageClose = DispatcherType::ExtractData<CommandCloseConnection>(&Message);
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
  printfRegisteredTypes();
  initProcessingCommand();
  CanConnectionInterface connectionCan(&hfdcan1);
  CommandCheckConnection commandCheck;

  connectionCan.exposeFlagTransmission(FlagSwitrchersCAN);

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

  //DAC_SinusGenerateTest();
    //ADCTest();


  eprintf("[ CAN ECHO WORK ]");
  while(true)
  {
//	  ConnectionRemote.GetIncommingMessages();
//	 *ConnectionRemote.RingBufferMessages | Dispatcher;
//    ConnectionRemote.SendPendingMessage();

    //void sendData(uint8_t ID, uint8_t Size, uint8_t* Data)
	//connectionCan.sendData(TypeRegister<CommandCheckConnection>::GetTypeID(), sizeof(CommandCheckConnection),(uint8_t*)(&commandCheck));
      connectionCan.receiveData();

    osDelay(1000);
    //ModuleDelayMeasureConnection.MeasureClosedLoopDelay();
    //ModuleDelayMeasureControlLoop.MeasureClosedLoopDelay();
    //osDelay(1);
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

  CommandCalibration Command;

  FLAG_MEASURE_THREAD_SUSPEND = true; osThreadSuspend(TaskMeasureHandle);
  for(;;)
  {
      if(osMessageQueueGetCount(QueueMessageCalibrationHandle) != 0)
      {
	     osMessageQueueGet(QueueMessageCalibrationHandle, &Command, 0, 0);
	     Command >> ModuleResponceMeasure;
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
#ifdef USE_FULL_ASSERT
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
