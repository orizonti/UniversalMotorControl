#include "UtiliteFunctions.h"
#include <stdarg.h>
#include "stm32h7xx_hal.h"

#include "stdio.h"
#include <string.h>
#include "main.h"
#include "DeviceSignalInterfaces.h"
#include <map>

extern UART_HandleTypeDef huart2;
char OutputBuffer[100];
#define DebugUart huart2

using DeviceTypeDACInternalDual = DeviceDACControl<DAC_HandleTypeDef, uint16_t,2, DAC_INTERNAL>;
extern DeviceTypeDACInternalDual DeviceDACInternal;

void eprintf(const char* str, ...)
{

    va_list args1;
    va_start(args1, str);
    vsprintf(OutputBuffer,str, args1);
    va_end(args1);

	HAL_UART_Transmit_IT(&DebugUart, (const uint8_t*)OutputBuffer, strlen(OutputBuffer));
	HAL_Delay(10);
}

void eprintf_repeat()
{
	HAL_UART_Transmit_IT(&DebugUart, (const uint8_t*)OutputBuffer, strlen(OutputBuffer));
	HAL_Delay(10);
}

//=======================================================================================

using DeviceTypeDACInternal     = DeviceDACControl<DAC_HandleTypeDef, uint16_t,1, DAC_INTERNAL>;
using DeviceTypeDACInternalDual = DeviceDACControl<DAC_HandleTypeDef, uint16_t,2, DAC_INTERNAL>;

using DeviceTypeDACExternal     = DeviceDACControl<SPI_HandleTypeDef, int16_t,1, DAC_SPI_8550> ;
using DeviceTypeDACExternalDual = DeviceDACControl<SPI_HandleTypeDef, int16_t,2, DAC_SPI_8550> ;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern std::map<SPI_TypeDef*,bool*> TransmissionFlagsSPI;

extern DeviceTypeDACInternalDual DeviceDACInternal;
extern DeviceTypeDACExternalDual DeviceDACExternal;
//==============================================================
bool FLAG_SPI3_TRANSMIT = 0;
bool FLAG_SPI4_TRANSMIT = 0;

SPI_HandleTypeDef* CURRENT_SPI_PORT      = &hspi3;
     uint16_t      CURRENT_SPI_CS_PIN    = SPI3_CS_W5500_Pin;
GPIO_TypeDef*      CURRENT_SPI_CS_PORT   = SPI3_CS_W5500_GPIO_Port;
     bool*         CURRENT_FLAG_TRANSMIT = &FLAG_SPI3_TRANSMIT;
     uint8_t       CONFIG_REGISTER = 0;


void WriteDACValue(int16_t Value, uint8_t Channel, uint8_t* Buffer)
{
	CURRENT_SPI_CS_PIN  = SPI3_CS_W5500_Pin;
	CURRENT_SPI_CS_PORT = SPI3_CS_W5500_GPIO_Port;
	CURRENT_SPI_PORT    = &hspi3;
	CURRENT_FLAG_TRANSMIT = &FLAG_SPI3_TRANSMIT;

	//if(Channel == 2)
	//{
	//CURRENT_SPI_CS_PIN  = SPI4_CS_Pin;
	//CURRENT_SPI_CS_PORT = SPI4_CS_GPIO_Port;
	//CURRENT_SPI_PORT    = &hspi4;
	//CURRENT_FLAG_TRANSMIT = &FLAG_SPI4_TRANSMIT;
	//}

	HAL_GPIO_WritePin(CURRENT_SPI_CS_PORT, CURRENT_SPI_CS_PIN, GPIO_PIN_RESET);

	Buffer[0] = CONFIG_REGISTER;
	Buffer[1] = Value >> 8;
	Buffer[2] = Value ;

	HAL_SPI_Transmit_IT(CURRENT_SPI_PORT, Buffer, 3);

	while( *CURRENT_FLAG_TRANSMIT == 0){}; *CURRENT_FLAG_TRANSMIT = 0;

	HAL_GPIO_WritePin(CURRENT_SPI_CS_PORT, CURRENT_SPI_CS_PIN, GPIO_PIN_SET);
}
//==============================================================

extern int16_t TestValue;

void TestSinusDAC_SPI()
{
  //ExternalDACControl.RegisterFlag(TransmissionFlagsSPI);

  uint16_t AMPLITUDE  = 28000;
  uint16_t OFFSET1 =  0;
  uint16_t OFFSET2 = 0;

  uint16_t COUNTER = 0;
  uint16_t PERIOD       = 360/2;

  std::pair<int16_t,int16_t> DACControlSignal;
  //====================================
  eprintf("[START SINUS GENERATE EXTERNAL ] \r\n"); //osDelay(2);

  while(true)
  {
    COUNTER++; if(COUNTER == PERIOD) COUNTER = 0;
    DACControlSignal.first  = OFFSET2 + AMPLITUDE*std::sin(COUNTER*2.0*M_PI/PERIOD);
    DACControlSignal.second = OFFSET1 + AMPLITUDE*std::sin(COUNTER*2.0*M_PI/PERIOD);
    //DACControlSignal.second += DACControlSignal.second/50;
    //DACControlSignal.second += AMPLITUDE*0.13;
    //DACControlSignal.first *= 0.96;
    //DACControlSignal.first = 0;

    DeviceDACExternal.SetCoord(DACControlSignal);
    HAL_Delay(1);

    //WriteDACValue(DAC_VALUE , 1, BUFFER_COMMAND);
    //WriteDACValue(DAC_VALUE2, 2, BUFFER_COMMAND);

  }
}

void TestSinusDAC_Internal()
{
  uint16_t AMPLITUDE  = 3000;
  uint16_t PERIOD     = 360/2;
  uint16_t COUNTER = 0;

  std::pair<uint16_t,uint16_t> DACControlSignal;
  //====================================
  eprintf("[START SINUS GENERATE INTERNAL ] \r\n"); //osDelay(2);

  while(true)
  {
    COUNTER++; if(COUNTER == PERIOD) COUNTER = 0;
    DACControlSignal.first  = AMPLITUDE*std::sin(COUNTER*2.0*M_PI/PERIOD);
    DACControlSignal.second = AMPLITUDE*std::cos(COUNTER*2.0*M_PI/PERIOD);

    DACControlSignal | DeviceDACInternal; HAL_Delay(1);

  }
}


