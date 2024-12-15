#ifndef GENERIC_INTERNAL_CONTROL_H
#define GENERIC_INTERNAL_CONTROL_H
#include <stdint.h>
#include "main.h"
#include "stm32g4xx_hal_dac.h"
#include "stm32g4xx_hal_spi.h"
#include <utility>
#include <map>
#include "PassSignalInterface.h"
#include <queue>

extern DAC_HandleTypeDef hdac1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart3;

class ParamRegulator: public PassValueClass<uint16_t>
{
  public:
  ParamRegulator(uint16_t Aim,uint16_t HoldRange,uint16_t LevelOutput): AimValue{Aim}, Range{HoldRange}, OutputValue{LevelOutput} {};
  uint16_t AimValue = 0;
  uint16_t SumValue = 0;
  uint16_t Range    = 0;
  uint16_t OutputValue = 0;
  uint8_t avarage_window = 10;

  std::queue<uint16_t> InputValues;
  void SetValue(const uint16_t& Value)
  {
    SumValue += Value;
    InputValues.push(Value); 
    if(InputValues.size() >= avarage_window)    { SumValue -= InputValues.front(); 
                                                  InputValues.pop()  ; } else return;

    if(AimValue < SumValue)         { OutputValue--; return; }
    if(AimValue > SumValue + Range) { OutputValue++;         }
  }
  const uint16_t& GetValue() { return OutputValue;};
};

class TimerControl: public PassValueClass<uint16_t>
{
 public:
 TimerControl(TIM_HandleTypeDef& htim): timer{&htim} { GetTicksFreq(); PeriodTicks = htim.Init.Period;}
 TIM_HandleTypeDef* timer = 0;
 uint16_t TicksInMks = 170/170; // TICK COUNT IN MKS
 uint16_t PeriodTicks = 100-1;
 uint16_t Period = PeriodTicks/TicksInMks;

  //uint16_t SysFreq = HAL_RCC_GetSysClockFreq()/1000000;
 uint16_t GetTicksFreq() { uint16_t SysFreq = 170; TicksInMks = SysFreq/timer->Init.Prescaler; return TicksInMks;}
 const uint16_t& GetPeriod()    { Period = PeriodTicks/TicksInMks;  return Period;}

 void ChangeTimerPeriod(uint16_t PeriodMks) {                PeriodTicks =  PeriodMks*TicksInMks-1;}
 void SetTimerPeriod   (uint16_t PeriodMks) { GetTicksFreq(); PeriodTicks =  PeriodMks*TicksInMks-1;}

     void SetValue(uint16_t Value) { ChangeTimerPeriod(Value);}
 const uint16_t& GetValue() { return GetPeriod();}

 void StartTimer(){ HAL_TIM_Base_Start_IT(timer); }
 void StopTimer(){HAL_TIM_Base_Stop_IT(timer);}
};

#endif //GENERIC_INTERNAL_CONTROL_H
