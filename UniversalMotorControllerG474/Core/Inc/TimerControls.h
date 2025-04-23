#ifndef GENERIC_TIMER_CONTROL_H
#define GENERIC_TIMER_CONTROL_H
#include <stdint.h>
#include "main.h"

class TimerControl: public PassValueClass<uint16_t>
{
 public:
 TimerControl(TIM_HandleTypeDef& htim): timer{&htim} { GetTicksFreq(); PeriodTicks = htim.Init.Period;}
 TIM_HandleTypeDef* timer = 0;
 uint16_t TicksInMicro = 170/170; // TICK COUNT IN MICROSECONDS
 uint16_t PeriodTicks = 100-1;
 uint16_t Period = PeriodTicks/TicksInMicro;

  //uint16_t SysFreq = HAL_RCC_GetSysClockFreq()/1000000;
 uint16_t GetTicksFreq() { uint16_t SysFreq = 170; TicksInMicro = SysFreq/timer->Init.Prescaler; return TicksInMicro;}
 const uint16_t& GetPeriod()    { Period = PeriodTicks/TicksInMicro;  return Period;}

 void ChangeTimerPeriod(uint16_t PeriodMicro) {                 PeriodTicks =  PeriodMicro*TicksInMicro-1;}
 void SetTimerPeriod   (uint16_t PeriodMicro) { GetTicksFreq(); PeriodTicks =  PeriodMicro*TicksInMicro-1;}

     void SetValue(uint16_t Value) { ChangeTimerPeriod(Value);}
 const uint16_t& GetValue() { return GetPeriod();}

 void StartTimer() { HAL_TIM_Base_Start_IT(timer); }
 void StopTimer()  { HAL_TIM_Base_Stop_IT(timer);  }
};

//======================================================================
template<int NUM = 1>
class TimerSoft
{
  public:

  static void InitTimer(void (timerCallback)(void*), uint16_t* param = nullptr, osTimerType_t RunType = osTimerPeriodic)
  {
  if(isTimerExist) return; isTimerExist = true;
  
  timerElapsed = false;
  timerLinearCalibrationID = osTimerNew(timerCallback, RunType, (void *)param, NULL);
  }

  static bool isTimerExist;
  static bool timerElapsed;
  static osTimerId timerLinearCalibrationID;

  static void start(uint16_t milli) { osTimerStart(timerLinearCalibrationID,milli); }
  static void stop() { osTimerStop(timerLinearCalibrationID); }
  static bool isElapsed() { return timerElapsed;  }
  static void reset()     { timerElapsed = false; }
  static void timerTickCallback(void* argument) { timerElapsed = true; }; 
};

template<int NUM> bool      TimerSoft<NUM>::isTimerExist = false;
template<int NUM> bool      TimerSoft<NUM>::timerElapsed = false;
template<int NUM> osTimerId TimerSoft<NUM>::timerLinearCalibrationID;
//======================================================================

#endif //GENERIC_TIMER_CONTROL_H
