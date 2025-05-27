#ifndef GENERIC_AIMING_CONTROL_H
#define GENERIC_AIMING_CONTROL_H
#include <stdint.h>
#include <cmsis_os2.h>
#include "RING_BUFFER/message_command_structures.h"
#include <utility>
#include "main.h"
#include <queue>
#include "DeviceSignalInterfaces.h"
#include <arm_math.h>
#include "PassSignalInterface.h"
#include <array>
class SystemScaleSettings
{
  public:
  constexpr static float ConvertPix_DAC = 3200.0/400;  // +-100PIX = +-32000 DAC VALUE
  constexpr static float ConvertSecs_TimerTick = 100.0/1000000; //10000 TICK IN 1 SEC
  constexpr static float InputSignalPeriod = 10.0/1000.0; //5MS PERIOD 200 HZ
  constexpr static float StepPeriod  = 1.0/10000; // 100 MKS CONTROL STEP
};

                              enum ExtrapolationTypes {LINEAR_EXTRAPOLATION = 0, DINAMIC_EXTRAPOLATION};
template<typename V,int N_CHANNEL, ExtrapolationTypes Type> 
                              class ModuleTypeExtrapolation;
      template<int N_CHANNEL> class ModuleTypePIDControl; 
   template<typename DEV_OUT> class ModuleTypeAimingControl;

//=================================================================
//uint16_t StepFreq = SystemScaleSettings::InputSignalPeriod/SystemScaleSettings::StepPeriod;
//eprintf("[ AIMING FIXED STEP ] FREQ: %d", StepFreq);

template<typename V,int N_CHANNEL, ExtrapolationTypes Type = LINEAR_EXTRAPOLATION>
class ModuleTypeExtrapolation : public PassValueClass<V>
{
  public:
  ModuleTypeExtrapolation() { };

  V InputValue;
  V OutputValue = 0;

  size_t avarage_size = 8;
  std::array<V,8> InputValueSample = {0};  
  std::array<V,8>::iterator CurrentInputSample = InputValueSample.begin();
  std::array<V,8>::iterator CurrentRemoveSample = InputValueSample.begin()+1;
  float IncrementValueAvarage = 0;

  void SetValue(const V& NewInputValue)
  {
    InputValue = NewInputValue;
                              *CurrentInputSample = (NewInputValue - InputValue)/avarage_size;
    IncrementValueAvarage += (*CurrentInputSample - *CurrentRemoveSample);
                                                    *CurrentRemoveSample = *CurrentInputSample;

      CurrentInputSample++; if(CurrentInputSample  == InputValueSample.end()) CurrentInputSample  = InputValueSample.begin();
     CurrentRemoveSample++; if(CurrentRemoveSample == InputValueSample.end()) CurrentRemoveSample = InputValueSample.begin();

  }
   
  const V& GetValue()
  {
	         OutputValue += IncrementValueAvarage;
    return OutputValue;
  }
};

template<typename V>
class ModuleTypeExtrapolation<V,2,LINEAR_EXTRAPOLATION> : public PassCoordClass<V>
{
  public:
  ModuleTypeExtrapolation() 
  { 
  } 
  std::pair<V,V> OutputSignal;
  std::pair<V,V> InputSignal;
  ModuleTypeExtrapolation<V,1,LINEAR_EXTRAPOLATION> AimingChannel1;
  ModuleTypeExtrapolation<V,1,LINEAR_EXTRAPOLATION> AimingChannel2;

  void SetCoord(const std::pair<V,V>& NewInput)
  {
    InputSignal = NewInput;
    InputSignal.first  | AimingChannel1;
    InputSignal.second | AimingChannel2;
  }
   
  const std::pair<V,V>& GetCoord()
  {
    AimingChannel1 | OutputSignal.first;
    AimingChannel2 | OutputSignal.second;
    return OutputSignal;
  }
};
//=====================================================================================

template<typename V,int N_CHANNEL>
class ModuleTypeExtrapolation<V,N_CHANNEL,DINAMIC_EXTRAPOLATION> : public PassValueClass<V>
{
  public:
  ModuleTypeExtrapolation() { }

  //float PixToDAC        = SystemScaleSettings::ConvertPix_DAC;
  //float SecToTickPeriod = SystemScaleSettings::ConvertSecs_TimerTick;
  //float StepPeriod   = SystemScaleSettings::StepPeriod;
  //float SecToTickPeriod = 1.0/65.0;
  float StepPeriod   = 10.0/1000000;
  V     OutputValue = 0;

  MessageAiming AimState; //AIM STATE IN DAC VALUES, AND TIMER PERIOD TICK SCALE

  void SetValue(const V& Value) { OutputValue = Value;}
  const V& GetValue() 
  { 
           OutputValue += AimState.Velocity*StepPeriod;
    return OutputValue;
  }

  void SetState(const MessageAiming& NewState) { AimState = NewState; } 
  friend ModuleTypeExtrapolation& operator|(MessageAiming& State , ModuleTypeExtrapolation& Receiver) { Receiver.SetState(State);  return Receiver;}

};

//=================================================================
template<typename V>
class ModuleTypeExtrapolation<V,2, DINAMIC_EXTRAPOLATION> : public PassCoordClass<V>
{
  public:
  ModuleTypeExtrapolation() { }

  ModuleTypeExtrapolation<V,1, DINAMIC_EXTRAPOLATION> AimingModule1;
  ModuleTypeExtrapolation<V,1, DINAMIC_EXTRAPOLATION> AimingModule2;

  void SetState(const MessageAimingDual& NewState) 
  { 
    AimingModule1->SetState(NewState);
    AimingModule2->SetState(NewState);
  };
  //friend ModuleTypeExtrapolation<V,Type,2>& operator|(MessageAiming& State , ModuleTypeExtrapolation<V,Type,2>& Receiver) { Receiver.SetState(State);  return Receiver;}
  friend ModuleTypeExtrapolation& operator|(MessageAimingDual& State , ModuleTypeExtrapolation& Receiver)
  { Receiver.SetState(State);  return Receiver;}


  void SetCoord(const std::pair<V,V>& CoordState) { AimingModule1.SetValue(CoordState.first); 
                                                    AimingModule2.SetValue(CoordState.second); };
                                                       
  const std::pair<V,V>& GetCoord() 
  { 
           AimCoord.first  = AimingModule1.GetValue();
           AimCoord.second = AimingModule2.GetValue();
    return AimCoord; 
  }

  std::pair<V,V> AimCoord;
};
//=================================================================

template<int N_CHANNEL = 1>
class ModuleTypePIDControl : public PassValueClass<float>
{
  public:
  ModuleTypePIDControl() { SetPIDParam(1,0.0,0.0);     }
  arm_pid_instance_f32 PIDParam;
  uint8_t ChannelCount = N_CHANNEL;

  float InputSignal   = 0.0;
  float OutputSignal  = 0.0;

  void SetPIDParam   (float K, float I, float D) { PIDParam.Kp = K; PIDParam.Ki = I; PIDParam.Kd = D; 
                                                                   arm_pid_init_f32(&PIDParam, 1); }

  void ChangePIDParam(float K, float I, float D) { PIDParam.Kp = K; PIDParam.Ki = I; PIDParam.Kd = D; 
                                                                   arm_pid_init_f32(&PIDParam, 0); }

    void SetValue(const float& Value) {InputSignal = Value; OutputSignal = arm_pid_f32(&PIDParam,InputSignal); }
  const float& GetValue() { return OutputSignal;};

};

template<>
class ModuleTypePIDControl<2> : public PassCoordClass<float>
{
  public:
  ModuleTypePIDControl() { SetPIDParam(1,0.0002,0.0001);     }
  arm_pid_instance_f32 PIDParam;
  uint8_t ChannelCount = 2;

  std::pair<float,float> InputSignal  = std::pair<float,float>(0,0);
  std::pair<float,float> OutputSignal = std::pair<float,float>(0,0);

     void SetPIDParam(float K, float I, float D) { PIDParam.Kp = K; PIDParam.Ki = I; PIDParam.Kd = D; 
                                                                   arm_pid_init_f32(&PIDParam, 1); }

  void ChangePIDParam(float K, float I, float D) { PIDParam.Kp = K; PIDParam.Ki = I; PIDParam.Kd = D; 
                                                                   arm_pid_init_f32(&PIDParam, 0); }

    void SetCoord(const std::pair<float,float>& Coord) 
    {
      InputSignal = Coord; OutputSignal.first = arm_pid_f32(&PIDParam,InputSignal.first); 
                           OutputSignal.second = arm_pid_f32(&PIDParam,InputSignal.second);
    }
  const std::pair<float,float>& GetCoord() { return OutputSignal;};
};
//================================================================================



#endif //GENERIC_AIMING_CONTROL_H
