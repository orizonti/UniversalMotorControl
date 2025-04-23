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
class SystemScaleSettings
{
  public:
  constexpr static float ConvertPix_DAC = 3200.0/400;  // +-100PIX = +-32000 DAC VALUE
  constexpr static float ConvertSecs_TimerTick = 100.0/1000000; //10000 TICK IN 1 SEC
  constexpr static float InputSignalPeriod = 10.0/1000.0; //5MS PERIOD 200 HZ
  constexpr static float StepPeriod  = 1.0/10000; // 100 MKS CONTROL STEP
};

template<int N_CHANNEL>
class AimingFixedStepClass : public PassValueClass<float>
{
  public:
  AimingFixedStepClass() 
  { 
   //eprintf("[ AIMING FIXED STEP ] FREQ: %d", StepFreq);
  } 
  uint16_t StepFreq = SystemScaleSettings::InputSignalPeriod/SystemScaleSettings::StepPeriod;

  float InputValue;
  float ValueVelocity = 0;
  float ValueAcceleration = 0;
  float OutputValue = 0;
  float NextOutputValue = 0;

  void SetValue(const float& NewInputValue)
  {
    ValueAcceleration = (NewInputValue - InputValue)/StepFreq - ValueVelocity;
        ValueVelocity = (NewInputValue - InputValue)/StepFreq;

                          InputValue = NewInputValue;
        NextOutputValue = InputValue;
  }
   
  const float& GetValue()
  {
	       OutputValue = NextOutputValue; NextOutputValue += ValueVelocity;
    return OutputValue;
  }
};

template<>
class AimingFixedStepClass<2> : public PassCoordClass<float>
{
  public:
  AimingFixedStepClass() 
  { 
  } 
  std::pair<float,float> OutputSignal;
  std::pair<float,float> InputSignal;
  AimingFixedStepClass<1> AimingChannel1;
  AimingFixedStepClass<1> AimingChannel2;

  void SetCoord(const std::pair<float,float>& NewInput)
  {
    InputSignal = NewInput;
    InputSignal.first  | AimingChannel1;
    InputSignal.second | AimingChannel2;
  }
   
  const std::pair<float,float>& GetCoord()
  {
    AimingChannel1 | OutputSignal.first;
    AimingChannel2 | OutputSignal.second;
    return OutputSignal;
  }
};



template<uint8_t N_CHANNEL = 1>
class AimingPIDClass : public PassValueClass<float>
{
  public:
  AimingPIDClass() { SetPIDParam(1,0.0,0.0);     }
  arm_pid_instance_f32 PIDParam;
  uint8_t ChannelCount = N_CHANNEL;

  float InputSignal   = 0.0;
  float OutputSignal  = 0.0;

  void SetPIDParam(float K, float I, float D) { PIDParam.Kp = K; PIDParam.Ki = I; PIDParam.Kd = D; arm_pid_init_f32(&PIDParam, 1); }
  void ChangePIDParam(float K, float I, float D) { PIDParam.Kp = K; PIDParam.Ki = I; PIDParam.Kd = D; arm_pid_init_f32(&PIDParam, 0); }

    void SetValue(const float& Value) {InputSignal = Value; OutputSignal = arm_pid_f32(&PIDParam,InputSignal); }
  const float& GetValue() { return OutputSignal;};

};

template<>
class AimingPIDClass<2> : public PassCoordClass<float>
{
  public:
  AimingPIDClass() { SetPIDParam(1,0.0002,0.0001);     }
  arm_pid_instance_f32 PIDParam;
  uint8_t ChannelCount = 2;

  std::pair<float,float> InputSignal  = std::pair<float,float>(0,0);
  std::pair<float,float> OutputSignal = std::pair<float,float>(0,0);

  void SetPIDParam(float K, float I, float D) { PIDParam.Kp = K; PIDParam.Ki = I; PIDParam.Kd = D; arm_pid_init_f32(&PIDParam, 1); }
  void ChangePIDParam(float K, float I, float D) { PIDParam.Kp = K; PIDParam.Ki = I; PIDParam.Kd = D; arm_pid_init_f32(&PIDParam, 0); }

    void SetCoord(const std::pair<float,float>& Coord) 
    {
      InputSignal = Coord; OutputSignal.first = arm_pid_f32(&PIDParam,InputSignal.first); 
                           OutputSignal.second = arm_pid_f32(&PIDParam,InputSignal.second);
    }
  const std::pair<float,float>& GetCoord() { return OutputSignal;};
};
//================================================================================

enum class AimingType { DIRECT = 0, PID_VELOCITY = 1};

template<typename V, AimingType Type = AimingType::DIRECT, int N_CHANNEL = 1>
class AimingDinamicClass : public PassValueClass<V>
{
  public:
  AimingDinamicClass() 
  {
    //if(Type == AimingType::DIRECT) eprintf("[ AIMING DINAMIC ] TYPE: %s", "PROLONG DIRECT");
    //if(Type == AimingType::PID_VELOCITY) eprintf("[ AIMING DINAMIC ] TYPE: %s", "LOOP PID VELOCITY");
  }

  float PixToDAC        = SystemScaleSettings::ConvertPix_DAC;
  //float SecToTickPeriod = SystemScaleSettings::ConvertSecs_TimerTick;
  //float StepPeriod   = SystemScaleSettings::StepPeriod;
  float SecToTickPeriod = 1.0/65.0;
  float StepPeriod   = 10.0/1000000;

  V     OutputValue = 0;
  V NextOutputValue = 0;

  MessageAiming AimState; //AIM STATE IN DAC VALUES, AND TIMER PERIOD TICK SCALE

  void SetValue(const V& Value) { AimState.Position = Value; NextOutputValue = AimState.Position; }
  const V& GetValue() 
  { 
                      OutputValue = NextOutputValue;

                                    //AimState.Velocity += AimState.Acceleration*StepPeriod;
    NextOutputValue = OutputValue + AimState.Velocity*StepPeriod;

               return OutputValue;
  }

  void SetState(const MessageAiming& NewState) { AimState.Position = NewState.Position;
                                                  AimState.Velocity = NewState.Velocity;
                                                  AimState.Acceleration = NewState.Acceleration;
                                                  NextOutputValue = AimState.Position;} // POS IN DAC
  friend AimingDinamicClass& operator|(MessageAiming& State , AimingDinamicClass& Receiver) { Receiver.SetState(State);  return Receiver;}

};


template<typename V>
class AimingDinamicClass<V,AimingType::PID_VELOCITY> : public PassValueClass<V>
{
  public:
  AimingDinamicClass() { }

  float PixToDAC        = SystemScaleSettings::ConvertPix_DAC;
  float SecToTickPeriod = SystemScaleSettings::ConvertSecs_TimerTick;
  float StepPeriod   = SystemScaleSettings::StepPeriod; 
  float StepFreq = SystemScaleSettings::InputSignalPeriod/SystemScaleSettings::StepPeriod;

  V OutputVelocity = 0;
  V OutputValue = 0; //OUTPUT POS IN DAC
  
  MessageAiming AimState;
  AimingPIDClass<1> PID;

  void SetValue(const V& Value) { AimState.Position = Value - 200;   //INPUT SIGNAL IN PIX
                                  //AimState.Position | PID | OutputVelocity; // Velocity PIX/SECS
															OutputVelocity /= StepFreq;
															OutputVelocity *= 50;
                                                             } // Velocity PIX/TICKS

  const V& GetValue() { OutputValue += OutputVelocity*StepPeriod; return OutputValue; }

  void SetState(const MessageAiming& NewState) { AimState = NewState; SetValue(AimState.Position);}
  friend AimingDinamicClass& operator|(MessageAiming& State , AimingDinamicClass& Receiver) { Receiver.SetState(State);  return Receiver;}


};

template<typename V, AimingType Type> //TWO CHANNEL CONTAINER FOR ONE CHANNEL AIMING CLASS
class AimingDinamicClass<V,Type,2> : public PassCoordClass<V>
{
  public:
  AimingDinamicClass() { Modules[0] = &AimingModule1;  Modules[1] = &AimingModule2;}

  AimingDinamicClass<V,Type,1> AimingModule1;
  AimingDinamicClass<V,Type,1> AimingModule2;

  void SetState(const MessageAiming& NewState) { Modules[CurrentChannel]->SetState(NewState);
                                                          CurrentChannel++; 
                                   if(CurrentChannel > 1) CurrentChannel = 0;
  };
  //friend AimingDinamicClass<V,Type,2>& operator|(MessageAiming& State , AimingDinamicClass<V,Type,2>& Receiver) { Receiver.SetState(State);  return Receiver;}
  friend AimingDinamicClass& operator|(MessageAiming& State , AimingDinamicClass& Receiver)
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


  AimingDinamicClass<V,Type>* Modules[2];

  uint8_t CurrentChannel = 0;
};

#endif //GENERIC_AIMING_CONTROL_H
