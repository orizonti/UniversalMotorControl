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
#include "./RING_BUFFER/message_command_structures.h"
#include "./RING_BUFFER/message_struct_generic.h"
#include "./RING_BUFFER/message_header_generic.h"

#include "AimingModules.h"
#include "TCPInterface.h"
#include "TimerControls.h"

extern DAC_HandleTypeDef hdac1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart3;

extern UDPConnectionInterface ConnectionControl;
extern osMessageQueueId_t QueueMessagePositionStateHandle;

template<typename DEV_OUT>
class AimingControlClass
{
  public:
  DEV_OUT* DeviceControl = nullptr;
  void LinkToDevice(DEV_OUT* Device) { DeviceControl = Device;};
  MessageAimingDual AimingState;

  void SetInput(MessageAimingDual& MessageAiming)
  {
             AimingState = MessageAiming;
    ControlSignal.first  = MessageAiming.Channel1.Position;
    ControlSignal.second = MessageAiming.Channel2.Position;
    ControlSignal >> *DeviceControl;
  }

  friend void operator>>(MessageAimingDual& MessageAiming, AimingControlClass AimingNode)
  {
    AimingNode.SetInput(MessageAiming);
  }

  std::pair<uint16_t,uint16_t> ControlSignal{0,0};
};
//=========================================================================================

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
class DeviceCalibrationClass
{
  enum CalibrationRegim {RegimIdle, RegimLinear, RegimStepMeasure};
  using SignalType = decltype(DEV_OUT::OutputSignal);
  using MessageMeasureType = MessageGeneric<MessageMeasure1,MESSAGE_HEADER_GENERIC>;
  #define COMMAND_GET_POSITION   0xA1
  #define COMMAND_GET_SERIES     0xA2
  #define COMMAND_MEASURE_LINEAR 0xA3
  #define COMMAND_MEASURE_STEP   0xA4
  #define COMMAND_CONTINOUS      0xA5
  #define COMMAND_IDLE           0xA6
  #define COMMAND_MEASURE_DELAY  0xA7
  #define COMMAND_RESPONSE       0xA8

  public:
  explicit DeviceCalibrationClass(DEV_IN* DeviceInput, DEV_OUT* DeviceOutput, DEV_OUT_MON* DeviceOutput2) 
                                                    { 
                                                      eprintf("INIT TIMERS \r\n");
                                                      Timer.InitTimer(GetPosition, &(Settings.StepSize)); 
                                                      TimerGenerator.InitTimer(GenerateTestSignal, &(Settings.StepSize)); 
                                                      TimerCalibration.InitTimer(MakeCalibrationStep, &(Settings.StepSize)); 
                                                      MeasureStore = new MessageMeasureType; 
                                                      DeviceControl = DeviceOutput;
                                                      DeviceMonitor = DeviceOutput2;
                                                      DeviceMeasure = DeviceInput;
                                                    };
  static DEV_OUT_MON* DeviceMonitor;
  static DEV_OUT* DeviceControl;
  static DEV_IN*  DeviceMeasure;
  bool STATE_IDLE = true;

  int COMMAND_COUNTER = 0;
  int COMMAND_PHASE = 0;
  void LinkToDevice(DEV_OUT* Device) { DeviceControl = Device;};
  void LinkToDevice(DEV_IN* Device) { DeviceMeasure = Device;};

      CalibrationRegim Regim{RegimIdle};
    MessageCalibration Settings;
    MessageMeasureType* MeasureStore;
  static MessagePositionState MessagePosition;
  TimerSoft<1> Timer;
  TimerSoft<2> TimerGenerator;
  static TimerSoft<3> TimerCalibration;

  static SignalType MeasureSignal;
  static SignalType InputSignal;
  static SignalType OutputSignal;
  static   uint32_t OutputSignalPhase;

  static void GetPosition(void* param) ;
  static void MakeCalibrationStep(void* param) ;
  static void GenerateTestSignal(void* param) ;

  void SetInput(SignalType Signal) { InputSignal = Signal;};
  bool isIdleState() { return STATE_IDLE; };
  void PerformCommand(MessageCalibration& MessageCalibration);
  void SetStateIdle();
  void ProcessStepMeasure();
  void ProcessContinousMeasure();
  void ProcessLinearCalibration();
  void ProcessMeasureDelay(uint8_t Form) 
  { 
    Timer.stop(); 
    TimerGenerator.start(2); 
    DeviceMeasure->SetModeContinous(false);
    eprintf("[ PROCESS MEASURE DELAY ] \r\n");
    STATE_IDLE = false;
  };


  void GetSeriesMeasure();
  void WaitSeriesMeasure();
  void StartSeriesMeasure();

  friend void operator>>(MessageCalibration& MessageCalibration, DeviceCalibrationClass& CalibrationNode)
  {
    CalibrationNode.PerformCommand(MessageCalibration);
  }

};


template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
void DeviceCalibrationClass<DEV_IN,DEV_OUT, DEV_OUT_MON>::ProcessLinearCalibration()
{
  Regim = RegimLinear;
  eprintf("[ START LINEAR CALIBRATION ] NUMBER_STEPS: %d %d \r\n",Settings.NumberSteps);
  DeviceMeasure->SetModeReady();
  TimerCalibration.start(2);
  OutputSignal = 0;
}

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
void DeviceCalibrationClass<DEV_IN,DEV_OUT, DEV_OUT_MON>::MakeCalibrationStep(void* param) 
{
  OutputSignal++; 
  OutputSignal | *DeviceControl; 
  MessagePosition.Position1 = OutputSignal; 
  MessagePosition.Measure1  = DeviceMeasure->GetValue();

  osMessageQueuePut(QueueMessagePositionStateHandle, &MessagePosition, 0, 0);
  //eprintf("MEASURE: %d STEP: %d \r\n", MessagePosition.Measure1, OutputSignal);

  if(OutputSignal < 3000) return;

  TimerCalibration.stop();

}

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
void DeviceCalibrationClass<DEV_IN,DEV_OUT, DEV_OUT_MON>::ProcessContinousMeasure()
{
  eprintf("[ CONTINOUS MEASURE ] \r\n");
  DeviceMeasure->SetModeContinous(false);
  Timer.start(1000);
}

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
void DeviceCalibrationClass<DEV_IN,DEV_OUT, DEV_OUT_MON>::GetPosition(void* param) 
{
  MessagePosition.Measure1  = DeviceMeasure->GetValue();
  //osMessageQueuePut(QueueMessagePositionStateHandle, &MessagePosition, 0, 0);
                  eprintf("[ GET POSITON  ] %d \r\n", MessagePosition.Measure1);
}


template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
void DeviceCalibrationClass<DEV_IN,DEV_OUT, DEV_OUT_MON>::ProcessStepMeasure()
{
  eprintf("======================================\r\n");
  eprintf("[ START STEP MEASURE ] AMPL: %d STEP: %d \r\n",Settings.StepSize, Settings.NumberSteps);
  Timer.stop(); 
  DeviceMeasure->SetModeContinous(true); 

  //OutputSignal = Settings.StepSize;
  OutputSignal = 2500;
  OutputSignal | *DeviceMonitor; 

  StartSeriesMeasure(); osDelay(10);
  OutputSignal | *DeviceControl; 
  WaitSeriesMeasure(); osDelay(5);

  OutputSignal = 0; OutputSignal | *DeviceControl; 
                    OutputSignal | *DeviceMonitor; 
  eprintf("======================================\r\n");
}

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
void DeviceCalibrationClass<DEV_IN,DEV_OUT, DEV_OUT_MON>::StartSeriesMeasure() 
{ 
  MeasureStore->DATA.Clear();
  DeviceMeasure->WriteSignal(MeasureStore->DATA.DataVector,MeasureStore->DATA.DataCapacity);
}

extern osThreadId_t TaskControlHandle;
template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
void DeviceCalibrationClass<DEV_IN,DEV_OUT, DEV_OUT_MON>::WaitSeriesMeasure() 
{ 

       osThreadSuspend(TaskControlHandle); osDelay(2);
  bool MeasureDone = DeviceMeasure->WaitEndMeasure();
   if( MeasureDone) ConnectionControl.sendData((uint8_t*)MeasureStore, sizeof(MessageMeasureType));
   if( MeasureDone) eprintf("[ STEP MEASURE DONE ] \r\n");
   if(!MeasureDone) eprintf("[ STEP MEASURE FAIL ] \r\n");
        osThreadResume(TaskControlHandle); 

  SetStateIdle();
}

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
void DeviceCalibrationClass<DEV_IN,DEV_OUT, DEV_OUT_MON>::GetSeriesMeasure() { StartSeriesMeasure(); WaitSeriesMeasure(); }


template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
void DeviceCalibrationClass<DEV_IN,DEV_OUT, DEV_OUT_MON>::SetStateIdle() 
{ 
  DeviceMeasure->SetModeReady(); STATE_IDLE = true; 
  Timer.stop(); 
  TimerGenerator.stop();
  eprintf("[CALIBRATION IDLE REGIM] \r\n"); 
};

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
void DeviceCalibrationClass<DEV_IN,DEV_OUT, DEV_OUT_MON>::PerformCommand(MessageCalibration& MessageCalibration)
{
      STATE_IDLE = false;
      Settings = MessageCalibration; InputSignal = Settings.Position;
  if(Settings.Command == COMMAND_RESPONSE)       
  { 
    //COMMAND_COUNTER++; if(COMMAND_COUNTER % 500) eprintf("[ CALIBRATION RESPONSE ] %d \r\n", MessageCalibration.Position);
    return; 
  };
  if(Settings.Command == COMMAND_GET_POSITION)   { GetPosition(&Settings.Channel); return; };
  if(Settings.Command == COMMAND_GET_SERIES)     { GetSeriesMeasure();             return; };
  if(Settings.Command == COMMAND_MEASURE_LINEAR) { ProcessLinearCalibration();     return; };
  if(Settings.Command == COMMAND_MEASURE_STEP)   { ProcessStepMeasure();           return; };
  if(Settings.Command == COMMAND_CONTINOUS)      { ProcessContinousMeasure();      return; };
  if(Settings.Command == COMMAND_IDLE)           { SetStateIdle();                 return; };
  if(Settings.Command == COMMAND_MEASURE_DELAY)  { ProcessMeasureDelay(1);         return; };
      STATE_IDLE = true;
}

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON>
void DeviceCalibrationClass<DEV_IN, DEV_OUT, DEV_OUT_MON>::GenerateTestSignal(void* param) 
{
 OutputSignalPhase++; if(OutputSignalPhase > 360) OutputSignalPhase = 0;
 OutputSignal = 1500 + 1500*sin(4*float(OutputSignalPhase)*M_PI/180);

 //MeasureSignal = DeviceMeasure->GetValue();

 MessagePosition.Measure1 = OutputSignal;
 MessagePosition.Measure1 = OutputSignal;
 MessagePosition.Position1 = OutputSignal;
 MessagePosition.Position2 = OutputSignal;

 osMessageQueuePut(QueueMessagePositionStateHandle, &MessagePosition, 0, 0);
   OutputSignal | *DeviceControl;
    //InputSignal | *DeviceMonitor;

  //MeasureSignal | *DeviceMonitor;
  //OutputSignal | *DeviceMonitor;
}


template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON> 
MessagePositionState DeviceCalibrationClass<DEV_IN,DEV_OUT,DEV_OUT_MON>::MessagePosition;

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON> 
            DEV_OUT* DeviceCalibrationClass<DEV_IN,DEV_OUT,DEV_OUT_MON>::DeviceControl;

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON> 
            DEV_IN*  DeviceCalibrationClass<DEV_IN,DEV_OUT,DEV_OUT_MON>::DeviceMeasure;

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON> 
       DEV_OUT_MON*  DeviceCalibrationClass<DEV_IN,DEV_OUT,DEV_OUT_MON>::DeviceMonitor;

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON> 
            uint32_t DeviceCalibrationClass<DEV_IN,DEV_OUT,DEV_OUT_MON>::OutputSignalPhase = 0;

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON> 
DeviceCalibrationClass<DEV_IN, DEV_OUT,DEV_OUT_MON>::SignalType DeviceCalibrationClass<DEV_IN,DEV_OUT,DEV_OUT_MON>::OutputSignal{0};

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON> 
DeviceCalibrationClass<DEV_IN, DEV_OUT,DEV_OUT_MON>::SignalType DeviceCalibrationClass<DEV_IN,DEV_OUT,DEV_OUT_MON>::MeasureSignal{0};

template<typename DEV_IN, typename DEV_OUT, typename DEV_OUT_MON> 
DeviceCalibrationClass<DEV_IN, DEV_OUT,DEV_OUT_MON>::SignalType DeviceCalibrationClass<DEV_IN,DEV_OUT,DEV_OUT_MON>::InputSignal{0};


#endif //GENERIC_INTERNAL_CONTROL_H
