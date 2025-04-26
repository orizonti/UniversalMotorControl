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

extern DeviceTypeConnectionUDP ConnectionRemote;
extern osMessageQueueId_t QueueMessagePositionStateHandle;


//=========================================================================================
#define COMMAND_IDLE                 0xC0
#define MODULE_TYPE_RESPONCE_MEASURE 0xA0
#define MODULE_TYPE_DELAY_MEASURE    0xB0

template<typename DEV_OUT1,typename DEV_OUT2, typename DEV_OUT3>
class ModuleTypeDelayMeasure
{
  using SignalType = decltype(DEV_OUT2::OutputSignal);

  #define COMMAND_MEASURE_PROCESS_START  0xB1

  public:

  static DEV_OUT1* DeviceLoop;
  static DEV_OUT2* DeviceLoopMonitor;
  static DEV_OUT3* DeviceDelayMonitor;

  static   uint16_t OutputSignalPhase;
  static SignalType OutputSignal;
  static SignalType InputSignal;

  static MessagePositionState MessagePosition;

  bool STATE_IDLE = true;

  explicit ModuleTypeDelayMeasure(DEV_OUT1* Device1, DEV_OUT2* Device2, DEV_OUT3* Device3) 
                              { 
                                DeviceLoop    = Device1;
                                DeviceLoopMonitor = Device2;
                                DeviceDelayMonitor = Device3;
                              };

  void StartProcessMeasureDelay(uint8_t Form)     
  { 
    eprintf("[ PROCESS MEASURE DELAY  ] \r\n"); 
    STATE_IDLE = false; 
  };

  void StopProcess() 
  {
    eprintf("[ PROCESS MEASURE DELAY STOP ] \r\n"); 
    STATE_IDLE = true; 
  }

  void MeasureClosedLoopDelay();

  void LinkToDeviceLoop(DEV_OUT1* Device)         { DeviceLoop    = Device;};
  void LinkToDeviceLoopMonitor(DEV_OUT2* Device)  { DeviceLoopMonitor = Device;};
  void LinkToDeviceDelayMonitor(DEV_OUT3* Device) { DeviceDelayMonitor = Device;};

  friend void operator|(MessagePositionState& InputPosition, ModuleTypeDelayMeasure& MeasureNode)
  {
    MeasureNode.SetInput(InputPosition);
  }        

  void SetInput(MessagePositionState& InputPosition) {InputSignal = InputPosition.Position1; }

  friend void operator|(MessageCalibration& MessageCalibration, ModuleTypeDelayMeasure& MeasureNode)
  {
    MeasureNode.PerformCommand(MessageCalibration);
  }        

  void PerformCommand(MessageCalibration& CommandMessage)
  {
   if(CommandMessage.Command == COMMAND_MEASURE_PROCESS_START) { if(STATE_IDLE) StartProcessMeasureDelay(1);};
   if(CommandMessage.Command == COMMAND_IDLE) StopProcess();
  }
};

template<typename DEV_IN, typename DEV_OUT>
class ModuleTypeResponceMeasure
{
  using SignalType = decltype(DEV_OUT::OutputSignal);
  using MessageMeasureType = MessageGeneric<MessageMeasure1,MESSAGE_HEADER_GENERIC>;

  #define COMMAND_GET_POSITION   0xA1
  #define COMMAND_GET_SERIES     0xA2
  #define COMMAND_MEASURE_LINEAR 0xA3
  #define COMMAND_MEASURE_STEP   0xA4
  #define COMMAND_CONTINOUS      0xA5

  public:
  explicit ModuleTypeResponceMeasure(DEV_IN* DeviceInput, DEV_OUT* DeviceOutput) 
                                                    { 
                                                      MeasureStore = new MessageMeasureType; 
                                                      DeviceControl = DeviceOutput;
                                                      DeviceSignalInput = DeviceInput;
                                                    };
  void InitModule() 
  { 
      eprintf("INIT TIMERS \r\n");
      Timer.InitTimer(GetPosition); 
      TimerCalibration.InitTimer(MakeCalibrationStep); 
  }
  static DEV_OUT* DeviceControl;
  static DEV_IN*  DeviceSignalInput;
  bool STATE_IDLE = true;

  void LinkToDevice(DEV_OUT* Device) { DeviceControl = Device;};
  void LinkToDevice(DEV_IN* Device)  { DeviceSignalInput = Device;};

    MessageCalibration  Settings;
    MessageMeasureType* MeasureStore;
  static MessagePositionState MessagePosition;

         TimerSoft<1,ModuleTypeResponceMeasure> Timer;
  static TimerSoft<2,ModuleTypeResponceMeasure> TimerCalibration;

  static SignalType MeasureSignal;
  static SignalType InputSignal;
  static SignalType OutputSignal;
  static   uint32_t OutputSignalPhase;

  void SetInput(SignalType Signal) { InputSignal = Signal;};
  bool isIdleState() { return STATE_IDLE; };
  void SetStateIdle();


  void ProcessStepMeasure();
  void ProcessLinearCalibration();
  void ProcessContinousMeasure();

  void StartReadSeriesMeasure();
  void EndReadSeriesMeasure();
  void GetSeriesMeasure();

  static void GetPosition(void* param) ;

  friend void operator>>(MessageCalibration& MessageCalibration, ModuleTypeResponceMeasure& CalibrationNode)
  {
    CalibrationNode.PerformCommand(MessageCalibration);
  }
               void PerformCommand(MessageCalibration& MessageCalibration);

  private:
  static void MakeCalibrationStep(void* param) ;
};


template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::ProcessLinearCalibration()
{
  eprintf("[ START LINEAR CALIBRATION ] NUMBER_STEPS: %d %d \r\n",Settings.NumberSteps);
  DeviceSignalInput->SetModeReady();
  TimerCalibration.start(2);
  OutputSignal = 0;
}

template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::MakeCalibrationStep(void* param) 
{
  OutputSignal++; 
  OutputSignal | *DeviceControl; 
  MessagePosition.Position1 = OutputSignal; 
  MessagePosition.Measure1  = DeviceSignalInput->GetValue();

  osMessageQueuePut(QueueMessagePositionStateHandle, &MessagePosition, 0, 0);
  //eprintf("MEASURE: %d STEP: %d \r\n", MessagePosition.Measure1, OutputSignal);

  if(OutputSignal < 3000) return;

  TimerCalibration.stop();

}

template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::ProcessContinousMeasure()
{
  eprintf("[ CONTINOUS MEASURE ] \r\n");
  DeviceSignalInput->SetModeContinous(false);
  Timer.start(5);
}

template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::GetPosition(void* param) 
{
  MessagePosition.Measure1  = DeviceSignalInput->GetValue();
  ConnectionRemote.PutMessageToSend(MessagePosition);
}


template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::ProcessStepMeasure()
{
  eprintf("======================================\r\n");
  eprintf("[ START STEP MEASURE ] AMPL: %d STEP: %d \r\n",Settings.Amplitude, Settings.NumberSteps);
  Timer.stop(); 
  DeviceSignalInput->SetModeContinous(true); 

  OutputSignal = 2000;
  StartReadSeriesMeasure(); osDelay(10);
  OutputSignal | *DeviceControl; 
    EndReadSeriesMeasure(); 

  OutputSignal = 0; OutputSignal | *DeviceControl; 
  eprintf("======================================\r\n");
}

template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::StartReadSeriesMeasure() 
{ 
  MeasureStore->DATA.Clear();
  DeviceSignalInput->WriteSignal(MeasureStore->DATA.DataVector,MeasureStore->DATA.DataCapacity);
}

extern osThreadId_t TaskControlHandle;
template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::EndReadSeriesMeasure() 
{ 
       osThreadSuspend(TaskControlHandle); osDelay(2);
  bool MeasureDone = DeviceSignalInput->WaitEndMeasure();
   if( MeasureDone) ConnectionRemote.sendData((uint8_t*)MeasureStore, sizeof(MessageMeasureType));
   if( MeasureDone) eprintf("[ STEP MEASURE DONE ] \r\n");
   if(!MeasureDone) eprintf("[ STEP MEASURE FAIL ] \r\n");
        osThreadResume(TaskControlHandle); 

  SetStateIdle();
}

template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::GetSeriesMeasure() { StartReadSeriesMeasure(); EndReadSeriesMeasure(); }

template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::SetStateIdle() 
{ 
  DeviceSignalInput->SetModeReady(); STATE_IDLE = true; 
  Timer.stop(); 
  eprintf("[CALIBRATION IDLE REGIM] \r\n"); 
};

template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::PerformCommand(MessageCalibration& MessageCalibration)
{
      STATE_IDLE = false;
      Settings = MessageCalibration; 
  if(Settings.Command == COMMAND_GET_POSITION)   { GetPosition(&Settings.Channel); return; };
  if(Settings.Command == COMMAND_GET_SERIES)     { GetSeriesMeasure();             return; };
  if(Settings.Command == COMMAND_MEASURE_LINEAR) { ProcessLinearCalibration();     return; };
  if(Settings.Command == COMMAND_MEASURE_STEP)   { ProcessStepMeasure();           return; };
  if(Settings.Command == COMMAND_CONTINOUS)      { ProcessContinousMeasure();      return; };
  if(Settings.Command == COMMAND_IDLE)           { SetStateIdle();                 return; };
      STATE_IDLE = true;
}

template<typename DEV_OUT1, typename DEV_OUT2, typename DEV_OUT3> 
void ModuleTypeDelayMeasure<DEV_OUT1, DEV_OUT2, DEV_OUT3>::MeasureClosedLoopDelay() 
{
 if(STATE_IDLE) return;

 OutputSignalPhase++; if(OutputSignalPhase > 360) OutputSignalPhase = 0;
 OutputSignal = 1500 + 1500*sin(4*float(OutputSignalPhase)*M_PI/180);

   OutputSignal | *DeviceLoop;
   OutputSignal | *DeviceLoopMonitor;
    InputSignal | *DeviceDelayMonitor;
}

//====================================================================================

template<typename DEV_IN, typename DEV_OUT> 
MessagePositionState ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::MessagePosition;

template<typename DEV_IN, typename DEV_OUT> 
            DEV_OUT* ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::DeviceControl = nullptr;

template<typename DEV_IN, typename DEV_OUT> 
            DEV_IN*  ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::DeviceSignalInput = nullptr;

template<typename DEV_IN, typename DEV_OUT> 
            uint32_t ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::OutputSignalPhase = 0;

template<typename DEV_IN, typename DEV_OUT> 
ModuleTypeResponceMeasure<DEV_IN, DEV_OUT>::SignalType ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::OutputSignal{0};

template<typename DEV_IN, typename DEV_OUT> 
ModuleTypeResponceMeasure<DEV_IN, DEV_OUT>::SignalType ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::MeasureSignal{0};

template<typename DEV_IN, typename DEV_OUT> 
ModuleTypeResponceMeasure<DEV_IN, DEV_OUT>::SignalType ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::InputSignal{0};

template<typename DEV_IN, typename DEV_OUT> 
TimerSoft<2,ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>> ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::TimerCalibration;

//====================================================================================
template<typename DEV_OUT1, typename DEV_OUT2, typename DEV_OUT3> 
       DEV_OUT1*  ModuleTypeDelayMeasure<DEV_OUT1,DEV_OUT2,DEV_OUT3>::DeviceLoop = nullptr;

template<typename DEV_OUT1, typename DEV_OUT2, typename DEV_OUT3> 
       DEV_OUT2*  ModuleTypeDelayMeasure<DEV_OUT1,DEV_OUT2,DEV_OUT3>::DeviceLoopMonitor = nullptr;

template<typename DEV_OUT1, typename DEV_OUT2, typename DEV_OUT3> 
       DEV_OUT3*  ModuleTypeDelayMeasure<DEV_OUT1,DEV_OUT2,DEV_OUT3>::DeviceDelayMonitor = nullptr;

template<typename DEV_OUT1, typename DEV_OUT2, typename DEV_OUT3> 
       uint16_t ModuleTypeDelayMeasure<DEV_OUT1,DEV_OUT2,DEV_OUT3>::OutputSignalPhase = 0;
template<typename DEV_OUT1, typename DEV_OUT2, typename DEV_OUT3> 
       decltype(DEV_OUT2::OutputSignal) ModuleTypeDelayMeasure<DEV_OUT1,DEV_OUT2,DEV_OUT3>::OutputSignal = 0;
template<typename DEV_OUT1, typename DEV_OUT2, typename DEV_OUT3> 
       decltype(DEV_OUT2::OutputSignal) ModuleTypeDelayMeasure<DEV_OUT1,DEV_OUT2,DEV_OUT3>::InputSignal = 0;
       
template<typename DEV_OUT1, typename DEV_OUT2, typename DEV_OUT3> 
MessagePositionState ModuleTypeDelayMeasure<DEV_OUT1,DEV_OUT2,DEV_OUT3>::MessagePosition;

#endif //GENERIC_INTERNAL_CONTROL_H
