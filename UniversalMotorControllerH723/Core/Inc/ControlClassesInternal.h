#ifndef GENERIC_INTERNAL_CONTROL_H
#define GENERIC_INTERNAL_CONTROL_H
#include <stdint.h>
#include "main.h"
#include "stm32h7xx_hal_dac.h"
#include "stm32h7xx_hal_spi.h"
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
extern float Gain1;
extern float Gain2;

extern int32_t Offset1;
extern int32_t Offset2;

extern int32_t Offset2_1;
extern int32_t Offset2_2;

extern int16_t RegSignalInput[50];
extern int16_t RegSignalOutput[50];
extern int16_t* PosSignalInput;
extern int16_t* PosSignalOutput;

template<typename DEV_IN, typename DEV_OUT>
class DeviceTypeScanator
{
  public:
  using SignalType = decltype(DEV_OUT::OutputSignal);
  using MessageStateType = MessageState1;

  #define DEVICE_TYPE_SCANATOR      0xD0
  #define COMMAND_MONITORING_TOGGLE 0xD1

  public:
  static DEV_OUT* DeviceControl;
  static DEV_IN*  DeviceInput1;
  static DEV_IN*  DeviceInput2;


  void WriteTrack(int16_t Input, int16_t Output)
  {

	  //if(std::abs(Input) < 500)
	  //{
	//	  PosSignalInput = RegSignalInput;
	//	  PosSignalOutput = RegSignalOutput;
	//	  return;
	  //}


	  if(PosSignalInput < RegSignalInput+50)
	  {
	  *PosSignalInput = Input; PosSignalInput++;
	  *PosSignalOutput = Output; PosSignalOutput++;
	  }

  }

  bool STATE_IDLE = true;
  bool STATE_MONITORING = false;

      std::pair<float, float>  InputSignal{0,0};
  std::pair<int16_t, int16_t> SensorSignal{0,0};
  std::pair<int16_t, int16_t> ControlSignal{0,0};
  MessageStateType MessageState;

  explicit DeviceTypeScanator(DEV_IN* DevInput1, DEV_IN* DevInput2, DEV_OUT* DevOut) 
                            { 
                              DeviceControl = DevOut;

                              DeviceInput1 = DevInput1;
                              DeviceInput2 = DevInput2;
                            }

  decltype(SensorSignal) GetSensor()
                                { SensorSignal.first  = (DeviceInput1->GetValue() + Offset1)*Gain1 + Offset2_1;
                                  SensorSignal.second = (DeviceInput2->GetValue() + Offset2)*Gain2 + Offset2_1;
                           return SensorSignal; }

  decltype(ControlSignal) GetControl() { return ControlSignal; }
  decltype(InputSignal)   GetInput()   { return InputSignal; }


  friend void operator|(std::pair<int16_t,int16_t>& Signal, DeviceTypeScanator& Device)
  {
    Device.InputSignal.second = Signal.first;
    Device.InputSignal.first  = Signal.second;
    //Device.GenerateOutputSensorLoop();
    //Device.GenerateOutputDirect();
  }

  void SetInput(MessageAimingDual& AimingCommand)
  {
    InputSignal.first  = AimingCommand.Channel1.Position;
    InputSignal.second = AimingCommand.Channel2.Position;

    GenerateOutputSensorLoop();
    //GenerateOutputDirect();
    //GenerateOutputIntegrator();
  };

  void GenerateOutputDirect()
  {
    ControlSignal.first   = InputSignal.first*1 ;
    ControlSignal.second  = InputSignal.second*1;

    WriteTrack(InputSignal.first, ControlSignal.first);

    if(abs(ControlSignal.first) > 29000 || abs(ControlSignal.second) > 29000) return;
    ControlSignal | *DeviceControl;
  }

  void GenerateOutputIntegrator()
  {
    ControlSignal.first   += InputSignal.first/1 ;
    ControlSignal.second  += InputSignal.second/1;

    WriteTrack(InputSignal.first, ControlSignal.first);

    if(abs(ControlSignal.first) > 29000 || abs(ControlSignal.second) > 29000) return;
    ControlSignal | *DeviceControl;
  }

  void GenerateOutputSensorLoop()
  {

    SensorSignal.first  = (DeviceInput1->GetValue() + Offset1)*Gain1 + Offset2_1;
    SensorSignal.second = (DeviceInput2->GetValue() + Offset2)*Gain2 + Offset2_2;

	//========================================================================
    ControlSignal.first   = InputSignal.first  + SensorSignal.first *1; //ControlSignal.first = 0;
    ControlSignal.second  = InputSignal.second + SensorSignal.second*1; //ControlSignal.second  = 0;
    //WriteTrack(InputSignal.first, SensorSignal.first);

	//if(std::abs(InputSignal.first) < 300 && std::abs(InputSignal.second) < 300)
	//{
    //ControlSignal.first   += InputSignal.first;
    //ControlSignal.second  += InputSignal.second;
	//}
	//else
	//{
    //ControlSignal.first   = InputSignal.first  + SensorSignal.first *0.99; //ControlSignal.first = 0;
    //ControlSignal.second  = InputSignal.second + SensorSignal.second*0.99; ControlSignal.second  = 0;
	//}

	//========================================================================

    if(abs(ControlSignal.first) > 29000 || abs(ControlSignal.second) > 29000) return;

    ControlSignal | *DeviceControl;

    //WriteTrack(InputSignal.first, SensorSignal.first);
  }

  void SetToNull()
  {
	  ControlSignal.first = 0; ControlSignal.second = 0;
	  ControlSignal | *DeviceControl;
  }
  //=================================================
  friend void operator|(MessageAimingDual& AimingCommand, DeviceTypeScanator& Device) { Device.SetInput(AimingCommand); }



  friend void operator|(CommandDevice<0>& Message, DeviceTypeScanator& Device) { Device.SetInput(Message); }
  void SetInput(CommandDevice<0>& Message)
  { 
    if(Message.Command == COMMAND_MONITORING_TOGGLE) STATE_MONITORING = !STATE_MONITORING; 
           eprintf("[ SCANATOR SET MONITORING %d ] \r\n", STATE_MONITORING);
  };

  private:
                                 ModuleTypePIDControl<2> ModulePID;
  ModuleTypeExtrapolation<float,2, LINEAR_EXTRAPOLATION> ModuleExtrapolation;

  PassCoordTransform<float,uint16_t,SystemScaleSettings::ConvertPix_DAC> PixToDAC;

};

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

  static MessagePositionState<0> MessagePosition;

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

  friend void operator|(MessagePositionState<0>& InputPosition, ModuleTypeDelayMeasure& MeasureNode)
  {
    MeasureNode.SetInput(InputPosition);
  }        

  void SetInput(MessagePositionState<0>& InputPosition) {InputSignal = InputPosition.Position1; }

  friend void operator|(CommandCalibration& CommandCalibration, ModuleTypeDelayMeasure& MeasureNode)
  {
    MeasureNode.PerformCommand(CommandCalibration);
  }        

  void PerformCommand(CommandCalibration& Message)
  {
   if(Message.Command == COMMAND_MEASURE_PROCESS_START) { if(STATE_IDLE) StartProcessMeasureDelay(1);};
   if(Message.Command == COMMAND_IDLE) StopProcess();
  }
};

template<typename DEV_IN, typename DEV_OUT>
class ModuleTypeResponceMeasure
{
  using SignalType = decltype(DEV_OUT::OutputSignal);
  using MessageMeasureType = MessageGeneric<MessageMeasure1,MESSAGE_HEADER_GENERIC>;

  #define COMMAND_GET_SERIES     0xA1
  #define COMMAND_MEASURE_LINEAR 0xA2
  #define COMMAND_MEASURE_STEP   0xA3

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
      TimerCalibration.InitTimer(MakeCalibrationStep); 
  }
  static DEV_OUT* DeviceControl;
  static DEV_IN*  DeviceSignalInput;
  bool STATE_IDLE = true;

  void LinkToDevice(DEV_OUT* Device) { DeviceControl = Device;};
  void LinkToDevice(DEV_IN* Device)  { DeviceSignalInput = Device;};

    CommandCalibration  Settings;
    MessageMeasureType* MeasureStore;
  static MessagePositionState<0> MessagePosition;

  static TimerSoft<1,ModuleTypeResponceMeasure> TimerCalibration;

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

  friend void operator>>(CommandCalibration& CommandCalibration, ModuleTypeResponceMeasure& CalibrationNode)
  {
    CalibrationNode.PerformCommand(CommandCalibration);
  }
               void PerformCommand(CommandCalibration& CommandCalibration);

  private:
  static void MakeCalibrationStep(void* param) ;
         void MakeStartMarkOnMeasure();
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
}


template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::MakeStartMarkOnMeasure()
{
  uint16_t* DataStart = MeasureStore->DATA.DataVector; 
  uint16_t*   DataEnd = MeasureStore->DATA.DataVector + MeasureStore->DATA.DataCapacity/2; 
  uint16_t* MarkerPos; 

            MarkerPos= std::find(DataStart, DataEnd, 0xF1F2);
            *(MarkerPos-10) = 10000; 
            *(MarkerPos-9 ) = 10000;
}

template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::ProcessStepMeasure()
{
  eprintf("======================================\r\n");
  eprintf("[ START STEP MEASURE RESPONSE ] AMPL: %d STORE: %d \r\n",Settings.Amplitude, MeasureStore->DATA.DataCapacity);
  DeviceSignalInput->SetModeReady(); osDelay(10);
  //DeviceSignalInput->SetMeasureFrequency(ADC_CLOCK_ASYNC_DIV32);
  MeasureStore->DATA.Clear();


  OutputSignal = 0; OutputSignal | *DeviceControl; osDelay(40);

  StartReadSeriesMeasure(); osDelay(2); MakeStartMarkOnMeasure();

  OutputSignal = 16000; OutputSignal | *DeviceControl; osDelay(40);

  EndReadSeriesMeasure();

  OutputSignal = 0; OutputSignal | *DeviceControl;
  DeviceSignalInput->SetModeContinous(true);
  //DeviceSignalInput->SetMeasureFrequency(ADC_CLOCK_ASYNC_DIV4);
  eprintf("======================================\r\n");
}

template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::GetSeriesMeasure() 
{ 
  DeviceSignalInput->SetModeContinous(false);
  StartReadSeriesMeasure(); 
  EndReadSeriesMeasure(); 
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

  bool MeasureDone = DeviceSignalInput->WaitEndMeasure();
   if( MeasureDone) ConnectionRemote.PutMessageToSend((uint8_t*)MeasureStore, sizeof(MessageMeasureType));
   if( MeasureDone) eprintf("[ STEP MEASURE DONE SIZE %d] \r\n", sizeof(MessageMeasureType));
   if(!MeasureDone) eprintf("[ STEP MEASURE FAIL ] \r\n");

  SetStateIdle();
}

template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::SetStateIdle() 
{ 
  STATE_IDLE = true; 
  eprintf("[CALIBRATION IDLE REGIM] \r\n"); 
};

template<typename DEV_IN, typename DEV_OUT>
void ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::PerformCommand(CommandCalibration& CommandCalibration)
{
      STATE_IDLE = false;
      Settings = CommandCalibration;
  if(Settings.Command == COMMAND_GET_SERIES)     { GetSeriesMeasure();             return; };
  if(Settings.Command == COMMAND_MEASURE_LINEAR) { ProcessLinearCalibration();     return; };
  if(Settings.Command == COMMAND_MEASURE_STEP)   { ProcessStepMeasure();           return; };
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
MessagePositionState<0> ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::MessagePosition;

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
TimerSoft<1,ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>> ModuleTypeResponceMeasure<DEV_IN,DEV_OUT>::TimerCalibration;

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
MessagePositionState<0> ModuleTypeDelayMeasure<DEV_OUT1,DEV_OUT2,DEV_OUT3>::MessagePosition;

//====================================================================================
template<typename DEV_IN, typename DEV_OUT> DEV_OUT* DeviceTypeScanator<DEV_IN,DEV_OUT>::DeviceControl = nullptr;
template<typename DEV_IN, typename DEV_OUT> DEV_IN*  DeviceTypeScanator<DEV_IN,DEV_OUT>::DeviceInput1  = nullptr;
template<typename DEV_IN, typename DEV_OUT> DEV_IN*  DeviceTypeScanator<DEV_IN,DEV_OUT>::DeviceInput2  = nullptr;

#endif //GENERIC_INTERNAL_CONTROL_H

    //if(STATE_MONITORING)
    //{
    //MessageState.Position1 = AimingCommand.Channel1.Position;
    //MessageState.Position2 = AimingCommand.Channel2.Position;
    // MessageState.Measure1 = SensorSignal.first;
    // MessageState.Measure2 = SensorSignal.second;
    //ConnectionRemote.PutMessageToSend(MessageState);
    //}
