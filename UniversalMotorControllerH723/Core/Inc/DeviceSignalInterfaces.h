#ifndef GENERIC_SIGNAL_CONTROL_H
#define GENERIC_SIGNAL_CONTROL_H
#include <stdint.h>
#include "main.h"
#include "stm32h7xx_hal_dac.h"
#include "stm32h7xx_hal_spi.h"

#include <utility>
#include <map>
#include "PassSignalInterface.h"
#include "TimerControls.h"


extern DAC_HandleTypeDef hdac1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart3;

#define DAC_INTERNAL 1
#define DAC_SPI_8653 2
#define DAC_SPI_8550 3

template<typename DEV_T, typename V, int N_CHAN = 1, int TYPE = DAC_INTERNAL>
class DeviceDACControl : public PassValueClass<V> {

public:
    DeviceDACControl(DEV_T& DeviceControl){};
	void SetValue(V Value) {};
	void Init(){};
private:
    DEV_T* Device;
};

//===============================================================================================
//DAC_INTERNAL

template<typename V>
class DeviceDACControl<DAC_HandleTypeDef,V,1, DAC_INTERNAL> : public PassValueClass<V> 
{
public:
    DeviceDACControl(DAC_HandleTypeDef& DeviceControl, int ChannelNumber) {Device = &DeviceControl; Channel = ChannelNumber;};
	void SetValue(const V& Value) { HAL_DAC_SetValue(Device,Channel,DAC_ALIGN_12B_R,Value);}
	void Init() { HAL_DAC_Start(Device,Channel); };

                               V OutputSignal = 0;
    const V& GetValue() { return OutputSignal;}
private:
    DAC_HandleTypeDef* Device = 0;
    int Channel = DAC_CHANNEL_1;
};


template<typename V>
class DeviceDACControl<DAC_HandleTypeDef,V,2, DAC_INTERNAL> : public PassCoordClass<V> 
{
public:
    DeviceDACControl(DAC_HandleTypeDef& DeviceControl) {Device = &DeviceControl;};

	void SetValue(const V& Output, uint8_t Channel) 
	{
		if(Channel == 1) HAL_DAC_SetValue(Device,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Output);
		if(Channel == 2) HAL_DAC_SetValue(Device,DAC_CHANNEL_2,DAC_ALIGN_12B_R,Output);
	}

	void SetCoord(const std::pair<V,V>& Output) 
	{
		HAL_DAC_SetValue(Device,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Output.first);
		HAL_DAC_SetValue(Device,DAC_CHANNEL_2,DAC_ALIGN_12B_R,Output.second);
	}

    const std::pair<V,V>& GetCoord() { return OutputSignal;}
                               std::pair<V,V> OutputSignal;

	void Init() { HAL_DAC_Start(Device,DAC_CHANNEL_1); 
                  HAL_DAC_Start(Device,DAC_CHANNEL_2); 
                  eprintf("[ INTERNAL DAC CONTROL CREATE ] \r\n"); };
private:
    DAC_HandleTypeDef* Device = 0;
};
//DAC_INTERNAL_END
//===============================================================================================

//===============================================================================================
//DAC_8653
template<typename V>
class DeviceDACControl<SPI_HandleTypeDef,V,1, DAC_SPI_8653> : public PassValueClass<V> 
{
public:
    DeviceDACControl(SPI_HandleTypeDef& DeviceControl, std::pair<GPIO_TypeDef*,uint16_t> PinSelect, uint8_t NumberChannel = 1) 
	{
    Device = &DeviceControl;
	CSPin = PinSelect;
	ChannelID = 0x18; if(NumberChannel == 2) ChannelID = 0x19;
	};

	void SetValue(const V& Value) 
	{
        OutputSignal = Value;
		HAL_GPIO_WritePin(CSPin.first, CSPin.second, GPIO_PIN_RESET);
		TransferBuffer[0] = ChannelID;
		TransferBuffer[1] = OutputSignal>>8;
		TransferBuffer[2] = OutputSignal;

		HAL_SPI_Transmit_IT(Device, TransferBuffer, 3); while(!TransmissionEndFlag); TransmissionEndFlag = false;
		HAL_GPIO_WritePin(CSPin.first, CSPin.second, GPIO_PIN_SET);
	}

                               V OutputSignal = 0;
    const V& GetValue() { return OutputSignal;}; //ALWAYS NULL, IF NEEDED ADD WRITE LAST OUTPUT

	static void Init();
    void RegisterFlag(std::map<SPI_TypeDef*,bool*>& FlagsRegister) { FlagsRegister[Device->Instance] = &TransmissionEndFlag;};

    static bool FLAG_INIT_DONE;
    static bool TransmissionEndFlag;
	static uint8_t TransferBuffer[3];
private:
    uint8_t ChannelID = 0x18;

    static SPI_HandleTypeDef* Device;
	static std::pair<GPIO_TypeDef*,uint16_t> CSPin;
    static void WaitEnd() { while(!TransmissionEndFlag); TransmissionEndFlag = false; };
    static void ActivatePort() { HAL_GPIO_WritePin(CSPin.first, CSPin.second, GPIO_PIN_RESET); HAL_Delay(5); }
	static void EndTransmission() {HAL_GPIO_WritePin(CSPin.first, CSPin.second, GPIO_PIN_SET); HAL_Delay(5); };
};

template<typename V>
class DeviceDACControl<SPI_HandleTypeDef,V,2,DAC_SPI_8653> : public PassCoordClass<V> 
{
public:
    DeviceDACControl(SPI_HandleTypeDef& DeviceControl, std::pair<GPIO_TypeDef*,uint16_t> PinSelect) : 
    Channel1{DeviceControl,PinSelect,1},
    Channel2{DeviceControl,PinSelect,2}
	{
	};

    DeviceDACControl<SPI_HandleTypeDef,V,1,DAC_SPI_8653> Channel1;
    DeviceDACControl<SPI_HandleTypeDef,V,1,DAC_SPI_8653> Channel2;
	void Init() { DeviceDACControl<SPI_HandleTypeDef,V,1>::Init(); };

	void SetValue(uint16_t Value) { Channel1.SetValue(Value), Channel2.SetValue(Value); }
	void SetCoord(const std::pair<V,V>& Output) { Channel1.SetValue(Output.first); 
                                                  Channel2.SetValue(Output.second); }
    const std::pair<V,V>& GetCoord() { return OutputSignal;}; //ALWAYS NULL, IF NEEDED ADD WRITE LAST OUTPUT

    void RegisterFlag(std::map<SPI_TypeDef*,bool*>& FlagsRegister) { Channel1.RegisterFlag(FlagsRegister);};

    std::pair<V,V> OutputSignal;
};

template<typename V> void DeviceDACControl<SPI_HandleTypeDef,V,1, DAC_SPI_8653>::Init()
{
  if(FLAG_INIT_DONE) return;
  eprintf("[ EXTERNAL DAC INIT START ] \r\n");
  HAL_GPIO_WritePin(CSPin.first, CSPin.second, GPIO_PIN_SET); HAL_Delay(100);

  //========================================
  TransferBuffer[0] = 0x20; //TransferBuffer[1] = 0x00;
  TransferBuffer[2] = 0x13;
  ActivatePort(); HAL_SPI_Transmit_IT(Device, TransferBuffer, 3); WaitEnd(); EndTransmission(); HAL_Delay(500);

  //========================================
  TransferBuffer[0] = 0x28; //TransferBuffer[1] = 0x00; 
  TransferBuffer[2] = 0x01;

  ActivatePort(); HAL_SPI_Transmit_IT(Device, TransferBuffer, 3); WaitEnd(); EndTransmission(); HAL_Delay(500);

  //========================================
  TransferBuffer[0] = 0x20; //TransferBuffer[1] = 0x00; 
  TransferBuffer[2] = 0x03;

  ActivatePort(); HAL_SPI_Transmit_IT(Device, TransferBuffer, 3); WaitEnd(); EndTransmission(); HAL_Delay(500);
  //=================================================================
  TransferBuffer[0] = 0x38; //TransferBuffer[1] = 0x00; 
  TransferBuffer[2] = 0x01;

  ActivatePort(); HAL_SPI_Transmit_IT(Device, TransferBuffer, 3); WaitEnd(); EndTransmission(); HAL_Delay(500);
  //=================================================================
  TransferBuffer[0] = 0x02; //TransferBuffer[1] = 0x00; 
  TransferBuffer[2] = 0x00;

  ActivatePort(); HAL_SPI_Transmit_IT(Device, TransferBuffer, 3); WaitEnd(); EndTransmission(); HAL_Delay(500);

  //=================================================================
  TransferBuffer[0] = 0x30; //TransferBuffer[1] = 0x00;
  TransferBuffer[2] = 0x03;

  ActivatePort(); HAL_SPI_Transmit_IT(Device, TransferBuffer, 3); WaitEnd(); EndTransmission(); HAL_Delay(500);
  FLAG_INIT_DONE = true;
  eprintf("[ EXTERNAL DAC INIT END ] \r\n");
};

template<typename V> SPI_HandleTypeDef*                DeviceDACControl<SPI_HandleTypeDef,V,1,DAC_SPI_8653>::Device = 0;
template<typename V> std::pair<GPIO_TypeDef*,uint16_t> DeviceDACControl<SPI_HandleTypeDef,V,1,DAC_SPI_8653>::CSPin;
template<typename V> bool    DeviceDACControl<SPI_HandleTypeDef,V,1,DAC_SPI_8653>::FLAG_INIT_DONE = false;
template<typename V> uint8_t DeviceDACControl<SPI_HandleTypeDef,V,1,DAC_SPI_8653>::TransferBuffer[3];
template<typename V> bool    DeviceDACControl<SPI_HandleTypeDef,V,1,DAC_SPI_8653>::TransmissionEndFlag;

//DAC_8653
//===============================================================================================

//================================================================================
//DAC_8550
template<typename V>
class DeviceDACControl<SPI_HandleTypeDef,V,1, DAC_SPI_8550> : public PassValueClass<V> 
{
public:
    DeviceDACControl(SPI_HandleTypeDef& DeviceControl, std::pair<GPIO_TypeDef*,uint16_t> PinSelect)
	{
    Device = &DeviceControl;
	CSPin = PinSelect;
	};
    uint8_t ConfigRegister = 0;

	void SetValue(const V& Value) 
	{
        OutputSignal = Value;
		HAL_GPIO_WritePin(CSPin.first, CSPin.second, GPIO_PIN_RESET);
		TransferBuffer[0] = ConfigRegister;
		TransferBuffer[1] = OutputSignal>>8;
		TransferBuffer[2] = OutputSignal;

		HAL_SPI_Transmit_IT(Device, TransferBuffer, 3); while(!TransmissionEndFlag); TransmissionEndFlag = false;
		HAL_GPIO_WritePin(CSPin.first, CSPin.second, GPIO_PIN_SET);
	}

                               V OutputSignal    = 0;
    const V& GetValue() { return OutputSignal;}; //ALWAYS NULL, IF NEEDED ADD WRITE LAST OUTPUT

	static void Init(){};
    void RegisterFlag(std::map<SPI_TypeDef*,bool*>& FlagsRegister) { FlagsRegister[Device->Instance] = &TransmissionEndFlag;};

    bool TransmissionEndFlag;
    SPI_HandleTypeDef* Device;
	std::pair<GPIO_TypeDef*,uint16_t> CSPin;

private:

	uint8_t TransferBuffer[3];
};

template<typename V>
class DeviceDACControl<SPI_HandleTypeDef,V,2, DAC_SPI_8550> : public PassCoordClass<V>
{
public:
    DeviceDACControl(SPI_HandleTypeDef& DeviceControl, std::pair<GPIO_TypeDef*,uint16_t> PinSelect,
    		            SPI_HandleTypeDef& DeviceControl2, std::pair<GPIO_TypeDef*,uint16_t> PinSelect2) :
    Channel1{DeviceControl,PinSelect},
    Channel2{DeviceControl2,PinSelect2}
	{
	};

    DeviceDACControl<SPI_HandleTypeDef,V,1,DAC_SPI_8550> Channel1;
    DeviceDACControl<SPI_HandleTypeDef,V,1,DAC_SPI_8550> Channel2;
	void Init() { };

	void SetCoord(const std::pair<V,V>& Output) { Channel1.SetValue(Output.first); Channel2.SetValue(Output.second); }

                               std::pair<V,V> OutputSignal{0,0};
    const std::pair<V,V>& GetCoord() { return OutputSignal;}; //ALWAYS NULL, IF NEEDED ADD WRITE LAST OUTPUT

    void RegisterFlag(std::map<SPI_TypeDef*,bool*>& FlagsRegister) { Channel1.RegisterFlag(FlagsRegister);
                                                                     Channel2.RegisterFlag(FlagsRegister);};
private:
};

//DAC_8550_END
//================================================================================

//================================================================================
//ADC_INPUT_DEVICE
template<int N_CHAN = 1>
class DeviceTypeADC : public PassValueClass<uint16_t> 
{
    //template<int NUM> class TimerSoftExt : public TimerSoft<NUM> { };

public:
    enum Mode {ReadyMode = 0, ContinousMode, ActiveMode};
    enum StateSwitcher { SignalFillBottom = 0 , SignalFillTop, SignalFillEnd};
#define SIGNAL_STORE_SIZE 1800

    explicit DeviceTypeADC(ADC_HandleTypeDef* DeviceControl, uint32_t Channel): 
    Device(DeviceControl), DeviceChannel(Channel) 
    { 
    };
    uint16_t OutputSignal = 0;
    uint32_t OutputSignalSum = 0;
    uint16_t AvarageSignal = 0;

    uint16_t SignalRegister[SIGNAL_STORE_SIZE];

    uint16_t AvarageSize = SIGNAL_STORE_SIZE/2 - 10;


    Mode Regim = ContinousMode; 
    StateSwitcher RegisterStateSwitcher = SignalFillBottom;

    void WriteSignal(uint16_t* Buffer, int Size);
    bool WaitEndMeasure() 
    { 
        bool Result = false;
        uint16_t WaitTime    = 2000;
        uint16_t TimeElapsed = 0;

        while( RegisterStateSwitcher != SignalFillEnd)
        {          TimeElapsed += 5; osDelay(5/portTICK_PERIOD_MS);
                if(TimeElapsed > WaitTime) break;               }

        Result = (RegisterStateSwitcher == SignalFillEnd);

        HAL_ADC_Stop_DMA(Device);

        return Result;
    };

    void ExposeStateSwitcher(std::map<ADC_TypeDef*, StateSwitcher* >& Switchers) { Switchers[Device->Instance] = &RegisterStateSwitcher; }

    void ReadSignal();

    const uint16_t& GetValue() { ReadSignal(); return OutputSignal;} ;
	           void SetValue(const uint16_t& Value) {};

	void Init() { HAL_ADCEx_Calibration_Start(Device, ADC_CALIB_MODE_MASK,ADC_SINGLE_ENDED); };

    void SetMeasureFrequency(uint32_t Devider);

    void SetModeReady() {SetModeContinous(false);};
    void StartWork(bool OnOff)
    {
    	if(OnOff) { HAL_ADC_Start_DMA(Device, (uint32_t*)SignalRegister, SIGNAL_STORE_SIZE); }
        else      { HAL_ADC_Stop_DMA(Device); }
    };

    void SetModeContinous(bool OnOff)
    {
        //if(Regim == ContinousMode && OnOff == true ) return;
        //if(Regim == ReadyMode     && OnOff == false) return;

        HAL_ADC_Stop_DMA(Device);  osDelay(1);

        Device->Init.ContinuousConvMode = ENABLE;
        Device->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
        Device->Init.DMAContinuousRequests = DISABLE;


        if(OnOff)
        {
        Device->Init.ContinuousConvMode = ENABLE;
        Device->Init.DMAContinuousRequests = ENABLE;
        Device->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
        }
        
        ReinitDevice();

        if( OnOff) eprintf("[ ADC SET CONTINOUS MODE ] %d \r\n ", OnOff );
        if(!OnOff) eprintf("[ ADC SET ONE SHOT MODE ] %d \r\n ", OnOff );

                  Regim = ReadyMode;
        if(OnOff) Regim = ContinousMode;
    }

    void SetChannel(uint32_t ADC_Channel) { DeviceChannel = ADC_Channel; };

    uint32_t DeviceChannel = ADC_CHANNEL_1;
private:
    ADC_HandleTypeDef* Device = nullptr;

    void ReinitDevice();
};

template<>
class DeviceTypeADC<2> : public PassCoordClass<uint16_t>
{

public:
    explicit DeviceTypeADC(ADC_HandleTypeDef* DeviceControl,
                               ADC_HandleTypeDef* DeviceControl2)
                               {
                                Channel1 = new DeviceTypeADC<1>(DeviceControl, ADC_CHANNEL_1);
                                Channel2 = new DeviceTypeADC<1>(DeviceControl2, ADC_CHANNEL_2);
                               };
    ~DeviceTypeADC() { delete Channel1; delete Channel2;}

    void SetMeasureFrequency(uint32_t Devider) {Channel1->SetMeasureFrequency(Devider);
                                                Channel2->SetMeasureFrequency(Devider);};

    //bool WriteSignal();

    void ExposeStateSwitchers(std::map<ADC_TypeDef*,DeviceTypeADC<1>::StateSwitcher* >& Switchers)
    {
         Channel1->ExposeStateSwitcher(Switchers);
         Channel2->ExposeStateSwitcher(Switchers);
    };

    std::pair<uint16_t,uint16_t> Signal{0,0};
    void ReadSignal() { Signal.first  = Channel1->GetValue();
                        Signal.second = Channel2->GetValue();}

	void Init() { Channel1->Init(); Channel2->Init(); };

    void SetCoord(const std::pair<uint16_t,uint16_t>& Value) { };
    const std::pair<uint16_t,uint16_t>& GetCoord() { ReadSignal(); return Signal;};

    uint16_t GetValue(uint8_t channel) { if(channel > 1 ) return Channel1->GetValue();
                                                                 Channel2->GetValue(); }

    void WriteSignal(uint16_t* Buffer, int Size, uint8_t Channel)
    {
        if(Channel == 1) Channel1->WriteSignal(Buffer,Size);
        if(Channel == 2) Channel2->WriteSignal(Buffer,Size);
    };

    void SetModeReady() {SetModeContinous(false);};
    void SetModeContinous(bool OnOff)
    {
        Channel1->SetModeContinous(OnOff);
        Channel2->SetModeContinous(OnOff);
    }

private:
    DeviceTypeADC<1>* Channel1 = nullptr;
    DeviceTypeADC<1>* Channel2 = nullptr;
};


template<int N_CHAN>
void DeviceTypeADC<N_CHAN>::ReadSignal()
{
    if(Regim == ReadyMode)
    {
    HAL_ADC_Start(Device);
    HAL_ADC_PollForConversion(Device,HAL_MAX_DELAY);
    OutputSignal = HAL_ADC_GetValue(Device);
    return;
    }


    uint8_t ReadPos = SIGNAL_STORE_SIZE/2 + 1;
    if(RegisterStateSwitcher == SignalFillTop) ReadPos = 0;

    AvarageSignal = 0;
    OutputSignalSum = 0;

    //for(int n = 0; n < 20; n++) AvarageSignal += SignalRegister[ReadPos + n]; AvarageSignal /= 20;
    //for(int n = 0; n < AvarageSize; n++)
    //{
    //  if(std::abs(SignalRegister[ReadPos + n] - AvarageSignal) > 5)
    //  OutputSignal += AvarageSignal;
    //  else
    //  OutputSignal += SignalRegister[ReadPos + n];
    //} OutputSignal /= AvarageSize;

    for(int n = 0; n < AvarageSize; n++)
    {
      OutputSignalSum += SignalRegister[ReadPos + n];
    }
      OutputSignal = OutputSignalSum/(AvarageSize);

}


template<int N_CHAN>
void DeviceTypeADC<N_CHAN>::WriteSignal(uint16_t* Buffer, int Size) 
{ 
    Regim = ActiveMode;
    RegisterStateSwitcher = SignalFillBottom;

    eprintf("[ START WRITE SIGNAL SIZE] %d \r\n", Size);
    HAL_ADC_Start_DMA(Device,(uint32_t*)Buffer,Size);
};

template<int N_CHAN>
void DeviceTypeADC<N_CHAN>::ReinitDevice() 
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    multimode.Mode = ADC_MODE_INDEPENDENT;

    sConfig.Rank = ADC_REGULAR_RANK_1;
    //sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.Channel = DeviceChannel;

        HAL_ADC_DeInit(Device);
    if (HAL_ADC_Init(Device) != HAL_OK)                                 {eprintf(" ADC INIT FAIL"); Error_Handler(); } 
    if (Device->Instance == ADC1) HAL_ADCEx_MultiModeConfigChannel(Device, &multimode);
    if (HAL_ADC_ConfigChannel(Device, &sConfig) != HAL_OK)              {eprintf(" ADC INIT FAIL"); Error_Handler(); } 

    HAL_ADCEx_Calibration_Start(Device,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
}

template<int N_CHAN>
void DeviceTypeADC<N_CHAN>::SetMeasureFrequency(uint32_t Devider) 
{
//DEVIDER = ADC_CLOCK_ASYNC_DIV16-256
//ADC_CLOCK = MC_CLOCK/DEVIDER; NUMBER_OPS = 6.5+64.5 = 71;
//MEASURE_PERIOD = NUMBER_OPS/ADC_CLOCK;
//MEASURE_PERIOD = NUMBER_OPS*DEVIDER/MC_CLOCK = NUMBER_OPS/ADC_CLOCK;
//MEASURE_PERIOD = 71*4/(129*1000.000) = 71/(129*1000.000/4) = 2.2015 micro;
//DEVIDER = MEASURE_PERIOD*MC_CLOCK/NUMBER_OPS;
// ADC_CLOCK_ASYNC_DIV64 = 35.224; REGISTRATION_TIME_MILLI = (35.224/1000)*10000 = 350 milli
// ADC_CLOCK_ASYNC_DIV32 = 17.612;
// ADC_CLOCK_ASYNC_DIV16 = 8.806;
// ADC_CLOCK_ASYNC_DIV8  = 4.403; REGISTRATION_TIME = 44 milli
// ADC_CLOCK_ASYNC_DIV4  = 2.2015;
//ADC T_CONV_BASE 16bit-8.5 14bit-7.5 12bit-6.5 

//ADC_SAMPLETIME_64CYCLES_5
//ADC_SAMPLETIME_32CYCLES_5

uint8_t PeriodMicro = 1000.000*44.5*Devider/(129*1000.000);
eprintf("[ ADC SET PERIOD ] %d MICRO %d TIME_MEASURE ", PeriodMicro, 10000*PeriodMicro/1000);

Device->Init.ClockPrescaler = Devider;
ReinitDevice();
}

//template<int N_CHAN> uint16_t DeviceTypeADC<N_CHAN>::SignalRegister[SIGNAL_STORE_SIZE];


//ADC_INPUT_DEVICE END
//================================================================================

class GenericEngineControl {

private:
public:
	//void PerfromCommand(MotorControlCommandStruct Command);
	void MoveWithVelocity(int Velocity, uint8_t Dir){};
	void MoveStep(int StepAmplitude, uint8_t Dir){};
	void MotorStop(){};
};

#endif //GENERIC_SIGNAL_CONTROL_H
