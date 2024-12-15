#ifndef GENERIC_SIGNAL_CONTROL_H
#define GENERIC_SIGNAL_CONTROL_H
#include <stdint.h>
#include "main.h"
#include "stm32g4xx_hal_dac.h"
#include "stm32g4xx_hal_spi.h"
#include <utility>
#include <map>
#include "PassSignalInterface.h"


extern DAC_HandleTypeDef hdac1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart3;


template<typename DEV_T, typename V, int N_CHAN = 1, int TYPE = 1>
class DeviceSignalControl : public PassValueClass<V> {

public:
    DeviceSignalControl(DEV_T& DeviceInterface){};
	void SetValue(V Value) {};
	void Init(){};
private:
    DEV_T* Device;
};

template<typename V>
class DeviceSignalControl<DAC_HandleTypeDef,V,1> : public PassValueClass<V> 
{
public:
    DeviceSignalControl(DAC_HandleTypeDef& DeviceInterface, int ChannelNumber) {Device = &DeviceInterface; Channel = ChannelNumber;};
	void SetValue(const V& Value) { HAL_DAC_SetValue(Device,Channel,DAC_ALIGN_12B_R,Value);}
    const V& GetValue() { return OutputValue;}
	void Init() { HAL_DAC_Start(Device,Channel); };
private:
    V OutputValue = 0; 
    DAC_HandleTypeDef* Device = 0;
    int Channel = DAC_CHANNEL_1;
};

//template<int GainPassSignal, typename V = float>
//class ValueGain : public PassValueClass<T>
//{
//  public:
//  T Diff;
//  int GainParam = 1;
//  void SetValue(T NewValue) override;
//  T& GetValue() override { return Diff;}
//};

template<typename V>
class DeviceSignalControl<DAC_HandleTypeDef,V,2> : public PassCoordClass<V> 
{
public:
    DeviceSignalControl(DAC_HandleTypeDef& DeviceInterface) {Device = &DeviceInterface;};

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

    const std::pair<V,V>& GetCoord() { return OutputCoord;}
	void Init() { HAL_DAC_Start(Device,DAC_CHANNEL_1); 
                  HAL_DAC_Start(Device,DAC_CHANNEL_2); 
                  eprintf("[ INTERNAL DAC CONTROL CREATE ] \r\n"); };
private:
    std::pair<V,V> OutputCoord;
    DAC_HandleTypeDef* Device = 0;
};

template<typename V>
class DeviceSignalControl<SPI_HandleTypeDef,V,1> : public PassValueClass<V> 
{
public:
    DeviceSignalControl(SPI_HandleTypeDef& DeviceInterface, std::pair<GPIO_TypeDef*,uint16_t> PinSelect, uint8_t NumberChannel = 1) 
	{
    Device = &DeviceInterface;
	CSPin = PinSelect;
	ChannelID = 0x18; if(NumberChannel == 2) ChannelID = 0x19;
	};
    uint16_t OutputValue = 0;

	void SetValue(const V& Value) 
	{
        OutputValue = Value;
		HAL_GPIO_WritePin(CSPin.first, CSPin.second, GPIO_PIN_RESET);
		TransferBuffer[0] = ChannelID;
		TransferBuffer[1] = OutputValue>>8;
		TransferBuffer[2] = OutputValue;

		HAL_SPI_Transmit_IT(Device, TransferBuffer, 3); while(!TransmissionEndFlag); TransmissionEndFlag = false;
		HAL_GPIO_WritePin(CSPin.first, CSPin.second, GPIO_PIN_SET);
	}

    const V& GetValue() { return CurrentOutput;}; //ALWAYS NULL, IF NEEDED ADD WRITE LAST OUTPUT

	static void Init();
    void RegisterFlag(std::map<SPI_TypeDef*,bool*>& FlagsRegister) { FlagsRegister[Device->Instance] = &TransmissionEndFlag;};

    static bool FLAG_INIT_DONE;
    static bool TransmissionEndFlag;
	static uint8_t TransferBuffer[3];
private:
    uint8_t ChannelID = 0x18;

    V CurrentOutput = 0;

    static SPI_HandleTypeDef* Device;
	static std::pair<GPIO_TypeDef*,uint16_t> CSPin;
    static void WaitEnd() { while(!TransmissionEndFlag); TransmissionEndFlag = false; };
    static void ActivatePort() { HAL_GPIO_WritePin(CSPin.first, CSPin.second, GPIO_PIN_RESET); HAL_Delay(5); }
	static void EndTransmission() {HAL_GPIO_WritePin(CSPin.first, CSPin.second, GPIO_PIN_SET); HAL_Delay(5); };
};

template<typename V>
class DeviceSignalControl<SPI_HandleTypeDef,V,2> : public PassCoordClass<V> 
{
public:
    DeviceSignalControl(SPI_HandleTypeDef& DeviceInterface, std::pair<GPIO_TypeDef*,uint16_t> PinSelect) : 
    Channel1{DeviceInterface,PinSelect,1},
    Channel2{DeviceInterface,PinSelect,2}
	{
	};

    DeviceSignalControl<SPI_HandleTypeDef,V,1> Channel1;
    DeviceSignalControl<SPI_HandleTypeDef,V,1> Channel2;
	void Init() { DeviceSignalControl<SPI_HandleTypeDef,V,1>::Init(); };

	void SetCoord(const std::pair<V,V>& Output) { Channel1.SetValue(Output.first); Channel2.SetValue(Output.second); }
    const std::pair<V,V>& GetCoord() { return CurrentOutput;}; //ALWAYS NULL, IF NEEDED ADD WRITE LAST OUTPUT

    void RegisterFlag(std::map<SPI_TypeDef*,bool*>& FlagsRegister) { Channel1.RegisterFlag(FlagsRegister);};
private:
    //std::pair<V,V> CurrentOutput = std::pair<V,V>(0,0);
    std::pair<V,V> CurrentOutput;
};

template<typename V> bool DeviceSignalControl<SPI_HandleTypeDef,V,1>::FLAG_INIT_DONE = false;
template<typename V> SPI_HandleTypeDef* DeviceSignalControl<SPI_HandleTypeDef,V,1>::Device = 0;
template<typename V> std::pair<GPIO_TypeDef*,uint16_t> DeviceSignalControl<SPI_HandleTypeDef,V,1>::CSPin;
template<typename V> uint8_t DeviceSignalControl<SPI_HandleTypeDef,V,1>::TransferBuffer[3];
template<typename V> bool DeviceSignalControl<SPI_HandleTypeDef,V,1>::TransmissionEndFlag;
template<typename V> void DeviceSignalControl<SPI_HandleTypeDef,V,1>::Init()
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


class GenericEngineControl {

private:
public:
	//void PerfromCommand(MotorControlCommandStruct Command);
	void MoveWithVelocity(int Velocity, uint8_t Dir){};
	void MoveStep(int StepAmplitude, uint8_t Dir){};
	void MotorStop(){};
};

#endif //GENERIC_SIGNAL_CONTROL_H
