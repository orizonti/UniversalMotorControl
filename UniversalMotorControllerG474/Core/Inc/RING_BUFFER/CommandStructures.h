#pragma once
#include <stdint.h>
#include "UtiliteFunctions.h"

class MESSAGE_HEADER_GENERIC
{
  public:
    uint16_t HEADER = 0x8220; 
    uint8_t  MESSAGE_TYPE  = 0xF1; 
    uint8_t  MESSAGE_IDENT = 1; 
    uint32_t MESSAGE_NUMBER = 0x0; 
    uint16_t DATA_SIZE = 1; 
    uint16_t DATA_SIZE2 = 1; 

    bool isValid() const { return HEADER == 0x8220; }
    void PrintHeader()
    {
    	eprintf("GET MESSAGE IDENT: %d SIZE: %d \r\n", MESSAGE_IDENT, DATA_SIZE);
    }
};

class AimStateStruct
{
  public:
  float Position;
  float Velocity;
  float Acceleration;
  float PositionRel;
};

struct MessageInternalMonitoring
{
    uint32_t Param1 = 0;
    uint32_t Param2 = 0;
    uint32_t Param3 = 0;
    uint32_t Param4 = 0;
    uint32_t Param5 = 0;
    uint32_t Param6 = 0;
    uint32_t Param7 = 0;
    uint32_t Param8 = 0;
};

struct ControlMessage1
{

    public:
    ControlMessage1() {};
    AimStateStruct Channel1;
    AimStateStruct Channel2;
    //void PrintMessage() { eprintf("MESSAGE DATA: %d %d %d %d\r\n", Param1, Param2, Param3, Param4); }
};

struct ControlMessage2
{
    public:
    ControlMessage2() {};
    uint16_t Position = 1;
    uint16_t Velocity = 0;
    uint16_t Acceleration = 100;
    uint16_t Param4 = 100;

    uint16_t Position2 = 1;
    uint16_t Velocity2 = 0;
    uint16_t Acceleration2 = 0;
    uint16_t Param2_4 = 0;
};

struct ControlMessage3
{
    float Param1 = 0;
    float Param2 = 100;
    float Param3 = 100;
    float Param4 = 100;
};

struct ControlMessage4
{
    uint16_t Param1 = 0; 
    uint16_t Param2 = 400;
    uint16_t Param3 = 410;
    uint16_t Param4 = 420;
};

struct ControlMessage5
{
    uint16_t Param1 = 0; 
    uint16_t Param2 = 500;
    uint16_t Param3 = 0; 
    uint16_t Param4 = 500;
};

enum class MessageTypes { MessageType1 = 1,
                          MessageType2 = 2, 
                          MessageType3 = 3,  
                          MessageType4 = 4,  
                          MessageType5 = 5,  
                          type_count};


struct MessagesListStruct
{
ControlMessage1* Message1;
ControlMessage2* Message2;
ControlMessage3* Message3;
ControlMessage4* Message4;
ControlMessage5* Message5;

void (*Call1)(ControlMessage1*);
void (*Call2)(ControlMessage2*);
void (*Call3)(ControlMessage3*);
void (*Call4)(ControlMessage4*);
void (*Call5)(ControlMessage5*);
};


class AimingSignalStruct {

private:
	uint8_t* StartSignal;
	uint8_t* EndSignal;
};

class MotorControlCommandStruct {

private:
	uint8_t MotorChannel1;
	int MotorSpeed;
	int MotorMoveDir;
	int MotorMoveStep;
};


class PIDParamStruct {

private:
	float PROP;
	float INT;
	float DIFF;
};

class AimingMonitoringStruct {

private:
	PIDParamStruct PIDParam;
	MotorControlCommandStruct MotorMoveParam;
};

class AimingRegimStruct {

private:
	uint8_t ManualControl;
	uint8_t PIDControlRegim;
	PIDParamStruct PIDParam;
};
