#pragma once
#include <stdint.h>

struct MessageCommand
{
  uint8_t Command;
};

struct MessageAiming
{
  float Position;
  float Velocity;
  float Acceleration;
  float PositionRel;
};

struct MessageAimingInt
{
  uint16_t Position;
  uint16_t Velocity;
  uint16_t Acceleration;
  uint16_t PositionRel;
};

struct MessageAimingDual    { MessageAiming    Channel1; MessageAiming    Channel2; };
struct MessageAimingDualInt { MessageAimingInt Channel1; MessageAimingInt Channel2; };

struct MessageCheckConnection { uint16_t Signal = 0xC5; }; //CONTROL_MESSAGE3
struct MessageCloseConnection { uint16_t Signal = 0xC6; }; //CONTROL_MESSAGE4

class MessageMotorControl //CONTROL_MESSAGE5
{
private:
	uint8_t MotorChannel1;
	uint8_t MotorMoveDir;
	uint8_t MotorMoveRegim;
	uint8_t reserv;
	uint16_t MotorSpeed;
	uint16_t MotorMoveStep;
};

struct MessageCalibration
{
    public:
    uint16_t  NodeType    = 0xA0;
    uint16_t  Command     = 0xA1;
    uint16_t  Channel     = 1;
    uint16_t  Amplitude   = 1;

    uint16_t  PeriodProcess = 1;
    uint16_t  TimeMeasure   = 1;
    uint16_t  NumberSteps   = 1;
    uint16_t  Reserve3      = 1;

};

struct MessagePositionState
{
    public:
    uint16_t Position1;
    uint16_t Position2;
    uint16_t Measure1;
    uint16_t Measure2;
};


template<int S>
class MessageMeasureSeries
{
public:
  MessageMeasureSeries() { Clear(); }

  uint16_t DataVector[S];
  uint16_t DataSize = 0;
  uint16_t DataCapacity = S;

  void Clear() { for(auto& val: DataVector) val = 0xF1F2;  DataSize = DataCapacity;}
  friend void operator>>(uint16_t Value, MessageMeasureSeries& Store) { Store.DataVector[Store.DataSize] = Value; 
                                                                                         Store.DataSize++; }

};


using MessageControl1 = MessageAimingDual;
using MessageControl2 = MessageCalibration;
using MessageControl3 = MessageCommand;
using MessageControl4 = MessageCheckConnection;
using MessageControl5 = MessageCloseConnection;

using MessageState1 = MessagePositionState;
using MessageMeasure1 = MessageMeasureSeries<10000>;



//====================================================================================
struct MessageInternalMonitoring
{
    uint32_t Param1 = 0; uint32_t Param2 = 0; uint32_t Param3 = 0; uint32_t Param4 = 0;
    uint32_t Param5 = 0; uint32_t Param6 = 0; uint32_t Param7 = 0; uint32_t Param8 = 0;
};
