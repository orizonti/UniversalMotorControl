#pragma once
#include <stdint.h>

template<int NUM_DEV>
struct CommandDevice
{
  public:
   uint8_t DeviceID = 0;
   uint8_t Command = 0;
  uint16_t Param = 0;
};

using CommandDeviceController   = CommandDevice<0>;
using CommandDeviceLaserPower   = CommandDevice<1>;
using CommandDeviceLaserPointer = CommandDevice<2>;
using CommandDeviceFocusator    = CommandDevice<3>;

template<int NUM_DEV>
struct MessageDevice
{
    uint8_t DeviceID = 0;
    uint8_t StateModule = 0;
   uint16_t Param1 = 0;
   uint16_t Param2 = 0;
   uint16_t Param3 = 0;
};

using MessageDeviceController   = MessageDevice<0>;
using MessageDeviceLaserPower   = MessageDevice<1>;
using MessageDeviceLaserPointer = MessageDevice<2>;
using MessageDeviceFocusator    = MessageDevice<3>;

template<int N_CHAN>
struct CommandSetPos
{
   public:
   float POS_X = 0;
   float POS_Y = 0;
};
using CommandSetPosRotary   = CommandSetPos<0>;
using CommandSetPosScanator = CommandSetPos<1>;

struct MessageAiming
{
  float Position;
  float Velocity;
  float Acceleration;
  float PositionRel;
};
struct MessageAimingDual    { MessageAiming    Channel1; MessageAiming    Channel2; };

struct CommandSetSpeed
{
   public:
   float SPEED_X = 0;
   float SPEED_Y = 0;
};

struct MessageMoveState
{
   public:
   int16_t POS_X = 0;
   int16_t POS_Y = 0;
   uint16_t SPEED_X = 1;
   uint16_t SPEED_Y = 1;
};

struct MessagePositionState
{
    public:
    uint16_t Position1;
    uint16_t Position2;
    uint16_t Measure1;
    uint16_t Measure2;
};
using MessageState1 = MessagePositionState;

struct CommandCalibration
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


struct CommandCheckConnection { uint8_t Signal = 0xC5; uint8_t Signal2 = 0xC5; };
struct CommandCloseConnection { uint8_t Signal = 0xC6; uint8_t Signal2 = 0xC5; };


template<int S>
class MessageMeasureSeries
{
public:
  MessageMeasureSeries() { Clear(); }

  uint16_t DataVector[S];
  uint16_t DataSize = 0;
  uint16_t DataCapacity = S-1;

  void Clear() { for(int n = 0; n < S; n++) DataVector[n] = 0xF1F2;  DataSize = DataCapacity;}
  friend void operator>>(uint16_t Value, MessageMeasureSeries& Store) { Store.DataVector[Store.DataSize] = Value; 
                                                                                         Store.DataSize++; }

};

using MessageMeasure1 = MessageMeasureSeries<10000>;


//====================================================================================
struct MessageInternalMonitoring
{
    uint32_t Param1 = 0; uint32_t Param2 = 0; uint32_t Param3 = 0; uint32_t Param4 = 0;
    uint32_t Param5 = 0; uint32_t Param6 = 0; uint32_t Param7 = 0; uint32_t Param8 = 0;
};
