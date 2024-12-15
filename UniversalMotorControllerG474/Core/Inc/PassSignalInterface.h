#ifndef PASS_SIGNAL_INTERFACE_H
#define PASS_SIGNAL_INTERFACE_H
//#include "AimingModules.h"
#include <utility>
#include <stdint.h>


template<typename T = double>
class PassValueClass
{
  public:
      virtual void SetValue(const T& Value) = 0;
  virtual const T& GetValue() = 0;

         PassValueClass& operator|(PassValueClass& Receiver) { Receiver.SetValue(this->GetValue()); return Receiver;}
                    void operator|(T& OutputValue) { OutputValue = this->GetValue();}
  friend PassValueClass& operator|(T& Value, PassValueClass& Receiver) { Receiver.SetValue(Value); return Receiver;}
};

template<typename T>
class PassCoordClass
{
  public:
                   virtual void SetCoord(const std::pair<T,T>& Value) = 0;
  virtual const std::pair<T,T>& GetCoord() = 0;

         PassCoordClass& operator|(PassCoordClass& Receiver) { Receiver.SetCoord(this->GetCoord()); return Receiver;}
                    void operator|(std::pair<T,T>& OutputCoord) { OutputCoord = this->GetCoord();}
  friend PassCoordClass& operator|(std::pair<T,T>& Coord, PassCoordClass& Receiver) { Receiver.SetCoord(Coord); return Receiver;}
};


template<typename T_IN, typename T_OUT, float G_VAL>
class PassValueTransform
{
  public:
  T_OUT GAIN = G_VAL;
  T_OUT OutValue; 
          virtual void SetValue(const T_IN& Value) { OutValue = Value*GAIN;};
  virtual const T_OUT& GetValue()  { return OutValue;};

         PassValueClass<T_OUT>& operator|(PassValueClass<T_OUT>& Receiver)           { Receiver.SetValue(OutValue);}
  friend    PassValueTransform& operator|(T_IN& Value, PassValueTransform& Receiver) { Receiver.SetValue(Value); return Receiver;}
};

template<typename T_IN, typename T_OUT, float G_VAL>
class PassCoordTransform
{
  public:
  T_OUT GAIN = G_VAL;
  std::pair<T_OUT,T_OUT> OutCoord; 

  virtual void SetCoord(const std::pair<T_IN,T_IN>& Coord) { OutCoord.first = Coord.first*GAIN;
                                                             OutCoord.second = Coord.second*GAIN; };
  virtual const std::pair<T_OUT,T_OUT>& GetCoord()  { return OutCoord;};

      PassCoordClass<T_OUT>& operator|(PassCoordClass<T_OUT>& Receiver) { Receiver.SetCoord(OutCoord); return Receiver;}
  friend PassCoordTransform& operator|(std::pair<T_IN,T_IN>& Coord, PassCoordTransform& Receiver) { Receiver.SetCoord(Coord); return Receiver;}
  friend PassCoordTransform& operator|(PassCoordClass<T_IN>& Sender, PassCoordTransform& Receiver) { Receiver.SetCoord(Sender.GetCoord()); return Receiver;}
};

#endif
