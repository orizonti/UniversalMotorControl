#ifndef RING_BUFFER_GENERIC_H
#define RING_BUFFER_GENERIC_H

#include "TypeRegisterEngine.h"
#include "MessageIteratorGeneric.h"
#include "MessageStructGeneric.h"
#include "MessageDispatcherGeneric.h"

#include <cstring>
#include <typeinfo>
#include "main.h"
#include <cstdio>
static char* LOC_TAG{"[BUFFER]"};

template< typename H, size_t M_S, size_t M_N, IteratorMode I_T = IteratorMode::ChunkedFixed> 
class RingBufferGeneric
{
    public:
    RingBufferGeneric();
    ~RingBufferGeneric();
  uint8_t* DATA;
	std::size_t  BufferSize = M_S*M_N;

	MessageIteratorGeneric<H,I_T> MessagePointer;
	MessageIteratorGeneric<H,I_T> IncommingPointer;

  MessageStructGeneric<void*,H>* CurrentMessage = 0;

    bool isMessageAvailable();
    void AppendData(uint8_t* Data, uint8_t Size);
    MessageStructGeneric<void*,H>* TakeMessagePtr();
	  MessageStructGeneric<void*,H>& TakeMessage();

    int CountMessagesInStore() { return MESSAGE_COUNTER;}

    int MESSAGE_COUNTER = 0;
    int MESSAGE_RECEIVED_COUNTER = 0;
    int MAX_MESSAGE_IN_STORE = M_N-2;

    int MAX_MESSAGE_SIZE = M_S;
    int MIN_MESSAGE_SIZE = M_S;

    template<typename T> void RegisterMessage(T MESSAGE);
                         void PrintBufferStatus();
};

template<typename H, size_t M_S, size_t M_N, IteratorMode I_T>
RingBufferGeneric<H,M_S,M_N,I_T>::RingBufferGeneric()
{
  DATA = new uint8_t[BufferSize]; 

  eprintf("================================== \r\n");
  eprintf( "[ CREATE RING BUFFER FIXED ] \r\n");
  eprintf( "MAX MESSAGE:    %d \r\n" , (int)MAX_MESSAGE_SIZE);
  eprintf( "MESSAGE NUMBER: %d \r\n" , (int)M_N);
  eprintf( "ITERATION_TYPE: %d \r\n" , (int)I_T);
  eprintf( "SIZE IN BYTES:  %d \r\n" , (int)BufferSize);

  MessagePointer = MessageIteratorGeneric<H,I_T>(DATA,M_S,M_N);  
  IncommingPointer = MessageIteratorGeneric<H,I_T>(DATA,M_S,M_N);  

  eprintf("RING BUFFER GENERIC SIZE RANGE: %d %d \r\n", MIN_MESSAGE_SIZE , MAX_MESSAGE_SIZE);
  eprintf("================================== \r\n");
}

template<typename H, size_t M_S, size_t M_N, IteratorMode I_T>
RingBufferGeneric<H,M_S,M_N,I_T>::~RingBufferGeneric() { delete DATA; }

template<typename H, size_t M_S, size_t M_N, IteratorMode I_T>
void RingBufferGeneric<H,M_S,M_N,I_T>::AppendData(uint8_t* Data, uint8_t Size)
{
   IncommingPointer.LoadData(Data, Size); 
   MessagePointer.RangeLimits = IncommingPointer.RangeLimits;
   MESSAGE_COUNTER = MessagePointer.StepsTo(IncommingPointer);
   //MESSAGE_RECEIVED_COUNTER = IncommingPointer.MessageNumber;

   if(MESSAGE_COUNTER > 70) MessagePointer++;
}

template<typename H, size_t M_S, size_t M_N, IteratorMode I_T>
MessageStructGeneric<void*,H>* RingBufferGeneric<H,M_S,M_N,I_T>::TakeMessagePtr()
{
  CurrentMessage = MessagePointer.GetMessagePtr(); if(!isMessageAvailable()) return CurrentMessage;

  MessagePointer++; MESSAGE_COUNTER  = MessagePointer.StepsTo(IncommingPointer); 
  
  if(IncommingPointer.MessageNumber > 10000) { IncommingPointer.MessageNumber -= MessagePointer.MessageNumber; MessagePointer.MessageNumber = 0;}
  return CurrentMessage;
}

template<typename H, size_t M_S, size_t M_N, IteratorMode I_T>
MessageStructGeneric<void*,H>& RingBufferGeneric<H,M_S,M_N,I_T>::TakeMessage()
{
  auto& Message = *MessagePointer; CurrentMessage = &Message; if(!isMessageAvailable()) return Message;

  MessagePointer++; MESSAGE_COUNTER  = MessagePointer.StepsTo(IncommingPointer); 

  if(IncommingPointer.MessageNumber > 10000) { IncommingPointer.MessageNumber -= MessagePointer.MessageNumber; MessagePointer.MessageNumber = 0;}
  return Message;
}

template<typename H, size_t M_S, size_t M_N, IteratorMode I_T>
bool RingBufferGeneric<H,M_S,M_N,I_T>::isMessageAvailable()
{
  return MessagePointer.StepsTo(IncommingPointer) > 0;
}

template<typename H, size_t M_S, size_t M_N, IteratorMode I_T>
template<typename T>
void RingBufferGeneric<H,M_S,M_N,I_T>::RegisterMessage(T MESSAGE)
{
    if(sizeof(MessageStructGeneric<T,H>) < MIN_MESSAGE_SIZE)
                                           MIN_MESSAGE_SIZE = sizeof(MessageStructGeneric<T,H>);

    TypeRegister<T>::RegisterType();
	MAX_MESSAGE_SIZE = TypeRegister<>::GetMaxTypeSize() + sizeof(H);
	MIN_MESSAGE_SIZE = TypeRegister<>::GetMinTypeSize() + sizeof(H);
}

template<typename H, size_t M_S, size_t M_N, IteratorMode I_T>
void RingBufferGeneric<H,M_S,M_N,I_T>::PrintBufferStatus()
{
   eprintf("BUFFER MIN: %d MAX: %d \r\n",MIN_MESSAGE_SIZE,MAX_MESSAGE_SIZE);
}

#endif //RING_BUFFER_GENERIC_H
