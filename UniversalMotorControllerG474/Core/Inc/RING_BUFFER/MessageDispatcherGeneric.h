#ifndef MESSAGE_DISPATCHER_GENERIC_H
#define MESSAGE_DISPATCHER_GENERIC_H

#include "CommandStructures.h"
#include "main.h"


//LIST_TYPE STRUCT TYPE THAT CONTAINS POINTERS TO ALL MESSAGE TYPE
template <typename T> struct DATA_TYPE_TAG{};
template<typename LIST_TYPE>
class MessageDispatcherGeneric 
{
public:

    //template<typename RingBufferType, typename ListFunction> void DispatchNextMessage(RingBufferType& RingBuffer);
    template<typename RingBufferType> void DispatchNextMessage(RingBufferType& RingBuffer);
    template<typename RingBufferType> friend void operator|(RingBufferType& RingBuffer, MessageDispatcherGeneric<LIST_TYPE>& Processor) { Processor.DispatchNextMessage(RingBuffer); }
    template<typename RingBufferType> void LinkToBuffer(RingBufferType& RingBuffer) { this->DispatchNextMessage(RingBuffer); }
    void SetDispatchList(LIST_TYPE MessageDispatchList) { MessagesList = MessageDispatchList;};

    template<typename DATA_TYPE, typename MESSAGE_TYPE> DATA_TYPE* ExtractData(MESSAGE_TYPE* Message, const DATA_TYPE_TAG<DATA_TYPE>& TAG)
    {
        return &(reinterpret_cast<MessageStructGeneric<DATA_TYPE,MESSAGE_HEADER_GENERIC>*>(Message)->DATA);
    };
    LIST_TYPE MessagesList;

};


template<> template<typename RingBufferType> 
void MessageDispatcherGeneric<MessagesListStruct>::DispatchNextMessage(RingBufferType& RingBuffer)
{
    if(!RingBuffer.isMessageAvailable()) return;

    auto& Message = RingBuffer.TakeMessage();
    //eprintf("[ GET MESSAGE TYPE: %d ]  \r\n", Message.HEADER.MESSAGE_IDENT);

    switch(Message.HEADER.MESSAGE_IDENT)
    {
        case (int)MessageTypes::MessageType1: 
        MessagesList.Message1 = ExtractData(&Message,DATA_TYPE_TAG<ControlMessage1>()); 
        MessagesList.Call1(MessagesList.Message1); 
        break;
        case (int)MessageTypes::MessageType2: 
        MessagesList.Message2 = ExtractData(&Message,DATA_TYPE_TAG<ControlMessage2>()); 
        MessagesList.Call2(MessagesList.Message2);
        break;
        case (int)MessageTypes::MessageType3: 
        MessagesList.Message3 = ExtractData(&Message,DATA_TYPE_TAG<ControlMessage3>()); 
        MessagesList.Call3(MessagesList.Message3);
        break;
        case (int)MessageTypes::MessageType4: 
        MessagesList.Message4 = ExtractData(&Message,DATA_TYPE_TAG<ControlMessage4>()); 
        MessagesList.Call4(MessagesList.Message4);
        break;
        case (int)MessageTypes::MessageType5: 
        MessagesList.Message5 = ExtractData(&Message,DATA_TYPE_TAG<ControlMessage5>()); 
        MessagesList.Call5(MessagesList.Message5);
        break;
        default: eprintf("[DISPATCHER] UNREGISTERED MESSAGE [%d] \r\n",Message.HEADER.MESSAGE_IDENT);
        break;
    }

    DispatchNextMessage(RingBuffer);
}


//template<typename M, typename LIST_TYPE>  //WHY I CANT PUT FRIEND METHOD DEFINITION OUT OFF CLASS ???
//void operator|(RingBufferEngine<M>& RingBuffer, MessageDispatcherGeneric<LIST_TYPE>& Processor) 
//{ Processor.DispatchNextMessage(RingBuffer); }


#endif //MESSAGE_DISPATCHER_GENERIC_H


