#ifndef TCPINTERFACE_H
#define TCPINTERFACE_H
#include "RING_BUFFER/message_command_structures.h"
#include "RING_BUFFER/engine_ring_buffer_generic.h"
#include "RING_BUFFER/message_dispatcher_generic.h"
#include "UtiliteFunctions.h"
#include <queue>
#include "cmsis_os2.h"                  

extern "C"
{
#include "W5500ReadWriteInterface.h"
}


class W5500InterfaceClass
{
public:
W5500InterfaceClass();

wiz_NetInfo NetParam;
static bool InterfaceEstablished;
static int SocketChannelCounter;

 uint8_t SocketNumber = 0;
uint16_t PortNumber   = 2328;

uint8_t DataReceived = 0;

void InitDevice();
void SocketCreateUDP(int NumberSocket);
};
using      HEADER_TYPE = MESSAGE_HEADER_GENERIC;
using      MessageType = MessageGeneric<void*, HEADER_TYPE>;
using RING_BUFFER_TYPE = RingBufferGeneric<HEADER_TYPE, 32,40, IteratorMode::Continous>;
using   DispatcherType = MessageDispatcher<HEADER_TYPE, RING_BUFFER_TYPE>;

class DeviceTypeConnectionUDP {
public:
       RING_BUFFER_TYPE* RingBufferMessages;

private:
	uint8_t  InputBufferSize = 60;
	uint8_t  InputWaitDataSize = 0;

	uint8_t* InputBuffer;

	int MessageCounter = 0;
	int BYTES_RECEIVED = 0;
	//uint16_t BaseSize = sizeof(MessageAimingDual) + sizeof(MESSAGE_HEADER_GENERIC);
	uint16_t BaseSize = 52;
	uint16_t BYTES_AVAILABLE = 0;

	W5500InterfaceClass* EthernetInterface;

	MessageGeneric<MessagePositionState,MESSAGE_HEADER_GENERIC> MessagePosition;
	MessageGeneric<MessageMeasure1, MESSAGE_HEADER_GENERIC>* MessageMeasure = nullptr;

	//uint8_t  IPRemote[4] = {192,168,1,113};
	uint8_t  IPRemote[4] = {192,168,1,59};
	//uint8_t  IPRemote[4] = {192,168,1,104};
	//uint8_t  IPRemote[4] = {0,0,0,0};
	uint16_t PortRemote  = 2329;
	uint16_t PortLocal   = 2328;
	uint8_t  ResultTransmission;
	uint8_t  NumberSocket = 0;
	std::queue<std::pair<uint8_t*,uint16_t>> DataToSend;

public:
    DeviceTypeConnectionUDP();
    ~DeviceTypeConnectionUDP();
	void sendData(uint8_t* Data, uint16_t DataSize);
    void sendCommand(uint8_t* Command, uint16_t DataSize);

	void GetIncommingMessages();
	void InitModule();

	void sendTestMessage();

	void sendMessage(MessagePositionState& Message);
	void PutMessageToSend(MessagePositionState& Message); 

	void SetInput(uint16_t Position) 
	{ 
		MessagePosition.DATA.Position1 = Position; 
	    sendCommand((uint8_t*)(&MessagePosition), sizeof(MessageGeneric<MessagePositionState,MESSAGE_HEADER_GENERIC>));
	};

	void PutMessageToSend(uint8_t* data, uint16_t size) { DataToSend.emplace(std::pair<uint8_t*, uint16_t>(data,size)); };

	void SendPendingMessage()
	{
	  while(!DataToSend.empty())  
	  { 
		if(DataToSend.front().second > 44)
		sendData(DataToSend.front().first, DataToSend.front().second); 
		else
		sendCommand(DataToSend.front().first, DataToSend.front().second); 
		   DataToSend.pop();
	  } 
	}

	friend void operator|(uint16_t Position, DeviceTypeConnectionUDP& Connection) { Connection.SetInput(Position); }
	friend void operator|(MessagePositionState& Message, DeviceTypeConnectionUDP& Connection) { Connection.sendMessage(Message); }
private:
	uint8_t sendDataChunked(uint8_t* Data, uint16_t DataSize);
};





#endif
