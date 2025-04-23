#ifndef TCPINTERFACE_H
#define TCPINTERFACE_H
#include "RING_BUFFER/message_command_structures.h"
#include "RING_BUFFER/engine_ring_buffer_generic.h"
#include "RING_BUFFER/message_dispatcher_generic.h"
#include "UtiliteFunctions.h"

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

class UDPConnectionInterface {
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


	uint8_t  IPRemote[4] = {192,168,1,113};
	//uint8_t  IPRemote[4] = {192,168,1,59};
	//uint8_t  IPRemote[4] = {0,0,0,0};
	uint16_t PortRemote  = 2329;
	uint16_t PortLocal   = 2328;
	uint8_t  ResultTransmission;
	uint8_t  NumberSocket = 0;

public:
    UDPConnectionInterface();
    ~UDPConnectionInterface();
	void sendData(uint8_t* Data, uint16_t DataSize);
    void sendCommand(uint8_t* Command, uint16_t DataSize);

	void sendMessage(MessagePositionState& Message);
	void PerformTransmission();
	void SendTestMessage();
	void Init();
private:
	uint8_t sendDataChunked(uint8_t* Data, uint16_t DataSize);
};





#endif
