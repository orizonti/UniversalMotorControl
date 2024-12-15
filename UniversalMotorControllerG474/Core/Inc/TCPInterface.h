#ifndef TCPINTERFACE_H
#define TCPINTERFACE_H
#include "RING_BUFFER/RingBufferGeneric.h"
#include "RING_BUFFER/CommandStructures.h"
#include "UtiliteFunctions.h"

extern "C"
{
#include "W5500ReadWriteInterface.h"
}

template<typename T> void DispatchMessage(T* Message) { eprintf("GET [ MESSAGE \n"); }
template<typename T> void DispatchMessageTest(T* Message) { eprintf("GET [ MESSAGE \n"); }

template<> void DispatchMessage(ControlMessage1* Message); 

class W5500InterfaceClass
{
public:
W5500InterfaceClass();

wiz_NetInfo NetParam;
static bool InterfaceEstablished;
static int SocketChannelCounter;

 uint8_t SocketNumber = 0;
uint16_t PortNumber   = 2323;

uint8_t DataReceived = 0;

void InitDevice();
void SocketCreate(int NumberSocket);
void SocketCreateUDP(int NumberSocket);
};

class TCPInterface {

private:
	uint8_t  InputBufferSize = 60;
	uint8_t  InputWaitDataSize = 0;

	uint8_t* InputBuffer;

	W5500InterfaceClass* EthernetInterface;
	int BYTES_RECEIVED = 0;

    RingBufferGeneric<MESSAGE_HEADER_GENERIC, 32,100, IteratorMode::Continous>* RingBufferMessages;
    MessageDispatcherGeneric<MessagesListStruct>* MessageDispatcher;

public:
    TCPInterface();
    ~TCPInterface();
	void SendData(uint8_t* Data, uint8_t DataSize);
	void PerformTransmission();
	void PerformTransmissionUDP();


};





#endif
