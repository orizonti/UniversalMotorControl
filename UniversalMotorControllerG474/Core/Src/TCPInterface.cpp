#include <TCPInterface.h>
#include <cmath>
#include "cmsis_os.h"
#include "DeviceSignalInterfaces.h"

#define WORK_SOCKET 0


extern osMessageQueueId_t QueueAimingControlHandle;
extern osMessageQueueId_t QueueAimingProcessingHandle;


TCPInterface::TCPInterface()
{


	InputBuffer = new uint8_t[InputBufferSize];

	EthernetInterface = new W5500InterfaceClass;
	//EthernetInterface->SocketCreate(0);
	EthernetInterface->SocketCreateUDP(0);

	RingBufferMessages = new std::remove_reference<decltype(*RingBufferMessages)>::type;

	RingBufferMessages->RegisterMessage(ControlMessage1());
	RingBufferMessages->RegisterMessage(ControlMessage2());
	RingBufferMessages->RegisterMessage(ControlMessage3());
	RingBufferMessages->RegisterMessage(ControlMessage4());
	RingBufferMessages->RegisterMessage(ControlMessage5());

	auto max = TypeRegister<>::GetMaxTypeSize();
	auto min = TypeRegister<>::GetMinTypeSize();
	eprintf("[ BUFFER MIN: %d MAX: %d \r\n",min, max);

    MessageDispatcher = new MessageDispatcherGeneric<MessagesListStruct>;

				   MessagesListStruct DispatcherList;
									  DispatcherList.Call1 = DispatchMessage;
									  DispatcherList.Call2 = DispatchMessage;
									  DispatcherList.Call3 = DispatchMessage;
									  DispatcherList.Call4 = DispatchMessage;
									  DispatcherList.Call5 = DispatchMessage;
	MessageDispatcher->SetDispatchList(DispatcherList);

}

TCPInterface::~TCPInterface()
{
	delete EthernetInterface;
	delete InputBuffer;

}


void TCPInterface::SendData(uint8_t* Data, uint8_t DataSize) { send(0,Data,DataSize); }

//===========================================================================
template<>
void DispatchMessage(ControlMessage1* Message)
{
    osMessageQueuePut(QueueAimingControlHandle,Message,0,0);
    //osMessageQueuePut(QueueAimingProcessingHandle,Message,0,0);
};

//===========================================================================
extern std::pair<float, float> ControlSignalDAC;
extern DeviceSignalControl<DAC_HandleTypeDef, uint16_t,2> InternalDACControl;

uint16_t MESSAGE_IN_BUFFER = 0;
uint16_t BYTES_AVAILABLE = 0;
int MessageRecCounter = 0;

template<>
void DispatchMessageTest(ControlMessage1* Message)
{
	MessageRecCounter++;  

    ControlSignalDAC.first = Message->Channel1.Position*8;
    ControlSignalDAC.second = Message->Channel1.Position*8;

    if(MessageRecCounter % 200 == 0) eprintf("GET [ AIMING ]: %d %d %d %d  \r\n", 
    																BYTES_AVAILABLE,
																	MessageRecCounter,
    																(int)Message->Channel1.PositionRel,
    																(int)MESSAGE_IN_BUFFER
    														);
    InternalDACControl.SetCoord(ControlSignalDAC);

};
//===========================================================================


void TCPInterface::PerformTransmission()
{
	eprintf("[TCP SERVER WORK ]\r\n");
	eprintf("[WAIT MESSAGE SIZE %d ]\r\n",RingBufferMessages->MIN_MESSAGE_SIZE);

    //MESSAGE_HEADER_GENERIC* HEADER = 0;;
	BYTES_RECEIVED = 0;

	while(1)
	{
      BYTES_AVAILABLE = getSn_RX_RSR(0); if(BYTES_AVAILABLE < 1) { osDelay(1); continue;};

		  while(BYTES_AVAILABLE >= 48)
		  {
		  BYTES_RECEIVED = recv(0, InputBuffer, 48);
		  BYTES_AVAILABLE -= BYTES_RECEIVED;

		  RingBufferMessages->AppendData(InputBuffer,BYTES_RECEIVED); BYTES_RECEIVED = 0;
		  }

           //MESSAGE_IN_BUFFER = RingBufferMessages->CountMessagesInStore();
	      *RingBufferMessages | *MessageDispatcher;

	}

}

void TCPInterface::PerformTransmissionUDP()
{
	eprintf("[UDP SERVER WORK ]\r\n");

	BYTES_RECEIVED = 0;
	uint8_t RemoteIP[4] = {192,168,0,111};
	uint16_t Port = 2323;

	while(1)
	{
      BYTES_AVAILABLE = getSn_RX_RSR(0); if(BYTES_AVAILABLE < 1) { osDelay(1); continue;};


		  while(BYTES_AVAILABLE >= 48)
		  {
		  BYTES_RECEIVED = recvfrom(0, InputBuffer,48,RemoteIP,&Port);
		  BYTES_AVAILABLE -= BYTES_RECEIVED;

		  RingBufferMessages->AppendData(InputBuffer,BYTES_RECEIVED); BYTES_RECEIVED = 0;
		  }

		//MESSAGE_IN_BUFFER = RingBufferMessages->CountMessagesInStore();
		*RingBufferMessages | *MessageDispatcher;
	}
}


//==============================================================


W5500InterfaceClass::W5500InterfaceClass()
{

if(W5500InterfaceClass::SocketChannelCounter >= 8) return;

SocketNumber = W5500InterfaceClass::SocketChannelCounter;  //CAN CREATE UP TO EIGHT SOCKETS
W5500InterfaceClass::SocketChannelCounter++;

if(W5500InterfaceClass::InterfaceEstablished) return;

       NetParam = { .mac = {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},
                            .ip = {192, 168, 1, 178},
                            .sn = {255, 255, 255, 0},
                            .gw =  {192, 168, 1, 1},
                            .dns = {0, 0, 0, 0},
                            .dhcp = NETINFO_STATIC };

InitDevice();
eprintf(" [ W5500 END INIT ] \r\n");

W5500InterfaceClass::InterfaceEstablished = true;

}

void W5500InterfaceClass::InitDevice()
{
	    HAL_GPIO_WritePin(W5500_Reset_GPIO_Port, W5500_Reset_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(W5500_Reset_GPIO_Port, W5500_Reset_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);

		reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
	    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
	    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

		uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};

	    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

	    wizchip_setnetinfo(&NetParam);

        eprintf(" [ W5500 RESET ] \r\n"); HAL_Delay(5);
        eprintf(" [ W5500 IP ] [%d:%d:%d:%d] \r\n", NetParam.ip[0], NetParam.ip[1], NetParam.ip[2], NetParam.ip[3]);
		ctlnetwork(CN_SET_NETINFO, (void*) &NetParam);

		HAL_Delay(1000);
}


void W5500InterfaceClass::SocketCreate(int NumberSocket)
{

    eprintf("SOCKET [0] TRY CREATE \r\n");
	uint8_t result = 0;
	uint8_t CreateResult = 0;
	uint8_t ListenResult = 0;
	uint8_t ConnectionResult = 0;

    CreateResult = socket(NumberSocket, Sn_MR_TCP, PortNumber, 0);
    ListenResult = listen(NumberSocket);

    eprintf("SOCKET [0] WAIT CONNECTION \r\n");
	while(getSn_SR(NumberSocket) == SOCK_LISTEN) 
	{ 
        eprintf("[ WAIT CONNECTION ] \r\n");
		osDelay(4000);
	} //WAIT CONNECTION

    if(getSn_SR(NumberSocket) != SOCK_ESTABLISHED)
    {
	ConnectionResult = 1;
	eprintf("[ CONNECTION FAIL ] %d \r\n", ListenResult);
	return;

    }

    uint8_t rIP[4];
    getsockopt(NumberSocket, SO_DESTIP, rIP);

	eprintf("[ CONNECTION ESTABLISHED ] \r\n");
	eprintf("[ IP:  %d.%d.%d.%d] \r\n", rIP[0], rIP[1], rIP[2], rIP[3]);

}

void W5500InterfaceClass::SocketCreateUDP(int NumberSocket)
{

    eprintf("UDP SOCKET [0] TRY CREATE \r\n");
	uint8_t CreateResult = 0;

    CreateResult = socket(NumberSocket, Sn_MR_UDP, PortNumber, 0);

    uint8_t rIP[4];
    getsockopt(NumberSocket, SO_DESTIP, rIP);

	eprintf("[ UDP CONNECTION ON %d] \r\n", CreateResult);
	eprintf("[ IP:  %d.%d.%d.%d] \r\n", rIP[0], rIP[1], rIP[2], rIP[3]);

}

bool W5500InterfaceClass::InterfaceEstablished = false;
int W5500InterfaceClass::SocketChannelCounter = 0;

//=======================================================
