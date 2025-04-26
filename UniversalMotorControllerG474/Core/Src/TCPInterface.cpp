#include <TCPInterface.h>
#include <cmath>
#include "cmsis_os.h"
#include "DeviceSignalInterfaces.h"
#include <type_traits>

extern osMessageQueueId_t QueueAimingControlHandle;
extern osMessageQueueId_t QueueAimingProcessingHandle;

uint8_t TEST_MESSAGE[30];

DeviceTypeConnectionUDP::DeviceTypeConnectionUDP()
{
}

void DeviceTypeConnectionUDP::InitModule()
{
	InputBuffer = new uint8_t[InputBufferSize];

	EthernetInterface = new W5500InterfaceClass;
	EthernetInterface->SocketCreateUDP(NumberSocket);

	RingBufferMessages = new RING_BUFFER_TYPE;
    MessageMeasure = new std::remove_pointer_t<decltype(MessageMeasure)>;

	eprintf("[ IP_REMOTE:  %d.%d.%d.%d] \r\n", IPRemote[0], IPRemote[1], IPRemote[2], IPRemote[3]);
	eprintf("[ PORT LOCAL: %d REMOTE: %d] \r\n", PortLocal, PortRemote);
}

void DeviceTypeConnectionUDP::sendTestMessage()
{
    sprintf((char*)TEST_MESSAGE,"HELLO FROM STM32\r\n");
    sendData(TEST_MESSAGE,strlen((char*)TEST_MESSAGE));
    eprintf("[ SEND TEST MESSAGE %d] \r\n", strlen((char*)TEST_MESSAGE));
}

DeviceTypeConnectionUDP::~DeviceTypeConnectionUDP()
{
	delete EthernetInterface;
	delete InputBuffer;
}

void MessageBytesPrint(int BYTES_RECEIVED, uint8_t* InputBuffer)
{
	eprintf("BYTES RECEIVED: %d \r\n", BYTES_RECEIVED);
	eprintf("BYTES: %x %x %x %x %x %x %x %x %x %x %x %x %x %x \r\n", InputBuffer[0], 
													InputBuffer[1], 
													InputBuffer[2], 
													InputBuffer[3], 
													InputBuffer[4], 
													InputBuffer[5], 
													InputBuffer[6], 
													InputBuffer[7],
													InputBuffer[8],
													InputBuffer[9],
													InputBuffer[11],
													InputBuffer[12],
													InputBuffer[13],
													InputBuffer[14]
												); 
													osDelay(100);
}

void DeviceTypeConnectionUDP::GetIncommingMessages()
{
	BYTES_RECEIVED = 0;
    BYTES_AVAILABLE = getSn_RX_RSR(0); 
	if(BYTES_AVAILABLE < TypeRegister<>::GetMinTypeSize() + sizeof(MESSAGE_HEADER_GENERIC)) return;

	BYTES_RECEIVED = recvfrom( NumberSocket, InputBuffer,BYTES_AVAILABLE, IPRemote, &PortLocal);
	RingBufferMessages->AppendData(InputBuffer,BYTES_RECEIVED); BYTES_RECEIVED = 0;

	//MessageBytesPrint(BYTES_AVAILABLE, InputBuffer);
}

void DeviceTypeConnectionUDP::sendData(uint8_t* Data, uint16_t DataSize)
{
	 ResultTransmission = sendDataChunked(Data, DataSize);
}

void DeviceTypeConnectionUDP::sendMessage(MessagePositionState& Message)
{
                         MessagePosition.DATA = Message;
	  sendData((uint8_t*)(&MessagePosition), sizeof(MessageGeneric<MessagePositionState,MESSAGE_HEADER_GENERIC>));
}

void DeviceTypeConnectionUDP::PutMessageToSend(MessagePositionState& Message)
{
                         MessagePosition.DATA = Message;
	  PutMessageToSend((uint8_t*)(&MessagePosition), sizeof(MessageGeneric<MessagePositionState,MESSAGE_HEADER_GENERIC>));
}

uint8_t DeviceTypeConnectionUDP::sendDataChunked(uint8_t* Data, uint16_t DataSize)
{
	while(DataSize >= 300)
	{
	ResultTransmission = sendto(NumberSocket,Data, 200, IPRemote, PortRemote); osDelay(1);
	DataSize -= ResultTransmission; Data += ResultTransmission;
	}
    if(DataSize == 0) return 0;

    osDelay(4);
   	//eprintf("END TRANSMISSION: %d REMAIN %d\r\n",ResultTransmission,DataSize);
   	ResultTransmission = sendto(NumberSocket,Data, DataSize, IPRemote, PortRemote);

   	return DataSize - ResultTransmission;

}

void DeviceTypeConnectionUDP::sendCommand(uint8_t* Command, uint16_t DataSize)
{
	sendto(NumberSocket,Command, DataSize, IPRemote, PortRemote); 
}


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

void W5500InterfaceClass::SocketCreateUDP(int NumberSocket)
{
    eprintf("UDP SOCKET [0] TRY CREATE \r\n");
	uint8_t CreateResult = 0;

    CreateResult = socket(NumberSocket, Sn_MR_UDP, PortNumber, 0);

    uint8_t rIP[4];
    getsockopt(NumberSocket, SO_DESTIP, rIP);

	eprintf("[ UDP CONNECTION ON %d] \r\n", CreateResult);
	eprintf("[ IP:  %d.%d.%d.%d PORT: %d] \r\n", rIP[0], rIP[1], rIP[2], rIP[3], PortNumber);

}

bool W5500InterfaceClass::InterfaceEstablished = false;
int W5500InterfaceClass::SocketChannelCounter = 0;

//=======================================================
