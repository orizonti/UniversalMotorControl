/*
 * W5500ReadWriteInterface.c
 *
 *  Created on: Feb 21, 2024
 *      Author: broms
 */
#include "main.h"
#include "W5500ReadWriteInterface.h"
#include "UtiliteFunctions.h"

extern SPI_HandleTypeDef hspi2;

void W5500_Select(void)   { HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); }
void W5500_Unselect(void) { HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);   }

void W5500_ReadBuff (uint8_t* buff, uint16_t len) { HAL_SPI_Receive(&hspi2, buff, len, HAL_MAX_DELAY); }
void W5500_WriteBuff(uint8_t* buff, uint16_t len) { HAL_SPI_Transmit(&hspi2, buff, len, HAL_MAX_DELAY); }

uint8_t W5500_ReadByte(void) { uint8_t byte; W5500_ReadBuff(&byte, sizeof(byte)); return byte; }

void W5500_WriteByte(uint8_t byte) { W5500_WriteBuff(&byte, sizeof(byte)); }

#define WORK_SOCKET 0
uint8_t rx_tx_buff_sizes[8] = {2, 2, 2, 2, 2, 2, 2, 2};
void W5500_Init(wiz_NetInfo NetParam)
{

	    HAL_GPIO_WritePin(W5500_Reset_GPIO_Port, W5500_Reset_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(W5500_Reset_GPIO_Port, W5500_Reset_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);

		reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect); HAL_Delay(10);
	    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte); HAL_Delay(10);
	    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff); HAL_Delay(10);

	    wizchip_init_reduct(rx_tx_buff_sizes, rx_tx_buff_sizes); HAL_Delay(10);

	    wizchip_setnetinfo(&NetParam);

		ctlnetwork(CN_SET_NETINFO, (void*) &NetParam);
		HAL_Delay(1000);
}

void W5500_SocketCreate()
{
    int result = 0;             
    uint8_t rIP[4];

    result = socket(WORK_SOCKET, Sn_MR_TCP, 2323, 0);
	                  if(result != WORK_SOCKET)   { eprintf("[ SOCKET ERROR ] %d \r\n", result); return; }
    result = listen(WORK_SOCKET);
	                  if(result != SOCK_OK)       { eprintf("[ LISTEN ERROR ] %d \r\n", result); return; }

	while(getSn_SR(WORK_SOCKET) == SOCK_LISTEN) HAL_Delay(2);
    if(getSn_SR(WORK_SOCKET) != SOCK_ESTABLISHED) { eprintf("[ ERROR SOCKET ESTABLISHING ] \r\n"); return; }

    getsockopt(WORK_SOCKET, SO_DESTIP, rIP);
												    eprintf("[ CONNECTION ESTABLISHED ]"); 
												    eprintf("[ IP ] [ %d.%d.%d.%d ]\r\n", rIP[0], rIP[1], rIP[2], rIP[3]); 
}


