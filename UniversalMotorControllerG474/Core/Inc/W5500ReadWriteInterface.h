/*
 * W5500ReadWriteInterface.h
 *
 *  Created on: Feb 21, 2024
 *      Author: broms
 */

#ifndef INC_W5500READWRITEINTERFACE_H_
#define INC_W5500READWRITEINTERFACE_H_


#include "socket.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>



void W5500_Select(void);
void W5500_Unselect(void);
void W5500_ReadBuff(uint8_t* buff, uint16_t len);
void W5500_WriteBuff(uint8_t* buff, uint16_t len);
uint8_t W5500_ReadByte(void);
void W5500_WriteByte(uint8_t byte);
void W5500_SocketCreate();
void W5500_Init(wiz_NetInfo NetParam);

#endif /* INC_W5500READWRITEINTERFACE_H_ */
