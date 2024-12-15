#include "UtiliteFunctions.h"
#include <stdarg.h>
#include "stm32g4xx_hal.h"

#include "stdio.h"
#include <string.h>

extern UART_HandleTypeDef huart3;
char OutputBuffer[100];
#define DebugUart huart3

void eprintf(const char* str, ...)
{

    va_list args1;
    va_start(args1, str);
    vsprintf(OutputBuffer,str, args1);
    va_end(args1);

	HAL_UART_Transmit_IT(&DebugUart, (const uint8_t*)OutputBuffer, strlen(OutputBuffer));
	HAL_Delay(10);
}

void eprintf_repeat()
{
	HAL_UART_Transmit_IT(&DebugUart, (const uint8_t*)OutputBuffer, strlen(OutputBuffer));
	HAL_Delay(10);
}