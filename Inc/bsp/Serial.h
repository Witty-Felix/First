#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

#define USART3_RX_BUFFER_SIZE 256

extern uint8_t USART3_RxBuffer[];
extern volatile uint16_t USART3_RxFinished;

extern volatile uint8_t Serial_RxFlag;
extern volatile uint8_t Serial_RxData;

void    Serial_Init(void);
void    Serial_SendByte(uint8_t Data);
void    Serial_SendArray(uint8_t* Array, uint16_t Length);
void    Serial_SendString(char* str);
void    Serial_SendNum(uint32_t Num, uint16_t Length);
uint8_t Get_USART3_Data(void);
void    flashUART3(void);

#endif
