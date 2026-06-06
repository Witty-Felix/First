#ifndef __RS485_H
#define __RS485_H

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "Delay.h"
 

/**************** 协议参数 ****************/
#define RADAR_ADDR          0x12
#define U1_RX_BUF_SIZE         12

/* RS485状态结构体 */
typedef struct
{
    uint8_t  U1_RX_FLAG;
    uint8_t  U1_Busy;
    uint8_t  U1_Err;
    uint8_t  U1_RxBuf[U1_RX_BUF_SIZE];
    uint8_t  U1_RxCnt;
    uint32_t U1_TxTime;
} RS485_TypeDef;

/* 全局变量 */
extern volatile RS485_TypeDef      g_RS485;
extern volatile uint16_t           RADAR_RX_DIST;
extern volatile uint16_t           RADAR_RX_TEMP;
extern volatile uint32_t  Timer_ms;

/* 接口函数 */
void    RS485_U1_Init(uint32_t bound);
void    RS485_U1_Poll(void);                       /* 轮询读雷达(不发打印) */
void    RS485_U1_SendArray(uint8_t* array, uint16_t length);
void    RS485_U1_ReadRegister(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num);
uint8_t RS485_U1_GetRxFlag(void);
void    RS485_U1_Data_Print(void);                 /* 打印滤波结果(不发指令) */
void    RS485_U2_Init(uint32_t bound);
void    RS485_U2_SendArray(uint8_t* array, uint16_t length);
void    LED_Show_no(void);
void    LED_Show_Red(void);
void    LED_Show_Blue(void);
void    LED_Show_Green(void);
void    LED_Show_Yellow(void);
void    LED_Show_Orange(void);
void    LED_Show_Color(void);
#endif
