#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "Serial.h"
#include <string.h>
#include "Timer.h"
#include "RS485.h"

/* 枚举 & 结构体 */
typedef enum
{
    FORWARD = 0,
    BACKWARD
}Motor_Dir_e;

typedef enum
{
    ENA = 0,
    DIS
}Motor_En_e;

typedef enum
{
    HOME_INIT = 0,
    HOME_CHECK,
    WAIT_START,
    WAIT_HIT,
    /* 定点运动状态 */
    GO_MID,             /* 正在前往中点(3200) */
    MID_STOP,           /* 在中点停2秒 */
    GO_END,             /* 正在前往终点(6400) */
    MID_STOP2,          /* 在终点停2秒 */
    BACK_MID,           /* 返回中点 */
    MID_STOP3,          /* 在中点停2秒 */
    GO_BACK,            /* 正在返回起点(0) */
    BACK_STOP           /* 回到起点停2秒，然后循环 */
}RunState;

typedef struct
{
    Motor_Dir_e Motor_Dir;
    Motor_En_e  Motor_En;
    uint16_t    Speed;
    uint32_t    Pulse_Max;
    uint32_t    Pulse_Count;
}Motor_t;

/* 全局变量 */
extern uint8_t           Motor_Flag;
extern uint8_t           Move_Stop;
extern RunState          g_RunState;
extern volatile Motor_t  Motor;
extern volatile uint32_t Timer_ms;
extern volatile uint8_t  Obstacle_Flag;

/* 接口函数 */
void Motor_Dir(Motor_Dir_e val);
void Motor_EN(Motor_En_e val);
void Motor_SetARR(uint16_t speed);
void Motor_SetCRR(uint16_t speed);
void Motor_Speed(uint16_t Speed);
void Motor_Pulse(FunctionalState val);
void Motor_Init(Motor_t* Motor);
void Motor_Start(Motor_t* Motor);
void Motor_Stop(void);
void Motor_Resume(void);
void Motor_Change_Dir(Motor_t* Motor);
void Motor_Check_State(Motor_t* Motor);

#endif
