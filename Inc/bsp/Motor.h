#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "Serial.h"
#include <string.h>

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
    WAIT_HIT
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
extern volatile uint8_t Motor_Flag;
extern volatile uint8_t Move_Stop;
extern volatile uint8_t Move_End;
extern RunState g_RunState;
extern Motor_t Motor;
extern volatile uint16_t pulse_per_round;
extern volatile uint8_t  hit_cnt;
extern volatile uint32_t Hit_ms;
extern volatile uint32_t SysTick_ms;


/* 接口函数 */
void Motor_DirEn_Init(void);
void Motor_Dir(Motor_Dir_e val);
void Motor_EN(Motor_En_e val);
void Motor_PWM_Init(uint16_t arr, uint16_t psc);
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
