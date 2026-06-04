#include "LED.h"
#include "LimSwc.h"
#include "Serial.h"
#include "Timer.h"
#include "Motor.h"
#include "RS485.h"
#include "System.h"
#include "Delay.h"

int main(void)
{
    /* 系统初始化 */
    System_Init();

    /* 外设初始化 */
    LED_Init();
    LimSwc_Init();
    Serial_Init();
    Timer_Init();
    Motor_Init(&Motor);
    RS485_U1_Init(9600);
    RS485_U2_Init(9600);

    Delay_ms(500);

    /* 应用启动 */
    g_RunState = HOME_INIT;
    // Motor_Start(&Motor);
    printf("Motor Starts!\n");

    

    if(1)
    {
        while (1)
        {
            /* 每200ms：轮询读雷达（数据在中断中滤波后存入全局变量） */
            if(Timer_Poll_Flag == 1)
            {
                Timer_Poll_Flag = 0;
                RS485_U1_Poll();
            }

            /* 每5s：打印滤波后的距离/温度 */
            if(Timer_Print_Flag == 1)
            {
                Timer_Print_Flag = 0;
                RS485_U1_Data_Print();
            }
            
            Motor_Check_State(&Motor);
            
        }
    }
    else
    {
        Motor_EN(DIS);
    }
}
